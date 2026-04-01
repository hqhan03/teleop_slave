"""
PyBullet SDLS IK solver for the Tesollo DG-5F right hand.

Loads the DG-5F URDF into PyBullet (headless), solves per-finger IK
to match target fingertip positions.
"""

import os
import re
import tempfile
import numpy as np

import pybullet as p
import pybullet_data


# DG-5F finger definitions
# URDF finger numbering: 1=Thumb, 2=Index, 3=Middle, 4=Ring, 5=Pinky
FINGER_DEFS = {
    'thumb':  {'urdf_prefix': '1', 'joints': ['rj_dg_1_1', 'rj_dg_1_2', 'rj_dg_1_3', 'rj_dg_1_4'], 'tip': 'rl_dg_1_tip'},
    'index':  {'urdf_prefix': '2', 'joints': ['rj_dg_2_1', 'rj_dg_2_2', 'rj_dg_2_3', 'rj_dg_2_4'], 'tip': 'rl_dg_2_tip'},
    'middle': {'urdf_prefix': '3', 'joints': ['rj_dg_3_1', 'rj_dg_3_2', 'rj_dg_3_3', 'rj_dg_3_4'], 'tip': 'rl_dg_3_tip'},
    'ring':   {'urdf_prefix': '4', 'joints': ['rj_dg_4_1', 'rj_dg_4_2', 'rj_dg_4_3', 'rj_dg_4_4'], 'tip': 'rl_dg_4_tip'},
    'pinky':  {'urdf_prefix': '5', 'joints': ['rj_dg_5_1', 'rj_dg_5_2', 'rj_dg_5_3', 'rj_dg_5_4'], 'tip': 'rl_dg_5_tip'},
}

# Joint ordering for output (matches tesollo_slave_node convention)
ALL_JOINT_NAMES = [
    'rj_dg_1_1', 'rj_dg_1_2', 'rj_dg_1_3', 'rj_dg_1_4',  # Thumb
    'rj_dg_2_1', 'rj_dg_2_2', 'rj_dg_2_3', 'rj_dg_2_4',  # Index
    'rj_dg_3_1', 'rj_dg_3_2', 'rj_dg_3_3', 'rj_dg_3_4',  # Middle
    'rj_dg_4_1', 'rj_dg_4_2', 'rj_dg_4_3', 'rj_dg_4_4',  # Ring
    'rj_dg_5_1', 'rj_dg_5_2', 'rj_dg_5_3', 'rj_dg_5_4',  # Pinky
]

PALM_LINK_NAME = 'rl_dg_palm'

# DG-5F reference pose used for solver startup and calibration geometry.
# This matches the existing Tesollo "neutral" pose used by tesollo_slave_node.
DEFAULT_REFERENCE_JOINTS_DEG = np.array([
    77.0, -80.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
], dtype=float)
DEFAULT_REFERENCE_JOINTS_RAD = np.deg2rad(DEFAULT_REFERENCE_JOINTS_DEG)

# Prevent fingertip-only IK from choosing hyperextension branches for the
# flexion joints of the non-thumb fingers. These joints are modeled with
# symmetric limits in the URDF, but the teleop use case wants the natural
# "curl forward from straight" branch.
DEFAULT_POSITIVE_FLEXION_JOINTS = {
    'rj_dg_2_2', 'rj_dg_2_3', 'rj_dg_2_4',
    'rj_dg_3_2', 'rj_dg_3_3', 'rj_dg_3_4',
    'rj_dg_4_2', 'rj_dg_4_3', 'rj_dg_4_4',
    'rj_dg_5_2', 'rj_dg_5_3', 'rj_dg_5_4',
}

# Proximal-to-distal flexion joint chains for biomechanical curl coupling.
# Enforces MCP >= PIP >= DIP so distal joints never flex more than proximal.
# Thumb excluded (different kinematics). Pinky _1 is Z-axis abduction,
# _2 is X-axis spread, so only _3/_4 (Y-axis flexion) participate.
FLEXION_COUPLING_CHAINS = {
    'index':  ('rj_dg_2_2', 'rj_dg_2_3', 'rj_dg_2_4'),
    'middle': ('rj_dg_3_2', 'rj_dg_3_3', 'rj_dg_3_4'),
    'ring':   ('rj_dg_4_2', 'rj_dg_4_3', 'rj_dg_4_4'),
    'pinky':  ('rj_dg_5_3', 'rj_dg_5_4'),
}


def _quat_to_matrix(quat_xyzw) -> np.ndarray:
    """Convert a PyBullet quaternion (x, y, z, w) into a 3x3 rotation matrix."""
    return np.array(p.getMatrixFromQuaternion(quat_xyzw), dtype=float).reshape(3, 3)


def _strip_meshes_from_urdf(urdf_path: str) -> str:
    """Create a temp URDF with visual/collision elements removed (IK only needs kinematics)."""
    with open(urdf_path, 'r') as f:
        content = f.read()

    # Remove <visual>...</visual> and <collision>...</collision> blocks
    content = re.sub(r'<visual>.*?</visual>', '', content, flags=re.DOTALL)
    content = re.sub(r'<collision>.*?</collision>', '', content, flags=re.DOTALL)

    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
    tmp.write(content)
    tmp.close()
    return tmp.name


class DG5FIKSolver:
    """PyBullet-based IK solver for the DG-5F hand."""

    def __init__(self, urdf_path: str, joint_damping: float = 0.1,
                 initial_joint_positions: np.ndarray | None = None,
                 prevent_hyperextension: bool = True,
                 current_pose_weight: float = 0.35,
                 enforce_curl_coupling: bool = True):
        self._physics_client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self._current_pose_weight = float(np.clip(current_pose_weight, 0.0, 1.0))
        self._enforce_curl_coupling = enforce_curl_coupling

        # Load URDF without meshes (faster, no package:// issues)
        stripped_urdf = _strip_meshes_from_urdf(urdf_path)
        self._robot_id = p.loadURDF(stripped_urdf, useFixedBase=True,
                                     physicsClientId=self._physics_client)
        os.unlink(stripped_urdf)

        # Build joint/link index maps
        self._joint_name_to_idx = {}
        self._link_name_to_idx = {}
        self._num_joints = p.getNumJoints(self._robot_id,
                                           physicsClientId=self._physics_client)

        # Separate movable (revolute) joints from fixed joints.
        # PyBullet IK returns values only for movable joints.
        self._movable_joint_indices = []  # pb joint idx → position in IK solution
        self._pb_idx_to_movable = {}      # pb joint idx → index in movable list

        for i in range(self._num_joints):
            info = p.getJointInfo(self._robot_id, i,
                                  physicsClientId=self._physics_client)
            joint_name = info[1].decode('utf-8')
            link_name = info[12].decode('utf-8')
            self._joint_name_to_idx[joint_name] = i
            self._link_name_to_idx[link_name] = i

            if info[2] != p.JOINT_FIXED:
                self._pb_idx_to_movable[i] = len(self._movable_joint_indices)
                self._movable_joint_indices.append(i)

        self._palm_link_idx = self._link_name_to_idx.get(PALM_LINK_NAME, -1)
        if self._palm_link_idx < 0:
            raise RuntimeError(f'Palm link not found in URDF: {PALM_LINK_NAME}')

        num_movable = len(self._movable_joint_indices)
        self._joint_lower = np.zeros(num_movable)
        self._joint_upper = np.zeros(num_movable)
        self._joint_ranges = np.zeros(num_movable)
        self._rest_poses = np.zeros(num_movable)
        self._joint_damping_arr = np.full(num_movable, joint_damping)

        for mi, pb_idx in enumerate(self._movable_joint_indices):
            info = p.getJointInfo(self._robot_id, pb_idx,
                                  physicsClientId=self._physics_client)
            joint_name = info[1].decode('utf-8')
            self._joint_lower[mi] = info[8]
            self._joint_upper[mi] = info[9]
            self._joint_ranges[mi] = info[9] - info[8]
            self._rest_poses[mi] = (info[8] + info[9]) / 2.0

            if prevent_hyperextension and joint_name in DEFAULT_POSITIVE_FLEXION_JOINTS:
                self._joint_lower[mi] = max(self._joint_lower[mi], 0.0)
                self._joint_ranges[mi] = self._joint_upper[mi] - self._joint_lower[mi]

        # Replace the midpoint-of-limits pose with an explicit DG-5F reference pose.
        # The midpoint tends to be semi-clenched, which is a poor match for glove
        # open-hand calibration and causes the hand to start in an unnatural pose.
        if initial_joint_positions is None:
            initial_joint_positions = DEFAULT_REFERENCE_JOINTS_RAD
        initial_joint_positions = np.asarray(initial_joint_positions, dtype=float)
        if initial_joint_positions.shape != (20,):
            raise ValueError(
                f'Expected initial_joint_positions to have shape (20,), got {initial_joint_positions.shape}')
        for i, jname in enumerate(ALL_JOINT_NAMES):
            pb_idx = self._joint_name_to_idx.get(jname)
            if pb_idx is None or pb_idx not in self._pb_idx_to_movable:
                continue
            mi = self._pb_idx_to_movable[pb_idx]
            self._rest_poses[mi] = np.clip(
                initial_joint_positions[i],
                self._joint_lower[mi],
                self._joint_upper[mi],
            )

        # Build per-finger data structures
        # Store movable-space indices for each finger's joints
        self._finger_data = {}
        for finger_name, fdef in FINGER_DEFS.items():
            movable_indices = []  # indices into the movable joint array
            pb_indices = []       # PyBullet joint indices
            for jname in fdef['joints']:
                if jname in self._joint_name_to_idx:
                    pb_idx = self._joint_name_to_idx[jname]
                    pb_indices.append(pb_idx)
                    if pb_idx in self._pb_idx_to_movable:
                        movable_indices.append(self._pb_idx_to_movable[pb_idx])
            tip_idx = self._link_name_to_idx.get(fdef['tip'], -1)
            self._finger_data[finger_name] = {
                'pb_indices': pb_indices,
                'movable_indices': movable_indices,
                'tip_link_idx': tip_idx,
            }

        # Pre-compute movable-space indices for curl coupling chains
        self._coupling_movable_indices = {}
        for finger_name, chain in FLEXION_COUPLING_CHAINS.items():
            mis = []
            for jname in chain:
                pb_idx = self._joint_name_to_idx.get(jname)
                if pb_idx is not None and pb_idx in self._pb_idx_to_movable:
                    mis.append(self._pb_idx_to_movable[pb_idx])
            if len(mis) >= 2:
                self._coupling_movable_indices[finger_name] = mis

        # Current joint state in movable-joint space (warm-start for temporal smoothness)
        self._current_joints = np.copy(self._rest_poses)
        self._set_joint_positions(self._current_joints)

    def _set_joint_positions(self, positions: np.ndarray):
        """Set all movable joint positions in the PyBullet simulation.
        positions is indexed by movable joint index."""
        for mi, pb_idx in enumerate(self._movable_joint_indices):
            p.resetJointState(self._robot_id, pb_idx, positions[mi],
                              physicsClientId=self._physics_client)

    def _get_palm_pose(self):
        state = p.getLinkState(self._robot_id, self._palm_link_idx,
                               computeForwardKinematics=True,
                               physicsClientId=self._physics_client)
        palm_pos = np.array(state[4], dtype=float)
        palm_rot = _quat_to_matrix(state[5])
        return palm_pos, palm_rot

    def _palm_to_world(self, pos_local: np.ndarray) -> np.ndarray:
        palm_pos, palm_rot = self._get_palm_pose()
        return palm_pos + palm_rot @ pos_local

    def _world_to_palm(self, pos_world: np.ndarray) -> np.ndarray:
        palm_pos, palm_rot = self._get_palm_pose()
        return palm_rot.T @ (pos_world - palm_pos)

    def _apply_curl_coupling(self, result_joints: np.ndarray, finger_name: str):
        """Enforce proximal-first curl: MCP >= PIP >= DIP for non-thumb fingers."""
        mis = self._coupling_movable_indices.get(finger_name)
        if mis is None:
            return
        # Walk distal-to-proximal: clamp each distal joint to not exceed its proximal neighbor
        for i in range(len(mis) - 1, 0, -1):
            distal_mi = mis[i]
            proximal_mi = mis[i - 1]
            if result_joints[distal_mi] > result_joints[proximal_mi]:
                result_joints[distal_mi] = result_joints[proximal_mi]

    def solve(self, fingertip_targets: dict, enabled_fingers: dict = None) -> np.ndarray:
        """
        Solve IK for the given fingertip targets.

        Args:
            fingertip_targets: Dict mapping finger name to palm-local xyz position
                               (np.ndarray of shape (3,))
                               Keys: 'thumb', 'index', 'middle', 'ring', 'pinky'
            enabled_fingers: Optional dict mapping finger name to bool. If None, all enabled.

        Returns:
            np.ndarray of shape (20,) — joint angles in ALL_JOINT_NAMES order
        """
        if enabled_fingers is None:
            enabled_fingers = {name: True for name in FINGER_DEFS}

        # Start from current state for warm-starting
        self._set_joint_positions(self._current_joints)

        result_joints = np.copy(self._current_joints)

        for finger_name, fdata in self._finger_data.items():
            if not enabled_fingers.get(finger_name, True):
                continue
            if finger_name not in fingertip_targets:
                continue

            target_pos_world = self._palm_to_world(np.asarray(fingertip_targets[finger_name], dtype=float))
            tip_idx = fdata['tip_link_idx']

            if tip_idx < 0:
                continue

            # PyBullet IK returns values for all movable joints
            rest_pose_target = (
                self._current_pose_weight * self._current_joints
                + (1.0 - self._current_pose_weight) * self._rest_poses
            )
            ik_solution = p.calculateInverseKinematics(
                self._robot_id,
                tip_idx,
                target_pos_world.tolist(),
                lowerLimits=self._joint_lower.tolist(),
                upperLimits=self._joint_upper.tolist(),
                jointRanges=self._joint_ranges.tolist(),
                restPoses=rest_pose_target.tolist(),
                jointDamping=self._joint_damping_arr.tolist(),
                maxNumIterations=100,
                residualThreshold=1e-4,
                physicsClientId=self._physics_client,
            )

            # Extract only this finger's joints from the solution
            # ik_solution is indexed by movable joint order
            for mi in fdata['movable_indices']:
                result_joints[mi] = ik_solution[mi]

            # Enforce biomechanical curl coupling: MCP >= PIP >= DIP
            if self._enforce_curl_coupling:
                self._apply_curl_coupling(result_joints, finger_name)

            # Update simulation state for this finger before solving next
            for mi in fdata['movable_indices']:
                pb_idx = self._movable_joint_indices[mi]
                p.resetJointState(self._robot_id, pb_idx,
                                  result_joints[mi],
                                  physicsClientId=self._physics_client)

        # Clamp to joint limits
        num_movable = len(self._movable_joint_indices)
        for mi in range(num_movable):
            result_joints[mi] = np.clip(result_joints[mi],
                                        self._joint_lower[mi],
                                        self._joint_upper[mi])

        # Update warm-start state
        self._current_joints = result_joints

        # Reorder to match ALL_JOINT_NAMES (20 joints)
        output = np.zeros(20)
        for i, jname in enumerate(ALL_JOINT_NAMES):
            if jname in self._joint_name_to_idx:
                pb_idx = self._joint_name_to_idx[jname]
                if pb_idx in self._pb_idx_to_movable:
                    mi = self._pb_idx_to_movable[pb_idx]
                    output[i] = result_joints[mi]

        return output

    def get_fingertip_position(self, finger_name: str) -> np.ndarray:
        """Get current fingertip position for a finger in palm-local coordinates."""
        fdata = self._finger_data.get(finger_name)
        if fdata is None or fdata['tip_link_idx'] < 0:
            return np.zeros(3)
        state = p.getLinkState(self._robot_id, fdata['tip_link_idx'],
                               computeForwardKinematics=True,
                               physicsClientId=self._physics_client)
        return self._world_to_palm(np.array(state[4], dtype=float))

    def get_palm_fingertip_positions(self) -> dict:
        """Get all current fingertip positions in the DG-5F palm frame."""
        return {name: self.get_fingertip_position(name) for name in FINGER_DEFS}

    def get_palm_fingertip_positions_for_joint_positions(self, joint_positions_output_order) -> dict:
        """Evaluate DG-5F palm-local fingertips at an explicit 20-joint pose."""
        joint_positions_output_order = np.asarray(joint_positions_output_order, dtype=float)
        if joint_positions_output_order.shape != (20,):
            raise ValueError(
                'Expected joint_positions_output_order to have shape (20,), '
                f'got {joint_positions_output_order.shape}')

        saved_joints = np.copy(self._current_joints)
        pose_joints = np.copy(self._current_joints)
        for i, joint_name in enumerate(ALL_JOINT_NAMES):
            pb_idx = self._joint_name_to_idx.get(joint_name)
            if pb_idx is None or pb_idx not in self._pb_idx_to_movable:
                continue
            mi = self._pb_idx_to_movable[pb_idx]
            pose_joints[mi] = np.clip(
                joint_positions_output_order[i],
                self._joint_lower[mi],
                self._joint_upper[mi],
            )

        self._set_joint_positions(pose_joints)
        positions = self.get_palm_fingertip_positions()
        self._set_joint_positions(saved_joints)
        return positions

    def shutdown(self):
        """Disconnect from PyBullet."""
        if self._physics_client >= 0:
            p.disconnect(self._physics_client)
            self._physics_client = -1
