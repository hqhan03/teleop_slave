"""
Fingertip IK Retargeting Node for Tesollo DG-5F.

Subscribes to Manus glove fingertip positions in a wrist-local hand frame,
solves IK via PyBullet,
and publishes joint commands to the DG-5F hand.
"""

import os
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from fingertip_ik_retargeter.dg5f_ik_solver import (
    ALL_JOINT_NAMES,
    DEFAULT_REFERENCE_JOINTS_DEG,
    DG5FIKSolver,
)
from fingertip_ik_retargeter.frame_calibration import (
    DEFAULT_CALIBRATION_PATH,
    MANUS_FINGER_ORDER,
    apply_axis_mapping,
    find_dg5f_urdf,
    load_calibration,
)
from fingertip_ik_retargeter.manus_frames import (
    CANONICAL_MANUS_HAND_FRAME,
    LEGACY_MANUS_HAND_FRAME,
    classify_manus_hand_frame,
)


def _quat_to_rotation_matrix(w, x, y, z):
    """Convert quaternion (w, x, y, z) to a 3x3 rotation matrix (numpy-only)."""
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


class FingertipIKNode(Node):
    def __init__(self):
        super().__init__('fingertip_ik_node')

        # Parameters
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('dummy_mode', False)
        self.declare_parameter('hand_scale', 1.0)
        self.declare_parameter('joint_damping', 0.1)
        self.declare_parameter('current_pose_weight', 0.35)
        self.declare_parameter('palm_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('initial_joint_positions_deg', DEFAULT_REFERENCE_JOINTS_DEG.tolist())
        self.declare_parameter('workspace_axis_scale', [1.0, 1.45, 1.0])
        self.declare_parameter('finger_target_scales', [1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('thumb_target_scale', 1.35)
        self.declare_parameter('prevent_hyperextension', True)
        self.declare_parameter('log_every_n_frames', 100)
        self.declare_parameter('input_frame_mode', 'palm_local')
        self.declare_parameter('calibration_file', DEFAULT_CALIBRATION_PATH)
        self.declare_parameter('enable_thumb', True)
        self.declare_parameter('enable_index', True)
        self.declare_parameter('enable_middle', True)
        self.declare_parameter('enable_ring', True)
        self.declare_parameter('enable_pinky', True)
        self.declare_parameter('manus_to_dg5f_axes', [1, 2, 0])
        self.declare_parameter('manus_to_dg5f_signs', [-1.0, 1.0, 1.0])

        urdf_path = self.get_parameter('urdf_path').value
        self._dummy_mode = self.get_parameter('dummy_mode').value
        self._input_frame_mode = self.get_parameter('input_frame_mode').value
        self._calibration_file = self.get_parameter('calibration_file').value
        self._hand_scale = self.get_parameter('hand_scale').value
        self._log_every_n_frames = max(1, int(self.get_parameter('log_every_n_frames').value))
        joint_damping = self.get_parameter('joint_damping').value
        current_pose_weight = float(self.get_parameter('current_pose_weight').value)
        initial_joint_positions_deg = self.get_parameter('initial_joint_positions_deg').value
        self._workspace_axis_scale = np.array(self.get_parameter('workspace_axis_scale').value, dtype=float)
        self._finger_target_scales = {
            name: float(scale)
            for name, scale in zip(MANUS_FINGER_ORDER, self.get_parameter('finger_target_scales').value)
        }
        self._thumb_target_scale = float(self.get_parameter('thumb_target_scale').value)
        prevent_hyperextension = bool(self.get_parameter('prevent_hyperextension').value)
        self._palm_offset = np.array(self.get_parameter('palm_offset').value)
        self._manus_to_dg5f_axes = list(self.get_parameter('manus_to_dg5f_axes').value)
        self._manus_to_dg5f_signs = np.array(self.get_parameter('manus_to_dg5f_signs').value)
        self._initial_joint_positions_deg = np.array(initial_joint_positions_deg, dtype=float)

        self._load_calibration_file()

        if self._input_frame_mode not in ('palm_local', 'legacy_tracker_world'):
            raise ValueError(
                f'Unsupported input_frame_mode={self._input_frame_mode}. '
                'Expected palm_local or legacy_tracker_world.')

        # Wrist pose state for legacy world-space inputs
        self._wrist_position = None  # np.array([x, y, z])
        self._wrist_inv_rot = None   # 3x3 inverse rotation matrix

        self._enabled_fingers = {
            'thumb': self.get_parameter('enable_thumb').value,
            'index': self.get_parameter('enable_index').value,
            'middle': self.get_parameter('enable_middle').value,
            'ring': self.get_parameter('enable_ring').value,
            'pinky': self.get_parameter('enable_pinky').value,
        }

        # Resolve URDF path
        if not urdf_path:
            urdf_path = find_dg5f_urdf()
        if not os.path.exists(urdf_path):
            self.get_logger().fatal(f'URDF not found: {urdf_path}')
            raise FileNotFoundError(f'URDF not found: {urdf_path}')

        self.get_logger().info(f'Loading DG-5F URDF: {urdf_path}')

        # Initialize IK solver
        self._solver = DG5FIKSolver(
            urdf_path,
            joint_damping=joint_damping,
            initial_joint_positions=np.deg2rad(self._initial_joint_positions_deg),
            prevent_hyperextension=prevent_hyperextension,
            current_pose_weight=current_pose_weight,
        )

        # Log DG-5F rest fingertip positions for reference
        rest_tips = self._solver.get_palm_fingertip_positions()
        self.get_logger().info('DG-5F rest fingertip positions (palm frame):')
        for name, pos in rest_tips.items():
            self.get_logger().info(f'  {name:7s}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]')

        # Publisher — same topic as tesollo_slave_node uses
        traj_topic = '/joint_trajectory_controller/joint_trajectory' if self._dummy_mode else \
                     '/dg5f_right/dg5f_right_controller/joint_trajectory'
        self._traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)
        self.get_logger().info(f'Publishing to: {traj_topic}')

        # Subscribers
        self._fingertip_sub = self.create_subscription(
            PoseArray,
            '/manus/fingertip_positions',
            self._fingertip_callback,
            10)

        self._legacy_world_mode = self._input_frame_mode == 'legacy_tracker_world'
        if self._legacy_world_mode:
            self._wrist_sub = self.create_subscription(
                PoseStamped,
                '/manus/wrist_pose',
                self._wrist_callback,
                10)
            self.get_logger().info(
                'Legacy tracker-world mode ENABLED — converting /manus/fingertip_positions '
                'from tracker world into the palm frame using /manus/wrist_pose')
        else:
            self.get_logger().info(
                'Hand-local mode ENABLED — consuming /manus/fingertip_positions '
                f'directly in the Manus wrist-local frame ({CANONICAL_MANUS_HAND_FRAME})')
        self.get_logger().info(
            f'Axis remap: DG5F[X,Y,Z] <- Manus[{self._manus_to_dg5f_axes}] '
            f'* {self._manus_to_dg5f_signs.tolist()}')

        self._msg_count = 0
        self._first_wrist_logged = False
        self._first_msg_logged = False
        self._warned_legacy_hand_frame = False
        self._warned_invalid_hand_frame = False

        enabled_list = [n for n, e in self._enabled_fingers.items() if e]
        self.get_logger().info(f'Enabled fingers: {enabled_list}')
        self.get_logger().info(f'Hand scale: {self._hand_scale}')
        self.get_logger().info(f'Current-pose IK weight: {current_pose_weight}')
        self.get_logger().info(f'Workspace axis scale: {np.round(self._workspace_axis_scale, 3).tolist()}')
        self.get_logger().info(f'Finger target scales: {self._finger_target_scales}')
        self.get_logger().info(f'Thumb target scale: {self._thumb_target_scale}')
        self.get_logger().info(f'Palm offset: {np.round(self._palm_offset, 6).tolist()}')
        self.get_logger().info(
            f'Initial DG-5F reference pose (deg): {np.round(self._initial_joint_positions_deg, 1).tolist()}')
        self.get_logger().info('Fingertip IK retargeter ready. Waiting for /manus/fingertip_positions...')

    def _load_calibration_file(self):
        calibration = load_calibration(self._calibration_file)
        if not calibration:
            self.get_logger().info(
                f'No calibration file found at {self._calibration_file}; using configured defaults.')
            return

        self._input_frame_mode = calibration.get('input_frame_mode', self._input_frame_mode)
        self._manus_to_dg5f_axes = list(calibration.get('manus_to_dg5f_axes', self._manus_to_dg5f_axes))
        self._manus_to_dg5f_signs = np.array(
            calibration.get('manus_to_dg5f_signs', self._manus_to_dg5f_signs), dtype=float)
        self._hand_scale = float(calibration.get('hand_scale', self._hand_scale))
        self._palm_offset = np.array(calibration.get('palm_offset', self._palm_offset), dtype=float)
        self._workspace_axis_scale = np.array(
            calibration.get('workspace_axis_scale', self._workspace_axis_scale), dtype=float)
        finger_target_scales = calibration.get(
            'finger_target_scales',
            [self._finger_target_scales[name] for name in MANUS_FINGER_ORDER],
        )
        self._finger_target_scales = {
            name: float(scale) for name, scale in zip(MANUS_FINGER_ORDER, finger_target_scales)
        }
        self._thumb_target_scale = float(
            calibration.get('thumb_target_scale', self._thumb_target_scale))
        self._initial_joint_positions_deg = np.array(
            calibration.get('initial_joint_positions_deg', self._initial_joint_positions_deg),
            dtype=float,
        )
        self.get_logger().info(f'Loaded frame calibration from {self._calibration_file}')

    def _wrist_callback(self, msg: PoseStamped):
        """Cache wrist position and inverse rotation matrix."""
        p = msg.pose.position
        q = msg.pose.orientation
        self._wrist_position = np.array([p.x, p.y, p.z])
        rot = _quat_to_rotation_matrix(q.w, q.x, q.y, q.z)
        self._wrist_inv_rot = rot.T  # inverse of rotation matrix = transpose

        if not self._first_wrist_logged:
            self._first_wrist_logged = True
            self.get_logger().info(
                f'First wrist pose received: pos=[{p.x:.4f}, {p.y:.4f}, {p.z:.4f}] '
                f'quat=[{q.w:.4f}, {q.x:.4f}, {q.y:.4f}, {q.z:.4f}]')

    def _tracker_world_to_palm(self, pos: np.ndarray) -> np.ndarray:
        """Transform a tracker-world position into the palm (wrist) frame."""
        return self._wrist_inv_rot @ (pos - self._wrist_position)

    def _fingertip_callback(self, msg: PoseArray):
        """Process incoming fingertip positions and publish IK-solved joint commands."""
        # Guard: need wrist pose before we can transform legacy world-space inputs
        if self._legacy_world_mode and self._wrist_position is None:
            self.get_logger().warn(
                'Waiting for first /manus/wrist_pose before processing fingertips...',
                throttle_duration_sec=2.0)
            return

        if not self._first_msg_logged:
            self._first_msg_logged = True
            self.get_logger().info(
                f'First message received on /manus/fingertip_positions '
                f'({len(msg.poses)} poses)')

        if len(msg.poses) < 5:
            self.get_logger().warn(f'Expected 5 fingertip poses, got {len(msg.poses)}', throttle_duration_sec=5.0)
            return

        # Check for zero data (no fingertip data from sender)
        has_data = False
        for pose in msg.poses[:5]:
            if pose.position.x != 0.0 or pose.position.y != 0.0 or pose.position.z != 0.0:
                has_data = True
                break
        if not has_data:
            self.get_logger().warn(
                'All fingertip positions are zero — sender may not be providing '
                'fingertip data. Check Manus SDK raw skeleton stream.',
                throttle_duration_sec=5.0)
            return

        if self._input_frame_mode == 'palm_local':
            frame_ok, is_legacy = classify_manus_hand_frame(msg.header.frame_id)
            if not frame_ok:
                if not self._warned_invalid_hand_frame:
                    self.get_logger().warn(
                        f'Ignoring fingertip message in unexpected frame {msg.header.frame_id!r}. '
                        f'Expected {CANONICAL_MANUS_HAND_FRAME!r} or legacy {LEGACY_MANUS_HAND_FRAME!r}.')
                    self._warned_invalid_hand_frame = True
                return
            if is_legacy and not self._warned_legacy_hand_frame:
                self.get_logger().warn(
                    f'Received legacy fingertip frame {LEGACY_MANUS_HAND_FRAME!r}; '
                    f'treating it as wrist-local. Please migrate publishers to '
                    f'{CANONICAL_MANUS_HAND_FRAME!r}.')
                self._warned_legacy_hand_frame = True

        # Extract fingertip positions and apply calibration/remapping
        fingertip_targets = {}
        for i, finger_name in enumerate(MANUS_FINGER_ORDER):
            if not self._enabled_fingers.get(finger_name, True):
                continue
            pose = msg.poses[i]
            pos = np.array([pose.position.x, pose.position.y, pose.position.z])

            if self._legacy_world_mode:
                pos = self._tracker_world_to_palm(pos)

            pos = apply_axis_mapping(pos, self._manus_to_dg5f_axes, self._manus_to_dg5f_signs)
            scale = self._hand_scale * self._workspace_axis_scale * self._finger_target_scales.get(finger_name, 1.0)
            if finger_name == 'thumb':
                scale = scale * self._thumb_target_scale
            pos = pos * scale + self._palm_offset
            fingertip_targets[finger_name] = pos

        # Solve IK
        joint_angles = self._solver.solve(fingertip_targets, self._enabled_fingers)
        solved_tips = self._solver.get_palm_fingertip_positions()

        # Publish JointTrajectory
        traj_msg = JointTrajectory()
        traj_msg.joint_names = list(ALL_JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = joint_angles.tolist()
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)  # 0.1s

        traj_msg.points.append(point)
        self._traj_pub.publish(traj_msg)

        # Periodic logging
        self._msg_count += 1
        if self._msg_count % self._log_every_n_frames == 1:
            error_parts = []
            for finger_name in MANUS_FINGER_ORDER:
                if finger_name not in fingertip_targets:
                    continue
                tip_error = np.linalg.norm(solved_tips[finger_name] - fingertip_targets[finger_name])
                error_parts.append(f'{finger_name}:{tip_error:.4f}m')
            wrist_str = ''
            if self._legacy_world_mode and self._wrist_position is not None:
                wrist_str = f' wrist={np.round(self._wrist_position, 4).tolist()}'
            self.get_logger().info(
                f'[IK] mode={self._input_frame_mode} frame={self._msg_count}{wrist_str} '
                f'errors=({" ".join(error_parts)}) '
                f'thumb_target={np.round(fingertip_targets.get("thumb", np.zeros(3)), 4).tolist()} '
                f'thumb_solved={np.round(solved_tips.get("thumb", np.zeros(3)), 4).tolist()}')

    def destroy_node(self):
        self._solver.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FingertipIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
