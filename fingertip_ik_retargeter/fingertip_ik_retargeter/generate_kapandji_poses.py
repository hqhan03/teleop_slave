"""Generate Kapandji test poses for the DG-5F hand using PyBullet IK.

Computes joint configurations where the thumb tip meets each fingertip
(index, middle, ring, pinky). These poses are used for multi-pose
calibration of the Manus-to-DG5F fingertip IK retargeter.
"""

import argparse
import os

import numpy as np

from fingertip_ik_retargeter.dg5f_ik_solver import (
    ALL_JOINT_NAMES,
    DEFAULT_REFERENCE_JOINTS_DEG,
    DG5FIKSolver,
)
from fingertip_ik_retargeter.frame_calibration import find_dg5f_urdf


# Seed poses for each Kapandji target.
# Thumb is partially opposed; target finger is partially flexed.
# Non-participating fingers are in a relaxed semi-flexed pose.
_RELAXED_IDLE = [15.0, 15.0, 10.0, 5.0]  # CMC-spread, MCP, PIP, DIP

_KAPANDJI_SEEDS = {
    'index': {
        'thumb_deg': [50.0, -90.0, -20.0, -15.0],
        'target_deg': [0.0, 45.0, 30.0, 20.0],
    },
    'middle': {
        'thumb_deg': [40.0, -110.0, -15.0, -10.0],
        'target_deg': [0.0, 40.0, 25.0, 15.0],
    },
    'ring': {
        'thumb_deg': [30.0, -130.0, -10.0, -10.0],
        'target_deg': [0.0, 45.0, 30.0, 20.0],
    },
    'pinky': {
        'thumb_deg': [25.0, -150.0, -5.0, -5.0],
        'target_deg': [30.0, 20.0, 45.0, 30.0],
    },
}

FINGER_INDICES = {
    'thumb': slice(0, 4),
    'index': slice(4, 8),
    'middle': slice(8, 12),
    'ring': slice(12, 16),
    'pinky': slice(16, 20),
}

FINGER_ORDER = ['thumb', 'index', 'middle', 'ring', 'pinky']


def _build_seed_pose(target_finger: str) -> np.ndarray:
    """Build a 20-DOF seed pose (degrees) for a Kapandji target."""
    seed = _KAPANDJI_SEEDS[target_finger]
    pose = np.zeros(20)

    # Thumb
    pose[FINGER_INDICES['thumb']] = seed['thumb_deg']

    # Target finger
    pose[FINGER_INDICES[target_finger]] = seed['target_deg']

    # Idle fingers: relaxed semi-flexed
    for finger in FINGER_ORDER:
        if finger in ('thumb', target_finger):
            continue
        if finger == 'pinky':
            # Pinky joint 1 is Z-axis abduction, joint 2 is X-axis spread
            pose[FINGER_INDICES[finger]] = [10.0, 10.0, 10.0, 5.0]
        else:
            pose[FINGER_INDICES[finger]] = _RELAXED_IDLE

    return pose


def _solve_kapandji_pose(solver: DG5FIKSolver, target_finger: str,
                         max_iterations: int = 30,
                         tolerance_m: float = 0.005) -> tuple[np.ndarray, float]:
    """Iteratively solve for a Kapandji pose where thumb tip meets target finger tip.

    Returns (joint_angles_deg_20, tip_distance_m).
    """
    seed_deg = _build_seed_pose(target_finger)
    seed_rad = np.deg2rad(seed_deg)

    # Reset solver state to the seed pose
    solver._current_joints = np.copy(solver._rest_poses)
    for i, jname in enumerate(ALL_JOINT_NAMES):
        if jname in solver._joint_name_to_idx:
            pb_idx = solver._joint_name_to_idx[jname]
            if pb_idx in solver._pb_idx_to_movable:
                mi = solver._pb_idx_to_movable[pb_idx]
                solver._current_joints[mi] = np.clip(
                    seed_rad[i],
                    solver._joint_lower[mi],
                    solver._joint_upper[mi],
                )
    solver._set_joint_positions(solver._current_joints)

    best_distance = float('inf')
    best_output = None

    for iteration in range(max_iterations):
        # Get current fingertip positions
        tips = solver.get_palm_fingertip_positions()
        thumb_tip = tips['thumb']
        finger_tip = tips[target_finger]

        distance = float(np.linalg.norm(thumb_tip - finger_tip))
        if best_output is None or distance < best_distance:
            best_distance = distance
            # Extract current joint state
            output = np.zeros(20)
            for i, jname in enumerate(ALL_JOINT_NAMES):
                if jname in solver._joint_name_to_idx:
                    pb_idx = solver._joint_name_to_idx[jname]
                    if pb_idx in solver._pb_idx_to_movable:
                        mi = solver._pb_idx_to_movable[pb_idx]
                        output[i] = solver._current_joints[mi]
            best_output = np.rad2deg(output)

        if distance < tolerance_m:
            break

        # Meeting point: midpoint biased slightly toward the target finger
        # (thumb has more range of motion for opposition)
        meeting_point = 0.45 * thumb_tip + 0.55 * finger_tip

        # Solve IK with only thumb and target finger enabled
        enabled = {name: False for name in FINGER_ORDER}
        enabled['thumb'] = True
        enabled[target_finger] = True

        targets = {
            'thumb': meeting_point,
            target_finger: meeting_point,
        }

        solver.solve(targets, enabled)

    # Final distance check
    tips = solver.get_palm_fingertip_positions()
    final_distance = float(np.linalg.norm(tips['thumb'] - tips[target_finger]))
    if final_distance < best_distance:
        best_distance = final_distance
        output = np.zeros(20)
        for i, jname in enumerate(ALL_JOINT_NAMES):
            if jname in solver._joint_name_to_idx:
                pb_idx = solver._joint_name_to_idx[jname]
                if pb_idx in solver._pb_idx_to_movable:
                    mi = solver._pb_idx_to_movable[pb_idx]
                    output[i] = solver._current_joints[mi]
        best_output = np.rad2deg(output)

    # Restore idle finger angles from the seed
    for finger in FINGER_ORDER:
        if finger in ('thumb', target_finger):
            continue
        best_output[FINGER_INDICES[finger]] = _build_seed_pose(target_finger)[FINGER_INDICES[finger]]

    return best_output, best_distance


def main():
    parser = argparse.ArgumentParser(description='Generate Kapandji test poses for DG-5F')
    parser.add_argument('--urdf', default='', help='Path to dg5f_right.urdf')
    parser.add_argument('--joint-damping', default=0.05, type=float)
    parser.add_argument('--max-iterations', default=50, type=int)
    parser.add_argument('--tolerance', default=0.008, type=float,
                        help='Acceptable thumb-to-finger tip distance in meters')
    args = parser.parse_args()

    urdf_path = args.urdf or find_dg5f_urdf()
    if not os.path.exists(urdf_path):
        print(f'URDF not found: {urdf_path}')
        raise SystemExit(1)

    print(f'URDF: {urdf_path}')
    print(f'Damping: {args.joint_damping}, Max iterations: {args.max_iterations}, '
          f'Tolerance: {args.tolerance}m')
    print()

    solver = DG5FIKSolver(
        urdf_path,
        joint_damping=args.joint_damping,
        prevent_hyperextension=False,  # allow full range for thumb opposition
        current_pose_weight=0.2,
    )

    try:
        targets = ['index', 'middle', 'ring', 'pinky']
        results = {}

        for target in targets:
            pose_deg, distance = _solve_kapandji_pose(
                solver, target,
                max_iterations=args.max_iterations,
                tolerance_m=args.tolerance,
            )
            results[target] = (pose_deg, distance)

            status = 'OK' if distance < args.tolerance else 'WARN'
            print(f'=== kapandji_{target} ({status}: tip distance = {distance*1000:.1f}mm) ===')

            # Print as Python list for copy-pasting into frame_calibration.py
            rounded = np.round(pose_deg, 1)
            lines = []
            for finger in FINGER_ORDER:
                sl = FINGER_INDICES[finger]
                vals = rounded[sl]
                lines.append(f'    {vals[0]}, {vals[1]}, {vals[2]}, {vals[3]},  # {finger.capitalize()}')
            print('{')
            print(f"    'name': 'kapandji_{target}',")
            print(f"    'description': 'KAPANDJI {target.upper()} "
                  f"\\u2014 touch thumb tip to {target} fingertip',")
            print(f"    'joint_positions_deg': [")
            for line in lines:
                print(line)
            print(f"    ],")
            print('},')

            # Print fingertip positions for validation
            tips = solver.get_palm_fingertip_positions_for_joint_positions(np.deg2rad(pose_deg))
            print(f'  thumb_tip:  {np.round(tips["thumb"], 4).tolist()}')
            print(f'  {target}_tip: {np.round(tips[target], 4).tolist()}')
            print()

            # Re-create solver for clean state between poses
            solver.shutdown()
            solver = DG5FIKSolver(
                urdf_path,
                joint_damping=args.joint_damping,
                prevent_hyperextension=False,
                current_pose_weight=0.2,
            )

        print('=== Summary ===')
        all_ok = True
        for target in targets:
            pose_deg, distance = results[target]
            status = 'OK' if distance < args.tolerance else 'FAIL'
            if distance >= args.tolerance:
                all_ok = False
            print(f'  kapandji_{target}: {distance*1000:.1f}mm [{status}]')

        if not all_ok:
            print('\nWARNING: Some poses exceed the tolerance. '
                  'Consider adjusting seeds or increasing iterations.')

    finally:
        solver.shutdown()


if __name__ == '__main__':
    main()
