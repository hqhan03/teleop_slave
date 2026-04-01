"""
Offline smoke test for the DG-5F fingertip IK solver.

This lets us validate the PyBullet retargeting path without requiring ROS topic
transport or DG-5F hardware.
"""

import argparse
import os

import numpy as np

from fingertip_ik_retargeter.dg5f_ik_solver import DG5FIKSolver, ALL_JOINT_NAMES
from fingertip_ik_retargeter.frame_calibration import find_dg5f_urdf


def _build_targets(rest_tips: dict[str, np.ndarray], profile: str) -> dict[str, np.ndarray]:
    if profile == 'curl':
        deltas = {
            'thumb': np.array([0.000, 0.010, -0.030]),
            'index': np.array([-0.010, 0.000, -0.020]),
            'middle': np.array([0.000, 0.000, -0.020]),
            'ring': np.array([0.000, 0.000, -0.015]),
            'pinky': np.array([0.000, 0.000, -0.010]),
        }
    elif profile == 'pinch':
        deltas = {
            'thumb': np.array([0.035, 0.055, -0.110]),
            'index': np.array([-0.030, 0.000, 0.050]),
            'middle': np.array([0.000, 0.000, -0.005]),
            'ring': np.array([0.000, 0.000, -0.005]),
            'pinky': np.array([0.000, 0.000, -0.005]),
        }
    else:
        raise ValueError(f'Unsupported profile: {profile}')

    return {name: rest_tips[name] + delta for name, delta in deltas.items()}


def main() -> int:
    parser = argparse.ArgumentParser(description='Offline DG-5F fingertip IK smoke test')
    parser.add_argument('--urdf', default='', help='Path to dg5f_right.urdf')
    parser.add_argument('--profile', default='curl', choices=['curl', 'pinch'],
                        help='Synthetic fingertip target profile to test')
    parser.add_argument('--joint-damping', default=0.1, type=float,
                        help='PyBullet IK damping value')
    args = parser.parse_args()

    urdf_path = args.urdf or find_dg5f_urdf()
    if not os.path.exists(urdf_path):
        print(f'URDF not found: {urdf_path}')
        return 1

    solver = DG5FIKSolver(urdf_path, joint_damping=args.joint_damping)
    try:
        rest_tips = solver.get_palm_fingertip_positions()
        targets = _build_targets(rest_tips, args.profile)
        solution = solver.solve(targets)
        solved_tips = solver.get_palm_fingertip_positions()

        print(f'profile={args.profile}')
        print(f'urdf={urdf_path}')
        print('rest_tips:')
        for name in ['thumb', 'index', 'middle', 'ring', 'pinky']:
            print(f'  {name:7s} {np.round(rest_tips[name], 4).tolist()}')

        print('joint_solution_rad:')
        for name, value in zip(ALL_JOINT_NAMES, solution):
            print(f'  {name:10s} {float(value): .6f}')

        print('tip_errors_m:')
        worst_error = 0.0
        for name in ['thumb', 'index', 'middle', 'ring', 'pinky']:
            error = float(np.linalg.norm(solved_tips[name] - targets[name]))
            worst_error = max(worst_error, error)
            print(
                f'  {name:7s} error={error:.5f} '
                f'target={np.round(targets[name], 4).tolist()} '
                f'solved={np.round(solved_tips[name], 4).tolist()}'
            )

        print(f'worst_tip_error_m={worst_error:.5f}')
        return 0
    finally:
        solver.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
