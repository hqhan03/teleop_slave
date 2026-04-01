"""Capture a wrist-local open-hand sample and save Manus->DG5F frame calibration."""

from __future__ import annotations

import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node

from fingertip_ik_retargeter.dg5f_ik_solver import DG5FIKSolver
from fingertip_ik_retargeter.dg5f_ik_solver import DEFAULT_REFERENCE_JOINTS_DEG
from fingertip_ik_retargeter.frame_calibration import (
    DEFAULT_CALIBRATION_PATH,
    DEFAULT_FINGER_TARGET_SCALES,
    DEFAULT_WORKSPACE_AXIS_SCALE,
    MANUS_FINGER_ORDER,
    compute_calibration,
    find_dg5f_urdf,
    save_calibration,
)
from fingertip_ik_retargeter.manus_frames import (
    CANONICAL_MANUS_HAND_FRAME,
    LEGACY_MANUS_HAND_FRAME,
    classify_manus_hand_frame,
)


class IKFrameCalibrator(Node):
    def __init__(self):
        super().__init__('ik_frame_calibrator')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('sample_count', 30)
        self.declare_parameter('output_path', DEFAULT_CALIBRATION_PATH)
        self.declare_parameter('initial_joint_positions_deg', DEFAULT_REFERENCE_JOINTS_DEG.tolist())

        urdf_path = self.get_parameter('urdf_path').value or find_dg5f_urdf()
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f'URDF not found: {urdf_path}')

        self._sample_count = int(self.get_parameter('sample_count').value)
        self._output_path = self.get_parameter('output_path').value
        initial_joint_positions_deg = np.array(
            self.get_parameter('initial_joint_positions_deg').value, dtype=float)
        self._initial_joint_positions_deg = initial_joint_positions_deg
        self._samples = []
        self._solver = DG5FIKSolver(
            urdf_path,
            initial_joint_positions=np.deg2rad(initial_joint_positions_deg),
        )
        self._dg5f_tips = self._solver.get_palm_fingertip_positions()
        self._warned_legacy_hand_frame = False
        self._warned_invalid_hand_frame = False

        self._sub = self.create_subscription(
            PoseArray,
            '/manus/fingertip_positions',
            self._callback,
            10,
        )

        self.get_logger().info(
            f'IK frame calibration ready. Hold an open hand in front of the glove; '
            f'capturing {self._sample_count} wrist-local samples from /manus/fingertip_positions.')
        self.get_logger().info(
            f'Using DG-5F reference pose for calibration (deg): '
            f'{np.round(initial_joint_positions_deg, 1).tolist()}')

    def _callback(self, msg: PoseArray):
        if len(msg.poses) < 5:
            self.get_logger().warn('Need 5 fingertip poses for calibration.', throttle_duration_sec=2.0)
            return
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

        sample = {}
        for idx, name in enumerate(MANUS_FINGER_ORDER):
            pose = msg.poses[idx]
            tip = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
            sample[name] = tip

        if not any(np.linalg.norm(tip) > 1e-8 for tip in sample.values()):
            self.get_logger().warn('Skipping all-zero fingertip frame.', throttle_duration_sec=2.0)
            return

        self._samples.append(sample)
        if len(self._samples) < self._sample_count:
            self.get_logger().info(f'Captured calibration sample {len(self._samples)}/{self._sample_count}')
            return

        manus_avg = {}
        for name in MANUS_FINGER_ORDER:
            manus_avg[name] = np.mean([sample[name] for sample in self._samples], axis=0)

        calibration = compute_calibration(manus_avg, self._dg5f_tips)
        calibration['initial_joint_positions_deg'] = self._initial_joint_positions_deg.tolist()
        calibration['workspace_axis_scale'] = list(DEFAULT_WORKSPACE_AXIS_SCALE)
        calibration['finger_target_scales'] = list(DEFAULT_FINGER_TARGET_SCALES)
        calibration['thumb_target_scale'] = 1.0
        save_calibration(calibration, self._output_path)
        self.get_logger().info(
            f'Saved IK frame calibration to {self._output_path}: '
            f'axes={calibration["manus_to_dg5f_axes"]} '
            f'signs={calibration["manus_to_dg5f_signs"]} '
            f'hand_scale={calibration["hand_scale"]:.4f}')
        self.destroy_node()
        rclpy.shutdown()

    def destroy_node(self):
        self._solver.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IKFrameCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
