"""Guided stage-2 multi-pose calibration for Manus wrist-local -> DG5F fingertip IK."""

from __future__ import annotations

import os
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from fingertip_ik_retargeter.dg5f_ik_solver import (
    ALL_JOINT_NAMES,
    DEFAULT_REFERENCE_JOINTS_DEG,
    DG5FIKSolver,
)
from fingertip_ik_retargeter.frame_calibration import (
    DEFAULT_CALIBRATION_PATH,
    DG5F_MULTI_POSE_CALIBRATION,
    MANUS_FINGER_ORDER,
    find_dg5f_urdf,
    fit_retarget_scales,
    load_calibration,
    save_calibration,
)
from fingertip_ik_retargeter.manus_frames import (
    CANONICAL_MANUS_HAND_FRAME,
    LEGACY_MANUS_HAND_FRAME,
    classify_manus_hand_frame,
)


class IKMultiPoseCalibrator(Node):
    def __init__(self):
        super().__init__('ik_multipose_calibrator')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('dummy_mode', False)
        self.declare_parameter('sample_count', 40)
        self.declare_parameter('settle_time_sec', 2.5)
        self.declare_parameter('command_period_sec', 0.25)
        self.declare_parameter('calibration_file', DEFAULT_CALIBRATION_PATH)
        self.declare_parameter('output_path', DEFAULT_CALIBRATION_PATH)
        self.declare_parameter('initial_joint_positions_deg', DEFAULT_REFERENCE_JOINTS_DEG.tolist())

        urdf_path = self.get_parameter('urdf_path').value or find_dg5f_urdf()
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f'URDF not found: {urdf_path}')

        self._dummy_mode = bool(self.get_parameter('dummy_mode').value)
        self._sample_count = int(self.get_parameter('sample_count').value)
        self._settle_time_sec = float(self.get_parameter('settle_time_sec').value)
        self._command_period_sec = float(self.get_parameter('command_period_sec').value)
        self._calibration_file = self.get_parameter('calibration_file').value
        self._output_path = self.get_parameter('output_path').value
        self._initial_joint_positions_deg = np.array(
            self.get_parameter('initial_joint_positions_deg').value, dtype=float)

        self._calibration = load_calibration(self._calibration_file)
        if not self._calibration:
            raise RuntimeError(
                f'No stage-1 calibration found at {self._calibration_file}. '
                'Run calibrate_ik_frame first.')

        required_keys = ('manus_to_dg5f_axes', 'manus_to_dg5f_signs', 'hand_scale')
        missing = [key for key in required_keys if key not in self._calibration]
        if missing:
            raise RuntimeError(
                f'Stage-1 calibration file is missing required keys {missing}. '
                'Re-run calibrate_ik_frame first.')

        if 'initial_joint_positions_deg' in self._calibration:
            self._initial_joint_positions_deg = np.array(
                self._calibration['initial_joint_positions_deg'], dtype=float)

        traj_topic = '/joint_trajectory_controller/joint_trajectory' if self._dummy_mode else \
                     '/dg5f_right/dg5f_right_controller/joint_trajectory'
        self._traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)
        self._sub = self.create_subscription(PoseArray, '/manus/fingertip_positions', self._callback, 10)

        self._capture_buffer = []
        self._capture_active = False
        self._warned_legacy_hand_frame = False
        self._warned_invalid_hand_frame = False

        self._solver = DG5FIKSolver(
            urdf_path,
            initial_joint_positions=np.deg2rad(self._initial_joint_positions_deg),
        )
        self._pose_tip_targets = {
            pose['name']: self._solver.get_palm_fingertip_positions_for_joint_positions(
                np.deg2rad(np.asarray(pose['joint_positions_deg'], dtype=float))
            )
            for pose in DG5F_MULTI_POSE_CALIBRATION
        }

        self.get_logger().info(
            f'Stage-2 multi-pose calibration ready. Publishing reference poses to {traj_topic}.')
        self.get_logger().info(
            f'Loaded stage-1 calibration from {self._calibration_file}. '
            f'Will save updated calibration to {self._output_path}.')

    def _callback(self, msg: PoseArray):
        if len(msg.poses) < 5:
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
            sample[name] = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)

        if not any(np.linalg.norm(tip) > 1e-8 for tip in sample.values()):
            return

        if self._capture_active:
            self._capture_buffer.append(sample)

    def _publish_pose_deg(self, pose_deg):
        pose_deg = np.asarray(pose_deg, dtype=float)
        traj_msg = JointTrajectory()
        traj_msg.joint_names = list(ALL_JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = np.deg2rad(pose_deg).tolist()
        point.time_from_start = Duration(sec=1, nanosec=0)
        traj_msg.points.append(point)
        self._traj_pub.publish(traj_msg)

    def _hold_pose(self, pose_deg, duration_sec: float):
        end_time = time.monotonic() + duration_sec
        next_pub_time = 0.0
        while time.monotonic() < end_time:
            now = time.monotonic()
            if now >= next_pub_time:
                self._publish_pose_deg(pose_deg)
                next_pub_time = now + self._command_period_sec
            rclpy.spin_once(self, timeout_sec=0.05)

    def _capture_average(self, pose_deg) -> dict:
        self._capture_buffer = []
        self._capture_active = True
        next_pub_time = 0.0
        while len(self._capture_buffer) < self._sample_count:
            now = time.monotonic()
            if now >= next_pub_time:
                self._publish_pose_deg(pose_deg)
                next_pub_time = now + self._command_period_sec
            rclpy.spin_once(self, timeout_sec=0.05)
        self._capture_active = False

        averaged = {}
        for name in MANUS_FINGER_ORDER:
            averaged[name] = np.mean([sample[name] for sample in self._capture_buffer], axis=0)
        return averaged

    def run(self):
        manus_pose_samples = {}
        total = len(DG5F_MULTI_POSE_CALIBRATION)

        for idx, pose in enumerate(DG5F_MULTI_POSE_CALIBRATION, start=1):
            pose_deg = pose['joint_positions_deg']
            self.get_logger().info(
                f'[{idx}/{total}] Commanding calibration pose "{pose["name"]}": {pose["description"]}')
            self._hold_pose(pose_deg, self._settle_time_sec)
            input(
                f'\nPose {idx}/{total}: {pose["description"]}\n'
                'Match your Manus glove hand to the robot hand, then press Enter to capture...')
            manus_pose_samples[pose['name']] = self._capture_average(pose_deg)
            self.get_logger().info(
                f'Captured {self._sample_count} fingertip frames for pose "{pose["name"]}".')

        gains = fit_retarget_scales(
            manus_pose_samples,
            self._pose_tip_targets,
            self._calibration['manus_to_dg5f_axes'],
            self._calibration['manus_to_dg5f_signs'],
            self._calibration['hand_scale'],
            self._calibration.get('palm_offset', [0.0, 0.0, 0.0]),
        )

        self._calibration.update(gains)
        self._calibration['thumb_target_scale'] = 1.0
        self._calibration['initial_joint_positions_deg'] = self._initial_joint_positions_deg.tolist()
        self._calibration['multipose_pose_order'] = [pose['name'] for pose in DG5F_MULTI_POSE_CALIBRATION]
        save_calibration(self._calibration, self._output_path)

        self.get_logger().info(
            f'Saved stage-2 calibration to {self._output_path}: '
            f'workspace_axis_scale={np.round(gains["workspace_axis_scale"], 4).tolist()} '
            f'finger_target_scales={np.round(gains["finger_target_scales"], 4).tolist()} '
            f'rmse={gains["multipose_rmse"]:.6f}')

    def destroy_node(self):
        self._solver.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IKMultiPoseCalibrator()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
