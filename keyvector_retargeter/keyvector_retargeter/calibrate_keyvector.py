"""Stage-1 and stage-2 calibration tool for the keyvector retargeter."""

from __future__ import annotations

import os
import time

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseArray
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .calibration import (
    CALIBRATION_POSES,
    CalibrationPoseCapture,
    DEFAULT_CALIBRATION_PATH,
    compute_stage1_calibration,
    fit_stage2_calibration,
    load_calibration,
    save_calibration,
)
from .dg5f_model import DEFAULT_REFERENCE_JOINTS_DEG, DG5FHandModel, find_dg5f_urdf
from .manus_landmarks import decode_manus_landmarks_msg
from .manus_joints import decode_manus_joint_state_msg


class KeyvectorCalibrationNode(Node):
    """Interactive calibration utility for stage-1 frame fitting and stage-2 beta fitting."""

    def __init__(self):
        super().__init__('calibrate_keyvector')

        self.declare_parameter('mode', 'stage1')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('sample_count', 30)
        self.declare_parameter('output_path', DEFAULT_CALIBRATION_PATH)
        self.declare_parameter('calibration_file', DEFAULT_CALIBRATION_PATH)
        self.declare_parameter('dummy_mode', False)
        self.declare_parameter('settle_time_sec', 2.5)
        self.declare_parameter('command_period_sec', 0.25)
        self.declare_parameter('reference_joint_positions_deg', DEFAULT_REFERENCE_JOINTS_DEG.tolist())

        self._mode = str(self.get_parameter('mode').value).strip().lower()
        self._sample_count = int(self.get_parameter('sample_count').value)
        self._output_path = os.path.expanduser(str(self.get_parameter('output_path').value))
        self._calibration_file = os.path.expanduser(str(self.get_parameter('calibration_file').value))
        self._dummy_mode = bool(self.get_parameter('dummy_mode').value)
        self._settle_time_sec = float(self.get_parameter('settle_time_sec').value)
        self._command_period_sec = float(self.get_parameter('command_period_sec').value)
        self._reference_joint_positions = np.deg2rad(
            np.asarray(self.get_parameter('reference_joint_positions_deg').value, dtype=float)
        )
        self._reference_joint_positions_deg = np.rad2deg(self._reference_joint_positions)

        urdf_path = self.get_parameter('urdf_path').value or find_dg5f_urdf()
        self._model = DG5FHandModel(urdf_path)
        self._samples = []
        self._joint_samples = []
        self._capture_active = False
        self._last_valid_hand = None
        self._last_valid_joint_sample = None

        self.create_subscription(PoseArray, '/manus/hand_landmarks', self._landmark_callback, 10)
        self.create_subscription(JointState, '/manus/finger_joints', self._joint_callback, 10)
        traj_topic = '/joint_trajectory_controller/joint_trajectory' if self._dummy_mode else '/dg5f_right/dg5f_right_controller/joint_trajectory'
        self._traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)

    def _landmark_callback(self, msg: PoseArray) -> None:
        try:
            hand = decode_manus_landmarks_msg(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return

        if not hand.is_finite() or hand.is_all_zero():
            return

        self._last_valid_hand = hand
        if self._capture_active:
            self._samples.append(hand)

    def _joint_callback(self, msg: JointState) -> None:
        try:
            joint_sample = decode_manus_joint_state_msg(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return

        if not joint_sample.is_finite():
            return

        self._last_valid_joint_sample = joint_sample
        if self._capture_active:
            self._joint_samples.append(joint_sample.positions_rad.copy())

    def _average_captured_hand(self):
        if not self._samples:
            raise RuntimeError('No valid Manus hand samples captured')

        accumulator = None
        for hand in self._samples:
            flat = []
            for finger in ('thumb', 'index', 'middle', 'ring', 'pinky'):
                finger_data = hand.fingers[finger]
                flat.extend([
                    finger_data.metacarpal, finger_data.proximal, finger_data.intermediate, finger_data.distal, finger_data.tip
                ])
            flat_arr = np.asarray(flat, dtype=float)
            accumulator = flat_arr if accumulator is None else accumulator + flat_arr

        averaged = accumulator / float(len(self._samples))
        from geometry_msgs.msg import Pose
        msg = PoseArray()
        msg.header.frame_id = 'manus_wrist_local'
        for point in averaged:
            pose = Pose()
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = float(point[2])
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        return decode_manus_landmarks_msg(msg)

    def _average_captured_joint_positions(self) -> np.ndarray:
        if not self._joint_samples:
            raise RuntimeError('No valid Manus joint samples captured')
        return np.mean(np.asarray(self._joint_samples, dtype=float), axis=0)

    def _capture_average(
        self,
        prompt: str | None = None,
        capture_message: str | None = None,
        capture_joint_positions: bool = False,
    ) -> object:
        if prompt:
            input(prompt)

        self.get_logger().info(
            capture_message or f'Capturing {self._sample_count} valid frames. Keep the glove steady until capture completes.')
        self._samples = []
        self._joint_samples = []
        self._capture_active = True
        while len(self._samples) < self._sample_count or (capture_joint_positions and len(self._joint_samples) < self._sample_count):
            rclpy.spin_once(self, timeout_sec=0.05)
        self._capture_active = False
        averaged_hand = self._average_captured_hand()
        averaged_joint_positions = self._average_captured_joint_positions() if capture_joint_positions else None
        return averaged_hand, averaged_joint_positions

    def _prepare_robot_pose(self, joint_positions_deg, pose_name: str, description: str) -> None:
        if self._dummy_mode:
            self.get_logger().info(
                f'Dummy mode enabled; not commanding the robot for "{pose_name}". '
                f'Use the glove pose directly: {description}')
            return

        self.get_logger().info(
            f'Commanding "{pose_name}" and holding for {self._settle_time_sec:.2f} s so you can mirror it with the glove.')
        self._hold_pose(joint_positions_deg, self._settle_time_sec)

    def _publish_pose_deg(self, joint_positions_deg) -> None:
        traj = JointTrajectory()
        traj.joint_names = list(self._model.joint_names)
        point = JointTrajectoryPoint()
        point.positions = np.deg2rad(np.asarray(joint_positions_deg, dtype=float)).tolist()
        point.time_from_start = Duration(sec=1, nanosec=0)
        traj.points.append(point)
        self._traj_pub.publish(traj)

    def _hold_pose(self, joint_positions_deg, duration_sec: float) -> None:
        end_time = time.monotonic() + duration_sec
        next_pub = 0.0
        while time.monotonic() < end_time:
            now = time.monotonic()
            if now >= next_pub:
                self._publish_pose_deg(joint_positions_deg)
                next_pub = now + self._command_period_sec
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self) -> None:
        if self._mode == 'stage1':
            self.get_logger().info(
                'Stage-1 calibration ready. The robot will move to the reference pose from '
                '`reference_joint_positions_deg`, then wait for you to mirror that pose with the Manus glove. '
                'For the most stable axis fit, use an open or clearly spread reference pose.')
            self._prepare_robot_pose(
                self._reference_joint_positions_deg,
                pose_name='stage1_reference',
                description='Reference pose used to fit the Manus-to-DG5F frame mapping.',
            )
            reference_hand, _ = self._capture_average(
                prompt=(
                    '\nStage-1 reference pose\n'
                    'Mirror the robot hand with the Manus glove, then press Enter to capture the reference pose...'
                ),
                capture_message=(
                    f'Capturing {self._sample_count} valid stage-1 frames from /manus/hand_landmarks. '
                    'Hold the glove steady until capture completes.'
                ),
            )
            calibration = compute_stage1_calibration(
                reference_hand,
                self._model,
                reference_joint_positions=self._reference_joint_positions,
            )
            save_calibration(calibration, self._output_path)
            self.get_logger().info(
                f'Saved stage-1 calibration to {self._output_path}: '
                f'axes={calibration["manus_to_dg5f_axes"]}, '
                f'signs={calibration["manus_to_dg5f_signs"]}, '
                f'hand_scale={calibration["hand_scale"]:.4f}, '
                f'palm_offset={np.round(calibration["palm_offset"], 5).tolist()}')
            return

        if self._mode != 'stage2':
            raise ValueError("mode must be 'stage1' or 'stage2'")

        calibration = load_calibration(self._calibration_file)
        captures = []
        self.get_logger().info(f'Stage-2 calibration ready. Loading seed calibration from {self._calibration_file}.')
        for pose in CALIBRATION_POSES:
            self._prepare_robot_pose(pose['joint_positions_deg'], pose_name=pose['name'], description=pose['description'])
            averaged_hand, averaged_joint_positions = self._capture_average(
                prompt=(
                    f'\nPose "{pose["name"]}": {pose["description"]}\n'
                    'Match the glove to the robot pose, then press Enter to capture...'
                ),
                capture_message=(
                    f'Capturing {self._sample_count} valid frames for "{pose["name"]}". '
                    'Hold the glove steady until capture completes.'
                ),
                capture_joint_positions=True,
            )
            robot_joint_positions = np.deg2rad(np.asarray(pose['joint_positions_deg'], dtype=float))
            robot_keypoints = self._model.get_virtual_keypoints(robot_joint_positions)
            captures.append(CalibrationPoseCapture(
                name=pose['name'],
                description=pose['description'],
                hand=averaged_hand,
                robot_keypoints=robot_keypoints,
                robot_joint_positions=robot_joint_positions,
                manus_joint_positions=averaged_joint_positions,
            ))
            self.get_logger().info(f'Captured {self._sample_count} frames for "{pose["name"]}".')

        updated = fit_stage2_calibration(captures, calibration)
        save_calibration(updated, self._output_path)
        self.get_logger().info(
            f'Saved stage-2 calibration to {self._output_path}: '
            f'workspace_axis_scale={np.round(updated["workspace_axis_scale"], 4).tolist()}, '
            f'stage2_rmse={updated["stage2_rmse"]:.6f}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyvectorCalibrationNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
