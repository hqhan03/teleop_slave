"""ROS 2 node for live DG-5F keyvector retargeting."""

from __future__ import annotations

from dataclasses import dataclass
import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseArray
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from .calibration import DEFAULT_CALIBRATION_PATH, load_calibration
from .dg5f_model import DEFAULT_REFERENCE_JOINTS_DEG, DG5FHandModel, PALM_FRAME_ID, find_dg5f_urdf
from .joint_command_filter import PerJointVelocityLimiter
from .keyvector_loss import KEYVECTOR_SPECS, compute_keyvector_endpoints
from .keyvector_solver import KeyvectorRetargetSolver
from .manus_landmarks import CANONICAL_MANUS_HAND_FRAME, ManusHandLandmarks, decode_manus_landmarks_msg
from .manus_joints import decode_manus_joint_state_msg, joint_prior_from_manus_sample


POSE_ARRAY_KEYPOINT_ORDER = ('mcp', 'pip', 'dip', 'tip')


@dataclass
class PendingHandSample:
    hand: ManusHandLandmarks
    received_time: object


class KeyvectorRetargetNode(Node):
    """Live ROS node for keyvector-based DG-5F retargeting."""

    def __init__(self):
        super().__init__('keyvector_retarget_node')

        self.declare_parameter('urdf_path', '')
        self.declare_parameter('calibration_file', DEFAULT_CALIBRATION_PATH)
        self.declare_parameter('dummy_mode', False)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('publish_time_from_start_sec', 0.0)
        self.declare_parameter('epsilon_contact_m', 0.015)
        self.declare_parameter('eta_thumb_close_m', 0.008)
        self.declare_parameter('eta_interfinger_sep_m', 0.018)
        self.declare_parameter('lambda_smooth', 0.0)
        self.declare_parameter('solver_max_nfev', 20)
        self.declare_parameter('solver_tol', 1e-4)
        self.declare_parameter('dropout_hold_sec', 0.2)
        self.declare_parameter('recovery_alpha', 0.15)
        self.declare_parameter('joint_prior_enabled', True)
        self.declare_parameter('joint_prior_timeout_sec', 0.10)
        self.declare_parameter('prior_seed_alpha', 0.7)
        self.declare_parameter('joint_prior_weight_flex', 0.05)
        self.declare_parameter('joint_prior_weight_spread', 0.03)
        self.declare_parameter('max_flex_velocity_deg_s', 480.0)
        self.declare_parameter('max_spread_velocity_deg_s', 300.0)
        self.declare_parameter('reference_joint_positions_deg', DEFAULT_REFERENCE_JOINTS_DEG.tolist())

        urdf_path = self.get_parameter('urdf_path').value or find_dg5f_urdf()
        calibration_file = self.get_parameter('calibration_file').value
        self._dummy_mode = bool(self.get_parameter('dummy_mode').value)
        self._control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self._min_solve_period_sec = 1.0 / max(self._control_rate_hz, 1.0)
        self._publish_time_from_start_sec = float(self.get_parameter('publish_time_from_start_sec').value)
        self._dropout_hold_sec = float(self.get_parameter('dropout_hold_sec').value)
        self._recovery_alpha = float(self.get_parameter('recovery_alpha').value)
        self._joint_prior_timeout_sec = float(self.get_parameter('joint_prior_timeout_sec').value)
        reference_joint_positions = np.deg2rad(np.asarray(self.get_parameter('reference_joint_positions_deg').value, dtype=float))

        self._model = DG5FHandModel(urdf_path)
        self._calibration = load_calibration(calibration_file)
        self._joint_prior_enabled = bool(self.get_parameter('joint_prior_enabled').value) and bool(
            self._calibration.get('joint_prior_enabled', True))
        if calibration_file and not self._calibration.get('stage2_success', False):
            self.get_logger().info(
                f'Loaded keyvector calibration from {calibration_file}. '
                'If the hand behavior looks wrong, run `calibrate_keyvector`.')

        self._solver = KeyvectorRetargetSolver(
            model=self._model,
            calibration=self._calibration,
            reference_joint_positions=reference_joint_positions,
            epsilon_contact_m=float(self.get_parameter('epsilon_contact_m').value),
            eta_thumb_close_m=float(self.get_parameter('eta_thumb_close_m').value),
            eta_interfinger_sep_m=float(self.get_parameter('eta_interfinger_sep_m').value),
            lambda_smooth=float(self.get_parameter('lambda_smooth').value),
            solver_max_nfev=int(self.get_parameter('solver_max_nfev').value),
            solver_tol=float(self.get_parameter('solver_tol').value),
            prior_seed_alpha=float(self.get_parameter('prior_seed_alpha').value),
            joint_prior_weight_flex=float(self.get_parameter('joint_prior_weight_flex').value),
            joint_prior_weight_spread=float(self.get_parameter('joint_prior_weight_spread').value),
        )
        self._command_limiter = PerJointVelocityLimiter(
            self._model.joint_names,
            max_flex_velocity_deg_s=float(self.get_parameter('max_flex_velocity_deg_s').value),
            max_spread_velocity_deg_s=float(self.get_parameter('max_spread_velocity_deg_s').value),
            initial_state=reference_joint_positions,
        )

        traj_topic = (
            '/joint_trajectory_controller/joint_trajectory'
            if self._dummy_mode else
            '/dg5f_right/dg5f_right_controller/joint_trajectory'
        )
        self._traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)
        self._human_keypoints_pub = self.create_publisher(PoseArray, 'keyvector/debug/human_keypoints', 10)
        self._robot_keypoints_pub = self.create_publisher(PoseArray, 'keyvector/debug/robot_keypoints', 10)
        self._human_vectors_pub = self.create_publisher(MarkerArray, 'keyvector/debug/human_keyvectors', 10)
        self._robot_vectors_pub = self.create_publisher(MarkerArray, 'keyvector/debug/robot_keyvectors', 10)
        self._cost_pub = self.create_publisher(Float64, 'keyvector/debug/solver_cost', 10)
        self._solve_time_pub = self.create_publisher(Float64, 'keyvector/debug/solver_time_ms', 10)
        self._success_pub = self.create_publisher(Bool, 'keyvector/debug/solver_success', 10)
        self._status_pub = self.create_publisher(String, 'keyvector/debug/solver_status', 10)
        self._q_prior_pub = self.create_publisher(JointState, 'keyvector/debug/q_prior', 10)
        self._q_solved_pub = self.create_publisher(JointState, 'keyvector/debug/q_solved', 10)
        self._q_commanded_pub = self.create_publisher(JointState, 'keyvector/debug/q_commanded', 10)
        self._landmark_age_pub = self.create_publisher(Float64, 'keyvector/debug/landmark_age_sec', 10)
        self._joint_age_pub = self.create_publisher(Float64, 'keyvector/debug/joint_prior_age_sec', 10)

        self._pending_sample: PendingHandSample | None = None
        self._last_landmark_time = None
        self._latest_joint_sample = None
        self._latest_joint_time = None
        self._last_solve_time = None
        self._last_command_time = None
        self._warned_frame = False
        self._warned_invalid = False

        self.create_subscription(PoseArray, '/manus/hand_landmarks', self._landmark_callback, 10)
        self.create_subscription(JointState, '/manus/finger_joints', self._joint_callback, 10)
        self.create_timer(self._min_solve_period_sec, self._control_callback)

        self.get_logger().info(
            f'Keyvector retargeter ready. Inputs=/manus/hand_landmarks ({CANONICAL_MANUS_HAND_FRAME}) '
            f'and /manus/finger_joints, output={traj_topic}')

    def _landmark_callback(self, msg: PoseArray) -> None:
        try:
            hand = decode_manus_landmarks_msg(msg)
        except ValueError as exc:
            if not self._warned_frame:
                self.get_logger().warn(str(exc))
                self._warned_frame = True
            return

        if not hand.is_finite():
            if not self._warned_invalid:
                self.get_logger().warn('Ignoring non-finite /manus/hand_landmarks sample.')
                self._warned_invalid = True
            return

        if hand.is_all_zero():
            return

        now = self.get_clock().now()
        self._pending_sample = PendingHandSample(hand=hand, received_time=now)
        self._last_landmark_time = now
        self._try_process_pending(now)

    def _joint_callback(self, msg: JointState) -> None:
        try:
            joint_sample = decode_manus_joint_state_msg(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return

        if not joint_sample.is_finite():
            return

        self._latest_joint_sample = joint_sample
        self._latest_joint_time = self.get_clock().now()

    def _control_callback(self) -> None:
        now = self.get_clock().now()
        if self._try_process_pending(now):
            return

        if self._pending_sample is not None:
            return

        if self._last_landmark_time is not None:
            age = (now - self._last_landmark_time).nanoseconds / 1e9
            if age <= self._dropout_hold_sec:
                current = self._command_limiter.current_state if self._command_limiter.current_state is not None else self._solver.q_prev
                self._publish_trajectory(current)
                self._publish_status(
                    True,
                    0.0,
                    0.0,
                    f'Holding last command for stale landmarks (age={age:.3f}s)',
                    mode='hold_last',
                    landmark_age_sec=age,
                    joint_prior_age_sec=self._joint_prior_age_sec(now),
                )
            else:
                base = self._command_limiter.current_state if self._command_limiter.current_state is not None else self._solver.q_prev
                desired = self._model.clip_to_limits((1.0 - self._recovery_alpha) * base + self._recovery_alpha * self._solver.reference_joint_positions)
                q_commanded = self._apply_command_filter(desired, now)
                self._publish_trajectory(q_commanded)
                self._publish_status(
                    True,
                    0.0,
                    0.0,
                    f'Easing toward reference pose after dropout (age={age:.3f}s)',
                    mode='recover_to_reference',
                    landmark_age_sec=age,
                    joint_prior_age_sec=self._joint_prior_age_sec(now),
                )

    def _publish_trajectory(self, joint_positions: np.ndarray) -> None:
        traj_msg = JointTrajectory()
        traj_msg.joint_names = list(self._model.joint_names)
        point = JointTrajectoryPoint()
        point.positions = np.asarray(joint_positions, dtype=float).tolist()
        point.time_from_start = Duration(
            sec=int(self._publish_time_from_start_sec),
            nanosec=int(round((self._publish_time_from_start_sec - int(self._publish_time_from_start_sec)) * 1e9)),
        )
        traj_msg.points.append(point)
        self._traj_pub.publish(traj_msg)

    def _publish_debug(
        self,
        result,
        q_commanded: np.ndarray,
        q_prior_debug: np.ndarray | None,
        landmark_age_sec: float,
        joint_prior_age_sec: float,
        mode: str,
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        self._human_keypoints_pub.publish(self._pose_array_from_keypoints(result.human_keypoints, stamp))
        self._robot_keypoints_pub.publish(self._pose_array_from_keypoints(result.robot_keypoints, stamp))
        self._human_vectors_pub.publish(self._marker_array_from_keypoints('human', result.human_keypoints, stamp))
        self._robot_vectors_pub.publish(self._marker_array_from_keypoints('robot', result.robot_keypoints, stamp))
        if q_prior_debug is not None:
            self._q_prior_pub.publish(self._joint_state_from_positions(q_prior_debug, stamp))
        self._q_solved_pub.publish(self._joint_state_from_positions(result.joint_positions, stamp))
        self._q_commanded_pub.publish(self._joint_state_from_positions(q_commanded, stamp))
        self._publish_status(
            result.success,
            result.cost,
            result.solve_time_ms,
            result.message,
            mode=mode,
            landmark_age_sec=landmark_age_sec,
            joint_prior_age_sec=joint_prior_age_sec,
        )

    def _publish_status(
        self,
        success: bool,
        cost: float,
        solve_time_ms: float,
        status: str,
        mode: str,
        landmark_age_sec: float,
        joint_prior_age_sec: float,
    ) -> None:
        self._success_pub.publish(Bool(data=bool(success)))
        self._cost_pub.publish(Float64(data=float(cost)))
        self._solve_time_pub.publish(Float64(data=float(solve_time_ms)))
        self._landmark_age_pub.publish(Float64(data=float(landmark_age_sec)))
        self._joint_age_pub.publish(Float64(data=float(joint_prior_age_sec)))
        self._status_pub.publish(String(data=f'{mode}: {status}'))

    def _pose_array_from_keypoints(self, keypoints: dict[str, dict[str, np.ndarray]], stamp) -> PoseArray:
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = PALM_FRAME_ID
        for finger in ('thumb', 'index', 'middle', 'ring', 'pinky'):
            for keypoint in POSE_ARRAY_KEYPOINT_ORDER:
                pose = Pose()
                point = np.asarray(keypoints[finger][keypoint], dtype=float)
                pose.position.x = float(point[0])
                pose.position.y = float(point[1])
                pose.position.z = float(point[2])
                pose.orientation.w = 1.0
                msg.poses.append(pose)
        return msg

    def _joint_state_from_positions(self, joint_positions: np.ndarray, stamp) -> JointState:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(self._model.joint_names)
        msg.position = np.asarray(joint_positions, dtype=float).tolist()
        return msg

    def _marker_array_from_keypoints(self, namespace: str, keypoints: dict[str, dict[str, np.ndarray]], stamp) -> MarkerArray:
        markers = MarkerArray()
        endpoints = compute_keyvector_endpoints(keypoints)
        for idx, spec in enumerate(KEYVECTOR_SPECS):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = PALM_FRAME_ID
            marker.ns = namespace
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            origin, target = endpoints[spec.name]
            marker.points = []
            p0 = Pose().position
            p0.x, p0.y, p0.z = float(origin[0]), float(origin[1]), float(origin[2])
            p1 = Pose().position
            p1.x, p1.y, p1.z = float(target[0]), float(target[1]), float(target[2])
            marker.points.append(p0)
            marker.points.append(p1)
            marker.scale.x = 0.003
            marker.scale.y = 0.006
            marker.scale.z = 0.008
            if spec.vector_set == 'self':
                marker.color.r, marker.color.g, marker.color.b = (0.2, 0.8, 0.2)
            elif spec.vector_set == 'S1':
                marker.color.r, marker.color.g, marker.color.b = (0.95, 0.65, 0.15)
            else:
                marker.color.r, marker.color.g, marker.color.b = (0.2, 0.55, 1.0)
            marker.color.a = 0.95 if namespace == 'robot' else 0.55
            markers.markers.append(marker)
        return markers

    def _joint_prior_age_sec(self, now) -> float:
        if self._latest_joint_time is None:
            return float('inf')
        return (now - self._latest_joint_time).nanoseconds / 1e9

    def _latest_joint_prior(self, now) -> tuple[np.ndarray | None, np.ndarray | None, float, bool]:
        age = self._joint_prior_age_sec(now)
        if self._latest_joint_sample is None:
            return None, None, age, False

        q_prior_debug = joint_prior_from_manus_sample(self._latest_joint_sample, self._calibration, model=self._model)
        fresh = self._joint_prior_enabled and age <= self._joint_prior_timeout_sec
        if not fresh:
            return None, q_prior_debug, age, False
        return q_prior_debug, q_prior_debug, age, True

    def _apply_command_filter(self, desired_joint_positions: np.ndarray, now) -> np.ndarray:
        dt_sec = None if self._last_command_time is None else (now - self._last_command_time).nanoseconds / 1e9
        q_commanded = self._command_limiter.apply(self._model.clip_to_limits(desired_joint_positions), dt_sec)
        self._last_command_time = now
        self._solver.reset(q_commanded)
        return q_commanded

    def _should_process_pending(self, now) -> bool:
        if self._pending_sample is None:
            return False
        if self._last_solve_time is None:
            return True
        return (now - self._last_solve_time).nanoseconds / 1e9 >= self._min_solve_period_sec

    def _try_process_pending(self, now) -> bool:
        if not self._should_process_pending(now):
            return False

        pending = self._pending_sample
        self._pending_sample = None
        self._last_solve_time = now

        q_prior_used, q_prior_debug, joint_age_sec, using_joint_prior = self._latest_joint_prior(now)
        result = self._solver.solve(
            pending.hand,
            q_prior=q_prior_used,
            prior_weights=self._solver.default_prior_weights if using_joint_prior else None,
        )
        q_commanded = self._apply_command_filter(result.joint_positions, now)
        landmark_age_sec = max((now - pending.received_time).nanoseconds / 1e9, 0.0)
        mode = 'hybrid' if using_joint_prior else 'landmarks_only_fallback'
        self._publish_trajectory(q_commanded)
        self._publish_debug(
            result,
            q_commanded=q_commanded,
            q_prior_debug=q_prior_debug,
            landmark_age_sec=landmark_age_sec,
            joint_prior_age_sec=joint_age_sec,
            mode=mode,
        )
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyvectorRetargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
