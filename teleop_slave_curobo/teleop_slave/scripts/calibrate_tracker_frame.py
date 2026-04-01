#!/usr/bin/env python3
"""Automated tracker-to-robot calibration using predefined poses.

Mounts the tracker rigidly on the FR5 end-effector, jogs the robot through
13 predefined poses (6 translational + 7 rotational), records paired
(robot, tracker) positions and quaternions at each pose, and computes:

- **Position axis mapping**: ``tracker_to_robot_axes`` and
  ``tracker_to_robot_signs`` (signed permutation from tracker XYZ to robot XYZ).
- **Orientation basis**: ``tracker_to_robot_rpy_deg`` (rotation that maps
  tracker rotation axes to robot rotation axes, via SVD / Wahba problem).

The results go into ``fr5_tracker_teleop.yaml``.

Prerequisites (each in a separate terminal):
    ros2 run teleop_slave fairino_lowlevel_controller_node \\
        --ros-args --params-file teleop_slave/config/fr5_tracker_teleop.yaml \\
        -p robot_ip:=192.168.58.2
    ros2 run teleop_slave master_bridge_node

Do NOT run fairino_slave_node during calibration.

Usage:
    ros2 run teleop_slave calibrate_tracker_frame.py
    ros2 run teleop_slave calibrate_tracker_frame.py --ros-args -p calibration_angle_deg:=25.0
    ros2 run teleop_slave calibrate_tracker_frame.py --ros-args -p calibration_step_m:=0.08
"""

from __future__ import annotations

import itertools
import os
import signal
import sys
import threading
import time

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool

# ---------------------------------------------------------------------------
# Quaternion helpers  (all quaternions use [x, y, z, w] layout)
# ---------------------------------------------------------------------------


def _normalize_quat(q: np.ndarray) -> np.ndarray:
    q = q / np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return q


def _quat_inverse(q: np.ndarray) -> np.ndarray:
    return _normalize_quat(np.array([-q[0], -q[1], -q[2], q[3]]))


def _quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return _normalize_quat(np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]))


def _quat_to_axis_angle(q: np.ndarray):
    q = _normalize_quat(q)
    sin_half = np.linalg.norm(q[:3])
    if sin_half < 1e-8:
        return np.array([0.0, 0.0, 1.0]), 0.0
    axis = q[:3] / sin_half
    angle = 2.0 * np.arctan2(sin_half, q[3])
    return axis, angle


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    q = _normalize_quat(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _matrix_to_rpy(R: np.ndarray) -> np.ndarray:
    """Intrinsic XYZ RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return np.array([roll, pitch, yaw])


def _rpy_to_quat(rpy_rad: np.ndarray) -> np.ndarray:
    cr = np.cos(rpy_rad[0] / 2); sr = np.sin(rpy_rad[0] / 2)
    cp = np.cos(rpy_rad[1] / 2); sp = np.sin(rpy_rad[1] / 2)
    cy = np.cos(rpy_rad[2] / 2); sy = np.sin(rpy_rad[2] / 2)
    return _normalize_quat(np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]))


def _average_quaternions(quats: list[np.ndarray]) -> np.ndarray:
    M = np.zeros((4, 4))
    for q in quats:
        q = q.reshape(4, 1)
        M += q @ q.T
    eigvals, eigvecs = np.linalg.eigh(M)
    return _normalize_quat(eigvecs[:, -1])


def _nearest_rotation(M: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(M)
    det = np.linalg.det(U @ Vt)
    D = np.diag([1.0, 1.0, det])
    return U @ D @ Vt


def _quat_angular_distance_deg(a: np.ndarray, b: np.ndarray) -> float:
    delta = _quat_multiply(_quat_inverse(a), b)
    _, angle = _quat_to_axis_angle(delta)
    return np.degrees(angle)


# ---------------------------------------------------------------------------
# Predefined calibration poses
# ---------------------------------------------------------------------------

_ORIENT_POSE_LABELS = [
    "Reference (initial)",
    "Roll +{0} deg",
    "Roll -{0} deg",
    "Pitch +{0} deg",
    "Pitch -{0} deg",
    "Yaw +{0} deg",
    "Yaw -{0} deg",
]


def _build_orientation_deltas(angle_deg: float) -> list[tuple[str, np.ndarray]]:
    """Return list of (label, quaternion_delta) for rotational calibration poses."""
    a = angle_deg
    deltas = [
        (0.0, 0.0, 0.0),
        (a, 0.0, 0.0),
        (-a, 0.0, 0.0),
        (0.0, a, 0.0),
        (0.0, -a, 0.0),
        (0.0, 0.0, a),
        (0.0, 0.0, -a),
    ]
    result = []
    for label_fmt, (r, p, y) in zip(_ORIENT_POSE_LABELS, deltas):
        label = label_fmt.format(a)
        q_delta = _rpy_to_quat(np.radians(np.array([r, p, y])))
        result.append((label, q_delta))
    return result


_TRANS_POSE_LABELS = [
    "+X ({0:.0f} mm)",
    "-X ({0:.0f} mm)",
    "+Y ({0:.0f} mm)",
    "-Y ({0:.0f} mm)",
    "+Z ({0:.0f} mm)",
    "-Z ({0:.0f} mm)",
]


def _build_translational_deltas(step_m: float) -> list[tuple[str, np.ndarray]]:
    """Return list of (label, xyz_delta_meters) for translational calibration poses."""
    s = step_m
    deltas = [
        (s,  0.0, 0.0),
        (-s, 0.0, 0.0),
        (0.0, s,  0.0),
        (0.0, -s, 0.0),
        (0.0, 0.0, s),
        (0.0, 0.0, -s),
    ]
    result = []
    for label_fmt, (dx, dy, dz) in zip(_TRANS_POSE_LABELS, deltas):
        label = label_fmt.format(step_m * 1000)
        result.append((label, np.array([dx, dy, dz])))
    return result


# ---------------------------------------------------------------------------
# SVD Wahba solver (orientation)
# ---------------------------------------------------------------------------


def _solve_basis_svd(
    pose_pairs: list[tuple[np.ndarray, np.ndarray]],
    min_angle_deg: float = 10.0,
) -> tuple[np.ndarray, dict]:
    """Compute basis rotation B from paired (q_robot, q_tracker) captures.

    Returns (B_matrix_3x3, diagnostics_dict).
    """
    n = len(pose_pairs)
    robot_axes = []
    tracker_axes = []
    angle_errors = []

    for i in range(n):
        for j in range(i + 1, n):
            q_r_i, q_t_i = pose_pairs[i]
            q_r_j, q_t_j = pose_pairs[j]

            delta_robot = _quat_multiply(_quat_inverse(q_r_i), q_r_j)
            delta_tracker = _quat_multiply(_quat_inverse(q_t_i), q_t_j)

            axis_r, angle_r = _quat_to_axis_angle(delta_robot)
            axis_t, angle_t = _quat_to_axis_angle(delta_tracker)

            angle_r_deg = np.degrees(angle_r)
            angle_t_deg = np.degrees(angle_t)

            # Skip pairs with too little rotation
            if angle_r_deg < min_angle_deg or angle_t_deg < min_angle_deg:
                continue

            robot_axes.append(axis_r)
            tracker_axes.append(axis_t)
            angle_errors.append(abs(angle_r_deg - angle_t_deg))

    if len(robot_axes) < 3:
        raise RuntimeError(
            f"Only {len(robot_axes)} valid axis pairs (need >= 3). "
            "Try more diverse orientations or a larger calibration angle."
        )

    # Build cross-covariance H and solve Wahba problem
    robot_axes_arr = np.array(robot_axes)    # (K, 3)
    tracker_axes_arr = np.array(tracker_axes)  # (K, 3)

    H = robot_axes_arr.T @ tracker_axes_arr  # (3, 3)
    B = _nearest_rotation(H)

    # Compute residuals: B should map tracker axes to robot axes
    mapped = (B @ tracker_axes_arr.T).T
    residuals_deg = []
    for r_ax, m_ax in zip(robot_axes_arr, mapped):
        dot = np.clip(np.dot(r_ax, m_ax), -1.0, 1.0)
        residuals_deg.append(np.degrees(np.arccos(dot)))

    sv = np.linalg.svd(H, compute_uv=False)

    diagnostics = {
        "num_pairs": len(robot_axes),
        "mean_residual_deg": float(np.mean(residuals_deg)),
        "max_residual_deg": float(np.max(residuals_deg)),
        "mean_angle_error_deg": float(np.mean(angle_errors)),
        "max_angle_error_deg": float(np.max(angle_errors)),
        "singular_values": sv.tolist(),
        "min_sv": float(sv.min()),
    }

    return B, diagnostics


# ---------------------------------------------------------------------------
# Signed-permutation solver (position axis mapping)
# ---------------------------------------------------------------------------


def _solve_position_mapping(
    robot_deltas: np.ndarray,
    tracker_deltas: np.ndarray,
) -> tuple[list[int], list[float], dict]:
    """Find the signed permutation mapping tracker axes to robot axes.

    The mapping matches ApplyAxisMapping semantics:
        robot_axis[i] = signs[i] * tracker_axis[axes[i]]

    Returns (axes, signs, diagnostics).
    """
    # Build cross-covariance matrix: H = robot_deltas.T @ tracker_deltas
    H = robot_deltas.T @ tracker_deltas  # (3, 3)

    # Brute-force search over all 48 signed permutations
    best_score = -np.inf
    best_perm = [0, 1, 2]
    best_signs = [1.0, 1.0, 1.0]

    for perm in itertools.permutations(range(3)):
        for sign_bits in range(8):
            signs = [1.0 if (sign_bits >> i) & 1 == 0 else -1.0 for i in range(3)]
            # Score = sum of signs[i] * H[i, perm[i]]
            score = sum(signs[i] * H[i, perm[i]] for i in range(3))
            if score > best_score:
                best_score = score
                best_perm = list(perm)
                best_signs = list(signs)

    # Compute residuals: apply mapping and compare to robot deltas
    mapped = np.zeros_like(tracker_deltas)
    for row in range(len(tracker_deltas)):
        for i in range(3):
            mapped[row, i] = best_signs[i] * tracker_deltas[row, best_perm[i]]

    residuals = np.linalg.norm(robot_deltas - mapped, axis=1)
    norms = np.linalg.norm(robot_deltas, axis=1)
    relative_errors = residuals / np.maximum(norms, 1e-6)

    diagnostics = {
        "score": float(best_score),
        "mean_residual_m": float(np.mean(residuals)),
        "max_residual_m": float(np.max(residuals)),
        "mean_relative_error": float(np.mean(relative_errors)),
    }

    return best_perm, best_signs, diagnostics


def _describe_axis_mapping(axes: list[int], signs: list[float]) -> str:
    """Human-readable description of the signed permutation."""
    axis_names = ["X", "Y", "Z"]
    parts = []
    for i in range(3):
        sign_str = "-" if signs[i] < 0 else ""
        parts.append(f"robot_{axis_names[i]} = {sign_str}tracker_{axis_names[axes[i]]}")
    return ", ".join(parts)


# ---------------------------------------------------------------------------
# Main calibration node
# ---------------------------------------------------------------------------

_DEFAULT_OUTPUT_PATH = os.path.expanduser(
    "~/.ros/tracker_calibration.yaml"
)

_SETTLE_ORIENT_DEG = 1.0
_SETTLE_POS_M = 0.002
_SETTLE_DURATION_SEC = 0.3
_SAMPLE_COUNT = 30
_POSE_TIMEOUT_SEC = 30.0
_PUBLISH_HZ = 50.0


class TrackerFrameCalibrator(Node):
    def __init__(self):
        super().__init__("tracker_frame_calibrator")
        self.declare_parameter("calibration_angle_deg", 20.0)
        self.declare_parameter("calibration_step_m", 0.05)
        self.declare_parameter("output_path", _DEFAULT_OUTPUT_PATH)

        self._angle_deg: float = self.get_parameter("calibration_angle_deg").value
        self._step_m: float = self.get_parameter("calibration_step_m").value
        self._output_path: str = self.get_parameter("output_path").value

        self._lock = threading.Lock()
        self._latest_robot_pose: PoseStamped | None = None
        self._latest_tracker_pose: PoseStamped | None = None
        self._robot_received = threading.Event()
        self._tracker_received = threading.Event()
        self._shutdown_flag = False

        self._robot_sub = self.create_subscription(
            PoseStamped, "/robot_pose", self._robot_cb, 10
        )
        self._tracker_sub = self.create_subscription(
            PoseStamped, "/manus/wrist_pose", self._tracker_cb, 10
        )
        self._pose_pub = self.create_publisher(
            PoseStamped, "/fr5/pose_target", 10
        )
        self._stream_client = self.create_client(
            SetBool, "/enable_streaming"
        )

        self.get_logger().info(
            f"Tracker calibration started "
            f"(angle={self._angle_deg} deg, step={self._step_m * 1000:.0f} mm)."
        )
        self.get_logger().info(
            "Waiting for /robot_pose and /manus/wrist_pose ..."
        )

        self._calib_thread = threading.Thread(
            target=self._run_calibration, daemon=True
        )
        self._calib_thread.start()

    # ---- ROS callbacks ----

    def _robot_cb(self, msg: PoseStamped):
        with self._lock:
            self._latest_robot_pose = msg
        self._robot_received.set()

    def _tracker_cb(self, msg: PoseStamped):
        with self._lock:
            self._latest_tracker_pose = msg
        self._tracker_received.set()

    # ---- Helpers ----

    def _get_robot_quat(self) -> np.ndarray:
        with self._lock:
            o = self._latest_robot_pose.pose.orientation
        return _normalize_quat(np.array([o.x, o.y, o.z, o.w]))

    def _get_robot_position(self) -> np.ndarray:
        with self._lock:
            p = self._latest_robot_pose.pose.position
        return np.array([p.x, p.y, p.z])

    def _get_tracker_quat(self) -> np.ndarray:
        with self._lock:
            o = self._latest_tracker_pose.pose.orientation
        return _normalize_quat(np.array([o.x, o.y, o.z, o.w]))

    def _get_tracker_position(self) -> np.ndarray:
        with self._lock:
            p = self._latest_tracker_pose.pose.position
        return np.array([p.x, p.y, p.z])

    def _call_streaming(self, enable: bool) -> bool:
        if not self._stream_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/enable_streaming service not available.")
            return False
        req = SetBool.Request()
        req.data = enable
        future = self._stream_client.call_async(req)
        # Spin until future completes
        timeout = time.time() + 5.0
        while not future.done() and time.time() < timeout:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().error("Streaming service call timed out.")
            return False
        resp = future.result()
        if resp is None or not resp.success:
            msg = resp.message if resp else "no response"
            self.get_logger().error(f"Streaming service failed: {msg}")
            return False
        return True

    def _publish_target(self, position: np.ndarray, orientation_q: np.ndarray):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.x = float(orientation_q[0])
        msg.pose.orientation.y = float(orientation_q[1])
        msg.pose.orientation.z = float(orientation_q[2])
        msg.pose.orientation.w = float(orientation_q[3])
        self._pose_pub.publish(msg)

    def _wait_for_settle(
        self,
        target_q: np.ndarray,
        target_pos: np.ndarray,
        timeout_sec: float = _POSE_TIMEOUT_SEC,
    ) -> bool:
        """Publish target at 50 Hz until robot settles or timeout."""
        period = 1.0 / _PUBLISH_HZ
        deadline = time.time() + timeout_sec
        settle_start: float | None = None

        while time.time() < deadline and not self._shutdown_flag:
            self._publish_target(target_pos, target_q)

            robot_q = self._get_robot_quat()
            robot_pos = self._get_robot_position()
            orient_err = _quat_angular_distance_deg(robot_q, target_q)
            pos_err = np.linalg.norm(robot_pos - target_pos)

            if orient_err < _SETTLE_ORIENT_DEG and pos_err < _SETTLE_POS_M:
                if settle_start is None:
                    settle_start = time.time()
                elif time.time() - settle_start >= _SETTLE_DURATION_SEC:
                    return True
            else:
                settle_start = None

            time.sleep(period)

        return False

    def _collect_samples(
        self, target_pos: np.ndarray, target_q: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Collect averaged (robot_q, tracker_q, robot_pos, tracker_pos)."""
        robot_q_samples = []
        tracker_q_samples = []
        robot_pos_samples = []
        tracker_pos_samples = []
        period = 1.0 / _PUBLISH_HZ

        for _ in range(_SAMPLE_COUNT):
            self._publish_target(target_pos, target_q)
            robot_q_samples.append(self._get_robot_quat())
            tracker_q_samples.append(self._get_tracker_quat())
            robot_pos_samples.append(self._get_robot_position())
            tracker_pos_samples.append(self._get_tracker_position())
            time.sleep(period)

        return (
            _average_quaternions(robot_q_samples),
            _average_quaternions(tracker_q_samples),
            np.mean(robot_pos_samples, axis=0),
            np.mean(tracker_pos_samples, axis=0),
        )

    def _safe_shutdown_streaming(self):
        try:
            self._call_streaming(False)
        except Exception:
            pass

    # ---- Main calibration procedure ----

    def _run_calibration(self):
        try:
            self._run_calibration_inner()
        except Exception as e:
            self.get_logger().error(f"Calibration failed: {e}")
            self._safe_shutdown_streaming()
        finally:
            self._shutdown_flag = True
            if rclpy.ok():
                rclpy.shutdown()

    def _run_calibration_inner(self):
        # Wait for data
        if not self._robot_received.wait(timeout=10.0):
            print("\nERROR: No data on /robot_pose after 10 seconds.")
            print("Is fairino_lowlevel_controller_node running?")
            return
        if not self._tracker_received.wait(timeout=10.0):
            print("\nERROR: No data on /manus/wrist_pose after 10 seconds.")
            print("Is master_bridge_node running?")
            return

        print("\nData received on both topics.")

        # Read initial pose
        initial_pos = self._get_robot_position()
        initial_q = self._get_robot_quat()
        print(f"  Initial robot position: [{initial_pos[0]:.4f}, {initial_pos[1]:.4f}, {initial_pos[2]:.4f}]")

        # Build both sets of calibration poses
        trans_poses = _build_translational_deltas(self._step_m)
        orient_poses = _build_orientation_deltas(self._angle_deg)
        # 1 reference + 6 translational + 6 rotational (reference shared)
        total_poses = 1 + len(trans_poses) + len(orient_poses) - 1

        # Safety warning
        print(f"\n{'=' * 60}")
        print(f"  WARNING: The robot WILL MOVE through {total_poses} poses.")
        print("  Ensure the workspace is clear and e-stop is accessible.")
        print(f"  Translational step: +/- {self._step_m * 1000:.0f} mm per axis.")
        print(f"  Rotational angle:   +/- {self._angle_deg} degrees per axis.")
        print(f"{'=' * 60}")
        print("\nPress ENTER to start calibration (or Ctrl+C to abort).")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            print("\nAborted.")
            return

        # Enable streaming
        print("Enabling robot streaming...")
        if not self._call_streaming(True):
            print("ERROR: Failed to enable streaming. Aborting.")
            return

        time.sleep(0.2)  # Let streaming stabilize

        pose_idx = 0
        orient_pairs: list[tuple[np.ndarray, np.ndarray]] = []
        robot_positions: list[np.ndarray] = []
        tracker_positions: list[np.ndarray] = []

        try:
            # --- Phase 1: Reference pose (shared) ---
            pose_idx += 1
            print(f"\n  Pose {pose_idx}/{total_poses}: Reference (initial)")
            print(f"    Collecting {_SAMPLE_COUNT} samples...", end="", flush=True)

            ref_rq, ref_tq, ref_rp, ref_tp = self._collect_samples(
                initial_pos, initial_q
            )
            orient_pairs.append((ref_rq, ref_tq))
            print(" done.")

            # --- Phase 2: Translational poses ---
            for label, xyz_delta in trans_poses:
                if self._shutdown_flag:
                    break

                pose_idx += 1
                target_pos = initial_pos + xyz_delta
                print(f"\n  Pose {pose_idx}/{total_poses}: Translation {label}")
                print(f"    Moving robot...", end="", flush=True)

                if not self._wait_for_settle(initial_q, target_pos):
                    print(f" TIMEOUT (>{_POSE_TIMEOUT_SEC}s)")
                    print("    ERROR: Robot could not reach this pose.")
                    print("    Try a more central starting position or smaller calibration_step_m.")
                    self._safe_shutdown_streaming()
                    return

                print(" settled.", flush=True)
                print(f"    Collecting {_SAMPLE_COUNT} samples...", end="", flush=True)

                _, _, rp, tp = self._collect_samples(target_pos, initial_q)
                robot_positions.append(rp)
                tracker_positions.append(tp)
                print(" done.")

            # Return to initial before rotational phase
            print(f"\n  Returning to initial pose...", end="", flush=True)
            self._wait_for_settle(initial_q, initial_pos, timeout_sec=15.0)
            print(" done.")

            # --- Phase 3: Rotational poses (skip reference, already captured) ---
            for label, q_delta in orient_poses[1:]:
                if self._shutdown_flag:
                    break

                pose_idx += 1
                target_q = _quat_multiply(initial_q, q_delta)
                print(f"\n  Pose {pose_idx}/{total_poses}: Orientation {label}")
                print(f"    Moving robot...", end="", flush=True)

                if not self._wait_for_settle(target_q, initial_pos):
                    print(f" TIMEOUT (>{_POSE_TIMEOUT_SEC}s)")
                    print("    ERROR: Robot could not reach this pose.")
                    print("    Try a different starting position or smaller calibration_angle_deg.")
                    self._safe_shutdown_streaming()
                    return

                print(" settled.", flush=True)
                print(f"    Collecting {_SAMPLE_COUNT} samples...", end="", flush=True)

                rq, tq, _, _ = self._collect_samples(initial_pos, target_q)
                orient_pairs.append((rq, tq))
                print(" done.")

            # Return to initial pose
            print(f"\n  Returning to initial pose...", end="", flush=True)
            self._wait_for_settle(initial_q, initial_pos, timeout_sec=15.0)
            print(" done.")

        finally:
            # Always disable streaming
            print("  Disabling streaming...")
            self._safe_shutdown_streaming()

        # --- Solve position mapping ---
        if len(robot_positions) < 3:
            print(f"\nERROR: Only captured {len(robot_positions)} translational poses (need >= 3).")
            return

        robot_deltas = np.array([rp - ref_rp for rp in robot_positions])
        tracker_deltas = np.array([tp - ref_tp for tp in tracker_positions])

        print(f"\nComputing position axis mapping from {len(robot_positions)} translational poses...")
        pos_axes, pos_signs, pos_diag = _solve_position_mapping(
            robot_deltas, tracker_deltas
        )

        # --- Solve orientation basis ---
        if len(orient_pairs) < 3:
            print(f"\nERROR: Only captured {len(orient_pairs)} orientation poses (need >= 3).")
            return

        print(f"Computing orientation basis from {len(orient_pairs)} rotational poses...")
        try:
            B, orient_diag = _solve_basis_svd(orient_pairs)
        except RuntimeError as e:
            print(f"ERROR: {e}")
            return

        rpy_rad = _matrix_to_rpy(B)
        rpy_deg = np.degrees(rpy_rad)
        basis_quat = _rpy_to_quat(rpy_rad)

        # --- Print results ---
        print(f"\n{'=' * 60}")
        print("  TRACKER CALIBRATION RESULT")
        print(f"{'=' * 60}")

        # Position mapping
        print()
        print(f"  Position axis mapping:")
        print(f"    tracker_to_robot_axes:  {pos_axes}")
        print(f"    tracker_to_robot_signs: [{pos_signs[0]:.1f}, {pos_signs[1]:.1f}, {pos_signs[2]:.1f}]")
        print(f"    Meaning: {_describe_axis_mapping(pos_axes, pos_signs)}")
        print(f"    Mean residual: {pos_diag['mean_residual_m']:.4f} m")
        print(f"    Max residual:  {pos_diag['max_residual_m']:.4f} m")
        if pos_diag["mean_relative_error"] > 0.3:
            print(f"    Mean relative error: {pos_diag['mean_relative_error']:.2f}"
                  "  (WARNING: high — tracker position data may be noisy)")
        else:
            print(f"    Mean relative error: {pos_diag['mean_relative_error']:.2f}  (OK)")

        # Orientation basis
        print()
        print(f"  Orientation basis:")
        print(f"    tracker_to_robot_rpy_deg: [{rpy_deg[0]:.1f}, {rpy_deg[1]:.1f}, {rpy_deg[2]:.1f}]")
        print(f"    Axis pairs used:       {orient_diag['num_pairs']}")
        print(f"    Mean residual:         {orient_diag['mean_residual_deg']:.2f} deg")
        print(f"    Max residual:          {orient_diag['max_residual_deg']:.2f} deg")
        print(f"    Mean angle mismatch:   {orient_diag['mean_angle_error_deg']:.2f} deg")
        print(f"    Max angle mismatch:    {orient_diag['max_angle_error_deg']:.2f} deg")
        print(f"    Singular values:       [{orient_diag['singular_values'][0]:.3f}, {orient_diag['singular_values'][1]:.3f}, {orient_diag['singular_values'][2]:.3f}]")
        print(f"    Min singular value:    {orient_diag['min_sv']:.3f}", end="")
        if orient_diag["min_sv"] < 0.3:
            print("  (WARNING: low — poses may not be diverse enough)")
        else:
            print("  (OK)")

        # Paste block
        print()
        print(f"  Paste into teleop_slave/config/fr5_tracker_teleop.yaml:")
        print(f"    tracker_to_robot_axes: {pos_axes}")
        print(f"    tracker_to_robot_signs: [{pos_signs[0]:.1f}, {pos_signs[1]:.1f}, {pos_signs[2]:.1f}]")
        print(f"    tracker_to_robot_rpy_deg: [{rpy_deg[0]:.1f}, {rpy_deg[1]:.1f}, {rpy_deg[2]:.1f}]")

        print()
        print(f"  Basis quaternion [x,y,z,w]: "
              f"[{basis_quat[0]:.6f}, {basis_quat[1]:.6f}, "
              f"{basis_quat[2]:.6f}, {basis_quat[3]:.6f}]")
        print(f"{'=' * 60}")

        # Save
        print(f"\nPress ENTER to save to {self._output_path} (or 'q' to quit without saving).")
        try:
            response = input().strip().lower()
        except (EOFError, KeyboardInterrupt):
            response = "q"

        if response == "q":
            print("Calibration not saved.")
            return

        self._save_result(
            rpy_deg, basis_quat, orient_diag, orient_pairs,
            pos_axes, pos_signs, pos_diag,
        )

    def _save_result(
        self,
        rpy_deg: np.ndarray,
        basis_quat: np.ndarray,
        orient_diagnostics: dict,
        orient_pairs: list[tuple[np.ndarray, np.ndarray]],
        pos_axes: list[int],
        pos_signs: list[float],
        pos_diagnostics: dict,
    ):
        data = {
            "tracker_to_robot_axes": pos_axes,
            "tracker_to_robot_signs": [round(float(v), 1) for v in pos_signs],
            "tracker_to_robot_rpy_deg": [round(float(v), 1) for v in rpy_deg],
            "basis_quaternion_xyzw": [round(float(v), 6) for v in basis_quat],
            "calibration_angle_deg": float(self._angle_deg),
            "calibration_step_m": float(self._step_m),
            "num_orientation_poses": len(orient_pairs),
            "num_translational_poses": 6,
            "diagnostics": {
                "orientation": {
                    "num_axis_pairs": orient_diagnostics["num_pairs"],
                    "mean_residual_deg": round(orient_diagnostics["mean_residual_deg"], 2),
                    "max_residual_deg": round(orient_diagnostics["max_residual_deg"], 2),
                    "mean_angle_mismatch_deg": round(orient_diagnostics["mean_angle_error_deg"], 2),
                    "singular_values": [round(v, 3) for v in orient_diagnostics["singular_values"]],
                },
                "position": {
                    "mean_residual_m": round(pos_diagnostics["mean_residual_m"], 4),
                    "max_residual_m": round(pos_diagnostics["max_residual_m"], 4),
                    "mean_relative_error": round(pos_diagnostics["mean_relative_error"], 4),
                },
            },
        }

        os.makedirs(os.path.dirname(self._output_path), exist_ok=True)
        with open(self._output_path, "w") as f:
            yaml.dump(data, f, default_flow_style=False)

        print(f"\n  Calibration saved to {self._output_path}")
        print(f"  Copy these lines into fr5_tracker_teleop.yaml:")
        print(f"    tracker_to_robot_axes: {pos_axes}")
        print(f"    tracker_to_robot_signs: [{pos_signs[0]:.1f}, {pos_signs[1]:.1f}, {pos_signs[2]:.1f}]")
        print(f"    tracker_to_robot_rpy_deg: [{rpy_deg[0]:.1f}, {rpy_deg[1]:.1f}, {rpy_deg[2]:.1f}]")


def main(args=None):
    rclpy.init(args=args)
    node = TrackerFrameCalibrator()

    # Handle Ctrl+C gracefully
    def shutdown_handler(sig, frame):
        node._shutdown_flag = True
        node._safe_shutdown_streaming()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
