#!/usr/bin/env python3
"""
Arm pipeline diagnostic plot: raw tracker vs filtered pose vs actual joints.

Subscribes to:
  /manus/wrist_pose      — raw Vive tracker (50 Hz)
  /fr5/pose_target       — after Butterworth + orientation smoothing (50 Hz)
  /robot_joint_states    — actual executed joint positions (125 Hz)

Shows 4 live subplots:
  1. Position deltas: raw vs filtered  (noise attenuation by Butterworth)
  2. Orientation deltas: raw vs filtered (SLERP alpha effectiveness)
  3. Robot joint positions (6 DOF)
  4. Robot joint velocities (deg/s) — vibration signature

Usage:
  python3 teleop_slave/scripts/plot_arm_pipeline.py
"""

import math
import threading
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

BUFFER_50HZ = 250   # 5 s at 50 Hz
BUFFER_125HZ = 625  # 5 s at 125 Hz

JOINT_NAMES_SHORT = ["J1", "J2", "J3", "J4", "J5", "J6"]


def quat_to_rpy(w, x, y, z):
    """Quaternion (w,x,y,z) -> (roll, pitch, yaw) in degrees."""
    # roll (x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw (z-axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class ArmPipelineRecorder(Node):
    def __init__(self):
        super().__init__("arm_pipeline_recorder")

        self.lock = threading.Lock()
        self.t0 = None

        # --- Raw tracker ---
        self.raw_t = deque(maxlen=BUFFER_50HZ)
        self.raw_dxyz = deque(maxlen=BUFFER_50HZ)
        self.raw_drpy = deque(maxlen=BUFFER_50HZ)
        self._prev_raw_xyz = None
        self._prev_raw_rpy = None

        # --- Filtered pose target ---
        self.filt_t = deque(maxlen=BUFFER_50HZ)
        self.filt_dxyz = deque(maxlen=BUFFER_50HZ)
        self.filt_drpy = deque(maxlen=BUFFER_50HZ)
        self._prev_filt_xyz = None
        self._prev_filt_rpy = None

        # --- Robot joints ---
        self.joint_t = deque(maxlen=BUFFER_125HZ)
        self.joint_pos = deque(maxlen=BUFFER_125HZ)   # (6,) deg
        self.joint_vel = deque(maxlen=BUFFER_125HZ)    # (6,) deg/s
        self._prev_joint_pos = None
        self._prev_joint_t = None

        self.create_subscription(PoseStamped, "manus/wrist_pose", self._raw_cb, 10)
        self.create_subscription(PoseStamped, "fr5/pose_target", self._filt_cb, 10)
        self.create_subscription(JointState, "robot_joint_states", self._joint_cb, 10)

    def _sec(self, stamp):
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    # ---------- raw tracker ----------
    def _raw_cb(self, msg: PoseStamped):
        t = self._sec(msg.header.stamp)
        p = msg.pose.position
        o = msg.pose.orientation
        xyz = np.array([p.x, p.y, p.z])
        rpy = np.array(quat_to_rpy(o.w, o.x, o.y, o.z))

        with self.lock:
            self.raw_t.append(t)
            if self._prev_raw_xyz is not None:
                self.raw_dxyz.append(xyz - self._prev_raw_xyz)
                self.raw_drpy.append(rpy - self._prev_raw_rpy)
            else:
                self.raw_dxyz.append(np.zeros(3))
                self.raw_drpy.append(np.zeros(3))
            self._prev_raw_xyz = xyz
            self._prev_raw_rpy = rpy

    # ---------- filtered pose target ----------
    def _filt_cb(self, msg: PoseStamped):
        t = self._sec(msg.header.stamp)
        p = msg.pose.position
        o = msg.pose.orientation
        xyz = np.array([p.x, p.y, p.z])
        rpy = np.array(quat_to_rpy(o.w, o.x, o.y, o.z))

        with self.lock:
            self.filt_t.append(t)
            if self._prev_filt_xyz is not None:
                self.filt_dxyz.append(xyz - self._prev_filt_xyz)
                self.filt_drpy.append(rpy - self._prev_filt_rpy)
            else:
                self.filt_dxyz.append(np.zeros(3))
                self.filt_drpy.append(np.zeros(3))
            self._prev_filt_xyz = xyz
            self._prev_filt_rpy = rpy

    # ---------- robot joints ----------
    def _joint_cb(self, msg: JointState):
        t = self._sec(msg.header.stamp)
        if len(msg.position) < 6:
            return
        pos_deg = np.array([math.degrees(msg.position[i]) for i in range(6)])

        with self.lock:
            self.joint_t.append(t)
            self.joint_pos.append(pos_deg)
            if self._prev_joint_pos is not None and self._prev_joint_t is not None:
                dt = t - self._prev_joint_t
                if dt > 0:
                    self.joint_vel.append((pos_deg - self._prev_joint_pos) / dt)
                else:
                    self.joint_vel.append(np.zeros(6))
            else:
                self.joint_vel.append(np.zeros(6))
            self._prev_joint_pos = pos_deg
            self._prev_joint_t = t


def main():
    rclpy.init()
    node = ArmPipelineRecorder()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=False)
    fig.suptitle("Arm Pipeline Diagnostic (live)", fontsize=13)
    fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.96], h_pad=2.5)

    xyz_labels = ["X", "Y", "Z"]
    rpy_labels = ["Roll", "Pitch", "Yaw"]
    raw_colors = ["#e74c3c", "#2ecc71", "#3498db"]        # bright for raw
    filt_colors = ["#c0392b", "#27ae60", "#2980b9"]        # darker for filtered
    joint_colors = ["#e74c3c", "#e67e22", "#f1c40f", "#2ecc71", "#3498db", "#9b59b6"]

    # --- Subplot 1: Position deltas ---
    ax = axes[0]
    ax.set_title("Position Δ per Frame: Raw (solid) vs Filtered (dashed)", fontsize=10)
    ax.set_ylabel("Δ (m)")
    ax.grid(True, alpha=0.3)
    raw_pos_lines = [ax.plot([], [], color=c, linewidth=0.8, label=f"raw Δ{l}")[0]
                     for c, l in zip(raw_colors, xyz_labels)]
    filt_pos_lines = [ax.plot([], [], color=c, linewidth=0.8, linestyle="--", label=f"filt Δ{l}")[0]
                      for c, l in zip(filt_colors, xyz_labels)]
    ax.legend(loc="upper left", fontsize=7, ncol=6)

    # --- Subplot 2: Orientation deltas ---
    ax = axes[1]
    ax.set_title("Orientation Δ per Frame: Raw (solid) vs Filtered (dashed)", fontsize=10)
    ax.set_ylabel("Δ (deg)")
    ax.grid(True, alpha=0.3)
    raw_rpy_lines = [ax.plot([], [], color=c, linewidth=0.8, label=f"raw Δ{l}")[0]
                     for c, l in zip(raw_colors, rpy_labels)]
    filt_rpy_lines = [ax.plot([], [], color=c, linewidth=0.8, linestyle="--", label=f"filt Δ{l}")[0]
                      for c, l in zip(filt_colors, rpy_labels)]
    ax.legend(loc="upper left", fontsize=7, ncol=6)

    # --- Subplot 3: Joint positions ---
    ax = axes[2]
    ax.set_title("Robot Joint Positions", fontsize=10)
    ax.set_ylabel("Angle (deg)")
    ax.grid(True, alpha=0.3)
    jpos_lines = [ax.plot([], [], color=c, linewidth=0.8, label=l)[0]
                  for c, l in zip(joint_colors, JOINT_NAMES_SHORT)]
    ax.legend(loc="upper left", fontsize=7, ncol=6)

    # --- Subplot 4: Joint velocities ---
    ax = axes[3]
    ax.set_title("Robot Joint Velocity (numerical derivative)", fontsize=10)
    ax.set_ylabel("Velocity (deg/s)")
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)
    jvel_lines = [ax.plot([], [], color=c, linewidth=0.8, label=l)[0]
                  for c, l in zip(joint_colors, JOINT_NAMES_SHORT)]
    ax.legend(loc="upper left", fontsize=7, ncol=6)

    all_lines = raw_pos_lines + filt_pos_lines + raw_rpy_lines + filt_rpy_lines + jpos_lines + jvel_lines

    def update(_frame):
        with node.lock:
            rt = np.array(node.raw_t) if node.raw_t else np.array([])
            rd = np.array(node.raw_dxyz) if node.raw_dxyz else np.empty((0, 3))
            rr = np.array(node.raw_drpy) if node.raw_drpy else np.empty((0, 3))
            ft = np.array(node.filt_t) if node.filt_t else np.array([])
            fd = np.array(node.filt_dxyz) if node.filt_dxyz else np.empty((0, 3))
            fr = np.array(node.filt_drpy) if node.filt_drpy else np.empty((0, 3))
            jt = np.array(node.joint_t) if node.joint_t else np.array([])
            jp = np.array(node.joint_pos) if node.joint_pos else np.empty((0, 6))
            jv = np.array(node.joint_vel) if node.joint_vel else np.empty((0, 6))

        # Position deltas
        if len(rt) > 1 and len(ft) > 1:
            for i, line in enumerate(raw_pos_lines):
                line.set_data(rt, rd[:, i])
            for i, line in enumerate(filt_pos_lines):
                line.set_data(ft, fd[:, i])
            t_min = min(rt[0], ft[0])
            t_max = max(rt[-1], ft[-1])
            axes[0].set_xlim(t_min, t_max)
            axes[0].relim(); axes[0].autoscale_view(scalex=False)

        # Orientation deltas
        if len(rt) > 1 and len(ft) > 1:
            for i, line in enumerate(raw_rpy_lines):
                line.set_data(rt, rr[:, i])
            for i, line in enumerate(filt_rpy_lines):
                line.set_data(ft, fr[:, i])
            axes[1].set_xlim(t_min, t_max)
            axes[1].relim(); axes[1].autoscale_view(scalex=False)

        # Joint positions
        if len(jt) > 1:
            for i, line in enumerate(jpos_lines):
                line.set_data(jt, jp[:, i])
            axes[2].set_xlim(jt[0], jt[-1])
            axes[2].relim(); axes[2].autoscale_view(scalex=False)

        # Joint velocities
        if len(jt) > 1:
            for i, line in enumerate(jvel_lines):
                line.set_data(jt, jv[:, i])
            axes[3].set_xlim(jt[0], jt[-1])
            axes[3].relim(); axes[3].autoscale_view(scalex=False)

        return all_lines

    _anim = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
