#!/usr/bin/env python3
"""
Live jitter diagnostic plot for Manus tracker + finger joint data.

Subscribes to /manus/wrist_pose and /manus/finger_joints, displays a 4-panel
live plot showing raw values and frame-to-frame deltas.  Only needs
master_bridge_node running — no robot required.

Usage:
    ros2 run teleop_slave master_bridge_node   # terminal 1
    python3 teleop_slave/scripts/plot_manus_jitter.py  # terminal 2
"""

import threading
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

BUFFER_LEN = 250  # ~5 s at 50 Hz
FINGER_INDICES = [0, 5, 9, 13, 17]  # thumb_spread, index_stretch, mid, ring, pinky
FINGER_LABELS = ["thumb_spr", "index_str", "middle_str", "ring_str", "pinky_str"]


class JitterRecorder(Node):
    def __init__(self):
        super().__init__("jitter_recorder")

        # Wrist buffers
        self.wrist_t = deque(maxlen=BUFFER_LEN)
        self.wrist_xyz = deque(maxlen=BUFFER_LEN)  # (x, y, z)
        self.wrist_dxyz = deque(maxlen=BUFFER_LEN)  # delta per frame
        self._prev_xyz = None

        # Finger buffers (5 representative joints)
        self.finger_t = deque(maxlen=BUFFER_LEN)
        self.finger_vals = deque(maxlen=BUFFER_LEN)  # (5,)
        self.finger_dvals = deque(maxlen=BUFFER_LEN)
        self._prev_fvals = None

        self.t0 = None
        self.lock = threading.Lock()

        self.create_subscription(PoseStamped, "manus/wrist_pose", self._wrist_cb, 10)
        self.create_subscription(JointState, "manus/finger_joints", self._finger_cb, 10)

    def _stamp_sec(self, msg_stamp):
        t = msg_stamp.sec + msg_stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def _wrist_cb(self, msg: PoseStamped):
        t = self._stamp_sec(msg.header.stamp)
        p = msg.pose.position
        xyz = np.array([p.x, p.y, p.z])
        with self.lock:
            self.wrist_t.append(t)
            self.wrist_xyz.append(xyz)
            if self._prev_xyz is not None:
                self.wrist_dxyz.append(xyz - self._prev_xyz)
            else:
                self.wrist_dxyz.append(np.zeros(3))
            self._prev_xyz = xyz

    def _finger_cb(self, msg: JointState):
        t = self._stamp_sec(msg.header.stamp)
        if len(msg.position) < 20:
            return
        vals = np.array([msg.position[i] for i in FINGER_INDICES])
        with self.lock:
            self.finger_t.append(t)
            self.finger_vals.append(vals)
            if self._prev_fvals is not None:
                self.finger_dvals.append(vals - self._prev_fvals)
            else:
                self.finger_dvals.append(np.zeros(5))
            self._prev_fvals = vals


def main():
    rclpy.init()
    node = JitterRecorder()

    # Spin ROS in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Set up matplotlib
    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=False)
    fig.suptitle("Manus Jitter Diagnostic (live)", fontsize=13)
    fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.96], h_pad=2.5)

    xyz_colors = ["#e74c3c", "#2ecc71", "#3498db"]  # R G B for X Y Z
    finger_colors = ["#e67e22", "#9b59b6", "#1abc9c", "#e74c3c", "#3498db"]

    # Axis labels
    axes[0].set_ylabel("Position (m)")
    axes[0].set_title("Tracker XYZ Position", fontsize=10)
    axes[1].set_ylabel("Delta (m/frame)")
    axes[1].set_title("Tracker Frame-to-Frame Delta", fontsize=10)
    axes[2].set_ylabel("Angle (rad)")
    axes[2].set_title("Representative Finger Joints", fontsize=10)
    axes[3].set_ylabel("Delta (rad/frame)")
    axes[3].set_title("Finger Joint Frame-to-Frame Delta", fontsize=10)
    axes[3].set_xlabel("Time (s)")

    for ax in axes:
        ax.grid(True, alpha=0.3)

    # Create line objects
    wrist_lines = [axes[0].plot([], [], color=c, linewidth=0.8, label=l)[0]
                   for c, l in zip(xyz_colors, ["X", "Y", "Z"])]
    dwrist_lines = [axes[1].plot([], [], color=c, linewidth=0.8, label=f"d{l}")[0]
                    for c, l in zip(xyz_colors, ["X", "Y", "Z"])]
    finger_lines = [axes[2].plot([], [], color=c, linewidth=0.8, label=l)[0]
                    for c, l in zip(finger_colors, FINGER_LABELS)]
    dfinger_lines = [axes[3].plot([], [], color=c, linewidth=0.8, label=f"d_{l}")[0]
                     for c, l in zip(finger_colors, FINGER_LABELS)]

    for ax in axes:
        ax.legend(loc="upper left", fontsize=7, ncol=5)

    def update(_frame):
        with node.lock:
            wt = np.array(node.wrist_t) if node.wrist_t else np.array([])
            wxyz = np.array(node.wrist_xyz) if node.wrist_xyz else np.empty((0, 3))
            wdxyz = np.array(node.wrist_dxyz) if node.wrist_dxyz else np.empty((0, 3))
            ft = np.array(node.finger_t) if node.finger_t else np.array([])
            fv = np.array(node.finger_vals) if node.finger_vals else np.empty((0, 5))
            fdv = np.array(node.finger_dvals) if node.finger_dvals else np.empty((0, 5))

        if len(wt) > 1:
            for i, line in enumerate(wrist_lines):
                line.set_data(wt, wxyz[:, i])
            for i, line in enumerate(dwrist_lines):
                line.set_data(wt, wdxyz[:, i])
            axes[0].set_xlim(wt[0], wt[-1])
            axes[0].relim(); axes[0].autoscale_view(scalex=False)
            axes[1].set_xlim(wt[0], wt[-1])
            axes[1].relim(); axes[1].autoscale_view(scalex=False)

        if len(ft) > 1:
            for i, line in enumerate(finger_lines):
                line.set_data(ft, fv[:, i])
            for i, line in enumerate(dfinger_lines):
                line.set_data(ft, fdv[:, i])
            axes[2].set_xlim(ft[0], ft[-1])
            axes[2].relim(); axes[2].autoscale_view(scalex=False)
            axes[3].set_xlim(ft[0], ft[-1])
            axes[3].relim(); axes[3].autoscale_view(scalex=False)

        return wrist_lines + dwrist_lines + finger_lines + dfinger_lines

    _anim = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
