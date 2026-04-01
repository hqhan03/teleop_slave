#!/usr/bin/python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


RAD_TO_DEG = 180.0 / math.pi

EXPECTED_MANUS_NAMES = [
    "thumb_mcp_spread", "thumb_mcp_stretch", "thumb_pip", "thumb_dip",
    "index_mcp_spread", "index_mcp_stretch", "index_pip", "index_dip",
    "middle_mcp_spread", "middle_mcp_stretch", "middle_pip", "middle_dip",
    "ring_mcp_spread", "ring_mcp_stretch", "ring_pip", "ring_dip",
    "pinky_mcp_spread", "pinky_mcp_stretch", "pinky_pip", "pinky_dip",
]

LEFT_JOINT_NAMES = [
    "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4",
    "lj_dg_2_1", "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4",
    "lj_dg_3_1", "lj_dg_3_2", "lj_dg_3_3", "lj_dg_3_4",
    "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3", "lj_dg_4_4",
    "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4",
]

RIGHT_JOINT_NAMES = [name.replace("lj_", "rj_") for name in LEFT_JOINT_NAMES]

LEFT_JOINT_LIMITS = {
    "lj_dg_1_1": (-0.8901179185171081, 0.3839724354387525),
    "lj_dg_1_2": (0.0, math.pi),
    "lj_dg_1_3": (-math.pi / 2, math.pi / 2),
    "lj_dg_1_4": (-math.pi / 2, math.pi / 2),
    "lj_dg_2_1": (-0.6108652381980153, 0.4188790204786391),
    "lj_dg_2_2": (0.0, 2.007128639793479),
    "lj_dg_2_3": (-math.pi / 2, math.pi / 2),
    "lj_dg_2_4": (-math.pi / 2, math.pi / 2),
    "lj_dg_3_1": (-0.6108652381980153, 0.6108652381980153),
    "lj_dg_3_2": (0.0, 1.9547687622336491),
    "lj_dg_3_3": (-math.pi / 2, math.pi / 2),
    "lj_dg_3_4": (-math.pi / 2, math.pi / 2),
    "lj_dg_4_1": (-0.4188790204786391, 0.6108652381980153),
    "lj_dg_4_2": (0.0, 1.9024088846738192),
    "lj_dg_4_3": (-math.pi / 2, math.pi / 2),
    "lj_dg_4_4": (-math.pi / 2, math.pi / 2),
    "lj_dg_5_1": (-1.0471975511965976, 0.017453292519943295),
    "lj_dg_5_2": (-0.6108652381980153, 0.4188790204786391),
    "lj_dg_5_3": (-math.pi / 2, math.pi / 2),
    "lj_dg_5_4": (-math.pi / 2, math.pi / 2),
}

RIGHT_JOINT_LIMITS = {
    "rj_dg_1_1": (-0.3839724354387525, 0.8901179185171081),
    "rj_dg_1_2": (-math.pi, 0.0),
    "rj_dg_1_3": (-math.pi / 2, math.pi / 2),
    "rj_dg_1_4": (-math.pi / 2, math.pi / 2),
    "rj_dg_2_1": (-0.4188790204786391, 0.6108652381980153),
    "rj_dg_2_2": (0.0, 2.007128639793479),
    "rj_dg_2_3": (-math.pi / 2, math.pi / 2),
    "rj_dg_2_4": (-math.pi / 2, math.pi / 2),
    "rj_dg_3_1": (-0.6108652381980153, 0.6108652381980153),
    "rj_dg_3_2": (0.0, 1.9547687622336491),
    "rj_dg_3_3": (-math.pi / 2, math.pi / 2),
    "rj_dg_3_4": (-math.pi / 2, math.pi / 2),
    "rj_dg_4_1": (-0.6108652381980153, 0.4188790204786391),
    "rj_dg_4_2": (0.0, 1.9024088846738192),
    "rj_dg_4_3": (-math.pi / 2, math.pi / 2),
    "rj_dg_4_4": (-math.pi / 2, math.pi / 2),
    "rj_dg_5_1": (-0.017453292519943295, 1.0471975511965976),
    "rj_dg_5_2": (-0.4188790204786391, 0.6108652381980153),
    "rj_dg_5_3": (-math.pi / 2, math.pi / 2),
    "rj_dg_5_4": (-math.pi / 2, math.pi / 2),
}


def clamp(value, lower, upper):
    return lower if value < lower else upper if value > upper else value


class ManusRetargetTrajectoryNode(Node):
    def __init__(self):
        super().__init__("manus_retarget_vendor")

        self.declare_parameter("dummy_mode", False)
        self.declare_parameter("hand_side", "right")
        self.declare_parameter("input_topic", "/manus/finger_joints")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("time_from_start_sec", 0.1)

        self._dummy_mode = bool(self.get_parameter("dummy_mode").value)
        self._hand_side = str(self.get_parameter("hand_side").value).strip().lower()
        self._input_topic = str(self.get_parameter("input_topic").value)
        self._time_from_start_sec = float(self.get_parameter("time_from_start_sec").value)

        if self._hand_side not in ("left", "right"):
            raise ValueError("hand_side must be 'left' or 'right'")

        output_topic = str(self.get_parameter("output_topic").value).strip()
        if not output_topic:
            if self._dummy_mode:
                output_topic = "/joint_trajectory_controller/joint_trajectory"
            elif self._hand_side == "left":
                output_topic = "/dg5f_left/dg5f_left_controller/joint_trajectory"
            else:
                output_topic = "/dg5f_right/dg5f_right_controller/joint_trajectory"

        if self._hand_side == "left":
            self._joint_names = LEFT_JOINT_NAMES
            self._joint_limits = LEFT_JOINT_LIMITS
        else:
            self._joint_names = RIGHT_JOINT_NAMES
            self._joint_limits = RIGHT_JOINT_LIMITS

        self._expected_index = {name: idx for idx, name in enumerate(EXPECTED_MANUS_NAMES)}
        self._warned_name_mismatch = False

        self._sub = self.create_subscription(
            JointState, self._input_topic, self._joint_state_callback, 10
        )
        self._pub = self.create_publisher(JointTrajectory, output_topic, 10)

        self.get_logger().info(
            f"Vendor retarget active: {self._input_topic} -> {output_topic} "
            f"(hand_side={self._hand_side}, dummy_mode={self._dummy_mode})"
        )

    def _joint_state_callback(self, msg: JointState):
        q_deg = self._extract_q_deg(msg)
        if q_deg is None:
            return

        out_vals = self._compute_mqd_from_q(q_deg, self._hand_side)
        clamped_vals = []
        for name, val in zip(self._joint_names, out_vals):
            lower, upper = self._joint_limits[name]
            clamped_vals.append(clamp(val, lower, upper))

        traj = JointTrajectory()
        traj.joint_names = self._joint_names

        point = JointTrajectoryPoint()
        point.positions = clamped_vals
        sec = max(self._time_from_start_sec, 0.0)
        point.time_from_start.sec = int(sec)
        point.time_from_start.nanosec = int(round((sec - int(sec)) * 1e9))
        traj.points.append(point)

        self._pub.publish(traj)

    def _extract_q_deg(self, msg: JointState):
        if len(msg.position) < 20:
            self.get_logger().warn(
                f"Ignoring JointState with {len(msg.position)} positions; expected at least 20"
            )
            return None

        if len(msg.name) >= 20:
            name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
            if all(name in name_to_pos for name in EXPECTED_MANUS_NAMES):
                ordered = [name_to_pos[name] for name in EXPECTED_MANUS_NAMES]
                return [value * RAD_TO_DEG for value in ordered]

        if msg.name and not self._warned_name_mismatch:
            self.get_logger().warn(
                "JointState names do not match expected Manus order; falling back to raw position order once"
            )
            self._warned_name_mismatch = True

        return [msg.position[idx] * RAD_TO_DEG for idx in range(20)]

    @staticmethod
    def _compute_mqd_from_q(q_deg, hand_side="left"):
        motor_count = 20
        pi = math.pi

        if q_deg is None:
            q_deg = [0.0] * motor_count
        elif len(q_deg) < motor_count:
            q_deg = list(q_deg) + [0.0] * (motor_count - len(q_deg))
        else:
            q_deg = list(q_deg[:motor_count])

        if hand_side == "left":
            dir_arr = [-1, 1, -1, -1,
                       1, 1, 1, 1,
                       1, 1, 1, 1,
                       1, 1, 1, 1,
                       -1, 1, 1, 1]
        else:
            dir_arr = [1, -1, 1, 1,
                       -1, 1, 1, 1,
                       -1, 1, 1, 1,
                       -1, 1, 1, 1,
                       1, -1, 1, 1]

        calib = [1, 1.6, 1.3, 1.3,
                 1, 1, 1.3, 1.7,
                 1, 1, 1.3, 1.7,
                 1, 1, 1.3, 1.7,
                 1, 1, 1, 1]

        qd = [0.0] * motor_count

        qd[0] = (58.5 - q_deg[1]) * (pi / 180.0)
        qd[1] = (q_deg[0] + 20.0) * (pi / 180.0)
        qd[2] = q_deg[2] * (pi / 180.0)
        qd[3] = 0.5 * (q_deg[2] + q_deg[3]) * (pi / 180.0)

        qd[4] = q_deg[4] * (pi / 180.0)
        qd[5] = q_deg[5] * (pi / 180.0)
        qd[6] = (q_deg[6] - 40.0) * (pi / 180.0)
        qd[7] = q_deg[7] * (pi / 180.0)

        qd[8] = q_deg[8] * (pi / 180.0)
        qd[9] = q_deg[9] * (pi / 180.0)
        qd[10] = (q_deg[10] - 30.0) * (pi / 180.0)
        qd[11] = q_deg[11] * (pi / 180.0)

        qd[12] = q_deg[12] * (pi / 180.0)
        qd[13] = q_deg[13] * (pi / 180.0)
        qd[14] = q_deg[14] * (pi / 180.0)
        qd[15] = q_deg[15] * (pi / 180.0)

        if q_deg[17] > 55.0 and q_deg[18] > 25.0 and q_deg[18] > 20.0:
            qd[16] = abs(q_deg[16]) * 2.0 * (pi / 180.0)
        else:
            qd[16] = abs(q_deg[16]) / 1.5 * (pi / 180.0)

        qd[17] = q_deg[16] * (pi / 180.0)
        qd[18] = q_deg[17] * (pi / 180.0)
        qd[19] = q_deg[18] * (pi / 180.0)

        mqd = [0.0] * motor_count
        for i in range(motor_count):
            mqd[i] = qd[i] * calib[i] * dir_arr[i]

        for i in range(motor_count):
            if i in [4, 8, 12, 16, 17]:
                continue

            if hand_side == "left":
                if i in [0, 2, 3]:
                    if mqd[i] >= 0.0:
                        mqd[i] = 0.0
                else:
                    if mqd[i] <= 0.0:
                        mqd[i] = 0.0
            else:
                if i == 1:
                    if mqd[i] >= 0.0:
                        mqd[i] = 0.0
                else:
                    if mqd[i] <= 0.0:
                        mqd[i] = 0.0

        return mqd


def main(args=None):
    rclpy.init(args=args)
    node = ManusRetargetTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
