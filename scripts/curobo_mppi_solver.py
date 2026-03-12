#!/usr/bin/env python3
"""
cuRobo IK Solver Node for Real-time Teleoperation
Subscribes to /curobo/pose_target (PoseStamped)
Publishes joint targets to /servo_target (JointState, radians)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

try:
    import torch
    from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.types.robot import RobotConfig
    from curobo.types.state import JointState as CuroboJointState
    from curobo.util_file import load_yaml
    CUROBO_AVAILABLE = True
except ImportError as e:
    CUROBO_AVAILABLE = False
    _import_error = str(e)


class CuroboIKSolverNode(Node):
    def __init__(self):
        super().__init__('curobo_mppi_solver')

        if not CUROBO_AVAILABLE:
            self.get_logger().error(f"cuRobo import failed: {_import_error}")
            return

        self.current_joint_state = None

        # Subscribe to target pose from fairino_slave_node
        self.pose_sub = self.create_subscription(
            PoseStamped, '/curobo/pose_target', self.pose_callback, 10)

        # Subscribe to current joint state from fairino_lowlevel_controller_node
        self.joint_sub = self.create_subscription(
            JointState, '/robot_joint_states', self.joint_callback, 10)

        # Publish joint targets to fairino_lowlevel_controller_node
        self.joint_pub = self.create_publisher(JointState, '/servo_target', 10)

        self.get_logger().info("=" * 50)
        self.get_logger().info("cuRobo IK Solver Node")
        self.get_logger().info("  Subscribing: /curobo/pose_target, /robot_joint_states")
        self.get_logger().info("  Publishing:  /servo_target")
        self.get_logger().info("  Initializing IKSolver (may take ~10s)...")
        self.get_logger().info("=" * 50)

        self.init_curobo()

    def init_curobo(self):
        tensor_args = TensorDeviceType()

        config_path = "/home/hq/teleop_slave/config/fr5.yml"
        config_dict = load_yaml(config_path)
        robot_cfg = RobotConfig.from_dict(config_dict, tensor_args)

        ik_config = IKSolverConfig.load_from_robot_config(
            robot_cfg,
            num_seeds=30,
            position_threshold=0.005,   # 5mm positional tolerance
            rotation_threshold=0.05,    # ~3 degree rotational tolerance
            tensor_args=tensor_args,
        )
        self.ik_solver = IKSolver(ik_config)
        self.get_logger().info("IKSolver ready. Waiting for /robot_joint_states...")

    def joint_callback(self, msg: JointState):
        if not CUROBO_AVAILABLE:
            return
        if len(msg.position) == 0:
            return

        pos_tensor = torch.tensor(
            [list(msg.position)], dtype=torch.float32, device='cuda')
        self.current_joint_state = CuroboJointState.from_position(pos_tensor)

    def pose_callback(self, msg: PoseStamped):
        if not CUROBO_AVAILABLE:
            return

        if self.current_joint_state is None:
            self.get_logger().warn_once(
                "Waiting for joint state on /robot_joint_states. "
                "Is fairino_lowlevel_controller_node running?")
            return

        pos = torch.tensor(
            [[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]],
            dtype=torch.float32, device='cuda')
        # cuRobo quaternion convention: [w, x, y, z]
        quat = torch.tensor(
            [[msg.pose.orientation.w, msg.pose.orientation.x,
              msg.pose.orientation.y, msg.pose.orientation.z]],
            dtype=torch.float32, device='cuda')

        target_pose = Pose(position=pos, quaternion=quat)

        # Solve IK with current joint position as seed for faster convergence
        # retract_config: 1D tensor (num_joints,) — default pose to regularize to
        # seed_config: JointState — warm start to guide IK toward current robot pose
        result = self.ik_solver.solve_single(
            target_pose,
            retract_config=self.current_joint_state.position[0],
            seed_config=self.current_joint_state)


        if result.success.item():
            # solution.position shape: (num_seeds, num_joints) — take first (best) seed
            joint_positions = result.solution.position[0].tolist()

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
            js.position = joint_positions
            self.joint_pub.publish(js)
        else:
            self.get_logger().warn(
                f"IK failed for target pos=[{msg.pose.position.x:.3f}, "
                f"{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}] — "
                "check workspace limits in fr5.yml")


def main(args=None):
    rclpy.init(args=args)
    node = CuroboIKSolverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
