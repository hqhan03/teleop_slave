#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import math

try:
    import torch
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.types.robot import RobotConfig
    from curobo.util_file import get_robot_configs_path, join_path, load_yaml
    CUROBO_AVAILABLE = True
except ImportError:
    CUROBO_AVAILABLE = False

class CuroboMppiSolver(Node):
    def __init__(self):
        super().__init__('curobo_mppi_solver')
        
        if not CUROBO_AVAILABLE:
            self.get_logger().error("cuRobo is not installed. Please install cuRobo via https://curobo.org/get_started/1_install.html")
            
        self.pose_sub = self.create_subscription(
            PoseStamped, '/curobo/pose_target', self.pose_callback, 10)
        
        self.joint_pub = self.create_publisher(
            JointState, '/servo_target', 10)
            
        self.get_logger().info("cuRobo MPPI Solver Node Started")
        self.get_logger().info("Listening to /curobo/pose_target to generate joint trajectories...")
        
        if CUROBO_AVAILABLE:
            self.init_curobo()
            
    def init_curobo(self):
        self.get_logger().info("Initializing cuRobo MotionGen... This takes a few seconds to compile CUDA kernels.")
        
        # NOTE: You will need an FR5 robot config yaml and URDF.
        # This is a template configuration showing how it connects to cuRobo.
        tensor_args = TensorDeviceType()
        
        # Example initialization with a generic config
        # robot_cfg = RobotConfig.from_yaml(join_path(get_robot_configs_path(), "fr5.yaml"), tensor_args)
        
        # motion_gen_config = MotionGenConfig.load_from_robot_config(
        #     robot_cfg,
        #     jnt_traj_pt_count=100,
        #     interpolation_dt=0.004, # 250Hz for Fairino
        #     use_cuda_graph=True
        # )
        # self.motion_gen = MotionGen(motion_gen_config)
        # self.motion_gen.warmup()
        
        self.get_logger().info("cuRobo MotionGen Initialization complete.")
        
    def pose_callback(self, msg: PoseStamped):
        if not CUROBO_AVAILABLE:
            self.get_logger().warn_once("cuRobo not installed. Returning dummy trajectory.")
            return
            
        # Example Usage:
        # pos = torch.tensor([[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]], device='cuda')
        # quat = torch.tensor([[msg.pose.orientation.w, msg.pose.orientation.x, 
        #                       msg.pose.orientation.y, msg.pose.orientation.z]], device='cuda')
        # target_pose = Pose(position=pos, quaternion=quat)
        
        # Run cuRobo MPPI/MotionGen
        # result = self.motion_gen.plan_single(
        #     start_state=self.current_joint_state, 
        #     goal_pose=target_pose
        # )
        # if result.success.item():
        #     next_action = result.optimized_plan[0] # Take first step
        #     
        #     # Publish to ServoJ target for Fairino Controller
        #     js = JointState()
        #     js.header.stamp = self.get_clock().now().to_msg()
        #     js.position = next_action.tolist()
        #     self.joint_pub.publish(js)
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CuroboMppiSolver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
