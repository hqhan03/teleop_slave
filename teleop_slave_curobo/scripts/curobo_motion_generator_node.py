#!/home/nrel/curobo_venv/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import torch

from curobo.geom.sdf.world import WorldConfig
from curobo.types.robot import RobotConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose as CuroboPose
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

class CuroboMotionGeneratorNode(Node):
    def __init__(self):
        super().__init__('curobo_motion_generator_node')
        
        self.declare_parameter('curobo_fr5_config', '/home/nrel/Desktop/tesollo_manus_teleop/teleop_slave_curobo/config/fr5.yml')
        cfg_path = self.get_parameter('curobo_fr5_config').value
        
        self.get_logger().info(f"Loading CuRobo config from {cfg_path}")
        import yaml
        tensor_args = TensorDeviceType()
        with open(cfg_path, 'r') as f:
            config_data = yaml.safe_load(f)
        robot_cfg = RobotConfig.from_dict(config_data['robot_cfg'], tensor_args)
        
        ik_config = IKSolverConfig.load_from_robot_config(
            robot_cfg,
            None,
            tensor_args,
            use_cuda_graph=True
        )
        self.ik_solver = IKSolver(ik_config)
        self.get_logger().info("CuRobo IK Solver initialized!")
        
        self.current_q = None
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/robot_joint_states',
            self.joint_state_cb,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/curobo/pose_target',
            self.pose_cb,
            10
        )
        
        self.joint_pub = self.create_publisher(
            JointState,
            '/servo_target',
            10
        )

    def joint_state_cb(self, msg: JointState):
        if len(msg.position) >= 6:
            self.current_q = torch.tensor([msg.position[:6]], dtype=torch.float32, device=self.ik_solver.tensor_args.device)

    def pose_cb(self, msg: PoseStamped):
        if self.current_q is None:
            return
            
        target_pose = CuroboPose(
            position=torch.tensor([[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]], device=self.ik_solver.tensor_args.device),
            quaternion=torch.tensor([[msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]], device=self.ik_solver.tensor_args.device)
        )
        
        result = self.ik_solver.solve_single(target_pose, self.current_q.view(1, -1))
        
        if result.success.item():
            # Update local seed immediately for next tick continuity
            self.current_q = result.solution.clone()
            
            out_msg = JointState()
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.position = result.solution.squeeze().cpu().numpy().tolist()
            self.joint_pub.publish(out_msg)
        else:
            self.get_logger().warn(f"IK Failed! position out of reach or in collision! target: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = CuroboMotionGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
