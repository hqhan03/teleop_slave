import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config").to_moveit_configs()
    frcobot_launch_dir = os.path.join(get_package_share_directory('fairino5_v6_moveit2_config'), 'launch')
    
    # Include the robot state publisher from the original FR5 workspace
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(frcobot_launch_dir, 'rsp.launch.py'))
    )

    # Use the RViz configuration from the moveit2 config package
    rviz_config_file = os.path.join(get_package_share_directory('fairino5_v6_moveit2_config'), 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ]
    )

    return LaunchDescription([
        rsp_launch,
        rviz_node
    ])
