import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('teleop_slave_curobo')
    config_file = os.path.join(pkg_dir, 'config', 'fr5_tracker_teleop.yaml')

    lowlevel_node = Node(
        package='teleop_slave_curobo',
        executable='fairino_lowlevel_controller_node',
        name='fairino_lowlevel_controller_node',
        parameters=[config_file],
        output='screen'
    )

    curobo_node = Node(
        package='teleop_slave_curobo',
        executable='curobo_motion_generator_node.py',
        name='curobo_motion_generator_node',
        output='screen'
    )

    return LaunchDescription([
        lowlevel_node
    ])
