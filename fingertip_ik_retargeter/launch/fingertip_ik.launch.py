import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('fingertip_ik_retargeter'),
        'config',
        'ik_params.yaml'
    )
    params_file = LaunchConfiguration('params_file')
    calibration_file = LaunchConfiguration('calibration_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_config,
            description='Path to the fingertip IK parameter YAML file.',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='~/.ros/manus_dg5f_ik_frame.yaml',
            description='Override path for the runtime calibration YAML.',
        ),
        Node(
            package='fingertip_ik_retargeter',
            executable='fingertip_ik_node',
            name='fingertip_ik_node',
            output='screen',
            parameters=[params_file, {'calibration_file': calibration_file}],
        ),
    ])
