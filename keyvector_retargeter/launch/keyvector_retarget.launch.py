import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('keyvector_retargeter'),
        'config',
        'keyvector_params.yaml',
    )
    params_file = LaunchConfiguration('params_file')
    calibration_file = LaunchConfiguration('calibration_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_config,
            description='Path to the keyvector retargeter parameter YAML file.',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='~/.ros/manus_dg5f_keyvector.yaml',
            description='Path to the keyvector calibration YAML file.',
        ),
        Node(
            package='keyvector_retargeter',
            executable='keyvector_retarget_node',
            name='keyvector_retarget_node',
            output='screen',
            parameters=[params_file, {'calibration_file': calibration_file}],
        ),
    ])
