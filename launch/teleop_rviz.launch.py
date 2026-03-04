import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
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
    )

    # Start the low-level controller with dummy simulation mode enabled to publish to /joint_states
    lowlevel_node = Node(
        package='teleop_slave',
        executable='fairino_lowlevel_controller_node',
        name='fairino_lowlevel_controller_node',
        output='screen',
        parameters=[{'dummy_mode': True}]
    )

    # High-level slave node mapping Manus inputs to Robot coordinates
    slave_node = Node(
        package='teleop_slave',
        executable='fairino_slave_node',
        name='fairino_slave_node',
        output='screen'
    )

    # Python cuRobo node
    curobo_node = Node(
        package='teleop_slave',
        executable='curobo_mppi_solver.py',
        name='curobo_mppi_solver',
        output='screen'
    )

    return LaunchDescription([
        rsp_launch,
        rviz_node,
        lowlevel_node,
        TimerAction(
            period=2.0,  # Wait a bit for the low-level to establish services
            actions=[slave_node, curobo_node]
        )
    ])
