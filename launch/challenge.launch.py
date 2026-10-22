import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('lab_motion')
    default_waypoints = os.path.join(pkg_share, 'config', 'waypoints.yaml')

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Launch mock TurtleBot3 simulator node (useful when offline / not connected to robot wifi)'
    )

    waypoints_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=default_waypoints,
        description='Path to YAML file defining challenge waypoints'
    )

    slew_rate_arg = DeclareLaunchArgument(
        'use_slew_rate',
        default_value='true',
        description='Enable acceleration slew-rate limiter bonus feature'
    )

    # Mock robot node (only launched when sim:=true)
    mock_robot_node = Node(
        package='lab_motion',
        executable='mock_turtlebot3',
        name='mock_turtlebot3',
        output='screen',
        condition=IfCondition(LaunchConfiguration('sim'))
    )

    # Challenge controller node
    challenge_node = Node(
        package='lab_motion',
        executable='challenge',
        name='challenge_controller',
        output='screen',
        parameters=[{
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'use_slew_rate': LaunchConfiguration('use_slew_rate'),
        }]
    )

    return LaunchDescription([
        sim_arg,
        waypoints_arg,
        slew_rate_arg,
        mock_robot_node,
        challenge_node
    ])
