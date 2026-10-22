from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mock_robot_node = Node(
        package='lab_motion',
        executable='mock_turtlebot3',
        name='mock_turtlebot3',
        output='screen'
    )

    return LaunchDescription([
        mock_robot_node
    ])
