from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    threshold = LaunchConfiguration('threshold')
    
    return LaunchDescription([
        DeclareLaunchArgument('threshold', default_value='0.5'),
        Node(package='lab_comm', executable='sensor_publisher',
             name='sensor_publisher',
             parameters=[{'rate_hz': 10.0, 'amplitude': 1.0}],
             output='screen'),
        Node(package='lab_comm', executable='processing_node',
             name='processing_node',
             parameters=[{'threshold': threshold}],
             output='screen'),
        Node(package='lab_comm', executable='reset_service',
             name='reset_service', output='screen'),
        Node(package='lab_comm', executable='motion_action_server',
             name='motion_action_server', output='screen'),
    ])
