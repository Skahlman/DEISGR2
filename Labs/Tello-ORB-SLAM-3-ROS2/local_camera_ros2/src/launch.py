from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_camera_publisher',
            executable='camera_publisher',
            name='camera_publisher_node',
            output='screen'
        ),
    ])
