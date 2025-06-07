from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gesture_recognition',
            executable='gesture_node_v2',
            name='gesture_node',
            output='screen',
        ),
    ])
