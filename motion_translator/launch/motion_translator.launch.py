from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motion_translator',
            executable='motion_translator_node_ML',
            name='motion_translator_node_ML',
            output='screen',
        )
    ])
