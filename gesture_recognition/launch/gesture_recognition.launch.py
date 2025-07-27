from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gesture_recognition',
            executable='gesture_node_ML_v3',
            name='gesture_node',
            output='screen',
        ),
    ])
