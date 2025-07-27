from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

rviz_config_file = os.path.join(
    get_package_share_directory('robot_control'),  # your package name here
    'rviz',
    'robot_visual.rviz'
)

def generate_launch_description():
    enable_gesture = LaunchConfiguration('enable_gesture', default='true')
    enable_translator = LaunchConfiguration('enable_translator', default='true')
    enable_robot = LaunchConfiguration('enable_robot', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('enable_gesture', default_value='true'),
        DeclareLaunchArgument('enable_translator', default_value='true'),
        DeclareLaunchArgument('enable_robot', default_value='true'),

        Node(
            package='gesture_recognition',
            executable='gesture_node_ML_v3',
            name='gesture_recognition_node',
            output='screen',
            condition=IfCondition(enable_gesture)
        ),
        Node(
            package='motion_translator',
            executable='motion_translator_node_ML',
            name='motion_translator_node',
            output='screen',
            condition=IfCondition(enable_translator)
        ),
        Node(
            package='robot_control',
            executable='robot_node',
            name='robot_control_node',
            output='screen',
            condition=IfCondition(enable_robot)
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
