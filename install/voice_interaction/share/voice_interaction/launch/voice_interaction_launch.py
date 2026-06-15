import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('voice_interaction')
    config_file = os.path.join(package_share, 'config', 'voice_params.yaml')

    return LaunchDescription([
        Node(
            package='voice_interaction',
            executable='voice_recognition_node',
            name='voice_recognition_node',
            parameters=[config_file],
            output='screen',
        ),
        Node(
            package='voice_interaction',
            executable='intent_parser_node',
            name='intent_parser_node',
            parameters=[config_file],
            output='screen',
        ),
        Node(
            package='voice_interaction',
            executable='voice_synthesis_node',
            name='voice_synthesis_node',
            parameters=[config_file],
            output='screen',
        ),
        Node(
            package='voice_interaction',
            executable='arm_executor_node',
            name='arm_executor_node',
            output='screen',
        ),
    ])
