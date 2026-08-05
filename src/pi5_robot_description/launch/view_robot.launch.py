#!/usr/bin/env python3
"""
查看完整机器人URDF的launch文件
用于在RViz中验证URDF模型是否正确
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 获取包路径
    package_dir = get_package_share_directory('pi5_robot_description')
    urdf_file = os.path.join(package_dir, 'urdf', 'pi5_arm_robot.urdf.xacro')

    # 使用xacro处理URDF文件
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description_content,
            }],
        ),

        # Joint State Publisher GUI (用于手动调整关节)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(package_dir, 'config', 'view_robot.rviz')],
        ),
    ])
