#!/usr/bin/env python3
"""
view_robot.launch.py — URDF 模型查看 Launch
==========================================

功能概述:
  在 RViz 中可视化机器人 URDF 模型，用于验证 URDF 是否正确。
  启动 joint_state_publisher_gui 提供关节滑块控制，
  可实时调整机械臂各关节角度，检查运动学和碰撞。

启动节点:
  - robot_state_publisher: 读取 URDF 并发布各 link 的 TF 变换
  - joint_state_publisher_gui: 关节状态GUI(滑块控制各关节角度)
  - rviz2: 可视化(使用 view_robot.rviz 配置)

参数:
  use_sim_time: 使用仿真时间(默认false)

用法:
  ros2 launch pi5_robot_description view_robot.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('pi5_robot_description')
    urdf_file = os.path.join(package_dir, 'urdf', 'arm_car.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description,
            }],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(package_dir, 'config', 'view_robot.rviz')],
        ),
    ])
