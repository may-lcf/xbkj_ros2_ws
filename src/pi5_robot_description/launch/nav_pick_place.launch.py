#!/usr/bin/env python3
"""
nav_pick_place.launch.py — 导航夹取与投放 Launch


功能概述:
  集成 Nav2 导航 + 深度相机 + 机械臂的自主夹取投放系统。
  机器人导航至指定夹取点，使用深度相机定位目标，机械臂执行夹取，
  然后导航至投放点完成投放。适用于物流搬运、分拣等场景。

启动节点:
  导航系统(复用 nav2_navigation.launch.py):
    - Nav2 全栈: map_server, amcl, controller/planner/behavior server,
      bt_navigator, waypoint_follower, velocity_smoother, collision_monitor
    - cmd_vel_bridge: 串口桥接
    - rplidar: 激光雷达
    - goal_pose_bridge: 目标位姿桥接

  深度处理:
    - depth_convert_node: 深度图格式转换(mono16 → 32FC1)
      输入: /aurora/depth/image_raw (Aurora原始深度)
      输出: /depth/image_converted (标准32位浮点深度)

  夹取投放(延时8秒启动):
    - nav_pick_place_node: 导航夹取投放核心节点
      功能: 接收夹取/投放点名称 → Nav2导航 → 深度定位 → 机械臂夹取/投放

参数:
  map: 地图YAML路径(默认 map.yaml)
  rviz: 是否启动RViz(默认false)
  pick_point: 夹取点名称(默认 "夹取点")
  place_point: 投放点名称(默认 "投放点")

用法:
  ros2 launch pi5_robot_description nav_pick_place.launch.py
  ros2 launch pi5_robot_description nav_pick_place.launch.py map:=/path/to/map.yaml rviz:=true
  ros2 launch pi5_robot_description nav_pick_place.launch.py pick_point:="A点" place_point:="B点"
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pi5_robot_desc_pkg = get_package_share_directory('pi5_robot_description')

    # 参数
    map_yaml = LaunchConfiguration('map', default=os.path.join(pi5_robot_desc_pkg, 'maps', 'map.yaml'))
    use_rviz = LaunchConfiguration('rviz', default='false')
    pick_point = LaunchConfiguration('pick_point', default='夹取点')
    place_point = LaunchConfiguration('place_point', default='投放点')

    # 1. 导航系统 (复用 nav2_navigation.launch.py)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pi5_robot_desc_pkg, 'launch', 'nav2_navigation.launch.py')),
        launch_arguments={
            'map': map_yaml,
            'rviz': use_rviz,
        }.items(),
    )

    # 2. 深度图转换节点 (mono16 → 32FC1)
    depth_convert = Node(
        package='pi5_robot_description',
        executable='depth_convert_node.py',
        name='depth_convert_node',
        output='screen',
        parameters=[{
            'depth_scale': 0.001,
            'input_topic': '/aurora/depth/image_raw',
            'output_topic': '/depth/image_converted',
        }],
    )

    # 3. 导航夹取投放节点 (延时启动，等待导航系统就绪)
    nav_pick_place = Node(
        package='pi5_robot_description',
        executable='nav_pick_place_node.py',
        name='nav_pick_place_node',
        output='screen',
        parameters=[{
            'pick_point': pick_point,
            'place_point': place_point,
            'frame_id': 'map',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_yaml),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('pick_point', default_value=pick_point),
        DeclareLaunchArgument('place_point', default_value=place_point),

        LogInfo(msg='=== 启动导航夹取与投放系统 ==='),

        # 导航系统
        nav2_launch,

        # 深度转换
        depth_convert,

        # 夹取投放节点 (延时 8 秒，等待 Nav2 就绪)
        TimerAction(period=8.0, actions=[nav_pick_place]),
    ])
