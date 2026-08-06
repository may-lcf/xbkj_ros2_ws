#!/usr/bin/env python3
"""
多点导航 Launch 文件

基于 nav2_navigation.launch.py，增加 waypoint_navigator 节点。
用户在 RViz 中用 "Publish Point" 工具标记目标点，然后发送 start 命令开始顺序导航。

用法：
  ros2 launch pi5_robot_description multi_waypoint_navigation.launch.py
  ros2 launch pi5_robot_description multi_waypoint_navigation.launch.py rviz:=true
  ros2 launch pi5_robot_description multi_waypoint_navigation.launch.py wait_time:=3.0 max_retries:=2
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
from launch_ros.actions import Node, LifecycleNode
from lifecycle_msgs.msg import Transition
import xacro


def generate_launch_description():
    pi5_robot_desc_pkg = get_package_share_directory('pi5_robot_description')
    rplidar_ros_pkg = get_package_share_directory('rplidar_ros')

    # ========== 参数 ==========
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map', default=os.path.join(pi5_robot_desc_pkg, 'maps', 'map_01.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(pi5_robot_desc_pkg, 'config', 'nav2_params.yaml'))
    use_rviz = LaunchConfiguration('rviz', default='true')  # 默认开启 RViz
    wait_time = LaunchConfiguration('wait_time', default='2.0')
    max_retries = LaunchConfiguration('max_retries', default='1')
    auto_yaw = LaunchConfiguration('auto_yaw', default='true')

    # ========== 基础节点 ==========

    # 1. Robot State Publisher
    urdf_file = os.path.join(pi5_robot_desc_pkg, 'urdf', 'pi5_arm_robot.urdf.xacro')
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # 2. cmd_vel 桥接节点 (里程计 + 速度指令)
    cmd_vel_bridge = Node(
        package='pi5_robot_description',
        executable='cmd_vel_bridge_node.py',
        name='cmd_vel_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            'cmd_vel_topic': '/cmd_vel_safety',
            'timeout_sec': 0.5,
        }],
    )

    # 3. RPLidar C1
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_ros_pkg, 'launch', 'rplidar_c1_launch.py')
        ),
    )

    # 4. RViz2 (默认开启，使用 waypoint_nav.rviz 配置)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pi5_robot_desc_pkg, 'config', 'waypoint_nav.rviz')],
        condition=IfCondition(use_rviz),
    )

    # 5. Waypoint Navigator — 多点导航核心节点
    waypoint_navigator_node = Node(
        package='pi5_robot_description',
        executable='waypoint_navigator.py',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'wait_time': wait_time,
            'max_retries': max_retries,
            'auto_yaw': auto_yaw,
            'frame_id': 'map',
        }],
    )

    # ========== Nav2 节点 (LifecycleNode) ==========

    collision_monitor_node = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[os.path.join(pi5_robot_desc_pkg, "config", "collision_monitor_params.yaml")],
    )

    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }],
        namespace='',
    )

    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
        namespace='',
    )

    controller_server_node = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        namespace='',
        remappings=[('/cmd_vel', 'unsmoothed_cmd_vel')],
    )

    planner_server_node = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
        namespace='',
    )

    behavior_server_node = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        namespace='',
        remappings=[('/cmd_vel', 'unsmoothed_cmd_vel')],
    )

    bt_navigator_node = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
        namespace='',
    )

    waypoint_follower_node = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
        namespace='',
    )

    velocity_smoother_node = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        namespace='',
        remappings=[('/cmd_vel', 'unsmoothed_cmd_vel')],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'collision_monitor',
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=map_yaml),
        DeclareLaunchArgument('params_file', default_value=params_file),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('wait_time', default_value='2.0'),
        DeclareLaunchArgument('max_retries', default_value='1'),
        DeclareLaunchArgument('auto_yaw', default_value='true'),

        LogInfo(msg='=== 启动多点导航系统 ==='),
        LogInfo(msg=['  等待时间: ', wait_time, 's | 重试次数: ', max_retries]),

        # 基础节点（立即启动）
        robot_state_publisher,
        cmd_vel_bridge,
        rplidar_launch,
        waypoint_navigator_node,  # 多点导航节点
        rviz_node,

        # Nav2 节点（延时 3 秒启动，等待传感器就绪）
        TimerAction(period=3.0, actions=[
            map_server_node,
            amcl_node,
            controller_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            waypoint_follower_node,
            velocity_smoother_node,
            collision_monitor_node,
            TimerAction(period=2.0, actions=[lifecycle_manager_node]),
        ]),
    ])
