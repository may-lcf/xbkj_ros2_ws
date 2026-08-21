#!/usr/bin/env python3
"""
voice_navigation.launch.py — 语音导航 Launch
============================================

功能概述:
  集成 Nav2 导航 + 语音交互(ASR+LLM+TTS) + YOLO区域检测的语音导航系统。
  用户通过语音发出导航指令(如 "去客厅")，LLM解析意图后调用Nav2导航，
  同时支持语音控制机械臂、查询状态等交互。

启动节点:
  Nav2 导航栈(与 nav2_navigation 一致):
    - robot_state_publisher, cmd_vel_bridge, rplidar, goal_pose_bridge
    - map_server, amcl, controller/planner/behavior server
    - bt_navigator, waypoint_follower, velocity_smoother, collision_monitor
    - lifecycle_manager
    - rviz2: 可视化(默认开启，使用 waypoint_nav.rviz)

  导航执行(立即启动):
    - nav_executor_node: 导航执行节点(接收目标点名称 → Nav2导航)
    - waypoint_recorder_node: 航点记录器(RViz标记 → named_waypoints.yaml)
    - yolo_zone_detect_node: YOLO区域检测(视觉区域识别)

  语音交互(立即启动，可通过use_voice禁用):
    - voice_recognition_node: 语音识别(ASR，麦克风 → 文本)
    - intent_parser_node: 意图解析(LLM，文本 → 结构化指令)
    - voice_synthesis_node: 语音合成(TTS，文本 → 语音播放)
    - arm_executor_node: 机械臂语音控制(解析臂控指令并执行)

数据流:
  麦克风 → voice_recognition → /voice_text → intent_parser
    → /nav_intent → nav_executor → Nav2 /goal_pose → 导航
    → /arm_intent → arm_executor → /arm_command → 机械臂
  intent_parser → /tts_text → voice_synthesis → 扬声器

参数:
  map: 地图YAML路径(默认 map_01.yaml)
  rviz: 是否启动RViz(默认true)
  use_voice: 是否启动语音节点(默认true，false时仅导航)
  waypoints_file: 命名航点文件(named_waypoints.yaml)

用法:
  ros2 launch pi5_robot_description voice_navigation.launch.py
  ros2 launch pi5_robot_description voice_navigation.launch.py map:=/path/to/map.yaml
  ros2 launch pi5_robot_description voice_navigation.launch.py use_voice:=false  # 不启动语音
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
# import xacro (not needed for plain URDF)


def generate_launch_description():
    pi5_robot_desc_pkg = get_package_share_directory('pi5_robot_description')
    rplidar_ros_pkg = get_package_share_directory('rplidar_ros')
    voice_pkg = get_package_share_directory('voice_interaction')

    # ========== 参数 ==========
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map', default=os.path.join(pi5_robot_desc_pkg, 'maps', 'map_01.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(pi5_robot_desc_pkg, 'config', 'nav2_params.yaml'))
    use_rviz = LaunchConfiguration('rviz', default='true')
    use_voice = LaunchConfiguration('use_voice', default='true')
    waypoints_file = LaunchConfiguration('waypoints_file', default=os.path.join(pi5_robot_desc_pkg, 'config', 'named_waypoints.yaml'))

    # ========== Nav2 基础节点 ==========

    urdf_file = os.path.join(pi5_robot_desc_pkg, 'urdf', 'arm_car.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = {'robot_description': f.read()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

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

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_ros_pkg, 'launch', 'rplidar_c1_launch.py')
        ),
        launch_arguments={'frame_id': 'laser_link'}.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pi5_robot_desc_pkg, 'config', 'waypoint_nav.rviz')],
        condition=IfCondition(use_rviz),
    )

    # ========== Nav2 节点 ==========

    collision_monitor_node = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[os.path.join(pi5_robot_desc_pkg, "config", "collision_monitor_params.yaml")],
    )

    map_server_node = LifecycleNode(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', namespace='',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
    )
    amcl_node = LifecycleNode(
        package='nav2_amcl', executable='amcl', name='amcl',
        output='screen', namespace='', parameters=[params_file],
    )
    controller_server_node = LifecycleNode(
        package='nav2_controller', executable='controller_server', name='controller_server',
        output='screen', namespace='', parameters=[params_file],
        remappings=[('/cmd_vel', 'unsmoothed_cmd_vel')],
    )
    planner_server_node = LifecycleNode(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen', namespace='', parameters=[params_file],
    )
    behavior_server_node = LifecycleNode(
        package='nav2_behaviors', executable='behavior_server', name='behavior_server',
        output='screen', namespace='', parameters=[params_file],
        remappings=[('/cmd_vel', 'unsmoothed_cmd_vel')],
    )
    bt_navigator_node = LifecycleNode(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
        output='screen', namespace='', parameters=[params_file],
    )
    waypoint_follower_node = LifecycleNode(
        package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
        output='screen', namespace='', parameters=[params_file],
    )
    velocity_smoother_node = LifecycleNode(
        package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
        output='screen', namespace='', parameters=[params_file],
        remappings=[('/cmd_vel', 'unsmoothed_cmd_vel')],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time, 'autostart': True,
            'node_names': [
                'map_server', 'amcl', 'controller_server', 'planner_server',
                'behavior_server', 'bt_navigator', 'waypoint_follower',
                'velocity_smoother', 'collision_monitor',
            ],
        }],
    )

    # ========== 导航执行节点 ==========

    nav_executor_node = Node(
        package='pi5_robot_description',
        executable='nav_executor_node.py',
        name='nav_executor_node',
        output='screen',
        parameters=[{
            'waypoints_file': waypoints_file,
            'frame_id': 'map',
            'wait_time': 2.0,
            'max_retries': 1,
        }],
    )
    waypoint_recorder_node = Node(
        package="pi5_robot_description",
        executable="waypoint_recorder_node.py",
        name="waypoint_recorder_node",
        output="screen",
        parameters=[{
            "waypoints_file": waypoints_file,
        }],
    )

    # ========== 语音交互节点 ==========

    voice_config = os.path.join(voice_pkg, 'config', 'voice_params.yaml')

    voice_recognition_node = Node(
        package='voice_interaction',
        executable='voice_recognition_node',
        name='voice_recognition_node',
        parameters=[voice_config],
        output='screen',
        condition=IfCondition(use_voice),
    )
    intent_parser_node = Node(
        package='voice_interaction',
        executable='intent_parser_node',
        name='intent_parser_node',
        parameters=[voice_config],
        output='screen',
        condition=IfCondition(use_voice),
    )
    voice_synthesis_node = Node(
        package='voice_interaction',
        executable='voice_synthesis_node',
        name='voice_synthesis_node',
        parameters=[voice_config],
        output='screen',
        condition=IfCondition(use_voice),
    )
    arm_executor_node = Node(
        package='voice_interaction',
        executable='arm_executor_node',
        name='arm_executor_node',
        output='screen',
        condition=IfCondition(use_voice),
    )

    # ========== YOLO 区域检测节点（需在 yolo_env 中运行） ==========
    yolo_zone_detect_node = Node(
        package='pi5_robot_description',
        executable='yolo_zone_detect_node.py',
        name='yolo_zone_detect_node',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=map_yaml),
        DeclareLaunchArgument('params_file', default_value=params_file),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_voice', default_value='true'),
        DeclareLaunchArgument('waypoints_file', default_value=waypoints_file),

        LogInfo(msg='=== 启动语音导航系统 ==='),

        # 基础节点（立即启动）
        robot_state_publisher,
        cmd_vel_bridge,
        rplidar_launch,
        rviz_node,
        nav_executor_node,
        waypoint_recorder_node,
        yolo_zone_detect_node,

        # 语音节点（立即启动）
        voice_recognition_node,
        intent_parser_node,
        voice_synthesis_node,
        arm_executor_node,

        # Nav2 节点（延时 3 秒）
        TimerAction(period=3.0, actions=[
            map_server_node, amcl_node, controller_server_node,
            planner_server_node, behavior_server_node, bt_navigator_node,
            waypoint_follower_node, velocity_smoother_node,
            collision_monitor_node,
            TimerAction(period=2.0, actions=[lifecycle_manager_node]),
        ]),
    ])
