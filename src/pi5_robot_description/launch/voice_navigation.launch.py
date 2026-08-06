#!/usr/bin/env python3
"""
语音导航 Launch 文件

集成 Nav2 导航栈 + 语音交互系统（ASR + LLM + TTS）+ 导航执行节点。

用法：
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
import xacro


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
