#!/usr/bin/env python3
"""
Nav2 导航 Launch 文件 (精简版)

只启动导航必需的节点，避免启动不需要的组件：
- 不启动: route_server, opennav_docking, smoother_server, odom_only_node, depth_camera
- 启动: cmd_vel_bridge (双职责: 里程计+速度), collision_monitor (安全层), rplidar, map_server, amcl,
        controller_server, planner_server, behavior_server,
        bt_navigator, waypoint_follower, velocity_smoother

注意：导航时不启动 odom_only_node，由 cmd_vel_bridge_node 同时负责里程计查询和速度指令。
      odom_only_node 仅在建图模式下使用（真正只读，不写串口）。

用法：
  ros2 launch pi5_robot_description nav2_navigation.launch.py
  ros2 launch pi5_robot_description nav2_navigation.launch.py map:=/path/to/map.yaml
  ros2 launch pi5_robot_description nav2_navigation.launch.py rviz:=true
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
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import xacro


def generate_launch_description():
    pi5_robot_desc_pkg = get_package_share_directory('pi5_robot_description')
    rplidar_ros_pkg = get_package_share_directory('rplidar_ros')

    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map', default=os.path.join(pi5_robot_desc_pkg, 'maps', 'map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(pi5_robot_desc_pkg, 'config', 'nav2_params.yaml'))
    use_rviz = LaunchConfiguration('rviz', default='false')

    # 1. Robot State Publisher — 发布 URDF 中的静态 TF
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

    # 2. cmd_vel 桥接节点 (双职责: 里程计查询 + 速度指令)
    #    导航时由这一个节点统一管理串口，避免串口冲突
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

    # 2.5 goal_pose 桥接 (RViz SetGoal -> /goal_pose -> /navigate_to_pose)
    goal_pose_bridge = Node(
        package='pi5_robot_description',
        executable='goal_pose_bridge.py',
        name='goal_pose_bridge',
        output='screen',
    )

    # 3. RPLidar C1
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_ros_pkg, 'launch', 'rplidar_c1_launch.py')
        ),
    )

    # 4. RViz2 (可选)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pi5_robot_desc_pkg, 'config', 'nav2.rviz')],
        condition=IfCondition(use_rviz),
    )


    # 3.5 Collision Monitor — 硬安全层 (独立于Nav2生命周期)
    #     监测 /scan, 对 cmd_vel_smoothed 进行 Stop/Slowdown/Limit 过滤
    collision_monitor_node = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[os.path.join(pi5_robot_desc_pkg, "config", "collision_monitor_params.yaml")],
    )

    # ========== Nav2 节点 (LifecycleNode) ==========

    # Map Server
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

    # AMCL
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
        namespace='',
    )

    # Controller Server
    controller_server_node = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        namespace='',
        remappings=[('/cmd_vel', 'unsmoothed_cmd_vel')],
    )

    # Planner Server
    planner_server_node = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
        namespace='',
    )

    # Behavior Server
    behavior_server_node = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        namespace='',
    )

    # BT Navigator
    bt_navigator_node = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
        namespace='',
    )

    # Waypoint Follower
    waypoint_follower_node = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
        namespace='',
    )

    # Velocity Smoother
    velocity_smoother_node = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        namespace='',
        remappings=[('/cmd_vel', 'unsmoothed_cmd_vel')],
    )

    # Lifecycle Manager — 管理所有 Nav2 节点的生命周期
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
        DeclareLaunchArgument('rviz', default_value='false'),

        LogInfo(msg='=== 启动导航系统 (精简版) ==='),

        # 基础节点
        robot_state_publisher,
        cmd_vel_bridge,
        rplidar_launch,
        goal_pose_bridge,
        rviz_node,

        # Nav2 节点 (延时 3 秒启动)
        TimerAction(period=3.0, actions=[
            map_server_node,
            amcl_node,
            controller_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            waypoint_follower_node,
            velocity_smoother_node,
            collision_monitor_node,    # Collision Monitor 随 Nav2 一起启动
            # Lifecycle Manager 最后启动，负责激活所有节点
            TimerAction(period=2.0, actions=[lifecycle_manager_node]),
        ]),
    ])
