#!/usr/bin/env python3
"""
Nav2 导航 + 深度相机避障 Launch 文件 (增强版)

比基础版多启动：
- Aurora 930 深度相机 (点云用于3D障碍检测)
- costmap 使用激光+点云双源融合

完整数据流:
  controller_server → unsmoothed_cmd_vel → velocity_smoother
    → cmd_vel_smoothed → collision_monitor (监测/scan + /aurora/points2)
    → cmd_vel_safety → cmd_vel_bridge_node → STM32

用法:
  ros2 launch pi5_robot_description nav2_navigation_with_depth.launch.py
  ros2 launch pi5_robot_description nav2_navigation_with_depth.launch.py rviz:=true
  ros2 launch pi5_robot_description nav2_navigation_with_depth.launch.py map:=/path/to/map.yaml
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
import xacro


def generate_launch_description():
    pi5_robot_desc_pkg = get_package_share_directory("pi5_robot_description")
    rplidar_ros_pkg = get_package_share_directory("rplidar_ros")

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    map_yaml = LaunchConfiguration(
        "map", default=os.path.join(pi5_robot_desc_pkg, "maps", "map.yaml")
    )
    params_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(pi5_robot_desc_pkg, "config", "nav2_params.yaml"),
    )
    use_rviz = LaunchConfiguration("rviz", default="false")

    # ==========================================
    # 1. Robot State Publisher
    # ==========================================
    urdf_file = os.path.join(pi5_robot_desc_pkg, "urdf", "pi5_arm_robot.urdf.xacro")
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ==========================================
    # 2. cmd_vel 桥接 (订阅安全过滤后的速度)
    # ==========================================
    cmd_vel_bridge = Node(
        package="pi5_robot_description",
        executable="cmd_vel_bridge_node.py",
        name="cmd_vel_bridge_node",
        output="screen",
        parameters=[{
            "serial_port": "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            "cmd_vel_topic": "/cmd_vel_safety",   # ← 来自 Collision Monitor
            "timeout_sec": 0.5,
        }],
    )

    # ==========================================
    # 3. goal_pose 桥接
    # ==========================================
    goal_pose_bridge = Node(
        package="pi5_robot_description",
        executable="goal_pose_bridge.py",
        name="goal_pose_bridge",
        output="screen",
    )

    # ==========================================
    # 4. RPLidar C1
    # ==========================================
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_ros_pkg, "launch", "rplidar_c1_launch.py")
        ),
    )

    # ==========================================
    # 5. Aurora 930 深度相机 (仅点云 + 深度图)
    # ==========================================
    aurora_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("deptrum-ros-driver-aurora930"),
                "launch",
                "aurora930_launch.py",
            )
        ),
    )

    # ==========================================
    # 6. Collision Monitor (安全层)
    #    输入: cmd_vel_smoothed + /scan + /aurora/points2
    #    输出: cmd_vel_safety (经过安全过滤)
    # ==========================================
    collision_monitor_node = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            os.path.join(pi5_robot_desc_pkg, "config", "collision_monitor_params.yaml")
        ],
    )

    # ==========================================
    # 7. RViz2 (可选)
    # ==========================================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pi5_robot_desc_pkg, "config", "nav2.rviz")],
        condition=IfCondition(use_rviz),
    )

    # ==========================================
    # 8-15. Nav2 Lifecycle 节点
    # ==========================================

    map_server_node = LifecycleNode(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
    )

    amcl_node = LifecycleNode(
        package="nav2_amcl", executable="amcl", name="amcl",
        output="screen", parameters=[params_file],
    )

    controller_server_node = LifecycleNode(
        package="nav2_controller", executable="controller_server", name="controller_server",
        output="screen", parameters=[params_file],
        remappings=[("/cmd_vel", "unsmoothed_cmd_vel")],
    )

    planner_server_node = LifecycleNode(
        package="nav2_planner", executable="planner_server", name="planner_server",
        output="screen", parameters=[params_file],
    )

    behavior_server_node = LifecycleNode(
        package="nav2_behaviors", executable="behavior_server", name="behavior_server",
        output="screen", parameters=[params_file],
        remappings=[("/cmd_vel", "unsmoothed_cmd_vel")],
    )

    bt_navigator_node = LifecycleNode(
        package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
        output="screen", parameters=[params_file],
    )

    waypoint_follower_node = LifecycleNode(
        package="nav2_waypoint_follower", executable="waypoint_follower",
        name="waypoint_follower", output="screen", parameters=[params_file],
    )

    velocity_smoother_node = LifecycleNode(
        package="nav2_velocity_smoother", executable="velocity_smoother",
                "collision_monitor",
        name="velocity_smoother", output="screen", parameters=[params_file],
                "collision_monitor",
        remappings=[("/cmd_vel", "unsmoothed_cmd_vel")],
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation", output="screen",
        parameters=[{
            "use_sim_time": use_sim_time, "autostart": True,
            "node_names": [
                "map_server", "amcl", "controller_server", "planner_server",
                "behavior_server", "bt_navigator", "waypoint_follower",
                "velocity_smoother",
                "collision_monitor",
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map", default_value=map_yaml),
        DeclareLaunchArgument("params_file", default_value=params_file),
        DeclareLaunchArgument("rviz", default_value="false"),

        LogInfo(msg="=== 启动导航系统 (增强版: 深度相机 + 避障) ==="),

        # ---- 传感器 + 桥接 (立即启动) ----
        robot_state_publisher,
        cmd_vel_bridge,
        rplidar_launch,
        aurora_launch,             # 深度相机
        goal_pose_bridge,
        rviz_node,

        # ---- Nav2 + Collision Monitor (延时启动) ----
        TimerAction(period=5.0, actions=[  # 等深度相机就绪
            map_server_node,
            amcl_node,
            controller_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            waypoint_follower_node,
            velocity_smoother_node,
            collision_monitor_node,       # 安全层随Nav2启动
            TimerAction(period=2.0, actions=[lifecycle_manager_node]),
        ]),
    ])
