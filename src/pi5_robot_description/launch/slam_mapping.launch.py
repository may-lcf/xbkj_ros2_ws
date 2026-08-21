#!/usr/bin/env python3
"""
slam_mapping.launch.py — Slam Toolbox 2D 建图 Launch (LifecycleNode 版)
=====================================================================

功能概述:
  使用 Slam Toolbox 进行2D激光建图。通过 odom_only_node 获取轮式里程计，
  RPLidar C1 提供激光扫描，Slam Toolbox 实时构建2D占据栅格地图。
  采用 LifecycleNode 管理，确保参数正确加载。

启动节点:
  传感器层:
    - odom_only_node: 轮式里程计(只读模式，不写串口，仅发布/odom)
    - robot_state_publisher: 发布 URDF 静态 TF
    - rplidar_c1: RPLidar C1 激光雷达驱动(frame_id=laser_link)

  建图核心:
    - slam_toolbox: Slam Toolbox 同步建图节点(LifecycleNode)
      - 订阅: /scan (激光), /odom (里程计), /tf, /tf_static
      - 发布: /map, /map_metadata
      - 自动生命周期管理: configure → activate

关键修复:
  sync_slam_toolbox_node 是 LifecycleNode，必须使用 LifecycleNode action
  并触发 configure → activate 生命周期转换，否则大部分参数不会被加载。

与导航模式的区别:
  - 使用 odom_only_node(只读) 而非 cmd_vel_bridge_node(双职责)
  - 不启动 Nav2 导航栈、collision_monitor 等
  - 仅建图，不导航

参数:
  use_sim_time: 使用仿真时间(默认false)
  autostart: 自动启动生命周期(默认true)
  use_lifecycle_manager: 使用外部生命周期管理器(默认false)

用法:
  ros2 launch pi5_robot_description slam_mapping.launch.py
  ros2 launch pi5_robot_description slam_mapping.launch.py use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_common.launch import RewrittenYaml


def launch_setup(context):
    pi5_robot_desc_pkg = get_package_share_directory('pi5_robot_description')
    rplidar_ros_pkg = get_package_share_directory('rplidar_ros')

    slam_config = os.path.join(pi5_robot_desc_pkg, 'config', 'slam_toolbox_mapping.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')
    autostart = LaunchConfiguration('autostart')

    # --- Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    use_lifecycle_manager_arg = DeclareLaunchArgument('use_lifecycle_manager', default_value='false')

    # 使用 RewrittenYaml 处理参数
    slam_params = RewrittenYaml(
        source_file=slam_config,
        param_rewrites={
            'use_sim_time': use_sim_time,
        },
        convert_types=True,
    )

    # 1. 里程计节点 (只读模式)
    odom_only_node = Node(
        package='pi5_robot_description',
        executable='odom_only_node.py',
        name='odom_only_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        }],
    )

    # 1.5 Robot State Publisher (URDF 静态 TF)
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

    # 2. RPLidar C1
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_ros_pkg, 'launch', 'rplidar_c1_launch.py')
        ),
        launch_arguments={'frame_id': 'laser_link'}.items(),
    )

    # 3. Slam Toolbox — 使用 LifecycleNode（关键修复！）
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[
            slam_params,
            {
                'use_lifecycle_manager': use_lifecycle_manager,
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
        ],
    )

    # Lifecycle 事件：自动 configure
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))),
    )

    # Lifecycle 事件：configure 完成后自动 activate
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[SlamMapping] SlamToolbox 配置完成，正在激活...'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))),
    )

    return [
        use_sim_time_arg,
        autostart_arg,
        use_lifecycle_manager_arg,
        odom_only_node,
        robot_state_publisher,
        rplidar_launch,
        slam_toolbox_node,
        configure_event,
        activate_event,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
