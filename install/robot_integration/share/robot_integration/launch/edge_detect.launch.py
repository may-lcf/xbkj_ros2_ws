"""
edge_detect.launch.py — 边缘检测系统启动文件

启动节点：
  1. STM32桥接节点（小车+机械臂通信）
  2. 边缘检测节点（深度图处理+运动控制）

用法：
  # 基本启动
  ros2 launch robot_integration edge_detect.launch.py

  # 指定串口
  ros2 launch robot_integration edge_detect.launch.py serial_port:=/dev/ttyUSB0

  # 自动启动任务
  ros2 launch robot_integration edge_detect.launch.py auto_start:=true

  # 启动深度相机（如果未启动）
  ros2 launch robot_integration edge_detect.launch.py enable_camera:=true
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    # ── 包路径 ──
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_dir = os.path.join(pkg_dir, 'config')

    # ── 启动参数 ──
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='STM32串口设备路径'
    )

    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='自动启动边缘检测任务'
    )

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='是否启动深度相机驱动'
    )

    # ── 参数文件 ──
    edge_detect_params = os.path.join(config_dir, 'edge_detect_params.yaml')

    # ── STM32 桥接节点 ──
    stm32_bridge_node = Node(
        package='robot_integration',
        executable='stm32_bridge_node',
        name='stm32_bridge_node',
        parameters=[
            {'serial_port': LaunchConfiguration('serial_port')},
        ],
        output='screen',
    )

    # ── 边缘检测节点 ──
    edge_detect_node = Node(
        package='robot_integration',
        executable='edge_detect_node',
        name='edge_detect_node',
        parameters=[
            edge_detect_params,
            {'auto_start': LaunchConfiguration('auto_start')},
        ],
        output='screen',
    )

    # ── 深度相机驱动（可选）──
    camera_launch = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'install',
        'deptrum-ros-driver-aurora930', 'share',
        'deptrum-ros-driver-aurora930', 'launch',
        'aurora930_launch.py'
    )

    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
        condition=IfCondition(LaunchConfiguration('enable_camera')),
    )

    # ── 组装启动文件 ──
    ld = LaunchDescription()

    # 添加参数
    ld.add_action(serial_port_arg)
    ld.add_action(auto_start_arg)
    ld.add_action(enable_camera_arg)

    # 启动相机（如果需要）
    ld.add_action(camera_node)

    # 启动STM32桥接
    ld.add_action(stm32_bridge_node)

    # 延迟启动边缘检测（等待桥接节点初始化）
    ld.add_action(TimerAction(
        period=2.0,
        actions=[edge_detect_node],
    ))

    return ld
