"""
balance_beam.launch.py — 独木桥导航系统启动文件

启动节点：
  1. Aurora 930 深度相机驱动（可选，enable_camera:=true）
  2. STM32桥接节点（小车+机械臂通信）
  3. 独木桥导航节点（深度图处理+运动控制）

用法：
  # 基本启动（不启动相机，需手动启动）
  ros2 launch robot_integration balance_beam.launch.py

  # 启动相机 + 自动启动任务
  ros2 launch robot_integration balance_beam.launch.py enable_camera:=true auto_start:=true

  # 启用调试模式
  ros2 launch robot_integration balance_beam.launch.py debug:=true
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
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='自动启动独木桥导航任务'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='启用调试模式'
    )

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='是否启动深度相机驱动（默认不启动，需手动启动）'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='STM32串口设备路径'
    )

    # ── 参数文件 ──
    balance_beam_params = os.path.join(config_dir, 'balance_beam_params.yaml')

    # ── Aurora 930 深度相机 ──
    camera_launch = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'src',
        'deptrum-ros-driver', 'launch_aurora930', 'launch',
        'aurora930_launch.py'
    )

    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
        condition=IfCondition(LaunchConfiguration('enable_camera')),
    )

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

    # ── 独木桥导航节点 ──
    balance_beam_node = Node(
        package='robot_integration',
        executable='balance_beam_node',
        name='balance_beam_node',
        parameters=[
            balance_beam_params,
            {'auto_start': LaunchConfiguration('auto_start')},
            {'debug_mode': LaunchConfiguration('debug')},
            {'publish_debug_image': LaunchConfiguration('debug')},
        ],
        output='screen',
    )

    # ── 组装启动文件 ──
    ld = LaunchDescription()

    # 添加参数
    ld.add_action(auto_start_arg)
    ld.add_action(debug_arg)
    ld.add_action(enable_camera_arg)
    ld.add_action(serial_port_arg)

    # 启动相机（如果 enable_camera:=true）
    ld.add_action(camera_node)

    # 启动STM32桥接
    ld.add_action(stm32_bridge_node)

    # 延迟启动导航节点（等待桥接初始化）
    ld.add_action(TimerAction(
        period=2.0,
        actions=[balance_beam_node],
    ))

    return ld
