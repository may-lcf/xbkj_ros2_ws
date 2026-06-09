import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    depth_arm.launch.py — 深度相机+机械臂全局启动文件

    启动节点：
      1. aurora930 深度相机驱动（deptrum-ros-driver）
      2. my_srv 各深度视觉节点（按需启动，默认启动 depth_distance）

    用法：
      ros2 launch my_srv depth_arm.launch.py
      ros2 launch my_srv depth_arm.launch.py enable_sorting:=true
      ros2 launch my_srv depth_arm.launch.py enable_stack:=true
      ros2 launch my_srv depth_arm.launch.py enable_track:=true
      ros2 launch my_srv depth_arm.launch.py enable_gesture:=true
    """

    pkg_my_srv = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..'
    )

    # ── 可配置参数 ────────────────────────────────────────────────────────────

    enable_camera = DeclareLaunchArgument(
        'enable_camera', default_value='true',
        description='启动 Aurora930 深度相机驱动')
    enable_distance = DeclareLaunchArgument(
        'enable_distance', default_value='true',
        description='启动 depth_distance_node 交互测距节点')
    enable_sorting = DeclareLaunchArgument(
        'enable_sorting', default_value='false',
        description='启动 depth_color_sorting_node 深度分拣')
    enable_stack = DeclareLaunchArgument(
        'enable_stack', default_value='false',
        description='启动 depth_color_stack_node 深度码垛')
    enable_track = DeclareLaunchArgument(
        'enable_track', default_value='false',
        description='启动 depth_color_track_node 深度追踪')
    enable_gesture = DeclareLaunchArgument(
        'enable_gesture', default_value='false',
        description='启动 hand_gesture 手势控制')
    enable_label = DeclareLaunchArgument(
        'enable_label', default_value='false',
        description='启动 depth_label_sorting_node 标签分拣')
    enable_number = DeclareLaunchArgument(
        'enable_number', default_value='false',
        description='启动 depth_num_sorting_node 数字分拣')

    ld = LaunchDescription([
        enable_camera,
        enable_distance,
        enable_sorting,
        enable_stack,
        enable_track,
        enable_gesture,
        enable_label,
        enable_number,
    ])

    # ── 相机驱动 ──────────────────────────────────────────────────────────────

    camera_launch = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'install',
        'deptrum-ros-driver-aurora930', 'share',
        'deptrum-ros-driver-aurora930', 'launch',
        'aurora930_launch.py'
    )

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
    ))

    # ── 深度工具节点（基础，始终运行） ────────────────────────────────────────

    ld.add_action(Node(
        package='my_srv',
        executable='depth_distance_node.py',
        name='depth_distance_node',
        output='screen',
    ))

    # ── 分拣节点 ─────────────────────────────────────────────────────────────

    ld.add_action(Node(
        package='my_srv',
        executable='depth_color_sorting_node.py',
        name='depth_color_sorting_node',
        output='screen',
    ))

    # ── 码垛节点 ─────────────────────────────────────────────────────────────

    ld.add_action(Node(
        package='my_srv',
        executable='depth_color_stack_node.py',
        name='depth_color_stack_node',
        output='screen',
    ))

    # ── 追踪节点 ─────────────────────────────────────────────────────────────

    ld.add_action(Node(
        package='my_srv',
        executable='depth_color_track_node.py',
        name='depth_color_track_node',
        output='screen',
    ))

    # ── 标签/数字分拣 ────────────────────────────────────────────────────────

    ld.add_action(Node(
        package='my_srv',
        executable='depth_label_sorting_node.py',
        name='depth_label_sorting_node',
        output='screen',
    ))

    ld.add_action(Node(
        package='my_srv',
        executable='depth_num_sorting_node.py',
        name='depth_num_sorting_node',
        output='screen',
    ))

    # ── 手势控制 ─────────────────────────────────────────────────────────────

    ld.add_action(Node(
        package='my_srv',
        executable='hand_gesture_arm_depth.py',
        name='hand_gesture_arm_depth',
        output='screen',
    ))

    return ld
