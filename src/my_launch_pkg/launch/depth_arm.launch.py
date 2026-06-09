"""
depth_arm.launch.py — 深度相机 + 机械臂联合启动文件

用法:
  ros2 launch my_launch_pkg depth_arm.launch.py

启动内容:
  1. deptrum Aurora 930 驱动（RGB + 深度 + 点云）
  2. 可选：深度功能节点
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_deptrum = get_package_share_directory('deptrum-ros-driver-aurora930')

    return LaunchDescription([
        # ── 1. Aurora 930 驱动 ──────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_deptrum, 'launch', 'aurora930_launch.py')
            ),
            launch_arguments={
                'rgb_enable':            'True',
                'depth_enable':          'True',
                'ir_enable':             'False',   # 不需要 IR 图像
                'point_cloud_enable':    'True',
                'rgb_fps':               '12',
                'align_mode':            'True',
                'depth_correction':      'True',
                'ir_fps':                '12',
                'resolution_mode_index': '2',
            }.items(),
        ),
    ])
