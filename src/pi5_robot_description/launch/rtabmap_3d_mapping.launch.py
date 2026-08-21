#!/usr/bin/env python3
"""
rtabmap_3d_mapping.launch.py — RTAB-Map 3D 建图 Launch
====================================================

功能概述:
  使用 RTAB-Map 进行3D建图，融合激光雷达、深度相机和轮式里程计。
  采用眼在手上配置(深度相机安装在机械臂夹爪)，禁用RTAB-Map内部
  视觉里程计，使用外部轮式里程计(/odom)，避免里程计冲突。

启动节点:
  传感器层:
    - robot_state_publisher: 发布 URDF 静态 TF
    - arm_fixed_pose_node: 机械臂固定到水平姿态(保证相机视角稳定)
    - odom_only_node: 轮式里程计(只读模式，不写串口)
    - rplidar_c1: RPLidar C1 激光雷达驱动
    - rgbd_sync: RGB-D 同步(同步RGB+深度+相机信息，不做深度转换)

  建图核心(延时启动):
    - rtabmap: RTAB-Map SLAM 节点
      - 禁用内部视觉里程计(Odom/Strategy=2)
      - 使用外部轮式里程计(/odom from odom_only_node)
      - 2D地面机器人模式(Reg/Force3DoF=true)
      - ORB 特征提取(600特征点，6最小内点)
      - 深度尺度: Aurora毫米 → 米(depth_scale=0.001)
    - rviz2: 可视化(可选，使用 rtabmap.rviz 配置)

关键配置:
  - Odom/Strategy=2: 禁用RTAB-Map内部视觉里程计
  - Reg/Force3DoF=true: 强制2D平面运动(地面机器人)
  - subscribe_rgbd=true: 订阅RGBD话题而非分开的RGB+深度
  - subscribe_scan=true: 订阅激光扫描用于地图构建

参数:
  rviz: 是否启动RViz(默认false)
  delete_db: 是否删除旧数据库(默认true，建新图时使用)

用法:
  ros2 launch pi5_robot_description rtabmap_3d_mapping.launch.py
  ros2 launch pi5_robot_description rtabmap_3d_mapping.launch.py rviz:=true
  ros2 launch pi5_robot_description rtabmap_3d_mapping.launch.py delete_db:=false  # 保留旧数据
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("pi5_robot_description")
    rplidar_pkg = get_package_share_directory("rplidar_ros")

    urdf_file = os.path.join(pkg, "urdf", "arm_car.urdf")
    with open(urdf_file, "r") as f:
        robot_description = {"robot_description": f.read()}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    arm_fixed_pose = Node(
        package="pi5_robot_description",
        executable="arm_fixed_pose_node.py",
        name="arm_fixed_pose_node",
        output="screen",
        parameters=[{"pose": "horizontal", "rate": 10.0}],
    )

    odom_only_node = Node(
        package="pi5_robot_description",
        executable="odom_only_node.py",
        name="odom_only_node",
        output="screen",
        parameters=[{
            "serial_port": "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            "baudrate": 115200,
        }],
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_pkg, "launch", "rplidar_c1_launch.py")
        ),
        launch_arguments={"frame_id": "laser_link"}.items(),
    )

    # rgbd_sync: 只做同步, 不做深度转换
    rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync",
        output="screen",
        parameters=[{
            "approx_sync": True,
            "approx_sync_max_interval": 0.05,
            "use_sim_time": False,
            "qos": 2,
        }],
        remappings=[
            ("rgb/image", "/aurora/rgb/image_raw"),
            ("rgb/camera_info", "/aurora/rgb/camera_info"),
            ("depth/image", "/aurora/depth/image_raw"),
        ],
    )

    def _launch_rtabmap(context, *args, **kwargs):
        delete_db = LaunchConfiguration("delete_db").perform(context)

        rtabmap_parameters = {
            "frame_id": "base_footprint",
            "use_sim_time": False,
            "subscribe_rgbd": True,
            "subscribe_scan": True,
            "use_action_for_goal": True,
            "qos_scan": 2,
            "qos_image": 2,
            "qos_imu": 2,
            "sync_queue_size": 10,
            "topic_queue_size": 10,
            "Rtabmap/DetectionRate": "1",

            # ===== 核心修复: 禁用 rtabmap 内部视觉里程计 =====
            # 使用外部轮式里程计 (odom_only_node 发布的 /odom)
            # Odom/Strategy: 0=F2M(默认,会冲突), 1=F2F, 2=不使用内部VO
            "Odom/Strategy": "2",

            # ===== 2D地面机器人: Force3DoF=true =====
            "Reg/Force3DoF": "true",
            "Reg/Strategy": "0",  # 视觉特征配准

            # 优化特征提取 (低帧率相机需要更多特征)
            "Vis/MaxFeatures": "600",
            "Vis/MinInliers": "6",
            "Vis/FeatureType": "2",  # 2=ORB (不需要 xfeatures2d)
            "Kp/DetectorStrategy": "2",  # 2=ORB 检测器, 与 FeatureType 一致

            # 网格/点云参数
            "Grid/RangeMin": "0.2",
            "Grid/Sensor": "true",
            "Optimizer/GravitySigma": "0",
            "RGBD/CreateOccupancyGrid": "true",
            "cloud_voxel_size": "0.02",
            "cloud_decimation": "2",

            # ICP 参数
            "ICP/MaxCorrespondenceDistance": "0.3",
            "ICP/Iterations": "10",

            # 回环检测
            "RGBD/ProximityPathMaxNeighbors": "10",

            # 记忆参数: 降低合并阈值, 保留更多节点
            "Mem/RehearsalSimilarity": "0.3",

            # 前端地图大小
            "OdomF2M/MaxSize": "2000",

            # 深度尺度: Aurora 毫米 -> 米
            "depth_scale": "0.001",
        }

        remappings = [
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("rgb/image", "/aurora/rgb/image_raw"),
            ("rgb/camera_info", "/aurora/rgb/camera_info"),
            ("depth/image", "/aurora/depth/image_raw"),
            # 不 remap odom, 避免与 rtabmap 内部里程计冲突
            # rtabmap 会通过 TF 自动获取 odom 变换
        ]

        rtabmap_args = ["-d"] if delete_db == "true" else ["--delete_db_on_start", "false"]

        rtabmap_node = Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[rtabmap_parameters],
            remappings=remappings,
            arguments=rtabmap_args,
        )

        return [TimerAction(period=2.0, actions=[rtabmap_node])]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg, "config", "rtabmap.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz", default="false")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("delete_db", default_value="true"),

        LogInfo(msg="========================================"),
        LogInfo(msg="  RTAB-Map 3D Mapping (外部里程计 + 眼在手上6DoF)"),
        LogInfo(msg="========================================"),

        robot_state_publisher,
        arm_fixed_pose,
        odom_only_node,
        rplidar_launch,

        TimerAction(period=3.0, actions=[rgbd_sync]),
        OpaqueFunction(function=_launch_rtabmap),

        rviz_node,
    ])
