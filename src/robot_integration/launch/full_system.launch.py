"""
完整系统启动文件

启动内容：
1. STM32 通信桥接节点
2. 小车控制节点
3. 机械臂控制节点
4. 协调器节点
5. 深度相机驱动（可选）
6. 激光雷达驱动（可选）

用法：
  ros2 launch robot_integration full_system.launch.py
  ros2 launch robot_integration full_system.launch.py enable_camera:=true
  ros2 launch robot_integration full_system.launch.py enable_lidar:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_robot_integration = get_package_share_directory('robot_integration')
    
    # ========== 参数声明 ==========
    enable_camera = DeclareLaunchArgument(
        'enable_camera', default_value='false',
        description='是否启动深度相机')
    enable_lidar = DeclareLaunchArgument(
        'enable_lidar', default_value='false',
        description='是否启动激光雷达')
    serial_port = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='STM32串口设备')
    control_mode = DeclareLaunchArgument(
        'control_mode', default_value='joy',
        description='控制模式: joy/keyboard/auto')
    
    # ========== 节点定义 ==========
    
    # STM32 通信桥接
    stm32_bridge = Node(
        package='robot_integration',
        executable='stm32_bridge_node',
        name='stm32_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': 115200,
            'publish_tf': True,
        }]
    )
    
    # 小车控制
    car_controller = Node(
        package='robot_integration',
        executable='car_controller_node',
        name='car_controller_node',
        output='screen',
        parameters=[{
            'linear_scale': 0.5,
            'angular_scale': 1.0,
            'control_mode': LaunchConfiguration('control_mode'),
        }]
    )
    
    # 机械臂控制
    arm_controller = Node(
        package='robot_integration',
        executable='arm_controller_node',
        name='arm_controller_node',
        output='screen'
    )
    
    # 协调器
    coordinator = Node(
        package='robot_integration',
        executable='coordinator_node',
        name='coordinator_node',
        output='screen'
    )
    
    # 手柄节点（如果使用手柄模式）
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 10.0,
        }],
        condition=LaunchConfiguration('control_mode', default='joy')
    )
    
    # 构建启动描述
    ld = LaunchDescription([
        enable_camera,
        enable_lidar,
        serial_port,
        control_mode,
        stm32_bridge,
        car_controller,
        arm_controller,
        coordinator,
        joy_node,
    ])
    
    # ========== 可选传感器 ==========
    
    # 深度相机
    camera_launch = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'install',
        'deptrum-ros-driver-aurora930', 'share',
        'deptrum-ros-driver-aurora930', 'launch',
        'aurora930_launch.py'
    )
    
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
        condition=LaunchConfiguration('enable_camera')
    ))
    
    # 激光雷达
    try:
        rplidar_launch = os.path.join(
            get_package_share_directory('rplidar_ros'),
            'launch', 'rplidar_c1_launch.py'
        )
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch),
            condition=LaunchConfiguration('enable_lidar')
        ))
    except:
        pass
    
    return ld
