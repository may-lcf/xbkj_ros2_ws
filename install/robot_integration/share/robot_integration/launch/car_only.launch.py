"""
仅小车控制启动文件

用法：
  ros2 launch robot_integration car_only.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    serial_port = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='STM32串口设备')
    
    return LaunchDescription([
        serial_port,
        # STM32 桥接
        Node(
            package='robot_integration',
            executable='stm32_bridge_node',
            name='stm32_bridge_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': 115200,
            }]
        ),
        # 小车控制
        Node(
            package='robot_integration',
            executable='car_controller_node',
            name='car_controller_node',
            output='screen',
            parameters=[{
                'linear_scale': 0.5,
                'angular_scale': 1.0,
                'control_mode': 'joy',
            }]
        ),
        # 手柄
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
            }]
        ),
    ])
