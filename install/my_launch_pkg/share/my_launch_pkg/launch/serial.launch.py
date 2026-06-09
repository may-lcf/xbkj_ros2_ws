from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 小车控制 节点
        Node(
            package='py_pkg',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',  # 位置调整到parameters前
            parameters=[{
                'port1': '/dev/usb_port2',
                'port2': '/dev/usb_port3',
                'cmd': '$DGT:0-12,1!',
                'baudrate': 115200,
                'timeout': 0.1,
                'debug': False,
            }]
        ),
        # 启动 joy 节点
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',  # 根据你的手柄设备修改，通常是 js0
                'deadzone': 0.05,         # 摇杆死区，避免微小漂移
                'autorepeat_rate': 10.0,  # 消息发布频率 (Hz)
            }]
        ),
        Node(
            package='py_pkg',
            executable='teleop_control_node',
            name='teleop_control_node',
            parameters=[
                # 最重要的映射配置：
                # {'linear_scale': 0.5},  
                # {'angular_scale': 1.0},  
            ],
        )
    ])