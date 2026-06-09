import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 启动 py_pkg 包中的 carnode.py
    car_node = Node(
        package='py_pkg',
        executable='usbcar',
        name='usbcar',
        output='screen',
    )
    # 启动 joy 节点
    joynode=Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',  # 根据你的手柄设备修改，通常是 js0
            'deadzone': 0.05,         # 摇杆死区，避免微小漂移
            'autorepeat_rate': 10.0,  # 消息发布频率 (Hz)
        }]
    )
    teleop_node=Node(
        package='py_pkg',
        executable='teleop_control_node',
        name='teleop_control_node',
        parameters=[
            # 最重要的映射配置：
            # {'linear_scale': 0.5},  
            # {'angular_scale': 1.0},  
        ],
    )

    # rplidar_ros_dir = get_package_share_directory('rplidar_ros')
    # rplidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(rplidar_ros_dir, 'launch', 'rplidar_c1_launch.py')
    #     ]),
    #     launch_arguments={'arg_name': 'value'}.items()
    # )

    return LaunchDescription([
        car_node,
        joynode,
        teleop_node
        # rplidar_launch
    ])