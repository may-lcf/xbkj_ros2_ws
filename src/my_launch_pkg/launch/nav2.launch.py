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
        executable='carnode',
        name='carnode',
        output='screen',
    )

    rplidar_ros_dir = get_package_share_directory('rplidar_ros')
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(rplidar_ros_dir, 'launch', 'rplidar_c1_launch.py')
        ]),
        launch_arguments={'arg_name': 'value'}.items()
    )

    return LaunchDescription([
        car_node,
        rplidar_launch
    ])