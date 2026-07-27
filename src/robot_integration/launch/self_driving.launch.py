import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_dir = os.path.join(pkg_dir, 'config')

    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0')
    auto_start_arg = DeclareLaunchArgument('auto_start', default_value='false')
    enable_camera_arg = DeclareLaunchArgument('enable_camera', default_value='false')

    self_driving_params = os.path.join(config_dir, 'self_driving_params.yaml')

    camera_launch = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src',
        'deptrum-ros-driver', 'launch_aurora930', 'launch', 'aurora930_launch.py')

    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
        condition=IfCondition(LaunchConfiguration('enable_camera')),
    )

    stm32_bridge_node = Node(
        package='robot_integration', executable='stm32_bridge_node',
        name='stm32_bridge_node',
        parameters=[{'serial_port': LaunchConfiguration('serial_port')}],
        output='screen',
    )

    self_driving_node = Node(
        package='robot_integration', executable='self_driving_node',
        name='self_driving_node',
        parameters=[
            self_driving_params,
            {'auto_start': LaunchConfiguration('auto_start')},
        ],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(serial_port_arg)
    ld.add_action(auto_start_arg)
    ld.add_action(enable_camera_arg)
    ld.add_action(camera_node)
    ld.add_action(stm32_bridge_node)
    ld.add_action(TimerAction(period=5.0, actions=[self_driving_node]))
    return ld
