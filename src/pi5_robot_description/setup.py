from setuptools import setup
import os
from glob import glob

package_name = 'pi5_robot_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lcf',
    maintainer_email='lcf@example.com',
    description='Pi5 机器人 URDF 描述包',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_only_node = pi5_robot_description.odom_only_node:main',
            'cmd_vel_bridge_node = pi5_robot_description.cmd_vel_bridge_node:main',
            'goal_pose_bridge = pi5_robot_description.goal_pose_bridge:main',
            'waypoint_navigator = pi5_robot_description.waypoint_navigator:main',
            'nav_executor_node = pi5_robot_description.nav_executor_node:main',
            'reactive_avoidance_node = pi5_robot_description.reactive_avoidance_node:main',
        ],
    },
)
