from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_integration'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lcf',
    maintainer_email='lcf@todo.todo',
    description='小车+机械臂集成控制包',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_bridge_node = robot_integration.stm32_bridge_node:main',
            'car_controller_node = robot_integration.car_controller_node:main',
            'arm_controller_node = robot_integration.arm_controller_node:main',
            'coordinator_node = robot_integration.coordinator_node:main',
        ],
    },
)
