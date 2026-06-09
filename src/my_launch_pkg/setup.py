import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_launch_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 添加这行来安装 launch 目录中的所有文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 如果需要安装其他文件，可以添加更多路径
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='Launch package for car and lidar integration',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [],
    },
)