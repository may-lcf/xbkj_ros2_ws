from setuptools import find_packages, setup

package_name = 'voice_interaction'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/voice_interaction_launch.py']),
        ('share/' + package_name + '/config', ['config/voice_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lcf',
    maintainer_email='lcf@todo.todo',
    description='语音交互功能包 - 阿里云ASR+LLM+TTS在线方案',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'voice_recognition_node = voice_interaction.voice_recognition_node:main',
            'intent_parser_node = voice_interaction.intent_parser_node:main',
            'voice_synthesis_node = voice_interaction.voice_synthesis_node:main',
            'arm_executor_node = voice_interaction.arm_executor_node:main',
        ],
    },
)
