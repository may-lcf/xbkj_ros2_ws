from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    从机启动文件：只运行 servo_node（串口控制节点）。
    
    同步原理：
      - 主机（任意功能节点）发布 /joint_commands
      - ROS 2 DDS 在同一局域网 + 相同 ROS_DOMAIN_ID 下自动跨机器共享 topic
      - 本节点订阅 /joint_commands，执行相同的串口指令，实现机械臂同步运动
    
    前提条件：
      - 与主机处于同一局域网
      - export ROS_DOMAIN_ID=0（与主机一致）
    """
    return LaunchDescription([
        # 关节控制节点：订阅 /joint_commands，经串口驱动机械臂
        # 主机所有功能节点（摇杆/语音/视觉分拣等）发布的指令均会自动同步到此
        Node(
            package='my_srv',
            executable='servo_node.py',
            name='servo_node',
            output='screen',
        ),
    ])
