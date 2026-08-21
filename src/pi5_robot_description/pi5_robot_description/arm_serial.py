#!/usr/bin/env python3
"""
arm_serial.py — 机械臂命令发送模块

通过 /arm_cmd 话题发送舵机命令，由 cmd_vel_bridge_node 统一转发到 STM32 串口。
避免多个节点同时操作同一串口导致数据冲突。
"""

import rclpy
from std_msgs.msg import String


# 预定义姿态
ARM_POSES = {
    # 观察位: 机械臂展开，相机朝前朝下，便于视觉检测
    'observe': '{#0P1550T1000!#1P1800T1000!#2P2000T1000!#3P0700T1000!#4P1500T1000!}',
    # 过渡位: 夹取/投放后先移到此位，再回观察位，避免碰撞
    'transition': '{#0P1550T1000!#1P1850T1000!#2P2100T1000!#3P1100T1000!#4P1500T1000!}',
    # 竖直归位: 所有关节中位
    'home': '{#0P1500T1000!#1P1500T1000!#2P1500T1000!#3P1500T1000!#4P1500T1000!}',
    # 夹取准备位: 便于下降抓取
    'pick_ready': '{#0P1500T1000!#1P1432T1000!#2P1871T1000!#3P0666T1000!#4P1500T1000!}',
}


class ArmSerial:
    """
    机械臂命令发送类

    通过 ROS2 话题 /arm_cmd 发送命令，
    由 cmd_vel_bridge_node 订阅并转发到 STM32 串口。
    """

    def __init__(self, node=None):
        """
        Args:
            node: ROS2 Node 实例，用于创建 publisher
        """
        self._node = node
        self._pub = None
        if node is not None:
            self._pub = node.create_publisher(String, '/arm_cmd', 10)

    def set_node(self, node):
        """设置 ROS2 Node（用于延迟初始化）"""
        self._node = node
        if self._pub is None:
            self._pub = node.create_publisher(String, '/arm_cmd', 10)

    def send(self, cmd_str):
        """
        发送舵机控制命令

        Args:
            cmd_str: 命令字符串，如 "{#0P1500T1000!#1P1800T1000!}"

        Returns:
            True 成功, False 失败
        """
        if self._pub is None:
            print('[ArmSerial] 未初始化，无法发送')
            return False
        try:
            msg = String()
            msg.data = cmd_str
            self._pub.publish(msg)
            return True
        except Exception as e:
            print(f'[ArmSerial] 发送失败: {e}')
            return False

    def send_pose(self, pose_name, delay=1.0):
        """
        发送预定义姿态

        Args:
            pose_name: 姿态名称 ('observe', 'home', 'pick_ready')
            delay: 发送后等待时间 (秒)

        Returns:
            True 成功, False 失败
        """
        import time
        cmd = ARM_POSES.get(pose_name)
        if cmd is None:
            print(f'[ArmSerial] 未知姿态: {pose_name}')
            return False
        ok = self.send(cmd)
        if ok and delay > 0:
            time.sleep(delay)
        return ok

    def send_gripper(self, pwm, time_ms=1000):
        """
        控制夹爪

        Args:
            pwm: PWM 值 (1500=中位, >1500闭合, <1500打开)
            time_ms: 运动时间
        """
        cmd = f'{{#5P{pwm:04d}T{time_ms:04d}!}}'
        return self.send(cmd)

    def close(self):
        """无操作"""
        pass


if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('arm_serial_test')
    arm = ArmSerial(node)
    arm.send_pose('observe')
    print('已发送观察位')
    node.destroy_node()
    rclpy.shutdown()
