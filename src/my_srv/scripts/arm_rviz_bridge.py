#!/usr/bin/env python3
"""
arm_rviz_bridge.py - RViz2 → 实体机械臂 桥接节点（功能1）

功能：
  订阅 PC 端 MoveIt/RViz2 发布的 /joint_states 话题，
  将关节角度（弧度）转换为舵机 PWM 值，通过串口发送到实体机械臂。

映射关系：
  RViz2 rad(-2.356 ~ 2.356) → PWM(500 ~ 2500)
  joint1~5 → 舵机 000~004
  joint6+joint7 → 舵机 005（夹爪）

用法（树莓派端）：
  python3 arm_rviz_bridge.py
  python3 arm_rviz_bridge.py --baud 115200 --hz 20

前提：
  - PC 端运行 ros2 launch config demo.launch.py（MoveIt + RViz2）
  - 树莓派和 PC 设置相同的 ROS_DOMAIN_ID
"""

import sys
import os
import time
import argparse

# z_uart.py 路径
_OPENCV_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '..', '..', '..', '..', '..', 'OpenCV')
_OPENCV_DIR = os.path.normpath(_OPENCV_DIR)
if _OPENCV_DIR not in sys.path:
    sys.path.insert(0, _OPENCV_DIR)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from z_uart import uart_send_str, setup_uart, close_uart

# ─── 关节名 → 舵机 ID 映射 ──────────────────────────────────────────────────
JOINT_TO_SERVO = {
    'joint1': 0, 'joint2': 1, 'joint3': 2,
    'joint4': 3, 'joint5': 4,
}


def rad_to_pwm(rad: float) -> int:
    """RViz2 弧度(-2.356~2.356) → 舵机 PWM(500~2500)"""
    pwm = int(rad * 1000.0 / 2.356 + 1500)
    return max(500, min(2500, pwm))


def gripper_rad_to_pwm(j6: float, j7: float) -> int:
    """夹爪关节弧度 → 舵机 005 PWM"""
    avg = (j6 + j7) / 2.0
    if avg <= -0.1:         # 接近 -0.2 → 闭合
        return 1700
    elif avg >= 0.4:        # 接近 0.5  → 张开
        return 1200
    else:
        # 线性插值：0.5(open) → -0.2(close)  ⟹  PWM 1200 → 1700
        t = (0.5 - avg) / 0.7   # 0 → open, 1 → closed
        return int(1200 + t * 500)


class ArmRvizBridge(Node):
    def __init__(self, min_interval: float = 0.05):
        super().__init__('arm_rviz_bridge')
        self.sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)
        self.min_interval = min_interval  # 最小发送间隔（秒）
        self.last_send_time = 0.0
        self.get_logger().info(
            f'桥接节点已启动，订阅 /joint_states，发送频率上限 {1.0/min_interval:.0f}Hz')

    def _joint_state_cb(self, msg: JointState) -> None:
        now = time.time()
        if now - self.last_send_time < self.min_interval:
            return
        self.last_send_time = now

        # 解析关节位置
        pwm_map: dict[int, int] = {}
        j6 = j7 = None

        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_TO_SERVO:
                pwm_map[JOINT_TO_SERVO[name]] = rad_to_pwm(pos)
            elif name == 'joint6':
                j6 = pos
            elif name == 'joint7':
                j7 = pos

        # 夹爪
        if j6 is not None and j7 is not None:
            pwm_map[5] = gripper_rad_to_pwm(j6, j7)

        # 构建联合指令并发送
        if pwm_map:
            cmd = '{'
            for sid in sorted(pwm_map):
                cmd += f"#{sid:03d}P{pwm_map[sid]:04d}T0100!"
            cmd += '}'
            uart_send_str(cmd)


def main() -> None:
    parser = argparse.ArgumentParser(
        description='RViz2 → 实体机械臂桥接节点')
    parser.add_argument('--baud', type=int, default=115200,
                        help='串口波特率（默认 115200）')
    parser.add_argument('--hz', type=float, default=20.0,
                        help='最大发送频率（默认 20Hz）')
    args = parser.parse_args()

    print("正在初始化串口...")
    if not setup_uart(args.baud):
        print("错误：串口初始化失败，请检查连接。")
        sys.exit(1)

    rclpy.init()
    node = ArmRvizBridge(min_interval=1.0 / args.hz)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n收到退出信号...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        close_uart()
        print("串口已关闭，退出。")


if __name__ == '__main__':
    main()
