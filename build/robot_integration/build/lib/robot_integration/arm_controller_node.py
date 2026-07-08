#!/usr/bin/env python3
"""
机械臂控制节点

功能：
1. 接收高层指令（抓取/放置/复位等）
2. 转换为STM32舵机指令
3. 发送到 /arm_command 话题
4. 支持关节角度控制和笛卡尔空间控制

STM32 舵机协议:
- 格式: #SSSPxxxxTxxxx!
- SSS: 舵机编号 (000-011)
- xxxx: PWM值 (500-2500, 中位1500)
- xxxx: 运动时间 (ms)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import json
import math


class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller_node')
        
        # ========== 舵机映射 ==========
        # 左臂: 舵机 000-005
        # 右臂: 舵机 006-011
        self.LEFT_ARM_SERVOS = [0, 1, 2, 3, 4, 5]
        self.RIGHT_ARM_SERVOS = [6, 7, 8, 9, 10, 11]
        
        # 关节名称
        self.JOINT_NAMES = ['base', 'shoulder', 'elbow', 'wrist', 'gripper', 'aux']
        
        # ========== 状态 ==========
        self.current_arm = 'left'  # left/right
        self.current_joint_angles = [0.0] * 6  # 弧度
        
        # ========== 订阅 ==========
        self.joint_cmd_sub = self.create_subscription(
            JointState, '/arm_joint_command', self.joint_command_callback, 10)
        self.arm_cmd_sub = self.create_subscription(
            String, '/arm_task_command', self.arm_task_callback, 10)
        
        # ========== 发布 ==========
        self.arm_command_pub = self.create_publisher(String, '/arm_command', 10)
        self.arm_state_pub = self.create_publisher(JointState, '/arm_state', 10)
        
        # ========== 预定义动作 ==========
        self.predefined_actions = {
            'home': self.action_home,
            'reset': self.action_home,
            'grip_open': self.action_grip_open,
            'grip_close': self.action_grip_close,
            'ready': self.action_ready,
            'wave': self.action_wave,
        }
        
        self.get_logger().info('Arm Controller Node 已启动')
    
    def joint_command_callback(self, msg):
        """关节角度控制回调"""
        # 确定使用哪个臂的舵机
        servos = self.LEFT_ARM_SERVOS if self.current_arm == 'left' else self.RIGHT_ARM_SERVOS
        
        # 发送每个关节的舵机指令
        for i, (angle, servo_id) in enumerate(zip(msg.position, servos)):
            # 弧度转PWM: 1500 ± 1000 * angle / (π * 0.75)
            pwm = int(1500 + angle * 1000 / (math.pi * 0.75))
            pwm = max(500, min(2500, pwm))
            
            # 计算运动时间
            time_ms = 1000
            if len(msg.velocity) > i and msg.velocity[i] > 0:
                time_ms = int(msg.velocity[i] * 1000)
            
            cmd = f'#{servo_id:03d}P{pwm:04d}T{time_ms:04d}!'
            self.send_arm_command(cmd)
        
        # 更新当前角度
        self.current_joint_angles = list(msg.position[:6])
        
        # 发布状态
        self.publish_arm_state()
    
    def arm_task_callback(self, msg):
        """任务命令回调"""
        try:
            task = json.loads(msg.data)
            action = task.get('action', '')
            params = task.get('parameters', {})
            
            if action in self.predefined_actions:
                self.predefined_actions[action](**params)
            elif action == 'select_arm':
                self.current_arm = params.get('arm', 'left')
                self.get_logger().info(f'选择{self.current_arm}臂')
            elif action == 'move_joint':
                self.move_single_joint(
                    params.get('joint', 0),
                    params.get('angle', 0),
                    params.get('time', 1000)
                )
            elif action == 'action_group':
                self.execute_action_group(
                    params.get('start', 0),
                    params.get('end', 0),
                    params.get('repeat', 1)
                )
            else:
                self.get_logger().warn(f'未知动作: {action}')
        except Exception as e:
            self.get_logger().error(f'任务解析失败: {e}')
    
    def send_arm_command(self, cmd):
        """发送机械臂命令"""
        msg = String()
        msg.data = cmd
        self.arm_command_pub.publish(msg)
        self.get_logger().debug(f'发送: {cmd}')
    
    def move_single_joint(self, joint_id, angle, time_ms=1000):
        """移动单个关节"""
        servos = self.LEFT_ARM_SERVOS if self.current_arm == 'left' else self.RIGHT_ARM_SERVOS
        if 0 <= joint_id < len(servos):
            servo_id = servos[joint_id]
            pwm = int(1500 + angle * 1000 / (math.pi * 0.75))
            pwm = max(500, min(2500, pwm))
            cmd = f'#{servo_id:03d}P{pwm:04d}T{time_ms:04d}!'
            self.send_arm_command(cmd)
    
    def execute_action_group(self, start, end, repeat=1):
        """执行动作组"""
        cmd = f'$DGT:{start}-{end},{repeat}!'
        self.send_arm_command(cmd)
    
    def action_home(self, **kwargs):
        """复位动作"""
        servos = self.LEFT_ARM_SERVOS if self.current_arm == 'left' else self.RIGHT_ARM_SERVOS
        for servo_id in servos:
            cmd = f'#{servo_id:03d}P1500T2000!'
            self.send_arm_command(cmd)
        self.current_joint_angles = [0.0] * 6
        self.get_logger().info('机械臂复位')
    
    def action_grip_open(self, **kwargs):
        """打开夹爪"""
        servos = self.LEFT_ARM_SERVOS if self.current_arm == 'left' else self.RIGHT_ARM_SERVOS
        gripper_id = servos[4]  # 夹爪是第5个舵机
        cmd = f'#{gripper_id:03d}P1000T1000!'
        self.send_arm_command(cmd)
        self.get_logger().info('夹爪打开')
    
    def action_grip_close(self, **kwargs):
        """关闭夹爪"""
        servos = self.LEFT_ARM_SERVOS if self.current_arm == 'left' else self.RIGHT_ARM_SERVOS
        gripper_id = servos[4]
        cmd = f'#{gripper_id:03d}P2000T1000!'
        self.send_arm_command(cmd)
        self.get_logger().info('夹爪关闭')
    
    def action_ready(self, **kwargs):
        """准备姿态"""
        servos = self.LEFT_ARM_SERVOS if self.current_arm == 'left' else self.RIGHT_ARM_SERVOS
        positions = [1500, 1800, 1200, 1500, 1500, 1500]  # 预设姿态
        for servo_id, pos in zip(servos, positions):
            cmd = f'#{servo_id:03d}P{pos:04d}T2000!'
            self.send_arm_command(cmd)
        self.get_logger().info('准备姿态')
    
    def action_wave(self, **kwargs):
        """挥手动作"""
        servos = self.LEFT_ARM_SERVOS if self.current_arm == 'left' else self.RIGHT_ARM_SERVOS
        # 简单的挥手动作序列
        import threading
        def wave_sequence():
            positions = [
                [1500, 2000, 1500, 1500, 1500, 1500],
                [1500, 2000, 1500, 2000, 1500, 1500],
                [1500, 2000, 1500, 1000, 1500, 1500],
                [1500, 2000, 1500, 2000, 1500, 1500],
                [1500, 1500, 1500, 1500, 1500, 1500],
            ]
            for pos_set in positions:
                for servo_id, pos in zip(servos, pos_set):
                    cmd = f'#{servo_id:03d}P{pos:04d}T500!'
                    self.send_arm_command(cmd)
                import time
                time.sleep(0.6)
        
        threading.Thread(target=wave_sequence, daemon=True).start()
        self.get_logger().info('挥手动作')
    
    def publish_arm_state(self):
        """发布机械臂状态"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES
        msg.position = self.current_joint_angles
        self.arm_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
