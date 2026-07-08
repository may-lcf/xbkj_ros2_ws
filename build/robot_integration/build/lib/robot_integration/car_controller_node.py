#!/usr/bin/env python3
"""
小车控制节点

功能：
1. 订阅手柄输入 /joy
2. 发布速度命令 /cmd_vel
3. 支持多种控制模式（手柄/键盘/自动）
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Int32


class CarControllerNode(Node):
    def __init__(self):
        super().__init__('car_controller_node')
        
        # ========== 参数 ==========
        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('control_mode', 'joy')  # joy/keyboard/auto
        
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.control_mode = self.get_parameter('control_mode').value
        
        # ========== 状态 ==========
        self.num = 0
        self.last_buttons = [0] * 12
        self.last_axes = [0] * 8
        self.aim = 0  # 0=左臂, 1=右臂
        
        # ========== 订阅 ==========
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_sub = self.create_subscription(String, '/car_command', self.car_command_callback, 10)
        
        # ========== 发布 ==========
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.num_pub = self.create_publisher(Int32, '/num_cmd', 10)
        self.aim_pub = self.create_publisher(Int32, '/aim_cmd', 10)
        
        self.get_logger().info(f'Car Controller Node 已启动 (模式: {self.control_mode})')
    
    def joy_callback(self, msg):
        """手柄控制回调"""
        twist = Twist()
        
        # 左摇杆控制移动
        twist.linear.x = msg.axes[1] * self.linear_scale
        twist.linear.y = msg.axes[0] * self.linear_scale
        twist.angular.z = msg.axes[3] * self.angular_scale
        
        self.cmd_vel_pub.publish(twist)
        
        # 按钮映射
        try:
            # 移动时重置num
            if msg.axes[1] != 0.0 or msg.axes[3] != 0.0:
                self.num = 0
            
            # 方向键
            elif msg.axes[6] == 1.0:
                self.num = 18  # 右
            elif msg.axes[6] == -1.0:
                self.num = 17  # 左
            elif msg.axes[6] == 0.0 and self.last_axes[6] != 0.0:
                self.num = 19  # 左右停止
            elif msg.axes[7] == 1.0:
                self.num = 21  # 上
            elif msg.axes[7] == -1.0:
                self.num = 20  # 下
            elif msg.axes[7] == 0.0 and self.last_axes[7] != 0.0:
                self.num = 22  # 上下停止
            
            # 按钮
            elif msg.buttons[0] == 1:
                self.num = 1  # A
            elif msg.buttons[0] == 0 and self.last_buttons[0] == 1:
                self.num = 2
            elif msg.buttons[3] == 1:
                self.num = 3  # Y
            elif msg.buttons[3] == 0 and self.last_buttons[3] == 1:
                self.num = 4
            elif msg.buttons[1] == 1:
                self.num = 5  # B
            elif msg.buttons[1] == 0 and self.last_buttons[1] == 1:
                self.num = 6
            elif msg.buttons[2] == 1:
                self.num = 7  # X
            elif msg.buttons[2] == 0 and self.last_buttons[2] == 1:
                self.num = 8
            elif msg.buttons[4] == 1:
                self.num = 9  # L1
            elif msg.buttons[4] == 0 and self.last_buttons[4] == 1:
                self.num = 10
            elif msg.buttons[5] == 1:
                self.num = 11  # R1
            elif msg.buttons[5] == 0 and self.last_buttons[5] == 1:
                self.num = 12
            elif msg.buttons[6] == 1:
                self.num = 13  # L2
            elif msg.buttons[6] == 0 and self.last_buttons[6] == 1:
                self.num = 14
            elif msg.buttons[7] == 1:
                self.num = 15  # R2
            elif msg.buttons[7] == 0 and self.last_buttons[7] == 1:
                self.num = 16
            # Select: 里程计复位
            elif msg.buttons[8] == 1 and self.last_buttons[8] == 0:
                self.num = 23
            # Start: 执行动作组
            elif msg.buttons[9] == 1 and self.last_buttons[9] == 0:
                self.num = 25
            # 左臂选择
            elif msg.buttons[10] == 1 and self.last_buttons[10] == 0:
                self.aim = 0
                self.publish_aim()
            # 右臂选择
            elif msg.buttons[11] == 1 and self.last_buttons[11] == 0:
                self.aim = 1
                self.publish_aim()
            
            self.publish_num()
        except IndexError:
            pass
        
        self.last_buttons = msg.buttons
        self.last_axes = msg.axes
    
    def car_command_callback(self, msg):
        """字符串命令回调"""
        cmd = msg.data.lower()
        twist = Twist()
        
        if cmd == 'forward':
            twist.linear.x = self.linear_scale
        elif cmd == 'backward':
            twist.linear.x = -self.linear_scale
        elif cmd == 'left':
            twist.linear.y = self.linear_scale
        elif cmd == 'right':
            twist.linear.y = -self.linear_scale
        elif cmd == 'turn_left':
            twist.angular.z = self.angular_scale
        elif cmd == 'turn_right':
            twist.angular.z = -self.angular_scale
        elif cmd == 'stop':
            pass  # twist默认全0
        
        self.cmd_vel_pub.publish(twist)
    
    def publish_num(self):
        msg = Int32()
        msg.data = self.num
        self.num_pub.publish(msg)
    
    def publish_aim(self):
        msg = Int32()
        msg.data = self.aim
        self.aim_pub.publish(msg)
        self.get_logger().info(f'切换到{"右" if self.aim == 1 else "左"}臂')


def main(args=None):
    rclpy.init(args=args)
    node = CarControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
