#!/usr/bin/env python3
"""
协调器节点

功能：
1. 协调小车和机械臂的动作
2. 提供高级任务接口（导航到目标、抓取物体等）
3. 整合视觉反馈
4. 状态机管理
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point
from std_msgs.msg import String, Int32
from sensor_msgs.msg import JointState
import json
import math
import time
import threading


class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator_node')
        
        # ========== 状态机 ==========
        self.state = 'idle'  # idle/navigating/aligning/grasping/placing
        self.current_task = None
        self.task_queue = []
        
        # ========== 机器人状态 ==========
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        
        # ========== 视觉目标 ==========
        self.detected_objects = []
        self.target_object = None
        
        # ========== 订阅 ==========
        self.odom_sub = self.create_subscription(
            String, '/odom_raw', self.odom_callback, 10)
        self.object_sub = self.create_subscription(
            String, '/detected_objects', self.object_callback, 10)
        self.task_sub = self.create_subscription(
            String, '/robot_task', self.task_callback, 10)
        self.voice_cmd_sub = self.create_subscription(
            String, '/voice_command', self.voice_command_callback, 10)
        
        # ========== 发布 ==========
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_task_pub = self.create_publisher(String, '/arm_task_command', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        
        # ========== 定时器 ==========
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        self.status_timer = self.create_timer(1.0, self.publish_status)  # 1Hz
        
        self.get_logger().info('Coordinator Node 已启动')
    
    def odom_callback(self, msg):
        """里程计回调"""
        try:
            data = json.loads(msg.data)
            self.robot_x = data.get('x', 0.0)
            self.robot_y = data.get('y', 0.0)
            self.robot_theta = data.get('theta', 0.0)
        except:
            pass
    
    def object_callback(self, msg):
        """视觉目标回调"""
        try:
            self.detected_objects = json.loads(msg.data)
        except:
            pass
    
    def task_callback(self, msg):
        """任务命令回调"""
        try:
            task = json.loads(msg.data)
            self.task_queue.append(task)
            self.get_logger().info(f'新任务: {task.get("type", "unknown")}')
        except Exception as e:
            self.get_logger().error(f'任务解析失败: {e}')
    
    def voice_command_callback(self, msg):
        """语音命令回调"""
        try:
            cmd = json.loads(msg.data)
            self.process_voice_command(cmd)
        except Exception as e:
            self.get_logger().error(f'语音命令处理失败: {e}')
    
    def process_voice_command(self, cmd):
        """处理语音命令"""
        func = cmd.get('function', '')
        params = cmd.get('parameters', {})
        
        if func == 'navigate':
            # 导航到指定位置
            self.task_queue.append({
                'type': 'navigate',
                'target_x': params.get('x', 0.0),
                'target_y': params.get('y', 0.0)
            })
        elif func == 'pick':
            # 抓取物体
            self.task_queue.append({
                'type': 'pick',
                'object': params.get('object', '')
            })
        elif func == 'place':
            # 放置物体
            self.task_queue.append({
                'type': 'place',
                'location': params.get('location', '')
            })
        elif func == 'follow':
            # 跟随目标
            self.task_queue.append({
                'type': 'follow',
                'target': params.get('target', '')
            })
    
    def control_loop(self):
        """主控制循环"""
        if not self.task_queue and self.state == 'idle':
            return
        
        # 获取当前任务
        if self.current_task is None and self.task_queue:
            self.current_task = self.task_queue.pop(0)
            self.state = self.current_task.get('type', 'idle')
        
        if self.current_task is None:
            return
        
        task_type = self.current_task.get('type', '')
        
        if task_type == 'navigate':
            self.handle_navigate()
        elif task_type == 'pick':
            self.handle_pick()
        elif task_type == 'place':
            self.handle_place()
        elif task_type == 'follow':
            self.handle_follow()
        else:
            self.current_task = None
            self.state = 'idle'
    
    def handle_navigate(self):
        """处理导航任务"""
        target_x = self.current_task.get('target_x', 0.0)
        target_y = self.current_task.get('target_y', 0.0)
        
        # 计算距离和角度
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.robot_theta
        
        # 归一化角度
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        twist = Twist()
        
        if distance > 0.1:  # 距离阈值
            if abs(angle_diff) > 0.2:  # 角度阈值
                # 先转向
                twist.angular.z = 0.3 if angle_diff > 0 else -0.3
            else:
                # 前进
                twist.linear.x = 0.3
                twist.angular.z = angle_diff * 0.5
        else:
            # 到达目标
            self.get_logger().info(f'到达目标: ({target_x}, {target_y})')
            self.current_task = None
            self.state = 'idle'
        
        self.cmd_vel_pub.publish(twist)
    
    def handle_pick(self):
        """处理抓取任务"""
        object_name = self.current_task.get('object', '')
        
        # 查找目标物体
        target = None
        for obj in self.detected_objects:
            if obj.get('name', '') == object_name:
                target = obj
                break
        
        if target is None:
            self.get_logger().warn(f'未找到物体: {object_name}')
            self.current_task = None
            self.state = 'idle'
            return
        
        # 简单的抓取逻辑：移动到物体上方，然后下降抓取
        msg = String()
        msg.data = json.dumps({
            'action': 'pick',
            'parameters': {
                'x': target.get('x', 0.0),
                'y': target.get('y', 0.0),
                'z': target.get('z', 0.0)
            }
        })
        self.arm_task_pub.publish(msg)
        
        self.current_task = None
        self.state = 'idle'
    
    def handle_place(self):
        """处理放置任务"""
        location = self.current_task.get('location', '')
        
        msg = String()
        msg.data = json.dumps({
            'action': 'place',
            'parameters': {
                'location': location
            }
        })
        self.arm_task_pub.publish(msg)
        
        self.current_task = None
        self.state = 'idle'
    
    def handle_follow(self):
        """处理跟随任务"""
        target_name = self.current_task.get('target', '')
        
        # 查找目标
        target = None
        for obj in self.detected_objects:
            if obj.get('name', '') == target_name:
                target = obj
                break
        
        if target is None:
            # 没有检测到目标，停止
            self.cmd_vel_pub.publish(Twist())
            return
        
        # 简单的跟随逻辑
        dx = target.get('x', 0.0)
        dy = target.get('y', 0.0)
        distance = math.sqrt(dx*dx + dy*dy)
        
        twist = Twist()
        if distance > 1.0:  # 保持1米距离
            twist.linear.x = 0.2
            twist.angular.z = math.atan2(dy, dx) * 0.5
        
        self.cmd_vel_pub.publish(twist)
    
    def publish_status(self):
        """发布机器人状态"""
        status = {
            'state': self.state,
            'position': {
                'x': self.robot_x,
                'y': self.robot_y,
                'theta': self.robot_theta
            },
            'current_task': self.current_task.get('type', 'none') if self.current_task else 'none',
            'queue_length': len(self.task_queue)
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
