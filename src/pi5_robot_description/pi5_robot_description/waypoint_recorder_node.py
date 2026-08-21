#!/usr/bin/env python3
"""
waypoint_recorder_node.py — 目标点记录节点

功能：
  订阅 /amcl_pose 获取当前小车位姿，
  通过命令行或语音指令记录为命名目标点（如"夹取点"、"投放点"），
  自动保存到 named_waypoints.yaml。

用法：
  ros2 run pi5_robot_description waypoint_recorder_node.py

  # 命令行记录：
  ros2 topic pub --once /waypoint_record std_msgs/String "data: '夹取点'"
  ros2 topic pub --once /waypoint_record std_msgs/String "data: '投放点'"

  # 语音指令（如果启动了语音系统）：
  # "记录当前位置为夹取点"

话题：
  订阅:
    /amcl_pose (geometry_msgs/PoseWithCovarianceStamped)
    /waypoint_record (std_msgs/String) — 记录指令
    /voice_command (std_msgs/String) — 语音指令（可选）
  发布:
    /waypoint_record_status (std_msgs/String) — 状态反馈
"""

import math
import os
import threading

import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class WaypointRecorderNode(Node):

    def __init__(self):
        super().__init__('waypoint_recorder_node')

        # 参数
        default_yaml = os.path.join(
            get_package_share_directory('pi5_robot_description'),
            'config', 'named_waypoints.yaml')
        self.declare_parameter('waypoints_file', default_yaml)
        self.waypoints_file = self.get_parameter('waypoints_file').value

        # 状态
        self._amcl_pose = None
        self._lock = threading.Lock()
        self.waypoints = {}

        # 加载已有目标点
        self._load_waypoints()

        # 订阅
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)
        self.create_subscription(
            String, '/waypoint_record', self._on_record_cmd, 10)
        self.create_subscription(
            String, '/voice_command', self._on_voice_cmd, 10)

        # 发布
        self.pub_status = self.create_publisher(String, '/waypoint_record_status', 10)

        self.get_logger().info(
            f'目标点记录节点已启动 | 文件: {self.waypoints_file}\n'
            f'  已有目标点: {list(self.waypoints.keys())}\n'
            f'  用法: ros2 topic pub --once /waypoint_record std_msgs/String "data: \'夹取点\'"')

    def _on_amcl(self, msg):
        with self._lock:
            self._amcl_pose = msg.pose.pose

    def _on_record_cmd(self, msg):
        """处理记录指令"""
        name = msg.data.strip()
        if not name:
            return
        self._record_waypoint(name)

    def _on_voice_cmd(self, msg):
        """处理语音指令，提取"记录xxx"模式"""
        try:
            import json
            cmd = json.loads(msg.data)
            func = cmd.get('function', '')
            params = cmd.get('parameters', {})
            if func == 'save_waypoint':
                name = params.get('name', '')
                if name:
                    self._record_waypoint(name)
        except Exception:
            # 尝试简单字符串匹配
            text = msg.data.strip()
            if '记录' in text:
                for keyword in ['夹取点', '投放点', '夹取', '投放']:
                    if keyword in text:
                        name = '夹取点' if '夹取' in keyword else '投放点'
                        self._record_waypoint(name)
                        return

    def _record_waypoint(self, name):
        """记录当前位置为目标点"""
        with self._lock:
            if self._amcl_pose is None:
                self._speak('尚未获取到定位信息，请等待定位完成')
                return
            pose = self._amcl_pose

        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self.waypoints[name] = {
            'x': round(x, 4),
            'y': round(y, 4),
            'yaw': round(yaw, 4),
        }
        self._save_waypoints()

        self.get_logger().info(
            f'✅ 已记录 {name}: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°')
        self._speak(f'已记录{name}，坐标({x:.2f}, {y:.2f})')

    def _load_waypoints(self):
        try:
            if os.path.exists(self.waypoints_file):
                with open(self.waypoints_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                if data and 'waypoints' in data:
                    self.waypoints = dict(data['waypoints'])
        except Exception as e:
            self.get_logger().warn(f'加载目标点失败: {e}')

    def _save_waypoints(self):
        try:
            data = {'waypoints': self.waypoints}
            os.makedirs(os.path.dirname(self.waypoints_file), exist_ok=True)
            with open(self.waypoints_file, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, allow_unicode=True, default_flow_style=False)
            self.get_logger().info(f'目标点已保存到 {self.waypoints_file}')
        except Exception as e:
            self.get_logger().error(f'保存目标点失败: {e}')

    def _speak(self, text):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)
        self.get_logger().info(f'📢 {text}')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
