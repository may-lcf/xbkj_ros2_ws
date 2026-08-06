#!/usr/bin/env python3
"""
导航执行节点 (Nav Executor)

订阅 /voice_command (JSON String)，执行导航相关指令。
发布 /speak_text (String) 给 TTS 节点语音回复。

支持的指令：
  navigate        — 多点顺序导航 {"targets": ["A", "B", "C"]}
  save_waypoint   — 记录当前位置为命名目标点 {"name": "A"}
  clear_waypoints — 清空所有命名目标点 {}
  list_waypoints  — 列出所有已记录的目标点 {}
  stop_navigation — 停止当前导航 {}

用法：
  ros2 run pi5_robot_description nav_executor_node
  ros2 run pi5_robot_description nav_executor_node --ros-args -p waypoints_file:=/path/to/named_waypoints.yaml
"""

import math
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

import yaml


class NavExecutorNode(Node):
    """语音导航执行节点"""

    def __init__(self):
        super().__init__('nav_executor_node')

        # ========== 参数 ==========
        default_yaml = os.path.join(
            get_package_share_directory('pi5_robot_description'),
            'config', 'named_waypoints.yaml')
        self.declare_parameter('waypoints_file', default_yaml)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('wait_time', 2.0)
        self.declare_parameter('max_retries', 1)

        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.frame_id = self.get_parameter('frame_id').value
        self.wait_time = self.get_parameter('wait_time').value
        self.max_retries = self.get_parameter('max_retries').value

        # ========== 状态 ==========
        self.waypoints = {}          # {name: (x, y, yaw)}
        self.nav_targets = []        # 当前导航目标列表
        self.nav_index = -1          # 当前导航索引
        self.is_navigating = False
        self.retry_count = 0
        self._goal_handle = None
        self._amcl_pose = None       # 最新的 AMCL 位姿

        # ========== 加载目标点 ==========
        self._load_waypoints()

        # ========== Action Client ==========
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # ========== 订阅 ==========
        self.sub_cmd = self.create_subscription(
            String, 'voice_command', self._on_voice_command, 10)
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)

        # ========== 发布 ==========
        self.pub_speak = self.create_publisher(String, 'speak_text', 10)

        self.get_logger().info(
            f'导航执行节点已启动 | 目标点: {list(self.waypoints.keys())}')

    # ================================================================
    #  目标点文件读写
    # ================================================================

    def _load_waypoints(self):
        """从 YAML 文件加载命名目标点"""
        try:
            if os.path.exists(self.waypoints_file):
                with open(self.waypoints_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                if data and 'waypoints' in data:
                    for name, pos in data['waypoints'].items():
                        self.waypoints[name] = (
                            float(pos.get('x', 0.0)),
                            float(pos.get('y', 0.0)),
                            float(pos.get('yaw', 0.0)))
                    self.get_logger().info(
                        f'加载了 {len(self.waypoints)} 个目标点: '
                        f'{list(self.waypoints.keys())}')
            else:
                self.get_logger().warn(
                    f'目标点文件不存在: {self.waypoints_file}')
        except Exception as e:
            self.get_logger().error(f'加载目标点失败: {e}')

    def _save_waypoints(self):
        """保存目标点到 YAML 文件"""
        try:
            data = {'waypoints': {}}
            for name, (x, y, yaw) in self.waypoints.items():
                data['waypoints'][name] = {
                    'x': round(x, 4),
                    'y': round(y, 4),
                    'yaw': round(yaw, 4)}
            os.makedirs(os.path.dirname(self.waypoints_file), exist_ok=True)
            with open(self.waypoints_file, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, allow_unicode=True, default_flow_style=False)
            self.get_logger().info(f'目标点已保存到 {self.waypoints_file}')
        except Exception as e:
            self.get_logger().error(f'保存目标点失败: {e}')

    # ================================================================
    #  AMCL 位姿
    # ================================================================

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        """保存最新的 AMCL 定位位姿"""
        self._amcl_pose = msg.pose.pose

    # ================================================================
    #  语音指令处理
    # ================================================================

    def _on_voice_command(self, msg: String):
        """处理来自 intent_parser 的语音指令"""
        try:
            cmd = eval(msg.data) if isinstance(msg.data, str) else msg.data
        except Exception:
            try:
                import json
                cmd = json.loads(msg.data)
            except Exception:
                self.get_logger().warn(f'无法解析指令: {msg.data}')
                return

        func = cmd.get('function', '')
        params = cmd.get('parameters', {})
        self.get_logger().info(f'收到指令: {func} {params}')

        if func == 'navigate':
            self._handle_navigate(params)
        elif func == 'save_waypoint':
            self._handle_save_waypoint(params)
        elif func == 'clear_waypoints':
            self._handle_clear_waypoints()
        elif func == 'list_waypoints':
            self._handle_list_waypoints()
        elif func == 'stop_navigation':
            self._handle_stop()
        else:
            # 不是导航指令，忽略
            pass

    def _handle_navigate(self, params):
        """处理多点导航指令"""
        targets = params.get('targets', [])
        if not targets:
            self._speak('没有指定目标点')
            return

        if self.is_navigating:
            self._speak('当前正在导航中，请先停止')
            return

        # 检查所有目标点是否存在
        missing = [t for t in targets if t not in self.waypoints]
        if missing:
            self._speak(f'未找到目标点：{"、".join(missing)}，请先记录这些点')
            return

        # 构建导航目标列表
        self.nav_targets = targets
        self.nav_index = 0
        self.retry_count = 0
        self.is_navigating = True

        # 生成语音回复
        if len(targets) == 1:
            reply = f'好的，去{targets[0]}'
        else:
            parts = [f'去{targets[0]}'] + [f'再去{t}' for t in targets[1:]]
            reply = '好的，' + '，'.join(parts)
        self._speak(reply)

        # 开始导航（在新线程中，等待 TTS 播放完再开始）
        threading.Timer(2.0, self._navigate_next).start()

    def _handle_save_waypoint(self, params):
        """记录当前位置为目标点"""
        name = params.get('name', '')
        if not name:
            self._speak('请指定目标点名称')
            return

        if self._amcl_pose is None:
            self._speak('尚未获取到定位信息，请等待 AMCL 定位完成')
            return

        x = self._amcl_pose.position.x
        y = self._amcl_pose.position.y
        q = self._amcl_pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self.waypoints[name] = (round(x, 4), round(y, 4), round(yaw, 4))
        self._save_waypoints()

        self.get_logger().info(
            f'已记录 {name}: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°')
        self._speak(f'已记录{name}点坐标')

    def _handle_clear_waypoints(self):
        """清空所有目标点"""
        self.waypoints.clear()
        self._save_waypoints()
        self._speak('已清空所有目标点')

    def _handle_list_waypoints(self):
        """列出所有目标点"""
        if not self.waypoints:
            self._speak('还没有记录任何目标点')
            return
        names = '、'.join(self.waypoints.keys())
        self._speak(f'已记录的目标点有：{names}')

    def _handle_stop(self):
        """停止当前导航"""
        if not self.is_navigating:
            self._speak('当前没有在导航')
            return
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self.is_navigating = False
        self.nav_targets = []
        self.nav_index = -1
        self._speak('已停止导航')

    # ================================================================
    #  导航逻辑
    # ================================================================

    def _navigate_next(self):
        """发送下一个导航目标"""
        if not self.is_navigating:
            return
        if self.nav_index >= len(self.nav_targets):
            self.is_navigating = False
            self.get_logger().info('所有目标点已完成')
            self._speak('所有目标点已到达')
            return

        name = self.nav_targets[self.nav_index]
        x, y, yaw = self.waypoints[name]
        total = len(self.nav_targets)

        self.get_logger().info(
            f'[{self.nav_index + 1}/{total}] 前往 {name}: '
            f'({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)')

        # 构建 PoseStamped
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 等待 action server
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server 未就绪')
            self._speak('导航服务未就绪')
            self.is_navigating = False
            return

        # 发送目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        """目标被接受或拒绝"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('目标被拒绝')
            self._handle_nav_failure()
            return

        self._goal_handle = goal_handle
        self.get_logger().info('目标已接受，正在导航...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        """导航结果"""
        result = future.result()
        status = result.status
        name = self.nav_targets[self.nav_index]

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✅ 到达 {name}')
            self.retry_count = 0

            # 语音回复
            remaining = len(self.nav_targets) - self.nav_index - 1
            if remaining > 0:
                next_name = self.nav_targets[self.nav_index + 1]
                self._speak(f'已到达{name}，正在前往{next_name}')
            else:
                self._speak(f'已到达{name}')

            # 等待后前往下一个点
            self.nav_index += 1
            timer = threading.Timer(
                self.wait_time,
                lambda: self._navigate_next() if self.is_navigating else None)
            timer.daemon = True
            timer.start()
        else:
            self.get_logger().warn(f'导航失败，状态码: {status}')
            self._handle_nav_failure()

    def _handle_nav_failure(self):
        """处理导航失败"""
        self._goal_handle = None
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.get_logger().info(f'重试 ({self.retry_count}/{self.max_retries})...')
            self._navigate_next()
        else:
            name = self.nav_targets[self.nav_index]
            self.get_logger().warn(f'跳过 {name}')
            self._speak(f'无法到达{name}，跳过')
            self.nav_index += 1
            self.retry_count = 0
            self._navigate_next()

    # ================================================================
    #  TTS
    # ================================================================

    def _speak(self, text):
        """发布语音文本给 TTS 节点"""
        msg = String()
        msg.data = text
        self.pub_speak.publish(msg)

    # ================================================================
    #  清理
    # ================================================================

    def destroy_node(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
