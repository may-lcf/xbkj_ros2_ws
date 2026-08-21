#!/usr/bin/env python3
"""
导航执行节点 (Nav Executor) — 区域检测版

订阅 /voice_command，执行多点区域导航。
到达每个区域后通过 YOLO 检测画面中的物品并语音播报。
启动时发送机械臂观察位指令。

用法：
  ros2 run pi5_robot_description nav_executor_node
"""

import json
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


# ═══════════════════════════════════════════════════════════════════════════════
#  区域配置
# ═══════════════════════════════════════════════════════════════════════════════

# 区域对应的目标点名称（必须与 named_waypoints.yaml 一致）
ZONE_NAMES = {'水果区', '饮品区', '足球区'}

# YOLO 物体中文名
OBJECTS_CN = {
    'orange': '橘子', 'pear': '梨', 'pomegranate': '石榴', 'apple': '苹果',
    'milk': '牛奶', 'coffee': '咖啡', 'iced_black_tea': '冰红茶',
    'cocktail': '鸡尾酒', 'football': '足球', 'goal': '球门',
}

# 机械臂观察位指令（3位通道号格式，适配 servo_node）
ARM_OBSERVE_CMD = '{#000P1550T1000!#001P1800T1000!#002P2000T1000!#003P0800T1000!#004P1500T1000!}'


class NavExecutorNode(Node):
    """语音导航执行节点（区域检测版）"""

    def __init__(self):
        super().__init__('nav_executor_node')

        # ========== 参数 ==========
        default_yaml = os.path.join(
            get_package_share_directory('pi5_robot_description'),
            'config', 'named_waypoints.yaml')
        self.declare_parameter('waypoints_file', default_yaml)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('wait_time', 3.0)   # 到达后等待检测的时间
        self.declare_parameter('max_retries', 1)

        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.frame_id = self.get_parameter('frame_id').value
        self.wait_time = self.get_parameter('wait_time').value
        self.max_retries = self.get_parameter('max_retries').value

        # ========== 状态 ==========
        self.waypoints = {}          # {name: (x, y, yaw)}
        self.nav_targets = []        # 当前导航目标列表
        self.nav_index = -1
        self.is_navigating = False
        self.retry_count = 0
        self._goal_handle = None
        self._amcl_pose = None

        # YOLO 检测结果
        self.zone_detections = {}    # {zone: [物体名, ...]}
        self._det_lock = threading.Lock()

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
        self.sub_yolo = self.create_subscription(
            String, '/yolo/zone_detections', self._on_yolo_detections, 10)

        # ========== 发布 ==========
        self.pub_speak = self.create_publisher(String, 'speak_text', 10)

        # ========== 发送机械臂观察位 ==========
        self._send_arm_observe()

        self.get_logger().info(
            f'导航执行节点已启动 | 目标点: {list(self.waypoints.keys())}')

    # ================================================================
    #  机械臂控制
    # ================================================================

    def _send_arm_observe(self):
        """启动时发送机械臂观察位指令（直接写串口）"""
        import subprocess
        def _send():
            try:
                cmd = ARM_OBSERVE_CMD
                subprocess.run(
                    ['bash', '-c', f"echo '{cmd}' > /dev/ttyARM"],
                    timeout=3, check=True)
                self.get_logger().info('已发送机械臂观察位指令')
                self._speak('机械臂已就绪，可以开始导航')
            except Exception as e:
                self.get_logger().warn(f'机械臂指令发送失败: {e}')
                self._speak('机械臂指令发送失败')
        threading.Timer(2.0, _send).start()

    # ================================================================
    #  YOLO 检测结果
    # ================================================================

    def _on_yolo_detections(self, msg: String):
        """接收 YOLO 区域检测结果"""
        try:
            data = json.loads(msg.data)
            with self._det_lock:
                self.zone_detections = data
        except Exception:
            pass

    def _get_zone_objects(self, zone_name):
        """获取指定区域当前检测到的物体中文名列表"""
        with self._det_lock:
            objects = self.zone_detections.get(zone_name, [])
            return [OBJECTS_CN.get(obj, obj) for obj in objects]

    # ================================================================
    #  目标点文件读写
    # ================================================================

    def _load_waypoints(self):
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
                self.get_logger().warn(f'目标点文件不存在: {self.waypoints_file}')
        except Exception as e:
            self.get_logger().error(f'加载目标点失败: {e}')

    def _save_waypoints(self):
        try:
            data = {'waypoints': {}}
            for name, (x, y, yaw) in self.waypoints.items():
                data['waypoints'][name] = {
                    'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4)}
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
        self._amcl_pose = msg.pose.pose

    # ================================================================
    #  语音指令处理
    # ================================================================

    def _on_voice_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except Exception:
            try:
                cmd = eval(msg.data)
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

    def _handle_navigate(self, params):
        targets = params.get('targets', [])
        if not targets:
            self._speak('没有指定目标区域')
            return
        if self.is_navigating:
            self.get_logger().warn(f'导航中，忽略新指令: {targets}')
            return

        missing = [t for t in targets if t not in self.waypoints]
        if missing:
            self._speak(f'未找到区域：{"、".join(missing)}，请先记录这些区域')
            return

        self.nav_targets = targets
        self.nav_index = 0
        self.retry_count = 0
        self.is_navigating = True

        # 语音回复
        if len(targets) == 1:
            reply = f'好的，去{targets[0]}看看'
        else:
            parts = [f'去{targets[0]}'] + [f'再去{t}' for t in targets[1:]]
            reply = '好的，' + '，'.join(parts)
        self._speak(reply)

        threading.Timer(3.0, self._navigate_next).start()

    def _handle_save_waypoint(self, params):
        name = params.get('name', '')
        if not name:
            self._speak('请指定区域名称')
            return
        if self._amcl_pose is None:
            self._speak('尚未获取到定位信息，请等待定位完成')
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
        self._speak(f'已记录{name}坐标')

    def _handle_clear_waypoints(self):
        self.waypoints.clear()
        self._save_waypoints()
        self._speak('已清空所有区域')

    def _handle_list_waypoints(self):
        if not self.waypoints:
            self._speak('还没有记录任何区域')
            return
        names = '、'.join(self.waypoints.keys())
        self._speak(f'已记录的区域有：{names}')

    def _handle_stop(self):
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
        if not self.is_navigating:
            return
        if self.nav_index >= len(self.nav_targets):
            self.is_navigating = False
            self.get_logger().info('所有区域已巡视完毕')
            self._speak('所有区域已巡视完毕')
            return

        name = self.nav_targets[self.nav_index]
        x, y, yaw = self.waypoints[name]
        total = len(self.nav_targets)

        self.get_logger().info(
            f'[{self.nav_index + 1}/{total}] 前往 {name}: '
            f'({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)')

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server 未就绪')
            self._speak('导航服务未就绪')
            self.is_navigating = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
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
        result = future.result()
        status = result.status
        name = self.nav_targets[self.nav_index]

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✅ 到达 {name}')
            self.retry_count = 0

            # 到达区域后，等待检测并播报
            if name in ZONE_NAMES:
                self._speak(f'已到达{name}，正在识别')
                # 等待 YOLO 检测结果稳定
                threading.Timer(2.0, self._announce_zone_objects, args=[name]).start()
            else:
                # 非区域目标（如原点），直接播报
                self._speak(f'已到达{name}')
                self._proceed_next()
        else:
            self.get_logger().warn(f'导航失败，状态码: {status}')
            self._handle_nav_failure()

    def _announce_zone_objects(self, zone_name):
        """播报区域检测到的物体"""
        objects = self._get_zone_objects(zone_name)

        if objects:
            obj_text = '、'.join(objects)
            announcement = f'{zone_name}检测到：{obj_text}'
            self.get_logger().info(f'📢 {announcement}')
            self._speak(announcement)
        else:
            self.get_logger().info(f'📢 {zone_name}未检测到已知物体')
            self._speak(f'{zone_name}没有检测到已知物品')

        # 播报完后等待一段时间再前往下一个区域
        threading.Timer(3.0, self._proceed_next).start()

    def _proceed_next(self):
        """前往下一个目标"""
        self.nav_index += 1
        self._navigate_next()

    def _handle_nav_failure(self):
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
