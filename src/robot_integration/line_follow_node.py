#!/usr/bin/env python3
"""
line_follow_node.py — 黄线巡迹节点

功能：
  1. 订阅 RGB + 深度图像（Aurora 930）
  2. HSV 色彩空间提取道路右侧黄线
  3. 深度信息过滤地面平面
  4. 分行扫描计算黄线位置
  5. PID 控制：保持黄线在图像右侧固定位置

黄线在道路右侧 → 小车沿黄线行驶
黄线偏左 = 小车偏右 → 右转修正
黄线偏右 = 小车偏左 → 左转修正

机械臂固定姿态：{#0P1550T1000!#1P1800T1000!#2P2000T1000!#3P0800T1000!#4P1500T1000!}

话题：
  订阅:
    /aurora/rgb/image_raw       (sensor_msgs/Image, bgr8)
    /aurora/depth/image_raw     (sensor_msgs/Image, mono16, mm)
    /line_follow/control        (std_msgs/String) — start/stop/reset
  发布:
    /cmd_vel                    (geometry_msgs/Twist)
    /arm_command                (std_msgs/String)
    /line_follow/status         (std_msgs/String) — JSON 状态
    /line_follow/debug_image    (sensor_msgs/Image)

用法：
  ros2 launch robot_integration line_follow.launch.py
  ros2 launch robot_integration line_follow.launch.py enable_camera:=true
  ros2 topic pub --once /line_follow/control std_msgs/String '{data: "start"}'
"""

import os
import sys
import time
import threading
import json
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge

try:
    from message_filters import Subscriber as MfSub, ApproximateTimeSynchronizer
    HAS_MESSAGE_FILTERS = True
except ImportError:
    HAS_MESSAGE_FILTERS = False


# ── 状态定义 ──
STATE_IDLE = 'IDLE'
STATE_INIT_ARM = 'INIT_ARM'
STATE_FOLLOWING = 'FOLLOWING'
STATE_STOPPED = 'STOPPED'

# ── 机械臂观察姿态（固定）──
OBSERVE_ARM_CMD = "{#0P1550T1000!#1P1800T1000!#2P2000T1000!#3P0800T1000!#4P1500T1000!}"


# ═══════════════════════════════════════════════════════════════════════════════
#  PID 控制器
# ═══════════════════════════════════════════════════════════════════════════════

class PIDController:
    """PID 控制器（参照 depth_color_track_node.py）"""

    def __init__(self, kp, ki, kd, integral_limit=1.0):
        self.target = 0.0
        self.last_error = 0.0
        self.sum_error = 0.0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit

    def compute(self, actual):
        error = self.target - actual
        self.sum_error += error
        # 积分限幅
        self.sum_error = max(-self.integral_limit,
                             min(self.integral_limit, self.sum_error))
        output = (self.kp * error +
                  self.ki * self.sum_error +
                  self.kd * (error - self.last_error))
        self.last_error = error
        return output

    def reset(self):
        self.last_error = 0.0
        self.sum_error = 0.0


# ═══════════════════════════════════════════════════════════════════════════════
#  LineFollowNode
# ═══════════════════════════════════════════════════════════════════════════════

class LineFollowNode(Node):

    def __init__(self):
        super().__init__('line_follow_node')

        # ── 状态 ──
        self.state = STATE_IDLE
        self.mission_active = False
        self.lost_count = 0

        # ── 图像缓存 ──
        self.latest_rgb = None
        self.latest_depth = None
        self._frame_lock = threading.Lock()
        self.depth_ready = threading.Event()
        self.rgb_ready = threading.Event()

        self.bridge = CvBridge()

        # ── 参数声明 ──
        self._declare_parameters()

        # ── PID 控制器（target = 标定工具的目标误差值）──
        kp = self.get_parameter('pid_kp').value
        ki = self.get_parameter('pid_ki').value
        kd = self.get_parameter('pid_kd').value
        il = self.get_parameter('integral_limit').value
        self.pid = PIDController(kp, ki, kd, il)
        self.pid.target = self.get_parameter('line_target_error').value

        # ── QoS（与 Aurora 930 发布者匹配：RELIABLE）──
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ── 图像订阅（同步 RGB + 深度）──
        if HAS_MESSAGE_FILTERS:
            rgb_sub = MfSub(self, Image, '/aurora/rgb/image_raw', qos)
            depth_sub = MfSub(self, Image, '/aurora/depth/image_raw', qos)
            self._sync = ApproximateTimeSynchronizer(
                [rgb_sub, depth_sub], queue_size=5, slop=0.1)
            self._sync.registerCallback(self._synced_callback)
            self.get_logger().info('[LineFollow] 使用 message_filters 同步 RGB+Depth')
        else:
            self.create_subscription(Image, '/aurora/rgb/image_raw',
                                     self._rgb_callback, qos)
            self.create_subscription(Image, '/aurora/depth/image_raw',
                                     self._depth_callback, qos)
            self.get_logger().info('[LineFollow] message_filters 不可用，使用独立订阅')

        # ── 外部控制订阅 ──
        self.create_subscription(
            String, '/line_follow/control',
            self._control_callback, 10)

        # ── 发布器 ──
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_cmd_pub = self.create_publisher(String, '/arm_command', 10)
        self.status_pub = self.create_publisher(String, '/line_follow/status', 10)
        self.debug_pub = self.create_publisher(Image, '/line_follow/debug_image', 10)

        # ── 控制循环（10Hz）──
        self.control_timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            '\033[1;36m[LineFollow]\033[0m 循迹节点已启动\n'
            '  发送 /line_follow/control 话题启动任务:\n'
            '    ros2 topic pub --once /line_follow/control std_msgs/String \'{data: "start"}\'\n'
            '  或设置 auto_start:=true 自动启动'
        )

        # ── 自动启动 ──
        if self.get_parameter('auto_start').value:
            self.get_logger().info('[LineFollow] auto_start=true，3秒后启动...')
            threading.Timer(3.0, self.start_mission).start()

    # ═══════════════════════════════════════════════════════════════════════════
    #  参数声明
    # ═══════════════════════════════════════════════════════════════════════════

    def _declare_parameters(self):
        # HSV 黄线阈值
        self.declare_parameter('hsv_h_min', 0)
        self.declare_parameter('hsv_h_max', 88)
        self.declare_parameter('hsv_s_min', 61)
        self.declare_parameter('hsv_s_max', 199)
        self.declare_parameter('hsv_v_min', 200)
        self.declare_parameter('hsv_v_max', 255)

        # 深度地面过滤
        self.declare_parameter('ground_depth_min_mm', 150)
        self.declare_parameter('ground_depth_max_mm', 600)

        # ROI
        self.declare_parameter('roi_y_start', 176)
        self.declare_parameter('roi_y_end', 400)
        self.declare_parameter('roi_x_left', 44)
        self.declare_parameter('roi_x_right', 297)

        # 形态学
        self.declare_parameter('morph_kernel_size', 5)
        self.declare_parameter('morph_open_iter', 2)
        self.declare_parameter('morph_close_iter', 2)

        # 黄线跟踪
        self.declare_parameter('scan_rows', 10)
        self.declare_parameter('min_line_pixels', 5)
        self.declare_parameter('line_target_error', -0.6)  # 目标误差（与标定工具一致）

        # PID
        self.declare_parameter('pid_kp', 0.30)
        self.declare_parameter('pid_ki', 0.01)
        self.declare_parameter('pid_kd', 0.15)
        self.declare_parameter('max_steering', 0.10)
        self.declare_parameter('integral_limit', 1.0)

        # 运动
        self.declare_parameter('move_speed', 0.15)

        # 道路丢失
        self.declare_parameter('max_lost_frames', 30)

        # 调试
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('auto_start', False)

    # ═══════════════════════════════════════════════════════════════════════════
    #  图像回调
    # ═══════════════════════════════════════════════════════════════════════════

    def _synced_callback(self, rgb_msg, depth_msg):
        """RGB + 深度同步回调"""
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(
                depth_msg.height, depth_msg.width)
            with self._frame_lock:
                self.latest_rgb = rgb
                self.latest_depth = depth
                if not self.depth_ready.is_set():
                    self.depth_ready.set()
                if not self.rgb_ready.is_set():
                    self.rgb_ready.set()
        except Exception as e:
            self.get_logger().error(f'同步回调错误: {e}')

    def _rgb_callback(self, msg):
        """独立 RGB 回调"""
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._frame_lock:
                self.latest_rgb = rgb
                if not self.rgb_ready.is_set():
                    self.rgb_ready.set()
        except Exception as e:
            self.get_logger().error(f'RGB 回调错误: {e}')

    def _depth_callback(self, msg):
        """独立深度回调"""
        try:
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width)
            with self._frame_lock:
                self.latest_depth = depth
                if not self.depth_ready.is_set():
                    self.depth_ready.set()
        except Exception as e:
            self.get_logger().error(f'深度回调错误: {e}')

    def _control_callback(self, msg):
        """外部控制指令"""
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            self.start_mission()
        elif cmd == 'stop':
            self.stop_mission()
        elif cmd == 'reset':
            self.reset_state()

    # ═══════════════════════════════════════════════════════════════════════════
    #  核心算法：黄线检测 + 深度地面过滤
    # ═══════════════════════════════════════════════════════════════════════════

    def detect_line(self, rgb, depth):
        """
        检测黄线位置，计算横向偏差

        黄线在道路左侧，小车沿黄线行驶：
        - 黄线在目标位置（左侧20%处）→ error=0 → 直走
        - 黄线偏右（小车偏左）→ error>0 → steering>0 → 右转跟上 ✓
        - 黄线偏左（小车偏右）→ error<0 → steering<0 → 左转跟上 ✓

        Returns:
            dict: {
                'line_mask': np.ndarray,
                'line_points': list of (cx, y),
                'lateral_error': float,
                'line_found': bool,
                'line_ratio': float,
            }
        """
        h, w = rgb.shape[:2]

        # ── 1. HSV 黄线提取 ──
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        h_min = self.get_parameter('hsv_h_min').value
        h_max = self.get_parameter('hsv_h_max').value
        s_min = self.get_parameter('hsv_s_min').value
        s_max = self.get_parameter('hsv_s_max').value
        v_min = self.get_parameter('hsv_v_min').value
        v_max = self.get_parameter('hsv_v_max').value

        lower = np.array([h_min, s_min, v_min], dtype=np.uint8)
        upper = np.array([h_max, s_max, v_max], dtype=np.uint8)
        line_mask = cv2.inRange(hsv, lower, upper)

        # ── 2. 深度地面过滤 ──
        if depth is not None:
            g_min = self.get_parameter('ground_depth_min_mm').value
            g_max = self.get_parameter('ground_depth_max_mm').value
            ground_mask = ((depth >= g_min) & (depth <= g_max)).astype(np.uint8) * 255
            line_mask = cv2.bitwise_and(line_mask, ground_mask)

        # ── 3. ROI 裁剪（上下+左右）──
        roi_y_start = self.get_parameter('roi_y_start').value
        roi_y_end = self.get_parameter('roi_y_end').value
        roi_x_left = self.get_parameter('roi_x_left').value
        roi_x_right = self.get_parameter('roi_x_right').value
        roi_y_start = min(roi_y_start, h - 10)
        roi_y_end = min(roi_y_end, h - 1)
        roi_x_left = max(0, min(roi_x_left, w - 10))
        roi_x_right = max(roi_x_left + 10, min(roi_x_right, w))
        line_mask[:roi_y_start, :] = 0
        line_mask[roi_y_end:, :] = 0
        line_mask[:, :roi_x_left] = 0
        line_mask[:, roi_x_right:] = 0

        # ── 4. 形态学处理 ──
        k_size = self.get_parameter('morph_kernel_size').value
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k_size, k_size))
        open_iter = self.get_parameter('morph_open_iter').value
        close_iter = self.get_parameter('morph_close_iter').value
        line_mask = cv2.morphologyEx(line_mask, cv2.MORPH_OPEN, kernel,
                                     iterations=open_iter)
        line_mask = cv2.morphologyEx(line_mask, cv2.MORPH_CLOSE, kernel,
                                     iterations=close_iter)

        # ── 5. 分行扫描找黄线位置 ──
        num_rows = self.get_parameter('scan_rows').value
        min_pixels = self.get_parameter('min_line_pixels').value
        line_points = []

        rows = np.linspace(roi_y_start, roi_y_end, num_rows, dtype=int)
        for y in rows:
            row = line_mask[y, :]
            pixels = np.where(row > 0)[0]
            if len(pixels) >= min_pixels:
                cx = int(np.mean(pixels))
                line_points.append((cx, y))

        # ── 6. 横向偏差（与标定工具一致：以图像中心为基准）──
        image_cx = w / 2.0
        lateral_error = 0.0
        line_found = False

        if line_points:
            total_weight = 0.0
            weighted_error = 0.0
            for i, (cx, y) in enumerate(line_points):
                weight = (i + 1)  # 近处权重高
                error = (cx - image_cx) / image_cx  # 与标定工具一致
                weighted_error += error * weight
                total_weight += weight
            lateral_error = weighted_error / total_weight if total_weight > 0 else 0.0
            line_found = True

        line_ratio = np.sum(line_mask > 0) / (line_mask.size) if line_mask.size > 0 else 0.0

        return {
            'line_mask': line_mask,
            'line_points': line_points,
            'lateral_error': lateral_error,
            'line_found': line_found,
            'line_ratio': line_ratio,
            'roi': (roi_x_left, roi_x_right, roi_y_start, roi_y_end),
        }

    # ═══════════════════════════════════════════════════════════════════════════
    #  控制循环
    # ═══════════════════════════════════════════════════════════════════════════

    def _control_loop(self):
        """主控制循环（10Hz）"""
        if not self.mission_active:
            return

        # 获取最新帧
        with self._frame_lock:
            rgb = self.latest_rgb
            depth = self.latest_depth

        if rgb is None:
            return

        # 黄线检测
        result = self.detect_line(rgb, depth)

        # 发布状态
        self._publish_status(result)

        # 调试图像
        if self.get_parameter('publish_debug_image').value:
            self._publish_debug_image(rgb, result)

        # 状态机
        if self.state == STATE_INIT_ARM:
            pass  # 等待机械臂到位

        elif self.state == STATE_FOLLOWING:
            self._handle_following(result)

    def _handle_following(self, result):
        """黄线巡迹控制"""
        cmd = Twist()
        max_steering = self.get_parameter('max_steering').value
        move_speed = self.get_parameter('move_speed').value
        max_lost = self.get_parameter('max_lost_frames').value

        if result['line_found']:
            # 黄线检测成功
            self.lost_count = 0

            # PID 计算转向（target=-0.6，error偏大→输出负→右转）
            steering = self.pid.compute(result['lateral_error'])
            steering = max(-max_steering, min(max_steering, steering))

            cmd.linear.x = move_speed
            cmd.angular.z = steering

            self.get_logger().info(
                f'[LineFollow] error={result["lateral_error"]:+.3f} '
                f'steering={steering:+.3f} speed={move_speed} '
                f'line={result["line_ratio"]:.1%}'
            )
        else:
            # 黄线丢失
            self.lost_count += 1
            self.get_logger().warn(
                f'[LineFollow] 黄线丢失 ({self.lost_count}/{max_lost})')

            if self.lost_count >= max_lost:
                self.get_logger().error('[LineFollow] 黄线持续丢失，停止任务')
                self.stop_mission()
                return

            # 丢失时慢速直行（尝试找回黄线）
            cmd.linear.x = move_speed * 0.5
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

    # ═══════════════════════════════════════════════════════════════════════════
    #  任务控制
    # ═══════════════════════════════════════════════════════════════════════════

    def start_mission(self):
        """启动循迹任务"""
        if self.mission_active:
            self.get_logger().warn('[LineFollow] 任务已在运行')
            return

        self.get_logger().info(
            '\033[1;36m[LineFollow]\033[0m 启动循迹任务\n'
            '  1. 停止小车...\n'
            '  2. 设置机械臂观察姿态...\n'
            '  3. 等待图像就绪...\n'
            '  4. 开始循迹...'
        )

        self.mission_active = True
        self.state = STATE_INIT_ARM
        self.lost_count = 0
        self.pid.reset()

        # 停车
        self._stop_car()

        # 新线程设置机械臂
        def init_arm():
            time.sleep(0.5)
            self._set_arm_observe_pose()
            self.get_logger().info('[LineFollow] 等待机械臂到位...')
            time.sleep(5.0)
            self._arm_ready()

        threading.Thread(target=init_arm, daemon=True).start()

    def _arm_ready(self):
        """机械臂到位回调"""
        if not self.mission_active:
            return

        # 等待图像就绪
        if not self.depth_ready.wait(timeout=10.0):
            self.get_logger().error('[LineFollow] 深度图未就绪，任务取消')
            self.stop_mission()
            return

        if not self.rgb_ready.wait(timeout=5.0):
            self.get_logger().error('[LineFollow] RGB 图像未就绪，任务取消')
            self.stop_mission()
            return

        self.get_logger().info(
            '\033[1;32m[LineFollow]\033[0m 就绪，开始循迹！')
        self.state = STATE_FOLLOWING

    def stop_mission(self):
        """停止任务"""
        self.get_logger().info('\033[1;31m[LineFollow]\033[0m 停止循迹任务')
        self.mission_active = False
        self.state = STATE_STOPPED
        self._stop_car()

    def reset_state(self):
        """重置状态"""
        self.get_logger().info('[LineFollow] 重置状态')
        self._stop_car()
        self.state = STATE_IDLE
        self.mission_active = False
        self.lost_count = 0
        self.pid.reset()

    def _stop_car(self):
        """停车"""
        self.cmd_vel_pub.publish(Twist())

    def _set_arm_observe_pose(self):
        """设置机械臂观察姿态"""
        self.get_logger().info(
            f'[LineFollow] 设置机械臂观察姿态: {OBSERVE_ARM_CMD}')
        cmd = String()
        cmd.data = OBSERVE_ARM_CMD
        self.arm_cmd_pub.publish(cmd)

    # ═══════════════════════════════════════════════════════════════════════════
    #  状态发布
    # ═══════════════════════════════════════════════════════════════════════════

    def _publish_status(self, result):
        """发布状态 JSON"""
        status = {
            'state': self.state,
            'line_found': result['line_found'],
            'lateral_error': float(result['lateral_error']),
            'line_ratio': float(result['line_ratio']),
            'lost_count': self.lost_count,
            'line_points_count': len(result['line_points']),
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════════════════
    #  调试图像
    # ═══════════════════════════════════════════════════════════════════════════

    def _publish_debug_image(self, rgb, result):
        """发布调试图像"""
        try:
            overlay = rgb.copy()
            h, w = overlay.shape[:2]

            # 黄线掩码半透明叠加（黄色）
            yellow_layer = np.zeros_like(overlay)
            yellow_layer[:, :, 0] = 0    # B
            yellow_layer[:, :, 1] = 255  # G
            yellow_layer[:, :, 2] = 255  # R
            mask_bool = result['line_mask'] > 0
            blended = cv2.addWeighted(rgb, 0.5, yellow_layer, 0.5, 0)
            overlay[mask_bool] = blended[mask_bool]

            # ROI 矩形框（青色）
            roi_l, roi_r, roi_s, roi_e = result['roi']
            cv2.rectangle(overlay, (roi_l, roi_s), (roi_r, roi_e), (255, 255, 0), 1)

            # 目标位置参考线（绿色竖线，error=-0.6的位置）
            target_error = self.get_parameter('line_target_error').value
            target_x = int((target_error + 1.0) * w / 2.0)  # error→像素位置
            cv2.line(overlay, (target_x, 0), (target_x, h), (0, 255, 0), 1)

            # 黄线点和连线（红色）
            pts = result['line_points']
            for cx, y in pts:
                cv2.circle(overlay, (cx, y), 5, (0, 0, 255), -1)
            if len(pts) > 1:
                for i in range(len(pts) - 1):
                    cv2.line(overlay, pts[i], pts[i + 1], (0, 0, 255), 2)

            # 横向偏差箭头
            if result['line_found'] and pts:
                total_w = 0
                wx = 0
                for i, (cx, y) in enumerate(pts):
                    wt = i + 1
                    wx += cx * wt
                    total_w += wt
                avg_cx = int(wx / total_w) if total_w > 0 else target_x
                arrow_y = roi_e - 30
                cv2.arrowedLine(overlay, (target_x, arrow_y), (avg_cx, arrow_y),
                               (0, 255, 255), 3, tipLength=0.3)

            # 信息文字
            cv2.putText(overlay, f"State: {self.state}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(overlay, f"Error: {result['lateral_error']:+.3f}",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(overlay, f"Line: {result['line_ratio']:.1%}",
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(overlay, f"Lost: {self.lost_count}",
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(overlay, f"Target X: {target_x}",
                       (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8'))
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_mission()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
