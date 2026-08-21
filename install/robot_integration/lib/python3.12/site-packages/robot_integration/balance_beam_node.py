#!/usr/bin/env python3
"""
balance_beam_node.py — 独木桥导航节点（丝滑版）

核心改进：
  1. 持续前进，不停车（丝滑）
  2. 边前进边微调方向（像人走独木桥）
  3. 只有画面中间大部分是深坑才算桥面丢失
"""

import os
import sys
import time
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge

# ── 状态定义 ──
STATE_IDLE = 'IDLE'
STATE_INIT_ARM = 'INIT_ARM'
STATE_CROSSING = 'CROSSING'        # 过桥中（持续前进+微调）
STATE_STOPPED = 'STOPPED'

# ── 观察姿态 ──
OBSERVE_ARM_CMD = "{#0P1550T1000!#1P1200T1000!#2P1600T1000!#3P0900T1000!#4P1500T1000!}"


class BalanceBeamNode(Node):

    def __init__(self):
        super().__init__('balance_beam_node')

        self.state = STATE_IDLE
        self.mission_active = False
        self.lateral_error = 0.0
        self.bridge_detected = False

        # 平滑滤波
        self.error_history = []
        self.prev_error = 0.0  # 上一次误差（用于D项）

        self.latest_depth = None
        self._depth_lock = threading.Lock()
        self.depth_ready = threading.Event()

        self.bridge = CvBridge()
        self._declare_parameters()

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Image, '/aurora/depth/image_raw', self._depth_callback, qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_cmd_pub = self.create_publisher(String, '/arm_command', 10)
        self.status_pub = self.create_publisher(String, '/balance_beam/status', 10)
        self.debug_pub = self.create_publisher(Image, '/balance_beam/debug_image', 10)
        self.create_subscription(String, '/balance_beam/control', self._control_callback, 10)

        self.control_timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            '\033[1;36m[BalanceBeam]\033[0m 已启动\n'
            '  ros2 topic pub --once /balance_beam/control std_msgs/String \'{data: "start"}\''
        )

        if self.get_parameter('auto_start').value:
            threading.Timer(3.0, self.start_mission).start()

    def _declare_parameters(self):
        self.declare_parameter('roi_y_start', 200)
        self.declare_parameter('roi_y_end', 400)
        self.declare_parameter('move_speed', 0.20)           # 前进速度
        self.declare_parameter('steering_kp', 0.15)          # P增益（比例响应）
        self.declare_parameter('steering_kd', 0.10)          # D增益（抑制振荡）
        self.declare_parameter('max_steering', 0.06)         # 最大转向限制
        self.declare_parameter('lost_threshold', 0.7)        # 丢失阈值
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('auto_start', False)

    def _depth_callback(self, msg: Image):
        try:
            if not hasattr(self, '_depth_cb_logged'):
                self._depth_cb_logged = True
                self.get_logger().info(f'[BalanceBeam] 收到深度图: {msg.width}x{msg.height}')
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            with self._depth_lock:
                self.latest_depth = depth
                if not self.depth_ready.is_set():
                    self.depth_ready.set()
        except Exception as e:
            self.get_logger().error(f'深度图错误: {e}')

    def _control_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            self.start_mission()
        elif cmd == 'stop':
            self.stop_mission()
        elif cmd == 'reset':
            self.reset_state()

    # ═══════════════════════════════════════════════════════════════════════════
    #  深度信息打印（1秒一次）
    # ═══════════════════════════════════════════════════════════════════════════

    def _print_depth_info(self):
        """每秒打印一次中间区域深度信息"""
        with self._depth_lock:
            depth = self.latest_depth

        if depth is None:
            return

        h, w = depth.shape
        roi_y_start = self.get_parameter('roi_y_start').value
        roi_y_end = self.get_parameter('roi_y_end').value

        roi_y_start = min(roi_y_start, h - 2)
        roi_y_end = min(roi_y_end, h)
        roi = depth[roi_y_start:roi_y_end, :]

        # 中间区域（中间50%）
        center_region = roi[:, w//4:3*w//4]

        # 统计
        total_pixels = center_region.size
        valid_pixels = np.logical_and(center_region >= 150, center_region <= 800)
        valid_count = np.sum(valid_pixels)
        valid_ratio = valid_count / total_pixels

        # 深度值统计（仅有效值）
        valid_depths = center_region[valid_pixels]
        if len(valid_depths) > 0:
            depth_min = int(valid_depths.min())
            depth_max = int(valid_depths.max())
            depth_mean = int(valid_depths.mean())
            depth_median = int(np.median(valid_depths))
        else:
            depth_min = depth_max = depth_mean = depth_median = 0

        # 无效深度分类
        too_close = np.sum(center_region < 150)  # 太近（可能相机被遮挡）
        too_far = np.sum(center_region > 800)    # 太远（深坑/无物体）
        zero_depth = np.sum(center_region == 0)   # 完全无效

        self.get_logger().info(
            f'\033[1;36m[Depth]\033[0m '
            f'中心区域: 有效率={valid_ratio:.2f} '
            f'深度范围=[{depth_min}, {depth_max}]mm '
            f'均值={depth_mean} 中位数={depth_median} '
            f'太近={too_close} 太远={too_far} 零值={zero_depth}'
        )

    # ═══════════════════════════════════════════════════════════════════════════
    #  桥面检测（丝滑版：同时检测桥面和判断是否真的丢失）
    # ═══════════════════════════════════════════════════════════════════════════

    def detect_bridge(self, depth_image):
        roi_y_start = self.get_parameter('roi_y_start').value
        roi_y_end = self.get_parameter('roi_y_end').value

        h, w = depth_image.shape
        cx = w // 2

        roi_y_start = min(roi_y_start, h - 2)
        roi_y_end = min(roi_y_end, h)
        roi = depth_image[roi_y_start:roi_y_end, :]

        # === 检测画面中间区域的有效深度比例 ===
        # 这是判断"桥面丢失"的关键：只有中间大部分是深坑才算丢失
        center_region = roi[:, w//4:3*w//4]  # 中间50%区域
        valid_in_center = np.logical_and(center_region >= 150, center_region <= 800)
        center_valid_ratio = np.sum(valid_in_center) / valid_in_center.size

        # 计算中间区域深度均值（用于判断是否过桥）
        valid_depths_in_center = center_region[valid_in_center]
        center_depth_mean = int(np.mean(valid_depths_in_center)) if len(valid_depths_in_center) > 0 else 0

        # === 检测桥面边缘 ===
        left_edges = []
        right_edges = []

        for row_idx in range(roi.shape[0] // 3, roi.shape[0] * 2 // 3, 5):
            row = roi[row_idx, :]
            valid = np.logical_and(row >= 150, row <= 800)

            le = 0
            for x in range(cx, 0, -1):
                if not valid[x]:
                    le = x + 1
                    break

            re = w - 1
            for x in range(cx, w):
                if not valid[x]:
                    re = x - 1
                    break

            if 100 < (re - le) < 500:
                left_edges.append(le)
                right_edges.append(re)

        # 检查是否真正丢失（中间大部分是深坑）
        lost_threshold = self.get_parameter('lost_threshold').value

        if len(left_edges) < 3:
            # 没有检测到桥面边缘
            if center_valid_ratio > lost_threshold:
                # 中间大部分是有效深度 → 快到岸了，继续前进不修正
                return {
                    'detected': True,
                    'lateral_error': 0.0,
                    'width': 0,
                    'center_x': cx,
                    'left_edge': 0,
                    'right_edge': w,
                    'center_valid_ratio': float(center_valid_ratio),
                    'center_depth_mean': center_depth_mean,
                    'truly_lost': False,
                }
            else:
                # 中间大部分是深坑 → 真正丢失
                return {
                    'detected': False,
                    'lateral_error': 0.0,
                    'width': 0,
                    'center_x': cx,
                    'left_edge': 0,
                    'right_edge': w,
                    'center_valid_ratio': float(center_valid_ratio),
                    'center_depth_mean': center_depth_mean,
                    'truly_lost': bool(center_valid_ratio < 0.3),
                }

        le = int(np.median(left_edges))
        re = int(np.median(right_edges))
        center = (le + re) // 2

        return {
            'detected': True,
            'lateral_error': float((center - cx) / (w / 2)),
            'width': int(re - le),
            'center_x': int(center),
            'left_edge': int(le),
            'right_edge': int(re),
            'center_valid_ratio': float(center_valid_ratio),
            'center_depth_mean': center_depth_mean,
            'truly_lost': False,
        }

    # ═══════════════════════════════════════════════════════════════════════════
    #  控制逻辑 — 丝滑版：持续前进 + 实时微调
    # ═══════════════════════════════════════════════════════════════════════════

    def _control_loop(self):
        if not self.mission_active:
            return

        with self._depth_lock:
            depth = self.latest_depth
        if depth is None:
            return

        result = self.detect_bridge(depth)
        self.bridge_detected = result['detected']
        self.lateral_error = result['lateral_error']

        self._publish_status(result)

        if self.get_parameter('publish_debug_image').value:
            self._publish_debug_image(depth, result)

        if self.state == STATE_INIT_ARM:
            pass

        elif self.state == STATE_CROSSING:
            self._handle_crossing(result)

    def _handle_crossing(self, result):
        """
        过桥：持续前进 + 实时微调方向
        不停车，丝滑通过
        """
        # 检查是否已过桥
        center_valid = result['center_valid_ratio']
        center_depth_mean = result.get('center_depth_mean', 0)

        # 条件1：中间有效深度低于50%（深坑/无效）
        # 条件2：中间深度均值>560mm（远离桥面，已过桥）
        if center_valid < 0.5 or center_depth_mean > 560:
            self.get_logger().info(
                f'\033[1;32m[BalanceBeam]\033[0m 已通过独木桥！'
                f'(有效率={center_valid:.2f}, 深度均值={center_depth_mean}mm)，停车'
            )
            self._stop_car()
            self.stop_mission()
            return

        # PD控制（比例+微分）
        kp = self.get_parameter('steering_kp').value
        kd = self.get_parameter('steering_kd').value
        max_steering = self.get_parameter('max_steering').value

        error = result['lateral_error']

        # 平滑滤波
        self.error_history.append(error)
        if len(self.error_history) > 5:
            self.error_history.pop(0)
        smoothed_error = np.mean(self.error_history)

        # PD转向计算
        # P项：响应当前误差
        # D项：响应误差变化率，抑制振荡
        p_term = smoothed_error * kp
        d_term = (smoothed_error - self.prev_error) * kd
        steering = p_term + d_term
        self.prev_error = smoothed_error
        steering = max(-max_steering, min(max_steering, steering))

        # 前进速度
        move_speed = self.get_parameter('move_speed').value

        # 发送命令（同时发线速度和角速度，但都很小）
        cmd = Twist()
        cmd.linear.x = move_speed
        cmd.angular.z = steering
        self.cmd_vel_pub.publish(cmd)

        self.get_logger().info(
            f'[BalanceBeam] 过桥: 偏移={smoothed_error:.3f}, '
            f'转向={steering:.3f}, 速度={move_speed}, '
            f'中心有效率={result["center_valid_ratio"]:.2f}'
        )

    # ═══════════════════════════════════════════════════════════════════════════
    #  任务控制
    # ═══════════════════════════════════════════════════════════════════════════

    def start_mission(self):
        if self.mission_active:
            return

        self.get_logger().info('\033[1;36m[BalanceBeam]\033[0m 启动')
        self.mission_active = True
        self.state = STATE_INIT_ARM
        self.error_history.clear()

        self._stop_car()

        def init_arm():
            time.sleep(0.5)
            self._set_arm_observe_pose()
            self.get_logger().info('[BalanceBeam] 等待机械臂...')
            time.sleep(5.0)
            self._arm_ready()

        threading.Thread(target=init_arm, daemon=True).start()

    def _arm_ready(self):
        if not self.mission_active:
            return
        if not self.depth_ready.wait(timeout=10.0):
            self.get_logger().error('[BalanceBeam] 深度图未就绪')
            self.stop_mission()
            return

        self.get_logger().info('\033[1;32m[BalanceBeam]\033[0m 就绪，开始过桥！')
        self.state = STATE_CROSSING

    def stop_mission(self):
        self.get_logger().info('\033[1;31m[BalanceBeam]\033[0m 停止')
        self.mission_active = False
        self.state = STATE_STOPPED
        self._stop_car()

    def reset_state(self):
        self._stop_car()
        self.state = STATE_IDLE
        self.mission_active = False
        self.error_history.clear()
        self.prev_error = 0.0

    def _stop_car(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def _set_arm_observe_pose(self):
        cmd = String()
        cmd.data = OBSERVE_ARM_CMD
        self.arm_cmd_pub.publish(cmd)

    # ═══════════════════════════════════════════════════════════════════════════
    #  调试
    # ═══════════════════════════════════════════════════════════════════════════

    def _publish_status(self, result):
        import json
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'bridge_detected': result['detected'],
            'lateral_error': float(result['lateral_error']),
            'center_valid_ratio': float(result['center_valid_ratio']),
            'truly_lost': result['truly_lost'],
        })
        self.status_pub.publish(msg)

    def _publish_debug_image(self, depth, result):
        try:
            dv = np.clip(depth, 0, 1000).astype(np.uint8)
            dv = cv2.equalizeHist(dv)
            img = cv2.cvtColor(dv, cv2.COLOR_GRAY2BGR)

            h, w = depth.shape
            ry1 = self.get_parameter('roi_y_start').value
            ry2 = self.get_parameter('roi_y_end').value

            cv2.rectangle(img, (0, ry1), (w-1, ry2), (255, 255, 0), 1)
            cv2.line(img, (w//2, 0), (w//2, h-1), (255, 0, 0), 1)

            # 绘制桥面边缘
            if result['detected']:
                cv2.line(img, (result['left_edge'], ry1), (result['left_edge'], ry2), (0, 255, 0), 2)
                cv2.line(img, (result['right_edge'], ry1), (result['right_edge'], ry2), (0, 255, 0), 2)
                cv2.line(img, (result['center_x'], ry1), (result['center_x'], ry2), (0, 0, 255), 2)

            # 显示信息
            cv2.putText(img, f"Err: {result['lateral_error']:.3f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(img, f"Valid: {result['center_valid_ratio']:.2f}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(img, f"State: {self.state}", (10, h-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            if result['truly_lost']:
                cv2.putText(img, "TRULY LOST!", (w//2-80, h//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = BalanceBeamNode()
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
