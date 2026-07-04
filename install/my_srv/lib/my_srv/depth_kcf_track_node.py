#!/usr/bin/env python3
"""
depth_kcf_track_node.py — 深度增强 KCF/CSRT 视觉追踪

功能:
  - 节点启动即发布相机画面（/depth_kcf_track/image_result）
  - 通过 /kcf/roi 话题接收框选坐标（x, y, w, h）
  - KCF/CSRT 算法持续追踪，PID 控制舵机实时跟随
  - 利用深度相机获取目标 3D 坐标

话题:
  订阅:
    /aurora/rgb/image_raw           (bgr8)
    /aurora/depth/image_raw         (mono16, mm)
    /kcf/algorithm                  (String) — "kcf" 或 "csrt"
    /kcf/roi                        (Int32MultiArray) — [x, y, w, h]
  发布:
    /depth_kcf_track/image_result   (调试画面)
  服务:
    /depth_kcf_track/enter          (Trigger) — 启动追踪模式
    /depth_kcf_track/exit           (Trigger) — 停止追踪

用法（Pi5 端）:
  python3 ~/ros2_ws/src/my_srv/scripts/depth_kcf_track_node.py

用法（Ubuntu PC 端，有显示器）:
  python3 kcf_roi_selector.py
  # 在弹出窗口中鼠标框选目标，按 ENTER 确认
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
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Int32MultiArray
from message_filters import ApproximateTimeSynchronizer
from cv_bridge import CvBridge

# ── 路径 ──
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
for p in (_SCRIPT_DIR, os.path.expanduser('~/ros2_ws/src/my_srv/scripts'),
          os.path.expanduser('~/OpenCV')):
    if p not in sys.path:
        sys.path.insert(0, p)

from depth_utils import DepthUtils
import z_uart
from example_interfaces.srv import Trigger
from z_uart import uart_send_str, setup_uart, close_uart


# ═══════════════════════════════════════════════════════════════════════════════
#  PID 控制器
# ═══════════════════════════════════════════════════════════════════════════════

class PIDController:
    def __init__(self, kp, ki, kd):
        self.Target_val = 0.0
        self.last_error = 0.0
        self.sum_error = 0.0
        self.kp, self.ki, self.kd = kp, ki, kd

    def PID_Realize(self, actual_val):
        err = self.Target_val - actual_val
        self.sum_error += err
        out = self.kp * err + self.ki * self.sum_error + self.kd * (err - self.last_error)
        self.last_error = err
        return out

    def reset(self):
        self.last_error = 0.0
        self.sum_error = 0.0


# ═══════════════════════════════════════════════════════════════════════════════
#  追踪器工厂
# ═══════════════════════════════════════════════════════════════════════════════

def create_tracker(algorithm='csrt'):
    algo = algorithm.lower()
    if algo == 'kcf':
        return cv2.TrackerKCF_create()
    else:
        return cv2.TrackerCSRT_create()


# ═══════════════════════════════════════════════════════════════════════════════
#  DepthKCFTrackNode
# ═══════════════════════════════════════════════════════════════════════════════

class DepthKCFTrackNode(Node):
    def __init__(self):
        super().__init__('depth_kcf_track_node')
        self.du = DepthUtils(self)

        self.latest_rgb = None
        self.latest_depth = None
        self._frame_lock = threading.Lock()
        self.bridge = CvBridge()

        self.width = 640
        self.height = 480

        # ── 追踪状态 ──
        self.tracker = None
        self.algorithm = 'csrt'
        self.tracking_active = False
        self.bbox = None
        self.track_center = (0, 0)
        self.target_depth_mm = 0
        self.lost_frames = 0
        self.max_lost_frames = 30
        self.template = None
        self.template_bbox = None

        # 舵机 PWM
        self.servo0 = 1500
        self.servo2 = 1700

        # PID 目标: 640×480 中心
        self.TARGET_CX, self.TARGET_CY = 320, 240
        self.pid_x = PIDController(kp=0.12, ki=0.001, kd=0.02)
        self.pid_y = PIDController(kp=0.12, ki=0.001, kd=0.02)

        # 运行控制
        self.active = True
        self.enter_active = False
        self._run_thread = None

        # 调试话题（压缩图像，避免 DDS 缓冲区溢出）
        self.debug_pub = self.create_publisher(CompressedImage, '/depth_kcf_track/image_result/compressed', 10)

        # 算法切换话题
        self.algo_sub = self.create_subscription(String, '/kcf/algorithm', self._algo_callback, 10)

        # ── ROI 接收话题（来自远程 PC 框选脚本）──
        self.roi_sub = self.create_subscription(Int32MultiArray, '/kcf/roi', self._roi_callback, 10)

        # RGB + 深度同步
        from message_filters import Subscriber as MfSub
        _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        rgb_sub = MfSub(self, Image, '/aurora/rgb/image_raw', _qos)
        depth_sub = MfSub(self, Image, '/aurora/depth/image_raw', _qos)
        self._sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        self._sync.registerCallback(self._synced_callback)

        # enter/exit 服务
        self.enter_srv = self.create_service(Trigger, '/depth_kcf_track/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/depth_kcf_track/exit', self.exit_callback)

        self.get_logger().info('\033[1;36m[DepthKCFTrack]\033[0m 深度增强 KCF/CSRT 追踪节点已启动')

        # ── 启动主循环线程 ──
        self._run_thread = threading.Thread(target=self.run, daemon=True)
        self._run_thread.start()

    # ══ ROI 接收 ══════════════════════════════════════════════════════════════

    def _roi_callback(self, msg: Int32MultiArray):
        """接收远程框选的 ROI: [x, y, w, h]"""
        data = msg.data
        if len(data) != 4:
            self.get_logger().warn(f'[KCF] ROI 格式错误: {data}，需要 [x, y, w, h]')
            return
        x, y, w, h = data
        if w < 10 or h < 10:
            self.get_logger().warn(f'[KCF] ROI 太小: ({x},{y},{w},{h})')
            return
        with self._frame_lock:
            frame = self.latest_rgb
        if frame is None:
            self.get_logger().warn('[KCF] 收到 ROI 但还没有帧')
            return
        bbox = (int(x), int(y), int(w), int(h))
        self._init_tracker(frame.copy(), bbox)
        self.enter_active = True
        self.get_logger().info(f'[KCF] 收到远程 ROI: ({x},{y},{w},{h})，开始追踪')

    # ══ 算法切换 ══════════════════════════════════════════════════════════════

    def _algo_callback(self, msg: String):
        algo = msg.data.strip().lower()
        if algo in ('kcf', 'csrt'):
            self.algorithm = algo
            if algo == 'kcf':
                self.pid_x = PIDController(kp=0.10, ki=0.001, kd=0.03)
                self.pid_y = PIDController(kp=0.10, ki=0.001, kd=0.03)
            else:
                self.pid_x = PIDController(kp=0.12, ki=0.001, kd=0.02)
                self.pid_y = PIDController(kp=0.12, ki=0.001, kd=0.02)
            self.get_logger().info(f'[KCF] 切换算法: {algo.upper()}')

    # ══ 同步回调 ══════════════════════════════════════════════════════════════

    def _synced_callback(self, rgb_msg: Image, depth_msg: Image):
        if not self.active:
            return
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'mono16')
            self.height, self.width = rgb.shape[:2]
            with self._frame_lock:
                self.latest_rgb = rgb
                self.latest_depth = depth
                self.du.latest_depth = depth
        except Exception:
            pass

    # ══ 深度获取 ══════════════════════════════════════════════════════════════

    def _get_depth_at(self, cx, cy):
        with self._frame_lock:
            dimg = self.latest_depth
        if dimg is None:
            return 0
        for r in range(0, 8):
            for dy in range(-r, r + 1, max(1, r)):
                for dx in range(-r, r + 1, max(1, r)):
                    d = self.du.get_depth_at(int(cx + dx), int(cy + dy), dimg)
                    if d is not None and d >= 150:
                        return int(d)
        return 0

    # ══ 追踪器初始化 ══════════════════════════════════════════════════════════

    def _init_tracker(self, frame, bbox):
        self.tracker = create_tracker(self.algorithm)
        self.tracker.init(frame, bbox)
        self.bbox = bbox
        self.tracking_active = True
        self.lost_frames = 0
        x, y, w, h = [int(v) for v in bbox]
        self.template = frame[y:y+h, x:x+w].copy()
        self.template_bbox = bbox
        self.pid_x.reset()
        self.pid_y.reset()
        self.get_logger().info(f'[KCF] 追踪器已初始化 ({self.algorithm.upper()}) bbox={bbox}')

    # ══ 目标丢失恢复 ══════════════════════════════════════════════════════════

    def _try_recover(self, frame):
        if self.template is None or self.template_bbox is None:
            return False

        tbbox = self.template_bbox
        tx, ty, tw, th = [int(v) for v in tbbox]

        expand = 0.8
        ex = max(0, int(tx - tw * expand))
        ey = max(0, int(ty - th * expand))
        ew = min(self.width - ex, int(tw * (1 + 2 * expand)))
        eh = min(self.height - ey, int(th * (1 + 2 * expand)))

        if ew > 20 and eh > 20:
            region = frame[ey:ey+eh, ex:ex+ew]
            th_h, th_w = self.template.shape[:2]
            if th_h < eh and th_w < ew:
                result = cv2.matchTemplate(region, self.template, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, max_loc = cv2.minMaxLoc(result)
                if max_val > 0.4:
                    new_bbox = (ex + max_loc[0], ey + max_loc[1], tw, th)
                    self.tracker = create_tracker(self.algorithm)
                    try:
                        self.tracker.init(frame, new_bbox)
                        self.bbox = new_bbox
                        self.tracking_active = True
                        self.lost_frames = 0
                        self.get_logger().info(f'[KCF] 恢复成功 (score={max_val:.2f})')
                        return True
                    except Exception:
                        pass

        if self.template.shape[0] < self.height and self.template.shape[1] < self.width:
            result = cv2.matchTemplate(frame, self.template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(result)
            if max_val > 0.5:
                new_bbox = (max_loc[0], max_loc[1], tw, th)
                self.tracker = create_tracker(self.algorithm)
                try:
                    self.tracker.init(frame, new_bbox)
                    self.bbox = new_bbox
                    self.tracking_active = True
                    self.lost_frames = 0
                    self.template = frame[max_loc[1]:max_loc[1]+th,
                                          max_loc[0]:max_loc[0]+tw].copy()
                    self.template_bbox = new_bbox
                    self.get_logger().info(f'[KCF] 全局恢复成功 (score={max_val:.2f})')
                    return True
                except Exception:
                    pass

        return False

    # ══ 主循环 ════════════════════════════════════════════════════════════════

    def run(self):
        if not self.du.wait_for_intrinsics(15.0):
            self.get_logger().error('[KCF] 内参超时')
            return

        self.get_logger().info('[KCF] 等待同步帧...')
        for _ in range(200):
            with self._frame_lock:
                if self.latest_rgb is not None:
                    break
            time.sleep(0.1)
        else:
            self.get_logger().error('[KCF] 无同步帧')
            return

        # ── 初始化串口 + 移到初始观察位置 ──
        try:
            if not setup_uart(115200):
                self.get_logger().error('[KCF] 串口初始化失败')
            else:
                uart_send_str('{#000P1500T1000!#001P1432T1000!#002P1700T1000!#003P1000T1000!#004P1481T1000!}')
                time.sleep(1)
                self.servo0 = 1500
                self.servo2 = 1700
                self.get_logger().info('\033[1;32m[KCF]\033[0m 机械臂已移到初始观察位置')
        except Exception as e:
            self.get_logger().error(f'[KCF] 串口初始化异常: {e}')

        self.get_logger().info('[KCF] 画面发布中，等待 /kcf/roi 话题框选目标')

        fc = 0
        last_time = time.time()

        while rclpy.ok():
            if not self.active:
                time.sleep(0.1)
                continue

            with self._frame_lock:
                if self.latest_rgb is None:
                    time.sleep(0.03)
                    continue
                frame = self.latest_rgb.copy()

            found = False
            fc += 1

            # ══ 追踪 ════════════════════════════════════════════════════════

            if self.enter_active and self.tracking_active and self.tracker is not None:
                success, new_bbox = self.tracker.update(frame)
                if success:
                    self.bbox = new_bbox
                    x, y, w, h = [int(v) for v in new_bbox]
                    self.track_center = (x + w // 2, y + h // 2)
                    self.target_depth_mm = self._get_depth_at(
                        self.track_center[0], self.track_center[1])
                    self.lost_frames = 0
                    found = True

                    if fc % 30 == 0:
                        cx, cy = self.track_center
                        tw, th = min(w, 120), min(h, 120)
                        tx = max(0, cx - tw // 2)
                        ty = max(0, cy - th // 2)
                        if tx + tw <= self.width and ty + th <= self.height:
                            self.template = frame[ty:ty+th, tx:tx+tw].copy()
                            self.template_bbox = (tx, ty, tw, th)
                else:
                    self.lost_frames += 1
                    if self.lost_frames <= self.max_lost_frames:
                        recovered = self._try_recover(frame)
                        if recovered:
                            found = True
                        else:
                            self.get_logger().warn(
                                f'[KCF] 目标丢失 ({self.lost_frames}/{self.max_lost_frames})')
                    else:
                        self.tracking_active = False
                        self.tracker = None
                        self.get_logger().warn('[KCF] 目标丢失超时，等待重新发送 ROI')

            # ══ 绘制调试画面 ════════════════════════════════════════════════

            debug_frame = frame.copy()

            if found and self.bbox is not None:
                x, y, w, h = [int(v) for v in self.bbox]
                cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.drawMarker(debug_frame, self.track_center,
                               (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
                label = f"d={self.target_depth_mm}mm"
                cv2.putText(debug_frame, label, (x, max(y - 8, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                cv2.putText(debug_frame, self.algorithm.upper(),
                            (x + w - 40, max(y - 8, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)
            else:
                if not self.enter_active:
                    cv2.putText(debug_frame, "Publish ROI to /kcf/roi [x,y,w,h]",
                                (10, self.height - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
                else:
                    cv2.putText(debug_frame, "Tracking lost - send new ROI",
                                (10, self.height - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            mode_str = f"Track: {self.algorithm.upper()}" if self.tracking_active else "Idle"
            cv2.putText(debug_frame, mode_str, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(debug_frame, f"S0={self.servo0} S2={self.servo2}",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            now = time.time()
            fps = 1.0 / max(now - last_time, 1e-6)
            last_time = now
            cv2.putText(debug_frame, f"FPS: {fps:.1f}", (self.width - 120, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            if fc % 2 == 0:
                try:
                    small = cv2.resize(debug_frame, (320, 240))
                    msg = CompressedImage()
                    msg.format = 'jpeg'
                    msg.data = np.array(cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 80])[1]).tobytes()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    self.debug_pub.publish(msg)
                except Exception:
                    pass

            # ── PID 追踪控制 ──
            if found and self.tracking_active and self.enter_active:
                self.pid_x.Target_val = self.TARGET_CX
                self.pid_y.Target_val = self.TARGET_CY
                dx = self.pid_x.PID_Realize(self.track_center[0])
                dy = self.pid_y.PID_Realize(self.track_center[1])

                max_step = 15
                dx = max(-max_step, min(max_step, dx))
                dy = max(-max_step, min(max_step, dy))

                self.servo0 += int(dx)
                self.servo2 -= int(dy)
                self.servo0 = max(600, min(2400, self.servo0))
                self.servo2 = max(600, min(2400, self.servo2))

                uart_send_str("{{#000P{:0>4d}T0000!#002P{:0>4d}T0000!}}".format(
                    self.servo0, self.servo2))

            if fc % 50 == 0:
                self.get_logger().info(
                    f'[KCF] f={fc} algo={self.algorithm} found={found} '
                    f'd={self.target_depth_mm}mm s0={self.servo0} s2={self.servo2} '
                    f'fps={fps:.1f} enter={self.enter_active}')

            time.sleep(0.03)

        self.get_logger().info('[KCF] 追踪循环结束')

    # ══ enter/exit 服务 ════════════════════════════════════════════════════════

    def enter_callback(self, request, response):
        self.get_logger().info('收到 Enter 服务，等待 /kcf/roi 框选目标')
        self.enter_active = True
        response.success = True
        response.message = '已进入追踪模式，请发布 /kcf/roi [x,y,w,h] 框选目标'
        return response

    def exit_callback(self, request, response):
        self.get_logger().info('收到 Exit 服务，停止追踪')
        self.enter_active = False
        self.tracking_active = False
        self.tracker = None
        response.success = True
        response.message = '追踪已停止'
        return response


# ═══════════════════════════════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = DepthKCFTrackNode()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.active = False
        node.tracking_active = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
