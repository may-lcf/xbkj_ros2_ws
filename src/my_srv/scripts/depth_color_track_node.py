#!/usr/bin/env python3
"""
depth_color_track_node.py — 深度增强颜色追踪

基于 color_track_node.py 的 PID 追踪架构 + Aurora 930 深度相机。

功能:
  - 使用深度相机进行颜色检测，PID 控制舵机追踪目标
  - 订阅 /color 话题选择追踪颜色
  - 实时显示目标深度信息

话题:
  订阅:
    /aurora/rgb/image_raw       (bgr8)
    /aurora/depth/image_raw     (mono16, mm)
    /color                      (String) — 追踪颜色: red/blue/green
  发布:
    /depth_color_track/image_result  (调试画面)
"""

import os, sys, time, threading
import numpy as np, cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from message_filters import ApproximateTimeSynchronizer
from cv_bridge import CvBridge

# ── 路径 ──
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
for p in (_SCRIPT_DIR, os.path.expanduser('~/ros2_ws/src/my_srv/scripts'), os.path.expanduser('~/OpenCV')):
    if p not in sys.path:
        sys.path.insert(0, p)

from depth_utils import DepthUtils
import z_uart
from example_interfaces.srv import Trigger
from z_uart import uart_send_str, setup_uart, close_uart


# ═══════════════════════════════════════════════════════════════════════════════
#  颜色阈值加载
# ═══════════════════════════════════════════════════════════════════════════════

def _load_thresholds(filename):
    for d in (_SCRIPT_DIR, os.getcwd(), os.path.expanduser('~/ros2_ws/src/my_srv/scripts')):
        fp = os.path.join(d, filename)
        if os.path.exists(fp):
            break
    else:
        fp = filename
    with open(fp) as f:
        nums = []
        for line in f:
            line = line.strip()
            if not line: continue
            for s in line.split():
                nums.append(int(s) if '.' not in s else float(s))
    lo = (int(nums[0]), int(nums[2]), int(nums[4]))
    hi = (int(nums[1]), int(nums[3]), int(nums[5]))
    return lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]


red_low = _load_thresholds('red.txt')
blue_low = _load_thresholds('blue.txt')
green_low = _load_thresholds('green.txt')


# ═══════════════════════════════════════════════════════════════════════════════
#  PID
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


# ═══════════════════════════════════════════════════════════════════════════════
#  DepthColorTrackNode
# ═══════════════════════════════════════════════════════════════════════════════

class DepthColorTrackNode(Node):
    def __init__(self):
        super().__init__('depth_color_track_node')
        self.du = DepthUtils(self)

        self.latest_rgb = None
        self.latest_depth = None
        self._frame_lock = threading.Lock()
        self.bridge = CvBridge()

        self.width = 640
        self.height = 480

        # ── 追踪状态 ──
        self.track_color = None         # 当前追踪颜色 (来自 /color 话题)
        self.detected_color = None
        self.target_rect = None
        self.block_cx = 0
        self.block_cy = 0
        self.target_depth_mm = 0

        # 舵机 PWM
        self.servo0 = 1500  # 底座旋转
        self.servo2 = 1871  # 肩关节

        # PID 目标: 640×480 中心
        self.TARGET_CX, self.TARGET_CY = 320, 240

        # 颜色阈值
        self.lower_red   = np.array(red_low[0:3], dtype=np.uint8)
        self.upper_red   = np.array(red_low[3:6], dtype=np.uint8)
        self.lower_blue  = np.array(blue_low[0:3], dtype=np.uint8)
        self.upper_blue  = np.array(blue_low[3:6], dtype=np.uint8)
        self.lower_green = np.array(green_low[0:3], dtype=np.uint8)
        self.upper_green = np.array(green_low[3:6], dtype=np.uint8)

        # PID (640x480 分辨率)
        self.pid_x = PIDController(kp=0.15, ki=0.0, kd=0.0)
        self.pid_y = PIDController(kp=0.15, ki=0.0, kd=0.0)

        # 追踪激活标志
        self.track_active = False
        self.active = False  # enter/exit 模式守卫
        self._run_thread = None

        # 调试话题
        self.debug_pub = self.create_publisher(Image, '/depth_color_track/image_result', 10)

        # /color 话题订阅
        self.color_sub = self.create_subscription(String, '/color', self._color_callback, 10)

        # RGB + 深度同步
        from message_filters import Subscriber as MfSub
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        rgb_sub = MfSub(self, Image, '/aurora/rgb/image_raw', _qos)
        depth_sub = MfSub(self, Image, '/aurora/depth/image_raw', _qos)
        self._sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        self._sync.registerCallback(self._synced_callback)

        # enter/exit 服务
        self.enter_srv = self.create_service(Trigger, '/depth_color_track/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/depth_color_track/exit', self.exit_callback)

        self.get_logger().info('\033[1;36m[DepthColorTrack]\033[0m 深度增强颜色追踪已启动')

    def _color_callback(self, msg: String):
        color = msg.data.strip().lower()
        if color in ('red', 'blue', 'green'):
            self.track_color = color
            self.get_logger().info(f'[Track] 追踪颜色: {color}')
        elif color == 'stop':
            self.track_color = None
            self.get_logger().info('[Track] 停止追踪')

    # ══ 同步 ══════════════════════════════════════════════════════════════════

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

    # ══ 颜色检测 ══════════════════════════════════════════════════════════════

    def _detect_color(self, mask):
        k = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if not cnts:
            return 0, (0, 0), None
        best, ba = None, -1
        for c in cnts:
            rect = cv2.minAreaRect(c)
            (cx, cy), (w, h), _ = rect
            a = cv2.contourArea(c)
            if min(w, h) < 15 or a < 500: continue
            if a > ba:
                ba = a
                best = (a, (cx, cy), rect)
        return best if best else (0, (0, 0), None)

    def _get_depth_at(self, cx, cy):
        """获取指定像素的深度值 (mm)，带邻域搜索"""
        with self._frame_lock:
            dimg = self.latest_depth
        if dimg is None: return 0
        for r in range(0, 6):
            for dy in range(-r, r + 1, max(1, r)):
                for dx in range(-r, r + 1, max(1, r)):
                    d = self.du.get_depth_at(int(cx + dx), int(cy + dy), dimg)
                    if d is not None and d >= 150:
                        return int(d)
        return 0

    # ══ 主循环 ════════════════════════════════════════════════════════════════

    def run(self):
        if not self.du.wait_for_intrinsics(15.0):
            self.get_logger().error('[DepthColorTrack] 内参超时'); return
        self.get_logger().info('[DepthColorTrack] 等待同步帧...')
        for _ in range(200):
            with self._frame_lock:
                if self.latest_rgb is not None: break
            time.sleep(0.1)
        else:
            self.get_logger().error('[DepthColorTrack] 无同步帧'); return

        self.track_active = True
        # 初始舵机位置
        uart_send_str('{#000P1500T1000!#001P1432T1000!#002P1871T1000!#003P0666T1000!#004P1481T1000!}')
        time.sleep(1)
        self.servo0 = 1500
        self.servo2 = 1871

        self.get_logger().info('\033[1;32m[DepthColorTrack]\033[0m 追踪就绪，等待 /color 指令')

        fc = 0
        while self.track_active and rclpy.ok():
            if not self.active:
                time.sleep(0.1)
                continue
            with self._frame_lock:
                if self.latest_rgb is None: time.sleep(0.03); continue
                frame = self.latest_rgb.copy()

            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

            # ── 颜色检测 ──
            self.detected_color = None
            self.target_rect = None
            found = False

            if self.track_color:
                if self.track_color == 'red':
                    _, _, rect = self._detect_color(cv2.inRange(lab, self.lower_red, self.upper_red))
                elif self.track_color == 'blue':
                    _, _, rect = self._detect_color(cv2.inRange(lab, self.lower_blue, self.upper_blue))
                elif self.track_color == 'green':
                    _, _, rect = self._detect_color(cv2.inRange(lab, self.lower_green, self.upper_green))
                else:
                    rect = None

                if rect:
                    self.target_rect = rect
                    self.block_cx, self.block_cy = rect[0]
                    self.detected_color = self.track_color
                    found = True
                    self.target_depth_mm = self._get_depth_at(self.block_cx, self.block_cy)
            else:
                # 无指定颜色时，检测所有颜色，选最大
                for cname, clo, chi in [
                    ('red', self.lower_red, self.upper_red),
                    ('blue', self.lower_blue, self.upper_blue),
                    ('green', self.lower_green, self.upper_green),
                ]:
                    _, _, rect = self._detect_color(cv2.inRange(lab, clo, chi))
                    if rect:
                        a = cv2.contourArea(cv2.boxPoints(rect))
                        if not found or a > cv2.contourArea(cv2.boxPoints(self.target_rect)):
                            self.target_rect = rect
                            self.block_cx, self.block_cy = rect[0]
                            self.detected_color = cname
                            found = True
                if found:
                    self.target_depth_mm = self._get_depth_at(self.block_cx, self.block_cy)

            fc += 1
            if fc % 100 == 0:
                self.get_logger().info(
                    f'[TRACK] f={fc} color={self.track_color} found={found} '
                    f'det={self.detected_color} d={self.target_depth_mm}mm '
                    f'servo0={self.servo0} servo2={self.servo2}')

            # ── 绘制调试画面 ──
            color_map = {'red': (0, 0, 255), 'blue': (255, 0, 0), 'green': (0, 255, 0)}
            if found and self.target_rect:
                c = color_map.get(self.detected_color, (255, 255, 255))
                box = cv2.boxPoints(self.target_rect)
                box_i = np.intp(box)
                cv2.drawContours(frame, [box_i], -1, c, 2)
                cv2.drawMarker(frame, (int(self.block_cx), int(self.block_cy)),
                               c, cv2.MARKER_CROSS, 18, 2)
                label = f"{self.detected_color} d={self.target_depth_mm}mm"
                cv2.putText(frame, label,
                            (int(box_i[:, 0].min()), max(int(box_i[:, 1].min()) - 8, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 2)

            # HUD
            mode = f"Track: {self.track_color}" if self.track_color else "Track: auto"
            cv2.putText(frame, mode, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"S0={self.servo0} S2={self.servo2}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
            except Exception:
                pass

            # ── PID 追踪 ──
            if found and self.track_color:
                self.pid_x.Target_val = self.TARGET_CX
                self.pid_y.Target_val = self.TARGET_CY
                dx = self.pid_x.PID_Realize(self.block_cx)
                dy = self.pid_y.PID_Realize(self.block_cy)
                self.servo0 += int(dx)
                self.servo2 -= int(dy)
                self.servo0 = max(600, min(2400, self.servo0))
                self.servo2 = max(600, min(2400, self.servo2))
                uart_send_str("{{#000P{:0>4d}T0000!#002P{:0>4d}T0000!}}".format(
                    self.servo0, self.servo2))

            time.sleep(0.03)

        self.get_logger().info('[DepthColorTrack] 结束')


# ═══════════════════════════════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════════════════════════════

    # ══ enter/exit 服务 ════════════════════════════════════════════════════════

    def enter_callback(self, request, response):
        self.get_logger().info('收到Enter服务，启动深度颜色追踪！')
        if not self.active:
            try:
                if not setup_uart(115200):
                    response.success = False
                    response.message = '串口初始化失败'
                    return response
                uart_send_str('{#000P1500T1000!#001P1432T1000!#002P1871T1000!#003P0666T1000!#004P1481T1000!}')
                time.sleep(1)
                # 重置状态
                self.move_status = 0
                self.move_x, self.move_y, self.move_z = 0, 118, 90
                self.servo0 = 1500
                self.servo2 = 1871
                self.active = True
                self.track_active = True
                self._run_thread = threading.Thread(target=self.run, daemon=True)
                self._run_thread.start()
            except Exception as e:
                self.get_logger().error(f'硬件初始化失败: {e}')
                response.success = False
                response.message = f'硬件初始化失败: {e}'
                return response
        response.success = True
        response.message = '深度颜色追踪已启动'
        return response

    def exit_callback(self, request, response):
        self.get_logger().info('收到Exit服务，停止深度颜色追踪！')
        if self.active:
            self.active = False
            self.track_active = False
            close_uart()
            if self._run_thread and self._run_thread.is_alive():
                self._run_thread.join(timeout=3.0)
        response.success = True
        response.message = '深度颜色追踪已停止'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DepthColorTrackNode()
    exec_ = MultiThreadedExecutor(); exec_.add_node(node)
    try:
        exec_.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.active = False
        node.track_active = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
