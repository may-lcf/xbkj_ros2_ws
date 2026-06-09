#!/usr/bin/env python3
"""
depth_label_track_node.py — 深度增强标签追踪

使用 Aurora 930 深度相机 + AprilTag 标签检测 + PID 舵机追踪。
订阅 /label (Int32) 选择追踪的标签 ID。
"""

import os, sys, time, threading
import numpy as np, cv2
import pupil_apriltags

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from message_filters import ApproximateTimeSynchronizer
from cv_bridge import CvBridge

_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
for p in (_SCRIPT_DIR, os.path.expanduser('~/ros2_ws/src/my_srv/scripts'), os.path.expanduser('~/OpenCV')):
    if p not in sys.path:
        sys.path.insert(0, p)

from depth_utils import DepthUtils
import z_uart
from example_interfaces.srv import Trigger
from z_uart import uart_send_str, setup_uart, close_uart


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


class DepthLabelTrackNode(Node):
    def __init__(self):
        super().__init__('depth_label_track_node')
        self.du = DepthUtils(self)
        self.detector = pupil_apriltags.Detector()

        self.latest_rgb = None
        self.latest_depth = None
        self._frame_lock = threading.Lock()
        self.bridge = CvBridge()
        self.width, self.height = 640, 480

        self.track_label = None
        self.find_label = 0
        self.block_cx, self.block_cy = 0, 0
        self.target_depth_mm = 0
        self.servo0, self.servo2 = 1500, 1871
        self.track_active = False
        self.active = False  # enter/exit 模式守卫
        self._run_thread = None

        self.TARGET_CX, self.TARGET_CY = 320, 240
        self.pid_x = PIDController(kp=0.06, ki=0.0, kd=0.0)
        self.pid_y = PIDController(kp=0.06, ki=0.0, kd=0.0)

        self.debug_pub = self.create_publisher(Image, '/depth_label_track/image_result', 10)
        self.label_sub = self.create_subscription(Int32, '/label', self._label_callback, 10)

        from message_filters import Subscriber as MfSub
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        rgb_sub = MfSub(self, Image, '/aurora/rgb/image_raw', _qos)
        depth_sub = MfSub(self, Image, '/aurora/depth/image_raw', _qos)
        self._sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        self._sync.registerCallback(self._synced_callback)

        # enter/exit 服务
        self.enter_srv = self.create_service(Trigger, '/depth_label_track/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/depth_label_track/exit', self.exit_callback)

        self.get_logger().info('\033[1;36m[DepthLabelTrack]\033[0m 深度增强标签追踪已启动')

    def _label_callback(self, msg):
        self.track_label = msg.data
        self.get_logger().info(f'[Track] 追踪标签: TAG{msg.data}')

    def _synced_callback(self, rgb_msg, depth_msg):
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

    def _get_depth_at(self, cx, cy):
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

    def run(self):
        if not self.du.wait_for_intrinsics(15.0): return
        for _ in range(200):
            with self._frame_lock:
                if self.latest_rgb is not None: break
            time.sleep(0.1)
        else:
            return

        self.track_active = True
        uart_send_str('{#000P1500T1000!#001P1432T1000!#002P1871T1000!#003P0666T1000!#004P1481T1000!}')
        time.sleep(1)
        self.servo0, self.servo2 = 1500, 1871
        self.get_logger().info('\033[1;32m[DepthLabelTrack]\033[0m 追踪就绪，等待 /label 指令')

        fc = 0
        while self.track_active and rclpy.ok():
            if not self.active:
                time.sleep(0.1)
                continue
            with self._frame_lock:
                if self.latest_rgb is None: time.sleep(0.03); continue
                frame = self.latest_rgb.copy()

            # 缩放到 320x240 再检测，大幅提升速度
            small = cv2.resize(frame, (320, 240))
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
            sx, sy = self.width / 320.0, self.height / 240.0
            self.find_label = 0
            found = False

            if self.track_label is not None:
                for tag in self.detector.detect(gray):
                    if tag.tag_id == self.track_label:
                        self.block_cx = int(tag.center[0] * sx)
                        self.block_cy = int(tag.center[1] * sy)
                        self.target_depth_mm = self._get_depth_at(self.block_cx, self.block_cy)
                        self.find_label = 1
                        found = True
                        cv2.rectangle(frame,
                                      (int(tag.corners[0][0] * sx), int(tag.corners[0][1] * sy)),
                                      (int(tag.corners[2][0] * sx), int(tag.corners[2][1] * sy)),
                                      (0, 0, 255), 2)
                        cv2.circle(frame, (self.block_cx, self.block_cy), 5, (0, 255, 0), -1)
                        cv2.putText(frame, f"TAG{tag.tag_id} d={self.target_depth_mm}mm",
                                    (self.block_cx + 10, self.block_cy - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        break

            fc += 1
            if fc % 50 == 0:
                now = time.time()
                fps = 50 / (now - self._last_log_t) if hasattr(self, '_last_log_t') else 0
                self._last_log_t = now
                self.get_logger().info(
                    f'[TRACK] f={fc} fps={fps:.1f} label={self.track_label} found={found} '
                    f'd={self.target_depth_mm}mm s0={self.servo0} s2={self.servo2}')

            # 每 3 帧发布一次 debug 画面，减少开销
            if fc % 3 == 0:
                mode = f"TAG{self.track_label}" if self.track_label else "None"
                cv2.putText(frame, f"Track: {mode}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, f"S0={self.servo0} S2={self.servo2}", (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                try:
                    self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
                except Exception:
                    pass

            if found and self.track_label is not None:
                self.pid_x.Target_val = self.TARGET_CX
                self.pid_y.Target_val = self.TARGET_CY
                self.servo0 += int(self.pid_x.PID_Realize(self.block_cx))
                self.servo2 -= int(self.pid_y.PID_Realize(self.block_cy))
                self.servo0 = max(600, min(2400, self.servo0))
                self.servo2 = max(600, min(2400, self.servo2))
                uart_send_str("{{#000P{:0>4d}T0000!#002P{:0>4d}T0000!}}".format(
                    self.servo0, self.servo2))

            time.sleep(0.03)

        self.get_logger().info('[DepthLabelTrack] 结束')


    # ══ enter/exit 服务 ════════════════════════════════════════════════════════

    def enter_callback(self, request, response):
        self.get_logger().info('收到Enter服务，启动深度标签追踪！')
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
        response.message = '深度标签追踪已启动'
        return response

    def exit_callback(self, request, response):
        self.get_logger().info('收到Exit服务，停止深度标签追踪！')
        if self.active:
            self.active = False
            self.track_active = False
            close_uart()
            if self._run_thread and self._run_thread.is_alive():
                self._run_thread.join(timeout=3.0)
        response.success = True
        response.message = '深度标签追踪已停止'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DepthLabelTrackNode()
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
