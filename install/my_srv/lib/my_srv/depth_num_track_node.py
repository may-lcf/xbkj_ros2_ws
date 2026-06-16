#!/usr/bin/env python3
"""
depth_num_track_node.py — 深度增强数字追踪

使用 Aurora 930 深度相机 + TFLite 数字检测 + PID 舵机追踪。
订阅 /num (Int32) 选择追踪的数字。
"""

import os, sys, time, threading
import numpy as np, cv2
from ai_edge_litert.interpreter import Interpreter

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

MODEL_PATH = os.path.join(os.path.expanduser('~'), 'OpenCV', 'trained2.tflite')
LABELS_PATH = os.path.join(os.path.expanduser('~'), 'OpenCV', 'labels.txt')
CONF_THRESHOLD = 0.4
MODEL_INPUT_H = 128
MODEL_INPUT_W = 128


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


class DepthNumTrackNode(Node):
    def __init__(self):
        super().__init__('depth_num_track_node')
        self.du = DepthUtils(self)

        self.latest_rgb = None
        self.latest_depth = None
        self._frame_lock = threading.Lock()
        self.bridge = CvBridge()
        self.width, self.height = 640, 480

        self.interpreter, self.input_det, self.output_det, self.labels = self._load_model()

        self.track_num = None
        self.find_num = 0
        self.block_cx, self.block_cy = 0, 0
        self.target_depth_mm = 0
        self.servo0, self.servo2 = 1500, 1871
        self.track_active = False
        self.active = False  # enter/exit 模式守卫
        self._run_thread = None

        self.TARGET_CX, self.TARGET_CY = 320, 240
        self.pid_x = PIDController(kp=0.06, ki=0.0, kd=0.0)
        self.pid_y = PIDController(kp=0.06, ki=0.0, kd=0.0)

        self.debug_pub = self.create_publisher(Image, '/depth_num_track/image_result', 10)
        self.num_sub = self.create_subscription(Int32, '/num', self._num_callback, 10)

        from message_filters import Subscriber as MfSub
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        rgb_sub = MfSub(self, Image, '/aurora/rgb/image_raw', _qos)
        depth_sub = MfSub(self, Image, '/aurora/depth/image_raw', _qos)
        self._sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        self._sync.registerCallback(self._synced_callback)

        # enter/exit 服务
        self.enter_srv = self.create_service(Trigger, '/depth_num_track/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/depth_num_track/exit', self.exit_callback)

        self.get_logger().info('\033[1;36m[DepthNumTrack]\033[0m 深度增强数字追踪已启动')

    def _load_model(self):
        interpreter = Interpreter(model_path=MODEL_PATH)
        interpreter.allocate_tensors()
        input_det = interpreter.get_input_details()[0]
        output_det = interpreter.get_output_details()[0]
        with open(LABELS_PATH, "r", encoding="utf-8") as f:
            labels = [line.strip() for line in f if line.strip()]
        return interpreter, input_det, output_det, labels

    def _num_callback(self, msg):
        self.track_num = str(msg.data)
        self.get_logger().info(f'[Track] 追踪数字: {self.track_num}')

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

    def _detect_numbers(self, frame):
        resized = cv2.resize(frame, (MODEL_INPUT_W, MODEL_INPUT_H))
        input_data = resized.astype(np.float32) / 255.0
        input_data = np.expand_dims(input_data, axis=0)
        self.interpreter.set_tensor(self.input_det['index'], input_data)
        self.interpreter.invoke()
        raw = self.interpreter.get_tensor(self.output_det['index']).squeeze(axis=0)
        detections = []
        conf_scores = raw[:, 4]
        cls_probs = raw[:, 5:]
        cls_ids = np.argmax(cls_probs, axis=1)
        total_confs = conf_scores * np.max(cls_probs, axis=1)
        for idx in np.where(total_confs > CONF_THRESHOLD)[0]:
            bx, by, bw, bh = raw[idx, :4] * 128
            x1, y1 = max(0, int(bx - bw / 2)), max(0, int(by - bh / 2))
            x2 = min(MODEL_INPUT_W - 1, int(bx + bw / 2))
            y2 = min(MODEL_INPUT_H - 1, int(by + bh / 2))
            sx, sy = self.width / MODEL_INPUT_W, self.height / MODEL_INPUT_H
            detections.append({
                "class_name": self.labels[int(cls_ids[idx])],
                "confidence": float(total_confs[idx]),
                "bbox": (int(x1 * sx), int(y1 * sy), int(x2 * sx), int(y2 * sy)),
                "center": (int((x1 + x2) / 2 * sx), int((y1 + y2) / 2 * sy))
            })
        return detections

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
        self.get_logger().info('\033[1;32m[DepthNumTrack]\033[0m 追踪就绪，等待 /num 指令')

        fc = 0
        while self.track_active and rclpy.ok():
            if not self.active:
                time.sleep(0.1)
                continue
            with self._frame_lock:
                if self.latest_rgb is None: time.sleep(0.03); continue
                frame = self.latest_rgb.copy()

            self.find_num = 0
            found = False

            if self.track_num is not None:
                detections = self._detect_numbers(frame)
                target_dets = [d for d in detections if d["class_name"] == self.track_num]
                if target_dets:
                    best = max(target_dets, key=lambda d: d["confidence"])
                    self.block_cx, self.block_cy = best["center"]
                    self.target_depth_mm = self._get_depth_at(self.block_cx, self.block_cy)
                    self.find_num = 1
                    found = True
                    x1, y1, x2, y2 = best["bbox"]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{best['class_name']} {best['confidence']:.2f} d={self.target_depth_mm}mm",
                                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            fc += 1
            if fc % 50 == 0:
                now = time.time()
                fps = 50 / (now - self._last_log_t) if hasattr(self, '_last_log_t') else 0
                self._last_log_t = now
                self.get_logger().info(
                    f'[TRACK] f={fc} fps={fps:.1f} num={self.track_num} found={found} '
                    f'd={self.target_depth_mm}mm s0={self.servo0} s2={self.servo2}')

            # 每 3 帧发布一次 debug 画面，减少开销
            if fc % 3 == 0:
                mode = f"NUM{self.track_num}" if self.track_num else "None"
                cv2.putText(frame, f"Track: {mode}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, f"S0={self.servo0} S2={self.servo2}", (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                try:
                    self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
                except Exception:
                    pass

            if found and self.track_num is not None:
                self.pid_x.Target_val = self.TARGET_CX
                self.pid_y.Target_val = self.TARGET_CY
                self.servo0 += int(self.pid_x.PID_Realize(self.block_cx))
                self.servo2 -= int(self.pid_y.PID_Realize(self.block_cy))
                self.servo0 = max(600, min(2400, self.servo0))
                self.servo2 = max(600, min(2400, self.servo2))
                uart_send_str("{{#000P{:0>4d}T0000!#002P{:0>4d}T0000!}}".format(
                    self.servo0, self.servo2))

            time.sleep(0.03)

        self.get_logger().info('[DepthNumTrack] 结束')


    # ══ enter/exit 服务 ════════════════════════════════════════════════════════

    def enter_callback(self, request, response):
        self.get_logger().info('收到Enter服务，启动深度数字追踪！')
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
        response.message = '深度数字追踪已启动'
        return response

    def exit_callback(self, request, response):
        self.get_logger().info('收到Exit服务，停止深度数字追踪！')
        if self.active:
            self.active = False
            self.track_active = False
            close_uart()
            if self._run_thread and self._run_thread.is_alive():
                self._run_thread.join(timeout=3.0)
        response.success = True
        response.message = '深度数字追踪已停止'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DepthNumTrackNode()
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
