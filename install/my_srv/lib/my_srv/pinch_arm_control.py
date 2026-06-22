#!/usr/bin/env python3
"""
pinch_arm_control.py — 大拇指+食指捏合连续控制机械臂

功能：
  - 检测 pinch 手势（拇指+食指伸展，其他三指弯曲）
  - openness 实时连续映射到 #001/#002 舵机 PWM
  - 支持 USB 单目相机 / Aurora 930 深度相机
  - 无手势时停在当前位置，不发送新指令

映射：
  openness ∈ [0.2, 0.4] → 线性映射
    0.2（闭合）→ #001=2164, #002=2191（蜷缩）
    0.4（张开）→ #001=1500, #002=1500（伸展）

ROS2 接口：
  订阅: /aurora/rgb/image_raw（深度相机模式）
  发布: /joint_commands
  发布: /pinch_arm/image_result（标注图像）
  服务: /pinch_arm/enter, /pinch_arm/exit
"""

import cv2
import time
import math
import numpy as np
import queue
import threading
import os
import sys
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from example_interfaces.srv import Trigger

import site
_user_site = site.getusersitepackages()
if _user_site not in sys.path:
    sys.path.insert(0, _user_site)

try:
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision
    from mediapipe.framework.formats import landmark_pb2
    MEDIAPIPE_OK = True
except ImportError:
    MEDIAPIPE_OK = False


# ── 常量 ──────────────────────────────────────────────────────────────────────

INIT_CMD = '{#000P1500T1000!#001P1666T1000!#002P1750T1000!#003P0905T1000!#004P1500T1000!#005P1500T1000!}'

# PWM 范围
PWM1_MIN, PWM1_MAX = 1500, 2164   # #001: 伸展→蜷缩
PWM2_MIN, PWM2_MAX = 1500, 2191   # #002: 伸展→蜷缩

# openness 映射范围
OPEN_MIN, OPEN_MAX = 0.2, 0.4

# 发送频率限制（秒）
SEND_INTERVAL = 0.15

# 平滑系数（0~1，越大越灵敏）
SMOOTH_ALPHA = 0.25

# openness 的典型范围（归一化后的拇指-食指距离）
# 实际值需要根据 debug 日志微调
OPEN_RAW_MIN = 0.005   # 完全捏合时的距离
OPEN_RAW_MAX = 0.35    # 完全张开时的距离


# ── 工具函数 ──────────────────────────────────────────────────────────────────

def _ndist(a, b):
    """归一化坐标下两点欧氏距离（MediaPipe 返回 0~1 归一化坐标）"""
    return float(np.linalg.norm(np.array([a.x - b.x, a.y - b.y])))


def detect_camera() -> str:
    """检测相机类型，返回 'depth' 或 'mono'"""
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=3)
        if '3251:1930' in result.stdout:
            return 'depth'
    except Exception:
        pass
    return 'mono'


def calc_openness(lm_list) -> float:
    """
    计算拇指-食指开放度，归一化到 [0, 1]
    0 = 完全捏合，1 = 完全张开
    """
    dist = _ndist(lm_list[4], lm_list[8])
    t = (dist - OPEN_RAW_MIN) / (OPEN_RAW_MAX - OPEN_RAW_MIN)
    return float(np.clip(t, 0.0, 1.0))


# ── 主节点 ────────────────────────────────────────────────────────────────────

class PinchArmControlNode(Node):
    def __init__(self):
        super().__init__('pinch_arm_control')

        if not MEDIAPIPE_OK:
            self.get_logger().error('MediaPipe 未安装！pip3 install mediapipe')
            raise RuntimeError('mediapipe not installed')

        model_path = self._find_model()
        if model_path is None:
            self.get_logger().error(
                '未找到 hand_landmarker.task！\n'
                '下载: wget -O ~/ros2_ws/src/my_srv/models/hand_landmarker.task \\\n'
                '  https://storage.googleapis.com/mediapipe-models/hand_landmarker/'
                'hand_landmarker/float16/1/hand_landmarker.task'
            )
            raise RuntimeError('model not found')
        self.get_logger().info(f'手势模型: {model_path}')

        # ── 参数 ──
        self.declare_parameter('camera_topic', '/aurora/rgb/image_raw')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 15)

        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_index = self.get_parameter('camera_index').value
        self.img_w = self.get_parameter('image_width').value
        self.img_h = self.get_parameter('image_height').value
        self.fps = self.get_parameter('fps').value

        self.camera_type = detect_camera()
        self.get_logger().info(f'相机检测: {self.camera_type}')

        # ── MediaPipe ──
        base_opts = mp_python.BaseOptions(model_asset_path=model_path)
        opts = vision.HandLandmarkerOptions(
            base_options=base_opts,
            min_hand_detection_confidence=0.3,
            min_hand_presence_confidence=0.3,
            min_tracking_confidence=0.3,
            num_hands=1,
        )
        self.detector = vision.HandLandmarker.create_from_options(opts)

        # ── 状态 ──
        self.active = False
        self.running = True
        self.camera_open = False
        self.cap = None
        self._ros_sub = None
        self.image_queue = queue.Queue(maxsize=2)

        # 连续控制状态
        self._smoothed = 0.3        # 平滑后的 openness
        self._current_pwm1 = 1666   # #001 当前 PWM
        self._current_pwm2 = 1750   # #002 当前 PWM
        self._send_ts = 0.0         # 上次发送时间

        # ── ROS2 ──
        _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.cmd_pub = self.create_publisher(String, '/joint_commands', _qos)
        self.bridge = CvBridge()
        self.result_pub = self.create_publisher(Image, '/pinch_arm/image_result', 10)

        self.create_service(Trigger, '/pinch_arm/enter', self._enter_cb)
        self.create_service(Trigger, '/pinch_arm/exit', self._exit_cb)

        # 常驻处理线程
        threading.Thread(target=self._proc_loop, daemon=True).start()

        self.get_logger().info(
            '\033[1;32mpinch_arm_control 已就绪\033[0m — '
            '调用 /pinch_arm/enter 启动，捏合拇指+食指控制机械臂'
        )

    # ── 模型路径 ──
    @staticmethod
    def _find_model():
        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            os.path.normpath(os.path.join(script_dir, '..', 'models', 'hand_landmarker.task')),
            os.path.expanduser('~/ros2_ws/src/my_srv/models/hand_landmarker.task'),
            os.path.expanduser('~/hand_landmarker.task'),
        ]
        for p in candidates:
            if os.path.isfile(p):
                return p
        return None

    # ── 发送指令 ──
    def _send(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

    # ── Enter 服务 ──
    def _enter_cb(self, request, response):
        self.get_logger().info('收到 Enter 服务，启动 pinch 连续控制...')
        if not self.active:
            try:
                if self.camera_type == 'depth':
                    _qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                                      history=HistoryPolicy.KEEP_LAST)
                    self._ros_sub = self.create_subscription(
                        Image, self.camera_topic, self._ros_image_cb, _qos)
                    self.get_logger().info(f'深度相机模式: {self.camera_topic}')
                else:
                    self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
                    if not self.cap.isOpened():
                        raise RuntimeError(f'摄像头 {self.camera_index} 无法打开')
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_w)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_h)
                    self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                    self.camera_open = True
                    self.get_logger().info(f'USB 相机模式: /dev/video{self.camera_index}')

                # 复位
                self._send(INIT_CMD)
                time.sleep(0.8)
                self._smoothed = 0.3
                self._current_pwm1 = 1666
                self._current_pwm2 = 1750
                self._send_ts = 0.0
                self.active = True

                if self.camera_type != 'depth':
                    threading.Thread(target=self._capture_loop, daemon=True).start()

                self.get_logger().info('✅ pinch 连续控制已启动，捏合拇指+食指控制机械臂')

            except Exception as e:
                self.get_logger().error(f'启动失败: {e}')
                if self.cap:
                    self.cap.release()
                    self.camera_open = False
                response.success = False
                response.message = str(e)
                return response

        response.success = True
        response.message = 'pinch 连续控制已启动'
        return response

    # ── Exit 服务 ──
    def _exit_cb(self, request, response):
        self.get_logger().info('收到 Exit 服务，停止 pinch 控制...')
        self.active = False
        if self.camera_open and self.cap:
            self.cap.release()
            self.cap = None
            self.camera_open = False
        if self._ros_sub:
            self.destroy_subscription(self._ros_sub)
            self._ros_sub = None
        self._send(INIT_CMD)
        response.success = True
        response.message = 'pinch 控制已停止'
        return response

    # ── USB 相机读取线程 ──
    def _capture_loop(self):
        while self.active and self.camera_open:
            if not self.cap or not self.cap.isOpened():
                time.sleep(0.05)
                continue
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            frame = cv2.flip(frame, 1)
            if self.image_queue.full():
                try:
                    self.image_queue.get_nowait()
                except queue.Empty:
                    pass
            self.image_queue.put(frame)

    # ── ROS 话题回调（深度相机）──
    def _ros_image_cb(self, msg: Image):
        if not self.active:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            frame = cv2.flip(frame, 1)
            if self.image_queue.full():
                try:
                    self.image_queue.get_nowait()
                except queue.Empty:
                    pass
            self.image_queue.put(frame)
        except Exception:
            pass

    # ── 常驻处理线程 ──
    def _proc_loop(self):
        frame_count = 0
        while self.running:
            try:
                frame = self.image_queue.get(block=True, timeout=1.0)
            except queue.Empty:
                continue

            if not self.active:
                continue

            frame_count += 1
            annotated = frame.copy()

            try:
                # MediaPipe 推理
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                result = self.detector.detect(mp_img)
                lm_list = result.hand_landmarks

                if lm_list:
                    hand = lm_list[0]

                    # 绘制关键点
                    proto = landmark_pb2.NormalizedLandmarkList()
                    proto.landmark.extend([
                        landmark_pb2.NormalizedLandmark(x=lm.x, y=lm.y, z=lm.z)
                        for lm in hand
                    ])
                    mp.solutions.drawing_utils.draw_landmarks(
                        annotated, proto,
                        mp.solutions.hands.HAND_CONNECTIONS,
                        mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
                        mp.solutions.drawing_styles.get_default_hand_connections_style(),
                    )

                    # 持续跟踪拇指-食指距离
                    raw_open = calc_openness(hand)
                    self._smoothed = (SMOOTH_ALPHA * raw_open
                                      + (1 - SMOOTH_ALPHA) * self._smoothed)

                    # 映射 PWM：张开→PWM小(伸展)，捏合→PWM大(蜷缩)
                    t = self._smoothed
                    pwm1 = int(PWM1_MAX + t * (PWM1_MIN - PWM1_MAX))  # 2164→1500
                    pwm2 = int(PWM2_MAX + t * (PWM2_MIN - PWM2_MAX))  # 2191→1500

                    # 节流发送
                    now = time.time()
                    if now - self._send_ts >= SEND_INTERVAL:
                        self._send_ts = now
                        self._current_pwm1 = pwm1
                        self._current_pwm2 = pwm2
                        self._send(
                            f'{{#001P{pwm1:04d}T800!'
                            f'#002P{pwm2:04d}T800!}}'
                        )

                    # debug 日志：每 30 帧打印
                    if frame_count % 30 == 0:
                        raw_d = _ndist(hand[4], hand[8])
                        self.get_logger().info(
                            f'[DEBUG] dist={raw_d:.4f} open={raw_open:.3f} '
                            f'smoothed={self._smoothed:.3f} '
                            f'#001={pwm1} #002={pwm2}')

                    # 标注
                    cv2.putText(annotated,
                                f'open={self._smoothed:.2f} '
                                f'#001={self._current_pwm1} #002={self._current_pwm2}',
                                (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                (0, 200, 255), 2)
                else:
                    # 无手检测，不发送指令，停在当前位置
                    cv2.putText(annotated, 'no hand',
                                (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                (60, 60, 60), 2)

            except Exception as e:
                self.get_logger().warn(f'处理帧异常: {e}')

            # 发布标注图像
            try:
                self.result_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
            except Exception:
                pass

    # ── 节点销毁 ──
    def destroy_node(self):
        self.get_logger().info('pinch_arm_control 正在关闭...')
        self.running = False
        self.active = False
        if self.camera_open and self.cap:
            self.cap.release()
        if self._ros_sub:
            self.destroy_subscription(self._ros_sub)
            self._ros_sub = None
        self._send(INIT_CMD)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PinchArmControlNode()
    except RuntimeError as e:
        print(f'[pinch_arm_control] 启动失败: {e}')
        rclpy.shutdown()
        return

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
