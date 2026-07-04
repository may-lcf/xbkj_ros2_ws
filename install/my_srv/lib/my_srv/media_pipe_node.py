#!/usr/bin/env python3
"""
media_pipe_node.py — MediaPipe 多模式视觉识别节点

功能：
  通过 mode 参数选择三种识别模式：
    face_contour    — 面部轮廓识别（468个面部关键点 + 网格）
    fingertip_trace — 指尖轨迹识别（5个指尖历史轨迹 + 手势）
    body_skeleton   — 人体骨架识别（33个身体关键点 + 骨架连线）

用法：
  ros2 run my_srv media_pipe_node.py --ros-args -p mode:=face_contour
  ros2 run my_srv media_pipe_node.py --ros-args -p mode:=fingertip_trace
  ros2 run my_srv media_pipe_node.py --ros-args -p mode:=body_skeleton

服务：
  /media_pipe/enter  — 启动识别
  /media_pipe/exit   — 停止识别

话题：
  /media_pipe/image_result — 发布标注后的图像

依赖：
  pip3 install mediapipe
  模型文件（自动查找 ~/ros2_ws/src/my_srv/models/ 下）：
    face_landmarker.task    — 面部轮廓模型
    hand_landmarker.task    — 手部关键点模型
    pose_landmarker_lite.task — 人体骨架模型（lite版，适合Pi5）
"""

import cv2
import time
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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from example_interfaces.srv import Trigger

# ── MediaPipe 可选导入 ──────────────────────────────────────────────────────
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

# ── 常量 ───────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, '..', 'models'))

VALID_MODES = ('face_contour', 'fingertip_trace', 'body_skeleton')

# 指尖关键点 ID（MediaPipe Hands 21个点中，5个指尖的索引）
FINGERTIP_IDS = [4, 8, 12, 16, 20]  # 拇指尖、食指尖、中指尖、无名指尖、小指尖
FINGERTIP_COLORS = [
    (0, 0, 255),    # 拇指 - 红
    (0, 165, 255),  # 食指 - 橙
    (0, 255, 0),    # 中指 - 绿
    (255, 0, 0),    # 无名指 - 蓝
    (255, 0, 255),  # 小指 - 紫
]

# Pose 连接关系（COCO 格式的骨架连线）
POSE_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 7),       # 头部左半
    (0, 4), (4, 5), (5, 6), (6, 8),       # 头部右半
    (9, 10),                                # 嘴巴
    (11, 12),                               # 肩膀
    (11, 13), (13, 15),                     # 左臂
    (12, 14), (14, 16),                     # 右臂
    (11, 23), (12, 24),                     # 躯干
    (23, 24),                               # 髋部
    (23, 25), (25, 27),                     # 左腿
    (24, 26), (26, 28),                     # 右腿
]

# ── 相机类型检测 ───────────────────────────────────────────────────────────
def detect_camera() -> str:
    """检测USB相机类型，返回 'depth' 或 'mono'"""
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=3)
        if '3251:1930' in result.stdout:
            return 'depth'
    except Exception:
        pass
    return 'mono'


def _find_model(name: str) -> str | None:
    """在多个候选路径下查找模型文件"""
    candidates = [
        os.path.join(MODELS_DIR, name),
        os.path.expanduser(os.path.join("~", "ros2_ws", "src", "my_srv", "models", name)),
        os.path.expanduser(os.path.join("~", name)),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p
    return None


# ── 手指角度与手势识别（移植自 hand_gesture_arm_node.py）──────────────────
def _vec_angle(v1: np.ndarray, v2: np.ndarray) -> float:
    d = np.linalg.norm(v1) * np.linalg.norm(v2)
    if d < 1e-6:
        return 0.0
    cos_val = float(np.clip(v1.dot(v2) / d, -1.0, 1.0))
    sin_val = float(np.cross(v1, v2) / d)
    return float(np.degrees(np.arctan2(sin_val, cos_val)))


def hand_angle(lm: np.ndarray) -> list:
    return [
        abs(_vec_angle(lm[3] - lm[4],  lm[0] - lm[2])),
        abs(_vec_angle(lm[0] - lm[6],  lm[7] - lm[8])),
        abs(_vec_angle(lm[0] - lm[10], lm[11] - lm[12])),
        abs(_vec_angle(lm[0] - lm[14], lm[15] - lm[16])),
        abs(_vec_angle(lm[0] - lm[18], lm[19] - lm[20])),
    ]


def h_gesture(a: list) -> str:
    T, Ts, Tt = 65.0, 49.0, 53.0
    if   a[0]>Tt and a[1]>T  and a[2]>T  and a[3]>T  and a[4]>T:   return "fist"
    elif a[0]<Ts and a[1]<Ts and a[2]>T  and a[3]>T  and a[4]>T:   return "one"
    elif a[0]<Ts and a[1]>T  and a[2]>T  and a[3]>T  and a[4]>T:   return "fist"
    elif a[0]>5  and a[1]<Ts and a[2]>T  and a[3]>T  and a[4]>T:   return "one"
    elif a[0]>Tt and a[1]<Ts and a[2]<Ts and a[3]>T  and a[4]>T:   return "two"
    elif a[0]>Tt and a[1]<Ts and a[2]<Ts and a[3]<Ts and a[4]>T:   return "three"
    elif a[0]>Tt and a[1]>T  and a[2]<Ts and a[3]<Ts and a[4]<Ts:  return "three"
    elif a[0]>Tt and a[1]<Ts and a[2]<Ts and a[3]<Ts and a[4]<Ts:  return "four"
    elif a[0]<Ts and a[1]<Ts and a[2]<Ts and a[3]<Ts and a[4]<Ts:  return "five"
    elif a[0]<Ts and a[1]>T  and a[2]>T  and a[3]>T  and a[4]<Ts:  return "six"
    return "none"


# ── 主节点 ───────────────────────────────────────────────────────────────────
class MediaPipeNode(Node):
    def __init__(self):
        super().__init__('media_pipe_node')

        if not MEDIAPIPE_OK:
            self.get_logger().error('MediaPipe 未安装！请执行: pip3 install mediapipe')
            raise RuntimeError('mediapipe not installed')

        # ── 参数 ──────────────────────────────────────────────────────────────
        self.declare_parameter('mode', 'fingertip_trace')
        self.declare_parameter('camera_topic', '/aurora/rgb/image_raw')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 15)
        self.declare_parameter('trace_length', 40)  # 指尖轨迹保留帧数

        self.mode         = self.get_parameter('mode').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_index = self.get_parameter('camera_index').value
        self.img_w        = self.get_parameter('image_width').value
        self.img_h        = self.get_parameter('image_height').value
        self.fps          = self.get_parameter('fps').value
        self.trace_len    = self.get_parameter('trace_length').value

        if self.mode not in VALID_MODES:
            self.get_logger().error(
                f'无效的 mode="{self.mode}"，可选值: {VALID_MODES}')
            raise RuntimeError(f'invalid mode: {self.mode}')

        # 自动检测相机类型
        self.camera_type = detect_camera()
        self.get_logger().info(f'模式: {self.mode}  相机: {self.camera_type}')

        # ── 加载模型 ──────────────────────────────────────────────────────────
        self._init_detector()

        # ── 状态变量 ──────────────────────────────────────────────────────────
        self.active      = False
        self.running     = True
        self.camera_open = False
        self.cap         = None
        self._ros_sub    = None
        self.image_queue = queue.Queue(maxsize=2)
        self.fps_counter = 0
        self.fps_time    = time.time()
        self.fps_val     = 0.0

        # 指尖轨迹缓存（mode=fingertip_trace 时使用）
        # 每个指尖一个列表，存最近 N 帧的 (x, y) 坐标
        self.traces = []  # 食指轨迹

        # ── ROS2 通信 ──────────────────────────────────────────────────────────
        _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.bridge     = CvBridge()
        self.result_pub = self.create_publisher(Image, '/media_pipe/image_result', 10)

        self.create_service(Trigger, '/media_pipe/enter', self.enter_callback)
        self.create_service(Trigger, '/media_pipe/exit',  self.exit_callback)

        # ── 图像处理线程 ──────────────────────────────────────────────────────
        threading.Thread(target=self._proc_loop, daemon=True).start()

        self.get_logger().info(
            f'\033[1;32mmedia_pipe_node 已就绪\033[0m [mode={self.mode}] — '
            '调用 /media_pipe/enter 启动')

    # ── 初始化检测器 ──────────────────────────────────────────────────────────
    def _init_detector(self):
        if self.mode == 'face_contour':
            model_path = _find_model('face_landmarker.task')
            if not model_path:
                raise RuntimeError(
                    f'未找到 face_landmarker.task，请下载到 {MODELS_DIR}/')
            base_opts = mp_python.BaseOptions(model_asset_path=model_path)
            opts = vision.FaceLandmarkerOptions(
                base_options=base_opts,
                output_face_blendshapes=False,
                min_face_detection_confidence=0.5,
                num_faces=1,
            )
            self.detector = vision.FaceLandmarker.create_from_options(opts)
            self.get_logger().info(f'FaceLandmarker 已加载: {model_path}')

        elif self.mode == 'fingertip_trace':
            model_path = _find_model('hand_landmarker.task')
            if not model_path:
                raise RuntimeError(
                    f'未找到 hand_landmarker.task，请下载到 {MODELS_DIR}/')
            base_opts = mp_python.BaseOptions(model_asset_path=model_path)
            opts = vision.HandLandmarkerOptions(
                base_options=base_opts,
                min_hand_detection_confidence=0.4,
                num_hands=1,
            )
            self.detector = vision.HandLandmarker.create_from_options(opts)
            self.get_logger().info(f'HandLandmarker 已加载: {model_path}')

        elif self.mode == 'body_skeleton':
            # 尝试 lite → full → heavy
            for name in ('pose_landmarker_lite.task',
                         'pose_landmarker_full.task',
                         'pose_landmarker_heavy.task'):
                model_path = _find_model(name)
                if model_path:
                    break
            if not model_path:
                raise RuntimeError(
                    f'未找到 pose_landmarker 模型，请下载到 {MODELS_DIR}/')
            base_opts = mp_python.BaseOptions(model_asset_path=model_path)
            opts = vision.PoseLandmarkerOptions(
                base_options=base_opts,
                min_pose_detection_confidence=0.5,
                num_poses=1,
            )
            self.detector = vision.PoseLandmarker.create_from_options(opts)
            self.get_logger().info(f'PoseLandmarker 已加载: {model_path}')

    # ── Enter/Exit 服务 ──────────────────────────────────────────────────────
    def enter_callback(self, request, response):
        self.get_logger().info(f'收到 Enter 服务，启动 [{self.mode}] ...')
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
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.img_w)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_h)
                    self.cap.set(cv2.CAP_PROP_FPS,          self.fps)
                    self.camera_open = True
                    self.get_logger().info(f'USB 相机模式: /dev/video{self.camera_index}')

                self.traces = []  # 食指轨迹
                self.fps_counter = 0
                self.fps_time = time.time()
                self.active = True

                if self.camera_type != 'depth':
                    threading.Thread(target=self._capture_loop, daemon=True).start()
                self.get_logger().info(f'✅ [{self.mode}] 识别已启动')

            except Exception as e:
                self.get_logger().error(f'启动失败: {e}')
                if self.cap:
                    self.cap.release()
                    self.camera_open = False
                response.success = False
                response.message = str(e)
                return response

        response.success = True
        response.message = f'[{self.mode}] 识别已启动'
        return response

    def exit_callback(self, request, response):
        self.get_logger().info(f'收到 Exit 服务，停止 [{self.mode}] ...')
        self.active = False
        if self.camera_open and self.cap:
            self.cap.release()
            self.cap = None
            self.camera_open = False
        if self._ros_sub:
            self.destroy_subscription(self._ros_sub)
            self._ros_sub = None
        response.success = True
        response.message = f'[{self.mode}] 识别已停止'
        return response

    # ── 摄像头读取（USB 相机）──────────────────────────────────────────────
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

    # ── ROS 话题回调（深度相机）────────────────────────────────────────────
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

    # ── 图像处理线程（常驻）──────────────────────────────────────────────────
    def _proc_loop(self):
        while self.running:
            try:
                frame = self.image_queue.get(block=True, timeout=1.0)
            except queue.Empty:
                continue
            if not self.active:
                continue

            annotated = frame.copy()

            try:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

                if self.mode == 'face_contour':
                    annotated = self._draw_face(annotated, mp_img)
                elif self.mode == 'fingertip_trace':
                    annotated = self._draw_hand(annotated, mp_img)
                elif self.mode == 'body_skeleton':
                    annotated = self._draw_pose(annotated, mp_img)

            except Exception as e:
                self.get_logger().warn(f'处理帧异常: {e}')

            # FPS 计算
            self.fps_counter += 1
            now = time.time()
            if now - self.fps_time >= 1.0:
                self.fps_val = self.fps_counter / (now - self.fps_time)
                self.fps_counter = 0
                self.fps_time = now
            cv2.putText(annotated, f'{self.mode}  FPS:{self.fps_val:.1f}',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 发布标注图像
            try:
                self.result_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
            except Exception:
                pass

    # ── 模式 A：面部轮廓 ──────────────────────────────────────────────────────
    def _draw_face(self, img: np.ndarray, mp_img) -> np.ndarray:
        result = self.detector.detect(mp_img)
        if not result.face_landmarks:
            cv2.putText(img, 'no face', (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)
            return img

        h, w = img.shape[:2]
        for face in result.face_landmarks:
            # 用绿色小圆点绘制 468 个面部关键点（稀疏点云效果）
            for lm in face:
                cx, cy = int(lm.x * w), int(lm.y * h)
                cv2.circle(img, (cx, cy), 1, (0, 255, 0), -1)
            # 鼻尖高亮标记
            nose = face[1]
            cv2.circle(img, (int(nose.x * w), int(nose.y * h)), 4, (0, 255, 255), -1)

        cv2.putText(img, f'faces: {len(result.face_landmarks)}',
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return img

    # ── 模式 B：指尖轨迹 ──────────────────────────────────────────────────────
    def _draw_hand(self, img: np.ndarray, mp_img) -> np.ndarray:
        result = self.detector.detect(mp_img)

        if not result.hand_landmarks:
            cv2.putText(img, 'no hand', (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)
            self.traces = []
            return img

        h, w = img.shape[:2]
        hand = result.hand_landmarks[0]

        # 绘制手部关键点和连线
        proto = landmark_pb2.NormalizedLandmarkList()
        proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=lm.x, y=lm.y, z=lm.z)
            for lm in hand
        ])
        mp.solutions.drawing_utils.draw_landmarks(
            img, proto,
            mp.solutions.hands.HAND_CONNECTIONS,
            mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
            mp.solutions.drawing_styles.get_default_hand_connections_style(),
        )

        # 更新食指轨迹（landmark #8）
        tx = int(hand[8].x * w)
        ty = int(hand[8].y * h)
        self.traces.append((tx, ty))
        if len(self.traces) > self.trace_len:
            self.traces.pop(0)

        # 绘制食指轨迹线（绿色渐变）
        for j in range(1, len(self.traces)):
            alpha = j / len(self.traces)
            thickness = max(1, int(alpha * 4))
            cv2.line(img, self.traces[j - 1], self.traces[j], (0, 255, 0), thickness)

        # 食指尖高亮
        cv2.circle(img, (tx, ty), 6, (0, 255, 0), -1)
        cv2.circle(img, (tx, ty), 8, (255, 255, 255), 1)

        # 手势识别
        pts = np.array([[lm.x * w, lm.y * h] for lm in hand])
        gesture = h_gesture(hand_angle(pts))
        cv2.putText(img, f'gesture: {gesture}',
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return img

    # ── 模式 C：人体骨架 ──────────────────────────────────────────────────────
    def _draw_pose(self, img: np.ndarray, mp_img) -> np.ndarray:
        result = self.detector.detect(mp_img)

        if not result.pose_landmarks:
            cv2.putText(img, 'no person', (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)
            return img

        h, w = img.shape[:2]
        landmarks = result.pose_landmarks[0]

        # 转为像素坐标
        pts = []
        for lm in landmarks:
            pts.append((int(lm.x * w), int(lm.y * h)))

        # 绘制骨架连线
        for (i, j) in POSE_CONNECTIONS:
            if i < len(pts) and j < len(pts):
                cv2.line(img, pts[i], pts[j], (0, 255, 0), 2)

        # 绘制关节点（分区域着色）
        for idx, (px, py) in enumerate(pts):
            if idx <= 10:        # 头部
                color = (0, 0, 255)
            elif idx <= 16:      # 手臂
                color = (255, 128, 0)
            elif idx <= 24:      # 躯干
                color = (255, 0, 0)
            else:                # 腿部
                color = (0, 255, 128)
            cv2.circle(img, (px, py), 4, color, -1)
            cv2.circle(img, (px, py), 6, (255, 255, 255), 1)

        cv2.putText(img, f'pose landmarks: {len(pts)}',
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return img

    # ── 节点销毁 ──────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info(f'media_pipe_node [{self.mode}] 正在关闭...')
        self.running = False
        self.active  = False
        if self.camera_open and self.cap:
            self.cap.release()
        if self._ros_sub:
            self.destroy_subscription(self._ros_sub)
            self._ros_sub = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MediaPipeNode()
    except RuntimeError as e:
        print(f'[media_pipe_node] 启动失败: {e}')
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
