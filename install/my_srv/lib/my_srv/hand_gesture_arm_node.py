#!/usr/bin/env python3
"""
hand_gesture_arm_node.py — 手势识别控制机械臂节点

功能：
  - 使用 MediaPipe HandLandmarker 识别手势
  - 将手势映射为机械臂舵机指令，发布到 /joint_commands（servo_node 转发到串口）
  - 提供 /hand_gesture_arm/enter 和 /hand_gesture_arm/exit 服务
  - 发布标注结果图像到 /hand_gesture_arm/image_result

手势映射：
  fist   (握拳)         → 爪子开合 (#005)
  five   (五指张开)      → 底座左右摇摆 (#000)
  one    (食指伸出)      → 大臂上下点头 (#001)
  two    (V字/两指)      → 招手展示动作
  three  (三指)          → 左右挥臂2次
  six    (大拇指+小指)   → 复位到初始姿态

依赖：
  pip3 install mediapipe
  模型文件: ~/ros2_ws/src/my_srv/models/hand_landmarker.task
  下载: wget -O ~/ros2_ws/src/my_srv/models/hand_landmarker.task \
    https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
"""

import cv2
import enum
import time
import math
import numpy as np
import queue
import threading
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from example_interfaces.srv import Trigger

# ── MediaPipe 可选导入 ──────────────────────────────────────────────────────
# pip3 install --user 时包安装在 ~/.local，ROS2 运行时 PYTHONPATH 不含此路径，
# 在此手动加入，确保节点进程也能找到 mediapipe
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

# ── 机械臂初始姿态 (来自 joy_arm_node.py) ────────────────────────────────────
INIT_CMD = '{#000P1500T1000!#001P1666T1000!#002P1750T1000!#003P0905T1000!#004P1500T1000!#005P1500T1000!}'

# ── MediaPipe 模型路径候选 ───────────────────────────────────────────────────
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_CANDIDATES = [
    os.path.normpath(os.path.join(_SCRIPT_DIR, '..', 'models', 'hand_landmarker.task')),
    os.path.expanduser('~/ros2_ws/src/my_srv/models/hand_landmarker.task'),
    os.path.expanduser('~/hand_landmarker.task'),
]

def _find_model() -> str | None:
    for p in MODEL_CANDIDATES:
        if os.path.isfile(p):
            return p
    return None


# ── 手指角度与手势识别（移植自 src_25.12.29/car_vision/hand_gesture_arm.py）──
def _vec_angle(v1: np.ndarray, v2: np.ndarray) -> float:
    """计算两2D向量夹角（度），范围 -180~180"""
    d = np.linalg.norm(v1) * np.linalg.norm(v2)
    if d < 1e-6:
        return 0.0
    cos_val = float(np.clip(v1.dot(v2) / d, -1.0, 1.0))
    sin_val = float(np.cross(v1, v2) / d)
    return float(np.degrees(np.arctan2(sin_val, cos_val)))


def hand_angle(lm: np.ndarray) -> list:
    """
    lm: shape (21,2) 像素坐标，MediaPipe 21个关键点
    返回5个手指的弯曲角度列表（绝对值）
    """
    return [
        abs(_vec_angle(lm[3] - lm[4],  lm[0] - lm[2])),   # 拇指
        abs(_vec_angle(lm[0] - lm[6],  lm[7] - lm[8])),   # 食指
        abs(_vec_angle(lm[0] - lm[10], lm[11] - lm[12])), # 中指
        abs(_vec_angle(lm[0] - lm[14], lm[15] - lm[16])), # 无名指
        abs(_vec_angle(lm[0] - lm[18], lm[19] - lm[20])), # 小指
    ]


def h_gesture(a: list) -> str:
    """通过5个手指弯曲角度识别手势名称"""
    T  = 65.0   # 弯曲阈值（大于T认为弯曲）
    Ts = 49.0   # 伸展阈值（小于Ts认为伸展）
    Tt = 53.0   # 拇指阈值

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


# ── 状态机 ───────────────────────────────────────────────────────────────────
class State(enum.Enum):
    NULL    = 0  # 空闲，可接受新手势
    RUNNING = 1  # 动作执行中，忽略新手势


# ── 主节点 ───────────────────────────────────────────────────────────────────
class HandGestureArmNode(Node):
    def __init__(self):
        super().__init__('hand_gesture_arm_node')

        if not MEDIAPIPE_OK:
            self.get_logger().error(
                'MediaPipe 未安装！请先执行: pip3 install mediapipe\n'
                '然后下载模型: wget -O ~/ros2_ws/src/my_srv/models/hand_landmarker.task '
                'https://storage.googleapis.com/mediapipe-models/hand_landmarker/'
                'hand_landmarker/float16/1/hand_landmarker.task'
            )
            raise RuntimeError('mediapipe not installed')

        model_path = _find_model()
        if model_path is None:
            self.get_logger().error(
                '未找到 hand_landmarker.task！\n'
                '下载命令:\n'
                '  mkdir -p ~/ros2_ws/src/my_srv/models\n'
                '  wget -O ~/ros2_ws/src/my_srv/models/hand_landmarker.task \\\n'
                '    https://storage.googleapis.com/mediapipe-models/hand_landmarker/'
                'hand_landmarker/float16/1/hand_landmarker.task'
            )
            raise RuntimeError('model file not found')

        self.get_logger().info(f'手势模型: {model_path}')

        # ── 参数 ──────────────────────────────────────────────────────────────
        self.declare_parameter('camera_index',   0)
        self.declare_parameter('debounce_count', 5)    # 连续N帧才触发，防误触
        self.declare_parameter('image_width',    640)
        self.declare_parameter('image_height',   480)
        self.declare_parameter('fps',            15)

        self.camera_index   = self.get_parameter('camera_index').value
        self.debounce_count = self.get_parameter('debounce_count').value
        self.img_w          = self.get_parameter('image_width').value
        self.img_h          = self.get_parameter('image_height').value
        self.fps            = self.get_parameter('fps').value

        # ── MediaPipe 检测器（只检测1只手，节省算力）───────────────────────────
        base_opts = mp_python.BaseOptions(model_asset_path=model_path)
        opts = vision.HandLandmarkerOptions(
            base_options=base_opts,
            min_hand_detection_confidence=0.4,
            num_hands=1,
        )
        self.detector = vision.HandLandmarker.create_from_options(opts)

        # ── 状态变量 ────────────────────────────────────────────────────────────
        self.active       = False
        self.running      = True
        self.camera_open  = False
        self.cap          = None
        self.state        = State.NULL
        self.count        = 0
        self.last_gesture = 'none'
        self.no_finger_ts = time.time()
        self.image_queue  = queue.Queue(maxsize=2)

        # ── ROS2 通信 ──────────────────────────────────────────────────────────
        _qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.cmd_pub    = self.create_publisher(String, '/joint_commands', _qos)
        self.bridge     = CvBridge()
        self.result_pub = self.create_publisher(Image, '/hand_gesture_arm/image_result', 10)

        # ── Enter / Exit 服务 ──────────────────────────────────────────────────
        self.create_service(Trigger, '/hand_gesture_arm/enter', self.enter_callback)
        self.create_service(Trigger, '/hand_gesture_arm/exit',  self.exit_callback)

        # ── 图像处理线程（常驻，active=False时只消耗队列不处理）─────────────────
        threading.Thread(target=self._proc_loop, daemon=True).start()

        self.get_logger().info(
            '\033[1;32mhand_gesture_arm_node 已就绪\033[0m — '
            '调用 /hand_gesture_arm/enter 启动'
        )

    # ── 服务：启动 ────────────────────────────────────────────────────────────
    def enter_callback(self, request, response):
        self.get_logger().info('收到 Enter 服务，启动手势臂控制...')
        if not self.active:
            try:
                self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
                if not self.cap.isOpened():
                    raise RuntimeError(f'摄像头 {self.camera_index} 无法打开')
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.img_w)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_h)
                self.cap.set(cv2.CAP_PROP_FPS,          self.fps)
                self.camera_open = True

                # 机械臂复位到初始姿态
                self._send(INIT_CMD)
                time.sleep(0.8)

                self.state        = State.NULL
                self.count        = 0
                self.last_gesture = 'none'
                self.no_finger_ts = time.time()
                self.active       = True

                threading.Thread(target=self._capture_loop, daemon=True).start()
                self.get_logger().info('✅ 手势臂控制已启动，举起手掌开始识别')

            except Exception as e:
                self.get_logger().error(f'启动失败: {e}')
                if self.cap:
                    self.cap.release()
                    self.camera_open = False
                response.success = False
                response.message = str(e)
                return response

        response.success = True
        response.message = '手势臂控制已启动'
        return response

    # ── 服务：停止 ────────────────────────────────────────────────────────────
    def exit_callback(self, request, response):
        self.get_logger().info('收到 Exit 服务，停止手势臂控制...')
        self.active = False
        if self.camera_open and self.cap:
            self.cap.release()
            self.cap = None
            self.camera_open = False
        self._send(INIT_CMD)   # 复位机械臂
        response.success = True
        response.message = '手势臂控制已停止'
        return response

    # ── 发布关节指令 ───────────────────────────────────────────────────────────
    def _send(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.get_logger().debug(f'→ {cmd}')

    # ── 手势动作执行（独立线程，避免阻塞图像处理）─────────────────────────────
    def _do_act(self, gesture: str):
        """
        每个手势对应一段动作序列，执行完毕后恢复状态机为 NULL。
        舵机编号说明（来自 joy_arm_node.py）：
          #000 底座旋转（1500=中，600=右，2400=左）
          #001 大臂（1666=初始，600=抬高，2400=放低）
          #002 小臂
          #003 腕俯仰（905=初始）
          #004 腕旋转（1500=中）
          #005 爪子（700=张开，2200=夹紧）
        """
        self.get_logger().info(f'🤖 执行手势: [{gesture}]')
        try:
            if gesture == 'fist':
                # 握拳 → 爪子夹紧后张开，回中
                self._send('{#005P2200T500!}')
                time.sleep(0.5)
                self._send('{#005P700T500!}')
                time.sleep(0.5)
                self._send('{#005P1500T400!}')
                time.sleep(0.4)

            elif gesture == 'five':
                # 五指张开 → 底座左右摇摆
                self._send('{#000P2200T600!}')
                time.sleep(0.6)
                self._send('{#000P800T600!}')
                time.sleep(0.6)
                self._send('{#000P1500T300!}')
                time.sleep(0.3)

            elif gesture == 'four':
                # 一指伸出 → 大臂上下点头
                self._send('{#001P1100T600!}')
                time.sleep(0.6)
                self._send('{#001P1900T600!}')
                time.sleep(0.6)
                self._send('{#001P1666T300!}')
                time.sleep(0.3)

            elif gesture == 'two':
                # V字手势 → 抬臂后腕关节左右招手
                self._send('{#001P1500T1000!#002P1500T1000!#003P1500T1000!}')
                time.sleep(1.0)
                for _ in range(2):
                    self._send('{#003P1900T400!}')
                    time.sleep(0.4)
                    self._send('{#003P1100T400!}')
                    time.sleep(0.4)
                self._send(INIT_CMD)
                time.sleep(1.0)

            elif gesture == 'three':
                # 三指 → 底座左右挥臂 2 次
                for _ in range(2):
                    self._send('{#000P2000T500!}')
                    time.sleep(0.5)
                    self._send('{#000P1000T500!}')
                    time.sleep(0.5)
                self._send('{#000P1500T300!}')
                time.sleep(0.3)

            elif gesture == 'one':
                # 四指 → 所有舵机归中（1500），然后复位
                self._send('{#000P1500T800!#001P1500T800!#002P1500T800!#003P1500T800!#004P1500T800!#005P1500T800!}')
                time.sleep(1.0)
                self._send(INIT_CMD)
                time.sleep(1.0)

            elif gesture == 'six':
                # 大拇指+小指 → 复位
                self._send(INIT_CMD)
                time.sleep(1.0)

        except Exception as e:
            self.get_logger().warn(f'动作执行异常: {e}')
        finally:
            time.sleep(0.3)
            self.count        = 0
            self.last_gesture = 'none'
            self.state        = State.NULL

    # ── 摄像头读取线程 ────────────────────────────────────────────────────────
    def _capture_loop(self):
        """持续读取摄像头帧，放入 image_queue（满了丢旧帧）"""
        while self.active and self.camera_open:
            if not self.cap or not self.cap.isOpened():
                time.sleep(0.05)
                continue
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            frame = cv2.flip(frame, 1)   # 左右镜像，使手势方向直觉一致
            if self.image_queue.full():
                try:
                    self.image_queue.get_nowait()
                except queue.Empty:
                    pass
            self.image_queue.put(frame)

    # ── 图像处理线程（常驻）──────────────────────────────────────────────────
    def _proc_loop(self):
        while self.running:
            # 取帧（超时1秒后重试，以便检查 self.running）
            try:
                frame = self.image_queue.get(block=True, timeout=1.0)
            except queue.Empty:
                continue

            if not self.active:
                continue

            annotated = frame.copy()

            try:
                # ── MediaPipe 推理 ──
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                result = self.detector.detect(mp_img)
                lm_list = result.hand_landmarks

                if lm_list:
                    self.no_finger_ts = time.time()
                    hand = lm_list[0]   # 只取第一只手
                    h, w = annotated.shape[:2]
                    pts  = np.array([[lm.x * w, lm.y * h] for lm in hand])

                    # 绘制关键点连线
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

                    # 手势识别
                    gesture = h_gesture(hand_angle(pts))
                    color = (0, 255, 0) if gesture != 'none' else (128, 128, 128)
                    cv2.putText(annotated, f'gesture: {gesture}  count:{self.count}',
                                (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

                    # 防抖：连续 debounce_count 帧同一手势才触发动作
                    if self.state == State.NULL:
                        if gesture == self.last_gesture and gesture != 'none':
                            self.count += 1
                        else:
                            self.count = 0
                        if self.count >= self.debounce_count:
                            self.state = State.RUNNING
                            threading.Thread(
                                target=self._do_act, args=(gesture,), daemon=True
                            ).start()
                    self.last_gesture = gesture

                else:
                    # 超过2秒检测不到手，重置状态机
                    cv2.putText(annotated, 'no hand', (10, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (100, 100, 100), 2)
                    if time.time() - self.no_finger_ts > 2.0:
                        self.state        = State.NULL
                        self.count        = 0
                        self.last_gesture = 'none'

                # 显示当前状态
                state_color = (0, 200, 255) if self.state == State.RUNNING else (200, 200, 200)
                cv2.putText(annotated, f'state: {self.state.name}',
                            (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)

            except Exception as e:
                self.get_logger().warn(f'处理帧异常: {e}')

            # 发布标注图像（供 web_video_server 显示）
            try:
                self.result_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
            except Exception:
                pass

    # ── 节点销毁 ──────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info('hand_gesture_arm_node 正在关闭...')
        self.running = False
        self.active  = False
        if self.camera_open and self.cap:
            self.cap.release()
        self._send(INIT_CMD)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HandGestureArmNode()
    except RuntimeError as e:
        print(f'[hand_gesture_arm_node] 启动失败: {e}')
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
