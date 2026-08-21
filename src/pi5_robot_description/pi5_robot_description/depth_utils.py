#!/usr/bin/env python3
"""
depth_utils.py — 深度相机工具模块 (从 my_srv 移植)

功能:
  1. 缓存 RGB 相机内参 (fx, fy, cx, cy)
  2. pixel_to_3d() — 像素坐标 + 深度值 → 相机坐标系 3D 点
  3. 加载 hand_eye_calib.yaml → T_cam2gripper 外参矩阵
  4. transform_cam_to_base() — 相机 3D → 基座 3D
  5. 深度图读取与有效值检查

坐标系:
  - 像素坐标: (u, v)，左上角原点，u 向右，v 向下
  - 相机坐标系: x 向右，y 向下，z 向前（深度方向）
  - 基座坐标系: x 向右，y 向前，z 向上
  - T_cam2gripper: 相机→末端执行器（hand_eye_calib.yaml 标定）
  - T_gripper2base: 末端→基座（FK 实时计算）
"""

import os
import threading
import numpy as np
import yaml

from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge


# 标定文件路径 (pi5_robot_description 包内)
_PKG_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_CONFIG_DIR = os.path.join(_PKG_DIR, 'config')
_HAND_EYE_YAML = os.path.join(_CONFIG_DIR, 'hand_eye_calib.yaml')
_INTRINSICS_CACHE = os.path.join(_CONFIG_DIR, 'camera_intrinsics.yaml')

# 也检查 my_srv 的标定文件 (如果本包内没有)
_MY_SRV_CALIB = os.path.expanduser('~/ros2_ws/src/my_srv/config/hand_eye_calib.yaml')
_MY_SRV_INTRINSICS = os.path.expanduser('~/ros2_ws/src/my_srv/config/camera_intrinsics.yaml')


class DepthUtils:
    """深度相机工具类"""

    def __init__(self, node=None):
        """
        Args:
            node: 可选 ROS2 Node，若提供则自动订阅相机内参话题
        """
        self._node = node
        self._bridge = CvBridge()

        # RGB 相机内参
        self.K = None
        self.fx = self.fy = self.cx = self.cy = None
        self.D = None
        self._intrinsics_ready = threading.Event()

        # 外参 T_cam2gripper
        self.T_cam2gripper = None
        self.R_cam2gripper = None
        self.t_cam2gripper = None

        # 最新深度帧 (mono16, mm)
        self.latest_depth = None
        self._depth_lock = threading.Lock()

        if node is not None:
            self._setup_subscriptions()

    def _setup_subscriptions(self):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self._node.create_subscription(
            CameraInfo, '/aurora/rgb/camera_info', self._rgb_info_cb, qos)
        self._node.create_subscription(
            Image, '/aurora/depth/image_raw', self._depth_cb, qos)

    def _rgb_info_cb(self, msg):
        if self.K is not None:
            return
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64)
        self.fx = self.K[0, 0]
        self.fy = self.K[1, 1]
        self.cx = self.K[0, 2]
        self.cy = self.K[1, 2]
        if self._node:
            self._node.get_logger().info(
                f'[DepthUtils] RGB 内参: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}')
        self._save_intrinsics_cache()
        self._intrinsics_ready.set()

    def _depth_cb(self, msg):
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, 'mono16')
            with self._depth_lock:
                self.latest_depth = depth
        except Exception:
            pass

    # ── 像素 → 3D ──

    def wait_for_intrinsics(self, timeout=15.0):
        if self.K is not None:
            return True
        if not self._intrinsics_ready.wait(timeout=timeout):
            if not self.load_intrinsics_cache():
                return False
        return self.K is not None

    def pixel_to_3d(self, u, v, depth_mm):
        """像素坐标 + 深度值 → 相机坐标系 3D 点 (米)"""
        if self.K is None:
            raise RuntimeError("相机内参未就绪")
        z = depth_mm / 1000.0
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return np.array([x, y, z], dtype=np.float64)

    def get_depth_at(self, u, v, depth_image=None):
        """读取指定像素的深度值 (mm)，无效返回 None"""
        if depth_image is None:
            with self._depth_lock:
                depth_image = self.latest_depth
        if depth_image is None:
            return None
        h, w = depth_image.shape[:2]
        if not (0 <= v < h and 0 <= u < w):
            return None
        depth_mm = int(depth_image[v, u])
        if depth_mm <= 0 or depth_mm > 4000:
            return None
        return depth_mm

    # ── 外参变换 ──

    def load_hand_eye_calib(self):
        """加载 hand_eye_calib.yaml → T_cam2gripper"""
        if self.T_cam2gripper is not None:
            return True
        # 优先使用本包内的标定文件，其次用 my_srv 的
        for path in [_HAND_EYE_YAML, _MY_SRV_CALIB]:
            if os.path.isfile(path):
                try:
                    with open(path) as f:
                        data = yaml.safe_load(f)
                    R = np.array(data['R_cam2gripper'], dtype=np.float64)
                    t = np.array(data['t_cam2gripper'], dtype=np.float64).reshape(3, 1)
                    self.R_cam2gripper = R
                    self.t_cam2gripper = t
                    self.T_cam2gripper = np.eye(4, dtype=np.float64)
                    self.T_cam2gripper[:3, :3] = R
                    self.T_cam2gripper[:3, 3] = t.flatten()
                    if self._node:
                        self._node.get_logger().info(
                            f'[DepthUtils] 外参已加载: {path}')
                    return True
                except Exception as e:
                    if self._node:
                        self._node.get_logger().warn(f'[DepthUtils] 加载失败 {path}: {e}')
        if self._node:
            self._node.get_logger().error('[DepthUtils] 未找到标定文件')
        return False

    def transform_cam_to_gripper(self, p_cam):
        """相机坐标系 → 末端执行器坐标系 (米)"""
        if self.T_cam2gripper is None:
            raise RuntimeError("外参未加载")
        p = np.append(p_cam.flatten()[:3], 1.0)
        p_ee = self.T_cam2gripper @ p
        return p_ee[:3]

    def transform_cam_to_base(self, p_cam, T_gripper2base):
        """
        相机坐标系 → 基座坐标系 (完整链路)

        Args:
            p_cam: [x, y, z] 米，相机坐标系
            T_gripper2base: (4,4) 末端→基座齐次矩阵 (米)

        Returns:
            [x, y, z] (3,) 米，基座坐标系
        """
        p_ee = self.transform_cam_to_gripper(p_cam)
        p = np.append(p_ee.flatten()[:3], 1.0)
        p_base = T_gripper2base @ p
        return p_base[:3]

    # ── 文件 I/O ──

    def _save_intrinsics_cache(self):
        if self.K is None:
            return
        os.makedirs(_CONFIG_DIR, exist_ok=True)
        data = {
            'K': self.K.tolist(),
            'D': self.D.tolist() if self.D is not None else [0, 0, 0, 0, 0],
        }
        with open(_INTRINSICS_CACHE, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

    def load_intrinsics_cache(self):
        """从缓存文件加载相机内参"""
        for path in [_INTRINSICS_CACHE, _MY_SRV_INTRINSICS]:
            if not os.path.isfile(path):
                continue
            try:
                with open(path) as f:
                    data = yaml.safe_load(f)
                self.K = np.array(data['K'], dtype=np.float64)
                self.D = np.array(data['D'], dtype=np.float64)
                self.fx = self.K[0, 0]
                self.fy = self.K[1, 1]
                self.cx = self.K[0, 2]
                self.cy = self.K[1, 2]
                self._intrinsics_ready.set()
                if self._node:
                    self._node.get_logger().info(
                        f'[DepthUtils] 从缓存加载内参: fx={self.fx:.1f}')
                return True
            except Exception:
                continue
        return False
