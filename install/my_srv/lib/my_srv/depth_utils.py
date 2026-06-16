#!/usr/bin/env python3
"""
depth_utils.py — 深度相机工具模块（Phase1 基础设施）

功能：
  1. 缓存 RGB 相机内参 (fx, fy, cx, cy)
  2. pixel_to_3d() — 像素坐标 + 深度值 → 相机坐标系 3D 点
  3. 加载 hand_eye_calib.yaml → T_cam2gripper 外参矩阵
  4. transform_cam_to_base() — 相机 3D → 基座 3D（需提供当前 T_gripper2base）
  5. 深度图读取与有效值检查

========== 坐标系说明 ==========
  - 像素坐标: (u, v)，左上角为原点，u 向右，v 向下
  - 相机坐标系 (p_cam): x 向右，y 向下，z 向前（深度方向）
  - 基座坐标系 (p_base): x 向右，y 向前，z 向上（与 z_move 一致）
  - T_cam2gripper: 相机→末端执行器（由 eye_in_hand_calib_node 标定得出）
  - T_gripper2base: 末端→基座（由当前关节角 FK 计算得出）

========== 使用 align_mode=True 的内参说明 ==========
  Aurora 930 配置了 align_mode=True，深度图已变形对齐到 RGB 图像。
  因此 pixel_to_3d() 使用 RGB 相机内参 (K_rgb) 进行反投影。

========== 用法示例 ==========
  from depth_utils import DepthUtils

  du = DepthUtils(node)
  # ... 等待相机就绪 ...
  cam_xyz = du.pixel_to_3d(u=320, v=240, depth_mm=500)
  # cam_xyz = [x, y, z]  in meters, camera frame

  T_g2b = get_current_arm_fk()  # 4x4
  base_xyz = du.transform_cam_to_base(cam_xyz, T_g2b)
"""

import os
import sys
import threading
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


# ── 路径 ──────────────────────────────────────────────────────────────────────
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
_CALIB_DIR = os.path.expanduser('~/ros2_ws/src/my_srv/config')
_HAND_EYE_YAML = os.path.join(_CALIB_DIR, 'hand_eye_calib.yaml')
_INTRINSICS_CACHE = os.path.join(_CALIB_DIR, 'camera_intrinsics.yaml')


class DepthUtils:
    """
    深度相机工具类。

    可独立使用，也可作为 Mixin 嵌入 ROS2 Node 中。
    """

    def __init__(self, node: Node = None):
        """
        Args:
            node: 可选，若提供则自动订阅相机内参话题。
                  若不提供，需手动调用 load_intrinsics_cache() 加载缓存。
        """
        self._node = node
        self._bridge = CvBridge()

        # ── RGB 相机内参 ──
        self.K = None       # (3,3) ndarray
        self.fx = self.fy = self.cx = self.cy = None
        self.D = None       # 畸变系数 (5,)
        self._intrinsics_ready = threading.Event()

        # ── 外参 T_cam2gripper ──
        self.T_cam2gripper = None   # (4,4) ndarray, 相机→末端执行器
        self.R_cam2gripper = None   # (3,3)
        self.t_cam2gripper = None   # (3,1)

        # ── 最新深度帧（mono16, mm）──
        self.latest_depth = None
        self._depth_lock = threading.Lock()

        # ── 订阅 ──
        if node is not None:
            self._setup_subscriptions()

    # ═══════════════════════════════════════════════════════════════════════════
    #  订阅
    # ═══════════════════════════════════════════════════════════════════════════

    def _setup_subscriptions(self):
        """订阅 RGB 内参话题 + 深度图话题。"""
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._node.create_subscription(
            CameraInfo, '/aurora/rgb/camera_info',
            self._rgb_info_cb, qos,
        )
        self._node.create_subscription(
            Image, '/aurora/depth/image_raw',
            self._depth_cb, qos,
        )

    def _rgb_info_cb(self, msg: CameraInfo):
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
                f'cx={self.cx:.1f} cy={self.cy:.1f}'
            )
        self._save_intrinsics_cache()
        self._intrinsics_ready.set()

    def _depth_cb(self, msg: Image):
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, 'mono16')
            with self._depth_lock:
                self.latest_depth = depth
        except Exception:
            pass

    # ═══════════════════════════════════════════════════════════════════════════
    #  像素 → 3D
    # ═══════════════════════════════════════════════════════════════════════════

    def wait_for_intrinsics(self, timeout: float = 15.0) -> bool:
        """等待相机内参就绪。"""
        if self.K is not None:
            return True
        if not self._intrinsics_ready.wait(timeout=timeout):
            if not self.load_intrinsics_cache():
                return False
        return self.K is not None

    def pixel_to_3d(self, u: float, v: float, depth_mm: float) -> np.ndarray:
        """
        像素坐标 + 深度值 → 相机坐标系 3D 点。

        Args:
            u, v: 像素坐标（浮点数，可亚像素）
            depth_mm: 深度值（毫米），直接从 mono16 深度图读取

        Returns:
            [x, y, z] (3,) ndarray，单位 米，相机坐标系
                x 向右，y 向下，z 向前

        公式:
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            z = depth_mm / 1000.0
        """
        if self.K is None:
            raise RuntimeError("相机内参未就绪，请先调用 wait_for_intrinsics()")

        z = depth_mm / 1000.0
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return np.array([x, y, z], dtype=np.float64)

    def pixel_to_3d_from_image(self, u: int, v: int,
                                depth_image: np.ndarray = None) -> np.ndarray | None:
        """
        从深度图中读取指定像素的深度值并转换为 3D 坐标。

        Args:
            u, v: 像素坐标（整数）
            depth_image: mono16 深度图 (mm)。若为 None，使用 latest_depth。

        Returns:
            [x, y, z] 或 None（深度值无效时）
        """
        if depth_image is None:
            with self._depth_lock:
                depth_image = self.latest_depth
            if depth_image is None:
                return None

        # 边界检查
        h, w = depth_image.shape[:2]
        if not (0 <= v < h and 0 <= u < w):
            return None

        depth_mm = int(depth_image[v, u])

        # 有效深度范围（与驱动配置一致，depth=0 表示无效）
        if depth_mm <= 0 or depth_mm > 4000:
            return None

        return self.pixel_to_3d(u, v, depth_mm)

    def get_depth_at(self, u: int, v: int,
                     depth_image: np.ndarray = None) -> int | None:
        """
        读取指定像素的深度值（mm）。

        Returns:
            深度值（mm）或 None（无效时）
        """
        if depth_image is None:
            with self._depth_lock:
                depth_image = self.latest_depth
            if depth_image is None:
                return None

        h, w = depth_image.shape[:2]
        if not (0 <= v < h and 0 <= u < w):
            return None

        depth_mm = int(depth_image[v, u])
        # Aurora 930 驱动已将无效深度设为 0，depth>0 即有效
        # 保留上限 4000mm（驱动配置 max depth）
        if depth_mm <= 0 or depth_mm > 4000:
            return None
        return depth_mm

    # ═══════════════════════════════════════════════════════════════════════════
    #  外参变换
    # ═══════════════════════════════════════════════════════════════════════════

    def load_hand_eye_calib(self) -> bool:
        """
        加载 hand_eye_calib.yaml → T_cam2gripper。

        Returns:
            True 如果加载成功。
        """
        if self.T_cam2gripper is not None:
            return True

        if not os.path.isfile(_HAND_EYE_YAML):
            if self._node:
                self._node.get_logger().error(
                    f'[DepthUtils] 标定文件不存在: {_HAND_EYE_YAML}'
                )
            return False

        try:
            with open(_HAND_EYE_YAML) as f:
                data = yaml.safe_load(f)

            R = np.array(data['R_cam2gripper'], dtype=np.float64)
            t = np.array(data['t_cam2gripper'], dtype=np.float64).reshape(3, 1)

            self.R_cam2gripper = R
            self.t_cam2gripper = t
            self.T_cam2gripper = np.eye(4, dtype=np.float64)
            self.T_cam2gripper[:3, :3] = R
            self.T_cam2gripper[:3,  3] = t.flatten()

            if self._node:
                self._node.get_logger().info(
                    f'[DepthUtils] 外参已加载 (method={data.get("method")}, '
                    f'poses={data.get("num_poses")}, '
                    f'err={data.get("reprojection_error")}px)\n'
                    f'  t_cam2gripper (m): {t.flatten().round(4)}'
                )
            return True
        except Exception as e:
            if self._node:
                self._node.get_logger().error(
                    f'[DepthUtils] 加载外参失败: {e}'
                )
            return False

    def transform_cam_to_gripper(self, p_cam: np.ndarray) -> np.ndarray:
        """
        相机坐标系 → 末端执行器坐标系。

        Args:
            p_cam: [x, y, z] 或 (3,) 或 (3,1)，单位 m，相机坐标系

        Returns:
            [x, y, z] (3,) ndarray，单位 m，末端执行器坐标系
        """
        if self.T_cam2gripper is None:
            raise RuntimeError("外参未加载，请先调用 load_hand_eye_calib()")

        p = np.append(p_cam.flatten()[:3], 1.0)
        p_ee = self.T_cam2gripper @ p
        return p_ee[:3]

    def transform_cam_to_base(self, p_cam: np.ndarray,
                               T_gripper2base: np.ndarray) -> np.ndarray:
        """
        相机坐标系 → 基座坐标系（完整链路）。

        Args:
            p_cam: [x, y, z]，单位 m，相机坐标系
            T_gripper2base: (4,4) ndarray，末端→基座的齐次变换矩阵

        Returns:
            [x, y, z] (3,) ndarray，单位 m，基座坐标系
        """
        p_ee = self.transform_cam_to_gripper(p_cam)
        p = np.append(p_ee.flatten()[:3], 1.0)
        p_base = T_gripper2base @ p
        return p_base[:3]

    def build_T_gripper2base(self, R_g2b: np.ndarray,
                              t_g2b: np.ndarray) -> np.ndarray:
        """
        构造 T_gripper2base 齐次矩阵。

        Args:
            R_g2b: (3,3) 旋转矩阵
            t_g2b: (3,) 或 (3,1) 平移向量，单位 m

        Returns:
            T (4,4) ndarray
        """
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R_g2b
        T[:3,  3] = t_g2b.flatten()[:3]
        return T

    # ═══════════════════════════════════════════════════════════════════════════
    #  文件 I/O
    # ═══════════════════════════════════════════════════════════════════════════

    def _save_intrinsics_cache(self):
        if self.K is None:
            return
        os.makedirs(_CALIB_DIR, exist_ok=True)
        data = {
            'K': self.K.tolist(),
            'D': self.D.tolist() if self.D is not None else [0, 0, 0, 0, 0],
        }
        with open(_INTRINSICS_CACHE, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

    def load_intrinsics_cache(self) -> bool:
        """从本地缓存文件加载相机内参。"""
        if not os.path.isfile(_INTRINSICS_CACHE):
            return False
        try:
            with open(_INTRINSICS_CACHE) as f:
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
                    f'[DepthUtils] 从缓存加载内参: fx={self.fx:.1f} fy={self.fy:.1f}'
                )
            return True
        except Exception as e:
            if self._node:
                self._node.get_logger().error(f'[DepthUtils] 加载内参缓存失败: {e}')
            return False

    # ═══════════════════════════════════════════════════════════════════════════
    #  深度热力图生成
    # ═══════════════════════════════════════════════════════════════════════════

    @staticmethod
    def depth_to_heatmap(depth_image: np.ndarray,
                          min_depth: int = 150,
                          max_depth: int = 2000) -> np.ndarray:
        """
        将 mono16 深度图转换为彩色热力图（BGR）。

        Args:
            depth_image: mono16 深度图 (mm)
            min_depth, max_depth: 色彩映射范围 (mm)

        Returns:
            BGR 彩色图 (uint8)
        """
        import cv2
        depth_clipped = np.clip(depth_image.astype(np.float32),
                                min_depth, max_depth)
        normalized = (depth_clipped - min_depth) / (max_depth - min_depth)
        normalized = (normalized * 255).astype(np.uint8)
        heatmap = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
        # 无效深度（0）显示为黑色
        heatmap[depth_image == 0] = [0, 0, 0]
        return heatmap


# ═══════════════════════════════════════════════════════════════════════════════
#  便捷函数（用于无 ROS2 节点场景）
# ═══════════════════════════════════════════════════════════════════════════════

_global_du: DepthUtils | None = None


def get_depth_utils() -> DepthUtils:
    """获取全局 DepthUtils 单例（用于快速访问）。"""
    global _global_du
    if _global_du is None:
        _global_du = DepthUtils()
        _global_du.load_intrinsics_cache()
        _global_du.load_hand_eye_calib()
    return _global_du


# ═══════════════════════════════════════════════════════════════════════════════
#  自测
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    print('=== depth_utils 自测 ===')

    # 测试内参缓存加载
    du = DepthUtils()
    if du.load_intrinsics_cache():
        print(f'✅ 内参: fx={du.fx:.1f}, fy={du.fy:.1f}, '
              f'cx={du.cx:.1f}, cy={du.cy:.1f}')
    else:
        print('⚠️  内参缓存不可用（需要先运行 aurora 驱动）')

    # 测试外参加载
    if du.load_hand_eye_calib():
        print(f'✅ 外参 T_cam2gripper 已加载')
        print(f'   t = {du.t_cam2gripper.flatten().round(4)} m')
        print(f'   R 正交性: '
              f'{np.linalg.norm(du.R_cam2gripper.T @ du.R_cam2gripper - np.eye(3)):.2e}')
    else:
        print('⚠️  外参不可用（需要先运行 eye_in_hand_calib_node 标定）')

    # 测试像素→3D
    if du.K is not None:
        p = du.pixel_to_3d(320, 240, 500)
        print(f'✅ pixel(320,240) + 500mm → cam_xyz = {p.round(4)} m')

    print('=== 自测完成 ===')
