#!/usr/bin/env python3
"""
yolo_detect_node.py — YOLO 目标检测节点

订阅 Aurora 930 相机话题，运行 YOLO NCNN 推理，发布检测结果。
颜色判断复用现有 LAB 阈值系统。

用法（需在 yolo_env 虚拟环境中运行）:
  source ~/yolo_env/bin/activate
  python3 ~/ros2_ws/src/my_srv/scripts/yolo_detect_node.py

话题:
  订阅: /aurora/rgb/image_raw, /aurora/depth/image_raw
  发布: /yolo/detections (JSON), /yolo/image_result (调试画面)
"""

import os, sys, json, threading
import numpy as np, cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

# ── YOLO ──
from ultralytics import YOLO
MODEL_PATH = os.path.expanduser('~/models/best_ncnn_model')

# ── 中文形状 → YOLO类别 ──
SHAPE_TO_YOLO = {
    '长方体': 'cuboid', '正方体': 'screwdriver', '圆柱体': 'cube',
    '球体': 'sphere', '圆球': 'sphere', '螺丝刀': 'cylinder',
}

# ── LAB 颜色阈值 ──
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, _SCRIPT_DIR)

def _load_thresholds(filename):
    for d in (_SCRIPT_DIR, os.path.expanduser('~/ros2_ws/src/my_srv/scripts')):
        fp = os.path.join(d, filename)
        if os.path.exists(fp):
            with open(fp) as f:
                nums = []
                for line in f:
                    for s in line.strip().split():
                        if s: nums.append(int(s) if '.' not in s else float(s))
            return (int(nums[0]), int(nums[2]), int(nums[4])), (int(nums[1]), int(nums[3]), int(nums[5]))
    return None, None

_red_lo, _red_hi = _load_thresholds('red.txt')
_green_lo, _green_hi = _load_thresholds('green.txt')
_blue_lo, _blue_hi = _load_thresholds('blue.txt')


class YoloDetectNode(Node):

    def __init__(self):
        super().__init__('yolo_detect_node')

        self.latest_rgb = None
        self.latest_depth = None
        self._frame_lock = threading.Lock()

        # 相机内参（延迟加载）
        self.fx = self.fy = self.cx = self.cy = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Image, '/aurora/rgb/image_raw', self._rgb_cb, qos)
        self.create_subscription(Image, '/aurora/depth/image_raw', self._depth_cb, qos)
        self.pub_det = self.create_publisher(String, '/yolo/detections', 10)
        self.pub_img = self.create_publisher(Image, '/yolo/image_result', 10)

        self.model = YOLO(MODEL_PATH)
        self.get_logger().info(f'YOLO 模型已加载: {MODEL_PATH}')

        # 内参话题
        from sensor_msgs.msg import CameraInfo
        self.create_subscription(CameraInfo, '/aurora/rgb/camera_info', self._info_cb, qos)

    def _info_cb(self, msg):
        if self.fx is not None:
            return
        K = np.array(msg.k).reshape(3, 3)
        self.fx, self.fy = K[0, 0], K[1, 1]
        self.cx, self.cy = K[0, 2], K[1, 2]
        self.get_logger().info(f'内参: fx={self.fx:.1f} fy={self.fy:.1f}')

    def _rgb_cb(self, msg):
        try:
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            with self._frame_lock:
                self.latest_rgb = rgb
                self.latest_depth_stamp = None
        except Exception:
            pass

    def _depth_cb(self, msg):
        try:
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            with self._frame_lock:
                self.latest_depth = depth
        except Exception:
            pass

    def _get_color(self, rgb_frame, x1, y1, x2, y2):
        """取检测框中心区域 LAB 均值，判断颜色"""
        h, w = rgb_frame.shape[:2]
        cx1, cy1 = max(0, x1), max(0, y1)
        cx2, cy2 = min(w, x2), min(h, y2)
        if cx2 - cx1 < 10 or cy2 - cy1 < 10:
            return 'unknown'
        region = rgb_frame[cy1:cy2, cx1:cx2]
        lab = cv2.cvtColor(region, cv2.COLOR_BGR2LAB)
        mean_l, mean_a, mean_b = lab.mean(axis=(0, 1))

        # 与现有阈值系统一致
        if _red_lo and _red_lo[1] <= mean_a <= _red_hi[1] and _red_lo[2] <= mean_b <= _red_hi[2]:
            return 'red'
        if _green_lo and _green_lo[1] <= mean_a <= _green_hi[1] and _green_lo[2] <= mean_b <= _green_hi[2]:
            return 'green'
        if _blue_lo and _blue_lo[1] <= mean_a <= _blue_hi[1] and _blue_lo[2] <= mean_b <= _blue_hi[2]:
            return 'blue'
        return 'unknown'

    def detect(self):
        """运行一次检测，返回检测结果列表"""
        with self._frame_lock:
            rgb = self.latest_rgb.copy() if self.latest_rgb is not None else None
            depth = self.latest_depth.copy() if self.latest_depth is not None else None
        if rgb is None:
            return []

        results = self.model(rgb, conf=0.4, verbose=False)
        detections = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            shape = results[0].names[cls_id]
            conf = float(box.conf[0])
            xyxy = box.xyxy[0].tolist()
            x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
            pix_cx = (x1 + x2) // 2
            pix_cy = (y1 + y2) // 2
            area = (x2 - x1) * (y2 - y1)

            # 颜色
            color = self._get_color(rgb, x1, y1, x2, y2)

            # 深度
            depth_mm = 0
            if depth is not None:
                for r in range(0, 15):
                    for dy in range(-r, r + 1):
                        for dx in range(-r, r + 1):
                            px, py = pix_cx + dx, pix_cy + dy
                            if 0 <= py < depth.shape[0] and 0 <= px < depth.shape[1]:
                                d = int(depth[py, px])
                                if 100 < d < 4000:
                                    depth_mm = d
                                    break
                        if depth_mm: break
                    if depth_mm: break

            # 3D 坐标
            xyz = None
            if depth_mm > 0 and self.fx is not None:
                z = depth_mm / 1000.0
                x = (pix_cx - self.cx) * z / self.fx
                y = (pix_cy - self.cy) * z / self.fy
                xyz = [round(x, 4), round(y, 4), round(z, 4)]

            detections.append({
                'shape': shape, 'color': color, 'confidence': round(conf, 3),
                'pixel': [pix_cx, pix_cy], 'bbox': [x1, y1, x2, y2],
                'area': area, 'depth_mm': depth_mm, 'xyz_cam': xyz,
            })

        # 发布
        msg = String()
        msg.data = json.dumps(detections, ensure_ascii=False)
        self.pub_det.publish(msg)

        # 调试图像
        annotated = results[0].plot()
        self.pub_img.publish(self._to_imgmsg(annotated))

        return detections

    def _to_imgmsg(self, img):
        msg = Image()
        msg.height, msg.width = img.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.detect()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
