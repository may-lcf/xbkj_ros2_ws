#!/usr/bin/env python3
"""
YOLO 区域检测节点

加载 NCNN 格式的 YOLO 模型，检测画面中的物体并按区域分类。
到达目标区域后，发布该区域检测到的物品列表供 nav_executor 播报。

用法（需在 yolo_env 虚拟环境中运行）:
  source ~/yolo_env/bin/activate
  python3 ~/ros2_ws/src/pi5_robot_description/pi5_robot_description/yolo_zone_detect_node.py

话题:
  订阅: /aurora/rgb/image_raw
  发布: /yolo/zone_detections (JSON)
"""

import os
import json
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


# ═══════════════════════════════════════════════════════════════════════════════
#  模型配置
# ═══════════════════════════════════════════════════════════════════════════════

MODEL_PATH = os.path.expanduser('~/models_1/best_ncnn_model')

# 类别中文名映射
CLASS_NAMES_CN = {
    'orange': '橘子',
    'pear': '梨',
    'pomegranate': '石榴',
    'apple': '苹果',
    'milk': '牛奶',
    'coffee': '咖啡',            
    'iced_black_tea': '冰红茶',
    'cocktail': '鸡尾酒',
    'football': '足球',
    'goal': '球门',
}

# 区域分类
ZONE_OBJECTS = {
    '水果区': ['orange', 'pear', 'pomegranate', 'apple'],
    '饮品区': ['milk', 'coffee', 'iced_black_tea', 'cocktail'],
    '足球区': ['football', 'goal'],
}

# 反向映射：物体 -> 区域
OBJECT_TO_ZONE = {}
for zone, objects in ZONE_OBJECTS.items():
    for obj in objects:
        OBJECT_TO_ZONE[obj] = zone


class YoloZoneDetectNode(Node):

    def __init__(self):
        super().__init__('yolo_zone_detect_node')

        self.latest_rgb = None
        self._frame_lock = threading.Lock()

        # 当前检测结果（按区域分类）
        self.zone_detections = {}  # {zone: [物体名, ...]}
        self._det_lock = threading.Lock()

        # 订阅相机
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/aurora/rgb/image_raw', self._rgb_cb, qos)

        # 发布检测结果
        self.pub_det = self.create_publisher(String, '/yolo/zone_detections', 10)

        # 加载模型
        self.get_logger().info(f'正在加载 YOLO 模型: {MODEL_PATH}')
        from ultralytics import YOLO
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info('YOLO 模型加载完成')

        # 后台检测线程
        self._running = True
        self._det_thread = threading.Thread(target=self._detect_loop, daemon=True)
        self._det_thread.start()

        self.get_logger().info('YOLO 区域检测节点已启动')

    def _rgb_cb(self, msg):
        try:
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            with self._frame_lock:
                self.latest_rgb = rgb
        except Exception:
            pass

    def _detect_loop(self):
        """后台检测循环"""
        while self._running and rclpy.ok():
            with self._frame_lock:
                rgb = self.latest_rgb.copy() if self.latest_rgb is not None else None
            if rgb is None:
                time.sleep(0.5)
                continue

            try:
                results = self.model(rgb, conf=0.4, verbose=False)
                zone_result = {}

                for box in results[0].boxes:
                    cls_id = int(box.cls[0])
                    class_name = results[0].names[cls_id]
                    conf = float(box.conf[0])
                    zone = OBJECT_TO_ZONE.get(class_name, '未知区域')

                    if zone not in zone_result:
                        zone_result[zone] = []
                    cn_name = CLASS_NAMES_CN.get(class_name, class_name)
                    if cn_name not in zone_result[zone]:
                        zone_result[zone].append(cn_name)

                with self._det_lock:
                    self.zone_detections = zone_result

                # 发布结果
                msg = String()
                msg.data = json.dumps(zone_result, ensure_ascii=False)
                self.pub_det.publish(msg)

            except Exception as e:
                self.get_logger().warn(f'检测异常: {e}')

            time.sleep(0.3)  # 约3fps

    def get_zone_objects(self, zone_name):
        """获取指定区域当前检测到的物体列表"""
        with self._det_lock:
            return list(self.zone_detections.get(zone_name, []))

    def get_all_detections(self):
        """获取所有区域的检测结果"""
        with self._det_lock:
            return dict(self.zone_detections)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloZoneDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
