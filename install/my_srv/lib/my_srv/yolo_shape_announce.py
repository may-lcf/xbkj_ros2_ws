#!/usr/bin/env python3
"""
yolo_shape_announce.py — YOLO 形状识别 + 语音播报

检测到新物体时自动语音播报其形状。
更换物体会自动重新识别并播报。
画面清空后重置，等待下一个物体。

前提: yolo_detect_node.py 和 voice_synthesis_node.py 必须在运行
"""

import os, sys, json, time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SHAPE_CN = {
    'cube': '圆柱体', 'cuboid': '长方体', 'cylinder': '螺丝刀',
    'sphere': '球体', 'screwdriver': '正方体',
}


class ShapeAnnounceNode(Node):

    def __init__(self):
        super().__init__('yolo_shape_announce')

        self.announced_shape = None     # 已播报的形状
        self.stable_count = 0           # 连续检测到同一形状的帧数
        self.stable_threshold = 3       # 连续几帧相同才播报
        self.empty_count = 0            # 连续空帧数
        self.empty_threshold = 5        # 连续几帧空才认为物体被移走

        self.create_subscription(String, '/yolo/detections', self._det_cb, 10)
        self.pub_speak = self.create_publisher(String, '/speak_text', 10)

        self.get_logger().info('形状识别播报已启动，放置物体即可识别')

    def _det_cb(self, msg):
        try:
            dets = json.loads(msg.data)
        except Exception:
            return

        # 画面为空 → 计数，连续空够阈值后重置
        if not dets:
            self.empty_count += 1
            self.stable_count = 0
            if self.empty_count >= self.empty_threshold:
                if self.announced_shape is not None:
                    self.get_logger().info('物体已移走，等待下一个')
                self.announced_shape = None
            return

        self.empty_count = 0

        # 取最大面积的物体
        best = max(dets, key=lambda d: d.get('area', 0))
        shape = best['shape']
        conf = best['confidence']

        if conf < 0.5:
            return

        # 和已播报的形状一样 → 不播报
        if shape == self.announced_shape:
            self.stable_count = 0
            return

        # 新形状 → 累计稳定帧数
        self.stable_count += 1

        if self.stable_count >= self.stable_threshold:
            shape_cn = SHAPE_CN.get(shape, shape)
            color = best.get('color', '')
            if color and color != 'unknown':
                text = f'这是{color}{shape_cn}'
            else:
                text = f'这是{shape_cn}'

            speak_msg = String()
            speak_msg.data = text
            self.pub_speak.publish(speak_msg)

            self.announced_shape = shape
            self.stable_count = 0
            self.get_logger().info(f'📢 {text} (置信度 {conf:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = ShapeAnnounceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
