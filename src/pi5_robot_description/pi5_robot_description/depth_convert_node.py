#!/usr/bin/env python3
"""
深度图单位转换节点
将 Aurora mono16 (毫米) 转换为 32FC1 (米)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np


class DepthConvertNode(Node):
    def __init__(self):
        super().__init__('depth_convert_node')
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('input_topic', '/aurora/depth/image_raw')
        self.declare_parameter('output_topic', '/depth/image_converted')

        self.scale = self.get_parameter('depth_scale').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(Image, output_topic, 10)
        self.sub = self.create_subscription(Image, input_topic, self.depth_cb, 10)
        self.get_logger().info(f'Depth convert: {input_topic} -> {output_topic} (scale={self.scale})')

    def depth_cb(self, msg):
        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.is_bigendian = 0
        out.step = msg.width * 4  # float32 = 4 bytes

        if msg.encoding == 'mono16':
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            arr_f = arr.astype(np.float32) * self.scale
            arr_f[arr == 0] = float('nan')  # 0值设为NaN
            out.encoding = '32FC1'
            out.data = arr_f.tobytes()
        elif msg.encoding == '32FC1':
            out = msg  # 已经是浮点，直接传递
        else:
            self.get_logger().warn(f'未知编码: {msg.encoding}')
            return

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthConvertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
