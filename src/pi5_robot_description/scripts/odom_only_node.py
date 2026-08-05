#!/usr/bin/env python3
"""
轻量级里程计节点 (只读模式)

功能：
1. 从 STM32 读取里程计数据
2. 发布 /odom 话题
3. 发布 TF 变换 (odom -> base_footprint)
4. 不发送任何控制命令，手柄可自由控制小车
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
import serial
import math


class OdomOnlyNode(Node):
    def __init__(self):
        super().__init__('odom_only_node')
        
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        
        self.serial_port = None
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.05)
            self.get_logger().info(f'串口已打开 (只读模式): {port}')
        except Exception as e:
            self.get_logger().error(f'串口打开失败: {e}')
        
        self.distance_x = 0.0
        self.distance_y = 0.0
        self.angle = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.last_angle = 0.0
        self.last_distance_x = 0.0
        self.last_distance_y = 0.0
        self.last_time = self.get_clock().now()
        self.serial_active = False
        self.last_serial_time = self.get_clock().now()
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tf()
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.reset_odom_srv = self.create_service(Trigger, '/reset_odometry', self._reset_odometry_cb)
        self.odom_timer = self.create_timer(0.5, self._read_and_publish_odom)
        
        self.get_logger().info('Odom Only Node 已启动 (只读模式)')
    
    def _publish_static_tf(self):
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'base_footprint'
        t1.child_frame_id = 'base_link'
        t1.transform.translation.z = 0.043
        t1.transform.rotation.w = 1.0
        
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'laser'
        t2.transform.translation.x = 0.057
        t2.transform.translation.z = 0.14
        t2.transform.rotation.w = 1.0
        
        self.static_tf_broadcaster.sendTransform([t1, t2])
        self.get_logger().info('已发布静态TF')
    
    def _reset_odometry_cb(self, request, response):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(b'$RESET!\n')
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.angle = 0.0
        response.success = True
        response.message = '里程计已重置'
        return response
    
    def _read_and_publish_odom(self):
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        
        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(b'$QUERY!\n')
            for _ in range(5):
                response = self.serial_port.readline().decode('utf-8', errors='replace').strip()
                if response.startswith('ODOM:'):
                    parts = response[5:].split(',')
                    self.distance_x = float(parts[0].split('=')[1])
                    self.distance_y = float(parts[1].split('=')[1])
                    self.angle = -float(parts[2].split('=')[1]) * math.pi / 180.0
                    self.serial_active = True
                    self.last_serial_time = current_time
                    break
        except Exception:
            pass
        
        use_serial = (
            self.serial_active and
            (current_time - self.last_serial_time).nanoseconds * 1e-9 <= 0.5
        )
        
        if use_serial:
            dx_inc = self.distance_x - self.last_distance_x
            dy_inc = self.distance_y - self.last_distance_y
            self.pos_x += dx_inc * math.cos(self.angle) - dy_inc * math.sin(self.angle)
            self.pos_y += dx_inc * math.sin(self.angle) + dy_inc * math.cos(self.angle)
        
        self.last_time = current_time
        self.last_angle = self.angle
        self.last_distance_x = self.distance_x
        self.last_distance_y = self.distance_y
        
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        q = quaternion_from_euler(0, 0, self.angle)
        odom.pose.pose.position.x = float(self.pos_x)
        odom.pose.pose.position.y = float(self.pos_y)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.odom_pub.publish(odom)
        
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = self.pos_x
        t.transform.translation.y = self.pos_y
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.tf_broadcaster.sendTransform(t)
    
    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomOnlyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
