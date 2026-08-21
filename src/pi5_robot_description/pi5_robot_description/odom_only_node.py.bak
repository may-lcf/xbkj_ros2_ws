#!/usr/bin/env python3
"""
轻量级里程计节点 (主动查询模式 - 用于建图)

功能：
1. 主动向 STM32 发送 $QUERY! 命令查询里程计数据
2. 发布 /odom 话题
3. 发布 TF 变换 (odom -> base_footprint)
4. 不发送任何控制命令（不订阅 cmd_vel）

注意：此节点仅在建图模式下使用。
      导航模式下由 cmd_vel_bridge_node 负责里程计+速度。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
import serial
import threading
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

        # 串口互斥锁
        self._serial_lock = threading.Lock()
        self.serial_port = None
        self._reconnect_count = 0

        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.01)
            self._reconnect_count = 0
            self.get_logger().info(f'串口已打开: {port}')
        except Exception as e:
            self.serial_port = None
            self.get_logger().error(f'串口打开失败: {e}')

        # 断线重连定时器 (每 3 秒检查一次)
        self._reconnect_timer = self.create_timer(3.0, self._try_reconnect)

        # ROS2 里程计位置
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.angle = 0.0

        # 速度估计
        self._prev_x = 0.0
        self._prev_y = 0.0
        self._prev_angle = 0.0
        self._prev_odom_time = self.get_clock().now()
        self._odom_initialized = False

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        # self._publish_static_tf()  # TF now in URDF

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 里程计查询定时器 (10Hz，主动查询)
        self.odom_timer = self.create_timer(0.1, self._query_and_publish_odom)

        self.get_logger().info('Odom Only Node started (active query mode)')

    def _open_serial(self):
        """打开串口"""
        try:
            port = self.get_parameter('serial_port').value
            baudrate = self.get_parameter('baudrate').value
            self.serial_port = serial.Serial(port, baudrate, timeout=0.01)
            self._reconnect_count = 0
            self.get_logger().info(f'串口已重新打开: {port}')
            return True
        except Exception as e:
            self.serial_port = None
            self.get_logger().warn(f'串口打开失败: {e}')
            return False

    def _close_serial(self):
        """安全关闭串口"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None

    def _handle_serial_error(self, context: str):
        """处理串口 I/O 错误"""
        self.get_logger().warn(f'{context}: 串口断开，将在 3 秒后重连')
        self._close_serial()

    def _try_reconnect(self):
        """定时尝试重连串口"""
        if self.serial_port and self.serial_port.is_open:
            return
        self._reconnect_count += 1
        if self._reconnect_count % 10 == 1:
            self.get_logger().info(f'正在尝试重连串口... (第 {self._reconnect_count} 次)')
        self._open_serial()

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
        t2.transform.translation.x = 0.109
        t2.transform.translation.z = 0.1565
        t2.transform.rotation.z = -0.7071
        t2.transform.rotation.w = 0.7071

        self.static_tf_broadcaster.sendTransform([t1, t2])
        self.get_logger().info('已发布静态TF')

    def _query_and_publish_odom(self):
        """主动查询 STM32 里程计并发布 odom + TF"""
        new_data = False

        with self._serial_lock:
            if not self.serial_port or not self.serial_port.is_open:
                return
            try:
                # 主动发送查询命令
                self.serial_port.write(b'$QUERY!\n')
                # 非阻塞读取：尝试读取响应
                for _ in range(3):
                    response = self.serial_port.readline().decode('utf-8', errors='replace').strip()
                    if response.startswith('ODOM:'):
                        parts = response[5:].split(',')
                        self.pos_x = float(parts[0].split('=')[1])
                        self.pos_y = float(parts[1].split('=')[1])
                        self.angle = float(parts[2].split('=')[1]) * math.pi / 180.0
                        new_data = True
                        break
            except serial.SerialException:
                self._handle_serial_error('odom query')
                return
            except Exception:
                pass

        current_time = self.get_clock().now()

        # 速度估计（差分法）
        vx_body = 0.0
        vy_body = 0.0
        wz = 0.0

        if new_data:
            dt = (current_time - self._prev_odom_time).nanoseconds * 1e-9
            if dt > 0.001:
                dx_world = self.pos_x - self._prev_x
                dy_world = self.pos_y - self._prev_y
                d_angle = self.angle - self._prev_angle

                while d_angle > math.pi:
                    d_angle -= 2.0 * math.pi
                while d_angle < -math.pi:
                    d_angle += 2.0 * math.pi

                cos_a = math.cos(self.angle)
                sin_a = math.sin(self.angle)
                vx_body = (dx_world * cos_a + dy_world * sin_a) / dt
                vy_body = (-dx_world * sin_a + dy_world * cos_a) / dt
                wz = d_angle / dt

            self._prev_x = self.pos_x
            self._prev_y = self.pos_y
            self._prev_angle = self.angle
            self._prev_odom_time = current_time
            self._odom_initialized = True

        q = quaternion_from_euler(0, 0, self.angle)

        # 发布 TF (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = self.pos_x
        t.transform.translation.y = self.pos_y
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.tf_broadcaster.sendTransform(t)

        # 发布 odom
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = float(self.pos_x)
        odom.pose.pose.position.y = float(self.pos_y)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom.pose.covariance = [
            0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.05,
        ]
        odom.twist.twist.linear.x = float(vx_body)
        odom.twist.twist.linear.y = float(vy_body)
        odom.twist.twist.angular.z = float(wz)
        odom.twist.covariance = [
            0.05, 0, 0, 0, 0, 0,
            0, 0.05, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.02,
        ]
        self.odom_pub.publish(odom)

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
