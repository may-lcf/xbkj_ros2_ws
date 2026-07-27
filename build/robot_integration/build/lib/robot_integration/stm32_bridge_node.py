#!/usr/bin/env python3
"""
STM32 统一通信桥接节点

功能：
1. 通过单个串口与 STM32 通信
2. 发布里程计数据到 /odom
3. 订阅 /cmd_vel 控制小车运动
4. 订阅 /arm_command 控制机械臂
5. 发布 TF 变换

STM32 协议（V4.0 底盘）：
- 速度指令: [vx,vy,omega] 浮点数(m/s, rad/s)
- 查询里程计: $QUERY! -> ODOM:X=...,Y=...,YAW=...
- 重置里程计: $RESET! -> ODOM reset OK
- 舵机指令: #idPpulseTtime!
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
import serial
import threading
import time
import math
import json


class STM32BridgeNode(Node):
    def __init__(self):
        super().__init__('stm32_bridge_node')
        
        # ========== 参数声明 ==========
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.001)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        
        # ========== 获取参数 ==========
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # ========== 串口初始化 ==========
        self.serial_port = None
        self.serial_active = False
        self.last_serial_time = self.get_clock().now()
        self._serial_lock = threading.Lock()  # 串口写入互斥锁
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=timeout)
            self.get_logger().info(f'串口已打开: {port} @ {baudrate}bps')
            # 发送初始化命令(新底盘协议)
            self.serial_port.write(b"$RESET!\n")
            import time; time.sleep(0.1)
            self.serial_port.write(b"$RESET!\n")
        except Exception as e:
            self.get_logger().error(f'串口打开失败: {e}')
        
        # ========== 里程计数据 ==========
        self._arm_sending = False  # 机械臂命令发送标志
        self.distance_x = 0.0
        self.distance_y = 0.0
        self.angle = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.last_angle = 0.0
        self.last_distance_x = 0.0
        self.last_distance_y = 0.0
        self.v_x = 0.0
        self.v_z = 0.0
        self.last_time = self.get_clock().now()
        
        # ========== 小车控制目标 ==========
        self.target_speed_x = 0.0
        self.target_speed_y = 0.0
        self.target_gyro_z = 0.0
        self.output_speed_x = 0.0
        self.output_speed_y = 0.0
        self.output_gyro_z = 0.0
        self.last_message = None
        
        # ========== TF 广播器 ==========
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()
        
        # ========== ROS2 订阅 ==========
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.arm_cmd_sub = self.create_subscription(
            String, '/arm_command', self.arm_command_callback, 10)
        self.arm_joint_sub = self.create_subscription(
            JointState, '/arm_joint_command', self.arm_joint_callback, 10)
        
        # ========== ROS2 发布 ==========
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.arm_feedback_pub = self.create_publisher(String, '/arm_feedback', 10)

        # ========== ROS2 服务 ==========
        self.reset_odom_srv = self.create_service(
            Trigger, '/reset_odometry', self._reset_odometry_cb)
        
        # ========== 定时器 ==========
        self.odom_timer = self.create_timer(0.5, self.publish_odom)  # 2Hz
        
        # ========== 线程 ==========
        self.running = True
        self.control_thread = threading.Thread(target=self.car_control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        self.get_logger().info('STM32 Bridge Node 已启动')
    
    def publish_static_tf(self):
        """发布静态TF变换"""
        # base_footprint -> base_link
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'base_footprint'
        t1.child_frame_id = 'base_link'
        t1.transform.translation.z = 0.043
        t1.transform.rotation.w = 1.0
        
        # base_link -> laser
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'laser'
        t2.transform.translation.x = 0.057
        t2.transform.translation.z = 0.14
        t2.transform.rotation.w = 1.0
        
        self.static_tf_broadcaster.sendTransform([t1, t2])
        self.get_logger().info('已发布静态TF: base_footprint -> base_link -> laser')
    
    def cmd_vel_callback(self, msg):
        """小车速度控制回调"""
        self.target_speed_x = msg.linear.x
        self.target_speed_y = msg.linear.y
        self.target_gyro_z = msg.angular.z
    
    def arm_command_callback(self, msg):
        """机械臂命令回调 - 暂停小车控制，确保STM32收到"""
        if self.serial_port and self.serial_port.is_open:
            cmd = msg.data + '\n'
            self._arm_sending = True  # 暂停小车控制循环
            with self._serial_lock:
                # 停车
                self.serial_port.write(b'[0,0,0]\n')
                time.sleep(0.05)
                # 发送机械臂命令3次
                for _ in range(3):
                    self.serial_port.write(cmd.encode('utf-8'))
                    time.sleep(0.05)
            self._arm_sending = False  # 恢复小车控制循环
            self.get_logger().info(f'发送机械臂命令: {cmd.strip()}')
    
    def arm_joint_callback(self, msg):
        """机械臂关节控制回调 - 转换为舵机指令"""
        if self.serial_port and self.serial_port.is_open:
            # JointState -> 舵机指令
            # 假设 joint 对应舵机 0-5
            for i, pos in enumerate(msg.position):
                # 将弧度转换为PWM值 (500-2500)
                # 中位1500对应0度，范围±1000对应±135度
                pwm = int(1500 + pos * 1000 / (math.pi * 0.75))
                pwm = max(500, min(2500, pwm))
                cmd = f'#{i:03d}P{pwm:04d}T1000!'
                with self._serial_lock:
                    self.serial_port.write((cmd + '\n').encode('utf-8'))
            self.get_logger().info(f'发送关节命令: {msg.name}')
    
    def car_control_loop(self):
        """小车控制循环 - 10ms周期"""
        while rclpy.ok() and self.running:
            if not self.serial_port or not self.serial_port.is_open:
                time.sleep(0.01)
                continue

            # 机械臂命令发送时暂停小车控制
            if self._arm_sending:
                time.sleep(0.01)
                continue
            
            # 速度缩放和限幅(新底盘直接接受m/s和rad/s浮点数,无需缩放)
            self.output_gyro_z = self.target_gyro_z
            self.output_speed_x = self.target_speed_x
            self.output_speed_y = self.target_speed_y
            # 速度限幅(新底盘直接接受m/s和rad/s浮点数)
            self.output_speed_x = max(-1.0, min(1.0, self.output_speed_x))
            self.output_speed_y = max(-1.0, min(1.0, self.output_speed_y))
            self.output_gyro_z = max(-3.0, min(3.0, self.output_gyro_z))

            
            # 构建控制消息
            message = f"[{self.output_speed_x:.3f},{self.output_speed_y:.3f},{self.output_gyro_z:.3f}]"
            # 发送控制命令（持续发送，STM32需要持续接收命令）
            with self._serial_lock:
                self.serial_port.write((message + '\n').encode('utf-8'))
            self.last_message = message
            
            time.sleep(0.01)
    
    def _reset_odometry_cb(self, request, response):
        """重置里程计服务回调 — 同时清零STM32和Pi端"""
        # 1. 清零 STM32 端
        if self.serial_port and self.serial_port.is_open:
            with self._serial_lock:
                self.serial_port.write(b"$RESET!\n")
        # 2. 清零 Pi 端
        self.distance_x = 0.0
        self.distance_y = 0.0
        self.angle = 0.0
        self.last_distance_x = 0.0
        self.last_distance_y = 0.0
        self.last_angle = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        response.success = True
        response.message = "里程计已重置(STM32+Pi)"
        self.get_logger().info('[Odom] 里程计已重置')
        return response

    def publish_odom(self):
        """主动查询并发布里程计(新底盘协议: $QUERY! -> ODOM:X=...,Y=...,YAW=...)"""
        if not self.serial_port or not self.serial_port.is_open:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9

        # 主动查询里程计
        with self._serial_lock:
            self.serial_port.timeout = 0.05  # 50ms for odom query
            self.serial_port.reset_input_buffer()  # 清空回显
            self.serial_port.write(b"$QUERY!\n")
            try:
                # 读取多行,跳过STM32回显(res: ...),找ODOM响应
                for _ in range(5):
                    response = self.serial_port.readline().decode("utf-8", errors="replace").strip()
                    if response.startswith("ODOM:"):
                        parts = response[5:].split(",")
                        x_val = float(parts[0].split("=")[1])
                        y_val = float(parts[1].split("=")[1])
                        yaw_deg = float(parts[2].split("=")[1])
                        self.distance_x = x_val
                        self.distance_y = y_val
                        self.angle = -yaw_deg * math.pi / 180.0
                        self.serial_active = True
                        self.last_serial_time = current_time
                        break
            except Exception:
                pass
            self.serial_port.timeout = 0.001  # restore timeout


        # 计算里程计(增量积分到世界坐标)
        use_serial = (
            self.serial_port is not None and
            self.serial_active and
            (current_time - self.last_serial_time).nanoseconds * 1e-9 <= 0.5
        )

        if use_serial:
            dx_inc = self.distance_x - self.last_distance_x
            dy_inc = self.distance_y - self.last_distance_y
            self.v_x = dx_inc / dt if dt > 0 else 0.0
            self.v_z = (self.angle - self.last_angle) / dt if dt > 0 else 0.0
            self.pos_x += dx_inc * math.cos(self.angle) - dy_inc * math.sin(self.angle)
            self.pos_y += dx_inc * math.sin(self.angle) + dy_inc * math.cos(self.angle)

        self.last_time = current_time
        self.last_angle = self.angle
        self.last_distance_x = self.distance_x
        self.last_distance_y = self.distance_y

        # 发布里程计消息
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        q = quaternion_from_euler(0, 0, self.angle)
        odom.pose.pose.position.x = float(self.pos_x)
        odom.pose.pose.position.y = float(self.pos_y)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom.twist.twist.linear.x = float(self.v_x)
        odom.twist.twist.angular.z = float(self.v_z)

        self.odom_pub.publish(odom)

        # 发布TF变换
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = self.pos_x
            t.transform.translation.y = self.pos_y
            t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        """节点销毁时清理资源"""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(b'[0.000,0.000,0.000]\n')  # 停车
            time.sleep(0.1)
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STM32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
