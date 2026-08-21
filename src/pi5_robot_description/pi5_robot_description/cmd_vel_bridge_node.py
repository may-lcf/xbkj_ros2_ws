#!/usr/bin/env python3
"""
cmd_vel 桥接节点 (导航专用 - 双职责版)

功能：
1. 订阅 /cmd_vel_smoothed (Nav2 速度指令)
2. 将速度指令转发给 STM32
3. 定时查询 STM32 里程计数据，发布 /odom 和 TF (含速度信息)
4. 超时看门狗：0.5s 没收到指令自动发零速
5. 退出时自动发零速

关键设计：
  - STM32 串口查询: 10Hz (ROS2 定时器)
  - odom TF 发布: 100Hz (独立线程，不受 executor 阻塞)
  - 时间戳回退 100ms，确保 AMCL 的 scan 时间戳总能找到匹配的 TF
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import serial
import threading
import math
import re
import time
import os
import fcntl


# 串口锁文件路径 — 防止多个进程同时打开同一串口
SERIAL_LOCK_DIR = '/tmp'


class CmdVelBridgeNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_smoothed')
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('odom_frame_id', 'odom')

        # 速度指令符号反转 (诊断用：如果小车往反方向走，设置对应轴为 true)
        self.declare_parameter("invert_vx", False)
        self.declare_parameter("invert_vy", False)
        self.declare_parameter("invert_wz", False)

        self.invert_vx = self.get_parameter("invert_vx").value
        self.invert_vy = self.get_parameter("invert_vy").value
        self.invert_wz = self.get_parameter("invert_wz").value

        # 诊断日志计数器
        self._cmd_count = 0
        self._cmd_log_interval = 50  # 每50条命令打印一次诊断

        self.get_logger().info(f"符号反转设置: invert_vx={self.invert_vx}, invert_vy={self.invert_vy}, invert_wz={self.invert_wz}")
        self.declare_parameter('base_frame_id', 'base_footprint')

        self._port = self.get_parameter('serial_port').value
        self._baudrate = self.get_parameter('baudrate').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.timeout_sec = self.get_parameter('timeout_sec').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value

        # 串口互斥锁 + 进程级锁文件
        self._serial_lock = threading.Lock()
        self._lock_file = None
        self.serial_port = None
        self._reconnect_count = 0

        # 尝试获取进程级串口锁 (防止多个 ROS2 launch 同时打开同一串口)
        if not self._acquire_serial_lock():
            self.get_logger().error(f'串口 {self._port} 已被其他进程占用！请先关闭占用进程。')
            raise RuntimeError(f'Serial port {self._port} is locked by another process')

        self._open_serial()

        # 断线重连定时器
        self._reconnect_timer = self.create_timer(3.0, self._try_reconnect)

        # 里程计数据 (由 _query_odom 更新，由 _tf_publish_loop 读取)
        self._odom_lock = threading.Lock()
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.angle = 0.0
        self._vx_body = 0.0
        self._vy_body = 0.0
        self._wz = 0.0
        self._prev_x = 0.0
        self._prev_y = 0.0
        self._prev_angle = 0.0
        self._prev_odom_time = self.get_clock().now()
        self._odom_initialized = False

        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # odom diagnostic counters
        self._odom_query_ok_count = 0
        self._odom_query_fail_count = 0
        # cmd_vel 订阅
        self._last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_sub = self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_cb, 10)

        # 机械臂命令订阅 (通过同一串口转发给 STM32)
        self.arm_cmd_sub = self.create_subscription(String, '/arm_cmd', self._arm_cmd_cb, 10)

        # STM32 串口查询定时器 (10Hz) — ROS2 executor 管理
        self.odom_query_timer = self.create_timer(0.1, self._query_odom)

        # 超时看门狗
        self._watchdog_timer = self.create_timer(0.2, self._watchdog_cb)

        # odom TF 发布线程 (100Hz, 独立线程，不受 executor 阻塞)
        self._tf_thread_stop = threading.Event()
        self._tf_thread = threading.Thread(target=self._tf_publish_loop, daemon=True)
        self._tf_thread.start()

        self.get_logger().info(f'cmd_vel 桥接节点已启动 (双职责: 订阅 {cmd_vel_topic} + 查询里程计)')

    def _acquire_serial_lock(self):
        """获取进程级串口锁文件，防止多个实例同时打开同一串口"""
        lock_name = self._port.replace('/', '_').replace('.', '_')
        lock_path = os.path.join(SERIAL_LOCK_DIR, f'.serial_lock{lock_name}')
        try:
            self._lock_file = open(lock_path, 'w')
            fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._lock_file.write(str(os.getpid()))
            self._lock_file.flush()
            self.get_logger().info(f'串口锁文件已获取: {lock_path}')
            return True
        except (IOError, OSError) as e:
            self.get_logger().error(f'无法获取串口锁 {lock_path}: {e}')
            if self._lock_file:
                self._lock_file.close()
                self._lock_file = None
            return False

    def _release_serial_lock(self):
        """释放进程级串口锁文件"""
        if self._lock_file:
            try:
                fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_UN)
                self._lock_file.close()
            except Exception:
                pass
            self._lock_file = None

    def _open_serial(self):
        try:
            self.serial_port = serial.Serial(self._port, self._baudrate, timeout=0.05)
            self._reconnect_count = 0
            self.get_logger().info(f'串口已打开: {self._port} (timeout=50ms)')
            return True
        except Exception as e:
            self.serial_port = None
            self.get_logger().warn(f'串口打开失败: {e}')
            return False

    def _close_serial(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None

    def _handle_serial_error(self, context: str):
        self.get_logger().warn(f'{context}: 串口断开，将在 3 秒后重连')
        self._close_serial()

    def _try_reconnect(self):
        if self.serial_port and self.serial_port.is_open:
            return
        self._reconnect_count += 1
        if self._reconnect_count % 10 == 1:
            self.get_logger().info(f'正在尝试重连串口... (第 {self._reconnect_count} 次)')
        self._open_serial()

    # ========== STM32 里程计查询 (10Hz, ROS2 定时器) ==========

    def _query_odom(self):
        """Query STM32 odometry (v3: fixed serial handling)"""
        with self._serial_lock:
            if not self.serial_port or not self.serial_port.is_open:
                self._odom_query_fail_count += 1
                if self._odom_query_fail_count % 50 == 1:
                    self.get_logger().warn("Odom skip: serial not open (fail=%d)" % self._odom_query_fail_count)
                return
            try:
                # 不再调用 reset_input_buffer() — 它会丢弃未读数据导致数据丢失
                # 改用读取并丢弃旧数据的方式清理缓冲区
                if self.serial_port.in_waiting > 0:
                    self.serial_port.read(self.serial_port.in_waiting)

                self.serial_port.write(b"$QUERY!\n")
                self.serial_port.flush()
                for attempt in range(10):
                    response = self.serial_port.readline().decode("utf-8", errors="replace").strip()
                    if not response:
                        continue
                    if not response.startswith("ODOM:"):
                        continue
                    match = re.search(r"X=([\-\d.]+).*?Y=([\-\d.]+).*?YAW=([\-\d.]+)", response)
                    if not match:
                        continue
                    try:
                        new_x = float(match.group(1))
                        new_y = float(match.group(2))
                        new_angle = float(match.group(3)) * math.pi / 180.0
                        current_time = self.get_clock().now()
                        with self._odom_lock:
                            if self._odom_initialized:
                                dt = (current_time - self._prev_odom_time).nanoseconds * 1e-9
                                if dt > 0.001:
                                    dx_world = new_x - self.pos_x
                                    dy_world = new_y - self.pos_y
                                    d_angle = new_angle - self.angle
                                    while d_angle > math.pi: d_angle -= 2.0 * math.pi
                                    while d_angle < -math.pi: d_angle += 2.0 * math.pi
                                    cos_a = math.cos(new_angle)
                                    sin_a = math.sin(new_angle)
                                    self._vx_body = (dx_world * cos_a + dy_world * sin_a) / dt
                                    self._vy_body = (-dx_world * sin_a + dy_world * cos_a) / dt
                                    self._wz = d_angle / dt
                            self.pos_x = new_x
                            self.pos_y = new_y
                            self.angle = new_angle
                            self._prev_odom_time = current_time
                            self._odom_initialized = True
                        self._odom_query_fail_count = 0
                        self._odom_query_ok_count += 1
                        if self._odom_query_ok_count % 100 == 0:
                            self.get_logger().info("[Odom #%d] X=%.3f Y=%.3f Yaw=%.1f Vx=%.3f Vy=%.3f Wz=%.3f" % (
                                self._odom_query_ok_count, new_x, new_y, math.degrees(new_angle),
                                self._vx_body, self._vy_body, self._wz))
                        break
                    except (ValueError, IndexError, KeyError) as e:
                        self.get_logger().warn("Odom parse fail: %s -> %s" % (response[:40], e))
                        continue
                else:
                    self._odom_query_fail_count += 1
                    if self._odom_query_fail_count % 10 == 1:
                        self.get_logger().error("Odom query fail x%d" % self._odom_query_fail_count)
                    # 连续失败 50 次才认为串口真正断开 (而非偶尔的超时)
                    if self._odom_query_fail_count >= 50:
                        self._handle_serial_error("odom query (连续%d次失败)" % self._odom_query_fail_count)
                        self._odom_query_fail_count = 0
            except serial.SerialException:
                self._handle_serial_error("odom query")
            except Exception as e:
                self._odom_query_fail_count += 1
                self.get_logger().error("Odom exception: %s: %s" % (type(e).__name__, e))

    # ========== odom TF 发布线程 (100Hz, 独立线程) ==========

    def _tf_publish_loop(self):
        """独立线程：100Hz 发布 odom TF，不受 ROS2 executor 阻塞

        使用 time.sleep 而非 ROS2 定时器，确保发布频率稳定。
        时间戳回退 100ms，保证 AMCL 的 scan 时间戳总能找到匹配的 TF。
        """
        # 等待第一次 odom 数据
        while not self._odom_initialized and not self._tf_thread_stop.is_set():
            time.sleep(0.1)

        self.get_logger().info('odom TF 发布线程已启动 (100Hz)')

        while not self._tf_thread_stop.is_set():
            loop_start = time.monotonic()

            if self._odom_initialized:
                # 直接使用当前时刻发布 TF/odom，保证与 scan/AMCL 时间戳一致
                now = self.get_clock().now()

                with self._odom_lock:
                    x = self.pos_x
                    y = self.pos_y
                    angle = self.angle
                    vx = self._vx_body
                    vy = self._vy_body
                    wz = self._wz

                q = quaternion_from_euler(0, 0, angle)
                stamp = now.to_msg()

                # 发布 TF (odom -> base_footprint)
                t = TransformStamped()
                t.header.stamp = stamp
                t.header.frame_id = self.odom_frame_id
                t.child_frame_id = self.base_frame_id
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                self.tf_broadcaster.sendTransform(t)

                # 发布 odom topic
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = self.odom_frame_id
                odom.child_frame_id = self.base_frame_id
                odom.pose.pose.position.x = float(x)
                odom.pose.pose.position.y = float(y)
                odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                odom.pose.covariance = [
                    0.1, 0, 0, 0, 0, 0,
                    0, 0.1, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0.05,
                ]
                odom.twist.twist.linear.x = float(vx)
                odom.twist.twist.linear.y = float(vy)
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

            # 精确 10ms 间隔
            elapsed = time.monotonic() - loop_start
            sleep_time = max(0, 0.01 - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ========== cmd_vel 转发 ==========

    def _cmd_vel_cb(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        # 最小速度钳位：非零但过小的速度指令替换为最小可驱动速度
        # 解决 RPP 控制器 curvature constraint 绕过 min_linear_vel 的问题
        MIN_LINEAR_VEL = 0.05   # 5cm/s
        MIN_ANGULAR_VEL = 0.05  # ~3°/s
        linear_speed = math.hypot(vx, vy)
        if 0.0 < linear_speed < MIN_LINEAR_VEL:
            scale = MIN_LINEAR_VEL / linear_speed
            vx *= scale
            vy *= scale
        if 0.0 < abs(omega) < MIN_ANGULAR_VEL:
            omega = MIN_ANGULAR_VEL if omega > 0 else -MIN_ANGULAR_VEL

        # 符号反转 (诊断用)
        if self.invert_vx:
            vx = -vx
        if self.invert_vy:
            vy = -vy
        if self.invert_wz:
            omega = -omega

        cmd = f'[{vx:.3f},{vy:.3f},{omega:.3f}]'.encode('utf-8')
        with self._serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.write(cmd)
                except serial.SerialException:
                    self._handle_serial_error('cmd_vel 发送')
                except Exception as e:
                    self.get_logger().warn(f'发送失败: {e}')

        self._cmd_count += 1
        if self._cmd_count % self._cmd_log_interval == 0:
            self.get_logger().info(
                f'[诊断 #{self._cmd_count}] 原始: ({msg.linear.x:.3f},{msg.linear.y:.3f},{msg.angular.z:.3f}) '
                f'-> 发送: ({vx:.3f},{vy:.3f},{omega:.3f})')

        self._last_cmd_vel_time = self.get_clock().now()

    # ========== 机械臂命令转发 ==========

    def _arm_cmd_cb(self, msg: String):
        """将机械臂命令通过同一串口转发给 STM32"""
        cmd = msg.data
        with self._serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.write(cmd.encode('utf-8'))
                    self.serial_port.flush()
                    self.get_logger().info(f'[Arm] 已转发: {cmd[:40]}...')
                except serial.SerialException:
                    self._handle_serial_error('arm_cmd 发送')
                except Exception as e:
                    self.get_logger().warn(f'[Arm] 发送失败: {e}')

    def _watchdog_cb(self):
        now = self.get_clock().now()
        elapsed = (now - self._last_cmd_vel_time).nanoseconds * 1e-9
        if elapsed > self.timeout_sec:
            with self._serial_lock:
                if self.serial_port and self.serial_port.is_open:
                    try:
                        self.serial_port.write(b'[0.000,0.000,0.000]')
                    except serial.SerialException:
                        self._handle_serial_error('看门狗')
                    except Exception:
                        pass

    def _send_zero(self):
        with self._serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.write(b'[0.000,0.000,0.000]')
                except Exception:
                    pass

    def destroy_node(self):
        self._tf_thread_stop.set()
        if self._tf_thread.is_alive():
            self._tf_thread.join(timeout=1.0)
        self._send_zero()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self._release_serial_lock()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
