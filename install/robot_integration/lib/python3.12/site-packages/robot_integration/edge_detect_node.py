#!/usr/bin/env python3
"""
edge_detect_node.py — 深度相机桌面边缘检测节点

功能：
  1. 控制机械臂到观察姿态（俯视前方）
  2. 订阅深度相机图像，检测桌面边缘
  3. 检测到边缘后自动停止、转向、继续前进

边缘检测原理：
  - 桌面是平面，深度值稳定（如 300-500mm）
  - 边缘处深度突变（变为无效值或大幅增加）
  - 通过分析深度图的梯度变化检测边缘

话题：
  订阅:
    /aurora/depth/image_raw  (sensor_msgs/Image, mono16, mm)
    /odom                    (nav_msgs/Odometry) - 可选
  发布:
    /cmd_vel                 (geometry_msgs/Twist) - 小车速度控制
    /arm_command             (std_msgs/String) - 机械臂舵机指令
    /edge_detect/status      (std_msgs/String) - 状态信息

用法：
  ros2 run robot_integration edge_detect_node --ros-args -p serial_port:=/dev/ttyUSB0
  ros2 launch robot_integration edge_detect.launch.py
"""

import os
import sys
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# ── 状态定义 ──────────────────────────────────────────────────────────────────
STATE_IDLE = 'IDLE'                    # 空闲，等待启动
STATE_INIT_ARM = 'INIT_ARM'            # 初始化机械臂姿态
STATE_MOVING = 'MOVING'                # 小车前进中
STATE_EDGE_DETECTED = 'EDGE_DETECTED'  # 检测到边缘
STATE_TURNING = 'TURNING'              # 转向中
STATE_VERIFYING = 'VERIFYING'          # 验证新方向安全
STATE_STOPPED = 'STOPPED'              # 停止（异常或手动停止）

# ── 观察姿态参数 ──────────────────────────────────────────────────────────────
# kinematics_move(0, 120, 200, 1000, alpha_hint=-55)
# 对应舵机指令需要根据实际IK计算，这里使用近似值
OBSERVE_ARM_CMD = "#000P1500T1000!#001P1200T1000!#002P1600T1000!#003P0900T1000!#004P1500T1000!"

# 观察姿态（多舵机同时控制命令格式）
OBSERVE_ARM_CMD = "{#0P1500T1000!#1P1200T1000!#2P1600T1000!#3P0900T1000!#4P1500T1000!}"


class EdgeDetectNode(Node):
    """深度相机桌面边缘检测节点"""

    def __init__(self):
        super().__init__('edge_detect_node')

        # ── 状态变量 ──
        self.state = STATE_IDLE
        self.edge_direction = 'none'      # 'left', 'right', 'center', 'none'
        self.edge_distance_mm = 0         # 边缘距离
        self.edge_count = 0               # 连续检测到边缘的帧数
        self.no_edge_count = 0            # 连续未检测到边缘的帧数
        self.turn_start_time = None       # 转向开始时间
        self.turn_angle_accumulated = 0   # 已累积的转向角度 (度)
        self.turn_cmd = None              # 当前转向命令（持续发送用）
        self.mission_active = False       # 任务是否激活

        # ── 深度图 ──
        self.latest_depth = None
        self._depth_lock = threading.Lock()
        self.depth_ready = threading.Event()

        # ── 参数声明 ──
        self.declare_parameter('roi_y_start', 240)         # ROI起始行（图像下半部分）
        self.declare_parameter('roi_y_end', 480)           # ROI结束行
        self.declare_parameter('depth_min_mm', 100)        # 有效深度最小值
        self.declare_parameter('depth_max_mm', 800)        # 有效深度最大值
        self.declare_parameter('edge_threshold_mm', 100)   # 深度突变阈值
        self.declare_parameter('edge_confirm_frames', 3)   # 确认边缘的连续帧数
        self.declare_parameter('safe_confirm_frames', 5)   # 确认安全的连续帧数
        self.declare_parameter('move_speed', 0.15)         # 前进速度 (m/s)
        self.declare_parameter('turn_speed', 0.4)          # 转向速度 (rad/s)
        self.declare_parameter('turn_angle_deg', 90)       # 转向角度 (度)
        self.declare_parameter('max_turn_angle_deg', 360)  # 最大连续旋转角度 (度)
        self.declare_parameter('edge_detect_threshold', 0.6)  # 检测边缘阈值：有效深度比例低于此值→有边缘，停止前进
        self.declare_parameter('safe_detect_threshold', 0.85) # 安全方向阈值：有效深度比例高于此值→安全，停止旋转
        self.declare_parameter('turn_direction', 'left')      # 固定旋转方向: 'left' 或 'right'
        self.declare_parameter('auto_start', False)        # 自动启动任务

        # ── QoS ──
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ── 订阅深度图 ──
        self.create_subscription(
            Image, '/aurora/depth/image_raw',
            self._depth_callback, qos,
        )

        # ── 发布器 ──
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_cmd_pub = self.create_publisher(String, '/arm_command', 10)
        self.arm_task_pub = self.create_publisher(String, '/arm_task_command', 10)
        self.status_pub = self.create_publisher(String, '/edge_detect/status', 10)

        # ── 服务/话题订阅（用于外部控制）──
        self.create_subscription(
            String, '/edge_detect/control',
            self._control_callback, 10,
        )

        # ── 主控制循环（10Hz）──
        self.control_timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            '\033[1;36m[EdgeDetect]\033[0m 边缘检测节点已启动\n'
            '  发送 /edge_detect/control 话题启动任务:\n'
            '    ros2 topic pub --once /edge_detect/control std_msgs/String \'{data: "start"}\'\n'
            '  或设置 auto_start:=true 自动启动'
        )

        # ── 自动启动 ──
        if self.get_parameter('auto_start').value:
            self.get_logger().info('[EdgeDetect] auto_start=true，3秒后启动任务...')
            threading.Timer(3.0, self.start_mission).start()

    # ═══════════════════════════════════════════════════════════════════════════
    #  回调函数
    # ═══════════════════════════════════════════════════════════════════════════

    def _depth_callback(self, msg: Image):
        """深度图回调"""
        try:
            if not hasattr(self, '_depth_cb_logged'):
                self._depth_cb_logged = True
                self.get_logger().info(
                    f'[EdgeDetect] 收到深度图: {msg.width}x{msg.height}, encoding={msg.encoding}')
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            with self._depth_lock:
                self.latest_depth = depth
                if not self.depth_ready.is_set():
                    self.depth_ready.set()
        except Exception as e:
            self.get_logger().error(f'深度图处理错误: {e}')

    def _control_callback(self, msg: String):
        """外部控制指令回调"""
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            self.start_mission()
        elif cmd == 'stop':
            self.stop_mission()
        elif cmd == 'reset':
            self.reset_state()
        else:
            self.get_logger().warn(f'未知控制指令: {cmd}')

    # ═══════════════════════════════════════════════════════════════════════════
    #  深度图边缘检测算法
    # ═══════════════════════════════════════════════════════════════════════════

    def detect_edge(self, depth_image: np.ndarray, threshold: float = None) -> tuple:
        """
        检测桌面边缘（简化算法）

        算法：
          1. 将图像分成左、中、右三个区域
          2. 统计每个区域的有效深度像素比例
          3. 如果某个区域的有效深度比例很低，说明该方向有边缘

        Args:
            depth_image: mono16 深度图 (mm)
            threshold: 有效深度比例阈值（低于此值认为有边缘）
                       None 时使用 edge_detect_threshold 参数

        Returns:
            (direction, valid_ratio, edge_mask)
            direction: 'left', 'right', 'center', 'none'
            valid_ratio: 中心区域有效深度比例
            edge_mask: 简化掩码
        """
        roi_y_start = self.get_parameter('roi_y_start').value
        depth_min = self.get_parameter('depth_min_mm').value
        depth_max = self.get_parameter('depth_max_mm').value
        if threshold is None:
            threshold = self.get_parameter('edge_detect_threshold').value

        h, w = depth_image.shape

        # 确保ROI范围有效（使用图像下半部分）
        roi_y_start = min(roi_y_start, h - 2)

        # 提取ROI（图像下半部分）
        roi = depth_image[roi_y_start:, :]

        # 有效深度掩码
        valid_mask = (roi >= depth_min) & (roi <= depth_max)

        # 将图像分成左、中、右三个区域
        left_region = valid_mask[:, :w//3]
        center_region = valid_mask[:, w//3:2*w//3]
        right_region = valid_mask[:, 2*w//3:]

        # 计算每个区域的有效深度比例
        left_ratio = np.sum(left_region) / left_region.size if left_region.size > 0 else 0
        center_ratio = np.sum(center_region) / center_region.size if center_region.size > 0 else 0
        right_ratio = np.sum(right_region) / right_region.size if right_region.size > 0 else 0

        # 判断边缘方向
        # 如果中心区域有效比例很低，说明正前方有边缘
        if center_ratio < threshold:
            # 进一步判断是偏左还是偏右
            if left_ratio < right_ratio:
                direction = 'left'
            else:
                direction = 'right'
        elif left_ratio < threshold:
            direction = 'left'
        elif right_ratio < threshold:
            direction = 'right'
        else:
            direction = 'none'

        return direction, center_ratio, valid_mask

    # ═══════════════════════════════════════════════════════════════════════════
    #  控制逻辑
    # ═══════════════════════════════════════════════════════════════════════════

    def _control_loop(self):
        """主控制循环（10Hz）"""
        if not self.mission_active:
            return

        # 获取深度图
        with self._depth_lock:
            depth = self.latest_depth

        if depth is None:
            if not hasattr(self, '_no_depth_warned'):
                self._no_depth_warned = True
                self.get_logger().warn('[EdgeDetect] 控制循环运行中，但深度图为空！depth_callback未收到数据')
            return

        # 根据状态选择阈值：前进时用高阈值（敏感检测边缘），旋转时用低阈值（严格确认安全）
        if self.state == STATE_TURNING:
            threshold = self.get_parameter('safe_detect_threshold').value
        else:
            threshold = self.get_parameter('edge_detect_threshold').value
        direction, distance, edge_mask = self.detect_edge(depth, threshold)

        # 发布状态
        self._publish_status(direction, distance)

        # 状态机
        if self.state == STATE_INIT_ARM:
            # 等待机械臂到位
            pass  # 由 start_mission 中的定时器控制

        elif self.state == STATE_MOVING:
            # 前进中，检测边缘
            if direction != 'none':
                self.edge_count += 1
                self.no_edge_count = 0
                self.edge_direction = direction
                self.edge_distance_mm = distance

                if self.edge_count >= self.get_parameter('edge_confirm_frames').value:
                    self.get_logger().warn(
                        f'\033[1;33m[EdgeDetect]\033[0m 检测到边缘! '
                        f'方向={direction}, 距离={distance:.0f}mm'
                    )
                    self.state = STATE_EDGE_DETECTED
                    self._stop_car()
            else:
                self.edge_count = 0
                self.no_edge_count += 1
                self._move_forward()

        elif self.state == STATE_EDGE_DETECTED:
            # 检测到边缘，准备转向（新一轮转向，重置累积角度）
            self.get_logger().info(
                f'\033[1;36m[EdgeDetect]\033[0m 准备转向: {self.edge_direction} → '
                f'{"右转" if self.edge_direction == "left" else "左转"}'
            )
            self.turn_angle_accumulated = 0
            self.turn_start_time = None
            self._turn_away(self.edge_direction)

        elif self.state == STATE_TURNING:
            # 持续发送转向命令
            if self.turn_cmd is not None:
                self.cmd_vel_pub.publish(self.turn_cmd)

            # 累积转向角度（基于时间估算）
            now = time.time()
            if self.turn_start_time is not None:
                elapsed = now - self.turn_start_time
                turn_speed = self.get_parameter('turn_speed').value
                self.turn_angle_accumulated = np.degrees(turn_speed * elapsed)

            # 检查是否超过最大旋转角度
            max_angle = self.get_parameter('max_turn_angle_deg').value
            if self.turn_angle_accumulated >= max_angle:
                self.get_logger().warn(
                    f'\033[1;31m[EdgeDetect]\033[0m 已旋转 {self.turn_angle_accumulated:.0f}° '
                    f'超过最大角度 {max_angle}°，停止任务'
                )
                self._stop_turning()
                self.stop_mission()
                return

            # 实时检测：旋转中发现安全方向，立即停转
            if direction == 'none':
                self.get_logger().info(
                    f'\033[1;32m[EdgeDetect]\033[0m 旋转中发现安全方向，'
                    f'已转 {self.turn_angle_accumulated:.0f}°，立即停转'
                )
                self._stop_turning()
                self.state = STATE_VERIFYING
                self.no_edge_count = 0

        elif self.state == STATE_VERIFYING:
            # 验证新方向是否安全
            if direction == 'none':
                self.no_edge_count += 1
                if self.no_edge_count >= self.get_parameter('safe_confirm_frames').value:
                    self.get_logger().info(
                        '\033[1;32m[EdgeDetect]\033[0m 新方向安全，继续前进'
                    )
                    self.state = STATE_MOVING
                    self.edge_count = 0
                    self.turn_angle_accumulated = 0
            else:
                self.no_edge_count = 0
                self.get_logger().warn(
                    '\033[1;33m[EdgeDetect]\033[0m 新方向仍有边缘，继续转向'
                )
                # 继续旋转（不重置累积角度），重新进入转向状态
                self.edge_direction = direction
                self._turn_away(direction)

    def _move_forward(self):
        """小车前进"""
        speed = self.get_parameter('move_speed').value
        cmd = Twist()
        cmd.linear.x = speed
        self.cmd_vel_pub.publish(cmd)

    def _stop_car(self):
        """停止小车"""
        cmd = Twist()  # 全零，停止
        self.cmd_vel_pub.publish(cmd)

    def _turn_away(self, direction: str):
        """开始转向避开边缘（持续旋转直到找到安全方向）"""
        turn_speed = self.get_parameter('turn_speed').value
        turn_dir_name = self.get_parameter('turn_direction').value  # 'left' 或 'right'

        # 固定旋转方向
        cmd = Twist()
        if turn_dir_name == 'right':
            cmd.angular.z = -turn_speed  # 右转（负值）
            turn_dir = '右'
        else:
            cmd.angular.z = turn_speed   # 左转（正值）
            turn_dir = '左'

        self.turn_cmd = cmd
        self.cmd_vel_pub.publish(cmd)

        # 记录转向开始时间（用于计算累积角度）
        if self.turn_start_time is None:
            self.turn_start_time = time.time()
            self.turn_angle_accumulated = 0

        self.state = STATE_TURNING

        self.get_logger().info(
            f'\033[1;36m[EdgeDetect]\033[0m 开始转向{turn_dir}，'
            f'持续旋转直到找到安全方向（已转{self.turn_angle_accumulated:.0f}°）'
        )

    def _stop_turning(self):
        """停止转向"""
        self._stop_car()
        self.turn_cmd = None
        self.turn_start_time = None

        # 发送 [0,0,0] 清除 STM32 角度保持模式
        clear_cmd = Twist()
        self.cmd_vel_pub.publish(clear_cmd)
        self.get_logger().info('[EdgeDetect] 已发送清除角度保持命令 [0,0,0]')

    # ═══════════════════════════════════════════════════════════════════════════
    #  任务控制
    # ═══════════════════════════════════════════════════════════════════════════

    def start_mission(self):
        """启动边缘检测任务"""
        if self.mission_active:
            self.get_logger().warn('[EdgeDetect] 任务已在运行')
            return

        self.get_logger().info(
            '\033[1;36m[EdgeDetect]\033[0m 启动边缘检测任务\n'
            '  1. 停止小车...\n'
            '  2. 设置机械臂观察姿态...\n'
            '  3. 等待深度图就绪...\n'
            '  4. 开始前进...'
        )

        self.mission_active = True
        self.state = STATE_INIT_ARM

        # 先停止小车
        self._stop_car()

        # 在新线程中设置机械臂观察姿态（避免阻塞）
        def init_arm_thread():
            time.sleep(0.5)  # 等待小车停止
            self._set_arm_observe_pose()
            # 等待机械臂到位
            self.get_logger().info('[EdgeDetect] 等待机械臂到位...')
            time.sleep(5.0)  # 增加等待时间
            self._arm_ready()

        threading.Thread(target=init_arm_thread, daemon=True).start()

    def _arm_ready(self):
        """机械臂到位回调"""
        if not self.mission_active:
            return

        # 等待深度图就绪
        if not self.depth_ready.wait(timeout=10.0):
            self.get_logger().error('[EdgeDetect] 深度图未就绪，任务取消')
            self.stop_mission()
            return

        self.get_logger().info(
            '\033[1;32m[EdgeDetect]\033[0m 机械臂就绪，开始前进检测边缘'
        )
        self.state = STATE_MOVING
        self.edge_count = 0
        self.no_edge_count = 0

    def stop_mission(self):
        """停止任务"""
        self.get_logger().info('\033[1;31m[EdgeDetect]\033[0m 停止边缘检测任务')
        self.mission_active = False
        self.state = STATE_STOPPED
        self.turn_cmd = None
        self.turn_start_time = None
        self._stop_car()

    def reset_state(self):
        """重置状态"""
        self.get_logger().info('[EdgeDetect] 重置状态')
        self._stop_car()
        self.state = STATE_IDLE
        self.mission_active = False
        self.edge_count = 0
        self.no_edge_count = 0
        self.edge_direction = 'none'
        self.turn_angle_accumulated = 0
        self.turn_cmd = None
        self.turn_start_time = None

    # ═══════════════════════════════════════════════════════════════════════════
    #  机械臂控制
    # ═══════════════════════════════════════════════════════════════════════════

    def _set_arm_observe_pose(self):
        """设置机械臂观察姿态（直接发送舵机指令）"""
        self.get_logger().info(
            f'[EdgeDetect] 设置机械臂观察姿态: {OBSERVE_ARM_CMD}'
        )

        # 通过 /arm_command 话题发送舵机指令
        cmd = String()
        cmd.data = OBSERVE_ARM_CMD
        self.arm_cmd_pub.publish(cmd)

        self.get_logger().info('[EdgeDetect] 已发送机械臂观察姿态命令')

    # ═══════════════════════════════════════════════════════════════════════════
    #  状态发布
    # ═══════════════════════════════════════════════════════════════════════════

    def _publish_status(self, direction: str, distance: float):
        """发布状态信息"""
        import json
        status = {
            'state': self.state,
            'edge_direction': direction,
            'edge_distance_mm': float(distance),
            'mission_active': self.mission_active,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


# ═══════════════════════════════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetectNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_mission()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
