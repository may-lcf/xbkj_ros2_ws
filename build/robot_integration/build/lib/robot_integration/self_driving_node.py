#!/usr/bin/env python3
"""
self_driving_node.py — 深度学习自动驾驶节点

双模式巡线状态机 + YOLO交通标志识别
- 状态A: 巡黄线（HSV黄色阈值 + PID）
- 状态B: 巡黑线（HSV黑色阈值 + PID）
- 交通标志: 左转/右转/红灯/绿灯
- 状态转换: A→B（右转标志+完成右转）, B→A（转向计数/里程计阈值）

订阅:
  /aurora/rgb/image_raw   — 相机图像
  /aurora/depth/image_raw — 深度图像
  /odom                   — 里程计
  /yolo/detections        — YOLO检测结果(JSON)
  /self_driving/control   — 外部控制指令

发布:
  /cmd_vel                — 速度指令
  /self_driving/status    — 状态(JSON)
  /self_driving/debug_image — 调试图像
"""

import math
import json
import threading
import time

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from std_srvs.srv import Trigger

# ── 状态定义 ──
STATE_IDLE = 'IDLE'
STATE_INIT_ARM = 'INIT_ARM'
STATE_FOLLOWING = 'FOLLOWING'
STATE_TURNING = 'TURNING'
STATE_PARKING = 'PARKING'
STATE_STOPPED = 'STOPPED'

# ── 机械臂观察位置（眼在手上，深度相机安装在夹爪）──
OBSERVE_ARM_CMD = '{#0P1550T1000!#1P1800T1000!#2P2000T1000!#3P0800T1000!#4P1500T1000!}'

# ── 巡线模式 ──
MODE_A_YELLOW = 'YELLOW'
MODE_B_BLACK = 'BLACK'


class SimplePID:
    """复用 line_follow_node 的 PID 控制器"""
    def __init__(self, kp, ki, kd, il=0.3):
        self.target = 0.0
        self.last_err = 0.0
        self.sum_err = 0.0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.il = il
        self.hist = []
        self.win = 3

    def compute(self, actual):
        err = self.target - actual
        self.hist.append(err)
        if len(self.hist) > self.win:
            self.hist.pop(0)
        se = np.mean(self.hist)
        kp = self.kp * (1.5 if abs(se) > 0.05 else 1.0)
        self.sum_err += se
        self.sum_err = max(-self.il, min(self.il, self.sum_err))
        d = se - self.last_err
        self.last_err = se
        return kp * se + self.ki * self.sum_err + self.kd * d

    def reset(self):
        self.last_err = 0.0
        self.sum_err = 0.0
        self.hist.clear()


class SelfDrivingNode(Node):
    def __init__(self):
        super().__init__('self_driving_node')

        # ── 图像数据 ──
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self._frame_lock = threading.Lock()
        self.depth_ready = threading.Event()
        self.rgb_ready = threading.Event()

        # ── 状态机 ──
        self.state = STATE_IDLE
        self.line_mode = MODE_A_YELLOW
        self.mission_active = False
        self.stopped_by_red = False  # 红灯停车标志

        # ── 里程计 ──
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # ── 巡线 ──
        self.pid = SimplePID(0.5, 0.0, 0.1)
        self.lost_count = 0
        self._last_steer = 0.0

        # ── 交通标志 ──
        self._right_count = 0
        self._left_count = 0
        self._turn_direction = None  # 'right' 或 'left'
        self._found_red_this_frame = False  # 本帧是否检测到红灯

        # ── 状态B 转向计数 + 2秒窗口 ──
        self.turn_count = 0
        self._turn_window_active = False
        self._turn_window_start = 0.0

        # ── 固定角度转向 ──
        self._turn_start = 0.0
        self._turn_ang_z = 0.0
        self._turn23_last = 0.0  # 黄线阈值转向冷却时间记录
        self._turn_is_traffic_sign = False  # 是否是交通标志触发的转向

        # ── 停车动作 ──
        self._parking_start = 0.0

        # ── 里程计重置服务客户端 ──
        self._reset_odom_client = self.create_client(Trigger, '/reset_odometry')

        # ── 参数声明 ──
        self._declare_params()

        # ── PID 初始化 ──
        self._init_pid_for_mode(self.line_mode)

        # ── QoS ──
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        # ── 订阅 ──
        self.create_subscription(Image, '/aurora/rgb/image_raw', self._rgb_cb, qos)
        self.create_subscription(Image, '/aurora/depth/image_raw', self._depth_cb, qos)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(String, '/yolo/detections', self._yolo_cb, 10)
        self.create_subscription(String, '/self_driving/control', self._ctrl_cb, 10)

        # ── 发布 ──
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_cmd_pub = self.create_publisher(String, '/arm_command', 10)
        self.status_pub = self.create_publisher(String, '/self_driving/status', 10)
        self.debug_pub = self.create_publisher(Image, '/self_driving/debug_image', 10)

        # ── 主循环定时器 (10Hz) ──
        self.create_timer(0.1, self._loop)

        # ── 状态日志 (1Hz) ──
        self.create_timer(1.0, self._log_status)

        self.get_logger().info('[SelfDriving] 节点已启动')

        # ── auto_start ──
        if self.get_parameter('auto_start').value:
            self.get_logger().info('[SelfDriving] auto_start in 3s...')
            threading.Timer(3.0, self.start_mission).start()

    # ══════════════════════════════════════════
    # 参数声明
    # ══════════════════════════════════════════

    def _declare_params(self):
        params = [
            ('auto_start', False),
            ('publish_debug_image', True),
            # ── 状态A 黄线参数 ──
            ('yellow_hsv_h_min', 0),   ('yellow_hsv_h_max', 88),
            ('yellow_hsv_s_min', 61),  ('yellow_hsv_s_max', 255),
            ('yellow_hsv_v_min', 173), ('yellow_hsv_v_max', 255),
            ('yellow_line_target_error', -0.60),
            ('yellow_pid_kp', 0.70), ('yellow_pid_ki', 0.00), ('yellow_pid_kd', 0.15),
            ('yellow_move_speed', 0.17),
            ('yellow_roi_y_start', 176), ('yellow_roi_y_end', 400),
            ('yellow_roi_x_left', 44), ('yellow_roi_x_right', 320),
            # ── 状态B 黑线参数 ──
            ('black_hsv_h_min', 0),   ('black_hsv_h_max', 180),
            ('black_hsv_s_min', 0),   ('black_hsv_s_max', 102),
            ('black_hsv_v_min', 19),  ('black_hsv_v_max', 255),
            ('black_line_target_error', -0.015),
            ('black_pid_kp', 0.80), ('black_pid_ki', 0.00), ('black_pid_kd', 0.20),
            ('black_move_speed', 0.12),
            ('black_roi_y_start', 248), ('black_roi_y_end', 400),
            ('black_roi_x_left', 110), ('black_roi_x_right', 530),
            # ── 公共参数 ──
            ('ground_depth_min_mm', 150), ('ground_depth_max_mm', 600),
            ('morph_kernel_size', 5),
            ('morph_open_iter', 2), ('morph_close_iter', 2),
            ('scan_rows', 10), ('min_line_pixels', 5),
            ('max_steering', 0.50), ('integral_limit', 0.3),
            ('max_lost_frames', 30),
            # ── 交通标志参数 ──
            ('yellow_turn_confirm_frames', 6),
            ('black_turn_confirm_frames', 8),
            ('turn_angular_z', 0.28),
            ('turn_duration', 2.992),
            ('odom_threshold', 1.3),
            ('red_area_threshold', 4.2),
            ('stop_area_min', 2.1),
            ('stop_area_max', 5.0),
            ('stop_move_duration', 2.5),
            ('green_area_threshold', 7.6),
            ('turn_window_duration', 2.0),
        ]
        for name, default in params:
            self.declare_parameter(name, default)

    def _p(self, name):
        """获取参数值的快捷方式"""
        return self.get_parameter(name).value

    # ══════════════════════════════════════════
    # PID 初始化（根据模式切换参数）
    # ══════════════════════════════════════════

    def _init_pid_for_mode(self, mode):
        if mode == MODE_A_YELLOW:
            self.pid = SimplePID(
                self._p('yellow_pid_kp'),
                self._p('yellow_pid_ki'),
                self._p('yellow_pid_kd'),
                self._p('integral_limit'))
            self.pid.target = self._p('yellow_line_target_error')
        else:
            self.pid = SimplePID(
                self._p('black_pid_kp'),
                self._p('black_pid_ki'),
                self._p('black_pid_kd'),
                self._p('integral_limit'))
            self.pid.target = self._p('black_line_target_error')
        self._last_steer = 0.0
        self.lost_count = 0

    # ══════════════════════════════════════════
    # 回调函数
    # ══════════════════════════════════════════

    def _rgb_cb(self, msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._frame_lock:
                self.latest_rgb = rgb
            if not self.rgb_ready.is_set():
                self.rgb_ready.set()
        except Exception:
            pass

    def _depth_cb(self, msg):
        try:
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            with self._frame_lock:
                self.latest_depth = depth
            if not self.depth_ready.is_set():
                self.depth_ready.set()
        except Exception:
            pass

    def _odom_cb(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny, cosy) * 180.0 / math.pi

    def _yolo_cb(self, msg):
        """处理YOLO检测结果 — 交通标志"""
        if not self.mission_active:
            return
        try:
            detections = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return

        # 每次收到 YOLO 消息都打印（调试用，确认消息到达）
        if detections:
            shapes = [d.get('shape', '?') for d in detections]
            self.get_logger().info(f'[YOLO] 收到 {len(detections)} 个检测: {shapes}')

        img_area = 640 * 480
        red_thresh = self._p('red_area_threshold')
        green_thresh = self._p('green_area_threshold')
        confirm_n = self._p('yellow_turn_confirm_frames') if self.line_mode == MODE_A_YELLOW \
            else self._p('black_turn_confirm_frames')

        found_red = False
        found_right = False
        found_left = False

        for det in detections:
            shape = det.get('shape', '')
            area = det.get('area', 0)
            area_pct = area / img_area * 100.0

            # ── 红灯: 面积 > 阈值 → 立即停车（任何状态都生效）──
            if shape == 'red_light' and area_pct > red_thresh:
                found_red = True
                self.get_logger().info(
                    f'[YOLO] red_light conf={det.get("confidence",0):.2f} area={area_pct:.1f}%')
                if not self.stopped_by_red:
                    self.stopped_by_red = True
                    self.cmd_vel_pub.publish(Twist())
                    self.get_logger().warn(f'[SelfDriving] 红灯停车 (area={area_pct:.1f}%)')

            # ── 绿灯: 面积 > 阈值 → 恢复行驶 ──
            elif shape == 'green_light' and area_pct > green_thresh:
                self.get_logger().info(
                    f'[YOLO] green_light conf={det.get("confidence",0):.2f} area={area_pct:.1f}%')
                if self.stopped_by_red:
                    self.stopped_by_red = False
                    self.get_logger().info(f'[SelfDriving] 绿灯通行 (area={area_pct:.1f}%)')

            # ── 转向标志: 仅在 FOLLOWING 状态下处理 ──
            elif shape == 'turn_right':
                found_right = True
                self.get_logger().info(
                    f'[YOLO] turn_right conf={det.get("confidence",0):.2f} count={self._right_count+1}/{confirm_n}')
                if self.state == STATE_FOLLOWING:
                    self._right_count += 1
                    if self._right_count >= confirm_n:
                        self._handle_turn_sign('right')
                        self._right_count = 0

            elif shape == 'turn_left':
                found_left = True
                self.get_logger().info(
                    f'[YOLO] turn_left conf={det.get("confidence",0):.2f} count={self._left_count+1}/{confirm_n}')
                if self.state == STATE_FOLLOWING:
                    self._left_count += 1
                    if self._left_count >= confirm_n:
                        self._handle_turn_sign('left')
                        self._left_count = 0

            # ── 停车标志: 面积在范围内 → 执行停车动作 ──
            elif shape == 'stop':
                stop_min = self._p('stop_area_min')
                stop_max = self._p('stop_area_max')
                if stop_min <= area_pct <= stop_max:
                    self.get_logger().warn(
                        f'[YOLO] stop conf={det.get("confidence",0):.2f} area={area_pct:.1f}% → 执行停车')
                    if self.state == STATE_FOLLOWING:
                        self._start_parking()

        # ── 红灯消失自动恢复 ──
        if not found_red and self.stopped_by_red:
            self.stopped_by_red = False
            self.get_logger().info('[SelfDriving] 红灯消失，恢复行驶')

        # ── 如果没有任何交通标志检测，减少日志噪音 ──
        if not any(d.get('shape', '') in ('turn_left', 'turn_right', 'red_light', 'green_light') for d in detections):
            pass  # 不打印，避免日志刷屏

        # ── 每帧结束重置未出现的转向计数（防误触发）──
        if not found_right:
            self._right_count = 0
        if not found_left:
            self._left_count = 0

    def _handle_turn_sign(self, direction):
        """处理转向标志确认"""
        if self.state != STATE_FOLLOWING:
            return

        if self.line_mode == MODE_A_YELLOW:
            # 状态A: 只处理右转 → 切换到状态B
            if direction == 'right':
                self.get_logger().warn('[SelfDriving] 状态A: 右转确认 → 执行右转')
                self._start_turning(direction, transition_to_b=True)
        else:
            # 状态B: 左转/右转都处理 → 转向计数
            self.get_logger().warn(f'[SelfDriving] 状态B: {direction}转确认 → 执行转向')
            self._start_turning(direction, transition_to_b=False)

    def _start_turning(self, direction, transition_to_b, is_traffic_sign=True):
        """开始固定角度转向"""
        self.state = STATE_TURNING
        self._turn_direction = direction
        self._turn_transition_to_b = transition_to_b
        self._turn_is_traffic_sign = is_traffic_sign
        self._turn_start = time.time()
        ang_z = self._p('turn_angular_z')
        # 右转标志牌: -0.28（与阈值转向方向一致）
        # 左转标志牌: +0.28（与阈值转向方向相反）
        self._turn_ang_z = -ang_z if direction == 'right' else ang_z
        self.get_logger().info(
            f'[SelfDriving] 转向开始: {direction}, ang_z={self._turn_ang_z:+.3f}, '
            f'交通标志={is_traffic_sign}')

    def _start_parking(self):
        """开始停车动作"""
        self.state = STATE_PARKING
        self._parking_start = time.time()
        self.get_logger().warn('[SelfDriving] 停车动作开始: 横移2秒')

    def _do_parking(self):
        """停车动作: 以[0, -0.15, 0]速度移动2秒后停止"""
        elapsed = time.time() - self._parking_start
        duration = self._p('stop_move_duration')
        if elapsed < duration:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.linear.y = -0.15
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
        else:
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().warn('[SelfDriving] 停车动作完成，任务停止')
            self.stop_mission()

    def _ctrl_cb(self, msg):
        c = msg.data.strip().lower()
        if c == 'start':
            self.start_mission()
        elif c == 'stop':
            self.stop_mission()
        elif c == 'reset':
            self.reset_state()

    # ══════════════════════════════════════════
    # 主循环
    # ══════════════════════════════════════════

    def _loop(self):
        if not self.mission_active:
            return

        with self._frame_lock:
            rgb = self.latest_rgb
            depth = self.latest_depth
        if rgb is None:
            return

        if self.state == STATE_INIT_ARM:
            return  # 等待机械臂初始化完成

        if self.state == STATE_FOLLOWING:
            self._do_follow(rgb, depth)

        elif self.state == STATE_TURNING:
            self._do_turning()

        elif self.state == STATE_PARKING:
            self._do_parking()

        # 2秒窗口判断（状态B，窗口激活时持续判断）
        if self._turn_window_active:
            if time.time() - self._turn_window_start > self._p('turn_window_duration'):
                self._turn_window_active = False
                self.get_logger().info('[SelfDriving] 2秒窗口关闭，继续巡线')
            else:
                self._check_b_to_a_transition()

    # ══════════════════════════════════════════
    # 巡线逻辑（复用 line_follow_node）
    # ══════════════════════════════════════════

    def _do_follow(self, rgb, depth):
        """巡线行驶"""
        # 红灯停车时不发速度指令
        if self.stopped_by_red:
            return

        result = self._detect_line(rgb, depth)
        self._publish_status(result)

        if self._p('publish_debug_image'):
            self._publish_debug(rgb, result)

        if result['line_found']:
            self.lost_count = 0
            err = result['lateral_error']
            ms = self._p('max_steering')
            speed = self._p('yellow_move_speed') if self.line_mode == MODE_A_YELLOW \
                else self._p('black_move_speed')

            # ── 黄线模式阈值转向（与 line_follow_node 一致）──
            # 当 err > -0.27 且距上次阈值转向 > 5秒时，触发固定角度左转
            if self.line_mode == MODE_A_YELLOW and self.state == STATE_FOLLOWING \
                    and err > -0.27 and time.time() - self._turn23_last > 5.0:
                self.get_logger().warn(
                    f'[SelfDriving] 黄线阈值转向: err={err:+.3f} > -0.27, 固定转向')
                self._turn23_last = time.time()
                self._start_turning('right', transition_to_b=False, is_traffic_sign=False)
                return

            # 黑线模式死区
            if self.line_mode == MODE_B_BLACK and abs(err) <= 0.007:
                st = self._last_steer
            else:
                st = self.pid.compute(err)
                st = max(-ms, min(ms, st))
            self._last_steer = st

            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = st
            self.cmd_vel_pub.publish(cmd)
        else:
            self.lost_count += 1
            if self.lost_count >= self._p('max_lost_frames'):
                self.get_logger().error('[SelfDriving] 线丢失过久，停止')
                self.stop_mission()
                return
            # 丢失时停车
            self.cmd_vel_pub.publish(Twist())

    def _detect_line(self, rgb, depth):
        """HSV巡线检测（复用 line_follow_node 逻辑）"""
        h, w = rgb.shape[:2]

        # 根据模式选择HSV参数
        if self.line_mode == MODE_A_YELLOW:
            lo = np.array([self._p('yellow_hsv_h_min'), self._p('yellow_hsv_s_min'),
                           self._p('yellow_hsv_v_min')], dtype=np.uint8)
            hi = np.array([self._p('yellow_hsv_h_max'), self._p('yellow_hsv_s_max'),
                           self._p('yellow_hsv_v_max')], dtype=np.uint8)
        else:
            lo = np.array([self._p('black_hsv_h_min'), self._p('black_hsv_s_min'),
                           self._p('black_hsv_v_min')], dtype=np.uint8)
            hi = np.array([self._p('black_hsv_h_max'), self._p('black_hsv_s_max'),
                           self._p('black_hsv_v_max')], dtype=np.uint8)

        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lo, hi)

        # 深度地面过滤
        if depth is not None:
            dmin = self._p('ground_depth_min_mm')
            dmax = self._p('ground_depth_max_mm')
            gm = ((depth >= dmin) & (depth <= dmax)).astype(np.uint8) * 255
            mask = cv2.bitwise_and(mask, gm)

        # ROI（根据模式选择）
        if self.line_mode == MODE_A_YELLOW:
            rys = min(self._p('yellow_roi_y_start'), h - 10)
            rye = min(self._p('yellow_roi_y_end'), h - 1)
            rxl = max(0, min(self._p('yellow_roi_x_left'), w - 10))
            rxr = max(rxl + 10, min(self._p('yellow_roi_x_right'), w))
        else:
            rys = min(self._p('black_roi_y_start'), h - 10)
            rye = min(self._p('black_roi_y_end'), h - 1)
            rxl = max(0, min(self._p('black_roi_x_left'), w - 10))
            rxr = max(rxl + 10, min(self._p('black_roi_x_right'), w))
        mask[:rys, :] = 0
        mask[rye:, :] = 0
        mask[:, :rxl] = 0
        mask[:, rxr:] = 0

        # 形态学
        ks = self._p('morph_kernel_size')
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (ks, ks))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k,
                                iterations=self._p('morph_open_iter'))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k,
                                iterations=self._p('morph_close_iter'))

        # 扫描行
        nr = self._p('scan_rows')
        mp = self._p('min_line_pixels')
        pts = []
        for y in np.linspace(rys, rye, nr, dtype=int):
            px = np.where(mask[y, :] > 0)[0]
            if len(px) >= mp:
                pts.append((int(np.mean(px)), y))

        # 加权误差
        icx = w / 2.0
        le = 0.0
        found = False
        if pts:
            tw = 0.0
            we = 0.0
            for i, (cx, y) in enumerate(pts):
                wt = i + 1
                we += (cx - icx) / icx * wt
                tw += wt
            le = we / tw if tw > 0 else 0.0
            found = True

        lr = np.sum(mask > 0) / mask.size if mask.size > 0 else 0.0
        return {
            'line_mask': mask, 'line_points': pts,
            'lateral_error': le, 'line_found': found,
            'line_ratio': lr, 'roi': (rxl, rxr, rys, rye)
        }

    # ══════════════════════════════════════════
    # 转向逻辑
    # ══════════════════════════════════════════

    def _do_turning(self):
        """固定角度转向执行"""
        elapsed = time.time() - self._turn_start
        duration = self._p('turn_duration')

        if elapsed < duration:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = self._turn_ang_z
            self.cmd_vel_pub.publish(cmd)
            deg = elapsed * abs(self._turn_ang_z) * 180.0 / math.pi
            self.get_logger().info(
                f'[SelfDriving] 转向中 {self._turn_direction}... {deg:.0f}deg {elapsed:.1f}/{duration:.1f}s')
        else:
            # 转向完成
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info(f'[SelfDriving] 转向完成: {self._turn_direction}')
            self.state = STATE_FOLLOWING
            self.pid.reset()

            if self._turn_transition_to_b:
                # 状态A → 状态B: 重置里程计，切换到黑线模式
                self._reset_odometry()
                self._switch_mode(MODE_B_BLACK)
            elif self._turn_is_traffic_sign:
                # 交通标志转向: 计数 + 开启2秒窗口
                self.turn_count += 1
                self._turn_window_active = True
                self._turn_window_start = time.time()
                self.get_logger().info(
                    f'[SelfDriving] 交通标志转向计数={self.turn_count}, 开启2秒判断窗口')
                # 立即判断一次
                self._check_b_to_a_transition()
            # else: 黄线阈值转向，不计数，不开窗口，直接继续巡线

    # ══════════════════════════════════════════
    # 状态B → 状态A 判断
    # ══════════════════════════════════════════

    def _check_b_to_a_transition(self):
        """判断是否满足 B→A 转换条件"""
        if self.line_mode != MODE_B_BLACK:
            return

        # 条件一: 转向计数 >= 2
        if self.turn_count >= 2:
            self.get_logger().warn('[SelfDriving] 条件一: 转向计数=2 → 切回黄线')
            self._switch_mode(MODE_A_YELLOW)
            return

        # 条件二: 转向计数 == 1 且 odom_x >= 1.3m
        if self.turn_count == 1 and self.odom_x >= self._p('odom_threshold'):
            self.get_logger().warn(
                f'[SelfDriving] 条件二: 计数=1 且 odom_x={self.odom_x:.3f}m >= 1.3m → 切回黄线')
            self._switch_mode(MODE_A_YELLOW)
            return

    # ══════════════════════════════════════════
    # 模式切换
    # ══════════════════════════════════════════

    def _switch_mode(self, mode):
        """切换巡线模式"""
        old = self.line_mode
        self.line_mode = mode
        self.turn_count = 0
        self._turn_window_active = False
        self._init_pid_for_mode(mode)
        self.get_logger().warn(f'[SelfDriving] 模式切换: {old} → {mode}')

    def _reset_odometry(self):
        """重置里程计 — 调用 stm32_bridge_node 的服务"""
        if self._reset_odom_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self._reset_odom_client.call_async(req)
            self.get_logger().info('[SelfDriving] 里程计重置请求已发送')
        else:
            self.get_logger().error('[SelfDriving] /reset_odometry 服务不可用')

    # ══════════════════════════════════════════
    # 任务控制
    # ══════════════════════════════════════════

    def start_mission(self):
        if self.mission_active:
            return
        self.get_logger().info('[SelfDriving] 任务启动')
        self.mission_active = True
        self.state = STATE_INIT_ARM
        self.line_mode = MODE_A_YELLOW
        self.stopped_by_red = False
        self.turn_count = 0
        self._turn_window_active = False
        self._right_count = 0
        self._left_count = 0
        self.lost_count = 0
        self._init_pid_for_mode(MODE_A_YELLOW)
        self.cmd_vel_pub.publish(Twist())
        # 机械臂初始化线程
        threading.Thread(target=self._init_arm_sequence, daemon=True).start()

    def _init_arm_sequence(self):
        """机械臂初始化：移到观察位置，等待相机就绪后开始巡线"""
        time.sleep(0.5)
        msg = String()
        msg.data = OBSERVE_ARM_CMD
        self.arm_cmd_pub.publish(msg)
        self.get_logger().info(f'[SelfDriving] 机械臂移到观察位置: {OBSERVE_ARM_CMD}')
        time.sleep(5.0)
        if not self.mission_active:
            return
        # 等待相机就绪（与 line_follow_node 一致）
        if not self.depth_ready.wait(timeout=10.0):
            self.get_logger().error('[SelfDriving] 深度相机未就绪，停止')
            self.stop_mission()
            return
        if not self.rgb_ready.wait(timeout=5.0):
            self.get_logger().error('[SelfDriving] RGB相机未就绪，停止')
            self.stop_mission()
            return
        self.state = STATE_FOLLOWING
        self.get_logger().info('[SelfDriving] 机械臂就绪，开始巡线')

    def stop_mission(self):
        self.get_logger().info('[SelfDriving] 任务停止')
        self.mission_active = False
        self.state = STATE_STOPPED
        self.cmd_vel_pub.publish(Twist())

    def reset_state(self):
        self.cmd_vel_pub.publish(Twist())
        self.state = STATE_IDLE
        self.mission_active = False
        self.line_mode = MODE_A_YELLOW
        self.stopped_by_red = False
        self.turn_count = 0
        self._turn_window_active = False
        self._right_count = 0
        self._left_count = 0
        self.lost_count = 0
        self._init_pid_for_mode(MODE_A_YELLOW)

    # ══════════════════════════════════════════
    # 状态发布 & 调试
    # ══════════════════════════════════════════

    def _publish_status(self, result):
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'line_mode': self.line_mode,
            'found': result['line_found'],
            'err': float(result['lateral_error']),
            'tgt': float(self.pid.target),
            'lr': float(result['line_ratio']),
            'lost': self.lost_count,
            'turn_count': self.turn_count,
            'odom_x': float(self.odom_x),
            'stopped_by_red': self.stopped_by_red,
        }, ensure_ascii=False)
        self.status_pub.publish(msg)

    def _publish_debug(self, rgb, result):
        try:
            ov = rgb.copy()
            h, w = ov.shape[:2]

            # 掩码叠加
            yl = np.zeros_like(ov)
            if self.line_mode == MODE_A_YELLOW:
                yl[:, :, 1] = 255
                yl[:, :, 2] = 255
            else:
                yl[:, :, 0] = 128
                yl[:, :, 1] = 128
                yl[:, :, 2] = 128
            mb = result['line_mask'] > 0
            bl = cv2.addWeighted(rgb, 0.5, yl, 0.5, 0)
            ov[mb] = bl[mb]

            # ROI矩形
            rl, rr, rs, re = result['roi']
            cv2.rectangle(ov, (rl, rs), (rr, re), (255, 255, 0), 1)

            # 目标参考线
            te = self.pid.target
            tx = int((te + 1.0) * w / 2.0)
            cv2.line(ov, (tx, 0), (tx, h), (0, 255, 0), 2)

            # 检测点连线
            pts = result['line_points']
            for cx, y in pts:
                cv2.circle(ov, (cx, y), 5, (0, 0, 255), -1)
            if len(pts) > 1:
                for i in range(len(pts) - 1):
                    cv2.line(ov, pts[i], pts[i + 1], (0, 0, 255), 2)

            # 状态信息
            info = (f"Mode:{self.line_mode} Err:{result['lateral_error']:+.3f} "
                    f"TC:{self.turn_count} OdomX:{self.odom_x:.2f}m")
            cv2.putText(ov, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            if self.stopped_by_red:
                cv2.putText(ov, "RED LIGHT STOP", (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(ov, encoding='bgr8'))
        except Exception:
            pass

    def _log_status(self):
        if self.mission_active:
            self.get_logger().info(
                f'[SelfDriving] state={self.state} mode={self.line_mode} '
                f'turn_count={self.turn_count} odom_x={self.odom_x:.3f}m '
                f'red_stop={self.stopped_by_red}')


def main(args=None):
    rclpy.init(args=args)
    node = SelfDrivingNode()
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
