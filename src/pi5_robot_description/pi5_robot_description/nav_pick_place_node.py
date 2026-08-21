#!/usr/bin/env python3
"""
nav_pick_place_node.py — 导航夹取与投放节点

功能流程:
  1. 导航到夹取点 (Nav2)
  2. 识别红色正方体 → 深度定位 → IK 计算 → 机械臂夹取
  3. 导航到投放点 (Nav2)
  4. 识别红色盒子 → 深度定位 → IK 计算 → 机械臂投放

用法:
  ros2 run pi5_robot_description nav_pick_place_node
  ros2 run pi5_robot_description nav_pick_place_node --ros-args -p pick_point:=夹取点 -p place_point:=投放点
"""

import math
import os
import sys
import threading
import time
import re

# 确保同目录下的模块可被导入 (ament_cmake 安装到 lib/pi5_robot_description/)
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import Image
from std_msgs.msg import String

import yaml
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer

from arm_kinematics import (
    find_best_alpha, build_arm_cmd, build_arm_cmd_with_gripper,
    pwms_to_angles, compute_T_base_to_ee_from_angles, T_mm_to_m,
)
from arm_serial import ArmSerial
from depth_utils import DepthUtils
from color_detector import ColorDetector


# ═══════════════════════════════════════════════════════════════════════════════
#  状态机定义
# ═══════════════════════════════════════════════════════════════════════════════

class State:
    IDLE = 0
    # 夹取阶段
    NAV_TO_PICK = 1
    PICK_VISION_SEARCH = 2
    PICK_WORLD_LOCATE = 3
    PICK_APPROACH = 4
    PICK_GRASP = 5
    PICK_LIFT = 6
    # 投放阶段
    NAV_TO_PLACE = 7
    PLACE_VISION_SEARCH = 8
    PLACE_WORLD_LOCATE = 9
    PLACE_APPROACH = 10
    PLACE_RELEASE = 11
    PLACE_LIFT = 12
    # 完成
    DONE = 13
    FAILED = 99


STATE_NAMES = {
    State.IDLE: 'IDLE',
    State.NAV_TO_PICK: 'NAV_TO_PICK',
    State.PICK_VISION_SEARCH: 'PICK_VISION_SEARCH',
    State.PICK_WORLD_LOCATE: 'PICK_WORLD_LOCATE',
    State.PICK_APPROACH: 'PICK_APPROACH',
    State.PICK_GRASP: 'PICK_GRASP',
    State.PICK_LIFT: 'PICK_LIFT',
    State.NAV_TO_PLACE: 'NAV_TO_PLACE',
    State.PLACE_VISION_SEARCH: 'PLACE_VISION_SEARCH',
    State.PLACE_WORLD_LOCATE: 'PLACE_WORLD_LOCATE',
    State.PLACE_APPROACH: 'PLACE_APPROACH',
    State.PLACE_RELEASE: 'PLACE_RELEASE',
    State.PLACE_LIFT: 'PLACE_LIFT',
    State.DONE: 'DONE',
    State.FAILED: 'FAILED',
}


# ═══════════════════════════════════════════════════════════════════════════════
#  主节点
# ═══════════════════════════════════════════════════════════════════════════════

class NavPickPlaceNode(Node):

    def __init__(self):
        super().__init__('nav_pick_place_node')

        # ── 参数 ──
        self.declare_parameter('pick_point', '夹取点')
        self.declare_parameter('place_point', '投放点')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('gripper_close_pwm', 1700)
        self.declare_parameter('gripper_open_pwm', 1200)
        self.declare_parameter('pick_hover_z', 80)
        self.declare_parameter('place_hover_z', 80)

        self.pick_point_name = self.get_parameter('pick_point').value
        self.place_point_name = self.get_parameter('place_point').value
        self.frame_id = self.get_parameter('frame_id').value
        self.gripper_close_pwm = self.get_parameter('gripper_close_pwm').value
        self.gripper_open_pwm = self.get_parameter('gripper_open_pwm').value
        self.pick_hover_z = self.get_parameter('pick_hover_z').value
        self.place_hover_z = self.get_parameter('place_hover_z').value

        # ── 状态 ──
        self.state = State.IDLE
        self.waypoints = {}
        self._goal_handle = None
        self._world_target_mm = None
        self._retry_count = 0
        self._max_retries = 2
        self._ik_nudge_count = 0
        self._max_ik_nudge = 3  # IK无解时最多前进重试次数
        self._pick_depth_mm = None  # 夹取时的相机深度(mm)，投放时复用

        # ── 图像帧 ──
        self.latest_rgb = None
        self.latest_depth = None
        self._frame_lock = threading.Lock()
        self.bridge = CvBridge()

        # ── 子模块 ──
        self.arm = ArmSerial(node=self)
        self.du = DepthUtils(self)
        self.detector = ColorDetector()

        # ── Nav2 Action Client ──
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── 加载目标点 ──
        self._load_waypoints()

        # ── 订阅 RGB + 深度同步 ──
        _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        from message_filters import Subscriber as MfSub
        rgb_sub = MfSub(self, Image, '/aurora/rgb/image_raw', _qos)
        depth_sub = MfSub(self, Image, '/aurora/depth/image_raw', _qos)
        self._sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        self._sync.registerCallback(self._synced_callback)

        # ── 发布 ──
        self.pub_status = self.create_publisher(String, '/pick_place_status', 10)
        self.pub_debug = self.create_publisher(Image, '/pick_place/debug_image', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_safety', 10)

        # ── 启动指令订阅 ──
        self.create_subscription(String, '/pick_place_command', self._on_command, 10)

        # ── 启动时立即移动到观察位 ──
        self.arm.send_pose('observe', delay=1.5)

        self.get_logger().info(
            f'导航夹取节点已启动 | 夹取点: {self.pick_point_name} | 投放点: {self.place_point_name}')

    # ═══════════════════════════════════════════════════════════════════════
    #  目标点加载
    # ═══════════════════════════════════════════════════════════════════════

    def _load_waypoints(self):
        from ament_index_python.packages import get_package_share_directory
        default_yaml = os.path.join(
            get_package_share_directory('pi5_robot_description'),
            'config', 'named_waypoints.yaml')
        self.declare_parameter('waypoints_file', default_yaml)
        wp_file = self.get_parameter('waypoints_file').value

        try:
            if os.path.exists(wp_file):
                with open(wp_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                if data and 'waypoints' in data:
                    for name, pos in data['waypoints'].items():
                        self.waypoints[name] = (
                            float(pos.get('x', 0.0)),
                            float(pos.get('y', 0.0)),
                            float(pos.get('yaw', 0.0)))
                    self.get_logger().info(
                        f'加载了 {len(self.waypoints)} 个目标点: {list(self.waypoints.keys())}')
        except Exception as e:
            self.get_logger().error(f'加载目标点失败: {e}')

    # ═══════════════════════════════════════════════════════════════════════
    #  图像回调
    # ═══════════════════════════════════════════════════════════════════════

    def _synced_callback(self, rgb_msg, depth_msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'mono16')
            with self._frame_lock:
                self.latest_rgb = rgb
                self.latest_depth = depth
                self.du.latest_depth = depth
        except Exception:
            pass

    # ═══════════════════════════════════════════════════════════════════════
    #  指令处理
    # ═══════════════════════════════════════════════════════════════════════

    def _on_command(self, msg):
        cmd = msg.data.strip()
        if cmd in ('start', '{"action": "start"}'):
            self._start_pick_place()

    def _start_pick_place(self):
        if self.state != State.IDLE and self.state != State.DONE:
            self.get_logger().warn(f'当前状态 {STATE_NAMES.get(self.state, "?")}，忽略启动指令')
            return

        if self.pick_point_name not in self.waypoints:
            self._speak(f'未找到夹取点: {self.pick_point_name}')
            return
        if self.place_point_name not in self.waypoints:
            self._speak(f'未找到投放点: {self.place_point_name}')
            return

        # 等待相机内参
        if not self.du.wait_for_intrinsics(10.0):
            self._speak('相机内参未就绪')
            return
        self.du.load_hand_eye_calib()

        self._retry_count = 0
        self._ik_nudge_count = 0
        self._world_target_mm = None
        self._speak('开始导航夹取任务')
        # 确保机械臂在观察位后再导航
        self.arm.send_pose('observe', delay=1.0)
        self._set_state(State.NAV_TO_PICK)

    # ═══════════════════════════════════════════════════════════════════════
    #  状态机
    # ═══════════════════════════════════════════════════════════════════════

    def _set_state(self, new_state):
        old = STATE_NAMES.get(self.state, '?')
        new = STATE_NAMES.get(new_state, '?')
        self.get_logger().info(f'状态: {old} → {new}')
        self.state = new_state
        self._publish_status(f'状态: {new}')

        # 状态入口动作
        handlers = {
            State.NAV_TO_PICK: self._enter_nav_to_pick,
            State.PICK_VISION_SEARCH: self._enter_vision_search,
            State.PICK_WORLD_LOCATE: self._enter_world_locate_pick,
            State.PICK_APPROACH: self._enter_pick_approach,
            State.PICK_GRASP: self._enter_pick_grasp,
            State.PICK_LIFT: self._enter_pick_lift,
            State.NAV_TO_PLACE: self._enter_nav_to_place,
            State.PLACE_VISION_SEARCH: self._enter_place_vision_search,
            State.PLACE_WORLD_LOCATE: self._enter_world_locate_place,
            State.PLACE_APPROACH: self._enter_place_approach,
            State.PLACE_RELEASE: self._enter_place_release,
            State.PLACE_LIFT: self._enter_place_lift,
            State.DONE: self._enter_done,
        }
        handler = handlers.get(new_state)
        if handler:
            handler()

    # ── 导航到夹取点 ──
    def _enter_nav_to_pick(self):
        x, y, yaw = self.waypoints[self.pick_point_name]
        self._speak(f'前往{self.pick_point_name}')
        self._navigate_to(x, y, yaw, self._on_pick_nav_result)

    # ── 视觉搜索红色物体 ──
    def _enter_vision_search(self):
        self._search_count = 0
        self._create_timer_once(0.5, self._vision_search_tick)

    def _vision_search_tick(self):
        with self._frame_lock:
            rgb = self.latest_rgb.copy() if self.latest_rgb is not None else None
        if rgb is None:
            self._search_count += 1
            if self._search_count < 60:
                self._create_timer_once(0.5, self._vision_search_tick)
            else:
                self._speak('未获取到相机画面')
                self._set_state(State.FAILED)
            return

        result = self.detector.detect_red(rgb, min_area=200)
        if result:
            area, center, rect = result
            self.get_logger().info(f'检测到红色物体: center=({center[0]:.0f},{center[1]:.0f}) area={area:.0f}')
            self._pick_center = center
            self._pick_rect = rect
            self._set_state(State.PICK_WORLD_LOCATE)
        else:
            self._search_count += 1
            if self._search_count < 60:
                self._create_timer_once(0.5, self._vision_search_tick)
            else:
                self._speak('未检测到红色物体')
                self._set_state(State.FAILED)

    # ── 世界坐标定位 (夹取) ──
    def _enter_world_locate_pick(self):
        # 保存夹取时的深度，投放时复用
        cx, cy = int(self._pick_center[0]), int(self._pick_center[1])
        with self._frame_lock:
            dimg = self.latest_depth
        if dimg is not None:
            for dy in range(-5, 6):
                for dx in range(-5, 6):
                    d = self.du.get_depth_at(cx + dx, cy + dy, dimg)
                    if d is not None and d >= 100:
                        self._pick_depth_mm = d
                        break
                if self._pick_depth_mm:
                    break
        self.get_logger().info(f'[夹取] 保存深度: {self._pick_depth_mm}mm')

        target = self._compute_world_target(self._pick_center)
        if target is None:
            self.get_logger().warn('世界定位失败，重试')
            self._retry_count += 1
            if self._retry_count < 3:
                self._set_state(State.PICK_VISION_SEARCH)
            else:
                self._speak('定位失败')
                self._set_state(State.FAILED)
            return

        self._world_target_mm = target
        self._retry_count = 0
        self.get_logger().info(
            f'[夹取] 世界坐标: ({target[0]:.0f}, {target[1]:.0f}, {target[2]:.0f})mm')
        self._set_state(State.PICK_APPROACH)

    # ── 机械臂飞到目标上方 ──
    def _enter_pick_approach(self):
        tx, ty, tz = self._world_target_mm
        Y_OFFSET = 170  # Y轴偏移补偿（实测夹取偏近）
        ty += Y_OFFSET
        hover_z = max(int(tz) + self.pick_hover_z, 80)
        grasp_z = max(int(tz) - 5, 10)
        self.get_logger().info(f'[夹取] Y偏移: {ty-Y_OFFSET:.0f}+{Y_OFFSET}={ty:.0f}')
        # 预先检查飞到和下降的IK
        a1, pwms1 = find_best_alpha(int(tx), int(ty), hover_z, alpha_hint=-82)
        a2, pwms2 = find_best_alpha(int(tx), int(ty), grasp_z, alpha_hint=-82)
        if a1 is None or a2 is None:
            self.get_logger().warn(f'[夹取] IK无解: 飞到({tx:.0f},{ty:.0f},{hover_z})={a1}, 下降({tx:.0f},{ty:.0f},{grasp_z})={a2}，小车前进重试')
            self._nudge_forward_and_retry(State.PICK_VISION_SEARCH)
            return
        self.get_logger().info(f'[夹取] IK检查通过: 飞到alpha={a1}°, 下降alpha={a2}°')
        self.get_logger().info(f'[夹取] 飞到 ({tx:.0f},{ty:.0f},{hover_z})')
        cmd = build_arm_cmd(pwms1, 2000)
        self.arm.send(cmd)
        self._create_timer_once(3.0, lambda: self._set_state(State.PICK_GRASP))

    # ── 下降 + 闭合夹爪 ──
    def _enter_pick_grasp(self):
        tx, ty, tz = self._world_target_mm
        Y_OFFSET = 170  # Y轴偏移补偿（实测夹取偏近）
        ty += Y_OFFSET
        grasp_z = max(int(tz) - 5, 10)
        self.get_logger().info(f'[夹取] Y偏移: {ty-Y_OFFSET:.0f}+{Y_OFFSET}={ty:.0f}')
        self.get_logger().info(f'[夹取] 下降到 ({tx:.0f},{ty:.0f},{grasp_z})')
        # 先打开夹爪
        self.arm.send_gripper(self.gripper_open_pwm, 1000)
        a, pwms = find_best_alpha(int(tx), int(ty), grasp_z, alpha_hint=-82)
        if a is None:
            self.get_logger().error(f'IK 无解: ({tx:.0f},{ty:.0f},{grasp_z})')
            self._set_state(State.FAILED)
            return
        self.get_logger().info(f'[夹取] IK: alpha={a}° pwms={pwms}')
        cmd = build_arm_cmd(pwms, 2000)
        self.arm.send(cmd)
        self._create_timer_once(2.5, self._do_grasp)

    def _do_grasp(self):
        self.get_logger().info('[夹取] 闭合夹爪')
        for _ in range(3):
            self.arm.send_gripper(self.gripper_close_pwm, 1000)
            time.sleep(0.4)
        self._set_state(State.PICK_LIFT)

    # ── 抬升 → 过渡位 → 观察位 ──
    def _enter_pick_lift(self):
        tx, ty, _ = self._world_target_mm
        self.get_logger().info(f'[夹取] 抬升到 z=150')
        a, pwms = find_best_alpha(int(tx), int(ty), 150, alpha_hint=-82)
        if a is None:
            self.arm.send_pose('transition', delay=1.5)
        else:
            self.arm.send(build_arm_cmd(pwms, 1000))
            time.sleep(1.5)
        # 先过渡位，再观察位
        self.arm.send_pose('transition', delay=1.5)
        self.arm.send_pose('observe', delay=1.0)
        self._set_state(State.NAV_TO_PLACE)

    # ── 导航到投放点 ──
    def _enter_nav_to_place(self):
        # 机械臂已在观察位，直接导航
        self._ik_nudge_count = 0  # 重置IK前进计数
        x, y, yaw = self.waypoints[self.place_point_name]
        self._speak(f'前往{self.place_point_name}')
        self._navigate_to(x, y, yaw, self._on_place_nav_result)

    # ── 视觉搜索红色盒子 ──
    def _enter_place_vision_search(self):
        self._search_count = 0
        self._create_timer_once(0.5, self._place_vision_search_tick)

    def _place_vision_search_tick(self):
        with self._frame_lock:
            rgb = self.latest_rgb.copy() if self.latest_rgb is not None else None
        if rgb is None:
            self._search_count += 1
            if self._search_count < 60:
                self._create_timer_once(0.5, self._place_vision_search_tick)
            else:
                self._speak('未获取到相机画面')
                self._set_state(State.FAILED)
            return

        # 检测红色盒子 (面积更大)
        result = self.detector.detect_red(rgb, min_area=500)
        if result:
            area, center, rect = result
            self.get_logger().info(f'检测到红色盒子: center=({center[0]:.0f},{center[1]:.0f}) area={area:.0f}')
            self._place_center = center
            self._place_rect = rect
            self._set_state(State.PLACE_WORLD_LOCATE)
        else:
            self._search_count += 1
            if self._search_count < 60:
                self._create_timer_once(0.5, self._place_vision_search_tick)
            else:
                self._speak('未检测到红色盒子')
                self._set_state(State.FAILED)

    # ── 世界坐标定位 (投放，深度无效时复用夹取深度) ──
    def _enter_world_locate_place(self):
        target = self._compute_world_target(self._place_center,
                                            fallback_depth_mm=self._pick_depth_mm)
        if target is None:
            self.get_logger().warn('投放定位失败，重试')
            self._retry_count += 1
            if self._retry_count < 3:
                self._set_state(State.PLACE_VISION_SEARCH)
            else:
                self._speak('投放定位失败')
                self._set_state(State.FAILED)
            return

        self._world_target_mm = target
        self._retry_count = 0
        self.get_logger().info(
            f'[投放] 世界坐标: ({target[0]:.0f}, {target[1]:.0f}, {target[2]:.0f})mm')
        self._set_state(State.PLACE_APPROACH)

    # ── 飞到盒子上方 ──
    def _enter_place_approach(self):
        tx, ty, tz = self._world_target_mm
        Y_OFFSET = 170  # Y轴偏移补偿
        ty += Y_OFFSET
        hover_z = max(int(tz) + self.place_hover_z, 80)
        place_z = max(int(tz) - 5, 10)
        self.get_logger().info(f'[投放] Y偏移: {ty-Y_OFFSET:.0f}+{Y_OFFSET}={ty:.0f}')
        # 预先检查飞到和下降的IK
        a1, pwms1 = find_best_alpha(int(tx), int(ty), hover_z, alpha_hint=-82)
        a2, pwms2 = find_best_alpha(int(tx), int(ty), place_z, alpha_hint=-82)
        if a1 is None or a2 is None:
            self.get_logger().warn(f'[投放] IK无解: 飞到({tx:.0f},{ty:.0f},{hover_z})={a1}, 下降({tx:.0f},{ty:.0f},{place_z})={a2}，小车前进重试')
            self._nudge_forward_and_retry(State.PLACE_VISION_SEARCH)
            return
        self.get_logger().info(f'[投放] IK检查通过: 飞到alpha={a1}°, 下降alpha={a2}°')
        self.get_logger().info(f'[投放] 飞到 ({tx:.0f},{ty:.0f},{hover_z})')
        cmd = build_arm_cmd(pwms1, 2000)
        self.arm.send(cmd)
        self._create_timer_once(3.0, self._do_place_descend)

    def _do_place_descend(self):
        tx, ty, tz = self._world_target_mm
        # 下降时不加Y偏移，因为approach已定位到位，直接往下即可
        place_z = max(int(tz) - 5, 10)
        self.get_logger().info(f'[投放] 下降到 ({tx:.0f},{ty:.0f},{place_z})')
        a, pwms = find_best_alpha(int(tx), int(ty), place_z, alpha_hint=-82)
        if a is None:
            # 下降IK无解，尝试用approach的Z直接释放
            self.get_logger().warn(f'[投放] 下降IK无解，直接释放')
            self._set_state(State.PLACE_RELEASE)
            return
        self.get_logger().info(f'[投放] IK: alpha={a}° pwms={pwms}')
        cmd = build_arm_cmd(pwms, 2000)
        self.arm.send(cmd)
        self._create_timer_once(2.5, lambda: self._set_state(State.PLACE_RELEASE))

    # ── 打开夹爪释放 ──
    def _enter_place_release(self):
        self.get_logger().info('[投放] 打开夹爪')
        for _ in range(3):
            self.arm.send_gripper(self.gripper_open_pwm, 1000)
            time.sleep(0.4)
        self._set_state(State.PLACE_LIFT)

    # ── 抬升 → 过渡位 → 观察位 ──
    def _enter_place_lift(self):
        tx, ty, _ = self._world_target_mm
        a, pwms = find_best_alpha(int(tx), int(ty), 150, alpha_hint=-82)
        if a is None:
            self.arm.send_pose('transition', delay=1.5)
        else:
            self.arm.send(build_arm_cmd(pwms, 1000))
            time.sleep(1.5)
        # 先过渡位，再观察位
        self.arm.send_pose('transition', delay=1.5)
        self.arm.send_pose('observe', delay=1.0)
        self._set_state(State.DONE)

    # ── 完成 ──
    def _enter_done(self):
        self._speak('夹取投放任务完成')
        self._world_target_mm = None

    # ═══════════════════════════════════════════════════════════════════════
    #  导航
    # ═══════════════════════════════════════════════════════════════════════

    def _navigate_to(self, x, y, yaw, result_callback):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server 未就绪')
            self._set_state(State.FAILED)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._on_goal_response(f, result_callback))

    def _on_goal_response(self, future, result_callback):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('导航目标被拒绝')
            self._handle_nav_failure()
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(result_callback)

    def _on_pick_nav_result(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✅ 到达{self.pick_point_name}')
            self._retry_count = 0
            # 机械臂已在观察位，直接开始视觉搜索
            self._set_state(State.PICK_VISION_SEARCH)
        else:
            self.get_logger().warn(f'导航失败: {result.status}')
            self._handle_nav_failure()

    def _on_place_nav_result(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✅ 到达{self.place_point_name}')
            self._retry_count = 0
            # 机械臂已在观察位，直接开始视觉搜索
            self._set_state(State.PLACE_VISION_SEARCH)
        else:
            self.get_logger().warn(f'导航失败: {result.status}')
            self._handle_nav_failure()

    def _handle_nav_failure(self):
        self._goal_handle = None
        self._retry_count += 1
        if self._retry_count <= self._max_retries:
            self.get_logger().info(f'重试导航 ({self._retry_count}/{self._max_retries})')
            if self.state == State.NAV_TO_PICK:
                self._set_state(State.NAV_TO_PICK)
            else:
                self._set_state(State.NAV_TO_PLACE)
        else:
            self._speak('导航失败')
            self._set_state(State.FAILED)

    # ═══════════════════════════════════════════════════════════════════════
    #  IK无解时前进重试
    # ═══════════════════════════════════════════════════════════════════════

    def _nudge_forward_and_retry(self, retry_state):
        """
        IK无解时，控制小车前进一点，然后重新视觉搜索

        Args:
            retry_state: 重试后进入的状态 (PICK_VISION_SEARCH 或 PLACE_VISION_SEARCH)
        """
        self._ik_nudge_count += 1
        if self._ik_nudge_count > self._max_ik_nudge:
            self.get_logger().error(f'IK无解，已前进{self._max_ik_nudge}次仍无法到达')
            self._ik_nudge_count = 0
            self._set_state(State.FAILED)
            return

        self.get_logger().info(f'[IK重试] 前进一小步 ({self._ik_nudge_count}/{self._max_ik_nudge})')
        # 发送前进速度指令，持续0.8秒
        twist = Twist()
        twist.linear.x = 0.08  # 8cm/s * 0.8s ≈ 6cm
        self._cmd_vel_pub.publish(twist)
        self._create_timer_once(0.8, lambda: self._stop_and_retry(retry_state))

    def _stop_and_retry(self, retry_state):
        """停止移动，重新进入视觉搜索"""
        twist = Twist()  # 零速
        self._cmd_vel_pub.publish(twist)
        self.get_logger().info('[IK重试] 停止移动，重新识别')
        self._set_state(retry_state)

    # ═══════════════════════════════════════════════════════════════════════
    #  世界坐标计算
    # ═══════════════════════════════════════════════════════════════════════

    def _compute_world_target(self, pixel_center, fallback_depth_mm=None):
        """
        用深度+FK计算物体世界坐标

        Args:
            pixel_center: (cx, cy) 像素坐标
            fallback_depth_mm: 回退深度(mm)，深度无效时使用

        Returns:
            (X, Y, Z) mm 或 None
        """
        cx, cy = int(pixel_center[0]), int(pixel_center[1])
        MIN_DEPTH = 100  # mm

        # 深度搜索 (带重试)
        dmm = None
        for attempt in range(4):
            with self._frame_lock:
                dimg = self.latest_depth
            if dimg is None:
                time.sleep(0.05)
                continue

            # 5×5 邻域
            if dmm is None:
                for dy in range(-5, 6):
                    for dx in range(-5, 6):
                        d = self.du.get_depth_at(cx + dx, cy + dy, dimg)
                        if d is not None and d >= MIN_DEPTH:
                            dmm = d
                            break
                    if dmm:
                        break

            # 15×15 邻域
            if dmm is None:
                for dy in range(-15, 16):
                    for dx in range(-15, 16):
                        d = self.du.get_depth_at(cx + dx, cy + dy, dimg)
                        if d is not None and d >= MIN_DEPTH:
                            dmm = d
                            break
                    if dmm:
                        break

            if dmm is not None:
                break
            if attempt < 3:
                time.sleep(0.05)

        # 深度无效时使用回退深度
        if dmm is None:
            if fallback_depth_mm is not None:
                dmm = fallback_depth_mm
                self.get_logger().info(f'深度无效: ({cx},{cy})，使用回退深度 {dmm}mm')
            else:
                self.get_logger().warn(f'深度无效: ({cx},{cy})')
                return None

        # 像素→相机3D
        p_cam = self.du.pixel_to_3d(cx, cy, dmm)

        # 读取当前关节PWM，计算FK
        pwms = self._read_joint_pwms()
        if pwms is None:
            self.get_logger().warn('无法读取关节PWM，使用默认FK')
            # 使用观察位的近似FK (对应 #0P1550 #1P1800 #2P2000 #3P0700)
            T_g2b_mm = compute_T_base_to_ee_from_angles(0, 45, 40, -108)
        else:
            th = pwms_to_angles(*pwms)
            T_g2b_mm = compute_T_base_to_ee_from_angles(*th)

        T_g2b = T_mm_to_m(T_g2b_mm)

        # 相机→基座
        p_base_m = self.du.transform_cam_to_base(p_cam, T_g2b)
        X, Y, Z = float(p_base_m[0]) * 1000, float(p_base_m[1]) * 1000, float(p_base_m[2]) * 1000

        self.get_logger().info(
            f'[世界] pix=({cx},{cy}) d={dmm}mm '
            f'cam=({p_cam[0]:.3f},{p_cam[1]:.3f},{p_cam[2]:.3f}) '
            f'base=({X:.0f},{Y:.0f},{Z:.0f})mm')

        return (X, Y, Z)

    def _read_joint_pwms(self):
        """读取当前关节 PWM 值 (从机械臂控制板)"""
        pwms = []
        for i in range(4):
            v = self._read_pwm(i)
            if v is None:
                return None
            pwms.append(v)
        return tuple(pwms)

    def _read_pwm(self, idx, timeout=0.6):
        """读取单个舵机 PWM"""
        self.arm.send(f'#{idx:03d}PRAD!')
        deadline = time.time() + timeout
        # 注意: 这里需要读取串口返回，简化实现用默认值
        # 实际实现需要在 arm_serial 中添加读取功能
        return None  # TODO: 实现 PWM 读取

    # ═══════════════════════════════════════════════════════════════════════
    #  辅助
    # ═══════════════════════════════════════════════════════════════════════

    def _speak(self, text):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)
        self.get_logger().info(f'📢 {text}')

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def _create_timer_once(self, delay_sec, callback):
        """创建一次性定时器"""
        timer = self.create_timer(delay_sec, lambda: self._timer_fire(timer, callback))

    def _timer_fire(self, timer, callback):
        timer.cancel()
        callback()

    def destroy_node(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self.arm.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavPickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
