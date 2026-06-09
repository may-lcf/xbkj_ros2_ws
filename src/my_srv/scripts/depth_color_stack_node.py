#!/usr/bin/env python3
"""
depth_color_stack_node.py — 深度增强颜色码垛 v2

完全对齐 depth_color_sorting_node.py 的坐标使用方式：
  - stage0: 世界系定位（像素+深度+外参+FK → base XYZ）
  - stage1: 夹爪旋转对齐
  - stage2: 飞到世界目标正上方（直接用世界坐标，不加偏移）
  - stage3: 下降到抓取深度（直接用世界坐标）
  - stage4: 抬升
  - stage5: 旋转到堆放区 + 寻找绿色基准
  - stage6: PID 对准堆放点
  - stage7: 分层放置
  - stage8: 归位
"""

import os, sys, re, time, math, threading
import numpy as np, cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer
from cv_bridge import CvBridge

# ── 路径 ──
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
for p in (_SCRIPT_DIR, os.path.expanduser('~/ros2_ws/src/my_srv/scripts'), os.path.expanduser('~/OpenCV')):
    if p not in sys.path:
        sys.path.insert(0, p)

from depth_utils import DepthUtils
import arm_fk
import z_uart
from example_interfaces.srv import Trigger
from z_uart import uart_send_str, setup_uart, close_uart
from z_move import kinematics_move


# ═══════════════════════════════════════════════════════════════════════════════
#  颜色阈值加载
# ═══════════════════════════════════════════════════════════════════════════════

def _load_thresholds(filename):
    for d in (_SCRIPT_DIR, os.getcwd(), os.path.expanduser('~/ros2_ws/src/my_srv/scripts')):
        fp = os.path.join(d, filename)
        if os.path.exists(fp):
            break
    else:
        fp = filename
    with open(fp) as f:
        nums = []
        for line in f:
            line = line.strip()
            if not line: continue
            for s in line.split():
                nums.append(int(s) if '.' not in s else float(s))
    lo = (int(nums[0]), int(nums[2]), int(nums[4]))
    hi = (int(nums[1]), int(nums[3]), int(nums[5]))
    return lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]


red_low = _load_thresholds('red.txt')
blue_low = _load_thresholds('blue.txt')
green_low = _load_thresholds('green.txt')


# ═══════════════════════════════════════════════════════════════════════════════
#  PID
# ═══════════════════════════════════════════════════════════════════════════════

class PIDController:
    def __init__(self, kp, ki, kd):
        self.Target_val = 0.0
        self.last_error = 0.0
        self.sum_error = 0.0
        self.kp, self.ki, self.kd = kp, ki, kd

    def PID_Realize(self, actual_val):
        err = self.Target_val - actual_val
        self.sum_error += err
        out = self.kp * err + self.ki * self.sum_error + self.kd * (err - self.last_error)
        self.last_error = err
        return out


# ═══════════════════════════════════════════════════════════════════════════════
#  DepthColorStackNode
# ═══════════════════════════════════════════════════════════════════════════════

class DepthColorStackNode(Node):
    def __init__(self):
        super().__init__('depth_color_stack_node')
        self.du = DepthUtils(self)

        self.latest_rgb = None
        self.latest_depth = None
        self._frame_lock = threading.Lock()
        self.bridge = CvBridge()

        self.width = 640
        self.height = 480

        # ── 状态机 ──
        self.move_x, self.move_y, self.move_z = 0, 120, 60
        self.move_status = 0
        self.target_rect = None
        self.color_read_succed = 0
        self.target_colors = ["red", "blue", "green"]
        self.current_color_index = 0

        # PID 目标: 深度相机 640×480 中心
        self.TARGET_CX, self.TARGET_CY = 320, 240
        self.block_cx, self.block_cy = self.TARGET_CX, self.TARGET_CY

        self.red_rect = self.blue_rect = self.green_rect = None
        self.spin_calw = 1500
        self.detected_color = None
        self.success_cnt = 0
        self.stack_active = False
        self.active = False  # enter/exit 模式守卫
        self._run_thread = None
        self.object_depth_mm = 0
        self.world_target_mm = None
        self._last_logged_status = -1

        # ── 码垛专用 ──
        self.mark_flag = 255        # 255=首次检测, 0=寻找堆放基准, 1=基准已找到
        self.bak_cx = -130          # 堆放基准 x (mm)
        self.bak_cy = 30            # 堆放基准 y (mm)
        self.block_cnt = 0          # 已码垛数量
        self.stack_world_mm = None  # 堆放基准世界坐标
        # 码垛高度 (mm)
        self.stack_height_one = 10
        self.stack_height_two = 46
        self.stack_height_three = 70
        # 放置延伸偏移 (仅在 stage6 对准后使用)
        self.place_offset_x = 60
        self.place_offset_y = 60
        # 每种颜色的夹取位置补偿
        self.color_pick_offset = {
            'red':   {'x': 0, 'y': -2},
            'blue':  {'x': 0, 'y':  3},
            'green': {'x': 0, 'y':  3},
        }

        # 颜色阈值
        self.lower_red   = np.array(red_low[0:3], dtype=np.uint8)
        self.upper_red   = np.array(red_low[3:6], dtype=np.uint8)
        self.lower_blue  = np.array(blue_low[0:3], dtype=np.uint8)
        self.upper_blue  = np.array(blue_low[3:6], dtype=np.uint8)
        self.lower_green = np.array(green_low[0:3], dtype=np.uint8)
        self.upper_green = np.array(green_low[3:6], dtype=np.uint8)

        # PID
        self.pid_x = PIDController(kp=0.01, ki=0.000, kd=0.0)
        self.pid_y = PIDController(kp=0.01, ki=0.000, kd=0.0)

        # 调试话题
        self.debug_pub = self.create_publisher(Image, '/depth_color_stack/image_result', 10)

        # RGB + 深度同步
        from message_filters import Subscriber as MfSub
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        rgb_sub = MfSub(self, Image, '/aurora/rgb/image_raw', _qos)
        depth_sub = MfSub(self, Image, '/aurora/depth/image_raw', _qos)
        self._sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        self._sync.registerCallback(self._synced_callback)

        # enter/exit 服务
        self.enter_srv = self.create_service(Trigger, '/depth_color_stack/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/depth_color_stack/exit', self.exit_callback)

        self.get_logger().info('\033[1;36m[DepthColorStack]\033[0m v2 世界系定位(对齐排序版)')

    # ══ 同步 ══════════════════════════════════════════════════════════════════

    def _synced_callback(self, rgb_msg: Image, depth_msg: Image):
        if not self.active:
            return
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'mono16')
            self.height, self.width = rgb.shape[:2]
            with self._frame_lock:
                self.latest_rgb = rgb
                self.latest_depth = depth
                self.du.latest_depth = depth
        except Exception:
            pass

    # ══ 颜色检测 ══════════════════════════════════════════════════════════════

    @staticmethod
    def _limit(d, mn, mx):
        return max(mn, min(mx, d))

    @staticmethod
    def _calc_angle(rect):
        _, _, ang = rect
        if ang <= 10 or ang >= 80: return 0
        if 10 < ang < 45: return -ang
        return 90 - ang

    def _detect_color(self, mask, require_square=True):
        k = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if not cnts:
            return 0, (0, 0), None
        best, ba = None, -1
        for c in cnts:
            rect = cv2.minAreaRect(c)
            (cx, cy), (w, h), _ = rect
            a = cv2.contourArea(c)
            if min(w, h) < 15 or a < 500: continue
            if require_square:
                side_min, side_max = min(w, h), max(w, h)
                if side_min > 0 and (side_min / side_max) < 0.85: continue
            if a > ba:
                ba = a
                best = (a, (cx, cy), rect)
        return best if best else (0, (0, 0), None)

    # ══ 世界系定位 (与排序版完全一致) ══════════════════════════════════════════

    def _read_pwm(self, idx, timeout=0.6):
        uart_send_str(f"#{idx:03d}PRAD!")
        dl = time.time() + timeout
        while time.time() < dl:
            if z_uart.uart_get_ok:
                d = z_uart.uart_receive_buf; z_uart.uart_receive_buf = ''; z_uart.uart_get_ok = 0
                m = re.search(r'#\d{3}P(\d+)!', d)
                if m: return int(m.group(1))
            time.sleep(0.005)
        return None

    def _read_joint_pwms(self):
        pwms = []
        for i in range(4):
            v = self._read_pwm(i)
            if v is None:
                self.get_logger().warn(f'[FK] 舵机{i:03d} PWM读取超时')
                return None
            pwms.append(v)
        return tuple(pwms)

    def _compute_world_target(self):
        """与排序版完全一致的世界坐标计算"""
        cx, cy = int(self.block_cx), int(self.block_cy)
        with self._frame_lock:
            dimg = self.latest_depth
        if dimg is None: return None

        # 邻域搜索有效深度 (与排序版一致: 先 5x5, 再扩大到 10x10)
        dmm = None
        found_at = None
        for dy in range(-5, 6):
            for dx in range(-5, 6):
                tx, ty = cx + dx, cy + dy
                d = self.du.get_depth_at(tx, ty, dimg)
                if d is not None and d >= 150:
                    dmm = d
                    found_at = (tx, ty)
                    break
            if dmm is not None:
                break

        if dmm is None:
            for dy in range(-10, 11, 2):
                for dx in range(-10, 11, 2):
                    tx, ty = cx + dx, cy + dy
                    d = self.du.get_depth_at(tx, ty, dimg)
                    if d is not None and d >= 150:
                        dmm = d
                        found_at = (tx, ty)
                        break
                if dmm is not None:
                    break

        if dmm is None:
            self.get_logger().warn(f'[世界] 深度无效 ({cx},{cy}) 邻域 20×20 也无效')
            return None

        if found_at and (found_at[0] != cx or found_at[1] != cy):
            self.get_logger().info(
                f'[世界] 邻域搜索: ({cx},{cy})→({found_at[0]},{found_at[1]}) d={dmm}mm')

        self.object_depth_mm = int(dmm)
        try:
            p_cam = self.du.pixel_to_3d(cx, cy, dmm)
        except Exception as e:
            self.get_logger().warn(f'[世界] pixel_to_3d 失败: {e}')
            return None
        pwms = self._read_joint_pwms()
        if pwms is None: return None
        th = arm_fk.pwms_to_angles(*pwms)
        try:
            T_g2b_mm = arm_fk.compute_T_base_to_ee_from_angles(*th)
            T_g2b = arm_fk.T_mm_to_m(T_g2b_mm)
        except Exception as e:
            self.get_logger().warn(f'[世界] FK 失败: {e}')
            return None
        try:
            p_base_m = self.du.transform_cam_to_base(p_cam, T_g2b)
        except Exception as e:
            self.get_logger().warn(f'[世界] cam→base 失败: {e}')
            return None
        X = float(p_base_m[0]) * 1000
        Y = float(p_base_m[1]) * 1000
        Z = float(p_base_m[2]) * 1000
        self.get_logger().info(
            f'[世界] pix=({cx},{cy}) d={dmm}mm cam=({p_cam[0]:.3f},{p_cam[1]:.3f},{p_cam[2]:.3f}) '
            f'base=({X:.0f},{Y:.0f},{Z:.0f})mm')
        return (X, Y, Z)

    # ══ 主循环 ════════════════════════════════════════════════════════════════

    def run(self):
        if not self.du.wait_for_intrinsics(15.0):
            self.get_logger().error('[DepthColorStack] 内参超时'); return
        self.du.load_hand_eye_calib()
        self.get_logger().info('[DepthColorStack] 等待同步帧...')
        for _ in range(200):
            with self._frame_lock:
                if self.latest_rgb is not None: break
            time.sleep(0.1)
        else:
            self.get_logger().error('[DepthColorStack] 无同步帧'); return

        self.stack_active = True
        # 初始位置 (与排序版一致)
        self.move_x, self.move_y, self.move_z = 0, 105, 150
        kinematics_move(self.move_x, self.move_y, self.move_z, 1000, alpha_hint=-82)
        time.sleep(2.0)  # 等待稳定
        self.move_status = 0
        self.current_color_index = 0
        self.block_cnt = 0
        self.mark_flag = 255
        self.get_logger().info('\033[1;32m[DepthColorStack]\033[0m 码垛启动')

        fc = 0
        while self.stack_active and rclpy.ok():
            if not self.active:
                time.sleep(0.1)
                continue
            with self._frame_lock:
                if self.latest_rgb is None: time.sleep(0.03); continue
                frame = self.latest_rgb.copy()

            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            m_blue  = cv2.inRange(lab, self.lower_blue,  self.upper_blue)
            m_red   = cv2.inRange(lab, self.lower_red,   self.upper_red)
            m_green = cv2.inRange(lab, self.lower_green, self.upper_green)

            # ── 颜色检测 ──
            self.detected_color = None
            found = False

            if self.mark_flag == 0:
                _, _, self.green_rect = self._detect_color(m_green)
                if self.green_rect:
                    self.block_cx, self.block_cy = self.green_rect[0]
                    self._draw_rect(frame, self.green_rect, (0, 255, 0), "GREEN-MARK")
                    self.detected_color = 'green'
                    found = True
            else:
                target = self._current_target()
                if target in ('red', None):
                    _, _, self.red_rect = self._detect_color(m_red)
                    if self.red_rect and target == 'red':
                        self.block_cx, self.block_cy = self.red_rect[0]
                        self._draw_rect(frame, self.red_rect, (0, 0, 255), "red")
                        self.detected_color = 'red'
                        found = True
                if target in ('blue', None):
                    _, _, self.blue_rect = self._detect_color(m_blue)
                    if self.blue_rect and target == 'blue':
                        self.block_cx, self.block_cy = self.blue_rect[0]
                        self._draw_rect(frame, self.blue_rect, (255, 0, 0), "blue")
                        self.detected_color = 'blue'
                        found = True
                if target in ('green', None):
                    _, _, self.green_rect = self._detect_color(m_green)
                    if self.green_rect and target == 'green':
                        self.block_cx, self.block_cy = self.green_rect[0]
                        self._draw_rect(frame, self.green_rect, (0, 255, 0), "green")
                        self.detected_color = 'green'
                        found = True

            self.color_read_succed = 1 if found else 0

            fc += 1
            if fc % 150 == 0 or self._last_logged_status != self.move_status:
                self.get_logger().info(
                    f'[STACK] f={fc} found={found} det={self.detected_color} '
                    f'st={self.move_status} mark={self.mark_flag} cnt={self.block_cnt}')
                self._last_logged_status = self.move_status

            # HUD
            cv2.putText(frame, f"Stack: {self.block_cnt}/3", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"Status: {self.move_status} Mark: {self.mark_flag}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            if found:
                cv2.putText(frame, f"DET: {self.detected_color}", (10, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
            except Exception:
                pass

            if self.stack_active and (self.color_read_succed or self.move_status >= 2):
                self._run_state_machine()

            time.sleep(0.03)

        self.get_logger().info('[DepthColorStack] 结束')

    def _current_target(self):
        if self.mark_flag == 0:
            return 'green'
        return self.target_colors[self.current_color_index] if self.target_colors else None

    def _lock_rect(self):
        mp = {'red': self.red_rect, 'blue': self.blue_rect, 'green': self.green_rect}
        if self.detected_color in mp:
            self.target_rect = mp[self.detected_color]

    # ══ 状态机 ════════════════════════════════════════════════════════════════

    def _run_state_machine(self):
        stages = [self._st0, self._st1, self._st2, self._st3, self._st4,
                  self._st5, self._st6, self._st7, self._st8]
        if self.move_status < len(stages):
            print(f'[Stage] 进入 stage {self.move_status}')
            stages[self.move_status]()

    # ── stage 0: 世界系定位 ──────────────────────────────────────────────────
    # 与排序版 _st0 完全一致：不移臂，从当前位姿做一次性世界坐标计算
    def _st0(self):
        self._lock_rect()
        self.world_target_mm = self._compute_world_target()
        if self.world_target_mm is None:
            self.get_logger().warn('[st0] 世界定位失败，重试')
            self.color_read_succed = 0
            return

        if self.mark_flag == 0:
            # 寻找堆放基准: 记录绿色标记的世界坐标
            self.stack_world_mm = self.world_target_mm
            self.bak_cx = int(self.stack_world_mm[0])
            self.bak_cy = int(self.stack_world_mm[1])
            self.mark_flag = 1
            self.get_logger().info(
                f'[st0] 堆放基准 XYZ=({self.bak_cx},{self.bak_cy},{self.stack_world_mm[2]:.0f})mm')
            self.color_read_succed = 0
            self.move_status = 5
            return

        self.color_read_succed = 1
        self.move_status = 1
        self.get_logger().info(
            f'[st0] {self.detected_color} XYZ=({self.world_target_mm[0]:.0f},'
            f'{self.world_target_mm[1]:.0f},{self.world_target_mm[2]:.0f})mm '
            f'pix=({self.block_cx:.0f},{self.block_cy:.0f})')

    # ── stage 1: 夹爪旋转 (与排序版 _st1 一致) ─────────────────────────────
    def _st1(self):
        self.move_status = 2
        ang = self._calc_angle(self.target_rect)
        self.spin_calw = self._limit(int(1500 - ang * 7.4), 1167, 1833)
        for _ in range(3):
            uart_send_str("#004P{:0^4}T800!".format(self.spin_calw))
            time.sleep(0.15)
        uart_send_str("#005P1000T500!")
        time.sleep(0.3)

    # ── stage 2: 飞到世界目标上方 (与排序版 _st2 完全一致) ───────────────────
    # 关键：直接用世界坐标 tx, ty，不加任何偏移
    def _st2(self):
        self.move_status = 3
        if self.world_target_mm is None: return
        tx, ty, tz = self.world_target_mm
        hover_z = max(int(tz) + 80, 60)
        self.move_x, self.move_y, self.move_z = int(tx), int(ty), hover_z
        self.get_logger().info(f'[st2] 飞到 ({self.move_x},{self.move_y},{hover_z})')
        if not kinematics_move(self.move_x, self.move_y, self.move_z, 1500, alpha_hint=-82):
            self.get_logger().error(f'[st2] IK 无解')
            self.move_status = 8
            return
        time.sleep(1.6)

    # ── stage 3: 下降抓取 (与排序版 _st3 完全一致) ──────────────────────────
    # 关节：直接用世界坐标 tx, ty，抓取深度 = tz - 5
    def _st3(self):
        self.move_status = 4
        if self.world_target_mm:
            tx, ty, tz = self.world_target_mm
            gz = max(int(tz) - 5, 5)
            self.move_x, self.move_y, self.move_z = int(tx), int(ty), gz
            self.get_logger().info(f'[st3] 下降到 ({self.move_x},{self.move_y},{gz})')
            if not kinematics_move(self.move_x, self.move_y, gz, 1200, alpha_hint=-82):
                self.get_logger().error(f'[st3] IK 无解')
                self.move_status = 8
                return
            time.sleep(1.3)
        for _ in range(3):
            uart_send_str("#005P1700T1000!")
            time.sleep(0.4)

    # ── stage 4: 抬升 (与排序版 _st4 一致) ──────────────────────────────────
    def _st4(self):
        self.move_status = 5
        self.block_cx = self.block_cy = 0
        if not kinematics_move(self.move_x, self.move_y, 150, 1000, alpha_hint=-82):
            self.get_logger().error(f'[st4] IK 无解')
            self.move_status = 8
            return
        time.sleep(1)
        uart_send_str("#004P1500T1000!")
        time.sleep(0.5)

    # ── stage 5: 旋转到堆放区 + 寻找基准 ───────────────────────────────────
    def _st5(self):
        if self.mark_flag == 255:
            # 首次：飞到默认堆放区，寻找绿色基准
            self.move_x, self.move_y = -130, 30
            kinematics_move(self.move_x, self.move_y, 150, 1000)
            time.sleep(1)
            uart_send_str("#004P1500T1500!")
            time.sleep(0.5)
            kinematics_move(self.move_x, self.move_y, 60, 1000)
            time.sleep(2.5)
            self.mark_flag = 0
            self.color_read_succed = 0
            return
        elif self.mark_flag == 1:
            # 基准已找到，飞到基准上方
            self.move_x, self.move_y = self.bak_cx, self.bak_cy
            kinematics_move(self.move_x, self.move_y, 150, 1000)
            time.sleep(1)
            uart_send_str("#004P1500T1500!")
            time.sleep(0.5)
            kinematics_move(self.move_x, self.move_y, 60, 1000)
            time.sleep(2.5)
            self.color_read_succed = 0
            self.move_status = 6
        elif self.mark_flag == 0:
            # PID 对准绿色标记
            self.pid_x.Target_val = self.TARGET_CX
            self.pid_y.Target_val = self.TARGET_CY
            self.move_y -= self.pid_x.PID_Realize(self.block_cx)
            self.move_x -= self.pid_y.PID_Realize(self.block_cy)
            self.move_x = self._limit(self.move_x, -200, 150)
            self.move_y = self._limit(self.move_y, -100, 250)
            kinematics_move(self.move_x, self.move_y, 60, 100)
            if abs(self.block_cx - self.TARGET_CX) <= 15 and abs(self.block_cy - self.TARGET_CY) <= 15:
                self.success_cnt += 1
                if self.success_cnt >= 2:
                    self.success_cnt = 0
                    self.mark_flag = 1
                    # 对准后加延伸偏移 (与排序版 stage7 一致)
                    l = math.hypot(self.move_x, self.move_y)
                    if l > 0:
                        sin_a, cos_a = self.move_y / l, self.move_x / l
                        self.bak_cx = int((l + self.place_offset_x) * cos_a)
                        self.bak_cy = int((l + self.place_offset_y) * sin_a)
                    else:
                        self.bak_cx = self.move_x
                        self.bak_cy = self.move_y
                    self.move_x = self.bak_cx
                    self.move_y = self.bak_cy
                    kinematics_move(self.move_x, self.move_y, 60, 1000)
                    time.sleep(1)
                    self.move_status = 6
                    self.get_logger().info(f'[st5] 堆放基准确认: ({self.bak_cx},{self.bak_cy})')
            else:
                self.success_cnt = 0
            self.color_read_succed = 0

    # ── stage 6: 放置 (分层码垛) ─────────────────────────────────────────────
    def _st6(self):
        self.block_cx = self.block_cy = 0
        self.move_status = 7
        if self.block_cnt == 0:
            h = self.stack_height_one
        elif self.block_cnt == 1:
            h = self.stack_height_two
        else:
            h = self.stack_height_three
        kinematics_move(self.move_x + self.block_cnt * 2, self.move_y + 5, h, 1200)
        time.sleep(2.5)
        for _ in range(3):
            uart_send_str("#005P1200T1000!")
            time.sleep(0.4)
        kinematics_move(self.move_x, self.move_y, 130, 1000)
        time.sleep(1)

    # ── stage 7: 归位 ────────────────────────────────────────────────────────
    def _st7(self):
        self.move_x, self.move_y = 0, 120
        self.block_cnt += 1
        self.block_cx = self.block_cy = 0
        if not kinematics_move(self.move_x, self.move_y, 150, 1000, alpha_hint=-82):
            self.get_logger().error(f'[st7] 归位 IK 无解')
            kinematics_move(0, 105, 150, 1000, alpha_hint=-82)
        time.sleep(2)

        if self.block_cnt >= 3:
            self.block_cnt = 0
            self.mark_flag = 255
            self.stack_world_mm = None
            self.get_logger().info('[完成] 3 层码垛完成，重置')

        if self.target_colors:
            self.current_color_index = (self.current_color_index + 1) % len(self.target_colors)
        self.color_read_succed = 0
        self.move_status = 0
        self.world_target_mm = None
        self.object_depth_mm = 0
        self.get_logger().info(f'[完成] 下一颜色: {self._current_target()}')

    def _st8(self):
        """错误恢复"""
        self.move_x, self.move_y = 0, 120
        kinematics_move(0, 105, 150, 1000, alpha_hint=-82)
        time.sleep(2)
        self.color_read_succed = 0
        self.move_status = 0
        self.world_target_mm = None

    # ══ 辅助 ══════════════════════════════════════════════════════════════════

    @staticmethod
    def _draw_rect(frame, rect, color, label):
        cx, cy = int(rect[0][0]), int(rect[0][1])
        box = cv2.boxPoints(rect)
        box_i = np.intp(box)
        cv2.drawContours(frame, [box_i], -1, color, 2)
        cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 18, 2)
        cv2.putText(frame, f"{label} ({cx},{cy})",
                    (int(box_i[:, 0].min()), max(int(box_i[:, 1].min()) - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)


# ═══════════════════════════════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════════════════════════════

    # ══ enter/exit 服务 ════════════════════════════════════════════════════════

    def enter_callback(self, request, response):
        self.get_logger().info('收到Enter服务，启动深度颜色码垛！')
        if not self.active:
            try:
                if not setup_uart(115200):
                    response.success = False
                    response.message = '串口初始化失败'
                    return response
                uart_send_str('{#000P1500T1000!#001P1432T1000!#002P1871T1000!#003P0666T1000!#004P1481T1000!}')
                time.sleep(1)
                # 重置状态
                self.move_status = 0
                self.move_x, self.move_y, self.move_z = 0, 105, 150
                self.block_cx, self.block_cy = 320, 240
                self.color_read_succed = 0
                self.success_cnt = 0
                self.active = True
                self.stack_active = True
                self._run_thread = threading.Thread(target=self.run, daemon=True)
                self._run_thread.start()
            except Exception as e:
                self.get_logger().error(f'硬件初始化失败: {e}')
                response.success = False
                response.message = f'硬件初始化失败: {e}'
                return response
        response.success = True
        response.message = '深度颜色码垛已启动'
        return response

    def exit_callback(self, request, response):
        self.get_logger().info('收到Exit服务，停止深度颜色码垛！')
        if self.active:
            self.active = False
            self.stack_active = False
            close_uart()
            if self._run_thread and self._run_thread.is_alive():
                self._run_thread.join(timeout=3.0)
        response.success = True
        response.message = '深度颜色码垛已停止'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DepthColorStackNode()
    exec_ = MultiThreadedExecutor(); exec_.add_node(node)
    try:
        exec_.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.active = False
        node.stack_active = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
