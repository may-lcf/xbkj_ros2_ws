#!/usr/bin/env python3
"""
depth_color_sorting_node.py — 深度增强颜色分拣（完整重构版，v3）

设计理念（参考 color_rect_pick_d.py 的世界系定位 + color_sorting_node.py 的 PID 框架）:
  stage0: PID 对准 → 物体移到图像中心
  stage0 成功后: 一次性世界系定位（像素+深度+外参+当前臂位姿 → base XYZ）
  stage1: 夹爪旋转对齐
  stage2: 机械臂飞到世界目标正上方（开环，不依赖视觉）
  stage3: 下降到抓取深度 + 闭合夹爪
  stage4-9: 抬升 → 分拣区 → 放下 → 归位 → 下一个颜色

========== 用法 ==========
  python3 ~/ros2_ws/install/my_srv/lib/my_srv/depth_color_sorting_node.py

========== 话题 ==========
  订阅:
    /aurora/rgb/image_raw       (bgr8)
    /aurora/depth/image_raw     (mono16, mm)
  发布:
    /depth_color_sorting/image_result  (调试画面)
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

from example_interfaces.srv import Trigger

from depth_utils import DepthUtils
import arm_fk
import z_uart
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
    # lab_tuner.py 格式: L_min L_max A_min A_max B_min B_max
    # 转换为 lower=(L_min, A_min, B_min), upper=(L_max, A_max, B_max)
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
#  DepthColorSortingNode
# ═══════════════════════════════════════════════════════════════════════════════

class DepthColorSortingNode(Node):
    def __init__(self):
        super().__init__('depth_color_sorting_node')
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
        self.sorting_active = False
        self.active = False  # enter/exit 模式守卫
        self.object_depth_mm = 0
        self.world_target_mm = None
        self._last_logged_status = -1
        self._run_thread = None

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
        self.debug_pub = self.create_publisher(Image, '/depth_color_sorting/image_result', 10)

        # RGB + 深度同步
        from message_filters import Subscriber as MfSub
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        rgb_sub = MfSub(self, Image, '/aurora/rgb/image_raw', _qos)
        depth_sub = MfSub(self, Image, '/aurora/depth/image_raw', _qos)
        self._sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        self._sync.registerCallback(self._synced_callback)

        # enter/exit 服务
        self.enter_srv = self.create_service(Trigger, '/depth_color_sorting/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/depth_color_sorting/exit', self.exit_callback)

        self.get_logger().info('\033[1;36m[DepthColorSort]\033[0m v3 世界系定位架构已启动 (等待 enter)')

    # ══ 同步 ══════════════════════════════════════════════════════════════════

    def _synced_callback(self, rgb_msg: Image, depth_msg: Image):
        if not self.active:
            return
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            if not hasattr(self, "_msg_dbg"): self._msg_dbg = True
            if self._msg_dbg:
                self.get_logger().info(f"[DEBUG3] enc={rgb_msg.encoding} step={rgb_msg.step} data_len={len(rgb_msg.data)} h={rgb_msg.height} w={rgb_msg.width}")
                self._msg_dbg = False
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'mono16')
            self.height, self.width = rgb.shape[:2]
            # 🔧 调试：每 300 帧打印一次深度统计
            if not hasattr(self, '_depth_diag_cnt'):
                self._depth_diag_cnt = 0
            self._depth_diag_cnt += 1
            if self._depth_diag_cnt % 300 == 0:
                valid_mask = (depth > 0) & (depth >= 150)
                valid_count = int(np.count_nonzero(valid_mask))
                total_count = depth.size
                self.get_logger().info(
                    f'[深度诊断] 有效像素: {valid_count}/{total_count} '
                    f'({100.*valid_count/total_count:.1f}%)')
            with self._frame_lock:
                self.latest_rgb = rgb
                self.latest_depth = depth
                self.du.latest_depth = depth
        except Exception:
            pass

    # ══ enter/exit 服务 ════════════════════════════════════════════════════════

    def enter_callback(self, request, response):
        self.get_logger().info('收到Enter服务，启动深度颜色分拣！')
        if not self.active:
            try:
                if not setup_uart(115200):
                    response.success = False
                    response.message = '串口初始化失败'
                    return response
                uart_send_str(
                    '{#000P1500T1000!#001P1432T1000!#002P1871T1000!'
                    '#003P0666T1000!#004P1481T1000!}')
                time.sleep(1)
                # 重置状态机
                self.move_status = 0
                self.move_x, self.move_y, self.move_z = 0, 105, 150
                self.block_cx, self.block_cy = self.TARGET_CX, self.TARGET_CY
                self.color_read_succed = 0
                self.success_cnt = 0
                self.world_target_mm = None
                self.object_depth_mm = 0
                self.current_color_index = 0
                self.active = True
                self.sorting_active = True
                self._run_thread = threading.Thread(target=self.run, daemon=True)
                self._run_thread.start()
            except Exception as e:
                self.get_logger().error(f'硬件初始化失败: {e}')
                response.success = False
                response.message = f'硬件初始化失败: {e}'
                return response
        response.success = True
        response.message = '深度颜色分拣已启动'
        return response

    def exit_callback(self, request, response):
        self.get_logger().info('收到Exit服务，停止深度颜色分拣！')
        if self.active:
            self.active = False
            self.sorting_active = False
            close_uart()
            if self._run_thread and self._run_thread.is_alive():
                self._run_thread.join(timeout=3.0)
        response.success = True
        response.message = '深度颜色分拣已停止'
        return response

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

    def _detect_color(self, mask):
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
            if min(w, h) < 15 or a < 150: continue
            if a > ba:
                ba = a
                best = (a, (cx, cy), rect)
        return best if best else (0, (0, 0), None)

    # ══ 主循环 ════════════════════════════════════════════════════════════════

    def run(self):
        if not self.du.wait_for_intrinsics(15.0):
            self.get_logger().error('[DepthColorSort] 内参超时'); return
        self.du.load_hand_eye_calib()
        self.get_logger().info('[DepthColorSort] 等待同步帧...')
        for _ in range(200):
            with self._frame_lock:
                if self.latest_rgb is not None: break
            time.sleep(0.1)
        else:
            self.get_logger().error('[DepthColorSort] 无同步帧'); return

        self.sorting_active = True
        # 🔧 初始位置: FK 反算自手动标定 PWM [1500,1432,1871,666] → (0,118,83)
        #    传入 alpha_hint=-82 确保 kinematics_move 选择与手动 PWM 一致的 alpha
        self.move_x, self.move_y, self.move_z = 0, 105, 150
        kinematics_move(self.move_x, self.move_y, self.move_z, 1000, alpha_hint=-82)
        time.sleep(2.5)
        self.move_status = 0
        self.current_color_index = 0
        self.get_logger().info('\033[1;32m[DepthColorSort]\033[0m 分拣启动')

        fc = 0
        while self.sorting_active and rclpy.ok():
            if not self.active:
                time.sleep(0.1)
                continue
            with self._frame_lock:
                if self.latest_rgb is None: time.sleep(0.03); continue
                frame = self.latest_rgb.copy()
            if fc == 10:
                cv2.imwrite("/tmp/debug_frame.png", frame)
                self.get_logger().info("[DEBUG4] frame saved to /tmp/debug_frame.png")
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

            m_blue  = cv2.inRange(lab, self.lower_blue,  self.upper_blue)
            m_red   = cv2.inRange(lab, self.lower_red,   self.upper_red)
            m_green = cv2.inRange(lab, self.lower_green, self.upper_green)
            # DEBUG2: 检查图像和mask
            # DEBUG5: mask_red=0 时保存帧和LAB分析
            if fc % 300 == 0 and cv2.countNonZero(m_red) == 0:
                cv2.imwrite("/tmp/debug_mask0_frame.png", frame)
                cv2.imwrite("/tmp/debug_mask0_lab.png", lab)
                cv2.imwrite("/tmp/debug_mask0_mred.png", m_red)
                self.get_logger().info(f"[DEBUG5] mask_red=0! thresholds lower={self.lower_red} upper={self.upper_red}")
                self.get_logger().info(f"[DEBUG5] frame mean={frame.mean(axis=(0,1)).astype(int)} LAB L_mean={lab[:,:,0].mean():.1f} A_mean={lab[:,:,1].mean():.1f} B_mean={lab[:,:,2].mean():.1f}")
                self.get_logger().info(f"[DEBUG5] LAB A range=[{lab[:,:,1].min()},{lab[:,:,1].max()}] B range=[{lab[:,:,2].min()},{lab[:,:,2].max()}]")
                # Check pixels above threshold
                above_A = np.sum(lab[:,:,1] >= self.lower_red[1])
                above_B = np.sum(lab[:,:,2] >= self.lower_red[2])
                self.get_logger().info(f"[DEBUG5] pixels A>={self.lower_red[1]}: {above_A}  B>={self.lower_red[2]}: {above_B}")
            if fc % 300 == 0:
                self.get_logger().info(f"[DEBUG2] shape={frame.shape} mean={frame.mean(axis=(0,1)).astype(int)} mask_red={cv2.countNonZero(m_red)} mask_blue={cv2.countNonZero(m_blue)} mask_green={cv2.countNonZero(m_green)}")
            if self.move_status == 6:
                hh = self.height*3 // 4
                m_blue[hh:, :] = m_red[hh:, :] = m_green[hh:, :] = 0
            target = self._current_target()
            self.detected_color = None
            found = False

            # DEBUG: 检查mask和轮廓
            if fc % 300 == 0:
                mask_pixels = cv2.countNonZero(m_red) if target == "red" else 0
                k_dbg = np.ones((3,3), np.uint8)
                m_dbg = cv2.morphologyEx(m_red, cv2.MORPH_OPEN, k_dbg)
                m_dbg = cv2.morphologyEx(m_dbg, cv2.MORPH_CLOSE, k_dbg)
                cnts_dbg = cv2.findContours(m_dbg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                cnts_filtered = [(cv2.contourArea(c), min(cv2.minAreaRect(c)[1])) for c in cnts_dbg if cv2.contourArea(c) >= 150 and min(cv2.minAreaRect(c)[1]) >= 15]
                self.get_logger().info(f"[DEBUG] mask_pixels={mask_pixels} contours={len(cnts_dbg)} filtered={len(cnts_filtered)} details={cnts_filtered[:3]}")
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
                    f'[DETECT] f={fc} tgt={target} found={found} '
                    f'det={self.detected_color} st={self.move_status} '
                    f'pix=({self.block_cx:.0f},{self.block_cy:.0f})')
                self._last_logged_status = self.move_status

            # HUD + 掩膜小窗
            cv2.putText(frame, f"Target: {target}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"Status: {self.move_status}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            if found:
                cv2.putText(frame, f"DET: {self.detected_color}", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            # 掩膜小窗
            mask_for_debug = None
            if target == 'red':
                mask_for_debug = m_red
            elif target == 'blue':
                mask_for_debug = m_blue
            elif target == 'green':
                mask_for_debug = m_green
            if mask_for_debug is not None:
                mcol = cv2.cvtColor(mask_for_debug, cv2.COLOR_GRAY2BGR)
                mcol[:, :, 2] = mask_for_debug
                small = cv2.resize(mcol, (160, 120))
                h, w = frame.shape[:2]
                if h >= 130 and w >= 170:
                    frame[h-130:h-10, w-170:w-10] = small
                    cv2.putText(frame, "mask", (w-170, h-135),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
            except Exception:
                pass

            if self.sorting_active and (self.color_read_succed or self.move_status >= 3):
                self._run_state_machine()
            elif self.move_status == 6:
                self._scan_at_place_zone()

            time.sleep(0.03)

        self.get_logger().info('[DepthColorSort] 结束')

    def _current_target(self):
        return self.target_colors[self.current_color_index] if self.target_colors else None

    def _lock_rect(self):
        mp = {'red': self.red_rect, 'blue': self.blue_rect, 'green': self.green_rect}
        if self.detected_color in mp:
            self.target_rect = mp[self.detected_color]

    # ══ 世界系定位 ════════════════════════════════════════════════════════════

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
        cx, cy = int(self.block_cx), int(self.block_cy)
        MIN_DEPTH = 100   # mm，低于此视为无效

        # 🔧 带重试的深度搜索：深度图可能延迟填充
        dmm = None
        found_at = None
        for attempt in range(4):
            with self._frame_lock:
                dimg = self.latest_depth
            if dimg is None:
                time.sleep(0.05); continue

            # 第1轮: 5×5 (step=1)
            if dmm is None:
                for dy in range(-5, 6):
                    for dx in range(-5, 6):
                        d = self.du.get_depth_at(cx + dx, cy + dy, dimg)
                        if d is not None and d >= MIN_DEPTH:
                            dmm = d; found_at = (cx + dx, cy + dy); break
                    if dmm: break

            # 第2轮: 15×15 (step=1)
            if dmm is None:
                for dy in range(-15, 16):
                    for dx in range(-15, 16):
                        d = self.du.get_depth_at(cx + dx, cy + dy, dimg)
                        if d is not None and d >= MIN_DEPTH:
                            dmm = d; found_at = (cx + dx, cy + dy); break
                    if dmm: break

            if dmm is not None:
                break
            # 深度图尚未就绪，等一等再试
            if attempt < 3:
                time.sleep(0.05)

        if dmm is None:
            self.get_logger().warn(
                f'[世界] 深度无效 ({cx},{cy}) 邻域 30×30 + 重试均无效')
            return None

        if found_at and (found_at[0] != cx or found_at[1] != cy):
            self.get_logger().info(
                f'[世界] 邻域搜索成功: ({cx},{cy})→({found_at[0]},{found_at[1]}) d={dmm}mm')
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
        X, Y, Z = float(p_base_m[0]) * 1000, float(p_base_m[1]) * 1000, float(p_base_m[2]) * 1000
        self.get_logger().info(
            f'[世界] pix=({cx},{cy}) d={dmm}mm cam=({p_cam[0]:.3f},{p_cam[1]:.3f},{p_cam[2]:.3f}) '
            f'base=({X:.0f},{Y:.0f},{Z:.0f})mm')
        return (X, Y, Z)

    # ══ 状态机 ════════════════════════════════════════════════════════════════

    def _run_state_machine(self):
        stages = [self._st0, self._st1, self._st2, self._st3, self._st4,
                  self._st5, self._st6, self._st7, self._st8, self._st9]
        if self.move_status < len(stages):
            print(f'[Stage] 进入 stage {self.move_status}')
            stages[self.move_status]()

    # ── stage 0: 固定位姿观测 + 世界定位（参考 color_rect_pick_d.py）───────
    def _st0(self):
        # 不移臂！从当前观察位姿直接做一次性世界坐标计算
        # 原理与 color_rect_pick_d.py 一致：static observation → world target → open-loop fly
        self._lock_rect()
        self.world_target_mm = self._compute_world_target()
        if self.world_target_mm is None:
            self.get_logger().warn('[st0] 世界定位失败，保持观察位姿重试')
            self.color_read_succed = 0
            return
        self.color_read_succed = 1
        self.move_status = 1
        self.get_logger().info(
            f'[st0] {self.detected_color} 世界 XYZ=({self.world_target_mm[0]:.0f},'
            f'{self.world_target_mm[1]:.0f},{self.world_target_mm[2]:.0f})mm '
            f'pix=({self.block_cx:.0f},{self.block_cy:.0f})')

    # ── stage 1: 夹爪旋转 ────────────────────────────────────────────────────
    def _st1(self):
        self.move_status = 2
        ang = self._calc_angle(self.target_rect)
        self.spin_calw = self._limit(int(1500 - ang * 7.4), 1167, 1833)
        for _ in range(3):
            uart_send_str("#004P{:0^4}T800!".format(self.spin_calw))
            time.sleep(0.15)
        uart_send_str("#005P1000T500!")
        time.sleep(0.3)

    # ── stage 2: 飞到世界目标上方 ────────────────────────────────────────────
    def _st2(self):
        self.move_status = 3
        if self.world_target_mm is None: return
        tx, ty, tz = self.world_target_mm
        hover_z = max(int(tz) + 80, 60)
        self.move_x, self.move_y, self.move_z = int(tx), int(ty), hover_z
        self.get_logger().info(f'[st2] 飞到 ({self.move_x},{self.move_y},{hover_z})')
        if not kinematics_move(self.move_x, self.move_y, self.move_z, 1500, alpha_hint=-82):
            self.get_logger().error(f'[st2] IK 无解 → 目标({self.move_x},{self.move_y},{hover_z})超出工作空间')
            self.move_status = 9
            return
        time.sleep(1.6)
        # st0 计算的世界坐标就是最终目标，物体不移动，无需复算

    # ── stage 3: 下降抓取 ────────────────────────────────────────────────────
    def _st3(self):
        self.move_status = 4
        if self.world_target_mm:
            tx, ty, tz = self.world_target_mm
            gz = max(int(tz) - 5, 5)
            self.move_x, self.move_y, self.move_z = int(tx), int(ty), gz
            if not kinematics_move(self.move_x, self.move_y, gz, 1200, alpha_hint=-82):
                self.get_logger().error(f'[st3] IK 无解 → 目标({self.move_x},{self.move_y},{gz})超出工作空间')
                self.move_status = 9
                return
            time.sleep(1.3)
        for _ in range(3):
            uart_send_str("#005P1700T1000!")
            time.sleep(0.4)

    # ── stage 4: 抬升 ────────────────────────────────────────────────────────
    def _st4(self):
        self.move_status = 5
        self.move_z = 150
        if not kinematics_move(self.move_x, self.move_y, 150, 1000, alpha_hint=-82):
            self.get_logger().error(f'[st4] IK 无解 → ({self.move_x},{self.move_y},150)')
            self.move_status = 9
            return
        time.sleep(1)

    # ── stage 5: 旋转到分拣区 ────────────────────────────────────────────────
    def _st5(self):
        self.block_cx = self.block_cy = 0
        self.move_x, self.move_y = -130, 60
        kinematics_move(self.move_x, self.move_y, 150, 1000)
        time.sleep(1); uart_send_str("#004P1500T1500!"); time.sleep(0.5)
        kinematics_move(self.move_x, self.move_y, 60, 1000)
        time.sleep(2.5)
        self.color_read_succed = 0
        self.move_status = 6

    # ── stage 6: PID 对准分拣区 ──────────────────────────────────────────────
    def _st6(self):
        self.pid_x.Target_val = self.TARGET_CX
        self.pid_y.Target_val = self.TARGET_CY
        self.move_y -= self.pid_x.PID_Realize(self.block_cx)
        self.move_x -= self.pid_y.PID_Realize(self.block_cy)
        self.move_x = self._limit(self.move_x, -200, 150)
        self.move_y = self._limit(self.move_y, -100, 250)
        kinematics_move(self.move_x, self.move_y, 60, 100)
        if abs(self.block_cx - self.TARGET_CX) <= 15 and abs(self.block_cy - self.TARGET_CY) <= 15:
            self.success_cnt += 1
            if self.success_cnt >= 3: self.success_cnt = 0; self.move_status = 7; return
        else:
            self.success_cnt = 0
        self.color_read_succed = 0

    def _scan_at_place_zone(self):
        if self.block_cx > self.TARGET_CX:
            self.move_y = self._limit(self.move_y + 2, -100, 250)
        else:
            self.move_y = self._limit(self.move_y - 2, -100, 250)
        kinematics_move(self.move_x, self.move_y, 60, 100)
        time.sleep(0.1)

    # ── stages 7-9 ──────────────────────────────────────────────────────────
    def _st7(self):
        self.block_cx = self.block_cy = 0; self.move_status = 8
        l = math.hypot(self.move_x, self.move_y)
        s, c = (self.move_y / l, self.move_x / l) if l > 0 else (0, 1)
        self.move_x, self.move_y = int((l + 65) * c), int((l + 65) * s)
        kinematics_move(self.move_x, self.move_y, 60, 1000); time.sleep(1)
        kinematics_move(self.move_x, self.move_y, 15, 1000); time.sleep(1)

    def _st8(self):
        self.move_status = 9
        for _ in range(3): uart_send_str("#005P1200T1000!"); time.sleep(0.4)
        kinematics_move(self.move_x, self.move_y, 70, 1000); time.sleep(1)

    def _st9(self):
        self.move_x, self.move_y = 0, 120
        self.block_cx = self.block_cy = 0
        if not kinematics_move(self.move_x, self.move_y, 150, 1000, alpha_hint=-82):
            self.get_logger().error(f'[st9] 归位 IK 无解，尝试默认初始位姿')
            kinematics_move(0, 105, 150, 1000, alpha_hint=-82)
        time.sleep(2)
        if self.target_colors:
            self.current_color_index = (self.current_color_index + 1) % len(self.target_colors)
        self.color_read_succed = 0; self.move_status = 0
        self.world_target_mm = None; self.object_depth_mm = 0
        self.get_logger().info(f'[完成] 下一颜色: {self._current_target()}')

    # ══ 辅助 ══════════════════════════════════════════════════════════════════

    @staticmethod
    def _draw_rect(frame, rect, color, label):
        cx, cy = int(rect[0][0]), int(rect[0][1])
        box = cv2.boxPoints(rect)
        box_i = np.intp(box)
        cv2.drawContours(frame, [box_i], -1, color, 2)
        cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 18, 2)
        cv2.putText(frame, f"{label} ({cx},{cy})", (int(box_i[:, 0].min()), max(int(box_i[:, 1].min()) - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)


# ═══════════════════════════════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = DepthColorSortingNode()
    exec_ = MultiThreadedExecutor(); exec_.add_node(node)
    try:
        exec_.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.active = False
        node.sorting_active = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
