#!/usr/bin/env python3
"""
color_track_pick_node.py — 颜色追踪抓取节点

复用 depth_color_track_node.py 的 LAB 颜色检测 + PID 追踪架构，
目标稳定 1 秒后自动执行抓取。

功能:
  - LAB 颜色检测（红/蓝/绿），30fps 实时追踪
  - PID 控制底座(servo0) + 肩关节(servo2) 保持目标在画面中心
  - 目标停止运动 1 秒后执行抓取
  - 抓取流程：开夹爪 → 飞向目标 → 关夹爪 → 抬升 → 放置区 → 开夹爪 → 回初始位置

话题:
  订阅:
    /aurora/rgb/image_raw       (bgr8)
    /aurora/depth/image_raw     (mono16, mm)
    /color                      (String) — 追踪颜色: red/blue/green
  发布:
    /color_track_pick/image_result  (调试画面)
    /color_track_pick/status        (JSON)
  服务:
    /color_track_pick/enter  (Trigger) — 启动
    /color_track_pick/exit   (Trigger) — 停止
"""

import os, sys, time, re, json, threading
import numpy as np, cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from message_filters import ApproximateTimeSynchronizer
from cv_bridge import CvBridge
from example_interfaces.srv import Trigger

# ── 路径 ──
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
for p in (_SCRIPT_DIR, os.path.expanduser('~/ros2_ws/src/my_srv/scripts'),
          os.path.expanduser('~/OpenCV')):
    if p not in sys.path:
        sys.path.insert(0, p)

from depth_utils import DepthUtils
import z_uart
import arm_fk
import z_move
from z_uart import uart_send_str, setup_uart, close_uart
from z_move import kinematics_move


# ═══════════════════════════════════════════════════════════════════════════════
#  颜色阈值加载（与 depth_color_track_node.py 完全一致）
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
            if not line:
                continue
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

    def reset(self):
        self.last_error = 0.0
        self.sum_error = 0.0

    def PID_Realize(self, actual_val):
        err = self.Target_val - actual_val
        self.sum_error += err
        out = self.kp * err + self.ki * self.sum_error + self.kd * (err - self.last_error)
        self.last_error = err
        return out


# ═══════════════════════════════════════════════════════════════════════════════
#  初始位置
# ═══════════════════════════════════════════════════════════════════════════════

INIT_POS = '#000P1500T1000!#001P1883T1000!#002P1939T1000!#003P0823T1000!#004P1500T1000!'
DROP_POS = '#000P2189T1000!#001P1165T1000!#002P1707T1000!#003P0828T1000!#004P1500T1000!'


# ═══════════════════════════════════════════════════════════════════════════════
#  ColorTrackPickNode
# ═══════════════════════════════════════════════════════════════════════════════

class ColorTrackPickNode(Node):
    def __init__(self):
        super().__init__('color_track_pick_node')
        self.du = DepthUtils(self)
        self.bridge = CvBridge()

        self.latest_rgb = None
        self.latest_depth = None
        self._frame_lock = threading.Lock()
        self.width, self.height = 640, 480

        # ── 追踪状态 ──
        self.track_color = None         # 当前追踪颜色 (来自 /color 话题)
        self.detected_color = None
        self.target_rect = None
        self.block_cx = 0
        self.block_cy = 0
        self.target_depth_mm = 0

        # 舵机 PWM
        self.servo0 = 1500  # 底座旋转
        self.servo2 = 1939  # 肩关节

        # PID 目标: 640×480 中心
        self.TARGET_CX, self.TARGET_CY = 320, 240

        # 颜色阈值
        self.lower_red   = np.array(red_low[0:3], dtype=np.uint8)
        self.upper_red   = np.array(red_low[3:6], dtype=np.uint8)
        self.lower_blue  = np.array(blue_low[0:3], dtype=np.uint8)
        self.upper_blue  = np.array(blue_low[3:6], dtype=np.uint8)
        self.lower_green = np.array(green_low[0:3], dtype=np.uint8)
        self.upper_green = np.array(green_low[3:6], dtype=np.uint8)

        # PID
        self.pid_x = PIDController(kp=0.15, ki=0.0, kd=0.0)
        self.pid_y = PIDController(kp=0.15, ki=0.0, kd=0.0)

        # 追踪/抓取状态
        self.active = False
        self.track_active = False
        self.picking = False
        self._run_thread = None

        # ── 运动检测 ──
        self.pos_history = []       # [(cx, cy, timestamp), ...]
        self.stable_start = None
        self.STABLE_VAR_THRESH = 60
        self.STABLE_WAIT = 1.0
        self._last_grasp_time = 0   # 上次抓取时间（冷却用）

        # ── 订阅 ──
        self.color_sub = self.create_subscription(String, '/color', self._color_callback, 10)

        from message_filters import Subscriber as MfSub
        _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        rgb_sub = MfSub(self, Image, '/aurora/rgb/image_raw', _qos)
        depth_sub = MfSub(self, Image, '/aurora/depth/image_raw', _qos)
        self._sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        self._sync.registerCallback(self._synced_callback)

        # ── 发布 ──
        self.debug_pub = self.create_publisher(Image, '/color_track_pick/image_result', 10)
        self.status_pub = self.create_publisher(String, '/color_track_pick/status', 10)

        # ── 服务 ──
        self.enter_srv = self.create_service(Trigger, '/color_track_pick/enter', self._enter_cb)
        self.exit_srv = self.create_service(Trigger, '/color_track_pick/exit', self._exit_cb)

        self.get_logger().info('\033[1;36m[ColorTrackPick]\033[0m 颜色追踪抓取节点已启动')

    # ══════════════════════════════════════════════════════════════════════════
    #  回调
    # ══════════════════════════════════════════════════════════════════════════

    def _color_callback(self, msg: String):
        color = msg.data.strip().lower()
        if color in ('red', 'blue', 'green'):
            self.track_color = color
            self.get_logger().info(f'[Track] 追踪颜色: {color}')
        elif color == 'stop':
            self.track_color = None
            self.get_logger().info('[Track] 停止追踪')

    def _synced_callback(self, rgb_msg, depth_msg):
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

    def _publish_status(self, status, message):
        msg = String()
        msg.data = json.dumps({'status': status, 'message': message}, ensure_ascii=False)
        self.status_pub.publish(msg)
        self.get_logger().info(f'[{status}] {message}')

    # ══════════════════════════════════════════════════════════════════════════
    #  服务
    # ══════════════════════════════════════════════════════════════════════════

    def _enter_cb(self, request, response):
        self.get_logger().info('收到 Enter 服务，启动颜色追踪抓取！')
        if self.active:
            response.success = True
            response.message = '已在运行'
            return response
        try:
            if not setup_uart(115200):
                response.success = False
                response.message = '串口初始化失败'
                return response
            uart_send_str(INIT_POS)
            time.sleep(1.5)
            self.servo0 = 1500
            self.servo2 = 1939
            self.active = True
            self.track_active = True
            self.picking = False
            self._run_thread = threading.Thread(target=self.run, daemon=True)
            self._run_thread.start()
        except Exception as e:
            response.success = False
            response.message = f'初始化失败: {e}'
            return response
        response.success = True
        response.message = '颜色追踪抓取已启动'
        return response

    def _exit_cb(self, request, response):
        self.get_logger().info('收到 Exit 服务，停止颜色追踪抓取！')
        if self.active:
            self.active = False
            self.track_active = False
            self.picking = False
            close_uart()
            if self._run_thread and self._run_thread.is_alive():
                self._run_thread.join(timeout=3.0)
        response.success = True
        response.message = '颜色追踪抓取已停止'
        return response

    # ══════════════════════════════════════════════════════════════════════════
    #  颜色检测（与 depth_color_track_node.py 一致）
    # ══════════════════════════════════════════════════════════════════════════

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
            if min(w, h) < 15 or a < 500:
                continue
            if a > ba:
                ba = a
                best = (a, (cx, cy), rect)
        return best if best else (0, (0, 0), None)

    def _get_depth_at(self, cx, cy):
        """获取指定像素的深度值 (mm)，带邻域搜索"""
        with self._frame_lock:
            dimg = self.latest_depth
        if dimg is None:
            return 0
        for r in range(0, 6):
            for dy in range(-r, r + 1, max(1, r)):
                for dx in range(-r, r + 1, max(1, r)):
                    d = self.du.get_depth_at(int(cx + dx), int(cy + dy), dimg)
                    if d is not None and d >= 150:
                        return int(d)
        return 0

    # ══════════════════════════════════════════════════════════════════════════
    #  运动检测
    # ══════════════════════════════════════════════════════════════════════════

    def _check_stable(self, cx, cy):
        now = time.time()
        self.pos_history.append((cx, cy, now))
        self.pos_history = [(x, y, t) for x, y, t in self.pos_history if now - t < 1.5]

        if len(self.pos_history) < 5:
            self.stable_start = None
            return False

        xs = [p[0] for p in self.pos_history]
        ys = [p[1] for p in self.pos_history]
        var = np.var(xs) + np.var(ys)

        if var < self.STABLE_VAR_THRESH:
            if self.stable_start is None:
                self.stable_start = now
            elif now - self.stable_start >= self.STABLE_WAIT:
                return True
        else:
            self.stable_start = None
        return False

    # ══════════════════════════════════════════════════════════════════════════
    #  坐标转换
    # ══════════════════════════════════════════════════════════════════════════

    def _read_pwm(self, idx, timeout=0.5):
        uart_send_str(f"#{idx:03d}PRAD!")
        dl = time.time() + timeout
        while time.time() < dl:
            if z_uart.uart_get_ok:
                d = z_uart.uart_receive_buf
                z_uart.uart_receive_buf = ''
                z_uart.uart_get_ok = 0
                m = re.search(r'#\d{3}P(\d+)!', d)
                if m:
                    return int(m.group(1))
            time.sleep(0.005)
        return None

    def _read_joint_pwms(self):
        pwms = []
        for i in range(4):
            v = self._read_pwm(i)
            if v is None:
                return None
            pwms.append(v)
        return tuple(pwms)

    def _compute_world_xyz(self, cx, cy, depth_mm):
        """
        沿相机光轴搜索第一个 IK 可达的抓取点。
        相机在末端上方50mm，光轴 = R[:,2]（z_ee 方向）。
        从当前末端位置沿光轴步进搜索，找到第一个 IK 可达的位置。
        """
        try:
            pwms = self._read_joint_pwms()
            if pwms is None:
                return None
            th = arm_fk.pwms_to_angles(*pwms)
            T_g2b_mm = arm_fk.compute_T_base_to_ee_from_angles(*th)
            ee_pos = T_g2b_mm[:3, 3]
            R = T_g2b_mm[:3, :3]
            optical_axis = R[:, 2]  # 相机光轴方向（z_ee）

            # 目标步进 = 深度 - 30 (相机偏移50 + L3=110 → 50+depth-110=depth-60)
            target_step = max(int(depth_mm) - 60, 30)

            # 从目标步进往回搜索，找最近的可达点
            best_result = None
            for step in range(target_step, 20, -10):
                target = ee_pos + optical_axis * step
                ix, iy, iz = int(target[0]), int(max(target[1], 50)), int(max(target[2], 50))
                for alpha in range(0, -136, -1):
                    if z_move.kinematics_analysis(ix, iy, iz, alpha) == 0:
                        best_result = (float(ix), float(iy), float(iz), alpha, step)
                        break
                if best_result:
                    break

            # 如果目标步进不可达，从目标步进往前搜索
            if not best_result:
                for step in range(target_step + 10, int(depth_mm) + 50, 10):
                    target = ee_pos + optical_axis * step
                    ix, iy, iz = int(target[0]), int(max(target[1], 50)), int(max(target[2], 50))
                    for alpha in range(0, -136, -1):
                        if z_move.kinematics_analysis(ix, iy, iz, alpha) == 0:
                            best_result = (float(ix), float(iy), float(iz), alpha, step)
                            break
                    if best_result:
                        break

            if best_result:
                rx, ry, rz, ra, rs = best_result
                self.get_logger().info(
                    f'抓取点: ({int(rx)},{int(ry)},{int(rz)}) alpha={ra} '
                    f'step={rs}mm target_step={target_step}mm '
                    f'ee=({int(ee_pos[0])},{int(ee_pos[1])},{int(ee_pos[2])}) '
                    f'depth={depth_mm}mm')
                return (rx, ry, rz)

            self.get_logger().warn(f'沿光轴搜索无可达点 (depth={depth_mm}mm, target_step={target_step}mm)')
            return None

        except Exception as e:
            self.get_logger().warn(f'坐标转换失败: {e}')
            return None

    def _execute_grasp(self, cx, cy, depth_mm, color_name):
        """
        执行抓取流程（不回初始位，直接在追踪位抓取）：
        1. 用当前 FK 计算目标位置
        2. 开夹爪 → 飞向目标 → 关夹爪 → 抬升 → 放置区 → 开夹爪 → 回初始位置
        """
        self.picking = True
        self._last_grasp_time = time.time()
        label = f"{color_name} 物体"

        try:
            # 1. 用当前追踪位的 FK 计算世界坐标
            self._publish_status('scanning', f'计算 {label} 抓取位置...')
            world_xyz = self._compute_world_xyz(cx, cy, depth_mm)
            if world_xyz is None:
                self._publish_status('error', f'无法计算 {label} 的3D坐标')
                return False

            tx, ty, tz = world_xyz
            self.get_logger().info(f'抓取目标: ({tx:.0f}, {ty:.0f}, {tz:.0f})mm')

            # 2. 打开夹爪
            self._publish_status('grasping', '打开夹爪...')
            for _ in range(3):
                uart_send_str("#005P1000T500!")
                time.sleep(0.3)

            # 3. 飞向目标物体
            self._publish_status('grasping', f'飞向 {label}...')
            if not kinematics_move(int(tx), int(ty), int(tz) + 20, 1000):
                self._publish_status('error', f'IK无解：({int(tx)},{int(ty)},{int(tz)}) 不可达')
                return False
            time.sleep(1.5)

            # 4. 关闭夹爪
            self._publish_status('grasping', '夹取物体...')
            for _ in range(3):
                uart_send_str("#005P1700T1000!")
                time.sleep(0.4)

            # 5. 抬升
            safe_z = max(int(tz) + 80, 150)
            kinematics_move(int(tx), int(ty), safe_z, 800)
            time.sleep(1.0)

            # 6. 移动到放置区
            self._publish_status('placing', '移动到放置区...')
            uart_send_str(DROP_POS)
            time.sleep(1.5)

            # 7. 打开夹爪放下物体
            for _ in range(3):
                uart_send_str("#005P1000T500!")
                time.sleep(0.3)

            # 8. 回初始位置
            self._publish_status('grasping', '回到初始位置...')
            uart_send_str(INIT_POS)
            time.sleep(1.5)
            self.servo0 = 1500
            self.servo2 = 1939

            self._publish_status('done', f'{label} 抓取完成')
            return True

        except Exception as e:
            self._publish_status('error', f'抓取异常: {e}')
            try:
                uart_send_str(INIT_POS)
                time.sleep(1.5)
                self.servo0 = 1500
                self.servo2 = 1939
            except Exception:
                pass
            return False
        finally:
            self.picking = False
            self.pid_x.reset()
            self.pid_y.reset()
            self.pos_history.clear()
            self.stable_start = None

    # ══════════════════════════════════════════════════════════════════════════
    #  主循环（追踪 + 检测 + 抓取）
    # ══════════════════════════════════════════════════════════════════════════

    def run(self):
        if not self.du.wait_for_intrinsics(15.0):
            self.get_logger().error('[ColorTrackPick] 内参超时')
            return

        self.get_logger().info('[ColorTrackPick] 等待同步帧...')
        for _ in range(200):
            with self._frame_lock:
                if self.latest_rgb is not None:
                    break
            time.sleep(0.1)
        else:
            self.get_logger().error('[ColorTrackPick] 无同步帧')
            return

        self.track_active = True
        self.get_logger().info('\033[1;32m[ColorTrackPick]\033[0m 追踪就绪，等待 /color 指令')

        while self.track_active and rclpy.ok():
            if not self.active or self.picking:
                time.sleep(0.1)
                continue

            with self._frame_lock:
                if self.latest_rgb is None:
                    time.sleep(0.03)
                    continue
                frame = self.latest_rgb.copy()

            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

            # ── 颜色检测 ──
            self.detected_color = None
            self.target_rect = None
            found = False

            if self.track_color:
                if self.track_color == 'red':
                    _, _, rect = self._detect_color(cv2.inRange(lab, self.lower_red, self.upper_red))
                elif self.track_color == 'blue':
                    _, _, rect = self._detect_color(cv2.inRange(lab, self.lower_blue, self.upper_blue))
                elif self.track_color == 'green':
                    _, _, rect = self._detect_color(cv2.inRange(lab, self.lower_green, self.upper_green))
                else:
                    rect = None

                if rect:
                    self.target_rect = rect
                    self.block_cx, self.block_cy = rect[0]
                    self.detected_color = self.track_color
                    found = True
                    self.target_depth_mm = self._get_depth_at(self.block_cx, self.block_cy)
            else:
                # 无指定颜色时，检测所有颜色，选最大
                for cname, clo, chi in [
                    ('red', self.lower_red, self.upper_red),
                    ('blue', self.lower_blue, self.upper_blue),
                    ('green', self.lower_green, self.upper_green),
                ]:
                    _, _, rect = self._detect_color(cv2.inRange(lab, clo, chi))
                    if rect:
                        a = cv2.contourArea(cv2.boxPoints(rect))
                        if not found or a > cv2.contourArea(cv2.boxPoints(self.target_rect)):
                            self.target_rect = rect
                            self.block_cx, self.block_cy = rect[0]
                            self.detected_color = cname
                            found = True
                if found:
                    self.target_depth_mm = self._get_depth_at(self.block_cx, self.block_cy)

            # ── PID 追踪 ──
            if found and self.track_color:
                self.pid_x.Target_val = self.TARGET_CX
                self.pid_y.Target_val = self.TARGET_CY
                dx = self.pid_x.PID_Realize(self.block_cx)
                dy = self.pid_y.PID_Realize(self.block_cy)
                self.servo0 += int(dx)
                self.servo2 -= int(dy)
                self.servo0 = max(600, min(2400, self.servo0))
                self.servo2 = max(600, min(2400, self.servo2))
                uart_send_str("{{#000P{:0>4d}T0000!#002P{:0>4d}T0000!}}".format(
                    self.servo0, self.servo2))

                # ── 运动检测 → 抓取（带 3 秒冷却）──
                now = time.time()
                if self._check_stable(self.block_cx, self.block_cy)                         and now - self._last_grasp_time > 3.0:
                    self.get_logger().info('[ColorTrackPick] 目标已稳定，开始抓取')
                    self._execute_grasp(
                        self.block_cx, self.block_cy,
                        self.target_depth_mm, self.detected_color)

            # ── 绘制调试画面 ──
            color_map = {'red': (0, 0, 255), 'blue': (255, 0, 0), 'green': (0, 255, 0)}
            if found and self.target_rect:
                c = color_map.get(self.detected_color, (255, 255, 255))
                box = cv2.boxPoints(self.target_rect)
                box_i = np.intp(box)
                cv2.drawContours(frame, [box_i], -1, c, 2)
                cv2.drawMarker(frame, (int(self.block_cx), int(self.block_cy)),
                               c, cv2.MARKER_CROSS, 18, 2)
                label = f"{self.detected_color} d={self.target_depth_mm}mm"
                cv2.putText(frame, label,
                            (int(box_i[:, 0].min()), max(int(box_i[:, 1].min()) - 8, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 2)

            # 中心十字
            cv2.drawMarker(frame, (self.TARGET_CX, self.TARGET_CY),
                           (255, 255, 255), cv2.MARKER_CROSS, 16, 1)

            # HUD
            mode = f"Track: {self.track_color}" if self.track_color else "Track: auto"
            cv2.putText(frame, mode, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"S0={self.servo0} S2={self.servo2}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            if self.stable_start:
                elapsed = time.time() - self.stable_start
                cv2.putText(frame, f"STABLE {elapsed:.1f}s", (10, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
            except Exception:
                pass

            time.sleep(0.03)

        self.get_logger().info('[ColorTrackPick] 结束')


def main(args=None):
    rclpy.init(args=args)
    node = ColorTrackPickNode()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.active = False
        node.track_active = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
