#!/usr/bin/env python3
"""
depth_label_sorting_node.py — 深度增强标签分拣

使用 Aurora 930 深度相机 + AprilTag 标签检测 + 世界系定位。
检测逻辑参考 label_sorting_node.py，坐标定位参考 depth_color_sorting_node.py。

标签 ID: 1=红色区域, 2=绿色区域, 3=蓝色区域
"""

import os, sys, re, time, math, threading
import numpy as np, cv2
import pupil_apriltags

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer
from cv_bridge import CvBridge

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

# tag_id → 分拣区颜色
TAG_COLOR_MAP = {1: 'red', 2: 'green', 3: 'blue'}


class DepthLabelSortingNode(Node):
    def __init__(self):
        super().__init__('depth_label_sorting_node')
        self.du = DepthUtils(self)
        self.detector = pupil_apriltags.Detector()

        self.latest_rgb = None
        self.latest_depth = None
        self._frame_lock = threading.Lock()
        self.bridge = CvBridge()
        self.width, self.height = 640, 480

        # 状态机
        self.move_x, self.move_y, self.move_z = 0, 120, 60
        self.move_status = 0
        self.target_rect = None
        self.color_read_succed = 0
        self.TARGET_CX, self.TARGET_CY = 320, 240
        self.block_cx, self.block_cy = self.TARGET_CX, self.TARGET_CY
        self.spin_calw = 1500
        self.detected_color = None
        self.success_cnt = 0
        self.sorting_active = False
        self.active = False  # enter/exit 模式守卫
        self._run_thread = None
        self.object_depth_mm = 0
        self.world_target_mm = None
        self._last_logged_status = -1

        # 标签检测
        self.block_angle = 0
        self.tag_id = 0
        self.target_ids = [1, 2, 3]
        self.current_id_index = 0

        # 颜色阈值（分拣区颜色检测）
        self.lower_red   = np.array(red_low[0:3], dtype=np.uint8)
        self.upper_red   = np.array(red_low[3:6], dtype=np.uint8)
        self.lower_blue  = np.array(blue_low[0:3], dtype=np.uint8)
        self.upper_blue  = np.array(blue_low[3:6], dtype=np.uint8)
        self.lower_green = np.array(green_low[0:3], dtype=np.uint8)
        self.upper_green = np.array(green_low[3:6], dtype=np.uint8)
        self.red_rect = self.blue_rect = self.green_rect = None

        # PID
        self.pid_x = PIDController(kp=0.03, ki=0.000, kd=0.0)
        self.pid_y = PIDController(kp=0.03, ki=0.000, kd=0.0)

        self.debug_pub = self.create_publisher(Image, '/depth_label_sorting/image_result', 10)

        from message_filters import Subscriber as MfSub
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        rgb_sub = MfSub(self, Image, '/aurora/rgb/image_raw', _qos)
        depth_sub = MfSub(self, Image, '/aurora/depth/image_raw', _qos)
        self._sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        self._sync.registerCallback(self._synced_callback)

        # enter/exit 服务
        self.enter_srv = self.create_service(Trigger, '/depth_label_sorting/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/depth_label_sorting/exit', self.exit_callback)

        self.get_logger().info('\033[1;36m[DepthLabelSort]\033[0m 深度增强标签分拣已启动')

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

    @staticmethod
    def _limit(d, mn, mx):
        return max(mn, min(mx, d))

    def _detect_apriltag(self, gray, target_id):
        """检测指定 ID 的 AprilTag，返回 (center, angle) 或 None"""
        for tag in self.detector.detect(gray):
            if tag.tag_id == target_id:
                cx, cy = int(tag.center[0]), int(tag.center[1])
                angle = self._calc_tag_angle(tag.corners)
                return (cx, cy), angle, tag.tag_id
        return None, 0, 0

    def _detect_color(self, mask):
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
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

    @staticmethod
    def _calc_tag_angle(corners):
        edges = []
        for i in range(4):
            edges.append(corners[(i + 1) % 4] - corners[i])
        lengths = [np.linalg.norm(e) for e in edges]
        main = edges[np.argmax(lengths)]
        ang = math.degrees(math.atan2(main[1], main[0]))
        if ang > 45: ang -= 90
        if ang < -45: ang += 90
        if abs(ang) <= 10: ang = 0
        return -ang

    def _compute_world_target(self):
        cx, cy = int(self.block_cx), int(self.block_cy)
        with self._frame_lock:
            dimg = self.latest_depth
        if dimg is None:
            self.get_logger().warn('[世界] depth is None')
            return None
        dmm = None
        for r in range(0, 11, 2):
            for dy in range(-r, r + 1, max(1, r)):
                for dx in range(-r, r + 1, max(1, r)):
                    d = self.du.get_depth_at(cx + dx, cy + dy, dimg)
                    if d is not None and d >= 150:
                        dmm = d; break
                if dmm is not None: break
            if dmm is not None: break
        if dmm is None:
            self.get_logger().warn(f'[世界] depth无效 pix=({cx},{cy})')
            return None
        self.object_depth_mm = int(dmm)
        p_cam = self.du.pixel_to_3d(cx, cy, dmm)
        pwms = self._read_joint_pwms()
        if pwms is None:
            self.get_logger().warn('[世界] PWM读取失败')
            return None
        th = arm_fk.pwms_to_angles(*pwms)
        T_g2b = arm_fk.T_mm_to_m(arm_fk.compute_T_base_to_ee_from_angles(*th))
        p_base = self.du.transform_cam_to_base(p_cam, T_g2b)
        X, Y, Z = float(p_base[0]) * 1000, float(p_base[1]) * 1000, float(p_base[2]) * 1000
        self.get_logger().info(
            f'[世界] pix=({cx},{cy}) d={dmm}mm base=({X:.0f},{Y:.0f},{Z:.0f})mm')
        return (X, Y, Z)

    def _read_pwm(self, idx, timeout=1.0):
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
            if v is None: return None
            pwms.append(v)
        return tuple(pwms)

    def run(self):
        if not self.du.wait_for_intrinsics(15.0):
            self.get_logger().error('内参超时'); return
        self.du.load_hand_eye_calib()
        for _ in range(200):
            with self._frame_lock:
                if self.latest_rgb is not None: break
            time.sleep(0.1)
        else:
            self.get_logger().error('无同步帧'); return

        self.sorting_active = True
        self.move_x, self.move_y, self.move_z = 0, 105, 150
        kinematics_move(self.move_x, self.move_y, self.move_z, 1000, alpha_hint=-82)
        time.sleep(2.0)
        self.move_status = 0
        self.current_id_index = 0
        self.get_logger().info('\033[1;32m[DepthLabelSort]\033[0m 分拣启动')

        fc = 0
        while self.sorting_active and rclpy.ok():
            if not self.active:
                time.sleep(0.1)
                continue
            with self._frame_lock:
                if self.latest_rgb is None: time.sleep(0.03); continue
                frame = self.latest_rgb.copy()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            target_id = self._current_target()
            self.detected_color = None
            found = False

            # 颜色区域检测（分拣区红/绿/蓝）
            if self.move_status == 6:
                hh = self.height * 3 // 4
                m_r = cv2.inRange(lab, self.lower_red, self.upper_red); m_r[hh:, :] = 0
                m_g = cv2.inRange(lab, self.lower_green, self.upper_green); m_g[hh:, :] = 0
                m_b = cv2.inRange(lab, self.lower_blue, self.upper_blue); m_b[hh:, :] = 0
                _, _, self.red_rect = self._detect_color(m_r)
                _, _, self.green_rect = self._detect_color(m_g)
                _, _, self.blue_rect = self._detect_color(m_b)
                sort_color = TAG_COLOR_MAP.get(target_id, 'red')
                rect_map = {'red': self.red_rect, 'green': self.green_rect, 'blue': self.blue_rect}
                target_rect = rect_map.get(sort_color)
                if target_rect:
                    self.block_cx, self.block_cy = target_rect[0]
                    self.detected_color = sort_color
                    found = True
                    box = cv2.boxPoints(target_rect)
                    cv2.drawContours(frame, [np.intp(box)], 0, (0, 255, 0), 2)
                    cv2.putText(frame, f"{sort_color} area", (int(target_rect[0][0]) + 10, int(target_rect[0][1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                center, angle, tid = self._detect_apriltag(gray, target_id)
                if center:
                    self.block_cx, self.block_cy = center
                    self.block_angle = angle
                    self.tag_id = tid
                    self.detected_color = f'tag_{tid}'
                    found = True
                    cv2.circle(frame, center, 8, (0, 255, 0), 2)
                    cv2.putText(frame, f"TAG{tid} ({center[0]},{center[1]})",
                                (center[0] + 10, center[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.color_read_succed = 1 if found else 0
            fc += 1
            if fc % 150 == 0 or self._last_logged_status != self.move_status:
                self.get_logger().info(
                    f'[TAG] f={fc} target={target_id} found={found} st={self.move_status}')
                self._last_logged_status = self.move_status

            cv2.putText(frame, f"Target: TAG{target_id}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"Status: {self.move_status}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
            except Exception:
                pass

            if self.sorting_active and (self.color_read_succed or self.move_status >= 2):
                self._run_state_machine()
            elif self.move_status == 6:
                self._scan_at_place_zone()

            time.sleep(0.03)

        self.get_logger().info('[DepthLabelSort] 结束')

    def _current_target(self):
        return self.target_ids[self.current_id_index] if self.target_ids else 1

    def _run_state_machine(self):
        stages = [self._st0, self._st1, self._st2, self._st3, self._st4,
                  self._st5, self._st6, self._st7, self._st8, self._st9]
        if self.move_status < len(stages):
            stages[self.move_status]()

    def _st0(self):
        self.world_target_mm = self._compute_world_target()
        if self.world_target_mm is None:
            self.color_read_succed = 0
            self.get_logger().warn(f'[st0] 世界坐标计算失败, tag={self.tag_id} pix=({self.block_cx},{self.block_cy})')
            return
        self.color_read_succed = 1
        self.move_status = 1
        self.get_logger().info(f'[st0] TAG{self.tag_id} XYZ={self.world_target_mm}')

    def _st1(self):
        self.move_status = 2
        self.spin_calw = self._limit(int(1500 - self.block_angle * 7.4), 1167, 1833)
        for _ in range(3):
            uart_send_str("#004P{:0^4}T800!".format(self.spin_calw))
            time.sleep(0.15)
        uart_send_str("#005P1000T500!")
        time.sleep(0.3)

    def _st2(self):
        self.move_status = 3
        if self.world_target_mm is None: return
        tx, ty, tz = self.world_target_mm
        hover_z = max(int(tz) + 80, 60)
        self.move_x, self.move_y, self.move_z = int(tx), int(ty), hover_z
        if not kinematics_move(self.move_x, self.move_y, hover_z, 1500, alpha_hint=-82):
            self.move_status = 9; return
        time.sleep(1.6)

    def _st3(self):
        self.move_status = 4
        if self.world_target_mm:
            tx, ty, tz = self.world_target_mm
            gz = max(int(tz) - 5, 5)
            self.move_x, self.move_y, self.move_z = int(tx), int(ty), gz
            if not kinematics_move(self.move_x, self.move_y, gz, 1200, alpha_hint=-82):
                self.move_status = 9; return
            time.sleep(1.3)
        for _ in range(3):
            uart_send_str("#005P1700T1000!")
            time.sleep(0.4)

    def _st4(self):
        self.move_status = 5
        self.block_cx = self.block_cy = 0
        if not kinematics_move(self.move_x, self.move_y, 150, 1000, alpha_hint=-82):
            self.move_status = 9; return
        time.sleep(1)

    def _st5(self):
        self.move_x, self.move_y = -130, 60
        kinematics_move(self.move_x, self.move_y, 150, 1000)
        time.sleep(1)
        uart_send_str("#004P1500T1500!")
        time.sleep(0.5)
        kinematics_move(self.move_x, self.move_y, 60, 1000)
        time.sleep(2.5)
        self.color_read_succed = 0
        self.move_status = 6

    def _scan_at_place_zone(self):
        """未检测到颜色区域时，左右扫描搜索"""
        if self.block_cx > self.TARGET_CX:
            self.move_y = self._limit(self.move_y + 2, -100, 250)
        else:
            self.move_y = self._limit(self.move_y - 2, -100, 250)
        kinematics_move(self.move_x, self.move_y, 60, 100)
        time.sleep(0.1)

    def _st6(self):
        # PID 对准分拣区颜色区域（参考 depth_color_sorting_node）
        if not self.color_read_succed:
            return
        self.pid_x.Target_val = self.TARGET_CX
        self.pid_y.Target_val = self.TARGET_CY
        self.move_y -= self.pid_x.PID_Realize(self.block_cx)
        self.move_x -= self.pid_y.PID_Realize(self.block_cy)
        self.move_x = self._limit(self.move_x, -200, 150)
        self.move_y = self._limit(self.move_y, -100, 250)
        kinematics_move(self.move_x, self.move_y, 60, 100)
        if abs(self.block_cx - self.TARGET_CX) <= 15 and abs(self.block_cy - self.TARGET_CY) <= 15:
            self.success_cnt += 1
            if self.success_cnt >= 3:
                self.success_cnt = 0
                self.move_status = 7
        else:
            self.success_cnt = 0
        self.color_read_succed = 0

    def _st7(self):
        # 延径向延伸 + 下降到放置高度
        self.move_status = 8
        l = math.hypot(self.move_x, self.move_y)
        if l > 0:
            s, c = self.move_y / l, self.move_x / l
            self.move_x = int((l + 65) * c)
            self.move_y = int((l + 65) * s)
        kinematics_move(self.move_x, self.move_y, 60, 1000)
        time.sleep(1)
        kinematics_move(self.move_x, self.move_y, 15, 1000)
        time.sleep(1)

    def _st8(self):
        # 松爪 + 抬升
        self.move_status = 9
        for _ in range(3):
            uart_send_str("#005P1200T1000!")
            time.sleep(0.4)
        kinematics_move(self.move_x, self.move_y, 70, 1000)
        time.sleep(1)

    def _st9(self):
        # 归位
        self.move_x, self.move_y = 0, 105
        self.block_cx = self.block_cy = 0
        kinematics_move(self.move_x, self.move_y, 150, 1000, alpha_hint=-82)
        time.sleep(2)

        self.current_id_index += 1
        if self.current_id_index >= len(self.target_ids):
            self.current_id_index = 0
            self.get_logger().info('[完成] 所有标签分拣完成，重置')

        self.color_read_succed = 0
        self.move_status = 0
        self.world_target_mm = None
        self.object_depth_mm = 0
        self.get_logger().info(f'[完成] 下一标签: TAG{self._current_target()}')



    def enter_callback(self, request, response):
        self.get_logger().info('收到Enter服务，启动深度标签分拣！')
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
                self.sorting_active = True
                self._run_thread = threading.Thread(target=self.run, daemon=True)
                self._run_thread.start()
            except Exception as e:
                self.get_logger().error(f'硬件初始化失败: {e}')
                response.success = False
                response.message = f'硬件初始化失败: {e}'
                return response
        response.success = True
        response.message = '深度标签分拣已启动'
        return response

    def exit_callback(self, request, response):
        self.get_logger().info('收到Exit服务，停止深度标签分拣！')
        if self.active:
            self.active = False
            self.sorting_active = False
            close_uart()
            if self._run_thread and self._run_thread.is_alive():
                self._run_thread.join(timeout=3.0)
        response.success = True
        response.message = '深度标签分拣已停止'
        return response


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


    # ══ enter/exit 服务 ════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = DepthLabelSortingNode()
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
