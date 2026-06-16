#!/usr/bin/env python3
"""
yolo_track_pick_node.py — YOLO 三维追踪抓取节点

结合 Aurora 930 深度相机 + YOLO 检测 + PID 追踪，实现：
  1. YOLO 识别目标物体
  2. PID 控制舵机追踪，保持目标处于画面中心
  3. 目标停止运动 1 秒后，扫描位置并执行抓取
  4. 抓取流程：开夹爪 → 飞向目标 → 关夹爪 → 回初始位置

前提: yolo_detect_node.py 必须在运行（提供 /yolo/detections 话题）

话题:
  订阅:
    /yolo/detections         (JSON) — YOLO 检测结果
    /aurora/rgb/image_raw    (bgr8)
    /aurora/depth/image_raw  (mono16, mm)
    /aurora/rgb/camera_info  (CameraInfo)
  发布:
    /yolo_track_pick/image_result  (调试画面)
    /yolo_track_pick/status        (JSON — 状态)
  服务:
    /yolo_track_pick/enter  (Trigger) — 启动追踪抓取
    /yolo_track_pick/exit   (Trigger) — 停止
  接收指令:
    /yolo_track_pick/cmd    (JSON) — {"shape": "sphere", "color": "红"}
"""

import os, sys, time, json, re, threading
import numpy as np, cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from example_interfaces.srv import Trigger

# ── 路径 ──
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
for p in (_SCRIPT_DIR, os.path.expanduser('~/ros2_ws/src/my_srv/scripts'),
          os.path.expanduser('~/OpenCV')):
    if p not in sys.path:
        sys.path.insert(0, p)

import z_uart
import arm_fk
from z_uart import uart_send_str, setup_uart, close_uart
from z_move import kinematics_move
from depth_utils import DepthUtils

# ── 中文形状 → YOLO 类别 ──
SHAPE_TO_YOLO = {
    '长方体': 'cuboid', '正方体': 'screwdriver', '圆柱体': 'cube',
    '球体': 'sphere', '圆球': 'sphere', '螺丝刀': 'cylinder',
}

# ── 颜色 ──
COLOR_MAP = {'红色': 'red', '绿色': 'green', '蓝色': 'blue',
             '红': 'red', '绿': 'green', '蓝': 'blue'}

# ── 初始位置 ──
INIT_POS = '#000P1500T1000!#001P1883T1000!#002P1939T1000!#003P0823T1000!#004P1500T1000!'

# ── PID ──
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


class YoloTrackPickNode(Node):
    def __init__(self):
        super().__init__('yolo_track_pick_node')
        self.du = DepthUtils(self)
        self.bridge = CvBridge()

        # ── 检测数据 ──
        self.latest_detections = []
        self._det_lock = threading.Lock()
        self.latest_depth = None
        self.latest_rgb = None
        self._frame_lock = threading.Lock()
        self.fx = self.fy = self.cx_cam = self.cy_cam = None

        # ── 状态 ──
        self.active = False
        self.picking = False
        self.target_shape = None    # YOLO 英文类别名
        self.target_color = None    # 英文颜色或 None

        # ── 舵机 ──
        self.servo0 = 1500  # 底座旋转
        self.servo2 = 1939  # 肩关节

        # ── PID (640x480) ──
        self.TARGET_CX, self.TARGET_CY = 320, 240
        self.pid_x = PIDController(kp=0.08, ki=0.0, kd=0.0)
        self.pid_y = PIDController(kp=0.08, ki=0.0, kd=0.0)

        # ── 运动检测 ──
        self.pos_history = []       # [(cx, cy, timestamp), ...]
        self.stable_start = None    # 开始稳定的时间
        self.STABLE_VAR_THRESH = 60  # 像素方差阈值
        self.STABLE_WAIT = 1.0      # 稳定等待秒数

        # ── 调试帧计数 ──
        self._dbg_fc = 0

        # ── QoS ──
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        # ── 订阅 ──
        self.create_subscription(String, '/yolo/detections', self._det_cb, 10)
        self.create_subscription(Image, '/aurora/rgb/image_raw', self._rgb_cb, qos)
        self.create_subscription(Image, '/aurora/depth/image_raw', self._depth_cb, qos)
        self.create_subscription(CameraInfo, '/aurora/rgb/camera_info', self._info_cb, qos)
        self.create_subscription(String, '/yolo_track_pick/cmd', self._cmd_cb, 10)

        # ── 发布 ──
        self.debug_pub = self.create_publisher(Image, '/yolo_track_pick/image_result', 10)
        self.status_pub = self.create_publisher(String, '/yolo_track_pick/status', 10)

        # ── 服务 ──
        self.create_service(Trigger, '/yolo_track_pick/enter', self._enter_cb)
        self.create_service(Trigger, '/yolo_track_pick/exit', self._exit_cb)

        self._run_thread = None
        self.get_logger().info('\033[1;36m[YoloTrackPick]\033[0m 三维追踪抓取节点已启动')

    # ══════════════════════════════════════════════════════════════════════════
    #  回调
    # ══════════════════════════════════════════════════════════════════════════

    def _info_cb(self, msg):
        if self.fx is not None:
            return
        K = np.array(msg.k).reshape(3, 3)
        self.fx, self.fy = K[0, 0], K[1, 1]
        self.cx_cam, self.cy_cam = K[0, 2], K[1, 2]
        self.get_logger().info(f'[YoloTrackPick] 内参: fx={self.fx:.1f} fy={self.fy:.1f}')

    def _rgb_cb(self, msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._frame_lock:
                self.latest_rgb = rgb
        except Exception:
            pass

    def _depth_cb(self, msg):
        try:
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            with self._frame_lock:
                self.latest_depth = depth
                self.du.latest_depth = depth
        except Exception:
            pass

    def _det_cb(self, msg):
        try:
            with self._det_lock:
                self.latest_detections = json.loads(msg.data)
        except Exception:
            pass

    def _cmd_cb(self, msg):
        """接收抓取指令 JSON: {"shape": "sphere", "color": "红"}"""
        try:
            cmd = json.loads(msg.data)
            shape_cn = cmd.get('shape', '')
            color_cn = cmd.get('color', '')
            self.target_shape = SHAPE_TO_YOLO.get(shape_cn, shape_cn)
            self.target_color = COLOR_MAP.get(color_cn, '') or None
            self.get_logger().info(
                f'[YoloTrackPick] 目标: shape={self.target_shape} color={self.target_color}')
        except Exception as e:
            self.get_logger().error(f'指令解析失败: {e}')

    def _publish_status(self, status, message):
        msg = String()
        msg.data = json.dumps({'status': status, 'message': message}, ensure_ascii=False)
        self.status_pub.publish(msg)
        self.get_logger().info(f'[{status}] {message}')

    # ══════════════════════════════════════════════════════════════════════════
    #  服务
    # ══════════════════════════════════════════════════════════════════════════

    def _enter_cb(self, request, response):
        self.get_logger().info('收到 Enter 服务，启动追踪抓取！')
        if self.active:
            response.success = True
            response.message = '已在运行'
            return response
        try:
            if not setup_uart(115200):
                response.success = False
                response.message = '串口初始化失败'
                return response
            # 回到初始位置
            uart_send_str(INIT_POS)
            time.sleep(1.5)
            self.servo0 = 1500
            self.servo2 = 1939
            self.active = True
            self.picking = False
            self._run_thread = threading.Thread(target=self._track_loop, daemon=True)
            self._run_thread.start()
        except Exception as e:
            response.success = False
            response.message = f'初始化失败: {e}'
            return response
        response.success = True
        response.message = '追踪抓取已启动'
        return response

    def _exit_cb(self, request, response):
        self.get_logger().info('收到 Exit 服务，停止追踪抓取！')
        self.active = False
        self.picking = False
        close_uart()
        if self._run_thread and self._run_thread.is_alive():
            self._run_thread.join(timeout=3.0)
        response.success = True
        response.message = '追踪抓取已停止'
        return response

    # ══════════════════════════════════════════════════════════════════════════
    #  目标查找
    # ══════════════════════════════════════════════════════════════════════════

    def _find_target(self):
        """从 YOLO 检测结果中查找目标，返回 (cx, cy, depth_mm, det) 或 None"""
        if self.target_shape is None:
            return None
        with self._det_lock:
            dets = list(self.latest_detections)

        # 按形状过滤
        candidates = [d for d in dets
                      if d['shape'] == self.target_shape and d.get('depth_mm', 0) > 100]

        if not candidates:
            return None

        # 按颜色过滤
        if self.target_color:
            colored = [d for d in candidates if d['color'] == self.target_color]
            if not colored:
                return None
            candidates = colored

        # 取置信度最高的
        candidates.sort(key=lambda d: d.get('confidence', 0), reverse=True)
        det = candidates[0]
        cx, cy = det['pixel']
        depth_mm = det.get('depth_mm', 0)
        return cx, cy, depth_mm, det

    # ══════════════════════════════════════════════════════════════════════════
    #  运动检测
    # ══════════════════════════════════════════════════════════════════════════

    def _check_stable(self, cx, cy):
        """检查目标是否已停止运动，返回 True 表示已稳定 STABLE_WAIT 秒"""
        now = time.time()
        self.pos_history.append((cx, cy, now))
        # 保留最近 1.5 秒的数据
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

    def _get_depth_at(self, px, py):
        """获取指定像素深度 (mm)，带邻域搜索"""
        with self._frame_lock:
            dimg = self.latest_depth
        if dimg is None:
            return 0
        for r in range(0, 10):
            for dy in range(-r, r + 1, max(1, r)):
                for dx in range(-r, r + 1, max(1, r)):
                    ux, uy = int(px + dx), int(py + dy)
                    if 0 <= uy < dimg.shape[0] and 0 <= ux < dimg.shape[1]:
                        d = int(dimg[uy, ux])
                        if 100 < d < 4000:
                            return d
        return 0

    def _compute_world_xyz(self, px, py, depth_mm):
        """像素坐标 + 深度 → 基座坐标 (mm)"""
        if self.fx is None or depth_mm <= 100:
            return None

        # pixel → camera 3D (m)
        z = depth_mm / 1000.0
        x = (px - self.cx_cam) * z / self.fx
        y = (py - self.cy_cam) * z / self.fy
        p_cam = np.array([x, y, z])

        # camera → base
        try:
            pwms = self._read_joint_pwms()
            if pwms is None:
                self.get_logger().warn('读取关节PWM失败')
                return None
            th = arm_fk.pwms_to_angles(*pwms)
            T_g2b_mm = arm_fk.compute_T_base_to_ee_from_angles(*th)
            T_g2b = arm_fk.T_mm_to_m(T_g2b_mm)

            calib_dir = os.path.expanduser('~/ros2_ws/src/my_srv/config')
            import yaml
            with open(os.path.join(calib_dir, 'hand_eye_calib.yaml')) as f:
                data = yaml.safe_load(f)
            R_c2g = np.array(data['R_cam2gripper'])
            t_c2g = np.array(data['t_cam2gripper']).reshape(3, 1)
            T_c2g = np.eye(4)
            T_c2g[:3, :3] = R_c2g
            T_c2g[:3, 3] = t_c2g.flatten()

            p = np.append(p_cam, 1.0)
            p_base = T_g2b @ T_c2g @ p
            return tuple(float(v) * 1000 for v in p_base[:3])
        except Exception as e:
            self.get_logger().warn(f'坐标转换失败: {e}')
            return None

    # ══════════════════════════════════════════════════════════════════════════
    #  抓取
    # ══════════════════════════════════════════════════════════════════════════

    def _execute_grasp(self, det):
        """执行抓取流程：开夹爪 → 飞向目标 → 关夹爪 → 回初始位置"""
        self.picking = True
        px, py = det['pixel']
        depth_mm = det.get('depth_mm', 0)
        label = f"{det.get('color', '')} {det['shape']}"

        try:
            # 1. 扫描目标世界坐标
            self._publish_status('scanning', f'扫描 {label} 位置...')
            world_xyz = self._compute_world_xyz(px, py, depth_mm)
            if world_xyz is None:
                self._publish_status('error', f'无法计算 {label} 的3D坐标')
                self.picking = False
                return False

            tx, ty, tz = world_xyz
            self.get_logger().info(f'世界坐标: ({tx:.0f}, {ty:.0f}, {tz:.0f})mm')

            # 2. 打开夹爪
            self._publish_status('grasping', '打开夹爪...')
            for _ in range(3):
                uart_send_str("#005P1000T500!")
                time.sleep(0.3)

            # 3. 先飞到目标上方（安全高度），再下降
            hover_z = max(int(tz) + 80, 80)
            self._publish_status('grasping', f'飞向 {label} 上方...')
            if not kinematics_move(int(tx), int(ty), hover_z, 1000):
                # 尝试不用 alpha_hint
                if not kinematics_move(int(tx), int(ty), hover_z, 1000, alpha_hint=None):
                    self._publish_status('error', f'IK无解：上方位置不可达')
                    self.picking = False
                    return False
            time.sleep(1.2)

            # 4. 下降到抓取高度
            grab_z = max(int(tz) - 5, 5)
            self._publish_status('grasping', f'下降抓取...')
            if not kinematics_move(int(tx), int(ty), grab_z, 800):
                if not kinematics_move(int(tx), int(ty), grab_z, 800, alpha_hint=None):
                    self._publish_status('error', f'IK无解：抓取位置不可达')
                    self.picking = False
                    return False
            time.sleep(1.0)

            # 5. 关闭夹爪
            self._publish_status('grasping', '夹取物体...')
            for _ in range(3):
                uart_send_str("#005P1700T1000!")
                time.sleep(0.4)

            # 6. 抬升
            kinematics_move(int(tx), int(ty), hover_z, 800)
            time.sleep(1.0)

            # 7. 回到初始位置
            self._publish_status('grasping', '回到初始位置...')
            uart_send_str(INIT_POS)
            time.sleep(1.5)
            self.servo0 = 1500
            self.servo2 = 1939

            # 重置状态
            self.pid_x.reset()
            self.pid_y.reset()
            self.pos_history.clear()
            self.stable_start = None

            self._publish_status('done', f'{label} 抓取完成')
            return True

        except Exception as e:
            self._publish_status('error', f'抓取异常: {e}')
            # 恢复到初始位置
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

    # ══════════════════════════════════════════════════════════════════════════
    #  主追踪循环
    # ══════════════════════════════════════════════════════════════════════════

    def _track_loop(self):
        if not self.du.wait_for_intrinsics(15.0):
            self.get_logger().error('[YoloTrackPick] 内参超时')
            return

        # 等待 YOLO 检测数据
        self._publish_status('waiting', '等待 YOLO 检测数据...')
        for _ in range(50):
            with self._det_lock:
                if self.latest_detections:
                    break
            time.sleep(0.1)
        else:
            self._publish_status('error', '未收到 YOLO 检测结果')
            return

        # 等待 RGB 帧
        for _ in range(30):
            with self._frame_lock:
                if self.latest_rgb is not None:
                    break
            time.sleep(0.1)

        self._publish_status('ready', '追踪就绪，等待 /yolo_track_pick/cmd 指令')

        while self.active and rclpy.ok():
            if self.picking:
                time.sleep(0.1)
                continue

            # 查找目标
            result = self._find_target()
            if result is None:
                # 无目标时仍发布画面
                self._draw_debug(None, None, 0, None)
                time.sleep(0.05)
                continue

            cx, cy, depth_mm, det = result

            # PID 追踪 —— 保持目标在画面中心
            self.pid_x.Target_val = self.TARGET_CX
            self.pid_y.Target_val = self.TARGET_CY
            dx = self.pid_x.PID_Realize(cx)
            dy = self.pid_y.PID_Realize(cy)
            self.servo0 += int(dx)
            self.servo2 -= int(dy)
            self.servo0 = max(600, min(2400, self.servo0))
            self.servo2 = max(600, min(2400, self.servo2))
            uart_send_str("{{#000P{:0>4d}T0000!#002P{:0>4d}T0000!}}".format(
                self.servo0, self.servo2))

            # 运动检测
            if self._check_stable(cx, cy):
                self.get_logger().info('[YoloTrackPick] 目标已稳定，开始抓取')
                self._execute_grasp(det)
                continue

            # 调试画面（每帧都更新）
            self._draw_debug(cx, cy, depth_mm, det)

            time.sleep(0.03)

        self.get_logger().info('[YoloTrackPick] 追踪循环结束')

    def _draw_debug(self, cx, cy, depth_mm, det):
        """在 RGB 画面上绘制追踪信息并发布"""
        with self._frame_lock:
            frame = self.latest_rgb.copy() if self.latest_rgb is not None else None
        if frame is None:
            return

        # 画中心十字（PID 目标）
        cv2.drawMarker(frame, (self.TARGET_CX, self.TARGET_CY),
                       (255, 255, 255), cv2.MARKER_CROSS, 20, 1)

        if cx is not None and det is not None:
            # 画目标位置
            color_map = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}
            c = color_map.get(det.get('color', ''), (0, 255, 0))
            cv2.drawMarker(frame, (int(cx), int(cy)), c, cv2.MARKER_CROSS, 24, 2)

            # 画 bbox
            bbox = det.get('bbox', [])
            if len(bbox) == 4:
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), c, 2)

            # 标签
            label = f"{det.get('color', '')} {det['shape']} d={depth_mm}mm"
            cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)

        # HUD
        cv2.putText(frame, f"S0={self.servo0} S2={self.servo2}",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        target_info = f"target: {self.target_shape} {self.target_color or ''}"
        cv2.putText(frame, target_info, (10, 78), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (200, 200, 200), 1)

        if self.stable_start:
            elapsed = time.time() - self.stable_start
            cv2.putText(frame, f"STABLE {elapsed:.1f}s", (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = YoloTrackPickNode()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.active = False
        node.picking = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
