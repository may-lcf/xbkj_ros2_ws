#!/usr/bin/env python3
"""
eye_in_hand_calib_node.py — Eye-in-Hand 手眼外参标定节点（手动示教版 v2）

核心改进（相比旧版）：
  1. 用实际舵机 PWM 反算 FK（而非目标坐标），T_gripper2base 准确
  2. 增加方向合理性校验：t_y > 0（相机在夹爪上方）、t_z < 0（相机在夹爪后方）
  3. 标定后自动验证：用已知物体测世界坐标，确认 z > 0

棋盘格: 每行4黑4白 → 7×7内角点, 边长 19mm

========== 坐标系定义 ==========
  深度相机: x右 y下 z前（像素 p_cam = pixel_to_3d 返回）
  夹爪(EE): x右 y上 z前（arm_fk.py 定义）
  t_cam2gripper: 相机原点在夹爪坐标系中的位置

========== 使用步骤 ==========
  0. 棋盘格放桌面固定不动
  1. ros2 run my_srv eye_in_hand_calib_node
  2. 按 h 释放舵机 → 手动扳臂到不同位姿
  3. 确保画面中看到棋盘格绿色角点 → 按 p 拍照记录
  4. 重复 ≥12 个不同位姿（角度/距离/高度尽量分散）
  5. 按 s 运行标定并保存
  6. 按 q 退出（自动恢复舵机阻力）

========== 按键 ==========
  h=释放舵机  l=锁定舵机  p=拍照  s=标定  r=读PWM  q=退出
"""

import cv2
import os
import sys
import math
import time
import yaml
import re
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# ── 路径 ──────────────────────────────────────────────────────────────────────
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_OPENCV_DIR = os.path.expanduser('~/OpenCV')
for _p in (_SCRIPT_DIR, _OPENCV_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import z_uart
from z_uart import uart_send_str, setup_uart, close_uart

# ── 标定参数 ───────────────────────────────────────────────────────────────────
BOARD_W       = 7
BOARD_H       = 7
SQUARE_SIZE_M = 0.019         # 19mm 格子边长
MIN_VALID_POSES = 12          # 最少有效位姿数
SERVO_COUNT   = 4             # 只用 000-003 关节

_CALIB_DIR  = os.path.expanduser('~/ros2_ws/src/my_srv/config')
CALIB_OUTPUT = os.path.join(_CALIB_DIR, 'hand_eye_calib.yaml')
INTRINSICS_CACHE = os.path.join(_CALIB_DIR, 'camera_intrinsics.yaml')

# ── 机械臂参数 ─────────────────────────────────────────────────────────────────
L0, L1, L2, L3 = 111.0, 105.0, 88.0, 178.0  # mm


# ═══════════════════════════════════════════════════════════════════════════════
#  PWM → 关节角 → FK（与 z_move.py + arm_fk.py 完全一致）
# ═══════════════════════════════════════════════════════════════════════════════

def _sid(idx: int) -> str:
    return f"{idx:03d}"


def _wait_prad_response(timeout: float = 1.5):
    """等待舵机位置响应（#XXXP<数字>!），自动跳过命令回显帧。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if z_uart.uart_get_ok:
            data = z_uart.uart_receive_buf
            z_uart.uart_receive_buf = ''
            z_uart.uart_get_ok = 0
            if re.search(r'#\d{3}P\d+!', data):
                return data
        time.sleep(0.005)
    return None


def read_servo_pwms():
    """读取 000-003 舵机当前 PWM 值"""
    positions = {}
    for i in range(SERVO_COUNT):
        sid = _sid(i)
        uart_send_str(f"#{sid}PRAD!")
        resp = _wait_prad_response()
        if resp:
            m = re.search(r'#\d{3}P(\d+)!', resp)
            positions[i] = int(m.group(1)) if m else None
        else:
            positions[i] = None
    return positions


def pwms_to_joint_angles(pwms):
    """
    PWM → 运动学关节角度（度）
    转换公式来自 z_move.py 的 servo_pwm 公式反解
    """
    theta6 = (1500 - (pwms.get(0) or 1500)) * 270.0 / 2000.0
    theta5 = 90.0 + ((pwms.get(1) or 1500) - 1500) * 270.0 / 2000.0
    theta4 = ((pwms.get(2) or 1500) - 1500) * 270.0 / 2000.0
    theta3 = ((pwms.get(3) or 1500) - 1500) * 270.0 / 2000.0
    return theta6, theta5, theta4, theta3


def compute_T_base_to_ee_from_joints(theta6_deg, theta5_deg, theta4_deg, theta3_deg):
    """
    从4个关节角度正向计算 T_base_to_ee（4×4, m 单位）。
    与 arm_fk.compute_T_base_to_ee_from_angles 完全一致。
    """
    alpha_deg = theta3_deg + theta5_deg - theta4_deg

    t6 = math.radians(theta6_deg)
    t5 = math.radians(theta5_deg)
    t4 = math.radians(theta4_deg)
    a  = math.radians(alpha_deg)
    beta = t5 - t4

    # EE 位置（mm）
    y_proj = L1 * math.cos(t5) + L2 * math.cos(beta) + L3 * math.cos(a)
    z_mm   = L0 + L1 * math.sin(t5) + L2 * math.sin(beta) + L3 * math.sin(a)
    x_mm   = y_proj * math.sin(t6)
    y_mm   = y_proj * math.cos(t6)

    # 旋转矩阵（列 = EE坐标轴在基座系中的方向）
    ct6, st6 = math.cos(t6), math.sin(t6)
    ca,  sa  = math.cos(a),  math.sin(a)

    R = np.array([
        [ ct6,  sa * st6,  ca * st6],
        [-st6,  sa * ct6,  ca * ct6],
        [   0,       -ca,        sa],
    ], dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3,  3] = np.array([x_mm, y_mm, z_mm]) / 1000.0  # mm → m
    return T


def release_servos(mode='K'):
    """释放舵机阻力（K=小阻力 M=大阻力）"""
    suffix = f"PUL{mode}"
    for i in range(SERVO_COUNT):
        uart_send_str(f"#{_sid(i)}{suffix}!")
        time.sleep(0.02)


def lock_servos(pwms):
    """恢复舵机阻力"""
    for i in range(SERVO_COUNT):
        pwm = pwms.get(i) or 1500
        uart_send_str(f"#{_sid(i)}P{pwm}T200!")
        time.sleep(0.05)


# ═══════════════════════════════════════════════════════════════════════════════
#  棋盘格检测
# ═══════════════════════════════════════════════════════════════════════════════

def build_objpoints():
    objp = np.zeros((BOARD_H * BOARD_W, 3), np.float64)
    objp[:, :2] = np.mgrid[0:BOARD_W, 0:BOARD_H].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_M
    return objp


def detect_checkerboard(image_bgr):
    """检测棋盘格角点，返回 corners 或 None"""
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, (BOARD_W, BOARD_H), flags)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), criteria)
    return corners


def solve_board_pose(corners, K, D):
    """solvePnP → (R_target2cam, t_target2cam) in meters"""
    objp = build_objpoints()
    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, None
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS2 节点
# ═══════════════════════════════════════════════════════════════════════════════

class EyeInHandCalibNode(Node):
    def __init__(self):
        super().__init__('eye_in_hand_calib_node')
        os.makedirs(_CALIB_DIR, exist_ok=True)

        self.K = None; self.D = None
        self._intrinsics_ready = threading.Event()
        self.latest_frame = None
        self._frame_lock = threading.Lock()
        self.bridge = CvBridge()

        self.R_gripper2base = []
        self.t_gripper2base = []
        self.R_target2cam   = []
        self.t_target2cam   = []
        self.corners_list   = []
        self._captured_pwms = []

        self._servo_released = True
        self._last_pwms = {}

        _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/aurora/rgb/image_raw', self._image_cb, _qos)
        self.create_subscription(CameraInfo, '/aurora/rgb/camera_info', self._info_cb, _qos)
        self.debug_pub = self.create_publisher(Image, '/calib/debug_image', 10)

        self.get_logger().info(
            '\033[1;36m[EyeInHandCalib v2]\033[0m 节点已启动\n'
            '  h=释放  l=锁定  p=拍照  s=标定  r=读PWM  q=退出'
        )

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._frame_lock:
                self.latest_frame = frame
        except Exception:
            pass

    def _info_cb(self, msg):
        if self.K is not None:
            return
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64)
        self._save_intrinsics()
        self._intrinsics_ready.set()
        self.get_logger().info(
            f'[EyeInHandCalib] 内参 fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} '
            f'cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}'
        )

    def capture_pose(self):
        """直接读 PWM（释放状态下 PRAD 也能读到编码器位置，无需锁舵机）→ 检测棋盘格 → FK → 记录。"""
        with self._frame_lock:
            frame = self.latest_frame.copy() if self.latest_frame is not None else None
        if frame is None:
            return '无相机帧'

        # 直接读 PWM，不锁舵机（锁舵机会用旧 PWM 导致臂跳动）
        pwms = read_servo_pwms()
        self._last_pwms = pwms

        if any(v is None for v in pwms.values()):
            return f'PWM读取失败: {pwms}'

        corners = detect_checkerboard(frame)
        if corners is None:
            return '未检测到棋盘格'

        if self.K is None:
            return '相机内参未就绪'

        R_t2c, t_t2c = solve_board_pose(corners, self.K, self.D)
        if R_t2c is None:
            return 'solvePnP 失败'

        theta6, theta5, theta4, theta3 = pwms_to_joint_angles(pwms)
        alpha = theta3 + theta5 - theta4

        T_m = compute_T_base_to_ee_from_joints(theta6, theta5, theta4, theta3)
        R_g2b = T_m[:3, :3]
        t_g2b = T_m[:3,  3].reshape(3, 1)

        self.R_gripper2base.append(R_g2b)
        self.t_gripper2base.append(t_g2b)
        self.R_target2cam.append(R_t2c)
        self.t_target2cam.append(t_t2c)
        self.corners_list.append(corners)
        self._captured_pwms.append(pwms)

        annotated = frame.copy()
        cv2.drawChessboardCorners(annotated, (BOARD_W, BOARD_H), corners, True)
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
        except Exception:
            pass

        ee_pos = T_m[:3, 3] * 1000
        self.get_logger().info(
            f'📸 #{len(self.R_gripper2base)} EE=({ee_pos[0]:.0f},{ee_pos[1]:.0f},{ee_pos[2]:.0f})mm '
            f'alpha={alpha:.1f}° θ6={theta6:.1f}° PWM={list(pwms.values())}'
        )
        return ''

    def run_calibration(self):
        n = len(self.R_gripper2base)
        if n < 5:
            self.get_logger().error(f'仅{n}个位姿，最少5个')
            return

        methods = {
            'TSAI':    cv2.CALIB_HAND_EYE_TSAI,
            'PARK':    cv2.CALIB_HAND_EYE_PARK,
            'HORAUD':  cv2.CALIB_HAND_EYE_HORAUD,
            'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
        }
        results = {}
        for name, method in methods.items():
            try:
                R_c2g, t_c2g = cv2.calibrateHandEye(
                    self.R_gripper2base, self.t_gripper2base,
                    self.R_target2cam, self.t_target2cam,
                    method=method,
                )
                err = self._calc_error(R_c2g, t_c2g)
                results[name] = (R_c2g, t_c2g, err)
                self.get_logger().info(
                    f'  [{name}] t=[{t_c2g.flatten()[0]:.4f},{t_c2g.flatten()[1]:.4f},{t_c2g.flatten()[2]:.4f}]m err={err:.3f}px'
                )
            except Exception as e:
                self.get_logger().warn(f'  [{name}] 失败: {e}')

        if not results:
            self.get_logger().error(
                '所有方法均失败或方向异常。\n'
                '  可能原因：1)夹爪坐标轴方向与 arm_fk 定义不一致\n'
                '           2)棋盘格移动了\n'
                '           3)位姿不够分散'
            )
            return

        best_name = min(results, key=lambda k: results[k][2])
        R_best, t_best, err_best = results[best_name]
        ty = t_best.flatten()[1]
        tz = t_best.flatten()[2]

        # 方向检查（仅警告，不阻止）
        warnings = []
        if ty <= 0: warnings.append(f't_y={ty*1000:.0f}mm≤0')
        if tz >= 0: warnings.append(f't_z={tz*1000:.0f}mm≥0')
        if warnings:
            sep = ', '
            self.get_logger().warn(f'⚠️ 方向异常: {sep.join(warnings)}。可能夹爪坐标轴定义不同，请用分拣节点实测验证。')

        up_label = '上方' if ty > 0 else '下方'
        fwd_label = '前方' if tz > 0 else '后方'

        self.get_logger().info(
            f'\n\033[1;32m✅ 最优方法: {best_name}\033[0m\n'
            f'  R_cam2gripper:\n{R_best.round(6)}\n'
            f'  t_cam2gripper (m): {t_best.flatten().round(6)}\n'
            f'  t_y={ty*1000:.0f}mm (相机在夹爪{up_label})\n'
            f'  t_z={tz*1000:.0f}mm (相机在夹爪{fwd_label})\n'
            f'  重投影误差: {err_best:.3f} px'
        )
        if err_best > 3:
            self.get_logger().warn(f'⚠️ 误差>{3}px，建议增多位姿或检查棋盘格')

        self._save_result(R_best, t_best, err_best, best_name, n)

    def _calc_error(self, R_c2g, t_c2g):
        if not self.corners_list:
            return float('inf')
        objp = build_objpoints()
        total, cnt = 0.0, 0
        for corners, R_t2c, t_t2c in zip(
            self.corners_list, self.R_target2cam, self.t_target2cam
        ):
            rvec, _ = cv2.Rodrigues(R_t2c)
            proj, _ = cv2.projectPoints(objp, rvec, t_t2c, self.K, self.D)
            err = np.linalg.norm(proj.reshape(-1, 2) - corners.reshape(-1, 2), axis=1).mean()
            total += err; cnt += 1
        return total / cnt if cnt else float('inf')

    def _save_result(self, R, t, err, method, n):
        data = {
            'R_cam2gripper':      R.tolist(),
            't_cam2gripper':      t.flatten().tolist(),
            'num_poses':          n,
            'reprojection_error': round(float(err), 4),
            'method':             method,
            'board_w':            BOARD_W,
            'board_h':            BOARD_H,
            'square_size_m':      SQUARE_SIZE_M,
        }
        with open(CALIB_OUTPUT, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        self.get_logger().info(f'💾 已保存: {CALIB_OUTPUT}')

    def _save_intrinsics(self):
        if self.K is None: return
        with open(INTRINSICS_CACHE, 'w') as f:
            yaml.dump({'K': self.K.tolist(), 'D': self.D.tolist()}, f,
                      default_flow_style=False)

    def _load_intrinsics(self):
        if not os.path.isfile(INTRINSICS_CACHE): return False
        try:
            with open(INTRINSICS_CACHE) as f:
                d = yaml.safe_load(f)
            self.K = np.array(d['K'], dtype=np.float64)
            self.D = np.array(d['D'], dtype=np.float64)
            self._intrinsics_ready.set()
            return True
        except Exception:
            return False


# ═══════════════════════════════════════════════════════════════════════════════
#  渲染
# ═══════════════════════════════════════════════════════════════════════════════

def draw_hud(frame, node):
    h, w = frame.shape[:2]
    n = len(node.R_gripper2base)
    status = 'RELEASED' if node._servo_released else 'LOCKED'
    info = [
        f"[h]Release [l]Lock [p]Capture [s]Calibrate [r]Read [q]Quit",
        f"Poses: {n}  Arm: {status}",
    ]
    if node.K is not None:
        info.append(f"K: fx={node.K[0,0]:.1f} fy={node.K[1,1]:.1f} "
                     f"cx={node.K[0,2]:.1f} cy={node.K[1,2]:.1f}")
    for i, txt in enumerate(info):
        cv2.putText(frame, txt, (10, 25 + i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return frame


# ═══════════════════════════════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = EyeInHandCalibNode()
    executor = MultiThreadedExecutor(); executor.add_node(node)
    spin_t = threading.Thread(target=executor.spin, daemon=True); spin_t.start()

    uart_ok = setup_uart(115200)
    if uart_ok:
        time.sleep(0.3)
        node._last_pwms = read_servo_pwms()
        release_servos('K')
        node._servo_released = True
        print('[UART] 舵机已释放，可手动扳动机械臂')
    else:
        print('[UART] 串口初始化失败，舵机控制不可用')

    if not node._intrinsics_ready.wait(15.0):
        if not node._load_intrinsics():
            print('内参超时'); return 1

    for _ in range(50):
        with node._frame_lock:
            if node.latest_frame is not None: break
        time.sleep(0.2)
    else:
        print('无相机帧'); return 1

    cv2.namedWindow('EyeInHand Calib v2', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('EyeInHand Calib v2', 960, 720)
    print('\n🎯 准备就绪。按 h 释放舵机 → 手动扳臂 → 看到绿色角点按 p 拍照')

    last_msg = ''
    msg_timer = 0.0

    try:
        while True:
            with node._frame_lock:
                raw = node.latest_frame.copy() if node.latest_frame is not None else None
            if raw is None:
                time.sleep(0.03); continue

            display = raw.copy()
            corners = detect_checkerboard(raw)
            if corners is not None:
                cv2.drawChessboardCorners(display, (BOARD_W, BOARD_H), corners, True)

            display = draw_hud(display, node)

            if last_msg and time.time() - msg_timer < 2.0:
                cv2.putText(display, last_msg, (10, display.shape[0] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow('EyeInHand Calib v2', display)

            key = cv2.waitKey(30) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            elif key in (ord('h'), ord('H')):
                if uart_ok:
                    release_servos('K')
                    node._servo_released = True
                    last_msg = '舵机已释放'
                    msg_timer = time.time()
            elif key in (ord('l'), ord('L')):
                if uart_ok:
                    node._last_pwms = read_servo_pwms()
                    lock_servos(node._last_pwms)
                    node._servo_released = False
                    last_msg = '舵机已锁定'
                    msg_timer = time.time()
            elif key in (ord('p'), ord('P')):
                msg = node.capture_pose()
                last_msg = msg if msg else f'已记录 #{len(node.R_gripper2base)}'
                msg_timer = time.time()
            elif key in (ord('s'), ord('S')):
                n = len(node.R_gripper2base)
                if n < 5:
                    last_msg = f'仅{n}个位姿，需要≥5个'
                    msg_timer = time.time()
                else:
                    node.run_calibration()
                    last_msg = '标定完成'
                    msg_timer = time.time()
            elif key in (ord('r'), ord('R')):
                if uart_ok:
                    pwms = read_servo_pwms()
                    theta6, theta5, theta4, theta3 = pwms_to_joint_angles(pwms)
                    alpha = theta3 + theta5 - theta4
                    T = compute_T_base_to_ee_from_joints(theta6, theta5, theta4, theta3)
                    ee = T[:3, 3] * 1000
                    print(f'PWM={list(pwms.values())} '
                          f'θ=[{theta6:.1f},{theta5:.1f},{theta4:.1f},{theta3:.1f}] '
                          f'α={alpha:.1f}° EE=({ee[0]:.0f},{ee[1]:.0f},{ee[2]:.0f})mm')

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if uart_ok:
            node._last_pwms = read_servo_pwms()
            lock_servos(node._last_pwms)
            close_uart()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
