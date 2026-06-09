#!/usr/bin/env python3
"""
depth_distance_node.py — 交互式 3D 测距节点（深度相机核心演示）

功能:
  1. 启动时自动释放机械臂舵机阻力，可手动扳动机械臂调整姿态
  2. 订阅 /aurora/rgb/image_raw + /aurora/depth/image_raw
  3. OpenCV 窗口实时显示 RGB 图像 + 深度热力图叠加
  4. 鼠标悬停 → 实时显示该像素到相机的距离
  5. 鼠标点击 → 锁定测距点，显示 3D 坐标（相机系 + 基座系）
  6. 发布带标注的结果图像到 /depth_display/image_result

========== 键盘控制 ==========
  h / H      切换热力图叠加模式（关闭/半透明/全热力图）
  c / C      清除锁定点
  r / R      重新读取当前舵机位置并打印
  l / L      切换舵机锁定/释放（释放后可手动扳动机械臂）
  q / Q / Esc  退出（自动恢复舵机阻力）

========== 话题 ==========
  订阅:
    /aurora/rgb/image_raw    (sensor_msgs/Image, bgr8)
    /aurora/depth/image_raw  (sensor_msgs/Image, mono16)
  发布:
    /depth_display/image_result  (sensor_msgs/Image, bgr8)
    
========== 操作流程 =========
1.节点启动 → 自动释放舵机 → 显示器弹出相机画面
2.手动扳动机械臂到不同姿态（测不同物体/位置）
3.鼠标悬停 → 实时看距离；鼠标左键点击 → 锁定测距点
4.按 h 切换热力图模式；按 c 清除锁定点
5.想再次调整姿态 → 按 l 释放舵机 → 手动扳动 → 按 l 锁定
6.按 r 随时查看当前关节角度
7.按 q 退出 → 自动恢复阻力

========== 用法 ==========
  ros2 run my_srv depth_distance_node.py
"""

import os
import sys
import re
import time
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ── 导入 depth_utils ──
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from depth_utils import DepthUtils

# ── 导入 z_uart（串口控制舵机）──
_OPENCV_DIR = os.path.expanduser('~/OpenCV')
if _OPENCV_DIR not in sys.path:
    sys.path.insert(0, _OPENCV_DIR)

import z_uart
from z_uart import uart_send_str, setup_uart, close_uart


# ═══════════════════════════════════════════════════════════════════════════════
#  舵机控制辅助函数（参考 arm_teach.py）
# ═══════════════════════════════════════════════════════════════════════════════

SERVO_COUNT = 6          # 舵机编号 000~005
READ_TIMEOUT = 1.5       # 等待舵机反馈超时（秒）
LOCK_TIME = 200          # 恢复阻力运动时间（ms），避免位置突变
LOCK_PWM_DEFAULT = 1500  # 无法读取位置时的默认 PWM


def _sid(idx: int) -> str:
    """返回 3 位补零的舵机编号字符串，例如 2 -> '002'"""
    return f"{idx:03d}"


def _wait_uart_response(timeout: float = READ_TIMEOUT) -> str | None:
    """等待 z_uart 接收线程完成一帧数据。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if z_uart.uart_get_ok:
            data = z_uart.uart_receive_buf
            z_uart.uart_receive_buf = ''
            z_uart.uart_get_ok = 0
            return data
        time.sleep(0.005)
    return None


def _parse_pwm(response: str) -> int | None:
    """从响应中解析 PWM 值，跳过命令回显帧。"""
    m = re.search(r'#\d{3}P(\d+)!', response)
    return int(m.group(1)) if m else None


def _wait_prad_response(timeout: float = READ_TIMEOUT) -> str | None:
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


def release_servos(mode: str = 'K') -> None:
    """释放 000~005 舵机阻力，使其可被手动扳动。
    mode='K' → 小阻力  mode='M' → 大阻力
    """
    cmd_suffix = f"PUL{mode}"
    print(f"  释放舵机阻力（{'小阻力' if mode == 'K' else '大阻力'}）...")
    for i in range(SERVO_COUNT):
        sid = _sid(i)
        cmd = f"#{sid}{cmd_suffix}!"
        uart_send_str(cmd)
        resp = _wait_uart_response(timeout=0.8)
        if resp:
            print(f"    舵机 {sid} <- {resp.strip()}")
        else:
            print(f"    舵机 {sid} 无响应（已发送 {cmd}）")
    print("  阻力已释放，现在可以手动扳动机械臂。\n")


def read_servo_positions() -> dict[str, int | None]:
    """依次读取 000~005 舵机当前位置，收到反馈后才请求下一个。"""
    positions: dict[str, int | None] = {}
    for i in range(SERVO_COUNT):
        sid = _sid(i)
        uart_send_str(f"#{sid}PRAD!")
        resp = _wait_prad_response()
        if resp:
            pwm = _parse_pwm(resp)
            positions[sid] = pwm
        else:
            positions[sid] = None
    return positions


def print_servo_positions(positions: dict[str, int | None]) -> None:
    """打印舵机位置表。"""
    parts = []
    for sid, pwm in positions.items():
        parts.append(f"{sid}:{pwm if pwm is not None else 'N/A':>5}")
    print("  ".join(parts))


def lock_servos(positions: dict[str, int | None] = None) -> None:
    """发送位置指令恢复舵机阻力。
    Args:
        positions: 舵机位置字典，None 时自动读取当前位置。
    """
    if positions is None:
        positions = read_servo_positions()
    print("  正在恢复舵机阻力...")
    for i in range(SERVO_COUNT):
        sid = _sid(i)
        pwm = positions.get(sid) or LOCK_PWM_DEFAULT
        cmd = f"#{sid}P{pwm}T{LOCK_TIME}!"
        uart_send_str(cmd)
        time.sleep(0.05)
        print(f"    舵机 {sid} 已锁定（PWM={pwm}）")
    print("  所有舵机已恢复阻力。\n")


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS2 节点
# ═══════════════════════════════════════════════════════════════════════════════

class DepthDistanceNode(Node):
    """交互式 3D 测距节点（含舵机控制）"""

    def __init__(self):
        super().__init__('depth_distance_node')

        # ── DepthUtils ──
        self.du = DepthUtils(self)

        # ── 帧缓冲 ──
        self.latest_rgb = None
        self._rgb_lock = threading.Lock()
        self.bridge = CvBridge()

        # ── 测距状态 ──
        self._hover_u = -1          # 鼠标悬停像素
        self._hover_v = -1
        self._hover_depth_mm = 0
        self._locked_points = []    # [(u, v, depth_mm, cam_xyz)]

        # ── 显示模式 ──
        self._heatmap_mode = 0      # 0=关闭 1=半透明 2=全热力图

        # ── 外参 ──
        self._has_extrinsics = False

        # ── 舵机状态 ──
        self._servo_released = True  # 初始释放
        self._servo_positions: dict[str, int | None] = {}

        # ── 订阅 RGB ──
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            Image, '/aurora/rgb/image_raw', self._rgb_cb, qos,
        )

        # ── 发布结果图像 ──
        self.result_pub = self.create_publisher(
            Image, '/depth_display/image_result', 10,
        )

        self.get_logger().info(
            '\033[1;36m[DepthDistance]\033[0m 节点已启动\n'
            '  鼠标悬停 → 实时显示距离\n'
            '  鼠标点击 → 锁定测距点\n'
            '  h=热力图  c=清除  r=读位置  l=释放/锁定  q=退出'
        )

    def _rgb_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._rgb_lock:
                self.latest_rgb = frame
        except Exception:
            pass

    # ── 舵机操作 ──
    def arm_release(self) -> bool:
        """释放舵机阻力（可手动扳动）。"""
        try:
            release_servos('K')
            self._servo_released = True
            return True
        except Exception as e:
            print(f"  ⚠ 释放舵机失败: {e}")
            return False

    def arm_lock(self) -> bool:
        """恢复舵机阻力。"""
        try:
            lock_servos(self._servo_positions)
            self._servo_released = False
            return True
        except Exception as e:
            print(f"  ⚠ 锁定舵机失败: {e}")
            return False

    def arm_read_positions(self) -> dict:
        """读取舵机当前位置。"""
        try:
            self._servo_positions = read_servo_positions()
            print_servo_positions(self._servo_positions)
        except Exception as e:
            print(f"  ⚠ 读取舵机位置失败: {e}")
        return self._servo_positions


# ═══════════════════════════════════════════════════════════════════════════════
#  鼠标回调
# ═══════════════════════════════════════════════════════════════════════════════

_mouse_u = -1
_mouse_v = -1
_mouse_lock = threading.Lock()


def _mouse_callback(event, x, y, flags, param):
    global _mouse_u, _mouse_v
    with _mouse_lock:
        _mouse_u, _mouse_v = x, y
    # 左键点击 → 锁定
    if event == cv2.EVENT_LBUTTONDOWN:
        node = param
        with node._rgb_lock:
            rgb = node.latest_rgb
        if rgb is not None:
            depth_mm = node.du.get_depth_at(x, y)
            if depth_mm is not None:
                try:
                    cam_xyz = node.du.pixel_to_3d(x, y, depth_mm)
                except RuntimeError:
                    return
                node._locked_points.append((x, y, depth_mm, cam_xyz))
                print(f'  📍 锁定点 #{len(node._locked_points)}: '
                      f'pixel=({x},{y})  depth={depth_mm}mm  '
                      f'cam_xyz=({cam_xyz[0]:.3f},{cam_xyz[1]:.3f},{cam_xyz[2]:.3f})m')
                # 如果有外参，也显示基座坐标
                if node._has_extrinsics:
                    # 基座坐标需要当前臂位姿的 T_gripper2base
                    # 这里显示末端执行器坐标
                    p_ee = node.du.transform_cam_to_gripper(cam_xyz)
                    print(f'           gripper_xyz=({p_ee[0]:.3f},{p_ee[1]:.3f},{p_ee[2]:.3f})m')


# ═══════════════════════════════════════════════════════════════════════════════
#  渲染
# ═══════════════════════════════════════════════════════════════════════════════

def draw_hud(display: np.ndarray, node: DepthDistanceNode) -> np.ndarray:
    """绘制 HUD 和测距信息。"""
    h, w = display.shape[:2]

    # 半透明顶栏
    overlay = display.copy()
    cv2.rectangle(overlay, (0, 0), (w, 95), (20, 20, 20), -1)
    display = cv2.addWeighted(overlay, 0.5, display, 0.5, 0)

    # 标题
    cv2.putText(display, "Depth Distance Measure — Hover to measure, Click to lock",
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    # 键盘提示
    status = "Released" if node._servo_released else "Locked"
    cv2.putText(display, f"[h]Heatmap  [c]Clear  [r]Read  [l]Lock/Release  [q]Quit   Arm: {status}",
                (10, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)

    # 悬停信息
    hu, hv = node._hover_u, node._hover_v
    if 0 <= hu < w and 0 <= hv < h:
        d_mm = node._hover_depth_mm
        if d_mm > 0:
            d_cm = d_mm / 10.0
            d_m = d_mm / 1000.0
            cv2.putText(display, f"Hover: ({hu},{hv}) | {d_cm:.1f}cm ({d_m:.3f}m)",
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        else:
            cv2.putText(display, f"Hover: ({hu},{hv}) | no depth",
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 100), 1)

    # 锁定点信息
    if node._locked_points:
        cv2.putText(display, f"Locked: {len(node._locked_points)} points  [c] to clear",
                    (10, 87), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    # 热力图模式
    modes = ['Off', 'Semi-transparent', 'Full']
    cv2.putText(display, f"Mode: {modes[node._heatmap_mode]}",
                (w - 180, 87), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    return display


def draw_locked_points(display: np.ndarray, node: DepthDistanceNode):
    """在图像上绘制锁定点和十字准线。"""
    for idx, (u, v, d_mm, cam_xyz) in enumerate(node._locked_points):
        # 十字准线
        cv2.drawMarker(display, (u, v), (0, 255, 0),
                       cv2.MARKER_CROSS, 20, 2)
        # 标签
        d_cm = d_mm / 10.0
        label = f"#{idx+1}: {d_cm:.1f}cm"
        cv2.putText(display, label, (u + 15, v - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        # 小圆点
        cv2.circle(display, (u, v), 4, (0, 255, 0), -1)

    # 悬停十字
    hu, hv = node._hover_u, node._hover_v
    if 0 <= hu < display.shape[1] and 0 <= hv < display.shape[0]:
        if node._hover_depth_mm > 0:
            cv2.drawMarker(display, (hu, hv), (0, 255, 255),
                           cv2.MARKER_TILTED_CROSS, 12, 1)


# ═══════════════════════════════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = DepthDistanceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("\n" + "=" * 55)
    print("\033[1;36m  深度相机 3D 测距演示（支持手动扳动机械臂）\033[0m")
    print("=" * 55)

    # ── 初始化串口 ──
    print("[UART] 正在初始化串口...")
    if not setup_uart(115200):
        print("\033[1;33m[警告]\033[0m 串口初始化失败，舵机控制不可用。")
        print("  深度测距功能仍可正常使用。")
        uart_ok = False
    else:
        uart_ok = True
        # 初始读取舵机位置
        print("[UART] 读取当前舵机位置...")
        node.arm_read_positions()
        # 释放舵机阻力，允许手动扳动
        print("[UART] 释放舵机阻力（可手动扳动机械臂）...")
        node.arm_release()
        print("  💡 提示: 按 [l] 切换锁定/释放, 按 [r] 重新读取位置\n")

    # 等待深度工具就绪
    print("[等待] 相机内参...")
    if not node.du.wait_for_intrinsics(timeout=15.0):
        print("\033[1;31m[错误]\033[0m 相机内参未就绪，请确保 Aurora 930 驱动运行中。")
        if uart_ok:
            node.arm_lock()
            close_uart()
        return 1

    print("[等待] 外参...")
    node._has_extrinsics = node.du.load_hand_eye_calib()

    print("[等待] 相机帧...")
    for _ in range(50):
        with node._rgb_lock:
            if node.latest_rgb is not None:
                break
        time.sleep(0.2)
    else:
        print("\033[1;31m[错误]\033[0m 无法获取相机图像")
        if uart_ok:
            node.arm_lock()
            close_uart()
        return 1

    print("[就绪] 鼠标移动查看距离，左键点击锁定测距点\n")

    cv2.namedWindow('Depth Distance Measure', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Depth Distance Measure', 960, 720)
    cv2.setMouseCallback('Depth Distance Measure', _mouse_callback, node)

    try:
        while True:
            # 获取帧
            with node._rgb_lock:
                rgb = node.latest_rgb.copy() if node.latest_rgb is not None else None

            if rgb is None:
                time.sleep(0.03)
                continue

            # 更新悬停深度
            global _mouse_u, _mouse_v
            with _mouse_lock:
                hu, hv = _mouse_u, _mouse_v
            node._hover_u = hu
            node._hover_v = hv
            if 0 <= hu < rgb.shape[1] and 0 <= hv < rgb.shape[0]:
                node._hover_depth_mm = node.du.get_depth_at(hu, hv) or 0
            else:
                node._hover_depth_mm = 0

            # 热力图叠加
            if node._heatmap_mode > 0:
                depth_img = node.du.latest_depth
                if depth_img is not None:
                    heatmap = DepthUtils.depth_to_heatmap(depth_img)
                    if depth_img.shape[:2] == rgb.shape[:2]:
                        if node._heatmap_mode == 2:
                            display = heatmap
                        else:
                            display = cv2.addWeighted(rgb, 0.4, heatmap, 0.6, 0)
                    else:
                        display = rgb.copy()
                else:
                    display = rgb.copy()
            else:
                display = rgb.copy()

            # 绘制 HUD + 锁定点
            display = draw_hud(display, node)
            draw_locked_points(display, node)

            cv2.imshow('Depth Distance Measure', display)

            # 发布结果图像
            try:
                node.result_pub.publish(
                    node.bridge.cv2_to_imgmsg(display, 'bgr8'))
            except Exception:
                pass

            # 键盘
            key = cv2.waitKey(30) & 0xFF

            if key in (ord('q'), ord('Q'), 27):
                break
            elif key in (ord('h'), ord('H')):
                node._heatmap_mode = (node._heatmap_mode + 1) % 3
                modes = ['Hotmap Off', 'Semi-transparent', 'Full Heatmap']
                print(f'  热力图模式: {modes[node._heatmap_mode]}')
            elif key in (ord('c'), ord('C')):
                node._locked_points.clear()
                print('  🧹 已清除所有锁定点')
            elif key in (ord('r'), ord('R')):
                # 读取舵机当前位置
                if uart_ok:
                    print('  📡 读取舵机当前位置:')
                    node.arm_read_positions()
                else:
                    print('  ⚠ 串口不可用')
            elif key in (ord('l'), ord('L')):
                # 切换锁定/释放
                if not uart_ok:
                    print('  ⚠ 串口不可用')
                elif node._servo_released:
                    print('  🔒 恢复舵机阻力...')
                    node.arm_lock()
                else:
                    print('  🔓 释放舵机阻力（可手动扳动）...')
                    node._servo_positions = read_servo_positions()
                    node.arm_release()

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        # 退出前恢复舵机阻力
        if uart_ok:
            print("\n退出前恢复舵机阻力...")
            try:
                node._servo_positions = read_servo_positions()
                lock_servos(node._servo_positions)
            except Exception:
                pass
            close_uart()
        node.destroy_node()
        rclpy.shutdown()
        print("\n退出。")


if __name__ == '__main__':
    sys.exit(main() or 0)
