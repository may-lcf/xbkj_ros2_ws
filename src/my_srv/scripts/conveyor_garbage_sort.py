#!/usr/bin/env python3
"""
conveyor_garbage_sort.py — 传送带垃圾分类节点

工作流:
  1. 启动后机械臂移到观察位置
  2. 等待指令（CLI输入或语音话题）
  3. 启动传送带，YOLO检测垃圾
  4. 垃圾到达画面中心 → 停止传送带
  5. 等待稳定3.5秒 → 深度定位 → 抓取 → 移到对应垃圾桶 → 投放
  6. 回观察位置，等待下一次指令

前提: yolo_detect_node.py 必须在运行（提供 /yolo/detections 话题）

用法:
  python3 ~/ros2_ws/src/my_srv/scripts/conveyor_garbage_sort.py

指令:
  CLI输入: 开始垃圾分类
  语音话题: /voice_command (String) 内容为 "开始垃圾分类"
"""

import os, sys, json, time, threading, subprocess, re
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

# ── 路径 ──
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
for p in (_SCRIPT_DIR, os.path.expanduser('~/ros2_ws/src/my_srv/scripts'),
          os.path.expanduser('~/OpenCV')):
    if p not in sys.path:
        sys.path.insert(0, p)

import arm_fk
import z_uart
from z_uart import uart_send_str, setup_uart, close_uart
from z_move import kinematics_move

# ── 运动学参数：垫高7cm ──

# ── 位置参数 ──
OBS_X, OBS_Y, OBS_Z = 0, 130, 120
ALPHA_OBS = -78      # 观察位置末端角度
ALPHA_GRASP = -75    # 抓取时末端角度
OFFSET_X = 0   # 正=向左, 负=向右
OFFSET_Z = 0   # 正=向上, 负=向下
OFFSET_Y = -5   # 正=向后, 负=向前


# ── 垃圾分类配置 ──
GARBAGE_CONFIG = {
    '可回收垃圾': {
        'labels': ['bottle', 'newspaper'],
        'bin_pwm': '#000P2319T2000!#001P1219T2000!#002P1864T2000!#003P1005T2000!#004P1538T2000!',
    },
    '有害垃圾': {
        'labels': ['battery'],
        'bin_pwm': '#000P2164T2000!#001P1336T2000!#002P2035T2000!#003P1045T2000!#004P1500T2000!',
    },
    '厨余垃圾': {
        'labels': ['banana_peel'],
        'bin_pwm': '#000P826T2000!#001P1116T2000!#002P1654T2000!#003P0832T2000!#004P1414T2000!',
    },
    '其他垃圾': {
        'labels': ['toilet_paper'],
        'bin_pwm': '#000P975T2000!#001P1127T2000!#002P1675T2000!#003P0853T2000!#004P1541T2000!',
    },
}

# ── YOLO类别 → 垃圾分类 + 中文名 ──
LABEL_TO_CATEGORY = {}
for cat, cfg in GARBAGE_CONFIG.items():
    for label in cfg['labels']:
        LABEL_TO_CATEGORY[label] = cat
LABEL_CN = {
    'banana_peel': '香蕉皮', 'battery': '电池', 'bottle': '塑料瓶',
    'newspaper': '报纸', 'toilet_paper': '卫生纸',
}

# ── 连续检测确认帧数 ──
CONFIRM_FRAMES = 1


class ConveyorGarbageSort(Node):

    def __init__(self):
        super().__init__('conveyor_garbage_sort')

        self.active = False
        self.latest_detections = []
        self._det_lock = threading.Lock()
        self.latest_depth = None
        self.fx = self.fy = self.cx_c = self.cy_c = None

        # YOLO 检测
        self.create_subscription(String, '/yolo/detections', self._det_cb, 10)

        # 深度图和内参
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/aurora/depth/image_raw', self._depth_cb, qos)
        self.create_subscription(CameraInfo, '/aurora/rgb/camera_info', self._info_cb, qos)

        # 语音指令
        self.create_subscription(String, '/voice_command', self._voice_cb, 10)

        # 发布状态
        self.pub_status = self.create_publisher(String, '/conveyor_sort/status', 10)
        self.pub_speak = self.create_publisher(String, '/voice_speak', 10)

        self.get_logger().info('传送带垃圾分类节点已启动')

    # ══════════════════════════════════════════════════════════════
    #  回调
    # ══════════════════════════════════════════════════════════════

    def _det_cb(self, msg):
        try:
            with self._det_lock:
                self.latest_detections = json.loads(msg.data)
        except Exception:
            pass

    def _depth_cb(self, msg):
        try:
            self.latest_depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        except Exception:
            pass

    def _info_cb(self, msg):
        if self.fx is not None:
            return
        K = np.array(msg.k).reshape(3, 3)
        self.fx, self.fy = K[0, 0], K[1, 1]
        self.cx_c, self.cy_c = K[0, 2], K[1, 2]

    def _voice_cb(self, msg):
        if '开始垃圾分类' in msg.data:
            self.get_logger().info('收到语音指令: 开始垃圾分类')
            self._start_sort()

    # ══════════════════════════════════════════════════════════════
    #  工具方法
    # ══════════════════════════════════════════════════════════════

    def _speak(self, text):
        m = String()
        m.data = text
        self.pub_speak.publish(m)

    def _publish_status(self, status, message):
        msg = String()
        msg.data = json.dumps({'status': status, 'message': message}, ensure_ascii=False)
        self.pub_status.publish(msg)
        self.get_logger().info(f'[{status}] {message}')

    def _conveyor_start(self):
        subprocess.Popen(
            ['python3', os.path.join(_SCRIPT_DIR, 'motor6_control.py'), 'forward', '1512', '9999'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self.get_logger().info('传送带启动')

    def _conveyor_stop(self):
        subprocess.Popen(
            ['python3', os.path.join(_SCRIPT_DIR, 'motor6_control.py'), 'stop'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self.get_logger().info('传送带停止')

    def _find_target_by_category(self, category):
        """在视野中查找指定分类的垃圾（不要求在画面中心）"""
        labels = GARBAGE_CONFIG.get(category, {}).get('labels', [])
        with self._det_lock:
            dets = list(self.latest_detections)
        for d in dets:
            if d.get('shape', '') in labels:
                return d
        return None

    def _check_center_target(self):
        """检查YOLO检测结果中，是否有垃圾出现在画面中心区域"""
        with self._det_lock:
            dets = list(self.latest_detections)

        h, w = None, None
        if self.latest_depth is not None:
            h, w = self.latest_depth.shape[:2]
        elif self.cx_c is not None:
            w, h = int(self.cx_c * 2), int(self.cy_c * 2)

        for d in dets:
            label = d.get('shape', '')
            if label not in LABEL_TO_CATEGORY:
                continue
            if h and w:
                px, py = d['pixel']
                mx, my = w * 0.25, h * 0.25
                if not (mx < px < w - mx and my < py < h - my):
                    continue
            return d
        return None

    def _compute_world_xyz(self, det):
        """像素+深度 → 基座3D坐标（手眼标定法，与 garbage_sort_node.py 一致）"""
        if self.fx is None or self.latest_depth is None:
            return None
        px, py = det['pixel']
        depth_mm = det.get('depth_mm', 0)
        if depth_mm <= 100:
            dimg = self.latest_depth
            for r in range(0, 15):
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        ux, uy = px + dx, py + dy
                        if 0 <= uy < dimg.shape[0] and 0 <= ux < dimg.shape[1]:
                            d = int(dimg[uy, ux])
                            if 100 < d < 4000:
                                depth_mm = d
                                break
                    if depth_mm > 100:
                        break
                if depth_mm > 100:
                    break
        if depth_mm <= 100:
            return None
        z = depth_mm / 1000.0
        x = (px - self.cx_c) * z / self.fx
        y = (py - self.cy_c) * z / self.fy
        p_cam = np.array([x, y, z])
        try:
            pwms = self._read_joint_pwms()
            if pwms is None:
                return None
            th = arm_fk.pwms_to_angles(*pwms)
            T_g2b_mm = arm_fk.compute_T_base_to_ee_from_angles(*th)
            T_g2b = arm_fk.T_mm_to_m(T_g2b_mm)
            # 调试：打印 T_g2b
            t = T_g2b[:3, 3]
            self.get_logger().info(
                f'[DEBUG] T_g2b t=({t[0]*1000:.0f},{t[1]*1000:.0f},{t[2]*1000:.0f})mm '
                f'pwms={pwms} angles=({th[0]:.1f},{th[1]:.1f},{th[2]:.1f},{th[3]:.1f})'
            )
            import yaml
            calib_dir = os.path.expanduser('~/ros2_ws/src/my_srv/config')
            with open(os.path.join(calib_dir, 'hand_eye_calib.yaml')) as f:
                data = yaml.safe_load(f)
            T_c2g = np.eye(4)
            T_c2g[:3, :3] = np.array(data['R_cam2gripper'])
            T_c2g[:3, 3] = np.array(data['t_cam2gripper']).flatten()
            p = np.append(p_cam, 1.0)
            p_base = T_g2b @ T_c2g @ p
            result = tuple(float(v) * 1000 for v in p_base[:3])
            self.get_logger().info(
                f'[DEBUG] cam=({p_cam[0]*1000:.0f},{p_cam[1]*1000:.0f},{p_cam[2]*1000:.0f})mm '
                f'base=({result[0]:.0f},{result[1]:.0f},{result[2]:.0f})mm'
            )
            return result
        except Exception as e:
            self.get_logger().warn(f'坐标转换失败: {e}')
            return None

    def _read_pwm(self, idx, timeout=0.6):
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

    # ══════════════════════════════════════════════════════════════
    #  主流程
    # ══════════════════════════════════════════════════════════════

    def _start_sort(self):
        if self.active:
            self._publish_status('busy', '正在执行中')
            return
        threading.Thread(target=self._execute_loop, daemon=True).start()

    def _execute_loop(self):
        """一次完整的垃圾分类循环：启动传送带 → 检测 → 抓取分拣 → 回观察位"""
        self.active = True
        try:
            if not setup_uart(115200):
                self._publish_status('error', '串口初始化失败')
                return

            # ── 1. 观察位置就位 ──
            self._publish_status('ready', '移到观察位置...')
            kinematics_move(OBS_X, OBS_Y, OBS_Z, 1500, alpha_hint=ALPHA_OBS)
            time.sleep(1.5)

            # ── 2. 启动传送带 ──
            self._conveyor_start()
            self._publish_status('conveyor', '传送带启动，等待垃圾...')

            # ── 3. 等待垃圾到达画面中心（多帧确认）──
            det = None
            confirm = 0
            for _ in range(300):  # 最多等30秒
                time.sleep(0.1)
                found = self._check_center_target()
                if found:
                    confirm += 1
                    if confirm >= CONFIRM_FRAMES:
                        det = found
                        break
                else:
                    confirm = 0

            # ── 4. 停止传送带 ──
            self._conveyor_stop()
            if det is None:
                self._speak('未检测到垃圾')
                self._publish_status('not_found', '未检测到垃圾')
                kinematics_move(OBS_X, OBS_Y, OBS_Z, 1500, alpha_hint=ALPHA_OBS)
                return

            label = det.get('shape', '')
            category = LABEL_TO_CATEGORY.get(label, '')
            label_cn = LABEL_CN.get(label, label)
            self._publish_status('found', f'检测到 {label_cn}（{category}）')

            # ── 5. 等待传送带稳定后重新定位 ──
            time.sleep(3.5)

            # ── 6. 重新检测目标（用最新数据，而非停传送带前的旧数据）──
            det = self._find_target_by_category(category)
            if det is None:
                self._speak('目标丢失')
                self._publish_status('lost', '稳定后未重新检测到目标')
                kinematics_move(OBS_X, OBS_Y, OBS_Z, 1500, alpha_hint=ALPHA_OBS)
                return
            label = det.get('shape', '')
            category = LABEL_TO_CATEGORY.get(label, '')
            label_cn = LABEL_CN.get(label, label)
            self.get_logger().info(f'重新检测到: {label_cn} ({category})')

            # ── 7. 深度定位 ──
            world_xyz = self._compute_world_xyz(det)
            if world_xyz is None:
                self._speak('无法定位垃圾位置')
                self._publish_status('error', '无法计算3D坐标')
                kinematics_move(OBS_X, OBS_Y, OBS_Z, 1500, alpha_hint=ALPHA_OBS)
                return
            tx, ty, tz = world_xyz[0] + OFFSET_X, world_xyz[1] + OFFSET_Y, world_xyz[2] + OFFSET_Z
            self.get_logger().info(f"[OFFSET] X={OFFSET_X} Y={OFFSET_Y} Z={OFFSET_Z} 原始=({world_xyz[0]:.0f},{world_xyz[1]:.0f},{world_xyz[2]:.0f})mm 偏移后=({tx:.0f},{ty:.0f},{tz:.0f})mm")
            self.get_logger().info(f'世界坐标: ({tx:.0f}, {ty:.0f}, {tz:.0f})mm')

            # ── 7. 飞到目标上方 ──
            self._publish_status('picking', f'移动到 {label_cn} 上方...')
            hover_z = max(int(tz) + 80, 60)
            if not kinematics_move(int(tx), int(ty), hover_z, 1500, alpha_hint=ALPHA_GRASP):
                self._speak('目标超出工作空间')
                self._publish_status('error', 'IK无解')
                return
            time.sleep(1.6)

            # ── 8. 打开夹爪 ──
            for _ in range(3):
                uart_send_str('#005P1000T500!')
                time.sleep(0.3)

            # ── 9. 下降抓取 ──
            grab_z = max(int(tz) - 5, 5)
            kinematics_move(int(tx), int(ty), grab_z, 1200, alpha_hint=ALPHA_GRASP)
            time.sleep(1.3)

            # ── 10. 关闭夹爪 ──
            for _ in range(3):
                uart_send_str('#005P1700T1000!')
                time.sleep(0.4)

            # ── 11. 抬升 ──
            self._publish_status('picking', f'{label_cn} 抓取成功，抬升...')
            kinematics_move(int(tx), int(ty), 150, 1000, alpha_hint=ALPHA_GRASP)
            time.sleep(1)

            # ── 12. 移到对应垃圾桶 ──
            config = GARBAGE_CONFIG.get(category)
            if config:
                self._publish_status('placing', f'移到{category}桶...')
                uart_send_str(config['bin_pwm'])
                time.sleep(2.5)

                # ── 13. 打开夹爪投放 ──
                for _ in range(3):
                    uart_send_str('#005P1000T500!')
                    time.sleep(0.3)

            # ── 14. 回观察位置 ──
            kinematics_move(OBS_X, OBS_Y, OBS_Z, 1500, alpha_hint=ALPHA_OBS)
            time.sleep(1)

            self._publish_status('done', f'{label_cn} 分拣完成（{category}）')

        except Exception as e:
            self._publish_status('error', f'异常: {e}')
            self.get_logger().error(f'分拣异常: {e}')
        finally:
            self.active = False

    def destroy_node(self):
        self._conveyor_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorGarbageSort()

    # CLI 交互线程
    def cli_loop():
        while rclpy.ok():
            try:
                line = sys.stdin.readline().strip()
                if '开始垃圾分类' in line:
                    node.get_logger().info('收到CLI指令: 开始垃圾分类')
                    node._start_sort()
            except (EOFError, KeyboardInterrupt):
                break

    cli_thread = threading.Thread(target=cli_loop, daemon=True)
    cli_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
