#!/usr/bin/env python3
"""
drink_pick_node.py — 语音控制导轨+机械臂抓取饮品节点

上电后机械臂位于观察位置，等待语音指令。
收到饮品名称后：导轨前进扫描 → YOLO检测到目标 → 停止导轨 → 定位抓取 →
回观察位 → 碰撞回零 → 移到放置区 → 放下 → 回观察位。

前提: yolo_detect_node.py 必须在运行（提供 /yolo/detections 话题）

用法:
  python3 ~/ros2_ws/src/my_srv/scripts/drink_pick_node.py

话题:
  订阅: /drink_pick/cmd (JSON), /yolo/detections (JSON)
  发布: /drink_pick/status (JSON), /voice_speak (String)
"""

import os
import sys
import json
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── 路径 ──
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
for p in (_SCRIPT_DIR, os.path.expanduser('~/ros2_ws/src/my_srv/scripts'),
          os.path.expanduser('~/OpenCV')):
    if p not in sys.path:
        sys.path.insert(0, p)

import z_uart
from z_move import kinematics_move, setup_kinematics
from motor_control import StepperMotor

# ── 饮品名 → YOLO 类别 ──
DRINK_MAP = {
    '咖啡': 'coffee', 'coffee': 'coffee',
    '冰红茶': 'iced_black_tea', '红茶': 'iced_black_tea',
    '果冻': 'jelly', 'jelly': 'jelly',
    '牛奶': 'milk', 'milk': 'milk',
    '草莓酸奶': 'strawberry_yogurt', '酸奶': 'strawberry_yogurt',
}

# ── 饮品夹取参数: (下降偏移mm, 推进mm) ──
GRAB_PARAMS = {
    'coffee':           (7, 18),
    'jelly':            (13, 15),
    'milk':             (30, 22),
    'strawberry_yogurt': (23, 16),
}
GRAB_DEFAULT = (35, 16)  # 冰红茶/牛奶/草莓酸奶

# ── 连续检测确认帧数 ──
CONFIRM_FRAMES = 2

# ── 末端角度 ──
ALPHA_OBS = -55    # 观察扫描：俯视桌面
ALPHA_GRASP = -5  # 夹取：近水平侧夹


class DrinkPickNode(Node):

    def __init__(self):
        super().__init__('drink_pick_node')

        self.active = False
        self.latest_detections = []
        self._det_lock = threading.Lock()

        # 深度图
        self.latest_depth = None
        self.fx = self.fy = self.cx_c = self.cy_c = None

        # 导轨电机
        self.motor = StepperMotor('/dev/ch340', addr=0x01)

        # 订阅
        self.create_subscription(String, '/drink_pick/cmd', self._cmd_cb, 10)
        self.create_subscription(String, '/yolo/detections', self._det_cb, 10)

        # 深度图和内参
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image, CameraInfo
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/aurora/depth/image_raw', self._depth_cb, qos)
        self.create_subscription(CameraInfo, '/aurora/rgb/camera_info', self._info_cb, qos)

        # 发布
        self.pub_status = self.create_publisher(String, '/drink_pick/status', 10)
        self.pub_speak = self.create_publisher(String, '/voice_speak', 10)
        self.pub_cmd = self.create_publisher(String, '/drink_pick/cmd', 10)

        # 更新运动学参数: L0=200mm (导轨安装, 135+65)
        import arm_fk
        arm_fk._L0_01mm = 2000
        arm_fk.L0 = 200.0
        setup_kinematics(200, 105, 88, 178)

        # 打印夹取参数
        print('\n=== 夹取参数 (下降mm, 推进mm) ===')
        for name, (drop, push) in GRAB_PARAMS.items():
            print(f'  {name:20s}: drop={drop}mm, push={push}mm')
        print(f'  {"default":20s}: drop={GRAB_DEFAULT[0]}mm, push={GRAB_DEFAULT[1]}mm')
        print()

        self.get_logger().info('饮品抓取节点已启动')

    # ══════════════════════════════════════════════════════════════════
    #  回调
    # ══════════════════════════════════════════════════════════════════

    def _det_cb(self, msg):
        try:
            with self._det_lock:
                self.latest_detections = json.loads(msg.data)
        except Exception:
            pass

    def _depth_cb(self, msg):
        try:
            self.latest_depth = __import__('numpy').frombuffer(
                msg.data, dtype=__import__('numpy').uint16
            ).reshape(msg.height, msg.width)
        except Exception:
            pass

    def _info_cb(self, msg):
        if self.fx is not None:
            return
        import numpy as np
        K = np.array(msg.k).reshape(3, 3)
        self.fx, self.fy = K[0, 0], K[1, 1]
        self.cx_c, self.cy_c = K[0, 2], K[1, 2]

    def _cmd_cb(self, msg):
        try:
            cmd = json.loads(msg.data)
            threading.Thread(target=self._execute_pick, args=(cmd,), daemon=True).start()
        except Exception as e:
            self.get_logger().error(f'指令解析失败: {e}')

    # ══════════════════════════════════════════════════════════════════
    #  工具方法
    # ══════════════════════════════════════════════════════════════════

    def _speak(self, text):
        msg = String()
        msg.data = text
        self.pub_speak.publish(msg)

    def _publish_status(self, status, message):
        msg = String()
        msg.data = json.dumps({'status': status, 'message': message}, ensure_ascii=False)
        self.pub_status.publish(msg)
        self.get_logger().info(f'[{status}] {message}')

    def _check_target(self, drink_yolo, center_only=False):
        """检查最新检测结果中是否有目标饮品，返回第一个匹配

        Args:
            center_only: True时只接受画面中心区域的检测（避免边缘误停）
        """
        with self._det_lock:
            dets = list(self.latest_detections)

        # 获取画面尺寸
        h, w = None, None
        if self.latest_depth is not None:
            h, w = self.latest_depth.shape[:2]
        elif self.fx is not None and self.cx_c is not None:
            w, h = int(self.cx_c * 2), int(self.cy_c * 2)

        for d in dets:
            if d.get('shape') != drink_yolo:
                continue
            if center_only and h and w:
                px, py = d['pixel']
                margin_x, margin_y = w * 0.25, h * 0.25
                if not (margin_x < px < w - margin_x and margin_y < py < h - margin_y):
                    continue
            return d
        return None

    def _compute_world_xyz(self, det):
        """像素坐标+深度 → 基座3D坐标 (复用 yolo_pick_node 逻辑)"""
        import numpy as np

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

        import arm_fk
        try:
            pwms = self._read_joint_pwms()
            if pwms is None:
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

    def _read_pwm(self, idx, timeout=0.6):
        z_uart.uart_send_str(f"#{idx:03d}PRAD!")
        dl = time.time() + timeout
        while time.time() < dl:
            if z_uart.uart_get_ok:
                d = z_uart.uart_receive_buf
                z_uart.uart_receive_buf = ''
                z_uart.uart_get_ok = 0
                import re
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

    # ══════════════════════════════════════════════════════════════════
    #  主流程
    # ══════════════════════════════════════════════════════════════════

    def _execute_pick(self, cmd):
        if self.active:
            self._publish_status('busy', '正在执行中，请等待')
            return

        drink_cn = cmd.get('drink', '')
        drink_yolo = DRINK_MAP.get(drink_cn, drink_cn)
        self.get_logger().info(f'目标饮品: {drink_cn} → {drink_yolo}')

        self.active = True
        try:
            # ── 初始化串口（必须在机械臂运动前） ──
            if not z_uart.setup_uart(115200):
                self._publish_status('error', '串口初始化失败')
                return

            # ── 初始观察位置 ──
            kinematics_move(0, 120, 170, 1000, alpha_hint=ALPHA_OBS)
            time.sleep(1.5)

            # ── Step 1: 导轨前进扫描（多帧确认） ──
            self._publish_status('searching', f'导轨前进，搜索{drink_cn}...')
            self.motor.run(0, 30)

            found = False
            confirm = 0
            for _ in range(200):  # ~20秒
                time.sleep(0.1)
                if self._check_target(drink_yolo, center_only=True):
                    confirm += 1
                    if confirm >= CONFIRM_FRAMES:
                        found = True
                        break
                else:
                    confirm = 0

            if found:
                # ── 检测到，停止导轨 ──
                self.motor.stop_smooth()
                self._publish_status('found', f'检测到{drink_cn}，准备抓取')
            else:
                # ── Step 2: 前进未找到，碰撞回零 ──
                self.motor.stop_smooth()
                self._publish_status('searching', f'前进未找到{drink_cn}，回零搜索...')

                homing_done = threading.Event()
                homing_result = [None]

                def do_home():
                    homing_result[0] = self.motor.home_and_wait()
                    homing_done.set()

                threading.Thread(target=do_home, daemon=True).start()

                found = False
                confirm = 0
                while not homing_done.is_set():
                    time.sleep(0.1)
                    if self._check_target(drink_yolo, center_only=True):
                        confirm += 1
                        if confirm >= CONFIRM_FRAMES:
                            self.motor.abort_home()
                            homing_done.wait(timeout=2)
                            found = True
                            break
                    else:
                        confirm = 0

                if found:
                    self._publish_status('found', f'回零途中检测到{drink_cn}，准备抓取')
                else:
                    # ── 一来一回都没找到 ──
                    self._speak(f'未检测到{drink_cn}')
                    self._publish_status('not_found', f'未检测到{drink_cn}')
                    kinematics_move(0, 120, 170, 1000, alpha_hint=ALPHA_OBS)
                    return

            # ── Step 3: 等稳定 ──
            time.sleep(2)

            # ── Step 4: 定位 ──
            det = self._check_target(drink_yolo)
            if det is None:
                self._speak(f'目标丢失')
                self._publish_status('lost', '目标丢失')
                return

            world_xyz = self._compute_world_xyz(det)
            if world_xyz is None:
                self._speak(f'无法定位{drink_cn}')
                self._publish_status('error', '无法计算3D坐标')
                return

            tx, ty, tz = world_xyz
            self.get_logger().info(f'世界坐标: ({tx:.0f}, {ty:.0f}, {tz:.0f})mm')

            # ── Step 5: 飞到目标上方 ──
            self._publish_status('picking', '移动到目标上方...')
            hover_z = max(int(tz) + 80, 60)
            if not kinematics_move(int(tx), int(ty), hover_z, 1500, alpha_hint=ALPHA_GRASP):
                self._speak('目标超出工作空间')
                self._publish_status('error', 'IK无解')
                return
            time.sleep(1.6)

            # ── Step 6: 打开夹爪 ──
            for _ in range(3):
                z_uart.uart_send_str('#005P1000T500!')
                time.sleep(0.3)

            # ── Step 7: 下降 + 向目标推进（饮品专属参数） ──
            drop, push = GRAB_PARAMS.get(drink_yolo, GRAB_DEFAULT)
            grab_z = max(int(tz) - drop, 5)
            grab_y = int(ty) + push
            kinematics_move(int(tx), grab_y, grab_z, 1200, alpha_hint=ALPHA_GRASP)
            time.sleep(1.3)

            # ── Step 8: 关闭夹爪 ──
            for _ in range(3):
                z_uart.uart_send_str('#005P1700T1000!')
                time.sleep(0.4)

            # ── Step 9: 抬升 ──
            self._publish_status('picking', '抓取成功，抬升...')
            kinematics_move(int(tx), int(ty), 150, 1000, alpha_hint=ALPHA_GRASP)
            time.sleep(1)

            # ── Step 10: 移到过渡位置（夹住物体） ──
            z_uart.uart_send_str(
                '#000P1566T2000!#001P1774T2000!#002P2289T2000!'
                '#003P1300T2000!#004P1500T2000!'
            )
            time.sleep(2.5)

            # ── Step 11: 碰撞回零 ──
            self._publish_status('homing', '导轨碰撞回零...')
            result = self.motor.home_and_wait()
            if '回零成功' not in result:
                self.get_logger().warn(f'回零异常: {result}')

            # ── Step 12: 移到放置区 ──
            self._publish_status('placing', '移到放置区...')
            z_uart.uart_send_str(
                '#000P1126T2000!#001P935T2000!#002P2175T2000!'
                '#003P2010T2000!#004P1500T2000!'
            )
            time.sleep(2.5)

            # ── Step 13: 打开夹爪放下 ──
            for _ in range(3):
                z_uart.uart_send_str('#005P1000T500!')
                time.sleep(0.3)

            # ── Step 14: 回观察位置 ──
            kinematics_move(0, 120, 170, 2000, alpha_hint=ALPHA_OBS)
            time.sleep(1)

            self._publish_status('done', f'{drink_cn}抓取完成')

        except Exception as e:
            self._publish_status('error', f'异常: {e}')
            self.get_logger().error(f'抓取异常: {e}')
        finally:
            self.active = False

    def destroy_node(self):
        try:
            self.motor.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DrinkPickNode()

    # 命令行交互线程
    def cli_loop():
        import sys
        while rclpy.ok():
            try:
                sys.stdout.write('\n请输入饮品名 (咖啡/冰红茶/果冻/牛奶/草莓酸奶): ')
                sys.stdout.flush()
                line = sys.stdin.readline().strip()
                if not line:
                    continue
                msg = String()
                msg.data = json.dumps({'drink': line}, ensure_ascii=False)
                node.pub_cmd.publish(msg)
                node.get_logger().info(f'CLI 发送: {msg.data}')
            except (EOFError, KeyboardInterrupt):
                break

    import threading
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
