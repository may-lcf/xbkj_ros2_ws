#!/usr/bin/env python3
"""
garbage_sort_node.py — 垃圾分类抓取节点

用法:
  python3 ~/ros2_ws/install/my_srv/lib/my_srv/garbage_sort_node.py

前提: yolo_detect_node.py 必须在运行

话题:
  订阅: /yolo/detections, /garbage_sort/cmd (JSON: {"garbage_type": "可回收垃圾"})
  发布: /garbage_sort/status (JSON)
"""

import os, sys, re, time, json, threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
for p in (_SCRIPT_DIR, os.path.expanduser('~/ros2_ws/src/my_srv/scripts'),
          os.path.expanduser('~/OpenCV')):
    if p not in sys.path:
        sys.path.insert(0, p)

import arm_fk
import z_uart
from z_uart import uart_send_str, setup_uart, close_uart
from z_move import kinematics_move

# ── 观察位置 ──
OBS_X, OBS_Y, OBS_Z = 0, 160, 120

# ── 垃圾分类 → YOLO类别 + 垃圾桶PWM ──
GARBAGE_CONFIG = {
    '可回收垃圾': {
        'labels': ['bottle', 'newspaper'],
        'bin_pwm': '#000P2057T2000!#001P1435T2000!#002P1838T2000!#003P0828T2000!#004P1500T2000!',
    },
    '有害垃圾': {
        'labels': ['battery'],
        'bin_pwm': '#000P1951T2000!#001P1220T2000!#002P1597T2000!#003P0839T2000!#004P1500T2000!',
    },
    '厨余垃圾': {
        'labels': ['banana_peel'],
        'bin_pwm': '#000P2254T2000!#001P1420T2000!#002P1820T2000!#003P0818T2000!#004P1500T2000!',
    },
    '其他垃圾': {
        'labels': ['toilet_paper'],
        'bin_pwm': '#000P2439T2000!#001P1437T2000!#002P1899T2000!#003P0928T2000!#004P1500T2000!',
    },
}

# ── 中文名称（用于语音回复）──
LABEL_CN = {
    'banana_peel': '香蕉皮', 'battery': '电池', 'bottle': '塑料瓶',
    'newspaper': '报纸', 'toilet_paper': '卫生纸',
}


class GarbageSortNode(Node):

    def __init__(self):
        super().__init__('garbage_sort_node')

        self.latest_detections = []
        self._det_lock = threading.Lock()
        self.latest_depth = None
        self.fx = self.fy = self.cx_c = self.cy_c = None
        self.active = False

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(String, '/yolo/detections', self._det_cb, 10)
        self.create_subscription(String, '/garbage_sort/cmd', self._cmd_cb, 10)
        self.create_subscription(Image, '/aurora/depth/image_raw', self._depth_cb, qos)
        self.create_subscription(CameraInfo, '/aurora/rgb/camera_info', self._info_cb, qos)

        self.pub_status = self.create_publisher(String, '/garbage_sort/status', 10)
        self.pub_speak = self.create_publisher(String, '/speak_text', 10)

        self.get_logger().info('垃圾分类节点已启动（等待指令）')

    def _info_cb(self, msg):
        if self.fx is not None:
            return
        K = np.array(msg.k).reshape(3, 3)
        self.fx, self.fy = K[0, 0], K[1, 1]
        self.cx_c, self.cy_c = K[0, 2], K[1, 2]

    def _depth_cb(self, msg):
        try:
            self.latest_depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        except Exception:
            pass

    def _det_cb(self, msg):
        try:
            with self._det_lock:
                self.latest_detections = json.loads(msg.data)
        except Exception:
            pass

    def _cmd_cb(self, msg):
        try:
            cmd = json.loads(msg.data)
            threading.Thread(target=self._execute_sort, args=(cmd,), daemon=True).start()
        except Exception as e:
            self.get_logger().error(f'指令解析失败: {e}')

    def _speak(self, text):
        m = String()
        m.data = text
        self.pub_speak.publish(m)

    def _publish_status(self, status, message):
        msg = String()
        msg.data = json.dumps({'status': status, 'message': message}, ensure_ascii=False)
        self.pub_status.publish(msg)
        self.get_logger().info(f'[{status}] {message}')

    def _find_targets(self, labels):
        """查找所有匹配的物体，按像素面积从大到小排序"""
        with self._det_lock:
            dets = list(self.latest_detections)
        candidates = [d for d in dets if d['shape'] in labels and d.get('depth_mm', 0) > 100]
        candidates.sort(key=lambda d: d.get('area', 0), reverse=True)
        return candidates

    def _compute_world_xyz(self, det):
        """像素+深度 → 基座3D坐标"""
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
            import yaml
            calib_dir = os.path.expanduser('~/ros2_ws/src/my_srv/config')
            with open(os.path.join(calib_dir, 'hand_eye_calib.yaml')) as f:
                data = yaml.safe_load(f)
            T_c2g = np.eye(4)
            T_c2g[:3, :3] = np.array(data['R_cam2gripper'])
            T_c2g[:3, 3] = np.array(data['t_cam2gripper']).flatten()
            p = np.append(p_cam, 1.0)
            p_base = T_g2b @ T_c2g @ p
            return tuple(float(v) * 1000 for v in p_base[:3])
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

    def _pick_one(self, det, label_cn):
        """单次抓取流程，返回 True/False"""
        world_xyz = self._compute_world_xyz(det)
        if world_xyz is None:
            self.get_logger().warn(f'无法计算 {label_cn} 的3D坐标')
            return False
        tx, ty, tz = world_xyz
        self.get_logger().info(f'世界坐标: ({tx:.0f},{ty:.0f},{tz:.0f})mm')

        # 飞到上方 + 开夹爪
        self._publish_status('picking', f'移动到 {label_cn} 上方...')
        hover_z = max(int(tz) + 80, 60)
        if not kinematics_move(int(tx), int(ty), hover_z, 1500, alpha_hint=-82):
            return False
        time.sleep(1.6)
        for _ in range(3):
            uart_send_str("#005P1000T500!")
            time.sleep(0.3)

        # 下降抓取
        grab_z = max(int(tz) - 5, 5)
        if not kinematics_move(int(tx), int(ty), grab_z, 1200, alpha_hint=-82):
            return False
        time.sleep(1.3)
        for _ in range(3):
            uart_send_str("#005P1700T1000!")
            time.sleep(0.4)

        # 抬升 → 回观察位置
        self._publish_status('picking', f'{label_cn} 抓取成功，抬升...')
        kinematics_move(int(tx), int(ty), 150, 1000, alpha_hint=-82)
        time.sleep(1)
        kinematics_move(OBS_X, OBS_Y, OBS_Z, 1000, alpha_hint=-82)
        time.sleep(1.5)
        return True

    def _execute_sort(self, cmd):
        """垃圾分类主流程"""
        if self.active:
            self._publish_status('busy', '正在处理中')
            return

        garbage_type = cmd.get('garbage_type', '')
        config = GARBAGE_CONFIG.get(garbage_type)
        if not config:
            self._publish_status('error', f'未知垃圾类型: {garbage_type}')
            return

        labels = config['labels']
        bin_pwm = config['bin_pwm']
        label_cn = '、'.join(LABEL_CN.get(l, l) for l in labels)

        self._publish_status('start', f'开始清理 {garbage_type}（{label_cn}）')

        self.active = True
        try:
            if not setup_uart(115200):
                self._publish_status('error', '串口初始化失败')
                return

            # 回到观察位置
            kinematics_move(OBS_X, OBS_Y, OBS_Z, 1000, alpha_hint=-82)
            time.sleep(1.5)

            # 等待检测数据
            for _ in range(50):
                with self._det_lock:
                    if self.latest_detections:
                        break
                time.sleep(0.1)
            else:
                self._publish_status('error', '未收到 YOLO 检测结果')
                return

            picked_count = 0
            max_rounds = 10

            for _ in range(max_rounds):
                time.sleep(2.0)
                targets = self._find_targets(labels)
                if not targets:
                    break

                det = targets[0]
                label_cn_item = LABEL_CN.get(det['shape'], det['shape'])
                self._publish_status('found',
                    f'第 {picked_count + 1} 个: {label_cn_item} (剩余 {len(targets) - 1} 个)')

                # 抓取
                if not self._pick_one(det, label_cn_item):
                    self.get_logger().warn(f'{label_cn_item} 抓取失败，跳过')
                    continue

                picked_count += 1

                # 移到对应垃圾桶
                self._publish_status('placing', f'移动到{garbage_type}桶...')
                uart_send_str(bin_pwm)
                time.sleep(2.5)

                # 打开夹爪
                for _ in range(3):
                    uart_send_str("#005P1000T500!")
                    time.sleep(0.3)

                # 回观察位置
                kinematics_move(OBS_X, OBS_Y, OBS_Z, 1000, alpha_hint=-82)

                # 等待1.5秒让机械臂稳定
                time.sleep(1.5)

            if picked_count > 0:
                self._publish_status('done', f'{garbage_type}清理完成，共 {picked_count} 个')
            else:
                self._publish_status('done', f'没有找到{garbage_type}')

        except Exception as e:
            self._publish_status('error', f'异常: {e}')
        finally:
            self.active = False


def main(args=None):
    rclpy.init(args=args)
    node = GarbageSortNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
