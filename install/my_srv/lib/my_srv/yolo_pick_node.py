#!/usr/bin/env python3
"""
yolo_pick_node.py — YOLO 物体抓取节点

接收语音指令，结合 YOLO 检测结果，控制机械臂抓取指定物体。

用法:
  python3 ~/ros2_ws/install/my_srv/lib/my_srv/yolo_pick_node.py

前提: yolo_detect_node.py 必须在运行（提供 /yolo/detections 话题）

话题:
  订阅: /yolo/detections (JSON), /yolo/pick_cmd (JSON)
  发布: /yolo/pick_status (JSON)
"""

import os, sys, re, time, json, threading
import numpy as np, cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

# ── 路径（复用现有模块）──
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
for p in (_SCRIPT_DIR, os.path.expanduser('~/ros2_ws/src/my_srv/scripts'),
          os.path.expanduser('~/OpenCV')):
    if p not in sys.path:
        sys.path.insert(0, p)

import arm_fk
import z_uart
from z_uart import uart_send_str, setup_uart, close_uart
from z_move import kinematics_move

# ── 中文形状 → YOLO 类别 ──
SHAPE_TO_YOLO = {
    '长方体': 'cuboid', '正方体': 'screwdriver', '圆柱体': 'cube',
    '球体': 'sphere', '圆球': 'sphere', '螺丝刀': 'cylinder',
}

# ── 颜色 ──
COLOR_MAP = {'红色': 'red', '绿色': 'green', '蓝色': 'blue',
             '红': 'red', '绿': 'green', '蓝': 'blue'}


class YoloPickNode(Node):

    def __init__(self):
        super().__init__('yolo_pick_node')

        self.latest_detections = []
        self._det_lock = threading.Lock()
        self.active = False

        # 深度图（用于世界坐标定位）
        self.latest_depth = None
        self.fx = self.fy = self.cx_c = self.cy_c = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(String, '/yolo/detections', self._det_cb, 10)
        self.create_subscription(String, '/yolo/pick_cmd', self._cmd_cb, 10)
        self.create_subscription(Image, '/aurora/depth/image_raw', self._depth_cb, qos)
        self.create_subscription(CameraInfo, '/aurora/rgb/camera_info', self._info_cb, qos)

        self.pub_status = self.create_publisher(String, '/yolo/pick_status', 10)

        self.get_logger().info('YOLO 抓取节点已启动（等待指令）')

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
            threading.Thread(target=self._execute_pick, args=(cmd,), daemon=True).start()
        except Exception as e:
            self.get_logger().error(f'指令解析失败: {e}')

    def _find_target(self, shape_yolo, color_en):
        """从最新检测结果中找目标物体"""
        with self._det_lock:
            dets = list(self.latest_detections)

        # 按形状过滤
        candidates = [d for d in dets if d['shape'] == shape_yolo and d.get('depth_mm', 0) > 100]

        if not candidates:
            return None

        if color_en:
            # 有颜色要求：精确匹配
            for d in candidates:
                if d['color'] == color_en:
                    return d
            return None

        # 无颜色要求：按像素面积从大到小排序
        candidates.sort(key=lambda d: d.get('area', 0), reverse=True)
        return candidates[0]

    def _compute_world_xyz(self, det):
        """用检测结果的像素坐标 + 深度图计算基座3D坐标（复用 depth_utils 逻辑）"""
        if self.fx is None or self.latest_depth is None:
            return None

        px, py = det['pixel']
        depth_mm = det.get('depth_mm', 0)

        # 如果检测结果没有深度，重新搜索
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

        # pixel → camera 3D
        z = depth_mm / 1000.0
        x = (px - self.cx_c) * z / self.fx
        y = (py - self.cy_c) * z / self.fy
        p_cam = np.array([x, y, z])

        # camera → base（复用 arm_fk）
        try:
            pwms = self._read_joint_pwms()
            if pwms is None:
                return None
            th = arm_fk.pwms_to_angles(*pwms)
            T_g2b_mm = arm_fk.compute_T_base_to_ee_from_angles(*th)
            T_g2b = arm_fk.T_mm_to_m(T_g2b_mm)

            # camera → gripper → base
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

    def _publish_status(self, status, message):
        msg = String()
        msg.data = json.dumps({'status': status, 'message': message}, ensure_ascii=False)
        self.pub_status.publish(msg)
        self.get_logger().info(f'[{status}] {message}')

    def _find_all_targets(self, shape_yolo, color_en):
        """查找所有匹配的物体，按像素面积从大到小排序"""
        with self._det_lock:
            dets = list(self.latest_detections)
        candidates = [d for d in dets if d['shape'] == shape_yolo and d.get('depth_mm', 0) > 100]
        if color_en:
            candidates = [d for d in candidates if d['color'] == color_en]
        candidates.sort(key=lambda d: d.get('area', 0), reverse=True)
        return candidates

    def _pick_one(self, det, label):
        """执行单次抓取流程，返回 True 成功 / False 失败"""
        # 计算世界坐标
        world_xyz = self._compute_world_xyz(det)
        if world_xyz is None:
            self.get_logger().warn(f'无法计算 {label} 的3D坐标，跳过')
            return False

        tx, ty, tz = world_xyz
        self.get_logger().info(f'世界坐标: ({tx:.0f}, {ty:.0f}, {tz:.0f})mm')

        # 飞到目标上方
        self._publish_status('picking', f'移动到 {label} 上方...')
        hover_z = max(int(tz) + 80, 60)
        if not kinematics_move(int(tx), int(ty), hover_z, 1500, alpha_hint=-82):
            self.get_logger().warn(f'IK无解：{label} 超出工作空间，跳过')
            return False
        time.sleep(1.6)

        # 打开夹爪
        for _ in range(3):
            uart_send_str("#005P1000T500!")
            time.sleep(0.3)

        # 下降抓取
        grab_z = max(int(tz) - 5, 5)
        if not kinematics_move(int(tx), int(ty), grab_z, 1200, alpha_hint=-82):
            self.get_logger().warn(f'IK无解：下降位置超出工作空间，跳过')
            return False
        time.sleep(1.3)

        # 闭合夹爪
        for _ in range(3):
            uart_send_str("#005P1700T1000!")
            time.sleep(0.4)

        # 抬升
        self._publish_status('picking', f'{label} 抓取成功，抬升...')
        kinematics_move(int(tx), int(ty), 150, 1000, alpha_hint=-82)
        time.sleep(1)

        # 移动到放置区
        self._publish_status('placing', '移动到放置区...')
        uart_send_str('#000P2189T1000!#001P1165T1000!#002P1707T1000!'
                      '#003P0828T1000!#004P1500T1000!')
        time.sleep(1.5)

        # 打开夹爪放下物体
        for _ in range(3):
            uart_send_str("#005P1000T500!")
            time.sleep(0.3)

        # 回到观察位置
        kinematics_move(0, 120, 120, 1000, alpha_hint=-82)
        time.sleep(1.5)
        return True

    def _execute_pick(self, cmd):
        """执行抓取（支持多个物体依次抓取）"""
        if self.active:
            self._publish_status('busy', '正在抓取中，请等待')
            return

        shape_cn = cmd.get('shape', '')
        color_cn = cmd.get('color', '')
        shape_yolo = SHAPE_TO_YOLO.get(shape_cn, shape_cn)
        color_en = COLOR_MAP.get(color_cn, '')

        self._publish_status('searching', f'正在查找 {color_cn}{shape_cn}...')
        self.get_logger().info(f'目标: shape={shape_yolo} color={color_en or "任意（全部抓取）"}')

        self.active = True
        try:
            # 1. 初始化串口
            if not setup_uart(115200):
                self._publish_status('error', '串口初始化失败')
                return

            # 初始位姿
            kinematics_move(0, 120, 120, 1000, alpha_hint=-82)
            time.sleep(1.5)

            # 2. 等待检测结果
            for _ in range(50):
                with self._det_lock:
                    if self.latest_detections:
                        break
                time.sleep(0.1)
            else:
                self._publish_status('error', '未收到 YOLO 检测结果，请确认 yolo_detect_node 在运行')
                return

            # 3. 循环抓取所有匹配物体
            picked_count = 0
            max_rounds = 10  # 最多抓10个防止死循环

            for round_idx in range(max_rounds):
                # 每次重新检测（抓取后画面可能变化）
                time.sleep(0.5)
                targets = self._find_all_targets(shape_yolo, color_en)

                if not targets:
                    if picked_count == 0:
                        self._publish_status('not_found', f'未找到 {color_cn}{shape_cn}')
                    else:
                        self._publish_status('done', f'全部抓取完成，共 {picked_count} 个')
                    return

                det = targets[0]  # 每次抓最大的
                label = f"{det['color']} {det['shape']}"
                self._publish_status('found',
                    f'第 {picked_count + 1} 个: {label} (剩余 {len(targets) - 1} 个)')

                success = self._pick_one(det, label)
                if success:
                    picked_count += 1
                    self.get_logger().info(f'已抓取 {picked_count} 个')
                else:
                    self.get_logger().warn(f'{label} 抓取失败，继续下一个')

            self._publish_status('done', f'抓取完成，共 {picked_count} 个')

        except Exception as e:
            self._publish_status('error', f'抓取异常: {e}')
        finally:
            self.active = False


def main(args=None):
    rclpy.init(args=args)
    node = YoloPickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
