#!/usr/bin/env python3
"""
height_check_node.py — 高度异常检测与抓取节点

通过 YOLO + 深度相机监测桌面上各物体的高度。
当检测到某物体比其他物体高出 >15mm（被叠放），自动抓取该物体到放置区。

前提: yolo_detect_node.py 必须在运行（提供 /yolo/detections 话题）

话题:
  订阅:
    /yolo/detections         (JSON) — YOLO 检测结果
    /aurora/depth/image_raw  (mono16, mm)
    /aurora/rgb/camera_info  (CameraInfo)
  发布:
    /height_check/status     (JSON — 状态)
  服务:
    /height_check/enter  (Trigger) — 启动监测
    /height_check/exit   (Trigger) — 停止
"""

import os, sys, time, json, re, threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from example_interfaces.srv import Trigger

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

# ── 常量 ──
INIT_POS = '#000P1500T1000!#001P1883T1000!#002P1939T1000!#003P0823T1000!#004P1500T1000!'
OBSERVE_POS = None  # 使用 kinematics_move(0, 110, 130) 代替
DROP_POS = '#000P2189T1000!#001P1165T1000!#002P1707T1000!#003P0828T1000!#004P1500T1000!'
HEIGHT_THRESH = 20  # 高度异常阈值 (mm)
ANOMALY_FRAMES = 3  # 连续检测到异常的帧数


class HeightCheckNode(Node):

    def __init__(self):
        super().__init__('height_check_node')

        self.latest_detections = []
        self._det_lock = threading.Lock()
        self.latest_depth = None
        self.fx = self.fy = self.cx_c = self.cy_c = None

        self.active = False
        self.picking = False
        self.baseline_depth = None    # 基线深度（最浅物体的 depth）
        self.anomaly_count = 0        # 连续异常帧计数
        self.anomaly_det = None       # 当前异常检测结果

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(String, '/yolo/detections', self._det_cb, 10)
        self.create_subscription(Image, '/aurora/depth/image_raw', self._depth_cb, qos)
        self.create_subscription(CameraInfo, '/aurora/rgb/camera_info', self._info_cb, qos)

        self.pub_status = self.create_publisher(String, '/height_check/status', 10)

        self.create_service(Trigger, '/height_check/enter', self._enter_cb)
        self.create_service(Trigger, '/height_check/exit', self._exit_cb)

        self._monitor_thread = None
        self.get_logger().info('\033[1;36m[HeightCheck]\033[0m 高度检测节点已启动')

    # ══════════════════════════════════════════════════════════════════════════
    #  回调
    # ══════════════════════════════════════════════════════════════════════════

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

    def _publish_status(self, status, message):
        msg = String()
        msg.data = json.dumps({'status': status, 'message': message}, ensure_ascii=False)
        self.pub_status.publish(msg)
        self.get_logger().info(f'[{status}] {message}')

    # ══════════════════════════════════════════════════════════════════════════
    #  服务
    # ══════════════════════════════════════════════════════════════════════════

    def _enter_cb(self, request, response):
        self.get_logger().info('收到 Enter 服务，启动高度检测！')
        if self.active:
            response.success = True
            response.message = '已在运行'
            return response
        try:
            if not setup_uart(115200):
                response.success = False
                response.message = '串口初始化失败'
                return response
            kinematics_move(0, 120, 120, 1000, alpha_hint=-82)
            time.sleep(1.5)
            self.get_logger().info('[HeightCheck] 观察位就绪，等待稳定...')
            time.sleep(1.0)
            self.active = True
            self.baseline_depth = None
            self.anomaly_count = 0
            self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self._monitor_thread.start()
        except Exception as e:
            response.success = False
            response.message = f'初始化失败: {e}'
            return response
        response.success = True
        response.message = '高度检测已启动'
        return response

    def _exit_cb(self, request, response):
        self.get_logger().info('收到 Exit 服务，停止高度检测！')
        self.active = False
        self.picking = False
        close_uart()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=3.0)
        response.success = True
        response.message = '高度检测已停止'
        return response

    # ══════════════════════════════════════════════════════════════════════════
    #  坐标转换（复用 yolo_pick_node 方法）
    # ══════════════════════════════════════════════════════════════════════════

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

    def _compute_world_xyz(self, det):
        """用检测结果的像素坐标 + 深度图计算基座3D坐标（复用 yolo_pick_node 方法）"""
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

        # camera → base
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

    # ══════════════════════════════════════════════════════════════════════════
    #  抓取流程（复用 yolo_pick_node）
    # ══════════════════════════════════════════════════════════════════════════

    def _pick_one(self, det, label):
        """执行单次抓取流程"""
        world_xyz = self._compute_world_xyz(det)
        if world_xyz is None:
            self.get_logger().warn(f'无法计算 {label} 的3D坐标，跳过')
            return False

        tx, ty, tz = world_xyz
        self.get_logger().info(f'世界坐标: ({tx:.0f}, {ty:.0f}, {tz:.0f})mm')

        try:
            # 打开夹爪
            for _ in range(3):
                uart_send_str("#005P1000T500!")
                time.sleep(0.3)

            # 直接飞向目标抓取
            self._publish_status('picking', f'飞向 {label}...')
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
            uart_send_str(DROP_POS)
            time.sleep(1.5)

            # 打开夹爪放下物体
            for _ in range(3):
                uart_send_str("#005P1000T500!")
                time.sleep(0.3)

            # 回到观察位置
            kinematics_move(0, 120, 120, 1000, alpha_hint=-82)
            time.sleep(1.5)
            self.get_logger().info('[HeightCheck] 回到观察位，等待稳定...')
            time.sleep(1.0)

            return True

        except Exception as e:
            self._publish_status('error', f'抓取异常: {e}')
            return False

    # ══════════════════════════════════════════════════════════════════════════
    #  主监测循环
    # ══════════════════════════════════════════════════════════════════════════

    def _monitor_loop(self):
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

        self._publish_status('ready', '高度监测就绪')
        self.get_logger().info('[HeightCheck] 开始监测，等待物体...')

        while self.active and rclpy.ok():
            if self.picking:
                time.sleep(0.2)
                continue

            # 获取检测结果
            with self._det_lock:
                dets = list(self.latest_detections)

            # 过滤有效物体（有深度的）
            valid = [d for d in dets if d.get('depth_mm', 0) > 100]

            if len(valid) < 2:
                # 不够 2 个物体，无法比较
                time.sleep(0.5)
                continue

            # 按深度排序（最浅在前 = 最高的物体）
            valid.sort(key=lambda d: d['depth_mm'])

            depths = [d['depth_mm'] for d in valid]
            median_depth = float(np.median(depths))

            # 建立基线：采集 10 帧取中位数，避免启动时深度不稳定
            if self.baseline_depth is None:
                if not hasattr(self, '_baseline_buf'):
                    self._baseline_buf = []
                self._baseline_buf.append(median_depth)
                if len(self._baseline_buf) < 6:
                    self.get_logger().info(
                        f'[HeightCheck] 采集基线中... ({len(self._baseline_buf)}/6)')
                    time.sleep(0.5)
                    continue
                self.baseline_depth = float(np.median(self._baseline_buf))
                del self._baseline_buf
                self.get_logger().info(
                    f'[HeightCheck] 基线建立: {self.baseline_depth:.0f}mm '
                    f'(共 {len(valid)} 个物体)')

            # 检查异常：深度 < 基线 - 15mm → 物体比其他的高
            anomaly = None
            for d in valid:
                if self.baseline_depth - d['depth_mm'] > HEIGHT_THRESH:
                    anomaly = d
                    break

            if anomaly:
                self.anomaly_count += 1
                label = f"{anomaly.get('color', '')} {anomaly['shape']}"
                height_diff = self.baseline_depth - anomaly['depth_mm']
                self.get_logger().warn(
                    f'[HeightCheck] 异常 #{self.anomaly_count}: {label} '
                    f'高出 {height_diff:.0f}mm (depth={anomaly["depth_mm"]}mm, '
                    f'baseline={self.baseline_depth:.0f}mm)')

                if self.anomaly_count >= ANOMALY_FRAMES:
                    # 确认异常，执行抓取
                    self._publish_status('anomaly',
                        f'{label} 高出 {height_diff:.0f}mm，抓取中...')
                    self.picking = True
                    success = self._pick_one(anomaly, label)
                    if success:
                        self._publish_status('done', f'{label} 已抓取到放置区')
                        # 重置基线（物体被移走后重新采集）
                        self.baseline_depth = None
                    else:
                        self._publish_status('error', f'{label} 抓取失败')
                    self.anomaly_count = 0
                    self.anomaly_det = None
                    self.picking = False
            else:
                # 无异常，更新基线（用滑动平均适应环境变化）
                if self.baseline_depth is not None:
                    self.baseline_depth = self.baseline_depth * 0.9 + median_depth * 0.1
                self.anomaly_count = 0
                self.anomaly_det = None

            time.sleep(0.5)

        self.get_logger().info('[HeightCheck] 监测结束')


def main(args=None):
    rclpy.init(args=args)
    node = HeightCheckNode()
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
