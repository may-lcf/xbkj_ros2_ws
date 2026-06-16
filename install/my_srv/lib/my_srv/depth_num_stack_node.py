#!/usr/bin/env python3
"""
depth_num_stack_node.py — 深度增强数字码垛

使用 Aurora 930 深度相机 + TFLite 数字检测 + 世界系定位。
检测逻辑参考 num_stack_node.py，坐标定位参考 depth_color_stack_node.py。
"""

import os, sys, re, time, math, threading
import numpy as np, cv2
from ai_edge_litert.interpreter import Interpreter

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

MODEL_PATH = os.path.join(os.path.expanduser('~'), 'OpenCV', 'trained2.tflite')
LABELS_PATH = os.path.join(os.path.expanduser('~'), 'OpenCV', 'labels.txt')
CONF_THRESHOLD = 0.4
MODEL_INPUT_H = 128
MODEL_INPUT_W = 128


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


class DepthNumStackNode(Node):
    def __init__(self):
        super().__init__('depth_num_stack_node')
        self.du = DepthUtils(self)

        self.latest_rgb = None
        self.latest_depth = None
        self._frame_lock = threading.Lock()
        self.bridge = CvBridge()
        self.width, self.height = 640, 480

        self.interpreter, self.input_det, self.output_det, self.labels = self._load_model()

        self.move_x, self.move_y, self.move_z = 0, 120, 60
        self.move_status = 0
        self.color_read_succed = 0
        self.TARGET_CX, self.TARGET_CY = 320, 240
        self.block_cx, self.block_cy = self.TARGET_CX, self.TARGET_CY
        self.detected_color = None
        self.success_cnt = 0
        self.stack_active = False
        self.active = False  # enter/exit 模式守卫
        self._run_thread = None
        self.world_target_mm = None
        self._last_logged_status = -1

        self.num = None
        self.target_nums = ["1", "2", "3"]
        self.current_num_index = 0

        self.mark_flag = 255
        self.bak_cx, self.bak_cy = -130, 30
        self.block_cnt = 0
        self.stack_world_mm = None
        self.stack_height_one = 10
        self.stack_height_two = 46
        self.stack_height_three = 70
        self.place_offset_x = 56
        self.place_offset_y = 56

        self.pid_x = PIDController(kp=0.01, ki=0.000, kd=0.0)
        self.pid_y = PIDController(kp=0.01, ki=0.000, kd=0.0)

        self.debug_pub = self.create_publisher(Image, '/depth_num_stack/image_result', 10)

        from message_filters import Subscriber as MfSub
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        rgb_sub = MfSub(self, Image, '/aurora/rgb/image_raw', _qos)
        depth_sub = MfSub(self, Image, '/aurora/depth/image_raw', _qos)
        self._sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        self._sync.registerCallback(self._synced_callback)

        # enter/exit 服务
        self.enter_srv = self.create_service(Trigger, '/depth_num_stack/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/depth_num_stack/exit', self.exit_callback)

        self.get_logger().info('\033[1;36m[DepthNumStack]\033[0m 深度增强数字码垛已启动')

    def _load_model(self):
        interpreter = Interpreter(model_path=MODEL_PATH)
        interpreter.allocate_tensors()
        input_det = interpreter.get_input_details()[0]
        output_det = interpreter.get_output_details()[0]
        with open(LABELS_PATH, "r", encoding="utf-8") as f:
            labels = [line.strip() for line in f if line.strip()]
        return interpreter, input_det, output_det, labels

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
    def _limit(d, mn, mx): return max(mn, min(mx, d))

    def _detect_numbers(self, frame):
        resized = cv2.resize(frame, (MODEL_INPUT_W, MODEL_INPUT_H))
        input_data = resized.astype(np.float32) / 255.0
        input_data = np.expand_dims(input_data, axis=0)
        self.interpreter.set_tensor(self.input_det['index'], input_data)
        self.interpreter.invoke()
        raw = self.interpreter.get_tensor(self.output_det['index']).squeeze(axis=0)
        detections = []
        conf_scores = raw[:, 4]
        cls_probs = raw[:, 5:]
        cls_ids = np.argmax(cls_probs, axis=1)
        total_confs = conf_scores * np.max(cls_probs, axis=1)
        for idx in np.where(total_confs > CONF_THRESHOLD)[0]:
            bx, by, bw, bh = raw[idx, :4] * 128
            x1, y1 = max(0, int(bx - bw / 2)), max(0, int(by - bh / 2))
            x2 = min(MODEL_INPUT_W - 1, int(bx + bw / 2))
            y2 = min(MODEL_INPUT_H - 1, int(by + bh / 2))
            sx, sy = self.width / MODEL_INPUT_W, self.height / MODEL_INPUT_H
            detections.append({
                "class_name": self.labels[int(cls_ids[idx])],
                "confidence": float(total_confs[idx]),
                "bbox": (int(x1 * sx), int(y1 * sy), int(x2 * sx), int(y2 * sy)),
                "center": (int((x1 + x2) / 2 * sx), int((y1 + y2) / 2 * sy))
            })
        return detections

    def _compute_world_target(self):
        cx, cy = int(self.block_cx), int(self.block_cy)
        with self._frame_lock:
            dimg = self.latest_depth
        if dimg is None: return None
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
        p_cam = self.du.pixel_to_3d(cx, cy, dmm)
        pwms = self._read_joint_pwms()
        if pwms is None:
            self.get_logger().warn('[世界] PWM读取失败')
            return None
        th = arm_fk.pwms_to_angles(*pwms)
        T_g2b = arm_fk.T_mm_to_m(arm_fk.compute_T_base_to_ee_from_angles(*th))
        p_base = self.du.transform_cam_to_base(p_cam, T_g2b)
        return (float(p_base[0]) * 1000, float(p_base[1]) * 1000, float(p_base[2]) * 1000)

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
        if not self.du.wait_for_intrinsics(15.0): return
        self.du.load_hand_eye_calib()
        for _ in range(200):
            with self._frame_lock:
                if self.latest_rgb is not None: break
            time.sleep(0.1)
        else:
            return

        self.stack_active = True
        self.move_x, self.move_y = 0, 105
        kinematics_move(self.move_x, self.move_y, 150, 1000, alpha_hint=-82)
        time.sleep(2.0)
        self.move_status = 0
        self.current_num_index = 0
        self.block_cnt = 0
        self.mark_flag = 255
        self.get_logger().info('\033[1;32m[DepthNumStack]\033[0m 码垛启动')

        fc = 0
        while self.stack_active and rclpy.ok():
            if not self.active:
                time.sleep(0.1)
                continue
            with self._frame_lock:
                if self.latest_rgb is None: time.sleep(0.03); continue
                frame = self.latest_rgb.copy()

            target_num = self._current_target()
            self.detected_color = None
            found = False

            detections = self._detect_numbers(frame)
            if self.mark_flag == 0:
                # 堆放基准用数字 "2" 作为标记（参考 num_stack_node）
                target_dets = [d for d in detections if d["class_name"] == "2"]
            else:
                target_dets = [d for d in detections if d["class_name"] == target_num]

            if target_dets:
                best = max(target_dets, key=lambda d: d["confidence"])
                self.block_cx, self.block_cy = best["center"]
                self.num = best["class_name"]
                self.detected_color = f'num_{self.num}'
                found = True
                x1, y1, x2, y2 = best["bbox"]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{self.num} {best['confidence']:.2f}",
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.color_read_succed = 1 if found else 0
            fc += 1
            if fc % 150 == 0 or self._last_logged_status != self.move_status:
                self.get_logger().info(
                    f'[NUM] f={fc} target={target_num} found={found} st={self.move_status} mark={self.mark_flag}')
                self._last_logged_status = self.move_status

            cv2.putText(frame, f"Stack: {self.block_cnt}/3 Target: {target_num}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
            except Exception:
                pass

            if self.stack_active and (self.color_read_succed or self.move_status >= 2):
                self._run_state_machine()

            time.sleep(0.03)

    def _current_target(self):
        if self.mark_flag == 0: return "2"  # 堆放基准标记
        return self.target_nums[self.current_num_index] if self.target_nums else "1"

    def _run_state_machine(self):
        stages = [self._st0, self._st1, self._st2, self._st3, self._st4,
                  self._st5, self._st6, self._st7, self._st8]
        if self.move_status < len(stages):
            stages[self.move_status]()

    def _st0(self):
        self.world_target_mm = self._compute_world_target()
        if self.world_target_mm is None:
            self.color_read_succed = 0
            self.get_logger().warn(f'[st0] 世界坐标计算失败, num={self.num} pix=({self.block_cx},{self.block_cy})')
            return
        if self.mark_flag == 0:
            self.bak_cx = int(self.world_target_mm[0])
            self.bak_cy = int(self.world_target_mm[1])
            self.mark_flag = 1
            self.get_logger().info(f'[st0] 堆放基准: ({self.bak_cx},{self.bak_cy})')
            self.color_read_succed = 0
            self.move_status = 5; return
        self.color_read_succed = 1
        self.move_status = 1

    def _st1(self):
        self.move_status = 2
        uart_send_str("#004P1500T800!")
        time.sleep(0.15)
        uart_send_str("#005P1000T500!")
        time.sleep(0.3)

    def _st2(self):
        self.move_status = 3
        if self.world_target_mm is None: return
        tx, ty, tz = self.world_target_mm
        self.move_x, self.move_y = int(tx), int(ty)
        if not kinematics_move(self.move_x, self.move_y, max(int(tz) + 80, 60), 1500, alpha_hint=-82):
            self.move_status = 8; return
        time.sleep(1.6)

    def _st3(self):
        self.move_status = 4
        if self.world_target_mm:
            tx, ty, tz = self.world_target_mm
            self.move_x, self.move_y = int(tx), int(ty)
            if not kinematics_move(self.move_x, self.move_y, max(int(tz) - 5, 5), 1200, alpha_hint=-82):
                self.move_status = 8; return
            time.sleep(1.3)
        for _ in range(3):
            uart_send_str("#005P1700T1000!")
            time.sleep(0.4)

    def _st4(self):
        self.move_status = 5
        self.block_cx = self.block_cy = 0
        if not kinematics_move(self.move_x, self.move_y, 150, 1000, alpha_hint=-82):
            self.move_status = 8; return
        time.sleep(1)
        uart_send_str("#004P1500T1000!")
        time.sleep(0.5)

    def _st5(self):
        if self.mark_flag == 255:
            # 数字码垛区在正 x 方向（参考 num_stack_node.py）
            self.move_x, self.move_y = 130, 20
            kinematics_move(self.move_x, self.move_y, 150, 1000)
            time.sleep(1)
            uart_send_str("#004P1500T1500!")
            time.sleep(0.5)
            kinematics_move(self.move_x, self.move_y, 60, 1000)
            time.sleep(2.5)
            self.mark_flag = 0
            self.color_read_succed = 0
        elif self.mark_flag == 1:
            self.move_x, self.move_y = self.bak_cx, self.bak_cy
            kinematics_move(self.move_x, self.move_y, 150, 1000)
            time.sleep(1)
            uart_send_str("#004P1500T1500!")
            time.sleep(0.5)
            kinematics_move(self.move_x, self.move_y, 60, 1000)
            time.sleep(2.5)
            self.color_read_succed = 0
            self.move_status = 6
        elif self.mark_flag == 0:
            if not self.color_read_succed:
                return
            self.pid_x.Target_val = self.TARGET_CX
            self.pid_y.Target_val = self.TARGET_CY
            # 正 x 侧用 +=（与负 x 侧的 -= 相反）
            self.move_y += self.pid_x.PID_Realize(self.block_cx)
            self.move_x += self.pid_y.PID_Realize(self.block_cy)
            self.move_x = self._limit(self.move_x, -200, 200)
            self.move_y = self._limit(self.move_y, -100, 250)
            kinematics_move(self.move_x, self.move_y, 60, 100)
            if abs(self.block_cx - self.TARGET_CX) <= 15 and abs(self.block_cy - self.TARGET_CY) <= 15:
                self.success_cnt += 1
                if self.success_cnt >= 2:
                    self.success_cnt = 0
                    self.mark_flag = 1
                    l = math.hypot(self.move_x, self.move_y)
                    if l > 0:
                        s, c = self.move_y / l, self.move_x / l
                        self.bak_cx = int((l + self.place_offset_x) * c)
                        self.bak_cy = int((l + self.place_offset_y) * s)
                    self.move_x, self.move_y = self.bak_cx, self.bak_cy
                    kinematics_move(self.move_x, self.move_y, 60, 1000)
                    time.sleep(1)
                    self.move_status = 6
            else:
                self.success_cnt = 0
            self.color_read_succed = 0

    def _st6(self):
        self.block_cx = self.block_cy = 0
        self.move_status = 7
        h = [self.stack_height_one, self.stack_height_two, self.stack_height_three][min(self.block_cnt, 2)]
        kinematics_move(self.move_x + self.block_cnt * 2, self.move_y + 5, h, 1200)
        time.sleep(2.5)
        for _ in range(3):
            uart_send_str("#005P1200T1000!")
            time.sleep(0.4)
        kinematics_move(self.move_x, self.move_y, 130, 1000)
        time.sleep(1)

    def _st7(self):
        self.move_x, self.move_y = 0, 120
        self.block_cnt += 1
        self.block_cx = self.block_cy = 0
        kinematics_move(self.move_x, self.move_y, 90, 1000, alpha_hint=-82)
        time.sleep(2)
        if self.block_cnt >= 3:
            self.block_cnt = 0
            self.mark_flag = 255
            self.stack_world_mm = None
            self.get_logger().info('[完成] 3 层码垛完成')
        self.current_num_index += 1
        if self.current_num_index >= len(self.target_nums):
            self.current_num_index = 0
        self.color_read_succed = 0
        self.move_status = 0
        self.world_target_mm = None
        self.get_logger().info(f'[完成] 下一数字: {self._current_target()}')

    def _st8(self):
        self.move_x, self.move_y = 0, 120
        kinematics_move(0, 105, 150, 1000, alpha_hint=-82)
        time.sleep(2)
        self.color_read_succed = 0
        self.move_status = 0
        self.world_target_mm = None


    # ══ enter/exit 服务 ════════════════════════════════════════════════════════

    def enter_callback(self, request, response):
        self.get_logger().info('收到Enter服务，启动深度数字码垛！')
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
                self.stack_active = True
                self._run_thread = threading.Thread(target=self.run, daemon=True)
                self._run_thread.start()
            except Exception as e:
                self.get_logger().error(f'硬件初始化失败: {e}')
                response.success = False
                response.message = f'硬件初始化失败: {e}'
                return response
        response.success = True
        response.message = '深度数字码垛已启动'
        return response

    def exit_callback(self, request, response):
        self.get_logger().info('收到Exit服务，停止深度数字码垛！')
        if self.active:
            self.active = False
            self.stack_active = False
            close_uart()
            if self._run_thread and self._run_thread.is_alive():
                self._run_thread.join(timeout=3.0)
        response.success = True
        response.message = '深度数字码垛已停止'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DepthNumStackNode()
    exec_ = MultiThreadedExecutor(); exec_.add_node(node)
    try:
        exec_.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.active = False
        node.stack_active = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
