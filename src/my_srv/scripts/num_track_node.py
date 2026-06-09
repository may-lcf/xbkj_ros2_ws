#!/usr/bin/env python3

import cv2
import glob
import os
import numpy as np
import time
import math
import rclpy
import threading
import pupil_apriltags 
from rclpy.node import Node
from sensor_msgs.msg import Image
from ai_edge_litert.interpreter import Interpreter
from cv_bridge import CvBridge
from std_msgs.msg import String, Int32, Bool
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import Trigger
from my_srv.srv import Add  

from z_uart import uart_send_str, setup_uart, close_uart 

MODEL_PATH = os.path.join(os.path.expanduser('~'), 'OpenCV', 'trained2.tflite')   # TFLite模型路径
LABELS_PATH = os.path.join(os.path.expanduser('~'), 'OpenCV', 'labels.txt')          # 数字文件
CONF_THRESHOLD = 0.7                # 置信度阈值
MODEL_INPUT_H = 128                 # 模型输入高度
MODEL_INPUT_W = 128                 # 模型输入宽度
DISPLAY_W = 320                     # 显示宽度
DISPLAY_H = 240                     # 显示高度

class PIDController:
    """PID 控制器"""
    def __init__(self, kp, ki, kd):
        self.Target_val = 0.0
        self.Actual_val = 0.0
        self.err = 0.0
        self.last_error = 0.0
        self.Prev_Error = 0.0
        self.sum_error = 0.0
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def PID_Realize(self, actual_val):
        self.Actual_val = actual_val
        self.err = self.Target_val - self.Actual_val
        self.sum_error += self.err
        output = (self.kp * self.err + 
                 self.ki * self.sum_error + 
                 self.kd * (self.err - self.last_error))
        self.last_error = self.err
        return output
    
class NumTrackNode(Node):
    def __init__(self):
        super().__init__('num_track_node')
        
        # 初始化参数
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_device', '')
        self.declare_parameter('uart_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        # 获取参数
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.camera_device = self.get_parameter('camera_device').get_parameter_value().string_value.strip()
        self.uart_port = self.get_parameter('uart_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.detector = pupil_apriltags.Detector()
        # 全局变量初始化
        self.servo0=1500
        self.servo2=2219
        self.move_x = 160
        self.move_y = 120
        self.block_cx = 0
        self.block_cy = 0
        self.width = 320
        self.hight = 240
        self.find_label = 0

        self.running = True  # 控制程序运行的标志
        self.sorting_active = False  # 追踪是否启动
        self.camera_open = False     # 摄像头是否已打开
        self.uart_open = False       # 串口是否已打开
        self.detections = None
        self.orig_size = None
        self.model_size = None
        self.camera_thread = None
        self.camera_lock = threading.Lock()
        self.cap = None
        self.camera_source = None
        # 追踪颜色模式：None表示不追踪，'red'/'green'/'blue'表示追踪特定颜色
        self.target_num = None
        self.target_detections =None

        self.pid_x = PIDController(kp=0.035, ki=0.0, kd=0.00)
        self.pid_y = PIDController(kp=0.035, ki=0.0, kd=0.00)
        self.interpreter, self.input_det, self.output_det, self.labels = self.load_model_and_labels()

        # ROS2 通信组件
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/num_track/image_raw', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.num_sub = self.create_subscription(String, '/num', self.set_track_num_callback, 10)
        self.enter_srv = self.create_service(Trigger, '/num_track/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/num_track/exit', self.exit_callback)

        # 启动控制线程
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        self.get_logger().info("标签追踪节点已就绪")

    def set_track_num_callback(self, msg):
        """设置追踪颜色的服务回调"""
        self.target_num = msg.data
        self.get_logger().info(f"✅✅✅ 设置追踪颜色: {self.target_num}")

    def resolve_camera_source(self):
        if self.camera_device:
            if os.path.exists(self.camera_device):
                return self.camera_device, f"camera_device={self.camera_device}"
            raise Exception(f"配置的摄像头设备不存在: {self.camera_device}")

        by_id_devices = sorted(glob.glob('/dev/v4l/by-id/*-video-index0'))
        if by_id_devices:
            return by_id_devices[0], f"by-id={by_id_devices[0]}"

        camera_path = f"/dev/video{self.camera_index}"
        if os.path.exists(camera_path):
            return self.camera_index, f"camera_index={self.camera_index} ({camera_path})"

        available_video_devices = sorted(glob.glob('/dev/video*'))
        raise Exception(
            f"未找到可用摄像头。当前配置是 {camera_path}，现有视频设备: {available_video_devices}"
        )

    def initialize_camera(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        self.camera_source, source_desc = self.resolve_camera_source()
        self.get_logger().info(f"准备打开摄像头: {source_desc}")
        self.cap = cv2.VideoCapture(self.camera_source, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise Exception(f"摄像头未打开（{source_desc}）")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.hight)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        time.sleep(0.5)

        for _ in range(10):
            ret, frame = self.cap.read()
            if ret and frame is not None and frame.size > 0:
                self.camera_open = True
                self.get_logger().info(f"摄像头初始化成功: {source_desc}")
                return
            time.sleep(0.1)

        self.cap.release()
        self.cap = None
        self.camera_open = False
        raise Exception(f"摄像头打开成功，但预热阶段无法读取有效帧（{source_desc}）")

    def wait_for_camera_device(self, timeout=15.0):
        """等待USB摄像头设备重新出现（USB断开重连后需要几秒重新枚举）"""
        deadline = time.time() + timeout
        attempt = 0
        while time.time() < deadline and self.running and self.sorting_active:
            attempt += 1
            by_id_devices = sorted(glob.glob('/dev/v4l/by-id/*-video-index0'))
            if by_id_devices:
                self.get_logger().info(f"USB摄像头设备已恢复: {by_id_devices[0]}（等待了{attempt}次）")
                return True
            if self.camera_device and os.path.exists(self.camera_device):
                self.get_logger().info(f"USB摄像头设备已恢复: {self.camera_device}（等待了{attempt}次）")
                return True
            time.sleep(1.0)
        self.get_logger().warn(f"等待USB摄像头设备超时（{timeout}秒）")
        return False

    def try_reinitialize_camera(self):
        try:
            if not self.wait_for_camera_device():
                return False
            time.sleep(0.5)
            self.initialize_camera()
            self.get_logger().info("摄像头重新初始化成功")
            return True
        except Exception as e:
            self.get_logger().error(f"摄像头重新初始化失败: {str(e)}")
            return False

    def enter_callback(self, request, response):
        self.get_logger().info("✅ 收到Enter服务，启动标签追踪并初始化硬件！")
        if not self.sorting_active:
            try:
                # 初始化摄像头
                if not self.camera_open:
                    self.initialize_camera()

                # 初始化串口
                if not self.uart_open:
                    setup_uart(self.baud_rate)
                    self.uart_open = True
                    self.get_logger().info("串口初始化成功")
                    # 发送初始机械臂位置指令（根据实际需求调整）
                    uart_send_str('{#000P1500T1000!#001P1666T1000!#002P2219T1000!#003P0905T1000!#004P1500T1000!}')

                # 启动追踪状态
                self.find_label = 0
                self.running = True
                self.sorting_active = True
                self.servo0=1500
                self.servo2=2219               
                self.move_x = 0
                self.move_y = 120
                self.block_cx = 0
                self.block_cy = 0
                self.detected_color = None  # 记录检测到的颜色
                self.detections = None
                self.target_detections =None
                self.camera_thread = None
                # 启动摄像头线程
                if not self.camera_thread:
                    self.camera_thread = threading.Thread(target=self.camera_processing_loop)
                    self.camera_thread.daemon = True
                    self.camera_thread.start()
                    self.get_logger().info("摄像头处理线程已启动")

            except Exception as e:
                self.get_logger().error(f"硬件初始化失败：{str(e)}")
                response.success = False
                response.message = f"硬件初始化失败：{str(e)}"
                return response

        response.success = True
        response.message = "标签追踪已启动"
        return response

    def exit_callback(self, request, response):
        self.get_logger().info("✅✅ 收到Exit服务，停止标签追踪并关闭硬件！")
        if self.sorting_active:
            try:
                # 停止追踪状态
                self.sorting_active = False
                self.target_num = None  # 同时停止颜色追踪
                
                # 关闭摄像头
                if self.camera_open:
                    if self.cap is not None:
                        self.cap.release()
                    self.cap = None
                    self.camera_open = False
                    self.get_logger().info("摄像头已关闭")

                # 关闭串口
                if self.uart_open:
                    close_uart() 
                    self.uart_open = False
                    self.get_logger().info("串口已关闭")
                # 等待摄像头线程结束
                if self.camera_thread and self.camera_thread.is_alive():
                    self.camera_thread.join(timeout=2.0)
                    self.camera_thread = None
            except Exception as e:
                self.get_logger().error(f"硬件关闭失败：{str(e)}")
                response.success = False
                response.message = f"硬件关闭失败：{str(e)}"
                return response

        response.success = True
        response.message = "标签追踪已停止，硬件已关闭"
        return response

    def load_model_and_labels(self):
        self.interpreter = Interpreter(model_path=MODEL_PATH)
        self.interpreter.allocate_tensors()
        self.input_det = self.interpreter.get_input_details()[0]
        self.output_det = self.interpreter.get_output_details()[0]
        
        with open(LABELS_PATH, "r", encoding="utf-8") as f:
            self.labels = [line.strip() for line in f if line.strip()]
        
        # expected_shape = np.array([1, MODEL_INPUT_H, MODEL_INPUT_W, 3])
        # if not np.array_equal(self.input_det["shape"], expected_shape):
        #     raise ValueError(f"模型输入形状错误！预期：{expected_shape}，实际：{self.input_det['shape']}")
        
        print(f"✅ 模型加载成功：{os.path.exists(MODEL_PATH)}")
        print(f"✅ 输入形状：{self.input_det['shape']}（H×W={MODEL_INPUT_H}×{MODEL_INPUT_W}）")
        print(f"✅ 输出形状：{self.output_det['shape']}")
        print(f"✅ 加载数字：{len(self.labels)}类")
        
        return self.interpreter, self.input_det, self.output_det, self.labels

    def preprocess_frame(self,frame):
        frame_resized = cv2.resize(frame, (MODEL_INPUT_W, MODEL_INPUT_H))
        input_data = frame_resized.astype(np.float32) / 255.0
        input_data = np.expand_dims(input_data, axis=0)
        return input_data

    def postprocess_output(self,raw_output, labels, conf_threshold, model_input_h, model_input_w):
        raw_output = raw_output.squeeze(axis=0)
        detections = [] 
        # 向量化处理
        conf_scores = raw_output[:, 4]  # 置信度分数
        cls_probs = raw_output[:, 5:]   # 类别概率
        
        # 找到每个锚点的最大类别概率和索引
        cls_ids = np.argmax(cls_probs, axis=1)
        cls_confs = np.max(cls_probs, axis=1)
        
        # 计算总置信度 = 对象置信度 * 类别置信度
        total_confs = conf_scores * cls_confs
        
        # 创建有效掩码（置信度高于阈值）
        valid_mask = total_confs > CONF_THRESHOLD
        
        if not np.any(valid_mask):
            return detections
            
        # 获取有效检测的索引
        valid_indices = np.where(valid_mask)[0]
        for anchor_idx in valid_indices:
            # 提取边界框坐标（归一化）
            bx_norm, by_norm, bw_norm, bh_norm = raw_output[anchor_idx, :4]

            
            # 转换为模型输入尺寸的绝对坐标
            bx = bx_norm * 128
            by = by_norm * 128
            bw = bw_norm * 128
            bh = bh_norm * 128
            
            # 计算边界框的左上角和右下角
            x1 = max(0, int(bx - bw / 2))
            y1 = max(0, int(by - bh / 2))
            x2 = min(MODEL_INPUT_W - 1, int(bx + bw / 2))
            y2 = min(MODEL_INPUT_H - 1, int(by + bh / 2))
            
            # 添加到检测结果
            detections.append({
                "class_id": int(cls_ids[anchor_idx]),
                "class_name": self.labels[int(cls_ids[anchor_idx])],
                "confidence": float(total_confs[anchor_idx]),
                "bbox": (x1, y1, x2, y2)  # 模型输入尺寸上的坐标
            })
        
        return detections

    def map_to_display_coords(self, bbox, orig_size, model_size):
        """将模型坐标映射到显示坐标"""
        x1, y1, x2, y2 = bbox
        orig_w, orig_h = orig_size
        model_w, model_h = model_size
        
        # 计算缩放比例
        scale_x = orig_w / model_w
        scale_y = orig_h / model_h
        
        # 映射到显示尺寸
        x1_disp = int(x1 * scale_x)
        y1_disp = int(y1 * scale_y)
        x2_disp = int(x2 * scale_x)
        y2_disp = int(y2 * scale_y)
        
        return (x1_disp, y1_disp, x2_disp, y2_disp)

    def camera_processing_loop(self):
        fail_count = 0
        max_fails = 10
        while self.running and self.sorting_active:
            try:
                if not self.camera_open or self.cap is None or not self.cap.isOpened():
                    self.get_logger().warn("检测到摄像头未就绪，等待USB设备恢复并重新初始化...")
                    if not self.try_reinitialize_camera():
                        time.sleep(3.0)
                        continue

                ret, frame = self.cap.read()
                if not ret or frame is None or frame.size == 0:
                    fail_count += 1
                    self.get_logger().warn(f"无法读取摄像头帧，重试中({fail_count}/{max_fails})...")
                    if fail_count >= max_fails:
                        self.get_logger().warn("摄像头连续读帧失败，准备重新初始化...")
                        if self.cap is not None:
                            self.cap.release()
                            self.cap = None
                        self.camera_open = False
                        fail_count = 0
                    time.sleep(0.1)
                    continue

                fail_count = 0
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.camera_pub.publish(img_msg)
                # 处理帧
                self.process_frame(frame)
                time.sleep(0.001)  # 约30fps

            except Exception as e:
                self.get_logger().error(f"摄像头处理失败: {str(e)}")
                time.sleep(0.1)

    def process_frame(self, frame):   
        frame = cv2.flip(frame, -1)

        orig_h, orig_w = frame.shape[:2]
        self.orig_size = (orig_w, orig_h)
        self.model_size = (MODEL_INPUT_W, MODEL_INPUT_H)
        # # 推理
        input_data = self.preprocess_frame(frame)  
        self.interpreter.set_tensor(self.input_det["index"], input_data)
        self.interpreter.invoke()
        self.raw_output = self.interpreter.get_tensor(self.output_det["index"])
        # 后处理（得到所有检测框）
        self.detections = self.postprocess_output(raw_output=self.raw_output,labels=self.labels,conf_threshold=CONF_THRESHOLD,model_input_h=MODEL_INPUT_H,model_input_w=MODEL_INPUT_W)
        self.target_detections = [det for det in  self.detections if det["class_name"] == self.target_num] 
        if self.target_detections:
            max_conf_det = max(self.target_detections, key=lambda x: x["confidence"])
            print(f"🔍 最大置信度框：类别={max_conf_det['class_name']}，置信度={max_conf_det['confidence']:.4f}")
            x1, y1, x2, y2 = self.map_to_display_coords(max_conf_det["bbox"], self.orig_size, self.model_size)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
            label = f"MAX: {max_conf_det['class_name']}: {max_conf_det['confidence']:.2f}"
            cv2.putText(frame, label, (x1, y1 - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)


        debug_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(debug_img_msg)

    def limit(self, dat, mn, mx):
        """限制数值范围"""
        if dat >= mx:
            return mx
        elif dat <= mn:
            return mn
        return dat   
    def sample_and_control(self):
        """控制逻辑"""
        if not self.sorting_active or not self.uart_open or not self.camera_open:
            return
            
        if self.target_detections:
            max_conf_det = max(self.target_detections, key=lambda x: x["confidence"])
            x1, y1, x2, y2 = self.map_to_display_coords(max_conf_det["bbox"], self.orig_size, self.model_size)
            self.block_cx=(x1+x2)/2
            self.block_cy=(y1+y2)/2
            self.pid_x.Target_val = 160
            self.pid_y.Target_val = 120
            self.servo0 += int(self.pid_x.PID_Realize(self.block_cx))
            self.servo2 -= int(self.pid_y.PID_Realize(self.block_cy))
            self.servo0 = self.limit(self.servo0, 600, 2400)
            self.servo2 = self.limit(self.servo2, 600, 2400)
            # print(f"servo0={self.servo0}====servo2={self.servo2}=====")
            uart_send_str("{{#000P{:0>4d}T0000!#002P{:0>4d}T0000!}}".format(self.servo0, self.servo2))
            
    def control_loop(self):
        """控制线程循环"""
        while self.running:
            try:
                self.sample_and_control()
                time.sleep(0.001)  # 20Hz控制频率
            except Exception as e:
                self.get_logger().error(f"控制循环错误: {str(e)}")
                break
    def destroy_node(self):
        """节点销毁时释放所有资源"""
        self.running = False
        self.sorting_active = False
        # 等待线程结束
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=2)
        # 确保关闭摄像头和串口（即使未调用exit_callback）
        if self.camera_open:
            if self.cap is not None:
                self.cap.release()
            self.cap = None
            self.camera_open = False
        
        if self.uart_open:
            close_uart()  
            self.uart_open = False
        
        self.get_logger().info("标签分拣节点已停止，所有硬件资源已释放")

def main(args=None):
    rclpy.init(args=args)
    
    node = NumTrackNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

