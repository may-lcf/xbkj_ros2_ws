#!/usr/bin/env python3

import cv2
import glob
import numpy as np
# import tensorflow as tf
from ai_edge_litert.interpreter import Interpreter
import time
import threading
import os
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Int32, Bool
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import Trigger
from my_srv.srv import Add  

from z_uart import uart_send_str, setup_uart, close_uart  
from z_move import kinematics_move

MODEL_PATH = os.path.join(os.path.expanduser('~'), 'OpenCV', 'trained2.tflite')   # TFLite模型路径
LABELS_PATH = os.path.join(os.path.expanduser('~'), 'OpenCV', 'labels.txt')          # 数字文件
CONF_THRESHOLD = 0.4                # 置信度阈值
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
    
class NumSortingNode(Node):
    def __init__(self):
        super().__init__('num_sorting_node')
        
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

        # 全局变量初始化
        self.clamp_offset_x = 43        # 如果夹取物体时偏左或偏右，加减此值(偏右-减小，偏左-增大)
        self.clamp_offset_y = 43        # 如果夹取物体时偏远或偏近，加减此值(偏远-减小，偏近-增大)
        self.clamp_offset_z = 20        # 如果夹取物体时偏高或偏低，加减此值(偏高-减小，偏低-增大)
        self.place_offset_x = 56        # 如果放置物体时偏远或偏近，加减此值(偏远-减小，偏近-增大)
        self.place_offset_y = 56        # 如果放置物体时偏左或偏右，加减此值(偏右-减小，偏左-增大)
        
        self.max_conf_det = None
        self.move_x=0
        self.move_y=120
        self.color_state = 255
        self.running = True  # 控制程序运行的标志
        self.spin_calw=1500#机械爪旋转角度
        self.move_status=0#机械臂移动的方式
        self.target_num = None
        self.target_flag = 0
        #中心点
        self.block_cx=0
        self.block_cy=0
        self.block_degress=0
        #用来记录已经抓取到数字
        self.block_cnt=0#记录抓取的物块数量
        self.sort_align_count = 0  # move_status==6 对准帧计数（防止卡限位死锁）
        #抓取计数
        self.capt=0#是否检测到
        self.cap_find=0#检测次数
        self.cap_right=0#是否向右寻找
        self.cap_left=0#是否向左寻找
        self.cap_ok=0#是否连续检测到，防止误测
        self.cap_ok_num = 0
        self.cap_find_ok=0#抓取过程中目标丢失则退回寻找函数
        self.move_ok=0#当退回寻找函数时屏蔽初始位置的传递
        self.color_read_succed=0
        self.bak_cx=0
        self.bak_cy=0
        self.apriltag_flag=0
        self.cap_num=0
        self.block_angle = 0
        self.num = None
        self.apriltag_no_det_count = 0  # apriltag_flag==1 时检测失败帧计数

        # 新增：目标数字集合
        self.target_numbers = {'1', '2', '3'}
        # self.target_numbers = set()  # 要分拣的数字集合
        self.current_target_num = None  # 当前正在寻找的数字
        self.all_numbers_completed = False  # 所有数字是否分拣完成

        self.running = True  # 控制程序运行的标志
        self.sorting_active = False  # 分拣是否启动
        self.camera_open = False     # 摄像头是否已打开
        self.uart_open = False       # 串口是否已打开
        self.width = 320
        self.hight = 240

        self.pid_x = PIDController(kp=0.06, ki=0.0, kd=0.00)
        self.pid_y = PIDController(kp=0.06, ki=0.0, kd=0.00)

        self.interpreter, self.input_det, self.output_det, self.labels = self.load_model_and_labels()

        # ROS2 通信组件
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/num_sorting/image_raw', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # 新增：分拣指令话题订阅
        self.sort_command_sub = self.create_subscription(
            String, 
            '/num_sorting/command', 
            self.sort_command_callback, 
            10
        )

        self.enter_srv = self.create_service(Trigger, '/num_sorting/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/num_sorting/exit', self.exit_callback)
        
        # 启动控制线程
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        self.camera_thread = None
        self.camera_lock = threading.Lock()
        self.cap = None
        self.camera_source = None
        
        self.get_logger().info("数字分拣节点已就绪，等待分拣指令")

    def sort_command_callback(self, msg):
        self.get_logger().info("✅✅✅ 收到cmd指令")
        """处理分拣指令话题回调"""
        try:
            command = msg.data.strip()
            self.get_logger().info(f"收到分拣指令: {command}")
            
            if command.startswith("sort_num"):
                # 解析数字
                numbers_str = command.replace("sort_num", "").strip()
                if numbers_str == "all":
                    # 分拣所有数字
                    self.target_numbers = {'1', '2', '3'}
                    self.all_numbers_completed = False
                    self.get_logger().info("设置分拣所有数字: 1, 2, 3")
                else:
                    # 解析单个或多个数字
                    self.target_numbers.clear()
                    for char in numbers_str:
                        if char.isdigit() and 1 <= int(char) <= 3:
                            self.target_numbers.add(char)
                    
                    if not self.target_numbers:
                        self.get_logger().warning("未提供有效的分拣数字（有效数字：1, 2, 3）")
                        return
                    
                    self.all_numbers_completed = False
                    self.get_logger().info(f"设置分拣数字: {sorted(self.target_numbers)}")
                
                # 重置当前目标数字
                self.current_target_num = None
                self.target_flag = 0
                self.capt = 0
                self.cap_find = 0
                
                
            elif command == "stop":
                # 停止分拣
                self.target_numbers.clear()
                self.current_target_num = None
                # self.all_numbers_completed = True
                self.get_logger().info("停止分拣")
                    
        except Exception as e:
            self.get_logger().error(f"处理分拣指令失败: {str(e)}")

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
        self.get_logger().info("✅ 收到Enter服务，启动数字分拣并初始化硬件！")
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
                    time.sleep(1)
                
                # 关键变量初始化
                self.get_logger().info("重置分拣状态变量...")
                self.running = True
                self.max_conf_det = None
                self.move_x=0
                self.move_y=120
                self.target_num = None
                self.current_target_num = None
                self.target_flag = 0
                self.capt = 0                  # 是否检测到目标（0=未检测，1=检测成功）
                self.cap_find = 0              # Apriltag检测次数（用于寻找目标）
                self.cap_right = 0             # 是否向右寻找目标
                self.cap_left = 0              # 是否向左寻找目标
                self.cap_ok = 0                # 连续检测计数（防止误测）
                self.cap_find_ok = 0           # 抓取过程中目标丢失时回退寻找
                self.move_ok = 0               # 传递夹爪坐标的标志
                self.block_cx = 0              # 物块中心X坐标（相机坐标系）
                self.block_cy = 0              # 物块中心Y坐标（相机坐标系）
                self.color_read_succed = 0     # 颜色识别成功标志（0=未识别，1=成功）
                self.apriltag_flag = 0         # Apriltag识别模式（0=识别数字，1=识别颜色）
                self.move_status = 0           # 机械臂当前阶段（0=初始，1~9=分拣步骤）
                self.all_numbers_completed = False  # 重置完成标志
                self.apriltag_no_det_count = 0 # 重置检测失败计数
                self.sort_align_count = 0      # 重置分拣放置对准计数
                self.camera_thread = None
                
                # 启动分拣状态
                self.sorting_active = True
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
        response.message = "数字分拣已启动"
        return response

    def exit_callback(self, request, response):
        self.get_logger().info("✅✅ 收到Exit服务，停止数字分拣并关闭硬件！")
        if self.sorting_active:
            try:
                # 停止分拣状态
                self.sorting_active = False
                
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
        response.message = "数字分拣已停止，硬件已关闭"
        return response

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
                self.process_frame(frame)
                time.sleep(0.06)  # 约30fps

            except Exception as e:
                self.get_logger().error(f"摄像头处理失败: {str(e)}")
                time.sleep(0.1)

    def calculate_angle_edge_based(self,corners):
        """
        基于边向量计算旋转角度
        """
        # 计算所有边向量
        edges = []
        for i in range(4):
            edge = corners[(i+1) % 4] - corners[i]
            edges.append(edge)
        # 找到最长的边作为主要方向
        edge_lengths = [np.linalg.norm(edge) for edge in edges]
        longest_edge_idx = np.argmax(edge_lengths)
        main_edge = edges[longest_edge_idx]
        # 计算与水平轴的夹角
        angle_rad = math.atan2(main_edge[1], main_edge[0])
        angle_deg = math.degrees(angle_rad)
        # 标准化角度到 [-90, 90] 范围
        if angle_deg > 45:
            angle_deg -= 90
        if angle_deg < -45:
            angle_deg += 90

        if abs(angle_deg)<=10:
            angle_deg=0
        return -angle_deg

    def detect_color(self, mask):
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts) > 0:
            best = None
            best_area = -1
            for cnt in cnts:
                rect = cv2.minAreaRect(cnt)
                (c_x, c_y), (c_w, c_h), _ = rect
                # 判断是否接近正方形（长宽比接近 1）
                area = cv2.contourArea(cnt)
                side_min = min(c_h, c_w)
                side_max = max(c_h, c_w)
                is_square = side_min > 0 and (side_min / side_max) >= 0.85
                if not is_square:
                    continue
                cond = 25 < c_h < 150 and 25 < c_w < 150 and area > 1200
                if cond and area > best_area:
                    best_area = area
                    best = (area, (c_x, c_y), rect)
            if best is not None:
                return best
        return 0, (0, 0), None
    
    def limit(self, dat, mn, mx):
        """限制数值范围"""
        if dat >= mx:
            return mx
        elif dat <= mn:
            return mn
        return dat

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

    def find_largest_square_near_center(self,image, min_area):
        # 1. 预处理：灰度化→高斯模糊→Canny边缘检测
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 0)
        edges = cv2.Canny(blurred, 50, 150)
        # 2. 形态学闭运算：连接断裂的边缘
        kernel = np.ones((11, 11), np.uint8)
        closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        # 3. 查找轮廓
        contours, _ = cv2.findContours(closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 4. 计算屏幕中心点
        screen_center = (image.shape[1] // 2, image.shape[0] // 2)
        # 5. 筛选正方形轮廓
        squares = []
        for cnt in contours:
            perimeter = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon=0.02 * perimeter, closed=True)
            # 判断是否为凸四边形
            if len(approx) == 4 and cv2.isContourConvex(approx):
                area = cv2.contourArea(approx)
                if area >= min_area:
                    # 计算矩形的长宽比来判断是否为正方形
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h
                    
                    # 判断是否为正方形（长宽比接近1）
                    if 0.1 <= aspect_ratio <= 1.9:  # 允许20%的容差
                        # 使用minAreaRect获取旋转矩形信息
                        rect = cv2.minAreaRect(cnt)
                        center, size, angle = rect
                        # 计算到屏幕中心的距离
                        distance = np.sqrt((center[0] - screen_center[0])**2 + 
                                        (center[1] - screen_center[1])**2)
                        # 获取四个顶点
                        box = cv2.boxPoints(rect)
                        box = np.intp(box) 
                        squares.append({
                            'center': center,
                            'size': min(size),  # 正方形边长（取较小值）
                            'angle': angle,
                            'vertices': box,
                            'distance_to_center': distance,
                            'area': area
                        })
        
        if not squares:
            return 0.1
        # 6. 找到离屏幕中心最近的最大正方形
        # 先按面积排序，找到较大的正方形
        squares_sorted_by_area = sorted(squares, key=lambda x: x['area'], reverse=True)
        # 在前N个最大正方形中找离中心最近的
        top_n = min(3, len(squares_sorted_by_area))  # 考虑前3个最大的
        candidate_squares = squares_sorted_by_area[:top_n]
        # 选择离中心最近的那个
        nearest_square = min(candidate_squares, key=lambda x: x['distance_to_center'])
        angle_deg = nearest_square['angle']
        if angle_deg <= 10 or angle_deg >= 80:
            angle_deg = 0
        elif angle_deg < 45 and angle_deg > 10:
            angle_deg = -angle
        elif angle_deg > 45 and angle_deg < 80:
            angle_deg = 90 - angle_deg
        return angle_deg

    def calculate_rectangle_angle(self, rect):
        """计算minAreaRect矩形倾角，归一化到[-45, 45]"""
        _, _, angle = rect
        # 兼容OpenCV各版本角度规范，统一归一化到 [-45, 45]
        if angle > 45:
            angle -= 90
        elif angle < -45:
            angle += 90
        if abs(angle) <= 5:
            angle = 0.0
        return angle

    def compute_angle_from_black_frame(self, frame, x1, y1, x2, y2):
        """
        通过裁剪bbox区域检测数字块白色正面的四角点来计算旋转角度。
        步骤：裁剪扩展bbox → 提取亮色区域（白色块面）→
              approxPolyDP找四角点 → 计算各边倾角 → 取最接近水平的偏角
        """
        h, w = frame.shape[:2]
        bbox_w = x2 - x1
        bbox_h = y2 - y1
        # 扩展裁剪区域（padding = bbox边长的1/3）
        pad = max(bbox_w, bbox_h) // 3
        rx1 = max(0, x1 - pad)
        ry1 = max(0, y1 - pad)
        rx2 = min(w, x2 + pad)
        ry2 = min(h, y2 + pad)
        crop = frame[ry1:ry2, rx1:rx2].copy()
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        # 提取亮色区域（数字块的白色正面在深色桌面上对比明显）
        _, mask = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            cv2.putText(frame, "no bright cnt", (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            return 0.0
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 200:
            cv2.putText(frame, "cnt too small", (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            return 0.0
        angle = 0.0
        peri = cv2.arcLength(largest, True)
        approx = None
        for eps_f in [0.05, 0.04, 0.06, 0.08]:
            approx = cv2.approxPolyDP(largest, eps_f * peri, True)
            if len(approx) == 4:
                break
        if approx is not None and len(approx) == 4:
            pts = approx.reshape(4, 2).astype(np.float32)
            edge_angles = []
            for i in range(4):
                p1 = pts[i]
                p2 = pts[(i + 1) % 4]
                dx = float(p2[0] - p1[0])
                dy = float(p2[1] - p1[1])
                ea = math.degrees(math.atan2(dy, dx))
                # 折叠到 [-45, 45]
                while ea > 45:
                    ea -= 90
                while ea < -45:
                    ea += 90
                edge_angles.append(ea)
            # 取绝对值最小的（最接近水平方向的边的偏角）
            angle = min(edge_angles, key=abs)
            # 可视化：画出检测到的四边形
            for i in range(4):
                p1 = (int(approx[i][0][0]) + rx1, int(approx[i][0][1]) + ry1)
                p2 = (int(approx[(i + 1) % 4][0][0]) + rx1, int(approx[(i + 1) % 4][0][1]) + ry1)
                cv2.line(frame, p1, p2, (0, 255, 0), 2)
            for pt in approx:
                cv2.circle(frame, (int(pt[0][0]) + rx1, int(pt[0][1]) + ry1), 4, (0, 255, 0), -1)
        else:
            # fallback: minAreaRect（无法得到4角点时）
            rect = cv2.minAreaRect(largest)
            _, _, raw_angle = rect
            if raw_angle > 45:
                raw_angle -= 90
            elif raw_angle < -45:
                raw_angle += 90
            angle = raw_angle if abs(raw_angle) > 5 else 0.0
            box = cv2.boxPoints(rect)
            box[:, 0] += rx1
            box[:, 1] += ry1
            box = np.intp(box)
            cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
        cv2.putText(frame, f"angle:{angle:.1f}", (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return angle

    def get_target_detections(self, detections):
        """根据目标数字集合过滤检测结果"""
        # if not self.target_numbers or self.all_numbers_completed:
        if not self.target_numbers :
            return []  # 如果没有设置目标数字或已完成，返回空列表
        
        # 过滤出目标数字的检测结果
        target_detections = [det for det in detections if det["class_name"] in self.target_numbers]
        
        # 如果当前有特定目标数字，优先选择该数字
        if self.current_target_num and self.current_target_num in self.target_numbers:
            current_target_dets = [det for det in target_detections if det["class_name"] == self.current_target_num]
            if current_target_dets:
                return current_target_dets
        
        return target_detections

    def select_next_target(self, detections):
        """选择下一个要分拣的目标数字"""
        # if not self.target_numbers or self.all_numbers_completed:
        if not self.target_numbers:
            return None
        
        # 过滤出目标数字的检测结果
        valid_detections = [det for det in detections if det["class_name"] in self.target_numbers]
        
        if not valid_detections:
            return None
        
        # 选择置信度最高的检测结果
        best_det = max(valid_detections, key=lambda x: x["confidence"])
        return best_det["class_name"]
    
    def check_all_numbers_completed(self):
        """检查是否所有目标数字都已分拣完成"""
        if not self.target_numbers:
            return True
        return False
 
    def process_frame(self, frame):   
        frame = cv2.flip(frame, -1)

        orig_h, orig_w = frame.shape[:2]
        orig_size = (orig_w, orig_h)
        model_size = (MODEL_INPUT_W, MODEL_INPUT_H)
        # # 推理
        input_data = self.preprocess_frame(frame)  
        self.interpreter.set_tensor(self.input_det["index"], input_data)
        self.interpreter.invoke()
        self.raw_output = self.interpreter.get_tensor(self.output_det["index"])
        # 后处理（得到所有检测框）
        detections = self.postprocess_output(raw_output=self.raw_output,labels=self.labels,conf_threshold=CONF_THRESHOLD,model_input_h=MODEL_INPUT_H,model_input_w=MODEL_INPUT_W)
        
        # 使用新的目标检测过滤方法
        target_detections = self.get_target_detections(detections)
        # 如果没有当前目标数字，选择下一个
        if self.current_target_num is None and target_detections and not self.all_numbers_completed:
            self.current_target_num = self.select_next_target(detections)
            if self.current_target_num:
                self.get_logger().info(f"开始分拣数字: {self.current_target_num}")

        
        if self.target_flag == 0 and self.current_target_num and not self.all_numbers_completed:    
            if target_detections:
                self.target_flag = 1
                
        if self.apriltag_flag==0 and not self.all_numbers_completed:
            # 预处理
            if self.cap_find<20 and self.capt==0:
                if self.cap_find==0:
                    kinematics_move(0,120,50,1500)
                    time.sleep(1.5)
                if target_detections:
                    max_conf_det = max(target_detections, key=lambda x: x["confidence"])
                    print(f"🔍 最大置信度框：类别={max_conf_det['class_name']}，置信度={max_conf_det['confidence']:.4f}")
                    self.cap_ok_num=self.cap_ok#上一次的cap_ok
                    self.cap_ok=self.cap_find
                    print(' cap_ok_num'+str(self.cap_ok_num))
                    if (self.cap_ok-self.cap_ok_num)==1:#连续两次检测到才判定为检测成功
                        self.capt=1
                self.cap_find+=1
                self.cap_right=1     
            elif self.cap_right==1 and self.cap_find>19 and self.cap_find<40 and self.capt==0:
                if self.cap_find<21:
                    kinematics_move(35,140,50,1500)
                    time.sleep(1.5)
                if target_detections:
                    max_conf_det = max(target_detections, key=lambda x: x["confidence"])
                    print(f"🔍 最大置信度框：类别={max_conf_det['class_name']}，置信度={max_conf_det['confidence']:.4f}")
                    self.cap_ok_num=self.cap_ok#上一次的cap_ok
                    self.cap_ok=self.cap_find
                    print(' cap_ok_num'+str(self.cap_ok_num))
                    if (self.cap_ok-self.cap_ok_num)==1:#连续两次检测到才判定为检测成功
                        self.capt=1
                        self.move_ok+=1
                    if self.move_ok==1:#传递夹爪坐标
                        self.move_x=35
                        self.move_y=140
                self.cap_left=1
                self.cap_find=self.cap_find+1
            elif self.cap_left==1 and self.capt==0 and self.cap_find>39:
                if self.cap_find<41:
                    kinematics_move(-35,140,50,1500)
                    time.sleep(1.5)
                if target_detections:
                    max_conf_det = max(target_detections, key=lambda x: x["confidence"])
                    print(f"🔍 最大置信度框：类别={max_conf_det['class_name']}，置信度={max_conf_det['confidence']:.4f}")
                    self.cap_ok_num=self.cap_ok#上一次的cap_ok
                    self.cap_ok=self.cap_find
                    print(' cap_ok_num'+str(self.cap_ok_num))
                    if (self.cap_ok-self.cap_ok_num)==1:#连续两次检测到才判定为检测成功
                        self.capt=1
                        self.move_ok+=1
                    if self.move_ok==1:#传递夹爪坐标
                        self.move_x=-35
                        self.move_y=140
                self.cap_right=1
                if self.cap_find<60:
                    self.cap_find=self.cap_find+1
                else :
                    self.cap_find=0
            if self.capt==1:
                self.cap_find_ok=0#抓取过程中目标丢失则退回寻找函数
                if target_detections:
                    max_conf_det = max(target_detections, key=lambda x: x["confidence"])
                    print(f"🔍 最大置信度框：类别={max_conf_det['class_name']}，置信度={max_conf_det['confidence']:.4f}")
                    x1, y1, x2, y2 = self.map_to_display_coords(max_conf_det["bbox"], orig_size, model_size)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
                    label = f"MAX: {max_conf_det['class_name']}: {max_conf_det['confidence']:.2f}"
                    cv2.putText(frame, label, (x1, y1 - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
                    self.block_angle = self.compute_angle_from_black_frame(frame, x1, y1, x2, y2)
                    self.block_cx=(x1+x2)/2
                    self.block_cy=(y1+y2)/2
                    self.num=max_conf_det['class_name']
                    self.color_read_succed=1
                    self.capt=1

        elif self.apriltag_flag==1 and not self.all_numbers_completed:
            # 分拣区对准阶段只检测画面上面三分之二，避免误识别夹爪上夹持的物体
            two_thirds_h = MODEL_INPUT_H * 2 // 3
            detections = [det for det in detections
                          if (det["bbox"][1] + det["bbox"][3]) / 2 < two_thirds_h]
            # 筛选出类别为 self.num 的检测结果
            target_detections = [det for det in detections if det["class_name"] == self.num]
            if target_detections:
                max_conf_det = max(target_detections, key=lambda x: x["confidence"])
                print(f"🔍 最大置信度框：类别={max_conf_det['class_name']}，置信度={max_conf_det['confidence']:.4f}")
                x1, y1, x2, y2 = self.map_to_display_coords(max_conf_det["bbox"], orig_size, model_size)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
                label = f"MAX: {max_conf_det['class_name']}: {max_conf_det['confidence']:.2f}"
                cv2.putText(frame, label, (x1, y1 - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
                self.block_cx=(x1+x2)/2
                self.block_cy=(y1+y2)/2
                self.color_read_succed=1
                self.apriltag_no_det_count=0
            else:
                self.apriltag_no_det_count += 1
                if self.apriltag_no_det_count >= 15:  # 约0.9秒检测不到则超时
                    self.get_logger().warn(f"分拣区连续{self.apriltag_no_det_count}帧未检测到，跳过对准直接放置")
                    self.apriltag_no_det_count = 0
                    self.move_status = 7
                    self.color_read_succed = 1

        debug_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(debug_img_msg)

        #************************运动机械臂**********************************
        if self.color_read_succed == 1 and self.sorting_active and not self.all_numbers_completed:
            if self.move_status == 1:
                self.move_status = 2
                # kinematics_move(self.move_x, self.move_y, 50, 100)
                time.sleep(0.1)
                self.block_cx=self.block_cy=0
                # block_angle 已由 capt==1 阶段的 compute_angle_from_black_frame 实时更新，直接使用
                self.get_logger().info(f"黑框计算倾角: {self.block_angle:.1f}°")
                self.spin_calw = 1500
                self.spin_calw -= self.block_angle * 7.4
                self.spin_calw = int(self.spin_calw)
                self.spin_calw = self.limit(self.spin_calw, 1167, 1833)
                # print(f"spin_calw:={self.spin_calw}°self.move_status={self.move_status}")
                for i in range(5):
                    uart_send_str("#4P{:0^4}T1000!".format(self.spin_calw))
                    time.sleep(0.2)
                l = math.sqrt(self.move_x**2 + self.move_y**2)
                sin = self.move_y / l
                cos = self.move_x / l
                self.move_x = int((l+self.clamp_offset_x) * cos) 
                self.move_y = int((l+self.clamp_offset_y) * sin)
                for i in range(1):
                    kinematics_move(self.move_x, self.move_y, 60, 1000)
                    time.sleep(1)
                    uart_send_str("#005P1000T1000!")
                    time.sleep(1)
                    self.move_status = 2

            elif self.move_status == 2:
                self.move_status = 3
                for i in range(1):
                    kinematics_move(self.move_x, self.move_y - 6, self.clamp_offset_z, 1000)
                    time.sleep(1)

            elif self.move_status==3:
                self.move_status=4
                for i in range(1):
                    uart_send_str("#005P1700T1000!\n")
                    time.sleep(1)

            elif self.move_status == 4:
                self.move_status = 5
                for i in range(1):
                    kinematics_move(self.move_x, self.move_y, 150, 1000)
                    time.sleep(1)

            elif self.move_status == 5:
                self.apriltag_flag=1
                if self.num == '1':
                    self.move_x = 145
                    self.move_y = 60
                elif self.num == '2':
                    self.move_x = 130
                    self.move_y = 20
                elif self.num == '3':
                    self.move_x = 130
                    self.move_y = -20
                for i in range(1):
                    kinematics_move(self.move_x, self.move_y, 120, 1000)
                    time.sleep(1)
                    uart_send_str("#004P1500T0500!")
                    time.sleep(0.5)
                for i in range(1):
                    kinematics_move(self.move_x, self.move_y, 60, 1000)
                    time.sleep(2.5)
                self.color_read_succed = 0
                self.move_status = 6

            elif self.move_status == 7:
                self.move_status = 8
                l = math.sqrt(self.move_x**2 + self.move_y**2)
                sin = self.move_y / l
                cos = self.move_x / l
                self.move_x = int((l+self.place_offset_x) * cos)
                self.move_y = int((l+self.place_offset_y) * sin)
                for i in range(1):
                    kinematics_move(self.move_x, self.move_y, 60, 1000)
                    time.sleep(1)
                for i in range(1):
                    kinematics_move(self.move_x, self.move_y, 15, 1000)
                    time.sleep(1)

            elif self.move_status == 8:
                self.move_status = 9
                for i in range(1):
                    uart_send_str("#005P1200T1000!")
                    time.sleep(1)
                for i in range(1):
                    kinematics_move(self.move_x, self.move_y, 70, 1000)
                    time.sleep(1)

            elif self.move_status == 9:
                # 完成一个数字的分拣
                self.block_cx=self.block_cy=0
                self.move_x = 0
                self.move_y = 120
                self.block_cx = self.block_cy = 0
                for i in range(1):
                    kinematics_move(self.move_x, self.move_y, 70, 1000)
                    time.sleep(1)  
                self.apriltag_flag=0
                self.color_read_succed = 0
                self.move_status = 0
                for i in range(1):
                    kinematics_move(self.move_x,self.move_y,50,1000)
                    time.sleep(1.5)
                
                # 从目标集合中移除已分拣的数字
                if self.num in self.target_numbers:
                    self.target_numbers.remove(self.num)
                    self.get_logger().info(f"数字 {self.num} 分拣完成，剩余目标: {self.target_numbers}")
                
                # 检查是否所有数字都已完成
                if not self.target_numbers:
                    # self.all_numbers_completed = True
                    self.get_logger().info("所有指定数字分拣完成！")
                    
                
                # 重置状态，准备分拣下一个数字
                self.current_target_num = None
                self.target_flag = 0
                self.capt=0
                self.cap_find=0
                self.cap_right=0
                self.cap_left=0
                self.cap_ok=0
                self.cap_find_ok=0
                self.apriltag_no_det_count=0

    def sample_and_control(self):
        """控制逻辑（仅在分拣激活且硬件就绪时执行）"""
        # if not self.sorting_active or not self.camera_open or not self.uart_open or self.all_numbers_completed:
        if not self.sorting_active or not self.camera_open or not self.uart_open :
            return
        if self.color_read_succed == 1:
            if self.move_status == 0:
                self.pid_x.Target_val = 160
                self.pid_y.Target_val = 120
                self.move_x -= self.pid_x.PID_Realize(self.block_cx)
                self.move_y += self.pid_y.PID_Realize(self.block_cy)
                self.move_x = self.limit(self.move_x, -150, 150)
                self.move_y = self.limit(self.move_y, -50, 250)
                # print(f"block_cx={self.block_cx},move_x={self.move_x}===block_cy={self.block_cy},move_y={self.move_y}")
                kinematics_move(self.move_x, self.move_y, 70, 100)
                self.color_read_succed = 0
                if abs(self.block_cy-120)<=4 and abs(self.block_cx-160)<=5:
                    self.move_status=1
                    # print("self.move_status=1")
                    self.apriltag_flag=255
                    self.color_read_succed=1

            elif self.move_status == 6:
                self.pid_x.Target_val = 160
                self.pid_y.Target_val = 120
                self.move_y += self.pid_x.PID_Realize(self.block_cx)
                self.move_x += self.pid_y.PID_Realize(self.block_cy)
                self.move_x = self.limit(self.move_x, -170, 180)
                self.move_y = self.limit(self.move_y, -50, 250)
                # print(f"block_cx={self.block_cx},move_x={self.move_x}===block_cy={self.block_cy},move_y={self.move_y}")
                kinematics_move(self.move_x, self.move_y, 70, 100)
                self.color_read_succed=0
                self.sort_align_count += 1
                if abs(self.block_cy - 120) <= 25 and abs(self.block_cx - 160) <= 25:
                    self.sort_align_count = 0
                    self.move_status = 7
                    self.color_read_succed = 1
                elif self.sort_align_count >= 150:  # 约3秒未对准，强制推进
                    self.get_logger().warn(
                        f"分拣放置对准超时（{self.sort_align_count}帧），强制推进，当前位置: x={self.move_x}, y={self.move_y}"
                    )
                    self.sort_align_count = 0
                    self.move_status = 7
                    self.color_read_succed = 1

    def control_loop(self):
        """控制线程循环（根据sorting_active标志决定是否执行分拣）"""
        while self.running:
            try:
                self.sample_and_control()
                time.sleep(0.06)    
            except Exception as e:
                self.get_logger().error(f"控制循环错误: {str(e)}")
                break
    
    def destroy_node(self):
        """节点销毁时释放所有资源"""
        self.running = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2)
        # 等待摄像头线程结束
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
        
        self.get_logger().info("数字分拣节点已停止，所有硬件资源已释放")

def main(args=None):
    rclpy.init(args=args)
    
    node = NumSortingNode()
    
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