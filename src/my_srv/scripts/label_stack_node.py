#!/usr/bin/env python3
import cv2
import glob
import os
import numpy as np
import time
import threading
import math
import rclpy
import pupil_apriltags 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import Trigger
from my_srv.srv import Add  

from z_uart import uart_send_str, setup_uart, close_uart  
from z_move import kinematics_move

_DATA_DIR = os.path.expanduser('~/ros2_ws')
loaded_red = []
loaded_green = []
loaded_blue = []
with open(os.path.join(_DATA_DIR, 'red.txt'), 'r') as f_red:
    for line in f_red:
        line = line.strip()  # 去除首尾空白（如换行符）
        if not line:  # 跳过空行
            continue
        # 将行分割成多个数字字符串（比如['10','20.5','30']）
        num_strs = line.split()  
        for s in num_strs:
            try:
                loaded_red.append(int(s))  # 先试转整数
            except ValueError:
                loaded_red.append(float(s))  # 失败则转浮点数
redlow1, redlow2, redlow3, redhigh1, redhigh2, redhigh3 = loaded_red

with open(os.path.join(_DATA_DIR, 'blue.txt'), 'r') as f_blue:
    for line in f_blue:
        line = line.strip()  # 去除首尾空白（如换行符）
        if not line:  # 跳过空行
            continue
        # 将行分割成多个数字字符串（比如['10','20.5','30']）
        num_strs = line.split()  
        for s in num_strs:
            try:
                loaded_blue.append(int(s))  # 先试转整数
            except ValueError:
                loaded_blue.append(float(s))  # 失败则转浮点数
bluelow1, bluelow2, bluelow3, bluehigh1, bluehigh2, bluehigh3  = loaded_blue

with open(os.path.join(_DATA_DIR, 'green.txt'), 'r') as f_green:
    for line in f_green:
        line = line.strip()  # 去除首尾空白（如换行符）
        if not line:  # 跳过空行
            continue
        # 将行分割成多个数字字符串（比如['10','20.5','30']）
        num_strs = line.split()  
        for s in num_strs:
            try:
                loaded_green.append(int(s))  # 先试转整数
            except ValueError:
                loaded_green.append(float(s))  # 失败则转浮点数
greenlow1, greenlow2, greenlow3, greenhigh1, greenhigh2, greenhigh3  = loaded_green


class PIDController:
    """PID 控制器（保持原有逻辑）"""
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

class LabelrStackNode(Node):
    def __init__(self):
        super().__init__('label_stack_node')
        
        # 初始化参数（保持原有参数配置）
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_device', '')
        self.declare_parameter('uart_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        # 获取参数（保持原有逻辑）
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.camera_device = self.get_parameter('camera_device').get_parameter_value().string_value.strip()
        self.uart_port = self.get_parameter('uart_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.detector = pupil_apriltags.Detector()
        # 全局变量初始化
        self.clamp_offset_x = 46        # 如果夹取物体时偏左或偏右，加减此值(偏右-减小，偏左-增大)
        self.clamp_offset_y = 46        # 如果夹取物体时偏远或偏近，加减此值(偏远-减小，偏近-增大)
        self.clamp_offset_z = 20        # 如果夹取物体时偏高或偏低，加减此值(偏高-减小，偏低-增大)
        self.place_offset_x = 65        # 如果放置物体时偏远或偏近，加减此值(偏远-减小，偏近-增大)
        self.place_offset_y = 65        # 如果放置物体时偏左或偏右，加减此值(偏右-减小，偏左-增大)
        self.stack_height_one = 10      # 码垛第一层高度（数值越大越高）
        self.stack_height_two = 46      # 码垛第二层高度（数值越大越高）
        self.stack_height_three = 70    # 码垛第三层高度（数值越大越高）
        
        self.move_x=0
        self.move_y=120
        self.spin_calw=1500#机械爪旋转角度
        self.move_status=0#机械臂移动的方式
        self.block_cx=0
        self.block_cy=0
        #用来记录已经抓取到标签
        self.mark_flag=255#判断是需要识别颜色基点基点还是apriltags码
        self.block_cnt=0#记录抓取的物块数量

        #抓取计数
        self.capt=0#是否检测到
        self.cap_find=0#检测次数
        self.cap_right=0#是否向右寻找
        self.cap_left=0#是否向左寻找
        self.cap_ok=0#是否连续检测到，防止误测
        self.cap_find_ok=0#抓取过程中目标丢失则退回寻找函数
        self.move_ok=0#当退回寻找函数时屏蔽初始位置的传递
        self.search_hit_count = 0  # 当前搜索位置累计检测命中次数
        self.color_read_succed=0
        self.block_angle = 0
        self.bak_cx=0
        self.bak_cy=0
        self.green_ok_cnt=0#连续检测到绿色基准点在中心区域的帧数
        self.stack_pid_attempt = 0  # 堆放点对中尝试次数（超限后强制进入堆放）
        self.current_stack_id = 1  # 当前要码垵的标签ID，按1→2→3顺序
        self.running = True  # 控制程序运行的标志
        self.stack_active = False  # 码垛是否启动
        self.camera_open = False     # 摄像头是否已打开
        self.uart_open = False       # 串口是否已打开

        # Lab颜色阈值（使用加载的值）
        self.lower_red = np.array([redlow1, redlow2, redlow3], dtype=np.uint8)
        self.upper_red = np.array([redhigh1, redhigh2, redhigh3], dtype=np.uint8)
        self.lower_blue = np.array([bluelow1, bluelow2, bluelow3], dtype=np.uint8)
        self.upper_blue = np.array([bluehigh1, bluehigh2, bluehigh3], dtype=np.uint8)
        self.lower_green = np.array([greenlow1, greenlow2, greenlow3], dtype=np.uint8)
        self.upper_green = np.array([greenhigh1, greenhigh2, greenhigh3], dtype=np.uint8)

        self.width = 320
        self.hight = 240
        self.pid_x = PIDController(kp=0.06, ki=0.0, kd=0.00)
        self.pid_y = PIDController(kp=0.06, ki=0.0, kd=0.00)
        # ROS2 通信组件
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/label_stack/image_raw', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.add_service = self.create_service(
            srv_type=Add,              
            srv_name="/Add",           
            callback=self.Color_callback  
        )
        self.enter_srv = self.create_service(Trigger, '/label_stack/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/label_stack/exit', self.exit_callback)
        # 启动控制线程
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        self.camera_thread = None
        self.camera_lock = threading.Lock()
        self.cap = None
        self.camera_source = None
        self.get_logger().info("标签码垛节点已就绪，等待Enter服务启动码垛")
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
        while time.time() < deadline and self.running and self.stack_active:
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
        self.get_logger().info("✅ 收到Enter服务，启动标签码垓并初始化硬件！")
        if not self.stack_active:
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
                self.get_logger().info("重置码垛状态变量...")
                self.running = True
                self.move_x=0
                self.move_y=120
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
                self.apriltag_flag = 0         # Apriltag识别模式（0=识别标签，1=识别颜色）
                self.move_status = 0           # 机械臂当前阶段（0=初始，1~9=码垛步骤）       
                self.mark_flag = 255           # 码垛点标志（255=未到达）
                self.block_cnt = 0
                self.green_ok_cnt = 0          # 重置绿色基准点对中连续帧计数
                self.stack_pid_attempt = 0     # 重置堆放点对中尝试次数
                self.current_stack_id = 1      # 重置为从标签ID=1开始码垵
                self.camera_thread = None
                # 启动码垛状态
                self.stack_active = True
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
        response.message = "标签码垛已启动"
        return response
    def Color_callback(self, request, response):
        try:
            color = request.color
            
            low_thresh = np.array([request.low_h, request.low_s, request.low_v], dtype=np.uint8)
            high_thresh = np.array([request.high_h, request.high_s, request.high_v], dtype=np.uint8)
            
            if color == 'red':
                self.lower_red = low_thresh
                self.upper_red = high_thresh
                self.get_logger().info(f"修改红色Lab阈值：低={low_thresh}，高={high_thresh}")
            elif color == 'blue':
                self.lower_blue = low_thresh
                self.upper_blue = high_thresh
                self.get_logger().info(f"修改蓝色Lab阈值：低={low_thresh}，高={high_thresh}")
            elif color == 'green':
                self.lower_green = low_thresh
                self.upper_green = high_thresh
                self.get_logger().info(f"修改绿色Lab阈值：低={low_thresh}，高={high_thresh}")
            else:
                raise ValueError("颜色类型错误：仅支持red/blue/green")
                
            response.success = True
            response.message = "Lab阈值修改成功"
            return response
        except Exception as e:
            self.get_logger().error(f"Lab阈值修改失败：{str(e)}")
            response.success = False
            response.message = f"修改失败：{str(e)}"
            return response
    def exit_callback(self, request, response):
        self.get_logger().info("✅✅ 收到Exit服务，停止标签码垛并关闭硬件！")
        if self.stack_active:
            try:
                # 停止码垛状态
                self.stack_active = False

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
        response.message = "标签码垛已停止，硬件已关闭"
        return response

    def camera_processing_loop(self):
        fail_count = 0
        max_fails = 10
        while self.running and self.stack_active:
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
                # 处理帧
                self.process_frame(frame)
                time.sleep(0.03)  # 约30fps

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
    
    def process_frame(self, frame):   
        frame = cv2.flip(frame, -1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        mask_blue = cv2.inRange(lab, self.lower_blue, self.upper_blue)
        mask_red2 = cv2.inRange(lab, self.lower_red, self.upper_red)
        mask_green = cv2.inRange(lab, self.lower_green, self.upper_green)
        if self.mark_flag==0:
            #寻找码垛堆放基准点
            green_area, green_center,self.green_rect = self.detect_color(mask_green)
            if self.green_rect:
                self.block_cx, self.block_cy = green_center
                self.color_read_succed=1
        else:#寻找要识别的标签
            if self.cap_find<20 and self.capt==0:
                if self.cap_find==0:
                    kinematics_move(0,120,50,1500)
                    time.sleep(1.5)
                    self.search_hit_count = 0  # 新搜索位置，重置命中计数
                for tag in self.detector.detect(gray): # defaults to TAG36H11 without "families".
                    if tag.tag_id != self.current_stack_id:
                        continue
                    self.search_hit_count += 1
                    if self.search_hit_count >= 2:
                        self.capt=1
                self.cap_find+=1
                self.cap_right=1     
            elif self.cap_right==1 and self.cap_find>19 and self.cap_find<40 and self.capt==0:
                if self.cap_find<21:
                    kinematics_move(35,140,50,1500)
                    time.sleep(1.5)
                    self.search_hit_count = 0  # 新搜索位置，重置命中计数
                for tag in self.detector.detect(gray): # defaults to TAG36H11 without "families".
                    if tag.tag_id != self.current_stack_id:
                        continue
                    self.search_hit_count += 1
                    if self.search_hit_count >= 2:
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
                    self.search_hit_count = 0  # 新搜索位置，重置命中计数
                for tag in self.detector.detect(gray): # defaults to TAG36H11 without "families".
                    if tag.tag_id != self.current_stack_id:
                        continue
                    self.search_hit_count += 1
                    if self.search_hit_count >= 2:
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
                for tag in self.detector.detect(gray): # defaults to TAG36H11 without "families".
                    if tag.tag_id != self.current_stack_id:
                        continue
                    cv2.rectangle(frame, (int(tag.corners[0][0]), int(tag.corners[0][1])), (int(tag.corners[2][0]), int(tag.corners[2][1])), (0, 0, 255), 2)
                    self.block_cx=tag.center[0]
                    self.block_cy=tag.center[1]
                    self.block_angle=self.calculate_angle_edge_based(tag.corners)
                    self.color_read_succed=1
                    self.capt=1
        debug_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(debug_img_msg)
        #************************运动机械臂**********************************
        if self.color_read_succed == 1 and self.stack_active:
            if self.move_status == 1:#第二阶段：机械爪旋转到和物块平齐
                self.move_status = 2
                self.block_cx=self.block_cy=0
                self.spin_calw = 1500
                self.spin_calw -= self.block_angle * 7.4
                self.spin_calw = int(self.spin_calw)
                self.spin_calw = self.limit(self.spin_calw, 1167, 1833)
                print(f"spin_calw:={self.spin_calw}°self.move_status={self.move_status}")
                for i in range(5):#重复发送，防止下位机没有接收到
                    uart_send_str("#4P{:0^4}T1000!".format(self.spin_calw))
                    time.sleep(0.2)
                l = math.sqrt(self.move_x**2 + self.move_y**2)
                sin = self.move_y / l
                cos = self.move_x / l
                self.move_x = int((l + self.clamp_offset_x) * cos) 
                self.move_y = int((l + self.clamp_offset_y) * sin)
                for i in range(1):#重复发送，防止下位机没有接收到
                    kinematics_move(self.move_x, self.move_y, 60, 1000)
                    time.sleep(1)
                    uart_send_str("#005P1000T1000!")
                    time.sleep(1)
                    self.move_status = 2
            elif self.move_status == 2:#第三阶段：机械爪移动到物块位置
                self.move_status = 3
                for i in range(1):#重复发送，防止下位机没有接收到
                    kinematics_move(self.move_x, self.move_y - 6, self.clamp_offset_z, 1000)
                    time.sleep(1)
            elif self.move_status==3:#第四阶段：机械爪抓取物块
                self.move_status=4
                for i in range(3):
                    uart_send_str("#005P1700T1000!\n")
                    time.sleep(0.4)
            elif self.move_status == 4:#第五阶段：机械臂抬起
                self.move_status = 5
                for i in range(1):#重复发送，防止下位机没有接收到
                    kinematics_move(self.move_x, self.move_y, 150, 1000)
                    time.sleep(1)
                    uart_send_str("#004P1500T1000!\n")#旋转机械爪
                    time.sleep(1)
            elif self.move_status==5:#第六阶段：机械臂旋转到要放下物块的指定位置   
                if self.mark_flag==1:
                    self.move_x=self.bak_cx
                    self.move_y=self.bak_cy
                    for i in range(1):
                        kinematics_move(self.move_x,self.move_y,120,1500)
                        time.sleep(2.5)
                else:
                    self.move_x=-130#
                    self.move_y=30#30
                    for i in range(1):
                        kinematics_move(self.move_x,self.move_y,60,1500)
                        time.sleep(2.5)
                self.move_status=6  
            elif self.move_status==7:#第7阶段：机械爪分开，放下物块
                self.block_cx=self.block_cy=0
                self.move_status=8
                for i in range(1):#调整机械爪高度
                    if self.block_cnt==0:
                        kinematics_move(self.move_x,self.move_y+5,self.stack_height_one,1200)
                    elif self.block_cnt==1:
                        kinematics_move(self.move_x+2,self.move_y+5,self.stack_height_two,1200)
                    elif self.block_cnt==2:
                        kinematics_move(self.move_x+4,self.move_y+5,self.stack_height_three,1200)
                    time.sleep(2)
                for i in range(3):
                    uart_send_str("#005P1200T1000!}\n")
                    time.sleep(0.4)
                for i in range(1):
                    kinematics_move(self.move_x,self.move_y,130,1000)
                    time.sleep(1)
            elif self.move_status==8:#第8阶段：归位 
                self.block_cx=self.block_cy=0
                self.move_x = 0
                self.move_y = 120
                self.move_status=0
                self.color_read_succed=0
                self.block_cnt+=1
                if self.block_cnt>2:
                    self.block_cnt=0
                # 按顺序推进到下一个标签ID
                self.get_logger().info(f"标签ID={self.current_stack_id} 码垵完成")
                if self.current_stack_id < 3:
                    self.current_stack_id += 1
                    self.get_logger().info(f"开始码垵下一个标签ID={self.current_stack_id}")
                else:
                    self.get_logger().info("所有标签（1→2→3）码垵完成！")
                # mark_flag=255
                for i in range(1):
                    kinematics_move(self.move_x,self.move_y,130,1000)
                    time.sleep(1.1)
                    kinematics_move(self.move_x,self.move_y,50,1000)
                    time.sleep(1.1)
                    self.capt=0#是否检测到
                    self.cap_find=0#检测次数
                    self.cap_right=0#是否向右寻找
                    self.cap_left=0#是否向左寻找
                    self.cap_ok=0#是否连续检测到，防止误测
                    self.cap_find_ok=0#抓取过程中目标丢失则退回寻找函数
                    self.move_ok=0#重置位置传递标志
                    self.search_hit_count=0#重置搜索命中计数

    def sample_and_control(self):
        """控制逻辑（仅在码垛激活且硬件就绪时执行）"""
        if not self.stack_active or not self.camera_open or not self.uart_open:
            return  # 未启动或硬件未就绪，直接返回
        if self.color_read_succed == 1:
            if self.move_status == 0:
                self.pid_x.Target_val = 160
                self.pid_y.Target_val = 120
                self.move_x -= self.pid_x.PID_Realize(self.block_cx)
                self.move_y += self.pid_y.PID_Realize(self.block_cy)
                self.move_x = self.limit(self.move_x, -150, 150)
                self.move_y = self.limit(self.move_y, -30, 250)
                # print(f"block_cx={self.block_cx},move_x={self.move_x}===block_cy={self.block_cy},move_y={self.move_y}")
                kinematics_move(self.move_x, self.move_y, 60, 100)
                self.color_read_succed = 0
                if abs(self.block_cy-120)<=3 and abs(self.block_cx-160)<=5: #寻找到物块，机械臂进入第二阶段
                    self.move_status=1
                    self.color_read_succed=1
            elif self.move_status == 6:
                if self.mark_flag==255:
                    #开始寻找堆放点
                    self.mark_flag=0
                    self.color_read_succed=0
                elif self.mark_flag==1:
                    #基准点已找到，跳过这个阶段
                    self.move_status=7
                elif self.mark_flag ==0:
                    self.pid_x.Target_val = 160
                    self.pid_y.Target_val = 120
                    self.move_y -= self.pid_x.PID_Realize(self.block_cx)
                    self.move_x -= self.pid_y.PID_Realize(self.block_cy)
                    self.move_x = self.limit(self.move_x, -170, 150)
                    self.move_y = self.limit(self.move_y, -30, 250)
                    # print(f"block_cx={self.block_cx},move_x={self.move_x}===block_cy={self.block_cy},move_y={self.move_y}")
                    kinematics_move(self.move_x, self.move_y, 65, 100)
                    self.color_read_succed=0
                    self.stack_pid_attempt += 1
                    if abs(self.block_cy - 120) <= 15 and abs(self.block_cx - 160) <= 15:
                        self.green_ok_cnt += 1
                    else:
                        self.green_ok_cnt = 0
                    # 连续2帧在±15px范围内确认对中，或尝试超过20次后强制进入堆放
                    if self.green_ok_cnt >= 2 or self.stack_pid_attempt >= 20:
                        if self.stack_pid_attempt >= 20:
                            self.get_logger().warn(
                                f"堆放点对中超时({self.stack_pid_attempt}次)，强制进入堆放阶段"
                            )
                        self.green_ok_cnt = 0
                        self.stack_pid_attempt = 0
                        self.move_status = 7
                        self.mark_flag=1
                        l=math.sqrt(self.move_x*self.move_x+self.move_y*self.move_y)
                        sin=self.move_y/l
                        cos=self.move_x/l
                        self.move_x=(l+self.place_offset_x)*cos
                        self.move_y=(l+self.place_offset_y)*sin
                        self.bak_cx=self.move_x
                        self.bak_cy=self.move_y
                        for i in range(1):#重复发送，防止下位机没有接收到
                            kinematics_move(self.move_x,self.move_y,65,1000)
                            time.sleep(1)
                        self.color_read_succed=1

    def control_loop(self):
        """控制线程循环（根据stack_active标志决定是否执行码垛）"""
        while self.running:
            try:
                self.sample_and_control()
                time.sleep(0.01)
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
        
        self.get_logger().info("标签码垛节点已停止，所有硬件资源已释放")

def main(args=None):
    rclpy.init(args=args)
    node = LabelrStackNode()
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