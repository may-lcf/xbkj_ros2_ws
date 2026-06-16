#!/usr/bin/env python3

import cv2
import numpy as np
import time
import threading
import os
import sys
import math
import glob
import rclpy
import pupil_apriltags 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Int32, Bool
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
    
class LabelSortingNode(Node):
    def __init__(self):
        super().__init__('label_sorting_node')
        
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
        self.clamp_offset_x = 52        # 如果夹取物体时偏左或偏右，加减此值(偏右-减小，偏左-增大)
        self.clamp_offset_y = 52        # 如果夹取物体时偏远或偏近，加减此值(偏远-减小，偏近-增大)
        self.clamp_offset_z = 20        # 如果夹取物体时偏高或偏低，加减此值(偏高-减小，偏低-增大)
        self.place_offset_x = 60        # 如果放置物体时偏远或偏近，加减此值(偏远-减小，偏近-增大)
        self.place_offset_y = 60        # 如果放置物体时偏左或偏右，加减此值(偏右-减小，偏左-增大)
        
        self.target_colors = [1, 2, 3] # 存储要分拣的颜色ID列表
        self.current_target_index = 0  # 当前目标在列表中的索引
        self.sorting_mode = "all"  # 分拣模式: "all", "selective"
        self.move_x=0
        self.move_y=120
        self.color_state = 255
        self.red_rect = None
        self.blue_rect = None
        self.green_rect = None
        self.spin_calw=1500#机械爪旋转角度
        self.move_status=0#机械臂移动的方式
        #中心点
        self.block_cx=0
        self.block_cy=0
        #用来记录已经抓取到标签
        self.block_cnt=0#记录抓取的物块数量
        #抓取计数
        self.capt=0#是否检测到
        self.cap_find=0#检测次数
        self.cap_right=0#是否向右寻找
        self.cap_left=0#是否向左寻找
        self.cap_ok=0#是否连续检测到，防止误测
        self.cap_ok_num = 0
        self.search_hit_count = 0  # 当前搜索位置累计检测命中次数
        self.cap_find_ok=0#抓取过程中目标丢失则退回寻找函数
        self.move_ok=0#当退回寻找函数时屏蔽初始位置的传递
        self.color_read_succed=0
        self.bak_cx=0
        self.bak_cy=0
        self.apriltag_flag=0
        self.cap_num=0
        self.block_angle = 0
        self.ID = 0
        self.current_sort_id = 1  # 当前要分拣的标签ID，按1→2→3顺序

        self.running = True  # 控制程序运行的标志
        self.sorting_active = False  # 分拣是否启动
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

        self.success_cnt = 0
        self.sort_pid_attempt = 0  # 分拣放置点对中尝试次数（超限后强制进入下一阶段）

        self.pid_x = PIDController(kp=0.04, ki=0.0, kd=0.00)
        self.pid_y = PIDController(kp=0.04, ki=0.0, kd=0.00)

        # ROS2 通信组件
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/label_sorting/image_raw', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        
        self.add_service = self.create_service(
            srv_type=Add,              
            srv_name="/Add",           
            callback=self.Color_callback  
        )
        self.enter_srv = self.create_service(Trigger, '/label_sorting/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/label_sorting/exit', self.exit_callback)
        
        # 启动控制线程
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        self.camera_thread = None
        self.camera_lock = threading.Lock()
        self.cap = None
        self.camera_source = None
        # 提前创建订阅，节点启动后即可接收分拣指令（无需等待 enter）
        self.sort_command_sub = self.create_subscription(
            String, '/label_sorting/sort_command', self.sort_command_callback, 10)
        
        self.get_logger().info("标签分拣节点已就绪，等待分拣指令...")

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
            # 优先检查 by-id 路径
            by_id_devices = sorted(glob.glob('/dev/v4l/by-id/*-video-index0'))
            if by_id_devices:
                self.get_logger().info(f"USB摄像头设备已恢复: {by_id_devices[0]}（等待了{attempt}次）")
                return True
            # 其次检查显式配置的设备
            if self.camera_device and os.path.exists(self.camera_device):
                self.get_logger().info(f"USB摄像头设备已恢复: {self.camera_device}（等待了{attempt}次）")
                return True
            time.sleep(1.0)
        self.get_logger().warn(f"等待USB摄像头设备超时（{timeout}秒）")
        return False

    def try_reinitialize_camera(self):
        try:
            # 先等 USB 设备重新出现
            if not self.wait_for_camera_device():
                return False
            # 设备出现后再额外等一小段，让驱动稳定
            time.sleep(0.5)
            self.initialize_camera()
            self.get_logger().info("摄像头重新初始化成功")
            return True
        except Exception as e:
            self.get_logger().error(f"摄像头重新初始化失败: {str(e)}")
            return False

    def sort_command_callback(self, msg):
        """处理分拣指令的话题回调"""
        try:
            color_command = msg.data.strip()
            self.get_logger().info(f"收到分拣指令: {color_command}")
            
            # 解析分拣命令
            if color_command.lower() == "all":
                # 分拣所有颜色
                self.target_colors = [1, 2, 3]
                self.sorting_mode = "all"
                self.current_target_index = 0
                self.get_logger().info("设置为分拣所有颜色标签")
                
            elif color_command.lower() == "stop":
                # 停止分拣
                self.sorting_active = False
                self.get_logger().info("分拣已停止")
                
            else:
                # 解析选择性的分拣命令
                commands = color_command.lower().split()
                self.target_colors = []
                
                for cmd in commands:
                    if cmd == "sort_1":
                        self.target_colors.append(1)
                    elif cmd == "sort_2":
                        self.target_colors.append(2)
                    elif cmd == "sort_3":
                        self.target_colors.append(3)
                    else:
                        self.get_logger().warn(f"未知的分拣命令: {cmd}")
                
                if self.target_colors:
                    self.sorting_mode = "selective"
                    self.current_target_index = 0
                    self.get_logger().info(f"设置为分拣颜色: {self.target_colors}")
                else:
                    self.get_logger().warn("没有有效的分拣命令")
            
            self.get_logger().info(f"✅✅✅目标颜色列表: {self.target_colors}, 模式: {self.sorting_mode}")
            
        except Exception as e:
            self.get_logger().error(f"处理分拣指令失败: {str(e)}")

    def enter_callback(self, request, response):
        self.get_logger().info("✅ 收到Enter服务，启动标签分拣并初始化硬件！")
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
                
                # # 如果没有设置目标颜色，默认分拣所有
                # if not self.target_colors:
                #     self.target_colors = [1, 2, 3]
                #     self.sorting_mode = "all"
                #     self.current_target_index = 0
                #     self.get_logger().info("使用默认设置：分拣所有颜色")
                
                # 关键变量初始化
                self.get_logger().info("重置分拣状态变量...")
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
                self.search_hit_count = 0      # 重置搜索位置命中计数
                self.block_cx = 0              # 物块中心X坐标（相机坐标系）
                self.block_cy = 0              # 物块中心Y坐标（相机坐标系）
                self.color_read_succed = 0     # 颜色识别成功标志（0=未识别，1=成功）
                self.apriltag_flag = 0         # Apriltag识别模式（0=识别标签，1=识别颜色）
                self.move_status = 0           # 机械臂当前阶段（0=初始，1~9=分拣步骤）
                self.color_state = 255         # 颜色分拣进度（初始为255，对应初始状态）
                self.success_cnt = 0
                self.sort_pid_attempt = 0      # 重置分拣放置点对中尝试次数
                self.current_sort_id = 1       # 重置为从标签ID=1开始分拣
                self.camera_thread = None
                # 启动分拣状态
                self.sorting_active = True
                # 启动摄像头线程
                if not self.camera_thread:
                    self.camera_thread = threading.Thread(target=self.camera_processing_loop)
                    self.camera_thread.daemon = True
                    self.camera_thread.start()
                    self.get_logger().info("摄像头处理线程已启动")

                    self.sort_command_sub = self.create_subscription(String,'/label_sorting/sort_command',self.sort_command_callback,10)

            except Exception as e:
                self.get_logger().error(f"硬件初始化失败：{str(e)}")
                response.success = False
                response.message = f"硬件初始化失败：{str(e)}"
                return response

        response.success = True
        response.message = "标签分拣已启动"
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
        self.get_logger().info("✅✅ 收到Exit服务，停止标签分拣并关闭硬件！")
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
        response.message = "标签分拣已停止，硬件已关闭"
        return response

    def is_target_color(self, color_id):
        """检查颜色ID是否在目标颜色列表中"""
        return color_id in self.target_colors

    def get_next_target(self):
        """获取下一个目标颜色"""
        if not self.target_colors:
            return None
            
        if self.sorting_mode == "all":
            # 循环分拣所有颜色
            self.current_target_index = (self.current_target_index + 1) % len(self.target_colors)
        else:
            # 选择性分拣模式，按顺序分拣
            self.current_target_index += 1
            if self.current_target_index >= len(self.target_colors):
                # 所有指定颜色都已分拣完成
                return None
                
        return self.target_colors[self.current_target_index]

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

        if self.apriltag_flag==0:#识别抓取标签
            if self.cap_find<20 and self.capt==0:
                if self.cap_find==0:
                    kinematics_move(0,120,50,1500)
                    time.sleep(1.5)
                    self.search_hit_count = 0  # 新搜索位置，重置命中计数
                for tag in self.detector.detect(gray): # defaults to TAG36H11 without "families".
                    if tag.tag_id != self.current_sort_id:
                        continue
                    self.search_hit_count += 1
                    if self.search_hit_count >= 1:
                        self.capt=1
                self.cap_find+=1
                self.cap_right=1     
            elif self.cap_right==1 and self.cap_find>19 and self.cap_find<40 and self.capt==0:
                if self.cap_find<21:
                    kinematics_move(35,140,50,1500)
                    time.sleep(1.5)
                    self.search_hit_count = 0  # 新搜索位置，重置命中计数
                for tag in self.detector.detect(gray): # defaults to TAG36H11 without "families".
                    if tag.tag_id != self.current_sort_id:
                        continue
                    self.search_hit_count += 1
                    if self.search_hit_count >= 1:
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
                    if tag.tag_id != self.current_sort_id:
                        continue
                    self.search_hit_count += 1
                    if self.search_hit_count >= 1:
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
                    if tag.tag_id != self.current_sort_id:
                        continue
                    cv2.rectangle(frame, (int(tag.corners[0][0]), int(tag.corners[0][1])), (int(tag.corners[2][0]), int(tag.corners[2][1])), (0, 0, 255), 2)
                    self.block_cx=tag.center[0]
                    self.block_cy=tag.center[1]
                    self.block_angle=self.calculate_angle_edge_based(tag.corners)
                    self.ID=tag.tag_id
                    self.color_read_succed=1
                    self.capt=1
        elif self.apriltag_flag==1:#抓取到标签后颜色分拣
            red_area, red_center, self.red_rect = self.detect_color(mask_red2)
            blue_area, blue_center, self.blue_rect = self.detect_color(mask_blue)
            green_area, green_center,self.green_rect = self.detect_color(mask_green)
            if self.red_rect and self.ID == 1:
                self.block_cx, self.block_cy = red_center
                self.color_read_succed=1
            if self.blue_rect and self.ID == 3:
                self.block_cx, self.block_cy = blue_center
                self.color_read_succed=1
            if self.green_rect and self.ID == 2:
                self.block_cx, self.block_cy = green_center
                self.color_read_succed=1

        debug_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(debug_img_msg)

        #************************运动机械臂**********************************
        if self.color_read_succed == 1 and self.sorting_active:
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
                self.move_x = int((l+self.clamp_offset_x) * cos) 
                self.move_y = int((l+self.clamp_offset_y) * sin)
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
                    # kinematics_move(self.move_x,self.move_y-6,15,1000)#修正y方向偏差
                    # time.sleep(1)
                    uart_send_str("#005P1700T1000!\n")
                    time.sleep(0.4)
            elif self.move_status == 4:#第五阶段：机械臂抬起
                self.move_status = 5
                for i in range(1):#重复发送，防止下位机没有接收到
                    kinematics_move(self.move_x, self.move_y, 150, 1000)
                    time.sleep(1)
            elif self.move_status == 5:#第六阶段：机械臂旋转到要放下物块的指定位置
                self.apriltag_flag=1
                # self.block_cx = self.block_cy = 0
                self.move_x = -130
                self.move_y = 30
                for i in range(1):
                    kinematics_move(self.move_x, self.move_y, 120, 1000)
                    time.sleep(1)
                    uart_send_str("#004P1500T1500!")
                    time.sleep(0.5)
                for i in range(1):
                    kinematics_move(self.move_x, self.move_y, 60, 1000)
                    time.sleep(2.5)
                self.move_status = 6
            elif self.move_status == 7:#第7阶段：机械爪分开，放下物块
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
            elif self.move_status == 8:#第8阶段：归位
                self.move_status = 9
                for i in range(3):
                    uart_send_str("#005P1200T1000!")
                    time.sleep(0.4)
                for i in range(1):
                    kinematics_move(self.move_x, self.move_y, 70, 1000)
                    time.sleep(1)
            elif self.move_status == 9:
                self.block_cx=self.block_cy=0
                self.move_x = 0
                self.move_y = 120
                self.block_cx = self.block_cy = 0
                for i in range(1):
                    kinematics_move(self.move_x, self.move_y, 70, 1000)
                    time.sleep(1)  
                
                # 按顺序推进到下一个标签ID
                self.get_logger().info(f"标签ID={self.current_sort_id} 分拣完成")
                if self.current_sort_id < 3:
                    self.current_sort_id += 1
                    self.get_logger().info(f"开始分拣下一个标签ID={self.current_sort_id}")
                else:
                    self.get_logger().info("所有标签（1→2→3）分拣完成！")
                
                self.apriltag_flag=0
                self.color_read_succed = 0
                self.move_status = 0
                for i in range(1):
                    kinematics_move(self.move_x,self.move_y,50,1000)
                    time.sleep(1.5)
                    self.capt=0#是否检测到
                    self.cap_find=0#检测次数
                    self.cap_right=0#是否向右寻找
                    self.cap_left=0#是否向左寻找
                    self.cap_ok=0#是否连续检测到，防止误测
                    self.cap_ok_num=0#重置上一次cap_ok，防止误判连续检测
                    self.cap_find_ok=0#抓取过程中目标丢失则退回寻找函数
                    self.move_ok=0#重置位置传递标志，确保下次搜索能正确设置move_x/move_y
                    self.search_hit_count=0#重置搜索命中计数

    def sample_and_control(self):
        """控制逻辑（仅在分拣激活且硬件就绪时执行）"""
        if not self.sorting_active or not self.camera_open or not self.uart_open:
            return  # 未启动或硬件未就绪，直接返回

        if self.color_read_succed == 1:
            if self.move_status == 0:
                self.pid_x.Target_val = 160
                self.pid_y.Target_val = 120
                self.move_x -= self.pid_x.PID_Realize(self.block_cx)
                self.move_y += self.pid_y.PID_Realize(self.block_cy)
                self.move_x = self.limit(self.move_x, -150, 150)
                self.move_y = self.limit(self.move_y, -50, 250)
                # print(f"block_cx={self.block_cx},move_x={self.move_x}===block_cy={self.block_cy},move_y={self.move_y}")
                kinematics_move(self.move_x, self.move_y, 60, 100)
                self.color_read_succed = 0
                if abs(self.block_cy-120)<=3 and abs(self.block_cx-160)<=5: #寻找到物块，机械臂进入第二阶段
                    self.success_cnt += 1
                    if self.success_cnt >= 3:
                        self.success_cnt = 0
                        self.color_read_succed = 1
                        self.move_status = 1
                        self.apriltag_flag=255  #不要进行条码识别
                else:
                    self.success_cnt = 0
            elif self.move_status == 6:
                self.pid_x.Target_val = 160
                self.pid_y.Target_val = 120
                self.move_y -= self.pid_x.PID_Realize(self.block_cx)
                self.move_x -= self.pid_y.PID_Realize(self.block_cy)
                self.move_x = self.limit(self.move_x, -170, 150)
                self.move_y = self.limit(self.move_y, -50, 250)
                # print(f"block_cx={self.block_cx},move_x={self.move_x}===block_cy={self.block_cy},move_y={self.move_y}")
                kinematics_move(self.move_x, self.move_y, 60, 100)
                self.sort_pid_attempt += 1
                if abs(self.block_cy - 120) <= 15 and abs(self.block_cx - 160) <= 15:
                    self.success_cnt += 1
                else:
                    self.success_cnt = 0
                # 连续2帧在±15px范围内确认对中，或尝试超过20次后强制进入下一阶段
                if self.success_cnt >= 2 or self.sort_pid_attempt >= 20:
                    if self.sort_pid_attempt >= 20:
                        self.get_logger().warn(
                            f"分拣放置点对中超时({self.sort_pid_attempt}次)，强制进入放置阶段"
                        )
                    self.success_cnt = 0
                    self.sort_pid_attempt = 0
                    self.move_status = 7
                    self.color_read_succed = 1

    def control_loop(self):
        """控制线程循环（根据sorting_active标志决定是否执行分拣）"""
        while self.running:
            try:
                self.sample_and_control()
                time.sleep(0.03)
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
        
        self.get_logger().info("标签分拣节点已停止，所有硬件资源已释放")

def main(args=None):
    rclpy.init(args=args)
    
    node = LabelSortingNode()
    
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