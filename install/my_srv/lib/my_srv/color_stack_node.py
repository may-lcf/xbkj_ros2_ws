#!/usr/bin/env python3

import cv2
import glob
import os
import numpy as np
import time
import threading

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import Trigger
from my_srv.srv import Add  


# 导入自定义模块
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
    
class ColorStackNode(Node):
    def __init__(self):
        super().__init__('color_stack_node')
        
        # 初始化参数（保持原有参数配置）
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
        self.clamp_offset_x = 53        # 如果夹取物体时偏左或偏右，加减此值(偏右-减小，偏左-增大)
        self.clamp_offset_y = 53        # 如果夹取物体时偏远或偏近，加减此值(偏远-减小，偏近-增大)
        self.clamp_offset_z = 20        # 如果夹取物体时偏高或偏低，加减此值(偏高-减小，偏低-增大)
        self.place_offset_x = 60        # 如果放置物体时偏远或偏近，加减此值(偏远-减小，偏近-增大)
        self.place_offset_y = 60        # 如果放置物体时偏左或偏右，加减此值(偏右-减小，偏左-增大)
        self.stack_height_one = 10      # 码垛第一层高度（数值越大越高）
        self.stack_height_two = 46      # 码垛第二层高度（数值越大越高）
        self.stack_height_three = 70    # 码垛第三层高度（数值越大越高）
        
        self.move_x=0
        self.move_y=120
        self.move_status=0
        self.target_rect = None
        self.color_read_succed = 0
        self.color_state = 255
        self.block_cx = 0
        self.block_cy = 0
        self.red_rect = None
        self.blue_rect = None
        self.green_rect = None
        self.running = True  # 控制程序运行的标志
        self.stack_active = False  # 码垛是否启动
        self.camera_open = False     # 摄像头是否已打开
        self.uart_open = False       # 串口是否已打开
        self.camera_thread = None
        self.camera_lock = threading.Lock()
        self.cap = None
        self.camera_source = None

        self.spin_calw=1500#机械爪旋转角度
        self.mark_flag=255#标志位
        self.bak_cx=-130#上次堆放坐标
        self.bak_cy=30
        self.block_cnt=0#记录抓取的物块数量
        self.detected_color = None  # 新增：记录检测到的颜色
        self.converge_count = 0       # 夹取收敛连续稳定帧计数
        self.stack_converge_count = 0 # 码垛收敛连续稳定帧计数
        self.stack_pid_attempt = 0    # 码垛堆放点对中尝试次数（超限后强制进入堆放）
        # 每种颜色的夹取位置补偿（单位mm，施加在l+60延伸前）
        # 正值=向前补偿，负值=向后补偿；根据实际偏差方向微调数值
        # 红色偏前→y补偿为负；绿/蓝偏后→y补偿为正
        self.color_pick_offset = {
            'red':   {'x': 0, 'y': -2},
            'blue':  {'x': 0, 'y':  3},
            'green': {'x': 0, 'y':  3},
        }

        # 颜色阈值（初始值，后续可通过服务修改）
        # self.lower_red1 = np.array([0,43,46])0 43 46 10 255 255
        # self.upper_red1 = np.array([10,255,255])
        # self.lower_red2 = np.array([156,43,46])156 43 46 180 255 255
        # self.upper_red2 = np.array([180,255,255])
        # self.lower_blue = np.array([100, 120, 70])100 120 70 130 255 255
        # self.upper_blue = np.array([130, 255, 255])
        # self.lower_green =np.array([40, 50, 50])40 50 50 90 255 255
        # self.upper_green = np.array([90,255,255])
        self.lower_red = np.array([redlow1,redlow2,redlow3])
        self.upper_red = np.array([redhigh1,redhigh2,redhigh3])
        self.lower_blue = np.array([bluelow1, bluelow2, bluelow3])
        self.upper_blue = np.array([bluehigh1, bluehigh2, bluehigh3])
        self.lower_green =np.array([greenlow1, greenlow2, greenlow3])
        self.upper_green = np.array([greenhigh1,greenhigh2,greenhigh3])
        print(self.lower_red)
        print(self.upper_red)
        self.width = 320
        self.hight = 240

        self.pid_x = PIDController(kp=0.06, ki=0.0, kd=0.00)
        self.pid_y = PIDController(kp=0.06, ki=0.0, kd=0.00)

        # ROS2 通信组件（新增 camera_pub 发布 /camera/image_raw）
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/color_stack/image_raw', 10)  # 调试图像
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)  # 新增：发布原始图像
        self.add_service = self.create_service(
            srv_type=Add,              # 服务类型：必须与 .srv 文件编译后的类型一致（如 my_srv/srv/Add）
            srv_name="/Add",           # 服务名称：必须严格等于你想注册的名称（这里是 /Add，大小写敏感）
            callback=self.Color_callback  # 回调函数：必须指向你的处理函数（注意拼写！）
        )
        self.enter_srv = self.create_service(Trigger, '/color_stack/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/color_stack/exit', self.exit_callback)
        # 启动控制线程
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        # 定时器在硬件初始化后启动（通过enter_callback触发）
        self.timer = None
        self.get_logger().info("色块码垛节点已就绪，等待Enter服务启动码垛")

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
        self.get_logger().info("✅ 收到Enter服务，启动色块码垓并初始化硬件！")
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
                # 启动码垛状态
                self.stack_active = True
                self.running = True
                self.block_cnt=0
                self.move_x=0
                self.move_y=120
                self.move_status=0
                self.target_rect = None
                self.color_read_succed = 0
                self.color_state = 255
                self.block_cx = 0
                self.block_cy = 0
                self.bak_cx=-130#上次堆放坐标
                self.bak_cy=30
                self.mark_flag=255#标志位
                self.detected_color = None
                self.converge_count = 0
                self.stack_converge_count = 0
                self.stack_pid_attempt = 0
                self.red_rect = None
                self.blue_rect = None
                self.green_rect = None
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
        response.message = "色块码垛已启动"
        return response

    def exit_callback(self, request, response):
        self.get_logger().info("✅✅ 收到Exit服务，停止色块码垛并关闭硬件！")
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
        response.message = "色块码垛已停止，硬件已关闭"
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

    def calculate_rectangle_angle(self, rect):
        """计算矩形倾角"""
        _, _, angle = rect
        if angle <= 10 or angle >= 80:
            angle = 0
        elif angle < 45 and angle > 10:
            angle = -angle
        elif angle > 45 and angle < 80:
            angle = 90 - angle
        return angle
    
    def detect_color(self,mask):
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts) > 0:
            best = None
            best_area = -1
            for cnt in cnts:
                rect = cv2.minAreaRect(cnt)
                (c_x, c_y), (c_w, c_h), _ = rect
                area = cv2.contourArea(cnt)
                # 判断是否接近正方形（长宽比接近 1）
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
        lab = cv2.cvtColor(frame,cv2.COLOR_BGR2LAB)

        # 创建颜色掩码
        mask_blue = cv2.inRange(lab, self.lower_blue, self.upper_blue)
        # mask_red1 = cv2.inRange(lab, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(lab, self.lower_red, self.upper_red)
        mask_green = cv2.inRange(lab, self.lower_green, self.upper_green)
        # 合并红色的两个掩码
        # mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        if self.mark_flag==0:
            #寻找码垛堆放基准点
            green_area, green_center,self.green_rect = self.detect_color(mask_green)
            if self.green_rect:
                self.block_cx, self.block_cy = green_center
                self.color_read_succed=1
        else:
            red_area, red_center, self.red_rect = self.detect_color(mask_red2)
            blue_area, blue_center, self.blue_rect = self.detect_color(mask_blue)
            green_area, green_center,self.green_rect = self.detect_color(mask_green)
            if self.color_state==255 or self.color_state==1:#红色
                if self.red_rect is not None:
                    self.block_cx, self.block_cy = red_center
                    box = cv2.boxPoints(self.red_rect)
                    cv2.drawContours(frame, [np.intp(box)], -1, (255, 0, 0), 2)
                    cv2.putText(frame, "red", (int(self.block_cx), int(self.block_cy)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    self.detected_color = "red"
                    self.color_read_succed=1
                    self.color_state=1
            if self.color_state==255 or self.color_state==2:#蓝色
                if self.blue_rect is not None:
                    self.block_cx, self.block_cy = blue_center
                    box = cv2.boxPoints(self.blue_rect)
                    cv2.drawContours(frame, [np.intp(box)], -1, (255, 0, 0), 2)
                    cv2.putText(frame, "blue", (int(self.block_cx), int(self.block_cy)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    self.detected_color = "blue" 
                    self.block_cx, self.block_cy = blue_center
                    self.color_read_succed=1
                    self.color_state=2
            if self.color_state==255 or self.color_state==3:#绿色
                if self.green_rect is not None:
                    self.block_cx, self.block_cy = green_center
                    box = cv2.boxPoints(self.green_rect)
                    cv2.drawContours(frame, [np.intp(box)], -1, (0, 255, 0), 2)  #
                    cv2.putText(frame, "green", (int(self.block_cx), int(self.block_cy)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    self.detected_color = "green"
                    self.color_read_succed=1
                    self.color_state=3

        debug_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(debug_img_msg)
        #************************运动机械臂**********************************
        if self.color_read_succed==1 and self.stack_active:#物块识别完毕
            if self.move_status==1:#第二阶段：机械爪旋转到和物块平齐
                self.move_status=2
                angle=self.calculate_rectangle_angle(self.target_rect)
                print(f"矩形倾角: {angle:.2f}°")
                self.spin_calw=1500
                self.spin_calw -= angle*7.4
                self.spin_calw = (int)(self.spin_calw)
                self.spin_calw = self.limit(self.spin_calw,1167,1833)
                print(f"spin_calw:={self.spin_calw}°")
                for i in range(5):#重复发送，防止下位机没有接收到
                    uart_send_str("#4P{:0^4}T1000!\n".format(self.spin_calw))#旋转和张开机械爪
                    time.sleep(0.2)
                # 施加每种颜色的系统性偏差补偿，消除LAB质心检测偏移
                corr = self.color_pick_offset.get(self.detected_color, {'x': 0, 'y': 0})
                self.move_x += corr['x']
                self.move_y += corr['y']
                self.get_logger().info(f"颜色补偿({self.detected_color}): dx={corr['x']}, dy={corr['y']}")
                l=math.sqrt(self.move_x*self.move_x+self.move_y*self.move_y)
                sin=self.move_y/l
                cos=self.move_x/l
                self.move_x=int((l+self.clamp_offset_x)*cos) #摄像头位于木块正上方，夹爪位置摄像头后方，加上一定距离，求解夹爪位于木块上方坐标
                self.move_y=int((l+self.clamp_offset_y)*sin)
                for i in range(1):#重复发送，防止下位机没有接收到
                    print('移动')
                    kinematics_move(self.move_x,self.move_y,50,1000)
                    time.sleep(1)
                    uart_send_str("#005P1000T1000!\n")
                    time.sleep(1)
                    self.move_status=2
            elif self.move_status == 2:#第三阶段：机械爪移动到物块位置
                self.move_status=3
                for i in range(1):
                    kinematics_move(self.move_x,self.move_y-6,self.clamp_offset_z,1000)
                    time.sleep(1)
            elif self.move_status == 3:#第四阶段：机械爪抓取物块
                self.move_status=4
                for i in range(3):
                    uart_send_str("#005P1700T1000!}\n")
                    time.sleep(0.4)
            elif self.move_status == 4:#第五阶段：机械臂抬起
                self.move_status=5
                self.block_cx=self.block_cy=0
                for i in range(1):
                    kinematics_move(self.move_x,self.move_y,150,1000)
                    time.sleep(1)
                    uart_send_str("#004P1500T1000!\n")#旋转机械爪
                    time.sleep(1)
            elif self.move_status == 5:#第六阶段：机械臂旋转到要放下物块的指定位置
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
                        kinematics_move(self.move_x,self.move_y,50,1500)
                        time.sleep(2.5)
                self.move_status=6
            elif self.move_status == 7:#第7阶段：机械爪分开，放下物块
                self.block_cx=self.block_cy=0
                self.move_status=8
                for i in range(1):#调整机械爪高度
                    if self.block_cnt==0:
                        kinematics_move(self.move_x,self.move_y+5,self.stack_height_one,1200)
                    elif self.block_cnt==1:
                        kinematics_move(self.move_x+2,self.move_y+5,self.stack_height_two,1200)
                    elif self.block_cnt==2:
                        kinematics_move(self.move_x+4,self.move_y+5,self.stack_height_three,1200)
                    time.sleep(2.5)
                for i in range(3):
                    uart_send_str("#005P1200T1000!}\n")
                    time.sleep(0.4)
                for i in range(1):
                    kinematics_move(self.move_x,self.move_y,130,1000)
                    time.sleep(1)
            elif self.move_status == 8:#第8阶段：归位
                self.move_x=0
                self.move_y=120
                self.block_cnt+=1
                self.block_cx = self.block_cy = 0
                for i in range(1):
                    kinematics_move(self.move_x,self.move_y,50,1000)
                    time.sleep(1)
                    self.move_status=0
                print("成功搬运")
                self.move_status=0
                self.color_state+=1
                if self.color_state>3:
                    self.color_state=255
                self.color_read_succed=0

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
                kinematics_move(self.move_x, self.move_y, 65, 100)
                self.color_read_succed = 0
                
                if abs(self.block_cy - 120) <= 4 and abs(self.block_cx - 160) <= 6:
                    if self.detected_color == "red":
                        self.color_state = 1
                        self.target_rect = self.red_rect
                    elif self.detected_color == "blue":
                        self.color_state = 2
                        self.target_rect = self.blue_rect
                    elif self.detected_color == "green":
                        self.color_state = 3
                        self.target_rect = self.green_rect
                    self.color_read_succed = 1
                    self.get_logger().info(f"识别完成，颜色为{self.detected_color}")
                    self.move_status = 1
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
                    self.color_read_succed = 0
                    self.stack_pid_attempt += 1
                    if abs(self.block_cy - 120) <= 15 and abs(self.block_cx - 160) <= 15:
                        self.stack_converge_count += 1
                    else:
                        self.stack_converge_count = 0
                    # 连续2帧在±15px范围内确认对中，或尝试超过20次后强制进入堆放
                    if self.stack_converge_count >= 2 or self.stack_pid_attempt >= 20:
                        if self.stack_pid_attempt >= 20:
                            self.get_logger().warn(
                                f"堆放点对中超时({self.stack_pid_attempt}次)，强制进入堆放阶段"
                            )
                        self.stack_converge_count = 0
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
                        kinematics_move(self.move_x,self.move_y,60,1000)
                        time.sleep(1)
                        self.color_read_succed = 1

    def control_loop(self):
        """控制线程循环（根据stack_active标志决定是否执行码垛）"""
        while self.running:
            try:
                self.sample_and_control()
                time.sleep(0.03)
            except Exception as e:
                self.get_logger().error(f"控制循环错误: {e}")
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
        
        self.get_logger().info("色块码垛节点已停止")

def main(args=None):
    rclpy.init(args=args)

    node = ColorStackNode()
    
    # 使用多线程执行器
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