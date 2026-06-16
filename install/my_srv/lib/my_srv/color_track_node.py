#!/usr/bin/env python3
import cv2
import glob
import os
import numpy as np
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import Trigger
from my_srv.srv import Add  

from z_uart import uart_send_str, setup_uart, close_uart  

# 加载Lab颜色阈值
_DATA_DIR = os.path.expanduser('~/ros2_ws')
loaded_red = []
loaded_green = []
loaded_blue = []
with open(os.path.join(_DATA_DIR, 'red.txt'), 'r') as f_red:
    for line in f_red:
        line = line.strip()
        if not line:
            continue
        num_strs = line.split()  
        for s in num_strs:
            try:
                loaded_red.append(int(s))
            except ValueError:
                loaded_red.append(float(s))
redlow1, redlow2, redlow3, redhigh1, redhigh2, redhigh3 = loaded_red

with open(os.path.join(_DATA_DIR, 'blue.txt'), 'r') as f_blue:
    for line in f_blue:
        line = line.strip()
        if not line:
            continue
        num_strs = line.split()  
        for s in num_strs:
            try:
                loaded_blue.append(int(s))
            except ValueError:
                loaded_blue.append(float(s))
bluelow1, bluelow2, bluelow3, bluehigh1, bluehigh2, bluehigh3  = loaded_blue

with open(os.path.join(_DATA_DIR, 'green.txt'), 'r') as f_green:
    for line in f_green:
        line = line.strip()
        if not line:
            continue
        num_strs = line.split()  
        for s in num_strs:
            try:
                loaded_green.append(int(s))
            except ValueError:
                loaded_green.append(float(s))
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

class ColorTrackNode(Node):
    def __init__(self):
        super().__init__('color_track_node')
        
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
        self.servo0 = 1500
        self.servo2 = 1500
        self.move_x = 160
        self.move_y = 120
        self.target_rect = None
        self.block_cx = 0
        self.block_cy = 0
        self.red_rect = None
        self.blue_rect = None
        self.green_rect = None
        self.detected_color = None
        self.color_labels = {0: "blue", 1: "red", 2: "green"}
        self.running = True
        self.sorting_active = False
        self.camera_open = False
        self.uart_open = False
        self.camera_thread = None
        self.camera_lock = threading.Lock()
        self.cap = None
        self.camera_source = None
        self.track_color = None
        
        # Lab颜色阈值（使用加载的值）
        self.lower_red = np.array([redlow1, redlow2, redlow3], dtype=np.uint8)
        self.upper_red = np.array([redhigh1, redhigh2, redhigh3], dtype=np.uint8)
        self.lower_blue = np.array([bluelow1, bluelow2, bluelow3], dtype=np.uint8)
        self.upper_blue = np.array([bluehigh1, bluehigh2, bluehigh3], dtype=np.uint8)
        self.lower_green = np.array([greenlow1, greenlow2, greenlow3], dtype=np.uint8)
        self.upper_green = np.array([greenhigh1, greenhigh2, greenhigh3], dtype=np.uint8)

        self.width = 320
        self.height = 240  

        self.pid_x = PIDController(kp=0.05, ki=0.0, kd=0.00)
        self.pid_y = PIDController(kp=0.05, ki=0.0, kd=0.00)

        # ROS2 通信组件
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/color_track/image_raw', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.color_sub = self.create_subscription(String, '/color', self.set_track_color_callback, 10)
        
        # 服务
        self.add_service = self.create_service(Add, "/Add", self.Color_callback)
        self.enter_srv = self.create_service(Trigger, '/color_track/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/color_track/exit', self.exit_callback)

        # 启动控制线程
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        self.get_logger().info("色块追踪节点已就绪（使用Lab颜色空间），等待Enter服务启动追踪")

    def set_track_color_callback(self, msg):
        """设置追踪颜色的服务回调"""
        self.track_color = msg.data
        self.get_logger().info(f"✅✅✅ 设置追踪颜色: {self.track_color}")

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
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
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
        self.get_logger().info("✅ 收到Enter服务，启动色块追踪并初始化硬件！")
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
                    uart_send_str('{#000P1500T1000!#001P1666T1000!#002P2219T1000!#003P0905T1000!#004P1500T1000!}')
                    time.sleep(1)

                # 启动追踪状态
                self.running = True
                self.sorting_active = True
                self.servo0 = 1500
                self.servo2 = 2219               
                self.move_x = 0
                self.move_y = 120
                self.target_rect = None
                self.block_cx = 0
                self.block_cy = 0
                self.red_rect = None
                self.blue_rect = None
                self.green_rect = None
                self.detected_color = None
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
        response.message = "色块追踪已启动"
        return response

    def exit_callback(self, request, response):
        self.get_logger().info("✅✅ 收到Exit服务，停止色块追踪并关闭硬件！")
        if self.sorting_active:
            try:
                # 停止追踪状态
                self.sorting_active = False
                self.track_color = None
                
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
        response.message = "色块追踪已停止，硬件已关闭"
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
                # 处理帧
                self.process_frame(frame)

                # 发布原始图像
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.camera_pub.publish(img_msg)

                time.sleep(0.001)  # 控制处理频率

            except Exception as e:
                self.get_logger().error(f"摄像头处理失败: {str(e)}")
                time.sleep(0.1)

    def detect_color(self, mask):
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts) > 0:
            best = None
            best_area = -1
            for cnt in cnts:
                rect = cv2.minAreaRect(cnt)
                (c_x, c_y), (c_w, c_h), _ = rect
                area = cv2.contourArea(cnt)
                side_min = min(c_h, c_w)
                side_max = max(c_h, c_w)
                is_square = side_min > 0 and (side_min / side_max) >= 0.85
                if not is_square:
                    continue
                cond = 15 < c_h < 250 and 15 < c_w < 250 and area > 500
                if cond and area > best_area:
                    best_area = area
                    best = (area, (c_x, c_y), rect)
            if best is not None:
                return best
        return 0, (0, 0), None
    
    def process_frame(self, frame):
        # 翻转图像
        frame = cv2.flip(frame, -1)
        
        # 转换为Lab颜色空间（关键修改）
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        
        # 根据追踪模式决定处理哪些颜色
        if self.track_color == 'red':
            # 处理红色
            mask_red = cv2.inRange(lab, self.lower_red, self.upper_red)
            red_area, red_center, self.red_rect = self.detect_color(mask_red)
            
            if red_area > 0:
                self.detected_color = 'red'
                self.target_center = red_center
                self.target_rect = self.red_rect
            else:
                self.detected_color = None
                self.target_center = (0, 0)
                self.target_rect = None
                
        elif self.track_color == 'green':
            # 处理绿色
            mask_green = cv2.inRange(lab, self.lower_green, self.upper_green)
            green_area, green_center, self.green_rect = self.detect_color(mask_green)
            
            if green_area > 0:
                self.detected_color = 'green'
                self.target_center = green_center
                self.target_rect = self.green_rect
            else:
                self.detected_color = None
                self.target_center = (0, 0)
                self.target_rect = None
                
        elif self.track_color == 'blue':
            # 处理蓝色
            mask_blue = cv2.inRange(lab, self.lower_blue, self.upper_blue)
            blue_area, blue_center, self.blue_rect = self.detect_color(mask_blue)
            
            if blue_area > 0:
                self.detected_color = 'blue'
                self.target_center = blue_center
                self.target_rect = self.blue_rect
            else:
                self.detected_color = None
                self.target_center = (0, 0)
                self.target_rect = None
                
        else:
            # 检测所有颜色但选择面积最大的
            mask_blue = cv2.inRange(lab, self.lower_blue, self.upper_blue)
            mask_red = cv2.inRange(lab, self.lower_red, self.upper_red)
            mask_green = cv2.inRange(lab, self.lower_green, self.upper_green)

            red_area, red_center, self.red_rect = self.detect_color(mask_red)
            green_area, green_center, self.green_rect = self.detect_color(mask_green)
            blue_area, blue_center, self.blue_rect = self.detect_color(mask_blue)

            color_areas = {
                'red': (red_area, red_center, self.red_rect),
                'blue': (blue_area, blue_center, self.blue_rect),
                'green': (green_area, green_center, self.green_rect)
            }
            
            max_area = 0
            self.detected_color = None
            self.target_center = (0, 0)
            self.target_rect = None
            
            for color, (area, center, rect) in color_areas.items():
                if area > max_area and area > 0:
                    max_area = area
                    self.detected_color = color
                    self.target_center = center
                    self.target_rect = rect

        # 如果检测到目标颜色，进行追踪
        if self.detected_color is not None and self.target_rect is not None:
            self.block_cx, self.block_cy = self.target_center
            box = cv2.boxPoints(self.target_rect)
            box = np.intp(box)
            
            # 根据颜色绘制轮廓和文字
            if self.detected_color == 'red':
                cv2.drawContours(frame, [box], -1, (0, 0, 255), 2)
                cv2.putText(frame, "Red", (int(self.block_cx), int(self.block_cy)-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            elif self.detected_color == 'green':
                cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)
                cv2.putText(frame, "Green", (int(self.block_cx), int(self.block_cy)-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            elif self.detected_color == 'blue':
                cv2.drawContours(frame, [box], -1, (255, 0, 0), 2)
                cv2.putText(frame, "Blue", (int(self.block_cx), int(self.block_cy)-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            # 显示当前追踪模式
            if self.track_color:
                cv2.putText(frame, f"Tracking: {self.track_color}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
        else:
            # 没有检测到目标时，显示追踪模式
            if self.track_color:
                cv2.putText(frame, f"Tracking: {self.track_color} - No target", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # 发布处理后的图像
        debug_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(debug_img_msg)

    def limit(self, dat, mn, mx):
        """限制数值范围"""
        return max(min(dat, mx), mn)
    
    def sample_and_control(self):
        """控制逻辑"""
        if not self.sorting_active or not self.uart_open or not self.camera_open:
            return
            
        if self.detected_color is not None and self.target_rect is not None:
            self.pid_x.Target_val = 160
            self.pid_y.Target_val = 120
            pid_x_output = self.pid_x.PID_Realize(self.block_cx)
            pid_y_output = self.pid_y.PID_Realize(self.block_cy)
            
            self.servo0 += int(pid_x_output)
            self.servo2 -= int(pid_y_output)
            
            self.servo0 = self.limit(self.servo0, 600, 2400)
            self.servo2 = self.limit(self.servo2, 600, 2400)
            
            # print(f"Servo0={self.servo0}, Servo2={self.servo2}")
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
        self.get_logger().info("正在关闭节点...")
        self.running = False
        self.sorting_active = False
        
        
        
        # 等待线程结束
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2)
            
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=2)
            
        # 释放硬件资源
        if self.camera_open:
            if self.cap is not None:
                self.cap.release()
            self.cap = None
            self.camera_open = False
            
        if self.uart_open:
            close_uart()
            self.uart_open = False
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ColorTrackNode()
    
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