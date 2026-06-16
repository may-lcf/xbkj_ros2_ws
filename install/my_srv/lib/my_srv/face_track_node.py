#!/usr/bin/env python3
import cv2
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import Trigger

import os
from z_uart import uart_send_str, setup_uart, close_uart  

_haar_path = os.path.join(os.path.expanduser('~'), 'OpenCV', 'haarcascade_frontalface_default.xml')
face_cascade = cv2.CascadeClassifier(_haar_path)
if face_cascade.empty():
    raise IOError('无法加载Haar Cascade模型，请检查文件路径是否正确')

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

class FaceTrackNode(Node):
    def __init__(self):
        super().__init__('face_track_node')
        
        # 初始化参数
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('uart_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        # 获取参数
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.uart_port = self.get_parameter('uart_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        # 全局变量初始化
        self.servo0 = 1500
        self.servo2 = 1500
        self.running = True
        self.sorting_active = False
        self.camera_open = False
        self.uart_open = False
        self.camera_thread = None
        self.camera_lock = threading.Lock()
        
        # 追踪相关变量
        self.max_area = 0
        self.max_face = None
        self.width = 320
        self.hight = 240

        self.pid_x = PIDController(kp=0.05, ki=0.0, kd=0.00)
        self.pid_y = PIDController(kp=0.05, ki=0.0, kd=0.00)

        # ROS2 通信组件
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/face_track/image_raw', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.enter_srv = self.create_service(Trigger, '/face_track/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/face_track/exit', self.exit_callback)
        
        # 启动控制线程
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        self.get_logger().info("人脸追踪节点已就绪，等待Enter服务启动追踪")
         
    def enter_callback(self, request, response):
        self.get_logger().info("✅ 收到Enter服务，启动人脸追踪并初始化硬件！")
        if not self.sorting_active:
            try:
                # 初始化摄像头
                if not self.camera_open:
                    self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
                    if not self.cap.isOpened():
                        raise Exception(f"摄像头未打开（索引={self.camera_index}）")
                    self.cap.set(3, self.width)
                    self.cap.set(4, self.hight)
                    self.camera_open = True
                    self.get_logger().info("摄像头初始化成功")

                # 初始化串口
                if not self.uart_open:
                    setup_uart(self.baud_rate)
                    self.uart_open = True
                    self.get_logger().info("串口初始化成功")
                    uart_send_str('{#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P0860T1000!#004P1500T1000!}')

                # 启动追踪状态
                self.sorting_active = True
                self.running = True
                self.servo0 = 1500
                self.servo2 = 1500               
                self.max_area = 0
                self.max_face = None
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
        response.message = "人脸追踪已启动"
        return response

    def exit_callback(self, request, response):
        self.get_logger().info("✅✅ 收到Exit服务，停止人脸追踪并关闭硬件！")
        if self.sorting_active:
            try:
                # 停止追踪状态
                self.sorting_active = False
                
                # 关闭摄像头
                if self.camera_open:
                    self.cap.release()
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
        response.message = "人脸追踪已停止，硬件已关闭"
        return response

    def camera_processing_loop(self):
        """摄像头处理线程"""
        while self.running and self.sorting_active and self.camera_open:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().error("无法读取摄像头帧！")
                    time.sleep(0.1)
                    continue
                    
                # 处理帧
                self.process_frame(frame)
                time.sleep(0.03)  # 约30fps
                
            except Exception as e:
                self.get_logger().error(f"摄像头处理失败: {str(e)}")
                time.sleep(0.1)

    def process_frame(self, frame):
        frame = cv2.flip(frame, -1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # 寻找最大的人脸
        self.max_area = 0
        self.max_face = None
        if len(faces) > 0:
            for (x, y, w, h) in faces:
                area = w * h
                if area > self.max_area:
                    self.max_area = area
                    self.max_face = (x, y, w, h)
    
        # 标记人脸并计算中心坐标
        if self.max_face is not None:
            x, y, w, h = self.max_face
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cx = int(x + w / 2)
            cy = int(y + h / 2)
            cv2.drawMarker(frame, (cx, cy), (0, 0, 255), markerType=cv2.MARKER_CROSS, 
                        markerSize=10, thickness=2)
        
        # 发布图像
        try:
            debug_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(debug_img_msg)
            self.camera_pub.publish(debug_img_msg)
        except Exception as e:
            self.get_logger().error(f"图像发布失败: {str(e)}")

    def limit(self, dat, mn, mx):
        """限制数值范围"""
        if dat >= mx:
            return mx
        elif dat <= mn:
            return mn
        return dat

    def sample_and_control(self):
        """控制逻辑"""
        if not self.sorting_active or not self.uart_open:
            return
        if self.max_face is not None:
            x, y, w, h = self.max_face
            cx = (x + w / 2)
            cy = (y + h / 2)
            self.pid_x.Target_val = 160
            self.pid_y.Target_val = 120
            self.servo0 += int(self.pid_x.PID_Realize(cx))
            self.servo2 -= int(self.pid_y.PID_Realize(cy))
            self.servo0 = self.limit(self.servo0, 600, 2400)
            self.servo2 = self.limit(self.servo2, 600, 2400)
            print(f"servo0={self.servo0}====servo2={self.servo2}=====")
            uart_send_str("{{#000P{:0>4d}T0000!#002P{:0>4d}T0000!}}".format(self.servo0, self.servo2))

    def control_loop(self):
        """控制线程循环"""
        while self.running:
            try:
                self.sample_and_control()
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"控制循环错误: {str(e)}")
                break

    def destroy_node(self):
        """节点销毁时释放所有资源"""
        self.get_logger().info("正在关闭节点...")
        self.running = False
        self.sorting_active = False
        
        # 等待控制线程结束
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2)
            
        # 等待摄像头线程结束
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=2)
            
        # 确保关闭硬件资源
        if self.camera_open:
            self.cap.release()
            self.camera_open = False
        
        if self.uart_open:
            close_uart()  
            self.uart_open = False
            
        self.get_logger().info("人脸追踪节点已停止，所有硬件资源已释放")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = FaceTrackNode()
    
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