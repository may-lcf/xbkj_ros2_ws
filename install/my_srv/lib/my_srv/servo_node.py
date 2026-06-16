#!/usr/bin/env python3
import cv2
import numpy as np
import time
import threading
import os
import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import Trigger



from z_uart import uart_send_str, setup_uart, close_uart  
from z_move import kinematics_move

class ServokNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        
        # 初始化参数
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('uart_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        
        # 获取参数
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.uart_port = self.get_parameter('uart_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.servo_cmd = None
        # 全局变量初始化
        self.running = True  # 控制程序运行的标志
        self.sorting_active = False  # 追踪是否启动
        self.uart_open = False       # 串口是否已打开
        self.timer = None
        self.camera_open = False     # 摄像头是否已打开
        # ROS2 通信组件
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/joint/image_raw', 10)
        # self.joint_sub = self.create_subscription(String, '/joint_commands', self.set_joint_cmds, 10)
        self.joint_sub = None
        self.width = 320
        self.hight = 240
        
        self.enter_srv = self.create_service(Trigger, '/servo/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/servo/exit', self.exit_callback)
        
        
        # 启动控制线程
        self.camera_thread = None
        self.camera_lock = threading.Lock()

        # 自动激活：启动时直接初始化串口并创建关节订阅（跳过摄像头）
        try:
            setup_uart(self.baud_rate)
            self.uart_open = True
            self.sorting_active = True
            _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
            self.joint_sub = self.create_subscription(String, '/joint_commands', self.set_joint_cmds, _qos)
            uart_send_str('{#000P1500T1000!#001P1666T1000!#002P2219T1000!#003P0905T1000!#004P1500T1000!}')
            self.get_logger().info("关节调节节点已就绪，串口已自动激活")
        except Exception as e:
            self.get_logger().error(f"串口自动激活失败: {str(e)}，请手动调用 /servo/enter")
            self.get_logger().info("关节调节节点已就绪，等待Enter服务启动追踪")

    def set_joint_cmds(self, msg):
        """设置追踪颜色的服务回调"""
        self.servo_cmd = msg.data
        uart_send_str(self.servo_cmd)
        self.get_logger().info(f"✅✅✅{self.servo_cmd}")

        
    def enter_callback(self, request, response):
        self.get_logger().info("✅ 收到Enter服务，启动关节调节并初始化硬件！")
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
                    # 发送初始机械臂位置指令（根据实际需求调整）
                    uart_send_str('{#000P1500T1000!#001P1666T1000!#002P2219T1000!#003P0905T1000!#004P1500T1000!}')
                    time.sleep(1)

                # 启动追踪状态
                self.sorting_active = True
                self.running = True
                self.servo_cmd = None
                self.camera_thread = None
                _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
                self.joint_sub = self.create_subscription(String, '/joint_commands', self.set_joint_cmds, _qos)
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
        response.message = "关节调节已启动"
        return response

    def exit_callback(self, request, response):
        self.get_logger().info("✅✅ 收到Exit服务，停止关节调节并关闭硬件！")
        if self.sorting_active:
            try:
                # 停止追踪状态
                self.sorting_active = False
                self.track_color = None  # 同时停止颜色追踪
                self.destroy_subscription(self.joint_sub)
                self.joint_sub = None
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
        response.message = "关节调节已停止，硬件已关闭"
        return response
    
    def camera_processing_loop(self):
        while self.running and self.sorting_active and self.camera_open:  
            try:
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().error("无法读取摄像头帧！")
                    return
                    # 处理帧
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.process_frame(frame)
                time.sleep(0.03)  # 约30fps
                
            except Exception as e:
                self.get_logger().error(f"摄像头处理失败: {str(e)}")
                time.sleep(0.1)
    def process_frame(self, frame):
        frame = cv2.flip(frame, -1)
        debug_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(debug_img_msg)


    def destroy_node(self):
        """节点销毁时释放所有资源"""
        self.running = False
        self.sorting_active = False
        # 等待摄像头线程结束
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=2)
        # 确保关闭摄像头和串口（即使未调用exit_callback）
        if self.camera_open:
            self.cap.release()
            self.camera_open = False
        if self.uart_open:
            close_uart()  
            self.uart_open = False
        
        self.get_logger().info("关节调节节点已停止，所有硬件资源已释放")

def main(args=None):
    rclpy.init(args=args)
    
    node = ServokNode()
    
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