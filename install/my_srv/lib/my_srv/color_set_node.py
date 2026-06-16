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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Int32, Bool
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import Trigger
from my_srv.srv import Add  

from z_uart import uart_send_str, setup_uart, close_uart  
from z_move import kinematics_move

class ColorSetNode(Node):
    def __init__(self):
        super().__init__('color_set_node')
        
        # 初始化参数
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('uart_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        # 获取参数
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.uart_port = self.get_parameter('uart_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # 颜色阈值（初始值，后续可通过服务修改）
        self.lower_red1 = np.array([0, 43, 46])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red = np.array([156, 43, 46])
        self.upper_red = np.array([180, 255, 255])
        self.lower_blue = np.array([100, 120, 70])
        self.upper_blue = np.array([130, 255, 255])
        self.lower_green = np.array([40, 50, 50])
        self.upper_green = np.array([90, 255, 255])
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([40, 255, 255])
        self.lower_purple = np.array([140, 50, 50])
        self.upper_purple = np.array([160, 255, 255])
        self.width = 320
        self.hight = 240
        self.block_cx = 0
        self.block_cy = 0
        self.running = True  # 控制程序运行的标志
        self.sorting_active = False  # 分拣是否启动
        self.camera_open = False     # 摄像头是否已打开
        self.uart_open = False       # 串口是否已打开
        # ROS2 通信组件
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/color_set/image_result', 10)
        self.red_pub = self.create_publisher(Image, '/color_red/image_result', 10)
        self.green_pub = self.create_publisher(Image, '/color_green/image_result', 10)
        self.blue_pub = self.create_publisher(Image, '/color_blue/image_result', 10)
        self.yellow_pub = self.create_publisher(Image, '/color_yellow/image_result', 10)
        self.purple_pub = self.create_publisher(Image, '/color_purple/image_result', 10)

        
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.add_service = self.create_service(
            srv_type=Add,              
            srv_name="/Add",           
            callback=self.Color_callback  
        )
        self.enter_srv = self.create_service(Trigger, '/color_set/enter', self.enter_callback)
        self.exit_srv = self.create_service(Trigger, '/color_set/exit', self.exit_callback)

        # 启动控制线程
        # self.control_thread = threading.Thread(target=self.control_loop)
        # self.control_thread.daemon = True
        # self.control_thread.start()

        self.camera_thread = None
        self.camera_lock = threading.Lock()
        
        self.get_logger().info("色块阈值调节节点已就绪，等待Enter服务启动分拣")

    def Color_callback(self, request, response):
        try:
            color = request.color
            low_thresh = np.array([request.low_h, request.low_s, request.low_v])
            high_thresh = np.array([request.high_h, request.high_s, request.high_v])
            if color == 'red':
                self.lower_red = low_thresh
                self.upper_red = high_thresh
                self.get_logger().info(f"修改红色掩码2阈值：低={low_thresh}，高={high_thresh}")
            elif color == 'blue':
                self.lower_blue = low_thresh
                self.upper_blue = high_thresh
                self.get_logger().info(f"修改蓝色阈值：低={low_thresh}，高={high_thresh}")
            elif color == 'green':
                self.lower_green = low_thresh
                self.upper_green = high_thresh
                self.get_logger().info(f"修改绿色阈值：低={low_thresh}，高={high_thresh}")
            elif color == 'yellow':
                self.lower_yellow = low_thresh
                self.upper_yellow = high_thresh
                self.get_logger().info(f"修改黄色阈值：低={low_thresh}，高={high_thresh}")
            elif color == 'purple':
                self.lower_purple = low_thresh
                self.upper_purple = high_thresh
                self.get_logger().info(f"修改紫色阈值：低={low_thresh}，高={high_thresh}")
            else:
                raise ValueError("颜色类型错误：仅支持red/blue/green")
            response.success = True
            response.message = "阈值修改成功"
            return response
        except Exception as e:
            self.get_logger().error(f"阈值修改失败：{str(e)}")
            response.success = False
            response.message = f"修改失败：{str(e)}"
            return response

    def enter_callback(self, request, response):
        self.get_logger().info("✅ 收到Enter服务，启动色块阈值调节并初始化硬件！")
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

                # 启动分拣状态
                self.sorting_active = True

                self.red_rect = None
                self.blue_rect = None
                self.green_rect = None

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
        response.message = "色块阈值调节节点已启动"
        return response

    def exit_callback(self, request, response):
        self.get_logger().info("✅✅ 收到Exit服务，停止色块阈值调节节点并关闭硬件！")
        if self.sorting_active:
            try:
                # 停止分拣状态
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
        response.message = "色块阈值调节节点已停止，硬件已关闭"
        return response
    
    def camera_processing_loop(self):
        while self.running and self.sorting_active and self.camera_open:  
            try:
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().error("无法读取摄像头帧！")
                    return
                    # 处理帧
                self.process_frame(frame)
                time.sleep(0.03)  # 约30fps
                
            except Exception as e:
                self.get_logger().error(f"摄像头处理失败: {str(e)}")
                time.sleep(0.1)
                
            except Exception as e:
                self.get_logger().error(f"摄像头处理失败: {str(e)}")

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

    def process_frame(self, frame):
        frame = cv2.flip(frame, -1)
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        mask_blue = cv2.inRange(lab, self.lower_blue, self.upper_blue)
        mask_red2 = cv2.inRange(lab, self.lower_red, self.upper_red)
        mask_green = cv2.inRange(lab, self.lower_green, self.upper_green)
        mask_yellow = cv2.inRange(lab, self.lower_yellow, self.upper_yellow)
        mask_purple = cv2.inRange(lab, self.lower_purple, self.upper_purple)
        # red_area, red_center, self.red_rect = self.detect_color(mask_red2)
        # if self.red_rect is not None:
        #     self.block_cx, self.block_cy = red_center
        #     box = cv2.boxPoints(self.red_rect)
        #     cv2.drawContours(frame, [np.int0(box)], -1, (0, 0, 255), 2)
        #     cv2.putText(frame, "red", (int(self.block_cx), int(self.block_cy)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # blue_area, blue_center, self.blue_rect = self.detect_color(mask_blue)
        # if self.blue_rect is not None:
        #     self.block_cx, self.block_cy = blue_center
        #     box = cv2.boxPoints(self.blue_rect)
        #     cv2.drawContours(frame, [np.intp(box)], -1, (255, 0, 0), 2)
        #     cv2.putText(frame, "blue", (int(self.block_cx), int(self.block_cy)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # green_area, green_center, self.green_rect = self.detect_color(mask_green)
        # if self.green_rect is not None:
        #     self.block_cx, self.block_cy = green_center
        #     box = cv2.boxPoints(self.green_rect)
        #     cv2.drawContours(frame, [np.int0(box)], -1, (0, 255, 0), 2)  
        #     cv2.putText(frame, "green", (int(self.block_cx), int(self.block_cy)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        debug_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        mask_red2_msg = self.bridge.cv2_to_imgmsg(mask_red2, "mono8")
        mask_blue_msg = self.bridge.cv2_to_imgmsg(mask_blue, "mono8")
        mask_green_msg = self.bridge.cv2_to_imgmsg(mask_green, "mono8")
        mask_yellow_msg = self.bridge.cv2_to_imgmsg(mask_blue, "mono8")
        mask_purple_msg = self.bridge.cv2_to_imgmsg(mask_green, "mono8")
        self.image_pub.publish(debug_img_msg)
        self.red_pub.publish(mask_red2_msg)
        self.blue_pub.publish(mask_blue_msg)
        self.green_pub.publish(mask_green_msg)
        self.yellow_pub.publish(mask_yellow_msg)
        self.purple_pub.publish(mask_purple_msg)
        # self.get_logger().info("123")


    
    def destroy_node(self):
        """节点销毁时释放所有资源"""
        self.running = False
        # if self.control_thread and self.control_thread.is_alive():
        #     self.control_thread.join(timeout=2)
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
        
        self.get_logger().info("色块分拣节点已停止，所有硬件资源已释放")

def main(args=None):
    rclpy.init(args=args)
    
    node = ColorSetNode()
    
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