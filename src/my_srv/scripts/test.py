#!/usr/bin/env python3

import cv2
import os
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import time
import numpy as np
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
from my_srv.srv import Add  
from example_interfaces.srv import Trigger
from std_srvs.srv import SetBool

from z_uart import uart_send_str, setup_uart, close_uart  


# 摄像头捕获类
class CameraCapture:
    def __init__(self, camera_index=0):
        # self.cap = cv2.VideoCapture(camera_index)
        # if not self.cap.isOpened():
        #     raise Exception("无法打开摄像头")
        
        # # 设置摄像头分辨率
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        self.frame = None
        self.running = False
        self.lock = threading.Lock()
        self.thread = None
    
    # def start(self):
    #     self.running = True
    #     self.thread = threading.Thread(target=self._capture_loop)
    #     self.thread.daemon = True
    #     self.thread.start()
        
    # def _capture_loop(self):
    #     while self.running:
    #         ret, frame = self.cap.read()
    #         if ret:
    #             with self.lock:
    #                 self.frame = frame
    #         time.sleep(0.01)
    
    # def get_frame(self):
    #     with self.lock:
    #         if self.frame is None:
    #             return None
    #         # 转换为JPEG格式
    #         ret, jpeg = cv2.imencode('.jpg', self.frame)
    #         return jpeg.tobytes() if ret else None
    
    # def stop(self):
    #     self.running = False
    #     if self.thread:
    #         self.thread.join()
    #     self.cap.release()

# ROS2节点类
class CameraHTTPServerNode(Node):
    def __init__(self):
        super().__init__('camera_http_server')
        
        # ROS2参数
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('publish_camera', False)
        
        self.camera_index = self.get_parameter('camera_index').value
        self.http_port = self.get_parameter('http_port').value
        self.publish_camera = self.get_parameter('publish_camera').value

        
        
        # 模式管理: '2d' 或 'depth'
        self.mode = '2d'

        # ROS2 模式切换服务
        self.set_mode_srv = self.create_service(SetBool, '/set_mode', self.set_mode_callback)
        self.get_mode_srv = self.create_service(Trigger, '/get_mode', self.get_mode_callback)

        # ROS2发布器
        self.control_publisher = self.create_publisher(String, 'http_control_commands', 10)
        self.joint_control_publisher = self.create_publisher(String, 'joint_commands', 10)
        
        if self.publish_camera:
            self.image_publisher = self.create_publisher(Image, 'camera_image', 10)
            self.bridge = CvBridge()
        
        # # 摄像头初始化
        # try:
        #     self.camera = CameraCapture(self.camera_index)
        #     self.camera.start()
        #     self.get_logger().info("摄像头初始化成功")
        # except Exception as e:
        #     self.get_logger().error(f"摄像头初始化失败: {e}")
        #     return
        
        # HTTP服务器设置
        self.server = None
        self.server_thread = None
        
        # 启动HTTP服务器
        self.start_http_server()
        
        self.get_logger().info(f"摄像头HTTP服务器节点已启动，端口: {self.http_port}")
    
    def get_service_name(self, base_name, action):
        """根据当前模式路由服务名"""
        if self.mode == 'depth':
            return f'/depth_{base_name}/{action}'
        return f'/{base_name}/{action}'


    def set_mode_callback(self, request, response):
        """ROS2 服务: 切换模式 (SetBool: True=depth, False=2d)"""
        new_mode = 'depth' if request.data else '2d'
        old_mode = self.mode
        if old_mode == new_mode:
            response.success = True
            response.message = f'Already in {new_mode} mode'
            return response

        self.get_logger().info(f'Mode switch: {old_mode} -> {new_mode}')
        base_names = [
            'color_sorting', 'color_stack',
            'label_sorting', 'label_stack',
            'num_sorting', 'num_stack',
            'color_track', 'label_track', 'num_track',
        ]
        for base_name in base_names:
            if old_mode == 'depth':
                srv_name = f'/depth_{base_name}/exit'
            else:
                srv_name = f'/{base_name}/exit'
            try:
                client = self.create_client(Trigger, srv_name)
                if client.service_is_ready():
                    req = Trigger.Request()
                    client.call_async(req)
            except Exception:
                pass

        self.mode = new_mode
        response.success = True
        response.message = f'Switched to {new_mode} mode'
        return response

    def get_mode_callback(self, request, response):
        """ROS2 服务: 查询当前模式"""
        response.success = True
        response.message = self.mode
        return response

    def start_http_server(self):
        """启动HTTP服务器线程"""
        host = '0.0.0.0'
        
        # 创建自定义的Handler类，能够访问ROS2节点
        class ROS2StreamHandler(BaseHTTPRequestHandler):
            def __init__(self, request, client_address, server):
                self.ros_node = server.ros_node
                self.color_pub = self.ros_node.create_publisher(String, '/color', 10)
                self.label_pub = self.ros_node.create_publisher(Int32, '/label', 10)
                self.num_pub = self.ros_node.create_publisher(String, '/num', 10)
                self.csort_pub = self.ros_node.create_publisher(String, '/color_sorting/command', 10)
                self.lsort_pub = self.ros_node.create_publisher(String, '/label_sorting/sort_command', 10)
                self.nsort_pub = self.ros_node.create_publisher(String, '/num_sorting/command', 10)
                # self.cset_client = self.ros_node.create_client(Add, '/track_color')

                
                
                
                
                


                self.sorting_mode = 1
                super().__init__(request, client_address, server)
                
            
            def log_message(self, format, *args):
                """重写日志方法，使用ROS2日志系统"""
                self.ros_node.get_logger().info(f"HTTP {self.client_address[0]} - {format % args}")
            
            def set_color_threshold(self, color, low_h, low_s, low_v, high_h, high_s, high_v):
                """
                调用Add服务修改颜色阈值
                :param color: 颜色名称（'red'/'blue'/'green'）
                :param low_h/l_s/l_v: 低阈值HSV分量
                :param high_h/h_s/h_v: 高阈值HSV分量
                """
                req = Add.Request()
                req.color = color
                req.low_h = low_h
                req.low_s = low_s
                req.low_v = low_v
                req.high_h = high_h
                req.high_s = high_s
                req.high_v = high_v

                future = self.add_cli.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                
                if future.result() is not None:
                    resp = future.result()
                    if resp.success:
                        self.get_logger().info(f"✅ 成功设置{color}阈值："
                                            f"低=({low_h},{low_s},{low_v}) 高=({high_h},{high_s},{high_v})")
                        return True
                    else:
                        self.get_logger().error(f"❌ 设置{color}阈值失败：{resp.message}")
                        return False
                else:
                    self.get_logger().error(f"❌ {color}阈值服务调用超时")
                    return False

            def _auto_exit_all(self, mode):
                """向指定模式的所有节点发送 exit（尽力而为）"""
                base_names = [
                    'color_sorting', 'color_stack',
                    'label_sorting', 'label_stack',
                    'num_sorting', 'num_stack',
                    'color_track', 'label_track', 'num_track',
                ]
                for base_name in base_names:
                    if mode == 'depth':
                        srv_name = f'/depth_{base_name}/exit'
                    else:
                        srv_name = f'/{base_name}/exit'
                    try:
                        client = self.ros_node.create_client(Trigger, srv_name)
                        if client.service_is_ready():
                            req = Trigger.Request()
                            client.call_async(req)
                    except Exception:
                        pass

            def do_OPTIONS(self):
                # 处理预检请求，返回完整CORS头
                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')  # 允许所有源（开发环境）
                self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')  # 允许的方法
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')  # 允许的请求头
                self.send_header('Access-Control-Max-Age', '86400')  # 预检缓存1天，减少请求次数
                self.end_headers()
            
            def do_GET(self):
                if self.path == '/':
                    # 提供HTML页面
                    self.send_response(200)
                    self.send_header('Content-type', 'text/html; charset=utf-8')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    html = self.get_html_content()
                    self.wfile.write(html.encode('utf-8'))
                
                elif 'stream' in self.path:
                    # 视频流
                    self.send_response(200)
                    self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    
                    try:
                        while True:
                            frame = self.ros_node.camera.get_frame()
                            if frame is None:
                                time.sleep(0.1)
                                continue
                            
                            self.wfile.write(b'--frame\r\n')
                            self.send_header('Content-type', 'image/jpeg')
                            self.send_header('Content-length', len(frame))
                            self.end_headers()
                            self.wfile.write(frame)
                            self.wfile.write(b'\r\n')
                            
                            # # 如果启用，发布到ROS2话题
                            # if self.ros_node.publish_camera:
                            #     self.ros_node.publish_camera_frame()
                                
                            time.sleep(0.03)  # 约30fps
                    except Exception as e:
                        self.ros_node.get_logger().info(f"客户端断开连接: {e}")
                

                elif self.path.startswith('/control'):
                    # GET方式的控制指令处理
                    self.send_response(200)
                    command = self.path.split('/')[-1]
                    self.ros_node.get_logger().info(f"收到GET控制指令: {command}")
                    
                    # 发布到ROS2话题
                    msg = String()
                    msg.data = f"GET:{command}"
                    self.ros_node.control_publisher.publish(msg)
                    
                    response_msg = f"指令 '{command}' 已接收并发布到ROS2"
                    self.send_response(200)
                    self.send_header('Content-type', 'text/plain; charset=utf-8')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(response_msg.encode('utf-8'))
                
                else:
                    self.send_response(404)
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
            
            def do_POST(self):
                if self.path.startswith('/joint'):
                    # 关节控制指令
                    content_length = int(self.headers['Content-Length']) if 'Content-Length' in self.headers else 0
                    post_data = self.rfile.read(content_length) if content_length > 0 else b''
                    
                    try:
                        data = json.loads(post_data.decode('utf-8'))
                        joint = data.get('joint')
                        angle = int(data.get('angle'))
                        
                        self.ros_node.get_logger().info(f"收到关节控制指令: 关节={joint}, 角度={angle}")
                        
                        # 发布到ROS2话题
                        joint_msg = String()
                        if joint == 255:
                            joint_msg.data = f"#255P1500T2000!"
                            self.ros_node.get_logger().error(joint_msg.data)
                            self.ros_node.joint_control_publisher.publish(joint_msg) 
                        else:
                            if angle == 3000:
                                joint_msg.data = f"#00{joint-1}PDST!"
                            elif angle == 2400:
                                joint_msg.data = f"#00{joint-1}P{angle}T2000!"
                            elif angle == 600:
                                joint_msg.data = f"#00{joint-1}P0{angle}T2000!"
                            elif angle < 1000:
                                joint_msg.data = f"#00{joint-1}P0{angle}T0500!"
                            else:
                                joint_msg.data = f"#00{joint-1}P{angle}T0500!"
                            self.ros_node.get_logger().error(joint_msg.data)
                            self.ros_node.joint_control_publisher.publish(joint_msg)
                        
                        response_msg = "OK"
                    except Exception as e:
                        self.ros_node.get_logger().error(f"处理关节指令时出错: {e}")
                        response_msg = f"错误: {str(e)}"
                    
                    self.send_response(200)
                    self.send_header('Content-type', 'text/plain; charset=utf-8')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(response_msg.encode('utf-8'))

                elif self.path.startswith('/set'):
                    content_length = int(self.headers['Content-Length']) if 'Content-Length' in self.headers else 0
                    post_data = self.rfile.read(content_length) if content_length > 0 else b''
                    def service_callback(fut):
                        try:
                            response = fut.result()
                            self.ros_node.get_logger().info(f"ROS阈值设置成功：{response}")
                        except Exception as e:
                            self.ros_node.get_logger().error(f"ROS服务调用失败：{e}")
                    try:
                        # 1. 解析POST JSON数据
                        data = json.loads(post_data.decode('utf-8'))
                        block_data = data.get('block')
                        if not block_data:
                            raise ValueError("请求体缺少必填字段'block'")
                        
                        # 2. 提取并验证参数
                        # 必填字段检查
                        required_fields = ['L_min', 'L_max', 'A_min', 'A_max', 'B_min', 'B_max', 'color']
                        missing_fields = [field for field in required_fields if block_data.get(field) is None]
                        if missing_fields:
                            raise ValueError(f"缺少必填字段：{','.join(missing_fields)}")
                        
                        # 提取参数
                        L_min = block_data['L_min']
                        L_max = block_data['L_max']
                        A_min = block_data['A_min']
                        A_max = block_data['A_max']
                        B_min = block_data['B_min']
                        B_max = block_data['B_max']
                        color = block_data['color'].lower()  # 统一小写，避免大小写问题
                        
                        # 验证颜色有效性（支持red/green/blue）
                        supported_colors = ['red', 'green', 'blue','yellow','purple']
                        if color not in supported_colors:
                            raise ValueError(f"不支持的颜色：{color}，可选值：{','.join(supported_colors)}")
                        
                        # 验证数值类型和范围（假设是0-255的整数，符合HSV/Lab等空间的常见范围）
                        int_fields = {
                            'L_min': L_min, 'L_max': L_max,
                            'A_min': A_min, 'A_max': A_max,
                            'B_min': B_min, 'B_max': B_max
                        }
                        for field, value in int_fields.items():
                            if not isinstance(value, int):
                                raise ValueError(f"{field}必须是整数")
                            if not (0 <= value <= 255):
                                raise ValueError(f"{field}必须在0-255之间")
                        
                        # # 验证min ≤ max
                        # if L_min > L_max or A_min > A_max or B_min > B_max:
                        #     raise ValueError("min值不能大于max值")
                        
                        # 3. 更新ROS节点的颜色阈值（修改这部分代码）
                        self.ros_node.get_logger().info(
                            f"收到{color}颜色阈值设置：L[{L_min}-{L_max}], A[{A_min}-{A_max}], B[{B_min}-{B_max}]"
                        )
                        client = self.ros_node.create_client(Add, '/Add')
                        req = Add.Request()
                        req.color = color
                        req.low_h = L_min
                        req.low_s = A_min
                        req.low_v = B_min
                        req.high_h = L_max
                        req.high_s = A_max
                        req.high_v = B_max

                        self.ros_node.get_logger().info(f"收到color: {color}")
                        _DATA_DIR = os.path.expanduser('~/ros2_ws')
                        if color == 'red':
                            with open(os.path.join(_DATA_DIR, 'red.txt'), 'w') as f_red:
                                f_red.write(f"{L_min} {A_min} {B_min} {L_max} {A_max} {B_max}")
                        elif color == 'blue':
                            with open(os.path.join(_DATA_DIR, 'blue.txt'), 'w') as f_blue:
                                f_blue.write(f"{L_min} {A_min} {B_min} {L_max} {A_max} {B_max}")
                        elif color == 'green':
                            with open(os.path.join(_DATA_DIR, 'green.txt'), 'w') as f_green:
                                f_green.write(f"{L_min} {A_min} {B_min} {L_max} {A_max} {B_max}")

                        # 关键修改：异步调用ROS服务，不阻塞HTTP请求处理
                        if not client.service_is_ready():
                            # 等待服务就绪（最多等1秒，避免无限阻塞）
                            client.wait_for_service(timeout_sec=1.0)
                            if not client.service_is_ready():
                                raise RuntimeError(f"ROS服务/Add未就绪，无法设置阈值")

                        # 发送请求（不阻塞，直接返回）
                        future = client.call_async(req)
                        # 给future添加回调（ROS事件循环会自动处理）
                        #future.add_done_callback(service_callback)

                        # 4. 直接返回HTTP成功响应（不需要等ROS服务执行完）
                        self.send_response(200)
                        self.send_header('Content-type', 'text/plain; charset=utf-8')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write("OK".encode('utf-8'))
                    except json.JSONDecodeError:
                        # JSON格式错误（添加CORS头）
                        self.send_response(400)
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write("错误：无效的JSON格式".encode('utf-8'))
                    except ValueError as ve:
                        # 参数验证失败（添加CORS头）
                        self.send_response(400)
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(f"错误：{str(ve)}".encode('utf-8'))
                    except Exception as e:
                        # 未知错误（添加CORS头+日志）
                        self.ros_node.get_logger().error(f"处理/set请求失败：{str(e)}")
                        self.send_response(500)
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(f"内部错误：{str(e)}".encode('utf-8'))


                elif self.path == '/set_mode':
                    content_length = int(self.headers['Content-Length']) if 'Content-Length' in self.headers else 0
                    post_data = self.rfile.read(content_length) if content_length > 0 else b''
                    try:
                        data = json.loads(post_data.decode('utf-8'))
                        new_mode = data.get('mode', '').lower()
                        if new_mode not in ('2d', 'depth'):
                            self.send_response(400)
                            self.send_header('Content-type', 'text/plain; charset=utf-8')
                            self.send_header('Access-Control-Allow-Origin', '*')
                            self.end_headers()
                            self.wfile.write("Invalid mode. Use '2d' or 'depth'.".encode('utf-8'))
                            return

                        old_mode = self.ros_node.mode
                        if old_mode == new_mode:
                            response_msg = f"Already in {new_mode} mode"
                        else:
                            self.ros_node.get_logger().info(f'Mode switch: {old_mode} -> {new_mode}')
                            self._auto_exit_all(old_mode)
                            self.ros_node.mode = new_mode
                            response_msg = f"Switched to {new_mode} mode"

                        self.send_response(200)
                        self.send_header('Content-type', 'text/plain; charset=utf-8')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(response_msg.encode('utf-8'))
                    except Exception as e:
                        self.send_response(500)
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(f"Error: {e}".encode('utf-8'))

                elif self.path == '/get_mode':
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(json.dumps({'mode': self.ros_node.mode}).encode('utf-8'))

                elif self.path.startswith('/control'):
                    # 通用控制指令
                    content_length = int(self.headers['Content-Length']) if 'Content-Length' in self.headers else 0
                    post_data = self.rfile.read(content_length) if content_length > 0 else b''

                    try:
                        command_str = post_data.decode('utf-8').strip()
                        self.ros_node.get_logger().info(f"收到POST控制指令: {command_str}")

                        # 命令→服务映射表 (base_name, action)
                        COMMAND_MAP = {
                            'color_sorting_enter': ('color_sorting', 'enter'),
                            'sort stop color':     ('color_sorting', 'exit'),
                            'color_stack_enter':   ('color_stack', 'enter'),
                            'color_stack_exit':    ('color_stack', 'exit'),
                            'label_sorting_enter': ('label_sorting', 'enter'),
                            'sort_stop_code':      ('label_sorting', 'exit'),
                            'label_stack_enter':   ('label_stack', 'enter'),
                            'label_stack_exit':    ('label_stack', 'exit'),
                            'num_sorting_enter':   ('num_sorting', 'enter'),
                            'sort_stop_num':       ('num_sorting', 'exit'),
                            'num_stack_enter':     ('num_stack', 'enter'),
                            'num_stack_exit':      ('num_stack', 'exit'),
                            'color_set_enter':     ('color_set', 'enter'),
                            'set_off':             ('color_set', 'exit'),
                            'track on':            ('color_track', 'enter'),
                            'track off':           ('color_track', 'exit'),
                            'label_track_on':      ('label_track', 'enter'),
                            'label_track_off':     ('label_track', 'exit'),
                            'num_track on':        ('num_track', 'enter'),
                            'num_track off':       ('num_track', 'exit'),
                            'face_track_on':       ('face_track', 'enter'),
                            'face_track_off':      ('face_track', 'exit'),
                            'uart_open':           ('servo', 'enter'),
                            'uart_close':          ('servo', 'exit'),
                        }

                        # 话题命令（不需要模式路由，直接发布话题）
                        TOPIC_COMMANDS = {
                            'track red':    (self.color_pub, String, 'red'),
                            'track green':  (self.color_pub, String, 'green'),
                            'track blue':   (self.color_pub, String, 'blue'),
                            'track label1': (self.label_pub, Int32, 1),
                            'track label2': (self.label_pub, Int32, 2),
                            'track label3': (self.label_pub, Int32, 3),
                            'num_track 1':  (self.num_pub, String, '1'),
                            'num_track 2':  (self.num_pub, String, '2'),
                            'num_track 3':  (self.num_pub, String, '3'),
                        }

                        if command_str in COMMAND_MAP:
                            base_name, action = COMMAND_MAP[command_str]
                            srv_name = self.ros_node.get_service_name(base_name, action)
                            client = self.ros_node.create_client(Trigger, srv_name)
                            req = Trigger.Request()
                            if not client.service_is_ready():
                                client.wait_for_service(timeout_sec=1.0)
                                if not client.service_is_ready():
                                    raise RuntimeError(f"ROS服务 {srv_name} 未就绪")
                            future = client.call_async(req)
                            response_msg = f"{command_str} OK (mode={self.ros_node.mode})"

                        elif command_str in TOPIC_COMMANDS:
                            pub, msg_type, data = TOPIC_COMMANDS[command_str]
                            msg = msg_type()
                            msg.data = data
                            pub.publish(msg)
                            response_msg = f"{command_str} OK"

                        else:
                            if self.sorting_mode == 1:
                                msg = String(data=command_str)
                                self.csort_pub.publish(msg)
                                self.lsort_pub.publish(msg)
                                self.nsort_pub.publish(msg)
                                response_msg = "OK"
                            else:
                                response_msg = "buOK"

                    except Exception as e:
                        self.ros_node.get_logger().error(f"处理控制指令时出错: {e}")
                        response_msg = f"处理控制指令时出错: {e}"

                    self.send_response(200)
                    self.send_header('Content-type', 'text/plain; charset=utf-8')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(response_msg.encode('utf-8'))
                
                else:
                    self.send_response(404)
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
            
            def get_html_content(self):
                """生成HTML页面内容"""
                local_ip = self.get_local_ip()
                port = self.ros_node.http_port
                return f"""
                <html>
                    <head>
                        <title>ROS2摄像头直播</title>
                        <meta charset="utf-8">
                        <style>
                            body {{ 
                                font-family: Arial, sans-serif; 
                                margin: 0; 
                                padding: 20px; 
                                background-color: #f5f5f5;
                            }}
                            .container {{ 
                                max-width: 1200px; 
                                margin: 0 auto; 
                                background: white; 
                                padding: 20px; 
                                border-radius: 10px;
                                box-shadow: 0 2px 10px rgba(0,0,0,0.1);
                            }}
                            h1 {{ 
                                color: #333; 
                                text-align: center; 
                                margin-bottom: 30px;
                            }}
                            .video-container {{
                                text-align: center;
                                margin-bottom: 30px;
                            }}
                            img {{ 
                                max-width: 100%; 
                                border: 3px solid #ddd;
                                border-radius: 5px;
                            }}
                            .controls {{
                                display: flex;
                                flex-wrap: wrap;
                                gap: 10px;
                                justify-content: center;
                                margin-bottom: 20px;
                            }}
                            button {{
                                padding: 10px 20px;
                                background: #007bff;
                                color: white;
                                border: none;
                                border-radius: 5px;
                                cursor: pointer;
                                font-size: 16px;
                            }}
                            button:hover {{
                                background: #0056b3;
                            }}
                            .info {{
                                background: #e9ecef;
                                padding: 15px;
                                border-radius: 5px;
                                margin-top: 20px;
                            }}
                            .joint-control {{
                                margin-top: 20px;
                                padding: 15px;
                                background: #f8f9fa;
                                border-radius: 5px;
                            }}
                            .joint-control input {{
                                padding: 5px;
                                margin: 0 5px;
                                width: 80px;
                            }}
                        </style>
                    </head>
                    <body>
                        <div class="container">
                            <h1>ROS2摄像头HTTP服务器</h1>
                            
                            <div class="video-container">
                                <img src="/stream" />
                            </div>
                            
                            <div class="controls">
                                <button onclick="sendControl('MOVE_FORWARD')">前进</button>
                                <button onclick="sendControl('MOVE_BACKWARD')">后退</button>
                                <button onclick="sendControl('TURN_LEFT')">左转</button>
                                <button onclick="sendControl('TURN_RIGHT')">右转</button>
                                <button onclick="sendControl('STOP')">停止</button>
                            </div>
                            
                            <div class="joint-control">
                                <h3>关节控制</h3>
                                <input type="number" id="jointId" placeholder="关节ID" value="1" min="1" max="10">
                                <input type="number" id="jointAngle" placeholder="角度" value="90" min="0" max="180">
                                <button onclick="controlJoint()">控制关节</button>
                            </div>
                            
                            <div class="info">
                                <p><strong>服务器地址:</strong> http://{local_ip}:{port}</p>
                                <p><strong>视频流地址:</strong> http://{local_ip}:{port}/stream</p>
                                <p><strong>ROS2节点:</strong> {self.ros_node.get_name()}</p>
                            </div>
                        </div>
                        
                        <script>
                            function sendControl(command) {{
                                fetch(`/api/control/${{command}}`)
                                    .then(response => response.text())
                                    .then(data => console.log('控制响应:', data))
                                    .catch(error => console.error('错误:', error));
                            }}
                            
                            function controlJoint() {{
                                const joint = document.getElementById('jointId').value;
                                const angle = document.getElementById('jointAngle').value;
                                
                                fetch('/joint', {{
                                    method: 'POST',
                                    headers: {{
                                        'Content-Type': 'application/json',
                                    }},
                                    body: JSON.stringify({{joint: joint, angle: angle}})
                                }})
                                .then(response => response.text())
                                .then(data => console.log('关节控制响应:', data))
                                .catch(error => console.error('错误:', error));
                            }}
                        </script>
                    </body>
                </html>
                """.encode('utf-8')
            
            def get_local_ip(self):
                """获取本地IP地址"""
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.connect(("8.8.8.8", 8080))
                    ip = s.getsockname()[0]
                    s.close()
                    return ip
                except:
                    return "127.0.0.1"
        
        # 多线程HTTP服务器
        class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
            def __init__(self, server_address, RequestHandlerClass, ros_node):
                self.ros_node = ros_node
                super().__init__(server_address, RequestHandlerClass)
        
        try:
            self.server = ThreadedHTTPServer((host, self.http_port), ROS2StreamHandler, self)
            self.server_thread = threading.Thread(target=self.server.serve_forever)
            self.server_thread.daemon = True
            self.server_thread.start()
            self.get_logger().info(f"HTTP服务器已在 {host}:{self.http_port} 启动")
        except Exception as e:
            self.get_logger().error(f"HTTP服务器启动失败: {e}")
    
    def publish_camera_frame(self):
        """发布摄像头帧到ROS2话题"""
        if not hasattr(self, 'bridge') or self.camera.frame is None:
            return
        
        try:
            # 转换OpenCV图像为ROS2 Image消息
            ros_image = self.bridge.cv2_to_imgmsg(self.camera.frame, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_frame"
            self.image_publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"发布图像时出错: {e}")
    
    def _wait_for_service(self, client, service_name: str, timeout_sec=5.0):
        """等待服务就绪"""
        while not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(f"{service_name}服务未就绪，重试中...")

    def destroy_node(self):
        """重写销毁节点方法，确保资源正确释放"""
        self.get_logger().info("正在关闭摄像头HTTP服务器节点...")
        
        # 停止摄像头
        if hasattr(self, 'camera'):
            self.camera.stop()
        
        # 关闭HTTP服务器
        if self.server:
            self.server.shutdown()
            self.server.server_close()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraHTTPServerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行出错: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()