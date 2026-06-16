#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import serial
import subprocess
import threading
import time
from std_srvs.srv import Trigger
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Vector3

from my_srv.srv import Add
from example_interfaces.srv import Trigger


def detect_camera():
    """检测USB相机类型，返回 'depth' 或 'mono'"""
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=3)
        if '3251:1930' in result.stdout:
            return 'depth'
    except Exception:
        pass
    return 'mono'

class ASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')

        # 检测相机类型，决定使用2D还是深度节点
        self.camera_mode = detect_camera()
        self.prefix = 'depth_' if self.camera_mode == 'depth' else ''
        self.get_logger().info(f'相机模式: {self.camera_mode}，路径前缀: "{self.prefix}"')

        # 声明参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('send_topic', '/serial/send')
        self.declare_parameter('receive_topic', '/serial/receive')
        self.declare_parameter('status_topic', '/serial/status')
        
        # 获取参数
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.send_topic = self.get_parameter('send_topic').get_parameter_value().string_value
        self.receive_topic = self.get_parameter('receive_topic').get_parameter_value().string_value
        self.status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        
        # 初始化串口
        self.serial_port = None
        self.serial_lock = threading.Lock()
        self.connect_serial()
        
        # 创建发布者
        qos_profile = QoSProfile(depth=10)
        self.receive_pub = self.create_publisher(String, self.receive_topic, qos_profile)
        self.status_pub = self.create_publisher(String, self.status_topic, qos_profile)
        
        # 创建订阅者
        self.send_sub = self.create_subscription(
            String, 
            self.send_topic, 
            self.send_callback, 
            qos_profile
        )
        
        # 创建服务
        self.send_service = self.create_service(
            Trigger, 
            'serial/send_string', 
            self.send_string_service
        )
        
        self.status_service = self.create_service(
            Trigger, 
            'serial/get_status', 
            self.get_status_service
        )
        
        # 状态变量
        self.connection_status = "Disconnected"
        self.last_received_time = time.time()
        self.rx_count = 0
        self.tx_count = 0

        self.receive_data = None
        self._last_log_time = 0.0
        
        self.color_pub = self.create_publisher(String, '/color', 10)
        self.label_pub = self.create_publisher(Int32, '/label', 10)
        self.num_pub = self.create_publisher(String, '/num', 10)
        self.csort_pub = self.create_publisher(String, '/color_sorting/command', 10)
        self.lsort_pub = self.create_publisher(String, '/label_sorting/sort_command', 10)
        self.nsort_pub = self.create_publisher(String, '/num_sorting/command', 10)
        self.joint_control_publisher = self.create_publisher(String, 'joint_commands', 10)

        # 启动接收线程
        self.running = True
        self.receive_thread = threading.Thread(target=self.receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        # 启动状态监控定时器
        self.status_timer = self.create_timer(1.0, self.publish_status)

        
        self.get_logger().info(f"串口通信节点已启动，端口: {self.port}, 波特率: {self.baudrate}")

    def connect_serial(self):
        """连接串口设备"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            if self.serial_port.is_open:
                self.connection_status = "Connected"
                self.get_logger().info(f"成功连接到串口: {self.port}")
                return True
            else:
                self.connection_status = "Connection failed"
                self.get_logger().error(f"无法打开串口: {self.port}")
                return False
        except serial.SerialException as e:
            self.connection_status = f"Error: {str(e)}"
            self.get_logger().error(f"串口连接错误: {str(e)}")
            return False

    def reconnect_serial(self):
        """重新连接串口"""
        self.get_logger().warn("尝试重新连接串口...")
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        time.sleep(1.0)
        return self.connect_serial()

    def send_data(self, data):
        """发送数据到串口"""
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().warn("串口未连接，尝试重新连接...")
            if not self.reconnect_serial():
                return False
        
        try:
            with self.serial_lock:
                self.serial_port.write(data.encode('utf-8'))
                self.tx_count += 1
                self.get_logger().debug(f"发送数据: {data.strip()}")
                return True
        except serial.SerialException as e:
            self.get_logger().error(f"发送数据失败: {str(e)}")
            self.connection_status = f"Send error: {str(e)}"
            return False

    def receive_loop(self):
        """接收数据的线程函数"""
        while self.running:
            if not self.serial_port or not self.serial_port.is_open:
                time.sleep(0.5)
                continue
                
            try:
                with self.serial_lock:
                    if self.serial_port.in_waiting > 0:
                        command_str = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                        now = time.time()
                        if now - self._last_log_time >= 1.0:
                            self.get_logger().info(f"[ASR收到] {repr(command_str)}")
                            self._last_log_time = now
                        if command_str.startswith('{') and command_str.endswith('}'):
                            joint_msg = String()
                            joint_msg.data = command_str
                            self.joint_control_publisher.publish(joint_msg)
                        elif command_str == "color_sorting_enter":
                            csort_on_client = self.create_client(Trigger, f'/{self.prefix}color_sorting/enter')
                            req = Trigger.Request()
                            if not csort_on_client.service_is_ready():
                                csort_on_client.wait_for_service(timeout_sec=1.0)
                                if not csort_on_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = csort_on_client.call_async(req)

                        elif command_str == "sort stop color":
                            self.sorting_mode = 0
                            csort_off_client = self.create_client(Trigger, f'/{self.prefix}color_sorting/exit')
                            req = Trigger.Request()
                            if not csort_off_client.service_is_ready():
                                csort_off_client.wait_for_service(timeout_sec=1.0)
                                if not csort_off_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = csort_off_client.call_async(req)
                            response_msg = "sort stop color"
                        elif command_str in ("sort red", "sort green", "sort blue",
                                             "sort red green", "sort red blue", "sort green blue",
                                             "sort red green blue"):
                            csort_enter = self.create_client(Trigger, f'/{self.prefix}color_sorting/enter')
                            if csort_enter.service_is_ready() or csort_enter.wait_for_service(timeout_sec=1.0):
                                req = Trigger.Request()
                                csort_enter.call_async(req)
                            sort_msg = String()
                            sort_msg.data = command_str
                            self.csort_pub.publish(sort_msg)
                            response_msg = f"{command_str} OK"
                        elif command_str == "color_stack_enter":
                            cstack_on_client = self.create_client(Trigger, f'/{self.prefix}color_stack/enter')
                            req = Trigger.Request()
                            if not cstack_on_client.service_is_ready():
                                cstack_on_client.wait_for_service(timeout_sec=1.0)
                                if not cstack_on_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = cstack_on_client.call_async(req)
                            response_msg = "color_stack_enter OK"
                        elif command_str == "color_stack_exit":
                            cstack_off_client = self.create_client(Trigger, f'/{self.prefix}color_stack/exit')
                            req = Trigger.Request()
                            if not cstack_off_client.service_is_ready():
                                cstack_off_client.wait_for_service(timeout_sec=1.0)
                                if not cstack_off_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = cstack_off_client.call_async(req)
                            response_msg = "color_stack_exit OK"
                        elif command_str == "label_sorting_enter":
                            lsort_on_client = self.create_client(Trigger, f'/{self.prefix}label_sorting/enter')
                            req = Trigger.Request()
                            if not lsort_on_client.service_is_ready():
                                lsort_on_client.wait_for_service(timeout_sec=1.0)
                                if not lsort_on_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = lsort_on_client.call_async(req)
                            response_msg = "label_sorting_enter OK"
                        elif command_str == "sort_stop_code":
                            lsort_off_client = self.create_client(Trigger, f'/{self.prefix}label_sorting/exit')
                            req = Trigger.Request()
                            if not lsort_off_client.service_is_ready():
                                lsort_off_client.wait_for_service(timeout_sec=1.0)
                                if not lsort_off_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = lsort_off_client.call_async(req)
                            response_msg = "sort_stop_code OK"
                        elif command_str == "label_stack_enter":
                            lstack_on_client = self.create_client(Trigger, f'/{self.prefix}label_stack/enter')
                            req = Trigger.Request()
                            if not lstack_on_client.service_is_ready():
                                lstack_on_client.wait_for_service(timeout_sec=1.0)
                                if not lstack_on_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = lstack_on_client.call_async(req)
                        elif command_str == "label_stack_exit":
                            lstack_off_client = self.create_client(Trigger, f'/{self.prefix}label_stack/exit')
                            req = Trigger.Request()
                            if not lstack_off_client.service_is_ready():
                                lstack_off_client.wait_for_service(timeout_sec=1.0)
                                if not lstack_off_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = lstack_off_client.call_async(req)
                        elif command_str == "num_sorting_enter":
                            nsort_on_client = self.create_client(Trigger, f'/{self.prefix}num_sorting/enter')
                            req = Trigger.Request()
                            if not nsort_on_client.service_is_ready():
                                nsort_on_client.wait_for_service(timeout_sec=1.0)
                                if not nsort_on_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = nsort_on_client.call_async(req)
                        elif command_str == "sort_stop_num":
                            nsort_off_client = self.create_client(Trigger, f'/{self.prefix}num_sorting/exit')
                            req = Trigger.Request()
                            if not nsort_off_client.service_is_ready():
                                nsort_off_client.wait_for_service(timeout_sec=1.0)
                                if not nsort_off_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = nsort_off_client.call_async(req)
                        elif command_str == "num_stack_enter":
                            nstack_on_client = self.create_client(Trigger, f'/{self.prefix}num_stack/enter')
                            req = Trigger.Request()
                            if not nstack_on_client.service_is_ready():
                                nstack_on_client.wait_for_service(timeout_sec=1.0)
                                if not nstack_on_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = nstack_on_client.call_async(req)
                        elif command_str == "num_stack_exit":
                            nstack_off_client = self.create_client(Trigger, f'/{self.prefix}num_stack/exit')
                            req = Trigger.Request()
                            if not nstack_off_client.service_is_ready():
                                nstack_off_client.wait_for_service(timeout_sec=1.0)
                                if not nstack_off_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = nstack_off_client.call_async(req)
                        elif command_str == "color_set_enter":
                            cset_on_client = self.create_client(Trigger, '/color_set/enter')
                            req = Trigger.Request()
                            if not cset_on_client.service_is_ready():
                                cset_on_client.wait_for_service(timeout_sec=1.0)
                                if not cset_on_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务/Add未就绪，无法设置阈值")
                            future = cset_on_client.call_async(req)
                        elif command_str == "set_off":
                            cset_off_client = self.create_client(Trigger, '/color_set/exit')
                            req = Trigger.Request()
                            if not cset_off_client.service_is_ready():
                                cset_off_client.wait_for_service(timeout_sec=1.0)
                                if not cset_off_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务/Add未就绪，无法设置阈值")
                            future = cset_off_client.call_async(req)
                        elif command_str == "track on":
                            ctrack_on_client = self.create_client(Trigger, f'/{self.prefix}color_track/enter')
                            req = Trigger.Request()
                            if not ctrack_on_client.service_is_ready():
                                ctrack_on_client.wait_for_service(timeout_sec=1.0)
                                if not ctrack_on_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = ctrack_on_client.call_async(req)
                        elif command_str == "track off":
                            ctrack_off_client = self.create_client(Trigger, f'/{self.prefix}color_track/exit')
                            req = Trigger.Request()
                            if not ctrack_off_client.service_is_ready():
                                ctrack_off_client.wait_for_service(timeout_sec=1.0)
                                if not ctrack_off_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = ctrack_off_client.call_async(req)
                        elif command_str == "track red":
                            msg = String(data='red')
                            self.color_pub.publish(msg)
                        elif command_str == "track green":
                            msg = String(data='green')
                            self.color_pub.publish(msg)
                        elif command_str == "track blue":
                            msg = String(data='blue')
                            self.color_pub.publish(msg)                   
                        elif command_str == "track labela":
                            msg = Int32()
                            msg.data = 1 
                            self.label_pub.publish(msg)                   
                        elif command_str == "track labelb":
                            msg = Int32()
                            msg.data = 2 
                            self.label_pub.publish(msg)                   
                        elif command_str == "track labelc":
                            msg = Int32()
                            msg.data = 3
                            self.label_pub.publish(msg)                   
                        elif command_str == "num_track a":
                            msg = String(data='1')
                            self.num_pub.publish(msg)                   
                        elif command_str == "num_track b":
                            msg = String(data='2')
                            self.num_pub.publish(msg)                   
                        elif command_str == "num_track c":
                            msg = String(data='3')
                            self.num_pub.publish(msg)                   
                            response_msg = "num_track 3 OK"
                        elif command_str == "uart_open":
                            ctrack_off_client = self.create_client(Trigger, '/servo/enter')
                            req = Trigger.Request()
                            if not ctrack_off_client.service_is_ready():
                                ctrack_off_client.wait_for_service(timeout_sec=1.0)
                                if not ctrack_off_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务/Add未就绪，无法设置阈值")
                            future = ctrack_off_client.call_async(req)
                        elif command_str == "uart_close":
                            ctrack_off_client = self.create_client(Trigger, '/servo/exit')
                            req = Trigger.Request()
                            if not ctrack_off_client.service_is_ready():
                                ctrack_off_client.wait_for_service(timeout_sec=1.0)
                                if not ctrack_off_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务/Add未就绪，无法设置阈值")
                            future = ctrack_off_client.call_async(req)
                        elif command_str == "face_track_on":
                            ftrack_on_client = self.create_client(Trigger, '/face_track/enter')
                            req = Trigger.Request()
                            if not ftrack_on_client.service_is_ready():
                                ftrack_on_client.wait_for_service(timeout_sec=1.0)
                                if not ftrack_on_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务/Add未就绪，无法设置阈值")
                            future = ftrack_on_client.call_async(req)
                        elif command_str == "face_track_off":
                            ftrack_off_client = self.create_client(Trigger, '/face_track/exit')
                            req = Trigger.Request()
                            if not ftrack_off_client.service_is_ready():
                                ftrack_off_client.wait_for_service(timeout_sec=1.0)
                                if not ftrack_off_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务/Add未就绪，无法设置阈值")
                            future = ftrack_off_client.call_async(req)                                                     
                        elif command_str == "label_track_on":
                            ftrack_on_client = self.create_client(Trigger, f'/{self.prefix}label_track/enter')
                            req = Trigger.Request()
                            if not ftrack_on_client.service_is_ready():
                                ftrack_on_client.wait_for_service(timeout_sec=1.0)
                                if not ftrack_on_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = ftrack_on_client.call_async(req)
                        elif command_str == "label_track_off":
                            ftrack_off_client = self.create_client(Trigger, f'/{self.prefix}label_track/exit')
                            req = Trigger.Request()
                            if not ftrack_off_client.service_is_ready():
                                ftrack_off_client.wait_for_service(timeout_sec=1.0)
                                if not ftrack_off_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = ftrack_off_client.call_async(req)
                        elif command_str == "num_track on":
                            ftrack_on_client = self.create_client(Trigger, f'/{self.prefix}num_track/enter')
                            req = Trigger.Request()
                            if not ftrack_on_client.service_is_ready():
                                ftrack_on_client.wait_for_service(timeout_sec=1.0)
                                if not ftrack_on_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = ftrack_on_client.call_async(req)
                        elif command_str == "num_track off":
                            ftrack_off_client = self.create_client(Trigger, f'/{self.prefix}num_track/exit')
                            req = Trigger.Request()
                            if not ftrack_off_client.service_is_ready():
                                ftrack_off_client.wait_for_service(timeout_sec=1.0)
                                if not ftrack_off_client.service_is_ready():
                                    raise RuntimeError(f"ROS服务未就绪")
                            future = ftrack_off_client.call_async(req)

                        elif command_str in ("sort_1", "sort_2", "sort_3",
                                             "sort_1 sort_2", "sort_1 sort_3", "sort_2 sort_3",
                                             "sort_1 sort_2 sort_3"):
                            lsort_enter = self.create_client(Trigger, f'/{self.prefix}label_sorting/enter')
                            if lsort_enter.service_is_ready() or lsort_enter.wait_for_service(timeout_sec=1.0):
                                lsort_enter.call_async(Trigger.Request())
                            lsort_msg = String()
                            lsort_msg.data = command_str
                            self.lsort_pub.publish(lsort_msg)
                            response_msg = f"{command_str} OK"

                        elif command_str.startswith("sort_num") and command_str != "sort_stop_num":
                            nsort_enter = self.create_client(Trigger, f'/{self.prefix}num_sorting/enter')
                            if nsort_enter.service_is_ready() or nsort_enter.wait_for_service(timeout_sec=1.0):
                                nsort_enter.call_async(Trigger.Request())
                            nsort_msg = String()
                            nsort_msg.data = command_str
                            self.nsort_pub.publish(nsort_msg)
                            response_msg = f"{command_str} OK"

                        else:
                            # 处理唤醒词应答（zai等）和其他未知命令
                            self.get_logger().info(f"[ASR] 已忽略: {repr(command_str)}")
                        # if data:
                        #     self.last_received_time = time.time()
                        #     self.rx_count += 1
                        #     self.get_logger().debug(f"接收数据: {data}")
                            
                        #     # 发布接收到的数据
                        #     msg = String()
                        #     msg.data = data
                        #     self.receive_pub.publish(msg)




            except serial.SerialException as e:
                self.get_logger().error(f"接收数据错误: {str(e)}")
                self.connection_status = f"Receive error: {str(e)}"
                time.sleep(0.5)
            except UnicodeDecodeError:
                self.get_logger().warn("接收数据解码错误")
            
            time.sleep(0.01)  # 避免CPU占用过高

    def publish_status(self):
        """发布状态信息"""
        status_msg = String()
        status_msg.data = (
            f"Status: {self.connection_status}, "
            f"RX: {self.rx_count}, TX: {self.tx_count}, "
            f"Last RX: {time.time() - self.last_received_time:.1f}s ago"
        )
        self.status_pub.publish(status_msg)

    def send_callback(self, msg):
        """发送话题回调函数"""
        self.send_data(msg.data + '\n')  # 添加换行符作为结束符

    def send_string_service(self, request, response):
        """发送字符串服务回调函数"""
        if self.send_data(request.data + '\n'):
            response.success = True
            response.message = "Data sent successfully"
        else:
            response.success = False
            response.message = f"Failed to send data: {self.connection_status}"
        return response

    def get_status_service(self, request, response):
        """获取状态服务回调函数"""
        response.success = self.serial_port and self.serial_port.is_open
        response.message = self.connection_status
        return response

    def shutdown(self):
        """关闭节点时的清理操作"""
        self.get_logger().info("正在关闭串口通信节点...")
        self.running = False
        
        if self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("串口已关闭")

def main(args=None):
    rclpy.init(args=args)
    
    node = ASRNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()