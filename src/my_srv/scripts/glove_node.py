#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import serial
import threading
import time
from std_srvs.srv import Trigger
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Vector3

from example_interfaces.srv import Trigger

class GLOVENode(Node):
    def __init__(self):
        super().__init__('glove_node')
        
        # 声明参数
        self.declare_parameter('port', '/dev/ttyAMA1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        
        # 获取参数
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        
        # 初始化串口
        self.serial_port = None
        self.serial_lock = threading.Lock()
        self.connect_serial()

        
        # 状态变量
        self.connection_status = "Disconnected"
        self.last_received_time = time.time()
        self.rx_count = 0
        self.tx_count = 0

        self.receive_data = None
        
        # 启动接收线程
        self.running = True
        self.receive_thread = threading.Thread(target=self.receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()


        self.joint_control_publisher = self.create_publisher(String, 'joint_commands', 10)
        
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
                        joint_msg = String()
                        joint_msg.data = command_str
                        self.joint_control_publisher.publish(joint_msg)
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
    
    node = GLOVENode()
    
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