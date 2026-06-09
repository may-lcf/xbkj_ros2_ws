#!/usr/bin/env python3

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32
import math 


import rclpy
from rclpy.node import Node
import serial
import threading
import time

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port1', '/dev/usb_port2'),
                ('port2', '/dev/usb_port3'),
                ('cmd', '$DGT:0-12,1!'),
                ('baudrate', 115200),
                ('timeout', 0.1),
                ('debug', False)
            ]
        )
        
        # 获取参数值
        port1 = self.get_parameter('port1').value
        port2 = self.get_parameter('port2').value
        self.cmd = self.get_parameter('cmd').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        debug = self.get_parameter('debug').value

        self.num_Sub = self.create_subscription(Int32, '/num_cmd', self.num_cmd_Callback, 10) 
        self.aim_Sub = self.create_subscription(Int32, '/aim_cmd', self.aim_cmd_Callback, 10) 

        self.Cmd_Vel_Sub = self.create_subscription(Twist, '/cmd_vel', self.Cmd_Vel_Callback, 10) 
        self.get_logger().info(f"Starting serial bridge: {port1} -> {port2} @ {baudrate} baud")
        self.message =None
        self.last_message = None

        self.date = None

        self.numsub = 0
        self.LR_aim=0
   
        self.output_gyroz = 0.0   
        self.output_speedX = 0.0  
        self.target_gyroz = 0.0
        self.target_speedX = 0.0
        self.output_speedY = 0.0  
        self.target_speedY = 0.0
        try:
            # 初始化串口
            self.serial1 = serial.Serial(
                port=port1,
                baudrate=baudrate,
                timeout=timeout
            )
            
            self.serial2 = serial.Serial(
                port=port2,
                baudrate=baudrate,
                timeout=timeout
            )
            
            self.get_logger().info(f"Serial ports opened successfully")
            
            # 创建转发线程
            self.running = True
            self.thread = threading.Thread(target=self._forward_data)
            self.thread.daemon = True
            self.thread.start()

            self.car_control_thread = threading.Thread(target=self.Car_Control)
            self.car_control_thread.start() 
            
            
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {str(e)}")
            self.running = False
            raise

    def Cmd_Vel_Callback(self, msg):
        self.target_speedX = msg.linear.x
        self.target_speedY = msg.linear.y
        self.target_gyroz = msg.angular.z 

    def num_cmd_Callback(self, msg):
        self.numsub = msg.data

    def aim_cmd_Callback(self, msg):
        self.LR_aim = msg.data

    def Car_Control(self):
        while rclpy.ok(): 
            self.output_gyroz = self.target_gyroz* 30
            self.output_speedX = self.target_speedX * 10
            self.output_speedY = self.target_speedY * 10
            
            if self.output_speedX > 0:
                if self.output_speedX > 5: 
                    self.output_speedX = 5
                elif self.output_speedX < 2:
                    self.output_speedX = 2
            elif self.output_speedX < 0:
                if self.output_speedX < -5:
                    self.output_speedX = -5
                elif  self.output_speedX > -2:
                    self.output_speedX = -2
            else:
                self.output_speedX = 0

            if self.output_speedY > 0:
                if self.output_speedY > 5: 
                    self.output_speedY = 5
                elif self.output_speedY < 2:
                    self.output_speedY = 2
            elif self.output_speedY < 0:
                if self.output_speedY < -5:
                    self.output_speedY = -5
                elif  self.output_speedY > -2:
                    self.output_speedY = -2
            else:
                self.output_speedY = 0
            
            if self.output_gyroz > 0:
                if self.output_gyroz > 15:
                    self.output_gyroz = 15
                elif self.output_gyroz <3:
                    self.output_gyroz = 3
            elif self.output_gyroz < 0:
                if self.output_gyroz < -15:
                    self.output_gyroz = -15
                elif self.output_gyroz >-3:
                    self.output_gyroz = -3
            else:
                self.output_gyroz = 0
            
            if self.LR_aim == 1:
                #RD 
                if self.numsub ==1 :
                    self.message = '#002P0600T2000!'
                elif self.numsub ==2 :
                    self.message = '#002PDST!'
                #RU
                elif self.numsub ==3 :
                    self.message = '#002P2400T2000!'
                elif self.numsub ==4 :
                    self.message = '#002PDST!'
                #RR
                elif self.numsub ==5 :
                    self.message = '#003P2400T2000!'
                elif self.numsub ==6 :
                    self.message = '#003PDST!'
                #RL
                elif self.numsub ==7 :
                    self.message = '#003P0600T2000!'
                elif self.numsub ==8 :
                    self.message = '#003PDST!'
                #L1
                elif self.numsub ==9 :
                    self.message = '#004P2400T2000!'
                elif self.numsub ==10 :
                    self.message = '#004PDST!'
                #R1
                elif self.numsub ==11:
                    self.message = '#004P0600T2000!'
                elif self.numsub ==12 :
                    self.message = '#004PDST!'
                #L2
                elif self.numsub ==13 :
                    self.message = '#005P0600T2000!'
                elif self.numsub ==14 :
                    self.message = '#005PDST!'  
                #R2
                elif self.numsub ==15 :
                    self.message = '#005P2400T2000!'
                elif self.numsub ==16 :
                    self.message = '#005PDST!'    
                #LR
                elif self.numsub ==17 :
                    self.message = '#000P2400T2000!'
                #LL
                elif self.numsub ==18 :
                    self.message = '#000P0600T2000!'
                elif self.numsub ==19 :
                    self.message = '#000PDST!'
                #LD
                elif self.numsub ==20 :
                    self.message = '#001P2400T2000!'
                #LU
                elif self.numsub ==21 :
                    self.message = '#001P0600T2000!'
                elif self.numsub ==22 :
                    self.message = '#001PDST!'
                #Select : Data Init
                elif self.numsub ==23 :
                    self.message = '(init?'
                #Start : Action
                elif self.numsub ==25 :
                    self.message = self.cmd   
                #发布控制命令
                else:
                    self.message = '[{},{},{}]'.format(self.output_speedX,self.output_speedY,self.output_gyroz)
            else:
                #RD 
                if self.numsub ==1 :
                    self.message = '#008P0600T2000!'
                elif self.numsub ==2 :
                    self.message = '#008PDST!'
                #RU
                elif self.numsub ==3 :
                    self.message = '#008P2400T2000!'
                elif self.numsub ==4 :
                    self.message = '#008PDST!'
                #RR
                elif self.numsub ==5 :
                    self.message = '#009P2400T2000!'
                elif self.numsub ==6 :
                    self.message = '#009PDST!'
                #RL
                elif self.numsub ==7 :
                    self.message = '#009P0600T2000!'
                elif self.numsub ==8 :
                    self.message = '#009PDST!'
                #L1
                elif self.numsub ==9 :
                    self.message = '#010P2400T2000!'
                elif self.numsub ==10 :
                    self.message = '#010PDST!'
                #R1
                elif self.numsub ==11:
                    self.message = '#010P0600T2000!'
                elif self.numsub ==12 :
                    self.message = '#010PDST!'
                #L2
                elif self.numsub ==13 :
                    self.message = '#011P0600T2000!'
                elif self.numsub ==14 :
                    self.message = '#011PDST!'  
                #R2
                elif self.numsub ==15 :
                    self.message = '#011P2400T2000!'
                elif self.numsub ==16 :
                    self.message = '#011PDST!'    
                #LR
                elif self.numsub ==17 :
                    self.message = '#006P2400T2000!'
                #LL
                elif self.numsub ==18 :
                    self.message = '#006P0600T2000!'
                elif self.numsub ==19 :
                    self.message = '#006PDST!'
                #LD
                elif self.numsub ==20 :
                    self.message = '#007P2400T2000!'
                #LU
                elif self.numsub ==21 :
                    self.message = '#007P0600T2000!'
                elif self.numsub ==22 :
                    self.message = '#007PDST!'
                #Select : Data Init
                elif self.numsub ==23 :
                    self.message = '(init?'
                #Start : Action
                elif self.numsub ==25 :
                    self.message = self.cmd  
                else:
                    self.message = '[{},{},{}]'.format(self.output_speedX,self.output_speedY,self.output_gyroz)
            
            if self.message != self.last_message or self.output_gyroz !=0:
                self.serial1.write(self.message.encode('utf-8'))
                self.get_logger().info(self.message.encode('utf-8'),throttle_duration_sec=0.02)
            self.last_message = self.message
            time.sleep(0.01)
    def _forward_data(self):
        """从串口1读取数据并转发到串口2"""
        while self.running and rclpy.ok():
            try:
                # 从串口1读取数据
                self.date = self.serial1.read(self.serial1.in_waiting or 1)
                # self.get_logger().info(self.date.decode('utf-8'),throttle_duration_sec=0.02)
                if self.date:
                    # 转发到串口2
                    self.serial2.write(self.date)
                    self.get_logger().debug(f"Forwarded {len(self.date)} bytes: {self.date.hex()}")
                    # 调试输出
                    if self.get_parameter('debug').value:
                        self.get_logger().debug(f"Forwarded {len(self.date)} bytes: {self.date.hex()}")
                        self.serial2.write(self.message)
                    time.sleep(0.005)
            except serial.SerialException as e:
                self.get_logger().error(f"Serial communication error: {str(e)}")
                time.sleep(1)  # 出错时暂停一下
    
    def shutdown(self):
        """关闭时的清理函数"""
        self.get_logger().info("Shutting down serial bridge")
        self.running = False
        
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        
        if hasattr(self, 'serial1') and self.serial1.is_open:
            self.serial1.close()
        
        if hasattr(self, 'serial2') and self.serial2.is_open:
            self.serial2.close()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    
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