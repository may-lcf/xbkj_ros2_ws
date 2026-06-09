#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class TeleopControlNode(Node):
    def __init__(self):
        super().__init__('teleop_control_node')
        # 参数配置
        self.declare_parameter('linear_scale', 0.5)  # 线速度缩放因子
        self.declare_parameter('angular_scale', 1.0) # 角速度缩放因子
        
        # 订阅摇杆话题
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # 发布控制指令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 发布舵机控制指令
        self.num_pub = self.create_publisher(Int32, '/num_cmd', 10)
        self.aim_pub = self.create_publisher(Int32, '/aim_cmd', 10)
        # 数字状态
        self.num = 0
        self.last_buttons = [0] * 12  # 假设有12个按钮
        self.last_axes =[0]*7
        #机械臂选择：1代表右臂，0代表左臂
        self.aim=0

        #遥控器类型选择
        self.usbType = 1
        
        self.get_logger().info("Teleop Control Node Started")

    def joy_callback(self, msg):
        # 获取参数
        linear_scale = self.get_parameter('linear_scale').value
        angular_scale = self.get_parameter('angular_scale').value
        
        # 处理摇杆控制 (假设左摇杆控制移动)
        twist = Twist()
        if self.usbType == 0:
            twist.linear.x = msg.axes[1] * linear_scale  +msg.axes[3] * linear_scale# 通常axes[1]是左摇杆上下
            twist.linear.y = msg.axes[0] * linear_scale 
            twist.angular.z = msg.axes[2] * angular_scale 
        elif self.usbType == 1:
            twist.linear.x = msg.axes[1] * linear_scale  +msg.axes[4] * linear_scale# 通常axes[1]是左摇杆上下
            twist.linear.y = msg.axes[0] * linear_scale 
            twist.angular.z = msg.axes[3] * angular_scale
        self.cmd_vel_pub.publish(twist)
        
        # 处理按钮事件
        try:
            if self.usbType == 0:
                # if self.num == 2 or self.num == 4 or self.num == 6 or self.num == 8 or self.num == 10 or self.num == 12 or self.num == 14 or self.num == 16 or self.num == 19 or self.num == 22 or self.num == 23 or self.num == 25:
                #     self.num =0
                #car move ----0
                if msg.axes[1] != 0.0 or msg.axes[2] != 0.0:
                    self.num =0
                #LR ----17
                elif msg.axes[6] == 1.0:
                    self.num =17
                #LL ----18
                elif msg.axes[6] == -1.0:
                    self.num =18
                elif msg.axes[6] == 0.0 and self.last_axes[6] != 0.0:
                    self.num = 19  
                #LD ----20
                elif msg.axes[7] == 1.0:
                    self.num =20
                #LU ----21
                elif msg.axes[7] == -1.0:
                    self.num =21
                elif msg.axes[7] == 0.0 and self.last_axes[7] != 0.0:
                    self.num = 22
                
                #RD ----1
                if msg.buttons[0] == 1:
                    self.num =1
                elif msg.buttons[0] == 0 and self.last_buttons[0] == 1:
                    self.num = 2
                #RU ----3
                elif msg.buttons[4] == 1:
                    self.num = 3
                elif msg.buttons[4] == 0 and self.last_buttons[4] == 1:
                    self.num = 4
                #RR ----5
                elif msg.buttons[1] == 1:
                    self.num = 5
                elif msg.buttons[1] == 0 and self.last_buttons[1] == 1:
                    self.num = 6
                #RL ----7
                elif msg.buttons[3] == 1:
                    self.num = 7
                elif msg.buttons[3] == 0 and self.last_buttons[3] == 1:
                    self.num = 8
                #L1 ----9
                elif msg.buttons[6] == 1:
                    self.num = 9
                elif msg.buttons[6] == 0 and self.last_buttons[6] == 1:
                    self.num = 10
                #R1 ----11
                elif msg.buttons[7] == 1:
                    self.num = 11
                elif msg.buttons[7] == 0 and self.last_buttons[7] == 1:
                    self.num = 12
                #L2 ----13
                elif msg.buttons[8] == 1:
                    self.num = 13
                elif msg.buttons[8] == 0 and self.last_buttons[8] == 1:
                    self.num = 14
                #R2 ----15
                elif msg.buttons[9] == 1:
                    self.num = 15
                elif msg.buttons[9] == 0 and self.last_buttons[9] == 1:
                    self.num = 16 
                # select: Odom Data Init    
                elif msg.buttons[10] == 1 and self.last_buttons[10] == 0:#Odom Data Init
                    self.num = 23  
                # start: Action       
                elif msg.buttons[11] == 1 and self.last_buttons[11] == 0:
                    self.num = 25
                elif msg.buttons[13] == 1 and self.last_buttons[13] == 0:
                    self.aim = 0   
                    self.publish_aim()
                #R
                elif msg.buttons[14] == 1 and self.last_buttons[14] == 0:
                    self.aim = 1
                    self.publish_aim()

            elif self.usbType == 1:
                # if self.num == 2 or self.num == 4 or self.num == 6 or self.num == 8 or self.num == 10 or self.num == 12 or self.num == 14 or self.num == 16 or self.num == 19 or self.num == 22 or self.num == 23 or self.num == 25:
                #     self.num =0
                #car move ----0
                if msg.axes[1] != 0.0 or msg.axes[3] != 0.0:
                    self.num =0
                #LR ----17
                elif msg.axes[6] == 1.0:
                    self.num =18
                #LL ----18
                elif msg.axes[6] == -1.0:
                    self.num =17
                elif msg.axes[6] == 0.0 and self.last_axes[6] != 0.0:
                    self.num = 19  
                #LD ----20
                elif msg.axes[7] == 1.0:
                    self.num =21
                #LU ----21
                elif msg.axes[7] == -1.0:
                    self.num =20
                elif msg.axes[7] == 0.0 and self.last_axes[7] != 0.0:
                    self.num = 22
                #L2
                elif msg.axes[2] == -1.0:
                    self.num = 13
                elif msg.axes[2] == 1.0 and self.last_axes[2] != 1.0:
                    self.num = 14
                #R2
                elif msg.axes[5] == -1.0:
                    self.num = 15
                elif msg.axes[5] == 1.0 and self.last_axes[5] != 1.0:
                    self.num = 16

                #RD ----1
                if msg.buttons[0] == 1:
                    self.num =1
                elif msg.buttons[0] == 0 and self.last_buttons[0] == 1:
                    self.num = 2
                #RU ----3
                elif msg.buttons[3] == 1:
                    self.num = 3
                elif msg.buttons[3] == 0 and self.last_buttons[3] == 1:
                    self.num = 4
                #RR ----5
                elif msg.buttons[1] == 1:
                    self.num = 5
                elif msg.buttons[1] == 0 and self.last_buttons[1] == 1:
                    self.num = 6
                #RL ----7
                elif msg.buttons[2] == 1:
                    self.num = 7
                elif msg.buttons[2] == 0 and self.last_buttons[2] == 1:
                    self.num = 8
                #L1 ----9
                elif msg.buttons[4] == 1:
                    self.num = 9
                elif msg.buttons[4] == 0 and self.last_buttons[4] == 1:
                    self.num = 10
                #R1 ----11
                elif msg.buttons[5] == 1:
                    self.num = 11
                elif msg.buttons[5] == 0 and self.last_buttons[5] == 1:
                    self.num = 12
                # select: Odom Data Init
                elif msg.buttons[6] == 1 and self.last_buttons[6] == 0:
                    self.num = 23    
                # start: Action    
                elif msg.buttons[7] == 1 and self.last_buttons[7] == 0:
                    self.num = 25
                elif msg.buttons[9] == 1 and self.last_buttons[9] == 0:
                    self.aim = 0   
                    self.publish_aim()
                #R
                elif msg.buttons[10] == 1 and self.last_buttons[10] == 0:
                    self.aim = 1
                    self.publish_aim()
            self.publish_num()
            # self.get_logger().info(f"Num: {self.num}")
        except IndexError:
            pass
        
        # 保存当前按钮状态用于下次比较
        self.last_buttons = msg.buttons
        self.last_axes = msg.axes

    def publish_num(self):
        msg=Int32()
        msg.data=self.num
        self.num_pub.publish(msg)

    def publish_aim(self):
        msg=Int32()
        msg.data=self.aim
        self.aim_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()