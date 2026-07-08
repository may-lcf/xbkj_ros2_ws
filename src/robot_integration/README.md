# Robot Integration 小车+机械臂集成控制包

## 系统架构

树莓派5 (ROS2) 通过串口与 STM32 通信，控制小车运动和6轴机械臂。

## ROS2 话题

### 订阅话题
- /cmd_vel (Twist): 速度控制
- /arm_command (String): 舵机指令
- /arm_joint_command (JointState): 关节控制
- /joy (Joy): 手柄输入

### 发布话题
- /odom (Odometry): 里程计
- /arm_state (JointState): 机械臂状态

## STM32 协议

- [x,y,z]: 小车速度
- #SSSPxxxxTxxxx!: 舵机控制
- (odom_x,odom_y,angle): 里程计反馈

## 使用方法

colcon build --packages-select robot_integration
source install/setup.bash
ros2 launch robot_integration full_system.launch.py

## 文件说明

- stm32_bridge_node.py: STM32通信桥接
- car_controller_node.py: 小车控制
- arm_controller_node.py: 机械臂控制
- coordinator_node.py: 协调器
