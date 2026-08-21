#!/usr/bin/env python3
"""
机械臂固定姿态关节状态发布节点

发布固定的关节角度，使机械臂保持在预设姿态。
用于 3D 建图时固定机械臂位置。

用法:
  ros2 run pi5_robot_description arm_fixed_pose_node --ros-args -p pose:=horizontal
  ros2 run pi5_robot_description arm_fixed_pose_node --ros-args -p pose:=vertical
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
import math


# 预定义姿态 (URDF 弧度)
POSES = {
    # 对应舵机: {#0P1550T1000!#1P1850T1000!#2P2100T1000!#3P1100T1000!#4P1500T1000!}
    "horizontal": {
        "joint1": 0.118,    # PWM1550, 底座旋转
        "joint2": 0.825,    # PWM1850, 肩关节
        "joint3": 1.414,    # PWM2100, 肘关节
        "joint4": -0.942,   # PWM1100, 腕关节
        "joint5": 0.000,    # PWM1500, 末端
        "joint6_r": 0.000,  # 夹爪
    },
    # 所有关节竖直 (PWM1500)
    "vertical": {
        "joint1": 0.0,
        "joint2": 0.0,
        "joint3": 0.0,
        "joint4": 0.0,
        "joint5": 0.0,
        "joint6_r": 0.0,
    },
}


class ArmFixedPoseNode(Node):
    def __init__(self):
        super().__init__("arm_fixed_pose_node")

        # 参数: 姿态名称
        self.declare_parameter("pose", "horizontal")
        self.declare_parameter("rate", 10.0)

        pose_name = self.get_parameter("pose").value
        rate = self.get_parameter("rate").value

        if pose_name not in POSES:
            self.get_logger().error(f"Unknown pose: {pose_name}. Available: {list(POSES.keys())}")
            pose_name = "vertical"

        self.pose = POSES[pose_name]
        self.get_logger().info(f"Arm fixed pose: {pose_name}")
        for joint, angle in self.pose.items():
            self.get_logger().info(f"  {joint}: {angle:.3f} rad ({math.degrees(angle):.1f} deg)")

        # 发布 /joint_states
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_joint_states)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.pose.keys())
        msg.position = [float(v) for v in self.pose.values()]
        msg.velocity = []
        msg.effort = []
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmFixedPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
