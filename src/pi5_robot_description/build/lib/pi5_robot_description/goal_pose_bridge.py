#!/usr/bin/env python3
"""
goal_pose_bridge 节点

作用：把 RViz 的 "2D Nav Goal / SetGoal" 工具发布的 /goal_pose (PoseStamped)
      转发到 Nav2 的 /navigate_to_pose action server。

背景：nav2_rviz_plugins/GoalTool 直接调 action server，但 RViz 长期运行的
      action client 在树莓派上偶发发现不了 server。改用话题方式（与 SetInitialPose
      同样可靠），再用本节点桥接到 action。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class GoalPoseBridge(Node):
    def __init__(self):
        super().__init__('goal_pose_bridge')
        self._ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._cb, 10)
        self.get_logger().info(
            'goal_pose_bridge 已启动: 监听 /goal_pose -> 转发到 /navigate_to_pose')

    def _cb(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        frame = msg.header.frame_id
        self.get_logger().info(
            f'收到 /goal_pose: ({x:.3f}, {y:.3f}) in {frame}, 发送到 /navigate_to_pose')

        goal = NavigateToPose.Goal()
        goal.pose = msg
        if not self._ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                '/navigate_to_pose action server 不可用! 请确认导航系统已启动')
            return
        self._ac.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
