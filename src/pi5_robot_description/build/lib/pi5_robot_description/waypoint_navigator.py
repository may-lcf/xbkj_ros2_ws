#!/usr/bin/env python3
"""
多点导航节点 (Waypoint Navigator)

功能：
  - 通过 RViz "Publish Point" 工具逐个标记目标点
  - 依次导航到每个目标点，利用 Nav2 现有的规划+避障能力
  - 到达后等待可配置秒数，自动前往下一个点
  - 支持暂停/恢复/取消/清除控制

用法：
  1. 在 RViz 中选择 "Publish Point" 工具，点击地图标记目标点
  2. 终端发送: ros2 topic pub --once /waypoint_cmd std_msgs/String "data: 'start'"
  3. 小车依次导航到每个目标点

话题：
  订阅:
    /clicked_point (geometry_msgs/PointStamped) - RViz 标记的目标点
    /waypoint_cmd (std_msgs/String) - 控制命令
  发布:
    /waypoint_status (std_msgs/String) - 状态反馈
    /waypoint_markers (visualization_msgs/MarkerArray) - 目标点可视化
    /nav_goal_pose (geometry_msgs/PoseStamped) - 当前导航目标
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class WaypointNavigator(Node):
    """多点顺序导航节点"""

    # 状态常量
    STATE_IDLE = 'IDLE'
    STATE_NAVIGATING = 'NAVIGATING'
    STATE_WAITING = 'WAITING'
    STATE_PAUSED = 'PAUSED'
    STATE_COMPLETE = 'COMPLETE'

    def __init__(self):
        super().__init__('waypoint_navigator')

        # ========== 参数声明 ==========
        self.declare_parameter('wait_time', 2.0)       # 到达每个点后等待秒数
        self.declare_parameter('max_retries', 1)        # 失败重试次数
        self.declare_parameter('frame_id', 'map')       # 目标点参考坐标系
        self.declare_parameter('auto_yaw', True)        # 自动计算朝向（指向下一个点）

        self.wait_time = self.get_parameter('wait_time').value
        self.max_retries = self.get_parameter('max_retries').value
        self.frame_id = self.get_parameter('frame_id').value
        self.auto_yaw = self.get_parameter('auto_yaw').value

        # ========== 状态变量 ==========
        self.waypoints = []          # [(x, y, yaw), ...]
        self.waypoint_labels = []    # [str, ...] 每个点的标签
        self.current_index = -1      # 当前目标索引
        self.state = self.STATE_IDLE
        self.retry_count = 0         # 当前目标重试次数
        self._paused_index = -1      # 暂停时保存的索引

        # ========== Action Client ==========
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None

        # ========== 订阅者 ==========
        self._clicked_point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self._on_clicked_point, 10)
        self._cmd_sub = self.create_subscription(
            String, '/waypoint_cmd', self._on_command, 10)

        # ========== 发布者 ==========
        self._status_pub = self.create_publisher(
            String, '/waypoint_status', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/waypoint_markers', 10)
        self._goal_pub = self.create_publisher(
            PoseStamped, '/nav_goal_pose', 10)

        # ========== 定时器：刷新标记显示 ==========
        self._marker_timer = self.create_timer(1.0, self._publish_markers)

        self.get_logger().info(
            '=== 多点导航节点已启动 ===\n'
            '  使用 RViz "Publish Point" 工具标记目标点\n'
            '  命令: ros2 topic pub --once /waypoint_cmd std_msgs/String "data: \'start\'"\n'
            f'  等待时间: {self.wait_time}s | 重试次数: {self.max_retries}')

    # ================================================================
    #  目标点收集
    # ================================================================

    def _on_clicked_point(self, msg: PointStamped):
        """RViz Publish Point 工具点击回调"""
        if self.state == self.STATE_NAVIGATING:
            self.get_logger().warn('导航进行中，无法添加目标点。请先取消或等待完成。')
            return

        x = msg.point.x
        y = msg.point.y

        # 计算朝向：指向下一个点的方向（如果启用 auto_yaw）
        if self.auto_yaw and len(self.waypoints) > 0:
            prev_x, prev_y, _ = self.waypoints[-1]
            dx = x - prev_x
            dy = y - prev_y
            yaw = math.atan2(dy, dx)
            # 更新上一个点的朝向，使其指向当前点
            self.waypoints[-1] = (prev_x, prev_y, yaw)
        else:
            yaw = 0.0

        self.waypoints.append((x, y, yaw))
        idx = len(self.waypoints)
        label = f'WP{idx}'
        self.waypoint_labels.append(label)

        self.get_logger().info(
            f'✅ 已添加第{idx}个目标点: ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}°)'
            f' | 共 {len(self.waypoints)} 个点')

        self._publish_status(f'已添加 {idx} 个目标点')
        self._publish_markers()

        # 重置状态（允许重新 start）
        if self.state == self.STATE_COMPLETE:
            self.state = self.STATE_IDLE

    # ================================================================
    #  命令处理
    # ================================================================

    def _on_command(self, msg: String):
        """处理控制命令"""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'收到命令: {cmd}')

        if cmd == 'start':
            self._cmd_start()
        elif cmd == 'pause':
            self._cmd_pause()
        elif cmd == 'resume':
            self._cmd_resume()
        elif cmd == 'cancel':
            self._cmd_cancel()
        elif cmd == 'clear':
            self._cmd_clear()
        elif cmd == 'list':
            self._cmd_list()
        else:
            self.get_logger().warn(f'未知命令: {cmd}')

    def _cmd_start(self):
        if len(self.waypoints) == 0:
            self.get_logger().warn('没有目标点！请先用 RViz "Publish Point" 工具标记目标点。')
            return
        if self.state == self.STATE_NAVIGATING:
            self.get_logger().warn('已在导航中，请勿重复启动。')
            return
        if self.state == self.STATE_PAUSED:
            self.get_logger().info('当前处于暂停状态，请使用 resume 命令恢复。')
            return

        self.get_logger().info(
            f'🚀 开始多点导航！共 {len(self.waypoints)} 个目标点')
        self.current_index = 0
        self.retry_count = 0
        self.state = self.STATE_IDLE  # 先设为 IDLE，_navigate_next 会切换
        self._navigate_next()

    def _cmd_pause(self):
        if self.state != self.STATE_NAVIGATING:
            self.get_logger().warn('当前不在导航状态，无法暂停。')
            return
        self._paused_index = self.current_index
        self.state = self.STATE_PAUSED
        # 取消当前导航目标
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self.get_logger().info('⏸️ 导航已暂停')
        self._publish_status('⏸️ 已暂停')

    def _cmd_resume(self):
        if self.state != self.STATE_PAUSED:
            self.get_logger().warn('当前不在暂停状态，无法恢复。')
            return
        self.current_index = self._paused_index
        self.state = self.STATE_IDLE
        self.get_logger().info(
            f'▶️ 恢复导航，继续前往第 {self.current_index + 1} 个目标点')
        self._navigate_next()

    def _cmd_cancel(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self.state = self.STATE_IDLE
        self.current_index = -1
        self.get_logger().info('🛑 导航已取消')
        self._publish_status('🛑 已取消')

    def _cmd_clear(self):
        if self.state in (self.STATE_NAVIGATING, self.STATE_WAITING):
            self.get_logger().warn('导航进行中，请先取消再清除。')
            return
        self.waypoints.clear()
        self.waypoint_labels.clear()
        self.current_index = -1
        self.state = self.STATE_IDLE
        self._publish_markers()
        self.get_logger().info('🗑️ 目标点列表已清空')
        self._publish_status('目标点已清空')

    def _cmd_list(self):
        if len(self.waypoints) == 0:
            self.get_logger().info('目标点列表为空。')
            return
        self.get_logger().info('=== 目标点列表 ===')
        for i, (x, y, yaw) in enumerate(self.waypoints):
            marker = '→' if i == self.current_index else ' '
            self.get_logger().info(
                f'  {marker} {self.waypoint_labels[i]}: '
                f'({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}°)')

    # ================================================================
    #  导航逻辑
    # ================================================================

    def _navigate_next(self):
        """发送下一个导航目标"""
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('🎉 所有目标点已完成！')
            self.state = self.STATE_COMPLETE
            self._publish_status('🎉 全部完成！')
            return

        x, y, yaw = self.waypoints[self.current_index]
        label = self.waypoint_labels[self.current_index]

        self.get_logger().info(
            f'📍 [{self.current_index + 1}/{len(self.waypoints)}] '
            f'前往 {label}: ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}°)')

        self._publish_status(
            f'📍 前往 {label} ({self.current_index + 1}/{len(self.waypoints)})')

        # 构建 PoseStamped
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        # yaw -> quaternion (只绕 Z 轴旋转)
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 发布当前目标（用于 RViz 可视化）
        self._goal_pub.publish(goal_pose)

        # 更新标记显示（当前目标高亮）
        self._publish_markers()

        # 等待 action server 就绪
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server 未就绪！')
            self._publish_status('❌ Action server 未就绪')
            self.state = self.STATE_IDLE
            return

        # 发送目标
        self.state = self.STATE_NAVIGATING
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self._send_goal_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._on_feedback)
        self._send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        """目标被接受或拒绝的回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ 目标被拒绝！')
            self._handle_goal_failure()
            return

        self._goal_handle = goal_handle
        self.get_logger().info('✅ 目标已接受，正在导航...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        """导航结果回调"""
        result = future.result()
        status = result.status

        # 保存当前索引，防止在回调执行期间 index 已变化
        idx = self.current_index

        # 如果已经不是导航状态（比如被 pause/cancel 或已切换到下一个点），忽略
        if self.state != self.STATE_NAVIGATING:
            self.get_logger().info(f'忽略过期的导航结果 (状态: {self.state})')
            return

        if idx < 0 or idx >= len(self.waypoints):
            self.get_logger().warn(f'忽略结果：索引 {idx} 越界')
            return

        label = self.waypoint_labels[idx]

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'✅ 到达 {label}！ 等待 {self.wait_time}s 后继续...')
            self._publish_status(f'✅ 到达 {label}，等待 {self.wait_time}s')
            self.state = self.STATE_WAITING
            self._goal_handle = None

            # 等待后前往下一个点（一次性定时器）
            timer = self.create_timer(
                self.wait_time, lambda: self._on_wait_complete(timer))
        else:
            self.get_logger().warn(f'⚠️ 导航失败，状态码: {status}')
            self._handle_goal_failure()

    def _on_feedback(self, feedback_msg):
        """导航过程中的反馈（可选：显示距离等信息）"""
        feedback = feedback_msg.feedback
        # feedback 包含: current_pose, navigation_time, estimated_time_remaining, number_of_recoveries, distance_remaining
        # 可以在这里打印进度，但为了避免刷屏，只在关键节点打印
        pass

    def _on_wait_complete(self, timer=None):
        """等待完成，前往下一个点"""
        # 取消一次性定时器，防止重复触发
        if timer is not None:
            timer.cancel()
        self.current_index += 1
        self.retry_count = 0
        self._navigate_next()

    def _handle_goal_failure(self):
        """处理导航失败"""
        self._goal_handle = None
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.get_logger().info(
                f'🔄 重试 ({self.retry_count}/{self.max_retries})...')
            self._navigate_next()  # 重试同一个点
        else:
            self.get_logger().warn(
                f'⚠️ 跳过 {self.waypoint_labels[self.current_index]}')
            self.current_index += 1
            self.retry_count = 0
            self._navigate_next()

    # ================================================================
    #  可视化
    # ================================================================

    def _publish_markers(self):
        """发布目标点标记到 RViz"""
        marker_array = MarkerArray()

        # 清除旧标记
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        for i, (x, y, yaw) in enumerate(self.waypoints):
            label = self.waypoint_labels[i]
            is_current = (i == self.current_index and
                          self.state in (self.STATE_NAVIGATING, self.STATE_WAITING))

            # ---- 球体标记 ----
            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'waypoint_spheres'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.1
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.15
            sphere.scale.y = 0.15
            sphere.scale.z = 0.15

            if is_current:
                sphere.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # 橙色=当前
            elif i < self.current_index:
                sphere.color = ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.6)  # 绿色=已到达
            else:
                sphere.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)  # 蓝色=待前往

            marker_array.markers.append(sphere)

            # ---- 文字标记 ----
            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'waypoint_text'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.3
            text.pose.orientation.w = 1.0
            text.scale.z = 0.12  # 字体大小
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = label
            marker_array.markers.append(text)

            # ---- 箭头标记（朝向） ----
            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'waypoint_arrows'
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.05
            arrow.pose.orientation.z = math.sin(yaw / 2.0)
            arrow.pose.orientation.w = math.cos(yaw / 2.0)
            arrow.scale.x = 0.25  # 箭头长度
            arrow.scale.y = 0.03  # 箭头宽度
            arrow.scale.z = 0.03
            arrow.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)  # 黄色
            marker_array.markers.append(arrow)

        # ---- 连线 ----
        if len(self.waypoints) >= 2:
            line_strip = Marker()
            line_strip.header.frame_id = self.frame_id
            line_strip.header.stamp = self.get_clock().now().to_msg()
            line_strip.ns = 'waypoint_line'
            line_strip.id = 9999
            line_strip.type = Marker.LINE_STRIP
            line_strip.action = Marker.ADD
            line_strip.scale.x = 0.02  # 线宽
            line_strip.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.5)  # 青色

            for x, y, _ in self.waypoints:
                p = Marker().pose.position  # 借用类型
                from geometry_msgs.msg import Point
                pt = Point()
                pt.x = x
                pt.y = y
                pt.z = 0.05
                line_strip.points.append(pt)

            marker_array.markers.append(line_strip)

        self._marker_pub.publish(marker_array)

    def _publish_status(self, text: str):
        """发布状态消息"""
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    # ================================================================
    #  清理
    # ================================================================

    def destroy_node(self):
        """节点关闭时取消导航"""
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
