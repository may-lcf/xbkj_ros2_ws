"""执行器节点 - 订阅 voice_command，调用 ROS2 服务/Topic 驱动机械臂和视觉系统

复用 asr_node.py 的相机检测逻辑，自动路由到 2D 或深度节点。
"""

import json
import subprocess

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Int32


def detect_camera():
    """检测USB相机类型，返回 'depth' 或 'mono'"""
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=3)
        if '3251:1930' in result.stdout:
            return 'depth'
    except Exception:
        pass
    return 'mono'


class ArmExecutorNode(Node):

    def __init__(self):
        super().__init__('arm_executor_node')

        self.camera_mode = detect_camera()
        self.prefix = 'depth_' if self.camera_mode == 'depth' else ''
        self.get_logger().info(
            f'执行器节点已启动 (相机: {self.camera_mode}, 前缀: "{self.prefix}")')

        # 追踪目标 Topic (2D 和深度共用)
        self.pub_color = self.create_publisher(String, '/color', 10)
        self.pub_label = self.create_publisher(Int32, '/label', 10)
        self.pub_num = self.create_publisher(String, '/num', 10)
        # 关节控制
        self.pub_joint = self.create_publisher(String, 'joint_commands', 10)

        self.sub_cmd = self.create_subscription(
            String, 'voice_command', self.cmd_callback, 10)

    def cmd_callback(self, msg):
        try:
            step = json.loads(msg.data)
            self.execute_step(step)
        except Exception as e:
            self.get_logger().error(f'指令执行失败: {e}')

    def execute_step(self, step):
        func = step.get('function', '')
        params = step.get('parameters', {})

        self.get_logger().info(f'执行: {func} {params}')

        # ── 视觉分拣/码垛 (enter/exit) ──
        if func in ('color_sorting', 'color_stack',
                     'label_sorting', 'label_stack',
                     'num_sorting', 'num_stack'):
            action = params.get('action', 'enter')
            self._call_trigger(f'/{self.prefix}{func}/{action}')

        # ── 视觉追踪 (enter/exit/track) ──
        elif func in ('color_track', 'label_track', 'num_track'):
            action = params.get('action', 'enter')
            if action in ('enter', 'exit'):
                self._call_trigger(f'/{self.prefix}{func}/{action}')
            elif action == 'track':
                if func == 'color_track':
                    msg = String()
                    msg.data = params.get('color', '')
                    self.pub_color.publish(msg)
                elif func == 'label_track':
                    msg = Int32()
                    msg.data = params.get('label', 0)
                    self.pub_label.publish(msg)
                elif func == 'num_track':
                    msg = Int32()
                    msg.data = str(params.get('num', ''))
                    self.pub_num.publish(msg)

        # ── 其他视觉 (无前缀) ──
        elif func in ('color_set', 'face_track'):
            action = params.get('action', 'enter')
            self._call_trigger(f'/{func}/{action}')

        # ── 舵机控制 ──
        elif func == 'servo':
            action = params.get('action', 'enter')
            self._call_trigger(f'/servo/{action}')

        # ── 关节控制 ──
        elif func == 'joint':
            msg = String()
            msg.data = json.dumps(params, ensure_ascii=False)
            self.pub_joint.publish(msg)

        # ── 固定动作/复位/停止 ──
        elif func == 'routine':
            action_name = params.get('action', '')
            msg = String()
            msg.data = json.dumps(
                {'action': action_name}, ensure_ascii=False)
            self.pub_joint.publish(msg)

        elif func in ('home', 'stop'):
            msg = String()
            msg.data = json.dumps({'action': func}, ensure_ascii=False)
            self.pub_joint.publish(msg)

        else:
            self.get_logger().warn(f'未知功能: {func}')

    def _call_trigger(self, service_name):
        """调用 Trigger 类型的 ROS2 服务"""
        client = self.create_client(Trigger, service_name)
        if client.service_is_ready() or client.wait_for_service(timeout_sec=1.0):
            client.call_async(Trigger.Request())
            self.get_logger().info(f'调用服务: {service_name}')
        else:
            self.get_logger().warn(f'服务不可用: {service_name}')


def main(args=None):
    rclpy.init(args=args)
    node = ArmExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
