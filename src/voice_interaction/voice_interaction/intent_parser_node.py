"""意图解析节点 - 通义千问 qwen-plus 解析自然语言为结构化指令

订阅 voice_text (ASR结果)，发布 voice_command (JSON动作) 和 speak_text (TTS文本)。
支持多轮对话上下文记忆。
LLM 流式输出：message 提取后立刻给 TTS (并行)，step 提取后给执行器。
"""

import os
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI


SYSTEM_PROMPT = """你是一个机械臂+视觉系统语音助手。用户将向你发出自然语言指令，
你的任务是提取并返回结构化信息。系统会自动根据相机类型选择2D或深度节点，你不需要关心。

回复格式：
1. 首先输出一个提示性消息：
{"message": "好的，我将执行您的指令"}
2. 然后按顺序输出控制指令，每个指令一个JSON：
{"step": {"order": 1, "function": "<功能>", "parameters": {<参数>}}}

=== 支持的功能 ===

【视觉分拣/码垛】(启动后自动运行，不需要额外参数)
- 颜色分拣: "color_sorting", {"action": "enter"|"exit"}
- 颜色码垛: "color_stack", {"action": "enter"|"exit"}
- 标签分拣: "label_sorting", {"action": "enter"|"exit"}
- 标签码垛: "label_stack", {"action": "enter"|"exit"}
- 数字分拣: "num_sorting", {"action": "enter"|"exit"}
- 数字码垛: "num_stack", {"action": "enter"|"exit"}

【视觉追踪】(先启动追踪，再设置追踪目标)
- 颜色追踪: "color_track", {"action": "enter"|"exit"|"track", "color": "red"|"green"|"blue"}
- 标签追踪: "label_track", {"action": "enter"|"exit"|"track", "label": 1|2|3}
- 数字追踪: "num_track", {"action": "enter"|"exit"|"track", "num": 1|2|3}

【其他视觉】
- 颜色设置: "color_set", {"action": "enter"|"exit"}
- 人脸追踪: "face_track", {"action": "enter"|"exit"}

【机械臂控制】
- 关节控制: "joint", {"id": 1-6, "angle": 角度}
- 固定动作: "routine", {"action": "夹爪开"|"夹爪关"|"恢复初始状态"|"比个耶"|"摇摇头"|"点点头"}
- 复位: "home", {}
- 停止: "stop", {}
- 舵机控制: "servo", {"action": "enter"|"exit"}

当前机械臂为5+1 dof：1号左右，2/3/4号前后，5号夹爪旋转，6号夹爪开合，范围[-90,90]度。

示例：
用户: "帮我把红色的挑出来"
{"message": "好的，我来分拣红色"}
{"step": {"order": 1, "function": "color_sorting", "parameters": {"action": "enter"}}}

用户: "开启颜色追踪，追踪蓝色"
{"message": "好的，开启颜色追踪蓝色"}
{"step": {"order": 1, "function": "color_track", "parameters": {"action": "enter"}}}
{"step": {"order": 2, "function": "color_track", "parameters": {"action": "track", "color": "blue"}}}

用户: "开启标签分拣"
{"message": "好的，开启标签分拣"}
{"step": {"order": 1, "function": "label_sorting", "parameters": {"action": "enter"}}}

用户: "停"
{"message": "好的，已停止"}
{"step": {"order": 1, "function": "stop", "parameters": {}}}

用户: "结束数字分拣"
{"message": "好的，已停止数字分拣"}
{"step": {"order": 1, "function": "num_sorting", "parameters": {"action": "exit"}}}

用户: "关闭颜色追踪"
{"message": "好的，已关闭颜色追踪"}
{"step": {"order": 1, "function": "color_track", "parameters": {"action": "exit"}}}

用户: "今天天气怎么样"
{"message": "我专注于机械臂控制，无法查询天气。"}

规则：
- 涉及机械臂/视觉控制的指令：必须同时输出 message 和 step
- 闲聊或无关问题：只输出 message，不输出 step
- "开始/开启/启动" → action: "enter"
- "结束/关闭/停止/退出" → action: "exit"

不要输出任何解释或说明，每个JSON对象必须独立且完整，一次一个。
"""


class IntentParserNode(Node):

    def __init__(self):
        super().__init__('intent_parser_node')
        self.declare_parameter('api_key', '')
        self.declare_parameter('base_url',
                               'https://dashscope.aliyuncs.com/compatible-mode/v1')
        self.declare_parameter('model', 'qwen-plus')
        self.declare_parameter('max_history', 10)

        api_key = self.get_parameter('api_key').value
        if not api_key:
            api_key = os.environ.get('DASHSCOPE_API_KEY', '')
        if not api_key:
            self.get_logger().error('未设置 api_key')
            return

        base_url = self.get_parameter('base_url').value
        self.model = self.get_parameter('model').value
        self.max_history = self.get_parameter('max_history').value

        self.client = OpenAI(base_url=base_url, api_key=api_key)

        # 对话上下文：system prompt + 滚动历史
        self.messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        self._history_lock = threading.Lock()

        self.sub_text = self.create_subscription(
            String, 'voice_text', self.text_callback, 10)
        self.pub_intent = self.create_publisher(String, 'voice_command', 10)
        self.pub_speak = self.create_publisher(String, 'speak_text', 10)

        self.get_logger().info(
            f'意图解析节点已启动 (模型: {self.model}, 上下文轮数: {self.max_history})')

    def text_callback(self, msg):
        text = msg.data
        self.get_logger().info(f'收到文本: {text}')
        threading.Thread(
            target=self._parse_async, args=(text,), daemon=True).start()

    def _parse_async(self, text):
        try:
            # 构建带上下文的 messages
            with self._history_lock:
                messages = list(self.messages)
                messages.append({"role": "user", "content": text})

            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                stream=True,
            )

            buffer = ""
            message_text = ""
            assistant_reply = ""

            for chunk in response:
                if not hasattr(chunk, 'choices') or not chunk.choices:
                    continue
                delta = chunk.choices[0].delta
                content = getattr(delta, 'content', None)
                if not content:
                    continue
                buffer += content
                assistant_reply += content

                # 实时提取 message → 立刻给 TTS (与 LLM 流式并行)
                if not message_text:
                    idx = buffer.find('"message"')
                    if idx != -1:
                        q1 = buffer.find('"', idx + 9)
                        q2 = buffer.find('"', q1 + 1)
                        if q1 != -1 and q2 != -1:
                            message_text = buffer[q1 + 1:q2]
                            self.get_logger().info(f'LLM回复: {message_text}')
                            speak_msg = String()
                            speak_msg.data = message_text
                            self.pub_speak.publish(speak_msg)

                # 提取 step JSON → 给执行器
                buffer = self._extract_steps(buffer)

            # 存入上下文历史
            if assistant_reply:
                with self._history_lock:
                    self.messages.append({"role": "user", "content": text})
                    self.messages.append({"role": "assistant", "content": assistant_reply})
                    # 保留 system + 最近 max_history*2 条 (每轮=user+assistant)
                    max_msgs = 1 + self.max_history * 2
                    if len(self.messages) > max_msgs:
                        self.messages = [self.messages[0]] + self.messages[-(max_msgs - 1):]

        except Exception as e:
            self.get_logger().error(f'LLM调用失败: {e}')

    def _extract_steps(self, buffer):
        """从缓冲区提取并发布 step JSON，返回剩余buffer"""
        while True:
            start = buffer.find('{')
            if start == -1:
                break
            depth = 0
            end = -1
            for i in range(start, len(buffer)):
                if buffer[i] == '{':
                    depth += 1
                elif buffer[i] == '}':
                    depth -= 1
                    if depth == 0:
                        end = i
                        break
            if end == -1:
                break
            json_str = buffer[start:end + 1]
            try:
                obj = json.loads(json_str)
                if 'step' in obj:
                    step = obj['step']
                    cmd_msg = String()
                    cmd_msg.data = json.dumps(step, ensure_ascii=False)
                    self.pub_intent.publish(cmd_msg)
                    self.get_logger().info(f'发布指令: {cmd_msg.data}')
            except json.JSONDecodeError:
                pass
            buffer = buffer[end + 1:]

        last_brace = buffer.rfind('}')
        if last_brace != -1:
            return buffer[last_brace + 1:]
        return buffer


def main(args=None):
    rclpy.init(args=args)
    node = IntentParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
