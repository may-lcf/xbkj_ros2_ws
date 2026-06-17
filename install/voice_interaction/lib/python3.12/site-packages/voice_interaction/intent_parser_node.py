"""意图解析节点 - 通义千问 qwen-plus 解析自然语言为结构化指令

订阅 voice_text (ASR结果)，发布 voice_command (JSON动作) 和 speak_text (TTS文本)。
支持多轮对话上下文记忆。
LLM 流式输出：message 提取后立刻给 TTS (并行)，step 提取后给执行器。

唤醒机制：
  - 沉睡状态：只听唤醒词，其他语音忽略
  - 唤醒状态：正常处理指令，15秒无指令自动回到沉睡
  - 说"退下"手动回到沉睡
"""

import os
import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI


# ═══════════════════════════════════════════════════════════════════════════════
#  唤醒词配置（可自定义修改）
# ═══════════════════════════════════════════════════════════════════════════════

WAKE_WORDS = ['小星小星', '小心小心', '小新小新', '小星星', '小新新']
DISMISS_WORD = '退下'
TIMEOUT_SEC = 15.0

WAKE_REPLY = '我在'
DISMISS_REPLY = '小星退下了，有需要再唤醒我'


# ═══════════════════════════════════════════════════════════════════════════════
#  状态
# ═══════════════════════════════════════════════════════════════════════════════

STATE_SLEEP = 0
STATE_AWAKE = 1


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

【YOLO物体抓取】(YOLO智能识别+机械臂抓取)
- yolo_pick: "yolo_pick", {"action": "pick", "shape": "<形状>", "color": "<颜色>"}
  支持形状: 正方体、长方体、圆柱、球体、螺丝刀
  支持颜色: 红色、绿色、蓝色（可选，不指定则按像素面积从大到小依次抓取该形状的所有物体）

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

用户: "停"
{"message": "好的，已停止"}
{"step": {"order": 1, "function": "stop", "parameters": {}}}

用户: "帮我拿红色长方体"
{"message": "好的，我来拿红色长方体"}
{"step": {"order": 1, "function": "yolo_pick", "parameters": {"action": "pick", "shape": "长方体", "color": "红色"}}}

用户: "拿螺丝刀"
{"message": "好的，我来拿螺丝刀"}
{"step": {"order": 1, "function": "yolo_pick", "parameters": {"action": "pick", "shape": "螺丝刀"}}}

用户: "请抓取红色正方体"
{"message": "好的，我来抓取红色正方体"}
{"step": {"order": 1, "function": "yolo_pick", "parameters": {"action": "pick", "shape": "正方体", "color": "红色"}}}

用户: "帮我夹取那个长方体"
{"message": "好的，我来夹取长方体"}
{"step": {"order": 1, "function": "yolo_pick", "parameters": {"action": "pick", "shape": "长方体"}}}

用户: "把圆柱体拿给我"
{"message": "好的，我来拿圆柱体"}
{"step": {"order": 1, "function": "yolo_pick", "parameters": {"action": "pick", "shape": "圆柱体"}}}

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

        # 对话上下文
        self.messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        self._history_lock = threading.Lock()

        # ── 唤醒状态机 ──
        self.state = STATE_SLEEP
        self._timer_lock = threading.Lock()
        self._timeout_timer = None

        self.sub_text = self.create_subscription(
            String, 'voice_text', self.text_callback, 10)
        self.pub_intent = self.create_publisher(String, 'voice_command', 10)
        self.pub_speak = self.create_publisher(String, 'speak_text', 10)

        self.get_logger().info(
            f'意图解析节点已启动 (模型: {self.model}, 状态: 沉睡)')

    # ══════════════════════════════════════════════════════════════════════════
    #  唤醒词检测
    # ══════════════════════════════════════════════════════════════════════════

    def _is_wake_word(self, text):
        """检查文本是否包含唤醒词"""
        clean = text.replace(' ', '').replace('，', '').replace('。', '')
        for w in WAKE_WORDS:
            if w in clean:
                return True
        return False

    def _is_dismiss(self, text):
        """检查是否为退下指令"""
        clean = text.replace(' ', '').replace('，', '').replace('。', '')
        return DISMISS_WORD in clean

    # ══════════════════════════════════════════════════════════════════════════
    #  状态切换
    # ══════════════════════════════════════════════════════════════════════════

    def _enter_awake(self):
        """进入唤醒状态"""
        self.state = STATE_AWAKE
        self._speak(WAKE_REPLY)
        self._reset_timeout()
        self.get_logger().info('🔔 已唤醒')

    def _enter_sleep(self):
        """进入沉睡状态"""
        self.state = STATE_SLEEP
        self._cancel_timeout()
        self._speak(DISMISS_REPLY)
        self.get_logger().info('💤 已沉睡')

    def _reset_timeout(self):
        """重置15秒超时计时器"""
        with self._timer_lock:
            if self._timeout_timer:
                self._timeout_timer.cancel()
            self._timeout_timer = threading.Timer(
                TIMEOUT_SEC, self._on_timeout)
            self._timeout_timer.daemon = True
            self._timeout_timer.start()

    def _cancel_timeout(self):
        """取消超时计时器"""
        with self._timer_lock:
            if self._timeout_timer:
                self._timeout_timer.cancel()
                self._timeout_timer = None

    def _on_timeout(self):
        """超时回调：自动回到沉睡"""
        self.get_logger().info(f'⏰ {TIMEOUT_SEC}秒无指令，自动沉睡')
        self._enter_sleep()

    def _speak(self, text):
        """发送TTS"""
        msg = String()
        msg.data = text
        self.pub_speak.publish(msg)

    # ══════════════════════════════════════════════════════════════════════════
    #  语音文本回调
    # ══════════════════════════════════════════════════════════════════════════

    def text_callback(self, msg):
        text = msg.data
        self.get_logger().info(f'收到文本: {text}')

        # ── 沉睡状态 ──
        if self.state == STATE_SLEEP:
            if self._is_wake_word(text):
                self._enter_awake()
            else:
                self.get_logger().info('💤 沉睡中，忽略')
            return

        # ── 唤醒状态 ──
        # 检查退下指令
        if self._is_dismiss(text):
            self._enter_sleep()
            return

        # 重置超时计时器
        self._reset_timeout()

        # 正常处理：传给LLM
        threading.Thread(
            target=self._parse_async, args=(text,), daemon=True).start()

    # ══════════════════════════════════════════════════════════════════════════
    #  LLM 解析
    # ══════════════════════════════════════════════════════════════════════════

    def _parse_async(self, text):
        try:
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

                buffer = self._extract_steps(buffer)

            if assistant_reply:
                with self._history_lock:
                    self.messages.append({"role": "user", "content": text})
                    self.messages.append({"role": "assistant", "content": assistant_reply})
                    max_msgs = 1 + self.max_history * 2
                    if len(self.messages) > max_msgs:
                        self.messages = [self.messages[0]] + self.messages[-(max_msgs - 1):]

        except Exception as e:
            self.get_logger().error(f'LLM调用失败: {e}')

    def _extract_steps(self, buffer):
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
