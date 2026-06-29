"""语音合成节点 - 阿里云 qwen-tts 云端实时合成

订阅 speak_text，调用 dashscope qwen-tts 合成语音并通过声卡播放。
"""

import os
import subprocess
import tempfile
import threading
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import dashscope
from dashscope.audio.qwen_tts import SpeechSynthesizer


class VoiceSynthesisNode(Node):

    def __init__(self):
        super().__init__('voice_synthesis_node')
        self.declare_parameter('api_key', '')
        self.declare_parameter('voice', 'Cherry')
        self.declare_parameter('audio_device', 'default')

        api_key = self.get_parameter('api_key').value
        if not api_key:
            api_key = os.environ.get('DASHSCOPE_API_KEY', '')
        if not api_key:
            self.get_logger().error('未设置 api_key')
            return

        dashscope.api_key = api_key
        self.voice = self.get_parameter('voice').value
        self.audio_device = self.get_parameter('audio_device').value
        self._wav_path = os.path.join(tempfile.gettempdir(), 'tts_output.wav')

        self.subscription = self.create_subscription(
            String, 'speak_text', self.speak_callback, 10)

        self.get_logger().info(
            f'语音合成节点已启动 (模型: qwen-tts, 音色: {self.voice})')

    def speak_callback(self, msg):
        text = msg.data
        if not text:
            return
        self.get_logger().info(f'语音合成: {text}')
        threading.Thread(
            target=self._speak, args=(text,), daemon=True).start()

    def _speak(self, text):
        try:
            response = SpeechSynthesizer.call(
                model='qwen-tts',
                text=text,
                voice=self.voice,
            )
            audio_url = response['output']['audio']['url']
            urllib.request.urlretrieve(audio_url, self._wav_path)
            subprocess.run(
                ['aplay', '-D', self.audio_device, self._wav_path],
                capture_output=True, timeout=30)
        except Exception as e:
            self.get_logger().error(f'语音合成失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceSynthesisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
