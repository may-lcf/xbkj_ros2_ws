"""语音合成节点 - 阿里云 qwen-tts 流式合成 + aplay 管道播放

订阅 speak_text，流式合成 PCM 音频并直接通过管道推送到 aplay 播放。
优化: 流式PCM直推(不写临时文件)，播放队列防重叠。
"""

import base64
import os
import queue
import subprocess
import threading

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

        # 播放队列: 防止多条 TTS 重叠
        self._play_queue = queue.Queue()
        self._play_thread = threading.Thread(
            target=self._play_loop, daemon=True)
        self._play_thread.start()

        self.subscription = self.create_subscription(
            String, 'speak_text', self.speak_callback, 10)

        self.get_logger().info(
            f'语音合成节点已启动 (模型: qwen-tts 流式, 音色: {self.voice})')

    def speak_callback(self, msg):
        text = msg.data
        if not text:
            return
        self.get_logger().info(f'语音合成: {text}')
        self._play_queue.put(text)

    def _play_loop(self):
        """播放线程: 从队列逐条取出文本，流式合成并播放"""
        while True:
            text = self._play_queue.get()
            try:
                self._stream_speak(text)
            except Exception as e:
                self.get_logger().error(f'语音合成失败: {e}')

    def _stream_speak(self, text):
        """流式合成: DashScope PCM -> aplay 管道，首块到达即开始播放"""
        response = SpeechSynthesizer.call(
            model='qwen-tts',
            text=text,
            voice=self.voice,
            stream=True,
        )

        proc = None
        try:
            for chunk in response:
                audio = chunk.get('output', {}).get('audio', {})
                data_b64 = audio.get('data', '')
                if not data_b64:
                    continue

                raw = base64.b64decode(data_b64)
                if not raw:
                    continue

                # 首块到达时启动 aplay 进程 (PCM: 24000Hz, S16_LE, mono)
                if proc is None:
                    proc = subprocess.Popen(
                        ['aplay', '-D', self.audio_device,
                         '-f', 'S16_LE', '-r', '24000', '-c', '1', '-t', 'raw', '-q', '-'],
                        stdin=subprocess.PIPE,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )

                try:
                    proc.stdin.write(raw)
                except BrokenPipeError:
                    break

        finally:
            if proc is not None:
                try:
                    proc.stdin.close()
                except Exception:
                    pass
                try:
                    proc.wait(timeout=5)
                except Exception:
                    proc.kill()


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
