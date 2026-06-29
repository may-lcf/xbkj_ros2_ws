"""语音识别节点 - 阿里云 Paraformer-realtime-v2 流式 ASR

音频采集: arecord 48kHz -> 降采样 16kHz -> VAD门控 -> 云端流式识别。
优化: 回调直接发布(消除100ms轮询)，webrtcvad本地VAD(静音不发送云端)。
"""

import os
import subprocess
import threading
import time

import numpy as np
import webrtcvad
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import dashscope
from dashscope.audio.asr import Recognition, RecognitionCallback

# VAD 每帧 20ms @ 16kHz = 320 samples = 640 bytes
_VAD_FRAME_SAMPLES = 320
_VAD_FRAME_BYTES = _VAD_FRAME_SAMPLES * 2
# 语音结束后额外发送的帧数 (20ms * 15 = 300ms)，防止句尾被截断
_HANGOVER_FRAMES = 15


class ASRCallback(RecognitionCallback):

    def __init__(self, publisher, logger=None):
        super().__init__()
        self.publisher = publisher
        self.logger = logger

    def on_event(self, result):
        sentence = result.get_sentence()
        if sentence and sentence.get('text'):
            text = sentence['text'].strip()
            is_end = sentence.get('is_sentence_end', False)
            if text and text[-1] in '。？！?!.~':
                is_end = True
            if self.logger:
                self.logger.info(f'ASR: text="{text}", is_end={is_end}')
            if is_end and text:
                msg = String()
                msg.data = text
                self.publisher.publish(msg)
                if self.logger:
                    self.logger.info(f'✅ 识别结果: {text}')

    def on_error(self, result):
        if self.logger:
            self.logger.error(f'ASR错误: {result}')

    def on_complete(self):
        if self.logger:
            self.logger.info('ASR连接完成')


class VoiceRecognitionNode(Node):

    def __init__(self):
        super().__init__('voice_recognition_node')
        self.declare_parameter('api_key', '')
        self.declare_parameter('asr_model', 'paraformer-realtime-v2')
        self.declare_parameter('audio_device', 'plughw:2,0')
        self.declare_parameter('record_rate', 48000)
        self.declare_parameter('asr_rate', 16000)
        self.declare_parameter('vad_aggressiveness', 2)

        api_key = self.get_parameter('api_key').value
        if not api_key:
            api_key = os.environ.get('DASHSCOPE_API_KEY', '')
        if not api_key:
            self.get_logger().error('未设置 api_key')
            return

        dashscope.api_key = api_key
        self.model = self.get_parameter('asr_model').value
        self.audio_device = self.get_parameter('audio_device').value
        self.record_rate = self.get_parameter('record_rate').value
        self.asr_rate = self.get_parameter('asr_rate').value
        self.vad_agg = self.get_parameter('vad_aggressiveness').value

        # 直接发布，不经过 queue/timer
        self.publisher_ = self.create_publisher(String, 'voice_text', 10)
        self.callback = ASRCallback(self.publisher_, self.get_logger())

        self._running = True
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'语音识别节点已启动 (模型: {self.model}, 设备: {self.audio_device}, '
            f'VAD: {self.vad_agg})')

    def _listen_loop(self):
        RECORD_CHUNK = 9600
        chunk_bytes = RECORD_CHUNK * 2
        ratio = self.record_rate // self.asr_rate  # 3

        vad = webrtcvad.Vad(self.vad_agg)

        while self._running:
            proc = None
            recognition = None
            try:
                test = subprocess.run(
                    ['arecord', '-D', self.audio_device, '-f', 'S16_LE',
                     '-r', str(self.record_rate), '-c', '1', '-d', '1',
                     '-q', '/dev/null'],
                    capture_output=True, timeout=5)
                if test.returncode != 0:
                    self.get_logger().warn(
                        f'麦克风设备不可用: {test.stderr.decode().strip()}')
                    time.sleep(5)
                    continue

                self.get_logger().info(f'正在聆听... ({self.audio_device})')

                proc = subprocess.Popen(
                    ['arecord', '-D', self.audio_device, '-f', 'S16_LE',
                     '-r', str(self.record_rate), '-c', '1',
                     '-t', 'raw', '-q', '-'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE)

                recognition = Recognition(
                    model=self.model,
                    callback=self.callback,
                    format='pcm',
                    sample_rate=self.asr_rate,
                    language_hints=['zh', 'en'])
                recognition.start()

                sending = False
                silence_count = 0

                while self._running:
                    data = proc.stdout.read(chunk_bytes)
                    if not data:
                        stderr = proc.stderr.read().decode().strip()
                        if stderr:
                            self.get_logger().warn(f'arecord退出: {stderr}')
                        break
                    samples = np.frombuffer(data, dtype=np.int16)
                    resampled = samples[::ratio]
                    pcm_bytes = resampled.tobytes()

                    # VAD: 逐帧检测 (每帧 20ms = 320 samples)
                    has_speech = False
                    for offset in range(0, len(pcm_bytes) - _VAD_FRAME_BYTES + 1,
                                        _VAD_FRAME_BYTES):
                        frame = pcm_bytes[offset:offset + _VAD_FRAME_BYTES]
                        if len(frame) == _VAD_FRAME_BYTES and vad.is_speech(
                                frame, self.asr_rate):
                            has_speech = True
                            break

                    if has_speech:
                        sending = True
                        silence_count = 0
                    elif sending:
                        silence_count += 1
                        if silence_count > _HANGOVER_FRAMES:
                            sending = False
                            silence_count = 0

                    if sending:
                        recognition.send_audio_frame(pcm_bytes)

            except subprocess.TimeoutExpired:
                self.get_logger().warn('麦克风检测超时')
                time.sleep(5)
            except Exception as e:
                self.get_logger().warn(f'识别出错: {e}')
                time.sleep(3)
            finally:
                try:
                    if recognition:
                        recognition.stop()
                except Exception:
                    pass
                if proc:
                    proc.terminate()
                    try:
                        proc.wait(timeout=2)
                    except Exception:
                        proc.kill()

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
