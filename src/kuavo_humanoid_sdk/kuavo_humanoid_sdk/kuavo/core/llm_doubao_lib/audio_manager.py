import asyncio
import uuid
import queue
import threading
import time
import random
from typing import Optional, Dict, Any
import wave
import pyaudio
import signal
from dataclasses import dataclass

from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.kuavo.core.llm_doubao_lib import config
from kuavo_humanoid_sdk.kuavo.core.llm_doubao_lib.realtime_dialog_client import RealtimeDialogClient


@dataclass
class AudioConfig:
    """音频配置数据类"""
    format: str
    bit_size: int
    channels: int
    sample_rate: int
    chunk: int


class AudioDeviceManager:
    """音频设备管理类，处理音频输入输出"""

    def __init__(self, input_config: AudioConfig, output_config: AudioConfig):
        self.input_config = input_config
        self.output_config = output_config
        self.pyaudio = pyaudio.PyAudio()
        self.input_stream: Optional[pyaudio.Stream] = None
        self.output_stream: Optional[pyaudio.Stream] = None

    def open_input_stream(self) -> pyaudio.Stream:
        """打开音频输入流"""
        # p = pyaudio.PyAudio()
        self.input_stream = self.pyaudio.open(
            format=self.input_config.bit_size,
            channels=self.input_config.channels,
            rate=self.input_config.sample_rate,
            input=True,
            frames_per_buffer=self.input_config.chunk
        )
        return self.input_stream

    def open_output_stream(self) -> pyaudio.Stream:
        """打开音频输出流"""
        self.output_stream = self.pyaudio.open(
            format=self.output_config.bit_size,
            channels=self.output_config.channels,
            rate=self.output_config.sample_rate,
            output=True,
            frames_per_buffer=self.output_config.chunk
        )
        return self.output_stream

    def cleanup(self) -> None:
        """清理音频设备资源"""
        for stream in [self.input_stream, self.output_stream]:
            if stream:
                stream.stop_stream()
                stream.close()
        self.pyaudio.terminate()


class DialogSession:
    """对话会话管理类"""

    def __init__(self, ws_config: Dict[str, Any], enable_signal_handler: bool = True):
        self.session_id = str(uuid.uuid4())
        self.client = RealtimeDialogClient(config=ws_config, session_id=self.session_id)
        self.audio_device = AudioDeviceManager(
            AudioConfig(**config.input_audio_config),
            AudioConfig(**config.output_audio_config)
        )

        self.is_running = True
        self.is_session_finished = False
        self.is_user_querying = False
        self.is_sending_chat_tts_text = False
        self.audio_buffer = b''

        # 只在主线程中设置信号处理器
        if enable_signal_handler:
            try:
                signal.signal(signal.SIGINT, self._keyboard_signal)
            except ValueError as e:
                # 如果不在主线程中，忽略这个错误
                SDKLogger.warning(f"Warning: Cannot set signal handler (not in main thread): {e}")
        
        # 初始化音频队列和输出流
        self.audio_queue = queue.Queue()
        self.output_stream = self.audio_device.open_output_stream()
        # 启动播放线程
        self.is_recording = True
        self.is_playing = True
        self.player_thread = threading.Thread(target=self._audio_player_thread)
        self.player_thread.daemon = True
        self.player_thread.start()

    def _audio_player_thread(self):
        """音频播放线程"""
        while self.is_playing:
            try:
                # 从队列获取音频数据
                audio_data = self.audio_queue.get(timeout=1.0)
                if audio_data is not None:
                    self.output_stream.write(audio_data)
            except queue.Empty:
                # 队列为空时等待一小段时间
                time.sleep(0.1)
            except Exception as e:
                SDKLogger.error(f"音频播放错误: {e}")
                time.sleep(0.1)

    def handle_server_response(self, response: Dict[str, Any]) -> None:
        if response == {}:
            return
        """处理服务器响应"""
        if response['message_type'] == 'SERVER_ACK' and isinstance(response.get('payload_msg'), bytes):
            # SDKLogger.info(f"\n接收到音频数据: {len(response['payload_msg'])} 字节")
            if self.is_sending_chat_tts_text:
                return
            audio_data = response['payload_msg']
            self.audio_queue.put(audio_data)
            self.audio_buffer += audio_data
        elif response['message_type'] == 'SERVER_FULL_RESPONSE':
            SDKLogger.info(f"服务器响应: {response}")
            event = response.get('event')
            payload_msg = response.get('payload_msg', {})

            if event == 450:
                SDKLogger.info(f"清空缓存音频: {response['session_id']}")
                while not self.audio_queue.empty():
                    try:
                        self.audio_queue.get_nowait()
                    except queue.Empty:
                        continue
                self.is_user_querying = True

            if event == 350 and self.is_sending_chat_tts_text and payload_msg.get("tts_type") == "chat_tts_text":
                while not self.audio_queue.empty():
                    try:
                        self.audio_queue.get_nowait()
                    except queue.Empty:
                        continue
                self.is_sending_chat_tts_text = False

            if event == 459:
                self.is_user_querying = False
        elif response['message_type'] == 'SERVER_ERROR':
            SDKLogger.error(f"服务器错误: {response['payload_msg']}")
            raise Exception("服务器错误")

    def _keyboard_signal(self, sig, frame):
        SDKLogger.info(f"receive keyboard Ctrl+C")
        self.is_recording = False
        self.is_playing = False
        self.is_running = False

    async def receive_loop(self):
        try:
            while True:
                response = await self.client.receive_server_response()
                self.handle_server_response(response)
                if 'event' in response and (response['event'] == 152 or response['event'] == 153):
                    SDKLogger.info(f"receive session finished event: {response['event']}")
                    self.is_session_finished = True
                    break
        except asyncio.CancelledError:
            SDKLogger.info("接收任务已取消")
        except Exception as e:
            SDKLogger.error(f"接收消息错误: {e}")

    async def process_microphone_input(self) -> None:
        await self.client.say_hello()
        """处理麦克风输入"""
        stream = self.audio_device.open_input_stream()
        SDKLogger.info("已打开麦克风，请讲话...")

        while self.is_recording:
            try:
                # 添加exception_on_overflow=False参数来忽略溢出错误
                audio_data = stream.read(config.input_audio_config["chunk"], exception_on_overflow=False)
                save_pcm_to_wav(audio_data, "input.pcm")
                await self.client.task_request(audio_data)
                await asyncio.sleep(0.01)  # 避免CPU过度使用
            except Exception as e:
                SDKLogger.error(f"读取麦克风数据出错: {e}")
                await asyncio.sleep(0.1)  # 给系统一些恢复时间

    async def start(self) -> None:
        """启动对话会话"""
        try:
            await self.client.connect()
            asyncio.create_task(self.process_microphone_input())
            asyncio.create_task(self.receive_loop())

            while self.is_running:
                await asyncio.sleep(0.1)

            await self.client.finish_session()
            while not self.is_session_finished:
                await asyncio.sleep(0.1)
            await self.client.finish_connection()
            await asyncio.sleep(0.1)
            await self.client.close()
            SDKLogger.info(f"dialog request logid: {self.client.logid}")
            # 将下发的 PCM 数据按输出音频配置保存为 WAV 文件
            save_audio_to_wav_file(self.audio_buffer, "output.wav")
        except Exception as e:
            SDKLogger.error(f"会话错误: {e}")
        finally:
            self.audio_device.cleanup()


def save_pcm_to_wav(pcm_data: bytes, filename: str) -> None:
    """保存PCM数据为WAV文件"""
    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(config.input_audio_config["channels"])
        wf.setsampwidth(2)  # paInt16 = 2 bytes
        wf.setframerate(config.input_audio_config["sample_rate"])
        wf.writeframes(pcm_data)


def save_audio_to_pcm_file(audio_data: bytes, filename: str) -> None:
    """保存原始PCM音频数据到文件"""
    if not audio_data:
        SDKLogger.info("No audio data to save.")
        return
    try:
        with open(filename, 'wb') as f:
            f.write(audio_data)
    except IOError as e:
        SDKLogger.error(f"Failed to save pcm file: {e}")


def save_audio_to_wav_file(audio_data: bytes, filename: str) -> None:
    """保存PCM格式音频数据为WAV文件（使用输出音频配置）"""
    if not audio_data:
        SDKLogger.info("No audio data to save.")
        return
    try:
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(config.output_audio_config["channels"])
            # paFloat32 = 4 bytes per sample
            wf.setsampwidth(4)
            wf.setframerate(config.output_audio_config["sample_rate"])
            wf.writeframes(audio_data)
    except IOError as e:
        SDKLogger.error(f"Failed to save wav file: {e}")
