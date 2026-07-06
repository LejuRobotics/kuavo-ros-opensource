'''
Description: 统一音频播放节点 — loundspeaker (文件转换) + audio_stream_player (PyAudio 播放)
  /play_music srv:     immediate=False → 追加 (队列语义); immediate=True → 先停下立即播放
  /stop_music srv:   同步停止，返回时已静音 (推荐)
  /stop_music topic:  fire-and-forget 停止 (兼容, 推荐迁移到 service)
  /audio_data sub:     接收外部 PCM 推流 (TTS / llm_doubao.py 兼容, 自动重采样至声卡采样率)
  /audio_status srv:   查询当前播放状态
  /get_used_audio_buffer_size srv: 查询缓冲大小
  /audio_playback_status topic: STATUS_RATE Hz 状态发布
'''
#!/usr/bin/env python3
import os
import time
import uuid
import wave
import threading
import subprocess
import tempfile

import numpy as np
import rospy
from std_msgs.msg import Bool, Int16MultiArray, Header
from std_srvs.srv import Trigger, TriggerResponse

try:
    import pyaudio
except ImportError:
    os.system("sudo apt-get install python3-pyaudio -y")
    import pyaudio

try:
    import samplerate
except ImportError:
    samplerate = None

from kuavo_msgs.srv import playmusic, playmusicResponse
from kuavo_audio_player.srv import audio_status, audio_statusResponse
from kuavo_msgs.msg import AudioPlaybackStatus


TARGET_RATE = 16000
TARGET_CHANNELS = 1
SAMPLE_WIDTH = 2           # bytes (16-bit)
FRAMES_PER_BUFFER = 8192   # ~0.5 s 缓冲 (16 kHz mono): 高于常见 0.2 s, 在转码频繁时降低欠载
BUFFER_MAX_BYTES = 256 * 1024  # _pcm_buffer 上限 (防 TTS 推流过快)
STATUS_RATE = 10
STREAM_RESTART_RETRIES = 3     # ALSA 设备偶尔 transient busy; 3 次覆盖多数热插场景
STREAM_RESTART_DELAY = 0.5      # 0.5 s 等待内核释放声卡设备节点
MIN_DB = -60.0   # vol→0⁺ 时的 dB 下限 (−60dB≈振幅×0.001, 接近底噪); vol=0 本身走线性 0 完全静音


class AudioPlayerNode:

    def __init__(self):
        # 必须先 init_node, 否则声卡检测循环中无法响应 SIGINT
        rospy.init_node('audio_player_node')

        self._music_dir = rospy.get_param('music_path', '/home/lab/.config/lejuconfig/music')
        self._temp_dir = tempfile.gettempdir()
        self._temp_file = None

        # PCM 环形缓冲 (生产者: ROS spinner 线程 → _on_play_music/_on_audio_data 追加;
        # 消费者: PortAudio 高优先级回调线程 → _audio_callback 读取; _lock 保护读写)
        self._buf = bytearray()
        self._read_pos = 0
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()   # True → 回调返回 paAbort

        # ── 预分配 PortAudio 回调用缓冲区, 避免每次 np.zeros/.copy ──
        self._silence = np.zeros(FRAMES_PER_BUFFER, dtype=np.int16)

        self._pa = pyaudio.PyAudio()
        try:
            info = self._pa.get_default_output_device_info()
            self._dev_rate = int(info.get('defaultSampleRate', TARGET_RATE))
        except Exception:
            self._dev_rate = TARGET_RATE
        rospy.loginfo(f"声卡采样率: {self._dev_rate} Hz")

        self._stream = None

        # 异步等声卡 — 不阻塞 init_node 后的 ROS 可用性
        self._wait_sound_card_timer = rospy.Timer(rospy.Duration(3.0), self._try_start)

        rospy.Subscriber('audio_data', Int16MultiArray, self._on_audio_data, queue_size=10)
        rospy.Subscriber('stop_music', Bool, self._on_stop_music, queue_size=10)
        rospy.Service('stop_music', Trigger, self._on_stop_music_srv)
        rospy.Service('play_music', playmusic, self._on_play_music)
        rospy.Service('audio_status', audio_status, self._on_audio_status)
        rospy.Service('get_used_audio_buffer_size', Trigger, self._on_get_buffer_size)
        self._status_pub = rospy.Publisher('audio_playback_status', AudioPlaybackStatus, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / STATUS_RATE), self._on_status_timer)

    # 初始化

    def _try_start(self, event=None):
        """定时器回调: 检测声卡, 就绪后启动流。异步, 不阻塞 ROS。"""
        if self._stream is not None and self._stream.is_active():
            if self._wait_sound_card_timer is not None:
                self._wait_sound_card_timer.shutdown()
            return
        if not self._check_sound_card():
            rospy.logwarn_throttle(10, "未检测到播音设备, 继续等待...")
            return
        if self._stream is not None:
            return
        self._init_stream()
        if self._stream is not None:
            rospy.loginfo("audio_player_node 初始化完成")
            if self._wait_sound_card_timer is not None:
                self._wait_sound_card_timer.shutdown()

    def _init_stream(self):
        for attempt in range(STREAM_RESTART_RETRIES):
            try:
                if self._stream is not None:
                    try:
                        self._stream.stop_stream()
                        self._stream.close()
                    except Exception:
                        pass
                self._stop_evt.clear()
                self._stream = self._pa.open(
                    format=pyaudio.paInt16,
                    channels=TARGET_CHANNELS,
                    rate=self._dev_rate,
                    output=True,
                    frames_per_buffer=FRAMES_PER_BUFFER,
                    stream_callback=self._audio_callback,
                    start=True,
                )
                rospy.loginfo("音频流 (回调模式) 初始化成功")
                return True
            except Exception as e:
                rospy.logerr(f"音频流初始化失败 ({attempt + 1}): {e}")
                time.sleep(STREAM_RESTART_DELAY)
        rospy.logerr("音频流初始化失败，已达最大重试次数")
        return False

    # PortAudio 回调

    def _audio_callback(self, in_data, frame_count, time_info, status):
        """PortAudio 回调 — 音频高优先级线程, 禁止阻塞/分配大内存。"""
        if self._stop_evt.is_set():
            return (self._silence, pyaudio.paAbort)

        needed = frame_count * SAMPLE_WIDTH
        with self._lock:
            available = len(self._buf) - self._read_pos
            if available < needed:
                self._buf = self._buf[self._read_pos:]
                self._read_pos = 0
                available = len(self._buf)

            take = min(available, needed)
            if take > 0:
                result = np.frombuffer(
                    memoryview(self._buf)[self._read_pos:self._read_pos + take],
                    dtype=np.int16).copy()
                needed -= take
                self._read_pos += take
                if needed > 0:
                    result = np.pad(result, (0, frame_count - len(result)), mode='constant')
                # 音量缩放在 _on_play_music 加载时一次性完成 (不进音频线程, TTS 路径不受影响)
                return (result, pyaudio.paContinue)

            # 空白 → 静音
            return (self._silence, pyaudio.paContinue)

    # 内部: 音量缩放

    @staticmethod
    def _volume_to_gain(vol):
        """volume 字段 (0–100) → ffmpeg volume 滤镜的 dB 增益字符串。

        映射: vol 线性映射到 dB 区间 [MIN_DB, 0], 即 vol=100→0dB(原声),
        vol=0→MIN_DB(近静音)。让 ffmpeg 负责实际的 dB→振幅换算
        (10^(dB/20)), 不在手算曲线。

        返回 (dB 字符串, is_mute)。vol==0 时 is_mute=True, 调用方
        走静音短路 (跳过 ffmpeg 全幅转码); 其余返回 "X.XXXXXXdB"
        形式字符串, is_mute=False。
        """
        v = max(0, min(vol, 100)) / 100.0
        if v <= 0.0:
            return ('0', True)
        db = MIN_DB + (0.0 - MIN_DB) * v    # MIN_DB→0 线性插值
        return (f"{db:.5f}dB", False)

    # 内部: 停止

    def _buffered_bytes(self):
        with self._lock:
            return max(0, len(self._buf) - self._read_pos)

    def _is_playing(self):
        return self._buffered_bytes() > 0

    def _clear_buf(self):
        with self._lock:
            self._buf = bytearray()
            self._read_pos = 0

    def _close_stream(self):
        """关闭当前流 (丢弃 ALSA DMA), 不重建。"""
        self._stop_evt.set()
        try:
            if self._stream is not None:
                self._stream.stop_stream()
                self._stream.close()
        except Exception:
            pass
        self._stream = None

    def _stop_and_recreate(self):
        """停止: 关闭流 (丢弃 ALSA DMA) → 清空缓冲 → 重建流。"""
        self._close_stream()
        self._clear_buf()
        self._init_stream()
        self._cleanup_temp()

    # 服务: /play_music
    #   req.immediate=True  → 先停止当前, 再加载新文件 (打断+播新, 无竞态)
    #   req.immediate=False → 追加到缓冲 (队列语义, 不打断, 默认)

    def _on_play_music(self, req):
        if req.immediate:
            self._stop_and_recreate()

        music_file = os.path.join(self._music_dir, req.music_number)
        if not os.path.exists(music_file):
            rospy.logerr(f"音频文件不存在: {music_file}")
            return playmusicResponse(success_flag=False)

        rospy.loginfo(f"播放: {music_file}  volume={req.volume}"
                      f"{' (打断)' if req.immediate else ''}")

        self._cleanup_temp()

        vol_filter, is_mute = self._volume_to_gain(req.volume)

        # ffmpeg 转码 + volume 滤镜缩放
        af = 'volume=0' if is_mute else f'volume={vol_filter}'
        wav_path = self._convert_to_wav(music_file, af)
        if wav_path is None:
            return playmusicResponse(success_flag=False)
        self._temp_file = wav_path

        try:
            with wave.open(wav_path, 'rb') as wf:
                if wf.getsampwidth() != SAMPLE_WIDTH:
                    rospy.logerr(f"不支持的采样宽度: {wf.getsampwidth()}")
                    return playmusicResponse(success_flag=False)
                raw = wf.readframes(wf.getnframes())
        except Exception as e:
            rospy.logerr(f"读取 WAV 失败: {e}")
            return playmusicResponse(success_flag=False)

        with self._lock:
            self._buf.extend(raw)

        dur = len(raw) / (self._dev_rate * SAMPLE_WIDTH)
        rospy.loginfo(f"音频就绪: {len(raw)} bytes ({dur:.1f}s), 已追加到缓冲")
        return playmusicResponse(success_flag=True)

    def _convert_to_wav(self, music_file, af):
        """ffmpeg → 声卡采样率 mono 16-bit WAV; 音量由 volume 滤镜缩放。

        永远转码 (无 fast path): 即使源 wav 格式匹配也走 ffmpeg, 保证音量对所有文件
        一致生效。TTS 走 /audio_data 不过此函数, 故音量缩放天然不影响 TTS。

        Args:
            af: ffmpeg -af 参数值 (如 "volume=0" 或 "volume=-30.00000dB"), 由
                _volume_to_gain 生成。调用方决定静音短路 (vol=0 用线性 0)。
        """
        t0 = time.time()
        try:
            temp = os.path.join(self._temp_dir, f"audio_{uuid.uuid4()}.wav")
            cmd = [
                'ffmpeg', '-i', music_file,
                '-f', 'wav', '-acodec', 'pcm_s16le',
                '-ar', str(self._dev_rate), '-ac', str(TARGET_CHANNELS),
            ]
            if af is not None:
                cmd += ['-af', af]
            cmd += ['-y', temp]
            r = subprocess.run(cmd, capture_output=True, text=True)
            if r.returncode != 0:
                rospy.logerr(f"FFmpeg 转换失败: {r.stderr}")
                return None
            rospy.loginfo(f"音频转换完成 ({time.time() - t0:.2f}s)")
            return temp
        except Exception as e:
            rospy.logerr(f"音频转换异常: {e}")
            return None

    # /stop_music: topic (兼容) + service (推荐)

    def _on_stop_music(self, msg):
        """话题 /stop_music — fire-and-forget, 推荐迁移到同名 service。"""
        if msg.data:
            rospy.logwarn_throttle(300, "/stop_music topic 已过时, 推荐迁移到 /stop_music service (std_srvs/Trigger)")
            self._stop_and_recreate()
            rospy.loginfo("/stop_music: 已停止")

    def _on_stop_music_srv(self, req):
        """服务 /stop_music — 同步停止, 返回时已静音。"""
        self._stop_and_recreate()
        rospy.loginfo("/stop_music service: 已停止")
        return TriggerResponse(success=True, message="stopped")

    # 话题: /audio_data (TTS PCM 推流兼容, 自动重采样至声卡采样率)

    def _on_audio_data(self, msg):
        try:
            audio = np.array(msg.data, dtype=np.int16)

            # 提取源采样率 (兼容老消息格式)
            src_rate = TARGET_RATE
            for dim in msg.layout.dim:
                if dim.label == "sample_rate" and dim.size > 0:
                    src_rate = int(dim.size)
                    break

            if src_rate != self._dev_rate and samplerate is not None:
                ratio = self._dev_rate / src_rate
                audio = audio.astype(np.float32) / 32768.0
                audio = samplerate.resample(audio, ratio, converter_type='sinc_fastest')
                audio = np.clip(audio * 32768.0, -32768, 32767).astype(np.int16)

            data = audio.tobytes()
            with self._lock:
                if len(self._buf) - self._read_pos + len(data) > BUFFER_MAX_BYTES:
                    # TTS 推流过快 — 清空旧的, 保留新数据
                    self._buf = bytearray()
                    self._read_pos = 0
                self._buf.extend(data)
        except Exception as e:
            rospy.logerr(f"处理 /audio_data 失败: {e}")

    # 服务: /audio_status

    def _on_audio_status(self, req):
        return audio_statusResponse(is_playing=self._is_playing())

    # 服务: /get_used_audio_buffer_size

    def _on_get_buffer_size(self, req):
        return TriggerResponse(success=True, message=str(self._buffered_bytes()))

    # 话题: /audio_playback_status (STATUS_RATE Hz)

    def _on_status_timer(self, event):
        buf = self._buffered_bytes()
        msg = AudioPlaybackStatus()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.buffer_size = buf
        if self._is_playing():
            msg.playing = True
            msg.message = f"Playing, buffer: {buf} bytes"
        elif buf > 0:
            msg.playing = False
            msg.message = f"Audio stopped, buffer: {buf} bytes"
        else:
            msg.playing = False
            msg.message = "Idle"
        try:
            self._status_pub.publish(msg)
        except Exception:
            pass

        # 流健康检查: 非主动停止 + 流异常 → 重建
        if (self._stream is not None
                and not self._stream.is_active()
                and not self._stop_evt.is_set()):
            rospy.logwarn("流异常停止 (非主动)，尝试重建")
            self._init_stream()

    # 声卡检测 & 清理

    @staticmethod
    def _check_sound_card():
        try:
            r = subprocess.run(
                'pactl list | grep -i Headphone', shell=True,
                capture_output=True, text=True)
            if r.stdout.strip() and "not available" not in r.stdout:
                return True
            r = subprocess.run(
                'pactl list | grep -i Speaker', shell=True,
                capture_output=True, text=True)
            if r.stdout.strip():
                return True
            r = subprocess.run(
                'aplay -l | grep -i Audio', shell=True,
                capture_output=True, text=True)
            return bool(r.stdout.strip())
        except Exception:
            return False

    def _cleanup_temp(self):
        if self._temp_file:
            try:
                os.remove(self._temp_file)
            except OSError:
                pass
            self._temp_file = None

    def shutdown(self):
        if self._wait_sound_card_timer is not None:
            self._wait_sound_card_timer.shutdown()
        self._close_stream()
        if self._pa:
            try:
                self._pa.terminate()
            except Exception:
                pass
        self._cleanup_temp()
        rospy.loginfo("audio_player_node 已关闭")

    def run(self):
        rospy.on_shutdown(self.shutdown)
        rospy.spin()


if __name__ == '__main__':
    AudioPlayerNode().run()
