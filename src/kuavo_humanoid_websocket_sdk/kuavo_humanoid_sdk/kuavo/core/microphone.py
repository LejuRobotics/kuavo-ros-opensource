import os
import time
import numpy as np
from kuavo_humanoid_sdk.kuavo.core.ros.microphone import Microphone
import contextlib, sys
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.common.optional_deps import require_optional

def _require_audio_deps():
    require_optional(["funasr"], "audio", "Audio")
    from funasr import AutoModel
    return AutoModel



class RobotMicrophoneCore:
    """
    The core logic for handling wake-up word detection using audio data provided by WebSocket nodes.
    """
    def __init__(self, subscribe_topic="/micphone_data"):
        self.microphone = Microphone(subscribe_topic)
        self._should_stop = False

        # VAD and ASR models
        AutoModel = _require_audio_deps()
        self.vad_model = AutoModel(model="fsmn-vad", model_revision="v2.0.4", disable_update=True)
        self.asr_model = AutoModel(model="paraformer-zh-streaming", model_revision="v2.0.4", disable_update=True)

        # Configuration
        self.CHUNK = 1024
        self.target_sample_rate = 16000
        self.target_mic_keywords = ['夸父', '鲁班',]

        # Streaming recognition parameters
        self.audio_buffer = []
        self.vad_cache = {}
        self.asr_cache = {}
        self.silence_frames = 0
        self.max_silence_frames = int(1.0 * self.target_sample_rate / self.CHUNK)

        # Speech segment state management
        self.speech_segment_buffer = []
        self.is_in_speech = False
        self.speech_start_time = 0
        self.consecutive_speech_frames = 0
        self.min_speech_frames = 3

        # VAD config
        self.vad_chunk_size = 300
        self.vad_chunk_stride = int(self.vad_chunk_size * self.target_sample_rate / 1000)

        # ASR config
        self.asr_chunk_size = [0, 10, 5]
        self.asr_encoder_chunk_look_back = 4
        self.asr_decoder_chunk_look_back = 1
        self.asr_chunk_stride = self.asr_chunk_size[1] * 960

        SDKLogger.debug("The audio processor node is ready.")

    def stop(self):
        """Signal the wait loop to stop."""
        self._should_stop = True

    def wait_for_wake_word(self, timeout_sec=60, wake_word='鲁班鲁班'):
        """
        Actively pull audio data, process it and wait for wake-up word detection.
        Returns True if a wake-up word is detected within the timeout period, otherwise returns False.
        """
        hot_word = [wake_word] + self.target_mic_keywords
        start_time = time.time()

        # Reset all state
        self.audio_buffer = []
        self.vad_cache = {}
        self.asr_cache = {}
        self.speech_segment_buffer = []
        self.is_in_speech = False
        self.silence_frames = 0

        while not self._should_stop:
            if time.time() - start_time > timeout_sec:
                SDKLogger.debug("Timeout has been reached. No wake-up word was detected.")
                return False

            new_data = self.microphone.get_data()
            if new_data:

                audio_np = np.frombuffer(new_data, dtype=np.int16).astype(np.float32) / 32768.0

                self.audio_buffer.extend(audio_np)
                current_volume = np.sqrt(np.mean(audio_np**2))

                is_speaking = current_volume > 0.01

                if is_speaking:
                    self.silence_frames = 0
                else:
                    self.silence_frames += 1

                if len(self.audio_buffer) >= self.vad_chunk_stride:

                    vad_chunk = np.array(self.audio_buffer[:self.vad_chunk_stride], dtype=np.float64)

                    is_final = self.silence_frames >= self.max_silence_frames

                    try:
                        with self.suppress_output():
                            vad_res = self.vad_model.generate(input=vad_chunk, cache=self.vad_cache, is_final=is_final, chunk_size=self.vad_chunk_size)

                        has_speech = False
                        if len(vad_res) > 0 and "value" in vad_res[0] and vad_res[0]["value"]:
                            has_speech = True

                        if has_speech and not self.is_in_speech:
                            self.is_in_speech = True
                            self.speech_segment_buffer = []
                            SDKLogger.debug("🎤 检测到语音开始")

                        if self.is_in_speech:
                            self.speech_segment_buffer.extend(vad_chunk)

                        if is_final and self.is_in_speech:
                            SDKLogger.debug("🔍 语音段结束，开始识别...")

                            if len(self.speech_segment_buffer) > 0:
                                speech_segment = np.array(self.speech_segment_buffer, dtype=np.float64)

                                with self.suppress_output():
                                    asr_res = self.asr_model.generate(input=speech_segment, cache=self.asr_cache, is_final=True,
                                                            chunk_size=self.asr_chunk_size,
                                                            encoder_chunk_look_back=self.asr_encoder_chunk_look_back,
                                                            decoder_chunk_look_back=self.asr_decoder_chunk_look_back, hotword=hot_word)

                                if len(asr_res) > 0:
                                    recognized_text = ""
                                    if "value" in asr_res[0] and asr_res[0]["value"]:
                                        recognized_text = " ".join(asr_res[0]["value"])
                                    elif "text" in asr_res[0] and asr_res[0]["text"]:
                                        recognized_text = asr_res[0]["text"]

                                    if recognized_text:
                                        SDKLogger.debug(f"📝 识别结果: {recognized_text}")
                                        if wake_word in recognized_text:
                                            return True
                                    else:
                                        SDKLogger.debug("❌ 未能识别出内容")
                                else:
                                    SDKLogger.debug("❌ ASR识别失败")

                            self.is_in_speech = False
                            self.speech_segment_buffer = []
                            self.vad_cache = {}
                            self.asr_cache = {}
                            SDKLogger.debug("--- 语音段处理完成 ---")

                        self.audio_buffer = self.audio_buffer[self.vad_chunk_stride:]

                    except Exception as e:
                        SDKLogger.debug(f"处理错误: {e}")
            time.sleep(0.01)

    @contextlib.contextmanager
    def suppress_output(self):
        """Temporarily suppress all stdout and stderr output."""
        old_stdout = sys.stdout
        old_stderr = sys.stderr
        with open(os.devnull, 'w') as devnull:
            sys.stdout = devnull
            sys.stderr = devnull
            try:
                yield
            finally:
                sys.stdout = old_stdout
                sys.stderr = old_stderr
