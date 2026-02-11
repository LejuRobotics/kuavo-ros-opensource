import os
import numpy as np
import rospy
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
    The core logic for handling wake-up word detection using audio data provided by ROS nodes.
    """
    def __init__(self, subscribe_topic="/micphone_data"):
        self.microphone = Microphone(subscribe_topic)
    
        # 创建VAD和ASR两个模型
        AutoModel = _require_audio_deps()
        self.vad_model = AutoModel(model="fsmn-vad", model_revision="v2.0.4", disable_update=True)
        self.asr_model = AutoModel(model="paraformer-zh-streaming", model_revision="v2.0.4", disable_update=True)
        
        # 配置参数
        self.CHUNK = 1024  # 每个缓冲区的帧数
        self.target_sample_rate = 16000  # ASR模型的目标采样率
        self.target_mic_keywords = ['夸父', '鲁班',]  # 识别麦克风设备的关键词

        # 流式识别相关参数
        self.audio_buffer = []  # 音频缓冲区
        self.vad_cache = {}  # VAD模型缓存
        self.asr_cache = {}  # ASR模型缓存
        self.silence_frames = 0  # 静音帧计数
        self.max_silence_frames = int(1.0 * self.target_sample_rate / self.CHUNK)  # 最大静音帧数（1秒）


        # 语音段状态管理
        self.speech_segment_buffer = []  # 语音段缓冲区
        self.is_in_speech = False  # 是否在语音段中
        self.speech_start_time = 0  # 语音开始时间
        self.consecutive_speech_frames = 0  # 连续语音帧计数
        self.min_speech_frames = 3  # 最小连续语音帧数，防止误触发

        # VAD配置
        self.vad_chunk_size = 300  # 毫秒，VAD模型的块大小
        self.vad_chunk_stride = int(self.vad_chunk_size * self.target_sample_rate / 1000)  # 3200 采样点

        # ASR配置
        self.asr_chunk_size = [0, 10, 5]  # [0, 10, 5] 表示600ms实时出字粒度
        self.asr_encoder_chunk_look_back = 4  # 编码器回看的块数
        self.asr_decoder_chunk_look_back = 1  # 解码器回看的块数
        self.asr_chunk_stride = self.asr_chunk_size[1] * 960  # 9600 采样点 (600ms * 16kHz)

        SDKLogger.debug("The audio processor node is ready.")

    def wait_for_wake_word(self, timeout_sec=60, wake_word='鲁班鲁班'):
        """
        Actively pull audio data, process it and wait for wake-up word detection.
        Returns True if a wake-up word is detected within the timeout period, otherwise returns False.
        """
        hot_word = [wake_word] + self.target_mic_keywords
        start_time = rospy.get_time()
        
        # 重置所有状态
        self.audio_buffer = []
        self.vad_cache = {}
        self.asr_cache = {}
        self.speech_segment_buffer = []
        self.is_in_speech = False
        self.silence_frames = 0
        
        while not rospy.is_shutdown():
            if rospy.get_time() - start_time > timeout_sec:
                SDKLogger.debug("Timeout has been reached. No wake-up word was detected.")
                return False

            new_data = self.microphone.get_data()
            if new_data:

                audio_np = np.frombuffer(new_data, dtype=np.int16).astype(np.float32) / 32768.0

                self.audio_buffer.extend(audio_np)
                # 检测是否有声音（简单的音量检测）
                current_volume = np.sqrt(np.mean(audio_np**2))
                
                # 调试：打印音量信息
                # print(f"current_volume: {current_volume:.6f}")
                
                # 调整音量阈值 - 对于归一化到[-1,1]的数据，静音应该在0.01以下
                is_speaking = current_volume > 0.01
                
                if is_speaking:
                    self.silence_frames = 0
                else:
                    self.silence_frames += 1
                
                # print(f"is_speaking: {is_speaking}")
                # 当缓冲区有足够的数据时进行VAD检测
                if len(self.audio_buffer) >= self.vad_chunk_stride:

                    # 提取一个VAD块的数据
                    vad_chunk = np.array(self.audio_buffer[:self.vad_chunk_stride], dtype=np.float64)
                    
                    # 判断是否为最后一个块（静音超过阈值）
                    is_final = self.silence_frames >= self.max_silence_frames
                    
                    try:
                        # 使用VAD检测语音活动
                        with self.suppress_output():
                            vad_res = self.vad_model.generate(input=vad_chunk, cache=self.vad_cache, is_final=is_final, chunk_size=self.vad_chunk_size)
                        
                        # print(vad_res)
                        # 检查VAD结果
                        has_speech = False
                        if len(vad_res) > 0 and "value" in vad_res[0] and vad_res[0]["value"]:
                            has_speech = True
                        
                        # 语音段管理
                        if has_speech and not self.is_in_speech:
                            # 开始新的语音段
                            self.is_in_speech = True
                            self.speech_segment_buffer = []
                            SDKLogger.debug("🎤 检测到语音开始")
                        
                        if self.is_in_speech:
                            # 在语音段中，累积音频数据
                            self.speech_segment_buffer.extend(vad_chunk)
                        
                        if is_final and self.is_in_speech:
                            # 语音段结束，进行ASR识别
                            SDKLogger.debug("🔍 语音段结束，开始识别...")
                            
                            if len(self.speech_segment_buffer) > 0:
                                # 将语音段数据转换为ASR格式
                                speech_segment = np.array(self.speech_segment_buffer, dtype=np.float64)
                                
                                # 使用ASR进行识别
                                with self.suppress_output():
                                    asr_res = self.asr_model.generate(input=speech_segment, cache=self.asr_cache, is_final=True, 
                                                            chunk_size=self.asr_chunk_size, 
                                                            encoder_chunk_look_back=self.asr_encoder_chunk_look_back, 
                                                            decoder_chunk_look_back=self.asr_decoder_chunk_look_back, hotword=hot_word)
                                
                                # 检查ASR结果
                                if len(asr_res) > 0:
                                    # 检查不同的结果字段
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
                            
                            # 重置语音段状态
                            self.is_in_speech = False
                            self.speech_segment_buffer = []
                            self.vad_cache = {}
                            self.asr_cache = {}
                            SDKLogger.debug("--- 语音段处理完成 ---")
                    
                        # 移除已处理的数据，保留一些重叠
                        # overlap = self.vad_chunk_stride // 2  # 50% 重叠
                        self.audio_buffer = self.audio_buffer[self.vad_chunk_stride:]
                            
                    except Exception as e:
                        SDKLogger.debug(f"处理错误: {e}")
            rospy.sleep(0.01)  # 减少睡眠时间以提高响应性

    # 创建输出抑制上下文管理器
    @contextlib.contextmanager
    def suppress_output(self):
        """临时抑制所有标准输出和错误输出"""
        # 保存原始的标准输出和错误输出
        old_stdout = sys.stdout
        old_stderr = sys.stderr
        # 重定向到空设备
        with open(os.devnull, 'w') as devnull:
            sys.stdout = devnull
            sys.stderr = devnull
            try:
                yield
            finally:
                # 恢复原始输出
                sys.stdout = old_stdout
                sys.stderr = old_stderr
