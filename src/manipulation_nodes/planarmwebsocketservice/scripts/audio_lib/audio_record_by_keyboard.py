# TODO: 空格开关录制音频

import rospy
from kuavo_msgs.msg import AudioReceiverData
import collections
import numpy as np # 用于将bytes转换为可处理的数值数组
import os
import wave
import datetime
try:
    from pynput import keyboard
except ImportError:
    print("pynput 未安装，正在安装 pynput...")
    command = "pip install pynput --no-input"
    os.system(command)
    from pynput import keyboard
# 如果你的模型需要特定的音频处理库，例如 librosa, scipy等，请在这里导入
# import librosa
# from scipy.signal import resample # 假设模型需要特定采样率

class AudioRecordByKeayboard:
    """
    ROS subscriber node to process audio data from a microphone,
    implementing a sliding window for keyword spotting and on-demand recording.
    """

    def __init__(self, node_name="audio_processor_node", subscribe_topic="/micphone_data"):
        rospy.init_node(node_name, anonymous=True)
        rospy.loginfo(f"ROS Audio Processor Node '{node_name}' initialized.")
        rospy.loginfo(f"Subscribing to topic: {subscribe_topic}")

        # --- 音频参数 (必须与发布者匹配) ---
        self.SAMPLE_RATE = 16000  # Hz, 接收的音频数据采样率 (与发布者一致)
        self.CHANNELS = 1         # 声道数
        self.BIT_RESOLUTION = 16  # 位深，例如 16 bits
        self.BYTES_PER_SAMPLE = self.BIT_RESOLUTION // 8 # 每个采样点占用的字节数 (16位 = 2字节)

        # --- 滑动窗口参数 ---
        self.WINDOW_DURATION_SEC = 3.0 # 识别窗口的持续时间（秒）
        self.OVERLAP_STEP_SEC = 1.0    # 窗口每次滑动的步长（秒），例如1秒意味着2秒重叠

        # 计算窗口和步长的字节数
        self.WINDOW_SIZE_BYTES = int(self.WINDOW_DURATION_SEC * self.SAMPLE_RATE * self.BYTES_PER_SAMPLE * self.CHANNELS)
        self.OVERLAP_STEP_BYTES = int(self.OVERLAP_STEP_SEC * self.SAMPLE_RATE * self.BYTES_PER_SAMPLE * self.CHANNELS)

        rospy.loginfo(f"Processing window size: {self.WINDOW_DURATION_SEC} sec ({self.WINDOW_SIZE_BYTES} bytes)")
        rospy.loginfo(f"Overlap step: {self.OVERLAP_STEP_SEC} sec ({self.OVERLAP_STEP_BYTES} bytes)")

        # 使用 deque 作为高效的环形缓冲区/队列来存储音频数据
        self.audio_buffer = collections.deque()
        self.current_buffer_size_bytes = 0

        # --- 录音相关参数 ---
        self.is_recording = False
        self.wave_file = None
        self.recording_path = "wav_recordings"
        if not os.path.exists(self.recording_path):
            os.makedirs(self.recording_path)
            rospy.loginfo(f"创建录音文件夹: {self.recording_path}")
        self._start_keyboard_listener()

        # --- ROS 订阅者设置 ---    
        self.subscriber = rospy.Subscriber(subscribe_topic, AudioReceiverData, self._audio_callback)
        rospy.on_shutdown(self._cleanup)

        rospy.loginfo("Audio processor node ready.")
        rospy.loginfo("按空格键开始/停止录制音频。")

    def _start_keyboard_listener(self):
        """启动一个非阻塞的键盘监听器。"""
        listener = keyboard.Listener(on_press=self.on_press)
        listener.daemon = True  # 确保主线程退出时监听线程也会退出
        listener.start()

    def on_press(self, key):
        """键盘按键事件的回调函数。"""
        if key == keyboard.Key.space:
            self.toggle_recording()

    def toggle_recording(self):
        """切换录制状态。"""
        self.is_recording = not self.is_recording
        if self.is_recording:
            # 开始录制
            if self.wave_file: # 安全措施
                self.wave_file.close()

            now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = os.path.join(self.recording_path, f"recording_{now}.wav")
            
            self.wave_file = wave.open(filename, 'wb')
            self.wave_file.setnchannels(self.CHANNELS)
            self.wave_file.setsampwidth(self.BYTES_PER_SAMPLE)
            self.wave_file.setframerate(self.SAMPLE_RATE)
            
            rospy.loginfo(f"开始录制, 文件将保存到: {filename}")
        else:
            # 停止录制
            if self.wave_file:
                rospy.loginfo(f"录制结束。文件已保存。")
                self.wave_file.close()
                self.wave_file = None

    def _audio_callback(self, msg: AudioReceiverData):
        """
        Callback function for incoming AudioData messages.
        It saves data for recording and accumulates data in a buffer for keyword spotting.
        """
        # --- 录音数据写入 ---
        # 检查是否处于录制状态，并直接将接收到的原始字节流写入文件
        if self.is_recording and self.wave_file:
            self.wave_file.writeframes(msg.data)
            
        # --- 关键词识别的缓冲处理 ---
        # 将接收到的 bytes 数据添加到 deque 尾部
        self.audio_buffer.append(msg.data)
        self.current_buffer_size_bytes += len(msg.data)

        # 当缓冲区中的数据足够进行至少一个处理窗口的提取时
        while self.current_buffer_size_bytes >= self.WINDOW_SIZE_BYTES:
            # 1. 从队列头部提取当前窗口的数据
            current_window_bytes = bytearray()
            temp_buffer_size = 0
            for chunk in self.audio_buffer:
                if temp_buffer_size + len(chunk) <= self.WINDOW_SIZE_BYTES:
                    current_window_bytes.extend(chunk)
                    temp_buffer_size += len(chunk)
                else:
                    # 如果当前chunk的一部分就能填满窗口
                    remaining_bytes_needed = self.WINDOW_SIZE_BYTES - temp_buffer_size
                    current_window_bytes.extend(chunk[:remaining_bytes_needed])
                    break # 窗口已满，退出循环

            # 2. 将 bytes 转换为模型可处理的格式 (例如 NumPy 数组)
            # 使用 np.frombuffer 将 bytes 转换为 int16 数组
            # 注意：如果你的模型需要浮点数或不同的采样率，在这里进行转换和重采样
            audio_array = np.frombuffer(current_window_bytes, dtype=np.int16)
            
            # --- 在这里调用你的关键词识别模型 ---
            # 例如:
            # is_keyword_detected = self._run_keyword_model(audio_array)
            # if is_keyword_detected:
            #     rospy.loginfo(f"Keyword '鲁班鲁班' detected in this {self.WINDOW_DURATION_SEC}s window!")
            # else:
            #     rospy.loginfo(f"No keyword detected in this {self.WINDOW_DURATION_SEC}s window.")
            
            # 示例: 简单打印数据长度 (实际会调用模型)
            rospy.loginfo(f"Processing a {self.WINDOW_DURATION_SEC}s window ({len(audio_array)} samples) - size: {len(current_window_bytes)} bytes")
            # --- 关键词识别模型调用结束 ---

            # 3. 滑动窗口：从队列头部移除 `OVERLAP_STEP_BYTES` 的数据
            bytes_removed = 0
            while self.audio_buffer and bytes_removed < self.OVERLAP_STEP_BYTES:
                first_chunk = self.audio_buffer[0]
                if bytes_removed + len(first_chunk) <= self.OVERLAP_STEP_BYTES:
                    # 整个 chunk 都需要移除
                    bytes_removed += len(first_chunk)
                    self.audio_buffer.popleft()
                else:
                    # 只移除 chunk 的一部分
                    remaining_to_remove = self.OVERLAP_STEP_BYTES - bytes_removed
                    self.audio_buffer[0] = first_chunk[remaining_to_remove:]
                    bytes_removed += remaining_to_remove
            self.current_buffer_size_bytes -= bytes_removed

            # 检查是否还有足够的剩余数据来形成下一个窗口
            if self.current_buffer_size_bytes < self.WINDOW_SIZE_BYTES:
                break
    
    def _cleanup(self):
        """
        在ROS节点关闭时执行清理操作。
        """
        rospy.loginfo("Audio processor node shutting down.")
        if self.is_recording and self.wave_file:
            rospy.loginfo("节点关闭，正在关闭录音文件...")
            self.wave_file.close()
            self.wave_file = None

    def run(self):
        """
        Starts the ROS node spinning.
        """
        rospy.spin()
        rospy.loginfo("Audio processor node shutting down.")

if __name__ == '__main__':
    try:
        processor = AudioRecordByKeayboard()
        processor.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception in main: {e}")