import os
import torch
import torch.nn as nn
import librosa
import numpy as np
import rospy
from kuavo_msgs.msg import AudioReceiverData
import collections
import rospkg
from wake_model import WakeRobanModel
import threading


class WakeRobanDetector:
    """
    提供一个接口，使用训练好的模型检测唤醒词 "roban roban"。
    """
    def __init__(self, model_path, n_mfcc=40, max_pad_len=200, target_sr=16000, threshold=0.6):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.n_mfcc = n_mfcc
        self.max_pad_len = max_pad_len
        self.target_sr = target_sr
        self.threshold = threshold

        if not os.path.exists(model_path):
            raise FileNotFoundError(f"模型权重文件未找到")
        
        self.model = self._load_model(model_path)
        rospy.loginfo(f"WakeRobanDetector 初始化完成。使用设备: {self.device}")

    def _load_model(self, model_path):
        """加载预训练的 WakeRobanModel 模型。"""
        model = WakeRobanModel(n_mfcc=self.n_mfcc, max_pad_len=self.max_pad_len).to(self.device)
        model.load_state_dict(torch.load(model_path, map_location=self.device))
        model.eval()
        rospy.loginfo(f"唤醒词模型已成功加载")
        return model
    
    def _preprocess_audio_for_model(self, audio_data_array, original_sample_rate):
        """
        将原始音频数组预处理为适合模型的 MFCC 特征。
        进行重采样和填充/截断。
        """
        # 确保 audio_data_array 是 float32 类型，以供 librosa 使用
        audio_float = audio_data_array.astype(np.float32)

        # 如果采样率不同则进行重采样
        if original_sample_rate != self.target_sr:
            processed_audio = librosa.resample(y=audio_float, orig_sr=original_sample_rate, target_sr=self.target_sr)
        else:
            processed_audio = audio_float

        # 计算 MFCC 特征
        # 对于 16kHz 音频，通常使用 hop_length=160 来获得 10ms 的帧
        mfccs = librosa.feature.mfcc(y=processed_audio, sr=self.target_sr, n_mfcc=self.n_mfcc, hop_length=160)

        # 填充或截断 MFCC 特征到 max_pad_len
        if mfccs.shape[1] > self.max_pad_len:
            mfccs = mfccs[:, :self.max_pad_len]
        else:
            pad_width = self.max_pad_len - mfccs.shape[1]
            mfccs = np.pad(mfccs, pad_width=((0, 0), (0, pad_width)), mode='constant')

        # 添加批次和通道维度 (batch_size, channels, n_mfcc, time_steps)
        mfccs_tensor = torch.tensor(mfccs, dtype=torch.float32).unsqueeze(0).unsqueeze(0)
        return mfccs_tensor
    
    def detect_wake_word(self, audio_data_array, original_sr):
        """
        在给定的音频数据 numpy 数组中检测唤醒词。
        返回一个元组: (is_detected: bool, probability: float)
        """
        try:
            mfccs_tensor = self._preprocess_audio_for_model(
                audio_data_array,
                original_sample_rate=original_sr
            )
            with torch.no_grad(): # 在推理阶段禁用梯度计算
                output = self.model(mfccs_tensor.to(self.device))
                probability = torch.sigmoid(output).item() # 将模型的原始输出通过 sigmoid 激活得到概率
            
            is_detected = probability > self.threshold
            return is_detected, probability
        except Exception as e:
            rospy.logerr(f"唤醒词检测过程中发生错误: {e}")
            return False, 0.0


class WakeRoban:
    """
    ROS 订阅者节点，用于处理来自麦克风的音频数据，
    实现滑动窗口进行关键词识别，并与 WakeRobanDetector 交互。
    """
    def __init__(self, node_name="wake_roban_processor", subscribe_topic="/micphone_data"):
        if not rospy.get_node_uri():
            rospy.init_node(node_name)
        rospy.loginfo(f"ROS 音频处理器节点 '{node_name}' 初始化。")
        rospy.loginfo(f"订阅主题: {subscribe_topic}")

        package_name = 'planarmwebsocketservice' 
        package_path = rospkg.RosPack().get_path(package_name) 
        model_name = "wake_word_cnn_lstm.pth"
        self.model_full_path = os.path.join(package_path, 'scripts', 'audio_lib', 'model', model_name) 

        # 音频参数
        self.SAMPLE_RATE = 16000
        self.CHANNELS = 1
        self.BIT_RESOLUTION = 16
        self.BYTES_PER_SAMPLE = self.BIT_RESOLUTION // 8 # 每个样本的字节数

        # 滑动窗口参数
        self.WINDOW_DURATION_SEC = 2.0 # 窗口持续时间（秒）
        self.OVERLAP_STEP_SEC = 0.5    # 重叠步长（秒）

        self.WINDOW_SIZE_BYTES = int(self.WINDOW_DURATION_SEC * self.SAMPLE_RATE * self.BYTES_PER_SAMPLE * self.CHANNELS)
        self.OVERLAP_STEP_BYTES = int(self.OVERLAP_STEP_SEC * self.SAMPLE_RATE * self.BYTES_PER_SAMPLE * self.CHANNELS)
        
        # 确保 OVERLAP_STEP_BYTES 不为零或相对于 WINDOW_SIZE_BYTES 过大
        if self.OVERLAP_STEP_BYTES <= 0:
            rospy.logwarn("计算的重叠步长为零或负数，默认为窗口大小的一半。")
            self.OVERLAP_STEP_BYTES = self.WINDOW_SIZE_BYTES // 2
        elif self.OVERLAP_STEP_BYTES > self.WINDOW_SIZE_BYTES:
            rospy.logwarn("重叠步长大于窗口大小。将重叠设置为窗口大小。")
            self.OVERLAP_STEP_BYTES = self.WINDOW_SIZE_BYTES

        rospy.loginfo(f"处理窗口大小: {self.WINDOW_DURATION_SEC} 秒 ({self.WINDOW_SIZE_BYTES} 字节)")
        rospy.loginfo(f"重叠步长: {self.OVERLAP_STEP_SEC} 秒 ({self.OVERLAP_STEP_BYTES} 字节)")

        self.audio_buffer = collections.deque() # 使用双端队列存储音频数据块
        self.current_buffer_size_bytes = 0    # 当前缓冲区中的总字节数

        self.detector = WakeRobanDetector(model_path=self.model_full_path, target_sr=self.SAMPLE_RATE)
        
        self.subscriber = rospy.Subscriber(subscribe_topic, AudioReceiverData, self._audio_callback)
        rospy.on_shutdown(self._cleanup)

        self.wake_word_detected = False
        self._detection_event = threading.Event()

        rospy.loginfo("音频处理器节点已准备就绪。")

    def _audio_callback(self, msg: AudioReceiverData):
        """
        传入音频数据的回调函数。将数据添加到缓冲区，并在积累足够数据后处理窗口。
        """
        # 如果唤醒词已被检测到，则停止处理新音频以进行检测
        if self.wake_word_detected:
            return

        self.audio_buffer.append(msg.data)
        self.current_buffer_size_bytes += len(msg.data)

        while self.current_buffer_size_bytes >= self.WINDOW_SIZE_BYTES:
            # 重构完整的字节字符串以提取窗口
            full_buffer = b''.join(self.audio_buffer)
            current_window_bytes = full_buffer[:self.WINDOW_SIZE_BYTES]

            # 将字节转换为 int16 类型的 numpy 数组
            audio_array = np.frombuffer(current_window_bytes, dtype=np.int16)
            
            is_detected, probability = self.detector.detect_wake_word(audio_array, self.SAMPLE_RATE)
            
            if is_detected:
                rospy.loginfo(f"唤醒词 'roban' 已 DETECTED! 概率: {probability:.4f}")
                self.wake_word_detected = True      # 设置检测标志
                self._detection_event.set()         # 发出检测发生信号，唤醒等待线程
                return 
            else:
                rospy.loginfo(f"未检测到唤醒词 'roban'。概率: {probability:.4f}")

            # 滑动窗口: 从双端队列中移除旧数据
            bytes_to_remove = self.OVERLAP_STEP_BYTES
            self.current_buffer_size_bytes -= bytes_to_remove
            
            bytes_removed_from_deque = 0
            while bytes_removed_from_deque < bytes_to_remove and self.audio_buffer:
                chunk_len = len(self.audio_buffer[0])
                if bytes_removed_from_deque + chunk_len <= bytes_to_remove:
                    bytes_removed_from_deque += chunk_len
                    self.audio_buffer.popleft() # 移除整个块
                else:
                    # 移除部分块
                    self.audio_buffer[0] = self.audio_buffer[0][bytes_to_remove - bytes_removed_from_deque:]
                    bytes_removed_from_deque = bytes_to_remove
    
    def wait_for_wake_word(self, timeout_sec=None):
        """
        Public method to wait for a wake word detection.
        Returns True if wake word is detected within timeout, False otherwise.
        """
        self.wake_word_detected = False
        self._detection_event.clear()

        rospy.loginfo(f"等待唤醒词检测 {timeout_sec if timeout_sec else '无限'} 秒...")
        
        start_time = rospy.get_time()
        
        while not rospy.is_shutdown():
            remaining_time = timeout_sec
            if timeout_sec is not None:
                elapsed_time = rospy.get_time() - start_time
                remaining_time = timeout_sec - elapsed_time
                if remaining_time <= 0:
                    rospy.loginfo("超时已到达。未检测到唤醒词。")
                    return False

            timeout_interval = min(0.1, remaining_time) if timeout_sec is not None else 0.1

            detected = self._detection_event.wait(timeout=timeout_interval)

            if detected:
                rospy.loginfo("唤醒词已检测到！")
                return True

        rospy.loginfo("ROS 节点已关闭，等待被中断。")
        return False

    def _cleanup(self):
        """节点关闭时执行清理操作。"""
        rospy.loginfo("音频处理器节点正在关闭。")


if __name__ == "__main__":
    audio_node = WakeRoban()

    if audio_node.wait_for_wake_word(timeout_sec=60):
        rospy.loginfo("唤醒词已成功检测！")
    else:
        rospy.loginfo("在指定超时时间内未检测到唤醒词。")

    rospy.loginfo("退出主执行块。")
