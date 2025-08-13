import os
import sys
from contextlib import contextmanager
try:    
    import pyaudio
except ImportError:
    print("pyaudio 未安装，先安装 pyaudio")
    command = "sudo apt-get install portaudio19-dev -y && sudo apt-get install python3-pyaudio -y"
    os.system(command)  
    import pyaudio
import rospy
from kuavo_msgs.msg import AudioReceiverData
import numpy as np
from scipy.signal import resample
try:
    import samplerate
except ImportError:
    print("samplerate 未安装，先安装 samplerate")
    command = "pip install samplerate -i https://mirrors.aliyun.com/pypi/simple/ --no-input"
    os.system(command)
    import samplerate

@contextmanager
def suppress_alsa_error():
    """
    A context manager to suppress ALSA error messages from underlying C libraries.
    It redirects stderr to /dev/null.
    """
    try:
        devnull = os.open(os.devnull, os.O_WRONLY)
        old_stderr_fileno = os.dup(2)
        sys.stderr.flush()
        os.dup2(devnull, 2)
        os.close(devnull)
        yield
    finally:
        os.dup2(old_stderr_fileno, 2)
        os.close(old_stderr_fileno)

class AudioReceiver:
    """
    A class to receive audio from a microphone and publish it to a ROS topic.
    """

    def __init__(self, node_name="audio_receiver_node", topic_name="/micphone_data",
                 chunk_size=1024, record_seconds=5,
                 target_mic_keywords=["Jieli Technology", "USB Composite Device"],
                 target_sample_rate=16000): # 添加目标采样率参数
        """
        Initializes the AudioReceiver.

        Args:
            node_name (str): ROS node name.
            topic_name (str): ROS topic name to publish audio data.
            chunk_size (int): Number of audio frames per buffer.
            record_seconds (int): Placeholder for total recording duration (can be continuous).
            target_mic_keywords (list): List of keywords to identify the desired microphone.
        """
        rospy.init_node(node_name)
        self.publisher = rospy.Publisher(topic_name, AudioReceiverData, queue_size=10)
        
        with suppress_alsa_error():
            self.audio = pyaudio.PyAudio()
        self.FORMAT = pyaudio.paInt16  # Fixed audio format: 16-bit integer
        self.CHUNK = chunk_size
        self.RECORD_SECONDS = record_seconds # This is more for example/initialization, can run continuously

        self.stream = None
        self.input_device_index = None
        self.mic_channels = None
        self.mic_rate = None
        self.target_mic_keywords = target_mic_keywords
        self.target_sample_rate = target_sample_rate

        self._find_and_initialize_microphone()

    def _find_and_initialize_microphone(self):
        """
        Finds the specified microphone and sets up its audio parameters.
        """
        rospy.loginfo("--- Available Audio Input Devices ---")
        with suppress_alsa_error():
            info = self.audio.get_host_api_info_by_index(0)
            num_devices = info.get('deviceCount')

            input_devices = []
            for i in range(0, num_devices):
                device_info = self.audio.get_device_info_by_host_api_device_index(0, i)
                if device_info.get('maxInputChannels') > 0:
                    input_devices.append(device_info)
        
        for device in input_devices:
            rospy.loginfo(f"Device Index: {device['index']}, Name: {device['name']}, "
                            f"Input Channels: {device['maxInputChannels']}, Default Sample Rate: {device['defaultSampleRate']}")

        if not input_devices:
            rospy.logerr("Error: No audio input devices found. Please check microphone connection or drivers.")
            rospy.signal_shutdown("No audio input devices.")
            return

        selected_device_info = None
        for device in input_devices:
            if any(keyword in device['name'] for keyword in self.target_mic_keywords):
                self.input_device_index = device['index']
                selected_device_info = device
                rospy.loginfo(f"\n--- Successfully selected device: {device['name']} (Index: {device['index']}) ---")
                break

        if self.input_device_index is None:
            rospy.logerr(f"未找到与关键词 {self.target_mic_keywords} 匹配的设备，节点将退出。")
            rospy.signal_shutdown("未找到指定的麦克风设备。")
            return

        if selected_device_info:
            self.mic_rate = int(selected_device_info['defaultSampleRate'])
            self.mic_channels = int(selected_device_info['maxInputChannels'])
            rospy.loginfo(f"Device parameters: Sample Rate={self.mic_rate}Hz, Channels={self.mic_channels}")
        else:
            rospy.logerr("Could not determine device parameters. Using default values, which might cause issues.")
            # Fallback to hardcoded defaults if selection fails
            self.mic_rate = 48000
            self.mic_channels = 1

        try:
            self.stream = self.audio.open(format=self.FORMAT,
                                          channels=self.mic_channels,
                                          rate=self.mic_rate,
                                          input=True,
                                          input_device_index=self.input_device_index,
                                          frames_per_buffer=self.CHUNK)
            rospy.loginfo("Microphone stream successfully opened.")
            rospy.loginfo(f"Listening with parameters: Sample Rate={self.mic_rate}Hz, "
                          f"Channels={self.mic_channels}, Format=paInt16, Chunk Size={self.CHUNK}")

        except Exception as e:
            rospy.logerr(f"Failed to open microphone stream: {e}")

    def start_listening_and_publishing(self):
        """
        开始连续监听麦克风并发布数据。
        """
        if self.stream is None:
            rospy.logerr("音频流未初始化。无法开始监听。")
            return

        rospy.loginfo(f"开始音频流并发布数据 (原始采样率: {self.mic_rate}Hz, 目标采样率: {self.target_sample_rate}Hz)...")
        try:
            while not rospy.is_shutdown():
                data = self.stream.read(self.CHUNK, exception_on_overflow=False) 
                
                # 将字节数据转换为 NumPy 数组进行处理。注意 pyaudio 默认输出的是 little-endian。
                # samplerate 期望 float32 或 float64 类型。
                audio_np = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0 # 归一化到 -1.0 到 1.0

                # --- 执行降采样 ---
                if self.mic_rate != self.target_sample_rate:

                    downsampled_audio_float = samplerate.resample(
                        audio_np, 
                        self.target_sample_rate / self.mic_rate,
                        converter_type='sinc_fastest'
                    )
                    
                    # 转换回 int16。需要先将浮点数数据乘以 32767，然后进行裁剪以防止溢出。
                    downsampled_audio = np.clip(downsampled_audio_float * 32767, -32768, 32767).astype(np.int16)
                    

                    # rospy.loginfo(f"已将 {self.mic_rate}Hz 的音频数据使用 samplerate 降采样到 {self.target_sample_rate}Hz. "
                    #               f"原始数据点: {len(audio_np)}, 降采样后数据点: {len(downsampled_audio)}")

                    
                    processed_data = downsampled_audio.tobytes() # 转换回字节
                else:
                    processed_data = data # 如果采样率一致，则不进行降采样，直接使用原始数据

                audio_msg = AudioReceiverData()
                audio_msg.data = processed_data 
                self.publisher.publish(audio_msg)
                # rospy.loginfo(f"发布了 {len(processed_data)} 字节的音频数据。") 

        except KeyboardInterrupt:
            rospy.loginfo("用户中断音频录制。")
        except Exception as e:
            rospy.logerr(f"音频流过程中发生错误: {e}")
        finally:
            self.stop_listening()

    def stop_listening(self):
        """
        Stops the audio stream and terminates PyAudio.
        """
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
            rospy.loginfo("Audio stream stopped and closed.")
        if self.audio:
            self.audio.terminate()
            rospy.loginfo("PyAudio terminated.")
        rospy.loginfo("Audio Receiver node shut down.")

if __name__ == '__main__':
    try:
        receiver = AudioReceiver(
            node_name="kuavo_audio_receiver",
            topic_name="/micphone_data",
            chunk_size=1024, # 根据你的延迟/缓冲区需求进行调整
            target_sample_rate=16000 # 设置目标降采样频率为 16kHz
        )
        receiver.start_listening_and_publishing()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception in main: {e}")