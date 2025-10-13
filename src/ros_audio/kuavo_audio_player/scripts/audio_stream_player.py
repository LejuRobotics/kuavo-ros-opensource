'''
Description: 本节点直接占用音频设备，对外订阅audio_data话题，然后播放音频。注意输入的音频数据流默认是16000Hz
'''
#!/usr/bin/env python3
import rospy
import numpy as np
import threading
import queue
import os
try:    
    import pyaudio
except ImportError:
    print("pyaudio 未安装，先安装 pyaudio")
    command = "sudo apt-get install python3-pyaudio -y"
    os.system(command)  
    import pyaudio
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from kuavo_msgs.srv import SetLEDMode, SetLEDModeRequest
from rosnode import get_node_names
import subprocess
import time
import signal
try:
    import samplerate
except ImportError:
    print("samplerate 未安装，先安装 samplerate")
    command = "pip install samplerate -i https://mirrors.aliyun.com/pypi/simple/ --no-input"
    os.system(command)
    import samplerate

class AudioStreamPlayerNode:
    # 音频配置常量
    DEFAULT_SAMPLE_RATE = 16000      # 默认接收到音频流采样率
    DEFAULT_CHANNELS = 1             # 默认声道数
    CHUNK_SIZE = 8192               # 音频块大小
    BUFFER_MAX_SIZE = 50            # 缓冲区最大块数（音频播放缓冲区的大小）
    SAMPLE_WIDTH_BYTES = 2          # 16-bit，即2字节
    
    # 音频数据处理常量
    FLOAT32_DIVISOR = 32768.0       # int16转float32的除数
    INT16_MIN = -32768              # int16最小值
    INT16_MAX = 32767               # int16最大值
    
    # 超时和重试配置
    QUEUE_PUT_TIMEOUT = 1           # 队列放入超时时间(秒)
    QUEUE_GET_TIMEOUT = 1           # 队列获取超时时间(秒)
    THREAD_JOIN_TIMEOUT = 1         # 线程加入超时时间(秒)
    EMPTY_COUNT_THRESHOLD = 100     # 空队列计数阈值
    
    # 主题队列配置
    SUBSCRIBER_QUEUE_SIZE = 10      # 订阅者队列大小
    
    def __init__(self):
        while not self.check_sound_card():
            print("未检测到播音设备，不启用播音功能！")
            # 退出节点
            # rospy.signal_shutdown("未检测到播音设备，不启用播音功能！")
            # exit(0)
            # 无限延时
            time.sleep(1000000)

        rospy.init_node('audio_stream_player_node')
        self.audio_subscriber = rospy.Subscriber('audio_data', Int16MultiArray, self.audio_callback, queue_size=self.SUBSCRIBER_QUEUE_SIZE)
        self.stop_music_subscriber = rospy.Subscriber('stop_music', Bool, self.stop_music_callback, queue_size=self.SUBSCRIBER_QUEUE_SIZE)
        # 获取当前音频缓冲区已经使用的大小
        self.buffer_status_service = rospy.Service('get_used_audio_buffer_size', Trigger, self.get_used_audio_buffer_size_callback)
        rospy.loginfo("已创建 audio_data 话题的订阅者（流式播放节点）")

        # 检查led_controller_node节点是否启动
        self.is_led_control = self.check_led_controller_node()

        # 初始化 PyAudio 播放
        self.chunk_size = self.CHUNK_SIZE
        self.buffer_queue = queue.Queue(maxsize=self.BUFFER_MAX_SIZE)  # 限制最大缓冲块数
        self.playing = True
        self.empty_count = 0
        self.p = pyaudio.PyAudio()
        self.is_breathing = False
        self.colors = [(0, 0, 255)] * 10
        
        # 获取声卡默认采样率
        try:
            device_info = self.p.get_default_output_device_info()
            self.rate = int(device_info.get('defaultSampleRate', self.DEFAULT_SAMPLE_RATE))
            rospy.loginfo(f"检测到声卡默认采样率: {self.rate}Hz")
        except Exception as e:
            rospy.logwarn(f"无法获取声卡默认采样率，使用默认值{self.DEFAULT_SAMPLE_RATE}Hz: {e}")
            self.rate = self.DEFAULT_SAMPLE_RATE
            
        self.channels = self.DEFAULT_CHANNELS
        self.stream = self.p.open(format=pyaudio.paInt16,
                                 channels=self.channels,
                                 rate=self.rate,
                                 output=True,
                                 frames_per_buffer=self.chunk_size)

        # 播放线程
        self.play_thread = threading.Thread(target=self.play_from_buffer)
        self.play_thread.daemon = True
        self.play_thread.start()

    def check_led_controller_node(self, timeout=10):
        node_name = '/led_controller_node'
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                active_nodes = get_node_names()
                
                if node_name in active_nodes:
                    rospy.loginfo(f"节点 {node_name} 检测成功，已启动")
                    return True
                    
                time.sleep(0.5)
                
            except Exception as e:
                rospy.logwarn(f"检查节点时发生错误: {e}")
                time.sleep(0.5)
        
        rospy.logwarn(f"节点 {node_name} 在 {timeout} 秒内未检测到")
        return False

    def check_sound_card(self):
        """
        检查声卡状态，特别是耳机和扬声器的可用性
        """
        # 检查耳机状态
        try:
            headphone_command = 'pactl list | grep -i Headphone'
            headphone_result = subprocess.run(headphone_command, shell=True, capture_output=True, text=True)
            print(headphone_result.stdout)
            if not bool(headphone_result.stdout.strip()):
                print(f"不存在耳机设备")
            # 检查耳机是否不可用
            else:
                headphone_available = "not available" not in headphone_result.stdout
                print(f"耳机状态: {'可用' if headphone_available else '不可用'}")
                if headphone_available:
                    return True
            # 检查扬声器状态
            speaker_command = 'pactl list | grep -i Speaker'
            speaker_result = subprocess.run(speaker_command, shell=True, capture_output=True, text=True)
            
            # 检查扬声器是否存在
            speaker_exists = bool(speaker_result.stdout.strip())
            print(f"扬声器状态: {'存在' if speaker_exists else '不存在'}")
            if speaker_exists:
                return True
            
            # root用户下检查扬声器状态
            root_speaker_command = 'aplay -l | grep -i Audio'
            root_speaker_result = subprocess.run(root_speaker_command, shell=True, capture_output=True, text=True)
            print(root_speaker_result.stdout)
            root_speaker_exists = bool(root_speaker_result.stdout.strip())
            print(f"root扬声器状态: {'存在' if root_speaker_exists else '不存在'}")
            if not root_speaker_exists:
                print(f"不存在扬声器设备")
            else:
                return True
            
            return False
            
        except Exception as e:
            print(f"检查声卡状态时出错: {str(e)}")
            return False
        
    def resample_audio(self, audio_chunk, source_sample_rate=None):
        if source_sample_rate is None:
            source_sample_rate = self.DEFAULT_SAMPLE_RATE
        try:
            # 将 int16 数据转换为 float32 范围 [-1, 1]
            audio_chunk = audio_chunk.astype(np.float32) / self.FLOAT32_DIVISOR
            resample_ratio = self.rate / source_sample_rate
            audio_chunk = samplerate.resample(audio_chunk, resample_ratio, converter_type='sinc_fastest')
            # 重新转为 int16
            audio_chunk = np.clip(audio_chunk * self.FLOAT32_DIVISOR, self.INT16_MIN, self.INT16_MAX).astype(np.int16)
            return audio_chunk
        except Exception as e:
            rospy.logerr(f"音频重采样失败: {e}")
            return np.zeros(self.chunk_size, dtype=np.int16)

    def audio_callback(self, msg):
        try:
            audio_chunk = np.array(msg.data, dtype=np.int16)
            
            # 检查消息中是否包含采样率信息
            source_sample_rate = self.DEFAULT_SAMPLE_RATE  # 默认采样率
            for dim in msg.layout.dim:
                if dim.label == "sample_rate" and dim.size > 0: # ru
                    source_sample_rate = int(dim.size)
                    break
            
            # 重采样到当前设备采样率
            audio_chunk = self.resample_audio(audio_chunk, source_sample_rate)
            
            # 放入播放队列
            self.buffer_queue.put(audio_chunk, timeout=self.QUEUE_PUT_TIMEOUT)  # 超时避免卡死
        except queue.Full:
            rospy.logwarn("音频缓冲区已满，丢弃音频块")
        except Exception as e:
            rospy.logerr(f"处理音频数据失败: {e}")


    def play_breathing(self,control_mode):
        rospy.wait_for_service('control_led')
    
        try:
            # 创建服务客户端
            led_service = rospy.ServiceProxy('control_led', SetLEDMode)
            
            # 创建请求
            request = SetLEDModeRequest()
            request.mode = control_mode  # 设置模式为1（呼吸模式）
            request.color1 = self.colors[0]
            request.color2 = self.colors[1]
            request.color3 = self.colors[2]
            request.color4 = self.colors[3]
            request.color5 = self.colors[4]
            request.color6 = self.colors[5]
            request.color7 = self.colors[6]
            request.color8 = self.colors[7]
            request.color9 = self.colors[8]
            request.color10 = self.colors[9]
            # print(request)
            
            # 调用服务
            response = led_service(request)
            
            # 输出结果
            if response.success:
                    rospy.loginfo("LED设置成功")
            else:
                    rospy.logerr("LED设置失败")
        except Exception as e:
                rospy.logerr(f"呼吸模式播放失败: {e}")

    def play_from_buffer(self):
        """音频播放线程，从队列中获取音频数据并直接播放"""
        while self.playing and not rospy.is_shutdown():
            try:
                chunk = self.buffer_queue.get(timeout=self.QUEUE_GET_TIMEOUT)
                
                if not self.is_breathing and self.is_led_control:
                    self.play_breathing(1)
                    self.is_breathing = not self.is_breathing
                self.stream.write(chunk.tobytes())
            except queue.Empty:
                if self.is_breathing and self.is_led_control:
                    self.play_breathing(0)
                    self.is_breathing = not self.is_breathing
                if(self.empty_count > self.EMPTY_COUNT_THRESHOLD):
                    rospy.logdebug("缓冲区为空，等待音频输入")
                    self.empty_count = 0
                self.empty_count += 1
            except Exception as e:
                rospy.logerr(f"播放缓冲区音频失败: {e}")

    def stop_music_callback(self, msg):
        """停止当前正在播放的音频"""
        if msg.data:
            try:
                # 清空缓冲区
                with self.buffer_queue.mutex:
                    self.buffer_queue.queue.clear()
                rospy.loginfo("已停止当前音频播放并清空缓冲区")
                return True
            except Exception as e:
                rospy.logerr(f"停止音频播放时出错: {e}")
                return False

    def get_used_audio_buffer_size_callback(self, request):
        response = TriggerResponse()
        response.success = True
        used_size = self.buffer_queue.qsize()
        response.message = f"{used_size}"
        return response

    def shutdown(self):
        """关闭节点时的清理工作"""
        self.playing = False
        if self.play_thread.is_alive():
            self.play_thread.join(timeout=self.THREAD_JOIN_TIMEOUT)
        
        if hasattr(self, 'stream') and self.stream:
            self.stream.stop_stream()
            self.stream.close()
            
        if hasattr(self, 'p') and self.p:
            self.p.terminate()
            
        rospy.loginfo("播放已停止，音频资源已释放")

    def run(self):
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

if __name__ == '__main__':
    player_node = AudioStreamPlayerNode()
    player_node.run() 