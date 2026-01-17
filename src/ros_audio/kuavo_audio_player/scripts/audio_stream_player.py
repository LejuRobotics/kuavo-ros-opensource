'''
Description: 改进版音频流播放节点，增加了流恢复机制和错误处理
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
from kuavo_msgs.msg import AudioPlaybackStatus
from std_msgs.msg import Header
from rosnode import get_node_names
import subprocess
import time
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
    BUFFER_MAX_SIZE = 500            # 缓冲区最大块数（音频播放缓冲区的大小）
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
    AUDIO_FINISH_THRESHOLD = 1      # 音频播放完成判断阈值（1秒无数据认为播放完成）
    STREAM_RESTART_MAX_RETRIES = 3  # 流重启最大重试次数
    STREAM_RESTART_DELAY = 0.5       # 流重启延迟（秒）
    
    # 主题队列配置
    SUBSCRIBER_QUEUE_SIZE = 10
    
    # 状态发布配置
    STATUS_PUBLISH_RATE = 10        # 状态发布频率 (Hz)
    
    def __init__(self):
        while not self.check_sound_card():
            print("未检测到播音设备，不启用播音功能！")
            time.sleep(1000000)

        rospy.init_node('audio_stream_player_node')
        self.audio_subscriber = rospy.Subscriber('audio_data', Int16MultiArray, self.audio_callback, queue_size=self.SUBSCRIBER_QUEUE_SIZE)
        self.stop_music_subscriber = rospy.Subscriber('stop_music', Bool, self.stop_music_callback, queue_size=self.SUBSCRIBER_QUEUE_SIZE)
        # 获取当前音频缓冲区已经使用的大小
        self.buffer_status_service = rospy.Service('get_used_audio_buffer_size', Trigger, self.get_used_audio_buffer_size_callback)
        # 创建音频播放状态发布器
        self.playback_status_publisher = rospy.Publisher('audio_playback_status', AudioPlaybackStatus, queue_size=10)
        rospy.loginfo("已创建 audio_data 话题的订阅者（流式播放节点）")

        # 检查led_controller_node节点是否启动
        self.is_led_control = self.check_led_controller_node()

        # 初始化 PyAudio 播放
        self.chunk_size = self.CHUNK_SIZE
        self.buffer_queue = queue.Queue(maxsize=self.BUFFER_MAX_SIZE)  # 限制最大缓冲块数
        self.playing = True  # 控制播放线程运行状态
        self.is_audio_playing = False  # 音频是否正在播放状态
        self.buffer_queue = queue.Queue(maxsize=self.BUFFER_MAX_SIZE)
        self.playing = True
        self.empty_count = 0
        self.stream_lock = threading.Lock()  # 添加流操作锁
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
        
        # 初始化音频流
        self.stream = None
        self.init_audio_stream()

        # 播放线程
        self.play_thread = threading.Thread(target=self.play_from_buffer)
        self.play_thread.daemon = True
        self.play_thread.start()
        
        # 创建状态发布定时器
        self.status_timer = rospy.Timer(rospy.Duration(1.0/self.STATUS_PUBLISH_RATE), self.status_timer_callback)
        rospy.loginfo(f"已创建状态发布定时器，频率: {self.STATUS_PUBLISH_RATE}Hz")

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
    def init_audio_stream(self):
        """初始化音频流，带重试机制"""
        for attempt in range(self.STREAM_RESTART_MAX_RETRIES):
            try:
                with self.stream_lock:
                    if self.stream is not None:
                        try:
                            self.stream.stop_stream()
                            self.stream.close()
                        except:
                            pass
                    
                    self.stream = self.p.open(
                        format=pyaudio.paInt16,
                        channels=self.channels,
                        rate=self.rate,
                        output=True,
                        frames_per_buffer=self.chunk_size,
                        # 增加缓冲区数量以减少 underrun
                        stream_callback=None,
                        start=True
                    )
                    rospy.loginfo("音频流初始化成功")
                    return True
            except Exception as e:
                rospy.logerr(f"初始化音频流失败 (尝试 {attempt + 1}/{self.STREAM_RESTART_MAX_RETRIES}): {e}")
                if attempt < self.STREAM_RESTART_MAX_RETRIES - 1:
                    time.sleep(self.STREAM_RESTART_DELAY)
        
        rospy.logerr("音频流初始化失败，已达到最大重试次数")
        return False

    def restart_audio_stream(self):
        """重启音频流"""
        rospy.logwarn("尝试重启音频流...")
        return self.init_audio_stream()

    def check_sound_card(self):
        """
        检查声卡状态，特别是耳机和扬声器的可用性
        """
        # 检查耳机状态
        try:
            headphone_command = 'pactl list | grep -i Headphone'
            headphone_result = subprocess.run(headphone_command, shell=True, 
                                            capture_output=True, text=True)
            if headphone_result.stdout.strip():
                headphone_available = "not available" not in headphone_result.stdout
                if headphone_available:
                    return True
            
            # 检查扬声器状态
            speaker_command = 'pactl list | grep -i Speaker'
            speaker_result = subprocess.run(speaker_command, shell=True, 
                                          capture_output=True, text=True)
            if speaker_result.stdout.strip():
                return True
            
            # root用户下检查扬声器状态
            root_speaker_command = 'aplay -l | grep -i Audio'
            root_speaker_result = subprocess.run(root_speaker_command, shell=True, 
                                                capture_output=True, text=True)
            if root_speaker_result.stdout.strip():
                return True
            
            return False
        except Exception as e:
            print(f"检查声卡状态时出错: {str(e)}")
            return False
        
    def resample_audio(self, audio_chunk, source_sample_rate=None):
        """音频重采样"""
        if source_sample_rate is None:
            source_sample_rate = self.DEFAULT_SAMPLE_RATE
        try:
            audio_chunk = audio_chunk.astype(np.float32) / self.FLOAT32_DIVISOR
            resample_ratio = self.rate / source_sample_rate
            audio_chunk = samplerate.resample(audio_chunk, resample_ratio, 
                                            converter_type='sinc_fastest')
            audio_chunk = np.clip(audio_chunk * self.FLOAT32_DIVISOR, 
                                 self.INT16_MIN, self.INT16_MAX).astype(np.int16)
            return audio_chunk
        except Exception as e:
            rospy.logerr(f"音频重采样失败: {e}")
            return np.zeros(self.chunk_size, dtype=np.int16)

    def audio_callback(self, msg):
        """音频数据回调"""
        try:
            audio_chunk = np.array(msg.data, dtype=np.int16)
            
            source_sample_rate = self.DEFAULT_SAMPLE_RATE
            for dim in msg.layout.dim:
                if dim.label == "sample_rate" and dim.size > 0:
                    source_sample_rate = int(dim.size)
                    break
            
            audio_chunk = self.resample_audio(audio_chunk, source_sample_rate)
            self.buffer_queue.put(audio_chunk, timeout=self.QUEUE_PUT_TIMEOUT)
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
        """音频播放线程，带流恢复机制"""
        consecutive_errors = 0
        max_consecutive_errors = 5
        
        while self.playing and not rospy.is_shutdown():
            try:
                chunk = self.buffer_queue.get(timeout=self.QUEUE_GET_TIMEOUT)
                
                with self.stream_lock:
                    self.is_audio_playing = True
                    if self.stream is None or not self.stream.is_active():
                        rospy.logwarn("音频流未激活，尝试重启")
                        if not self.restart_audio_stream():
                            rospy.logerr("无法重启音频流，跳过此音频块")
                            continue
                    
                    try:
                        self.stream.write(chunk.tobytes(), 
                                        exception_on_underflow=False)
                        consecutive_errors = 0  # 成功后重置错误计数
                    except IOError as e:
                        if e.errno == pyaudio.paOutputUnderflowed:
                            rospy.logwarn("检测到 underflow，继续播放")
                        else:
                            raise
                        
            except queue.Empty:
                self.is_audio_playing = False
                if self.empty_count > self.EMPTY_COUNT_THRESHOLD:
                    rospy.logdebug("缓冲区为空，等待音频输入")
                    self.empty_count = 0
                self.empty_count += 1
                
            except Exception as e:
                consecutive_errors += 1
                rospy.logerr(f"播放缓冲区音频失败 ({consecutive_errors}/{max_consecutive_errors}): {e}")
                
                # 如果连续错误次数过多，尝试重启流
                if consecutive_errors >= max_consecutive_errors:
                    rospy.logerr("连续错误次数过多，尝试重启音频流")
                    if self.restart_audio_stream():
                        consecutive_errors = 0
                        # 清空缓冲区以避免播放旧数据
                        with self.buffer_queue.mutex:
                            self.buffer_queue.queue.clear()
                        rospy.loginfo("音频流已重启，缓冲区已清空")
                    else:
                        rospy.logerr("无法重启音频流，等待一段时间后重试")
                        time.sleep(2)
                        consecutive_errors = 0

    def stop_music_callback(self, msg):
        """停止当前正在播放的音频"""
        if msg.data:
            try:
                with self.buffer_queue.mutex:
                    self.buffer_queue.queue.clear()
                # 重置播放状态
                self.is_audio_playing = False
                rospy.loginfo("已停止当前音频播放并清空缓冲区")
                # 发布音频播放状态
                self.publish_audio_status()
                return True
            except Exception as e:
                rospy.logerr(f"停止音频播放时出错: {e}")
                return False

    def get_used_audio_buffer_size_callback(self, request):
        """获取缓冲区使用大小"""
        response = TriggerResponse()
        response.success = True
        used_size = self.buffer_queue.qsize()
        response.message = f"{used_size}"
        return response

    def status_timer_callback(self, event):
        """定时器回调函数，定期发布音频播放状态"""
        self.publish_audio_status()

    def publish_audio_status(self):
        """发布音频播放状态到topic"""
        try:
            msg = AudioPlaybackStatus()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            
            # 使用专门的播放状态变量
            if not hasattr(self, 'playing') or not self.playing:
                # 播放器已停止或未初始化
                msg.playing = False
                msg.message = "Player stopped"
            elif self.is_audio_playing:
                # 音频正在播放
                buffer_size = self.buffer_queue.qsize()
                msg.playing = True
                msg.message = f"Playing, buffer size: {buffer_size}"
            else:
                # 音频未在播放，检查缓冲区状态
                buffer_size = self.buffer_queue.qsize()
                if buffer_size > 0:
                    # 有数据但未播放，可能是刚刚停止播放
                    msg.playing = False
                    msg.message = f"Audio stopped, buffer size: {buffer_size}"
                else:
                    # 缓冲区为空，播放完成
                    msg.playing = False
                    msg.message = "Playback completed or no audio input"
            
            msg.buffer_size = self.buffer_queue.qsize()
            self.playback_status_publisher.publish(msg)
            
        except Exception as e:
            rospy.logerr(f"发布音频状态失败: {e}")


    def shutdown(self):
        """关闭节点时的清理工作"""
        self.playing = False
        if self.play_thread.is_alive():
            self.play_thread.join(timeout=self.THREAD_JOIN_TIMEOUT)
        
        with self.stream_lock:
            if self.stream:
                try:
                    self.stream.stop_stream()
                    self.stream.close()
                except:
                    pass
            
        if self.p:
            try:
                self.p.terminate()
            except:
                pass
            
        rospy.loginfo("播放已停止，音频资源已释放")

    def run(self):
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

if __name__ == '__main__':
    player_node = AudioStreamPlayerNode()
    player_node.run()
