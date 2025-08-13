'''
Description: 音频文件播放节点，通过服务接口播放音频文件并发布到audio_data话题
'''
#!/usr/bin/env python3
import time
import rospy
import numpy as np
import os
import wave
import tempfile
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, MultiArrayLayout
from std_msgs.msg import Bool
from kuavo_msgs.srv import playmusic, playmusicResponse
import subprocess
import signal
import uuid

class MusicPlayerNode:
    # 音频配置,转换后的音频文件采样率
    DEFAULT_SAMPLE_RATE = 16000
    DEFAULT_CHANNELS = 1
    SAMPLE_WIDTH_BYTES = 2  # 16-bit
    
    # 播放控制
    PUBLISH_RATE_HZ = 10
    CHUNK_SIZE = 65536
    DEFAULT_GAIN = 1.0
    
    # FFmpeg参数
    FFMPEG_FORMAT = 'wav'
    FFMPEG_CODEC = 'pcm_s16le'
    
    def __init__(self):
        # 检查音频设备
        while not self.check_sound_card():
            print("未检测到播音设备，等待设备连接...")
            rospy.sleep(10000)
        
        # 检测get_used_audio_buffer_size服务是否启动
        while True:
            try:
                rospy.wait_for_service('/get_used_audio_buffer_size', timeout=5)
                rospy.loginfo("get_used_audio_buffer_size服务已启动")
                break
            except rospy.ROSException:
                print("未检测到get_used_audio_buffer_size服务，等待服务启动...")
                rospy.sleep(1)

        rospy.init_node('music_player_node')
        
        # 获取音乐目录路径
        self.music_directory = rospy.get_param('music_path', '/home/lab/.config/lejuconfig/music')
        
        # 创建服务和话题
        self.service = rospy.Service('play_music', playmusic, self.play_music_callback)
        self.audio_publisher = rospy.Publisher('audio_data', Int16MultiArray, queue_size=10)
        self.stop_music_subscriber = rospy.Subscriber('stop_music', Bool, self.stop_music_callback, queue_size=10)
        
        # 初始化变量
        self.temp_dir = tempfile.gettempdir()
        
        rospy.loginfo("音频播放节点初始化完成")

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
    def play_music_callback(self, req):
        """处理播放音乐请求"""
        try:
            music_file = os.path.join(self.music_directory, f"{req.music_number}")
            
            if not os.path.exists(music_file):
                rospy.logerr(f"音频文件不存在: {music_file}")
                return playmusicResponse(success_flag=False)
            
            rospy.loginfo(f"开始播放音频文件: {music_file}")
            
            # 转换为标准WAV格式
            wav_file_path = self.convert_to_wav(music_file, req.volume)
            if not wav_file_path:
                return playmusicResponse(success_flag=False)
            
            # 读取并发布音频数据
            success = self.publish_audio_data(wav_file_path)
            
            # 清理临时文件
            if wav_file_path != music_file:
                try:
                    os.remove(wav_file_path)
                except:
                    pass
            
            return playmusicResponse(success_flag=success)
            
        except Exception as e:
            rospy.logerr(f"播放音乐出错: {e}")
            return playmusicResponse(success_flag=False)

    def convert_to_wav(self, music_file, volume):
        """转换音频文件为标准WAV格式"""
        start_time = time.time()
        try:
            # 如果已经是WAV文件，检查格式是否符合要求
            if music_file.lower().endswith('.wav'):
                with wave.open(music_file, 'rb') as wf:
                    if (wf.getnchannels() == self.DEFAULT_CHANNELS and 
                        wf.getsampwidth() == self.SAMPLE_WIDTH_BYTES and 
                        wf.getframerate() == self.DEFAULT_SAMPLE_RATE):
                        return music_file  # 格式正确，直接使用
            
            # 需要转换，创建临时文件
            temp_wav = os.path.join(self.temp_dir, f"temp_audio_{uuid.uuid4()}.wav")
            
            ffmpeg_cmd = [
                'ffmpeg', '-i', music_file,
                '-f', self.FFMPEG_FORMAT,
                '-acodec', self.FFMPEG_CODEC,
                '-ar', str(self.DEFAULT_SAMPLE_RATE),
                '-ac', str(self.DEFAULT_CHANNELS),
                '-y', temp_wav
            ]
            
            result = subprocess.run(ffmpeg_cmd, capture_output=True, text=True)
            if result.returncode != 0:
                rospy.logerr(f"FFmpeg转换失败: {result.stderr}")
                return None
            
            end_time = time.time()
            rospy.loginfo(f"音频转换完成，耗时: {end_time - start_time}秒")
            return temp_wav
            
        except Exception as e:
            rospy.logerr(f"音频转换失败: {e}")
            return None

    def publish_audio_data(self, wav_file_path):
        """读取WAV文件并发布音频数据"""
        try:
            rate = rospy.Rate(self.PUBLISH_RATE_HZ)
            
            with wave.open(wav_file_path, 'rb') as wav_file:
                # 验证音频格式
                if wav_file.getsampwidth() != self.SAMPLE_WIDTH_BYTES:
                    rospy.logerr(f"不支持的采样宽度: {wav_file.getsampwidth()}")
                    return False
                
                # 读取所有音频数据
                audio_data = wav_file.readframes(wav_file.getnframes())
                audio_array = np.frombuffer(audio_data, dtype=np.int16)
                
                rospy.loginfo(f"音频信息: 长度={len(audio_array)}, 采样率={wav_file.getframerate()}")
                
                # 分块发布
                for i in range(0, len(audio_array), self.CHUNK_SIZE):
                    if rospy.is_shutdown():
                        break
                    
                    chunk = audio_array[i:i+self.CHUNK_SIZE]
                    
                    # 应用增益
                    chunk = (chunk * self.DEFAULT_GAIN).astype(np.int16)
                    
                    # 创建并发布消息
                    msg = Int16MultiArray()
                    msg.data = chunk.tolist()
                    
                    # 添加元数据，暂时不使用
                    # dim = MultiArrayDimension()
                    # dim.label = "audio_samples"
                    # dim.size = len(chunk)
                    # dim.stride = 1
                    # msg.layout = MultiArrayLayout(dim=[dim], data_offset=0)
                    
                    self.audio_publisher.publish(msg)
                    rate.sleep()
                
                rospy.loginfo("音频发布完成")
                return True
                
        except Exception as e:
            rospy.logerr(f"发布音频数据失败: {e}")
            return False

    def stop_music_callback(self, msg):
        """停止播放音乐"""
        if msg.data:
            rospy.loginfo("收到停止播放请求")
            # 这里可以添加停止逻辑，如清空发布队列等
            return True

    def run(self):
        """运行节点"""
        rospy.loginfo("音频播放节点开始运行")
        rospy.spin()

if __name__ == '__main__':
    try:
        player_node = MusicPlayerNode()
        player_node.run()
    except rospy.ROSInterruptException:
        pass
