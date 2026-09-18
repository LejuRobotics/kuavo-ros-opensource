#!/usr/bin/env python3
import rospy
import numpy as np
import sys
import os
import io
import wave
from std_msgs.msg import Int16MultiArray, MultiArrayDimension

audio_path = "/home/lab/.config/lejuconfig/music" # 直接从默认的音频文件目录中读取音频文件

class AudioStreamTestNode:
    # 音频配置常量
    SAMPLE_WIDTH_BYTES = 2          # 16-bit，即2字节
    
    # 发布控制常量
    PUBLISH_RATE_HZ = 10            # 发布频率(Hz)
    LARGE_CHUNK_SIZE = 999999       # 大块音频数据大小
    DEFAULT_GAIN = 3                # 默认音频增益倍数
    
    # 主题队列配置
    PUBLISHER_QUEUE_SIZE = 10       # 发布者队列大小
    
    # 默认音频文件
    DEFAULT_AUDIO_FILE = "1_挥手.wav"
    
    # 等待时间配置
    STARTUP_WAIT_TIME = 1.0         # 启动等待时间(秒)
    COMPLETION_WAIT_TIME = 5.0      # 完成等待时间(秒)
    
    def __init__(self):
        rospy.init_node('audio_stream_test_node')
        self.audio_pub = rospy.Publisher('audio_data', Int16MultiArray, 
                                       queue_size=self.PUBLISHER_QUEUE_SIZE)
        rospy.loginfo("音频流测试节点已启动")
        
        # 设置音频块大小与loundspeaker.py相同
        self.chunk_size = self.LARGE_CHUNK_SIZE
        # 设置发布速率
        self.rate = rospy.Rate(self.PUBLISH_RATE_HZ)  # 10Hz，可以根据需要调整
        
        # 音频文件路径
        # self.audio_file = os.path.join(
        #     os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        #     "assets", "1_挥手.wav"
        # )
        # 改为直接从默认的音频文件目录中读取音频文件
        self.audio_file = os.path.join(audio_path, self.DEFAULT_AUDIO_FILE)
        rospy.loginfo(f"音频文件路径: {self.audio_file}")
        
    def read_and_publish_audio(self):
        """读取音频文件并将数据发布到话题"""
        try:
            if not os.path.exists(self.audio_file):
                rospy.logerr(f"音频文件不存在: {self.audio_file}")
                return False
                
            rospy.loginfo(f"正在读取音频文件: {self.audio_file}")
            
            with wave.open(self.audio_file, 'rb') as wav_file:
                channels = wav_file.getnchannels()
                sample_width = wav_file.getsampwidth()
                frame_rate = wav_file.getframerate()
                n_frames = wav_file.getnframes()
                
                rospy.loginfo(f"WAV文件信息: 通道数={channels}, 采样宽度={sample_width}, "
                           f"采样率={frame_rate}, 总帧数={n_frames}")
                
                # 读取所有音频数据
                audio_data = wav_file.readframes(n_frames)
            
            if sample_width == self.SAMPLE_WIDTH_BYTES:  # 16-bit音频
                audio_array = np.frombuffer(audio_data, dtype=np.int16)
            else:
                rospy.logerr(f"不支持的采样宽度: {sample_width}")
                return False
            
            rospy.loginfo(f"开始发布音频数据，总长度: {len(audio_array)}")
            
            # 分块发布音频数据
            for i in range(0, len(audio_array), self.chunk_size):
                if rospy.is_shutdown():
                    break
                
                # 获取当前块
                chunk = audio_array[i:i+self.chunk_size]

                # 默认放大倍数
                chunk = chunk * self.DEFAULT_GAIN
                
                # 创建消息
                msg = Int16MultiArray()
                msg.data = chunk.tolist()
                
                # 添加元数据
                # msg.layout.dim.append(MultiArrayDimension())
                # msg.layout.dim[0].label = "audio_samples"
                # msg.layout.dim[0].size = len(chunk)
                # msg.layout.dim[0].stride = 1
                
                # 发布消息
                self.audio_pub.publish(msg)
                rospy.loginfo(f"发布音频块 {i//self.chunk_size + 1}/{len(audio_array)//self.chunk_size + 1}, 大小: {len(chunk)}")
                
                self.rate.sleep()
                
            rospy.loginfo("音频数据发布完成")
            return True
                
        except Exception as e:
            rospy.logerr(f"读取或发布音频数据时出错: {e}")
            return False

    def run(self):
        # 等待一段时间，确保订阅者已准备好
        rospy.sleep(self.STARTUP_WAIT_TIME)
        
        # 读取并发布音频数据
        self.read_and_publish_audio()
        
        # 保持节点运行一段时间，确保所有消息都被发送
        rospy.sleep(self.COMPLETION_WAIT_TIME)
        rospy.loginfo("测试完成")

def main():
    """主函数"""
    try:
        audio_stream_test = AudioStreamTestNode()
        audio_stream_test.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
