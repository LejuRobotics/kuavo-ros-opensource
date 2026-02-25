#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import time

try:
    import pygame
    print("pygame 库已成功安装。")
except ImportError as e:
    print(f"pygame 库未安装或存在导入错误: {e}")

from std_msgs.msg import Int8  # 假设话题消息是整数类型


class ROSAudioPlayer:
    def __init__(self):
        # 1. 初始化pygame音频模块
        pygame.mixer.init()
        # 允许同时加载多个音频（非必须，但增强兼容性）
        pygame.mixer.music.set_volume(0.8)  # 设置默认音量（0.0-1.0）

        # 2. 从ROS参数服务器加载音频路径映射
        self.audio_map = self.load_audio_config()

        # 3. 初始化ROS节点并订阅话题
        rospy.init_node('alarm_player_node', anonymous=True)  # 节点名：audio_player_node
        # 订阅话题：/music_trigger，消息类型Int8，回调函数self.callback
        rospy.Subscriber("/music_trigger", Int8, self.callback)

        # 4. 添加播放状态标志
        self.is_playing = False

        rospy.loginfo("ROS音频播放器初始化完成，等待话题消息...")
        rospy.loginfo(f"加载音频映射配置: {self.audio_map}")

    def load_audio_config(self):
        """从ROS参数服务器加载音频配置"""
        audio_map = {}

        # 从参数服务器获取音频映射配置
        # 参数名格式：/audio_player/audio_0, /audio_player/audio_1, 等等
        param_prefix = rospy.get_name() + "/audio_"

        # 尝试从参数服务器读取配置
        for i in range(10):  # 假设最多10个音频文件
            param_name = f"{param_prefix}{i}"
            if rospy.has_param(param_name):
                audio_path = rospy.get_param(param_name)
                # 检查文件是否存在
                if os.path.isfile(audio_path):
                    audio_map[i] = audio_path
                    rospy.loginfo(f"加载音频配置: {i} -> {audio_path}")
                else:
                    rospy.logwarn(f"音频文件不存在: {audio_path}")
            else:
                # 如果没有找到更多参数，停止搜索
                if i == 0:
                    rospy.logwarn(f"未找到音频配置参数 {param_name}，使用默认配置")
                break

        # 如果没有从参数服务器加载到配置，使用默认配置
        if not audio_map:
            rospy.loginfo("使用默认音频配置")
            audio_map = {
                0: "/home/lab/lpf/kuavo-ros-control/src/humanoid-control/h12pro_controller_node/music/alarm.mp3",
                1: "/home/lab/lpf/kuavo-ros-control/src/humanoid-control/h12pro_controller_node/music/tray_fell_down.mp3",
                2: "/home/lab/lpf/kuavo-ros-control/src/humanoid-control/h12pro_controller_node/music/alarm.mp3",
                3: "/home/lab/lpf/kuavo-ros-control/src/humanoid-control/h12pro_controller_node/music/alarm.mp3"
            }

        return audio_map

    def play_single_audio(self, audio_path):
        """播放单个音频文件"""
        if not os.path.isfile(audio_path):
            rospy.logerr(f"音频文件不存在：{audio_path}")
            return False

        try:
            pygame.mixer.music.load(audio_path)
            pygame.mixer.music.play()

            # 等待播放完成
            while pygame.mixer.music.get_busy():
                time.sleep(0.1)

            return True
        except Exception as e:
            rospy.logerr(f"播放失败：{str(e)}")
            return False

    def play_audio_sequence(self, audio_paths):
        """按顺序播放多个音频文件"""
        for audio_path in audio_paths:
            if not os.path.isfile(audio_path):
                rospy.logerr(f"音频文件不存在：{audio_path}")
                continue

            try:
                pygame.mixer.music.load(audio_path)
                pygame.mixer.music.play()
                rospy.loginfo(f"正在播放：{os.path.basename(audio_path)}")

                # 等待当前音频播放完成
                while pygame.mixer.music.get_busy():
                    time.sleep(0.1)

            except Exception as e:
                rospy.logerr(f"播放失败：{str(e)}")

    def callback(self, msg):
        """话题消息回调函数：根据收到的数字播放对应音频组合"""
        trigger_num = msg.data  # 获取消息中的整数

        self.is_playing = True

        try:
            # 根据不同的消息数据播放不同的音频组合
            if trigger_num == 0:
                rospy.loginfo("收到指令 0，不播放音乐")
                self.is_playing = False
                pygame.mixer.music.stop()
                time.sleep(2)  # 短暂延迟确保停止

            elif trigger_num == 1:
                rospy.loginfo("收到指令 1，播放 audio_0 + audio_1")
                if 1 in self.audio_map:
                    self.play_audio_sequence([self.audio_map[1]])
                else:
                    rospy.logerr("无法播放：audio_0 或 audio_1 未配置")

            elif trigger_num == 2:
                rospy.loginfo("收到指令 2，播放 audio_0 + audio_2")
                if 2 in self.audio_map:
                    self.play_audio_sequence([self.audio_map[2]])
                else:
                    rospy.logerr("无法播放：audio_0 或 audio_2 未配置")

            elif trigger_num == 3:
                rospy.loginfo("收到指令 3，播放 audio_0 + audio_3")
                if 3 in self.audio_map:
                    self.play_audio_sequence([self.audio_map[3]])
                else:
                    rospy.logerr("无法播放：audio_0 或 audio_3 未配置")

            else:
                rospy.logwarn(f"无效指令：{trigger_num}，仅支持0-3")

        except Exception as e:
            rospy.logerr(f"播放过程中发生错误：{str(e)}")
        finally:
            self.is_playing = False

    def run(self):
        """保持节点运行"""
        rospy.spin()  # 阻塞等待话题消息


if __name__ == '__main__':
    try:
        # 创建播放器实例并运行
        player = ROSAudioPlayer()
        player.run()
    except rospy.ROSInterruptException:
        # 处理ROS中断（如Ctrl+C）
        rospy.loginfo("节点被中断，退出")
        pygame.mixer.music.stop()  # 停止播放
