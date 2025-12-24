#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.kuavo.core.audio import KuavoRobotAudioCore   

class KuavoRobotAudio:
    """Kuavo 机器人音频系统接口，用于控制音频播放功能。
    
    提供音乐文件播放功能。
    """
    
    def __init__(self):
        """初始化音频系统。"""
        self.audio = KuavoRobotAudioCore()

    def play_audio(self, file_name: str, volume: int = 100, speed: float = 1.0) -> bool:
        """播放指定的音频文件。
        
        Args:
            file_name (str): 要播放的音频文件名
                
        Returns:
            bool: 如果播放请求成功发送返回True，否则返回False
        """
        return self.audio.play_audio(file_name, volume, speed)
    
    def stop_music(self):
        """停止当前正在播放的音频。"""
        return self.audio.stop_music()
    
    def text_to_speech(self, text: str, volume: float = 0.5) -> bool:
        """将指定文本合成并播放。
        
        Args:
            text (str): 要播放的文本
                
        Returns:
            bool: 如果播放请求成功发送返回True，否则返回False
        """
        return self.audio.text_to_speech(text, volume)
