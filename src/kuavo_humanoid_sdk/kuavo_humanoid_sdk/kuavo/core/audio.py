import time
import math
import threading
import numpy as np
from typing import Tuple
from transitions import Machine, State

from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.kuavo.core.ros.audio import Audio

class KuavoRobotAudioCore:
    def __init__(self):
        self.robot_audio = Audio()

    def play_audio(self, music_number: str, volume: int = 100, speed: float = 1.0) -> bool:
        """
        play music
        """
        return self.robot_audio.play_audio(music_number, volume, speed)

    def stop_music(self) -> bool:
        """
        stop music
        """
        return self.robot_audio.stop_audio()

    def text_to_speech(self, text: str, volume: float = 0.5) -> bool:
        """
        text to speech
        """
        return self.robot_audio.text_to_speech(text, volume)

