#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.kuavo.core.microphone import RobotMicrophoneCore

class RobotMicrophone: 
    def __init__(self):
        """Initialize the microphone system."""
        self.microphone = RobotMicrophoneCore()

    def wait_for_wake_word(self, timeout_sec: int = 60) -> bool:
        """Wait for the wake word to be detected.
        
        Args:
            timeout_sec (int): Timeout in seconds
                
        Returns:
            bool: True if the wake word was detected within the timeout, False otherwise
        """
        return self.microphone.wait_for_wake_word(timeout_sec=timeout_sec)
