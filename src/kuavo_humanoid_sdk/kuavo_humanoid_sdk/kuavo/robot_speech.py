#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.kuavo.core.llm_doubao import RobotLLMDoubaoCore

class RobotSpeech:
    """
    RobotSpeech is a wrapper class that provides an interface to interact with the Doubao speech system.
    """

    def __init__(self):
        """Initialize ``RobotSpeech`` wrapper and the underlying speech core."""
        self.doubao_speech_core = RobotLLMDoubaoCore()

    def establish_doubao_speech_connection(self, app_id: str, access_key: str) -> bool:
        """Establish a connection to the Doubao service."""
        return self.doubao_speech_core.verify_connection(app_id, access_key)

    def start_speech(self):
        """Start the speech system."""
        self.doubao_speech_core.start_speech_system()

    def stop_speech(self):
        """Stop the speech system."""
        self.doubao_speech_core.stop_speech_system()
