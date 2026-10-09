#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK
import roslibpy

class AudioWebsocket:
    """WebSocket-based audio system interface for controlling audio playback functionality of Kuavo humanoid robot.
    
    Provides functionality to play music files through WebSocket connection.
    """
    
    def __init__(self):
        """Initialize the WebSocket audio system."""
        websocket = WebSocketKuavoSDK()
        self._audio_stop_publisher = roslibpy.Topic(websocket.client, 'stop_music', 'std_msgs/Bool')
        self._audio_stop_publisher.advertise()
        self._audio_data_publisher = roslibpy.Topic(websocket.client, 'audio_data', 'std_msgs/Int16MultiArray')
        self._audio_data_publisher.advertise()

    def play_audio(self, file_name: str, volume: int = 100, speed: float = 1.0) -> bool:
        """Play the specified audio file through WebSocket.
        
        Args:
            file_name (str): Name of the audio file to play
            volume (float): Volume level (0.0 to 1.0)
            speed (float): Playback speed
                
        Returns:
            bool: True if the play request was successfully sent, False otherwise
        """
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, 'play_music', 'kuavo_msgs/playmusic')
            
            volume = min(max(volume, 0), 100)
            request = {
                "music_number": file_name,
                "volume": volume
            }
            
            response = service.call(request)
            SDKLogger.info(f"[Robot Audio] Requested to play audio file: {file_name}")
            return True
        except Exception as e:
            SDKLogger.error(f"[Robot Audio] Failed to play audio file: {str(e)}")
            return False
    
    def stop_audio(self) -> bool:
        """Stop the currently playing audio through WebSocket."""
        try:
            msg = {
                "data": True
            }
            self._audio_stop_publisher.publish(roslibpy.Message(msg))
            SDKLogger.info("[Robot Audio] Requested to stop audio playback")
            return True
        except Exception as e:
            SDKLogger.error(f"[Robot Audio] Failed to stop audio playback: {str(e)}")
            return False
    
    def text_to_speech(self, text: str, volume: float = 0.5) -> bool:
        """Synthesize and play the specified text through WebSocket.
        
        Args:
            text (str): Text to be played
            volume (float): Volume level (0.0 to 1.0)
                
        Returns:
            bool: True if the play request was successfully sent, False otherwise
        """
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, 'speech_synthesis', 'kuavo_msgs/SpeechSynthesis')
            
            request = {
                "data": text,
                "volume": volume
            }
            
            response = service.call(request)
            SDKLogger.info(f"[Robot Audio] Requested to play audio text: {text}")
            return True
        except Exception as e:
            SDKLogger.error(f"[Robot Audio] Failed to play audio text: {str(e)}")
            return False

    def publish_audio_chunk(self, audio_chunk, gain: int = 1) -> bool:
        """Publish a single audio chunk for real-time audio streaming.

        Args:
            audio_chunk: List of int audio samples
            gain: Amplification gain factor (default 1)

        Returns:
            bool: True if the publish was successful, False otherwise
        """
        try:
            if not audio_chunk:
                return False
            amplified_chunk = [int(sample * gain) for sample in audio_chunk]
            msg = {"data": amplified_chunk}
            self._audio_data_publisher.publish(roslibpy.Message(msg))
            return True
        except Exception as e:
            SDKLogger.error(f"[Robot Audio] Failed to publish audio chunk: {e}")
            return False


