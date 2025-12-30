#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore

try:    
    import rospy
    from std_msgs.msg import Bool
    from kuavo_humanoid_sdk.msg.kuavo_msgs.srv import  playmusic, playmusicRequest
    from kuavo_humanoid_sdk.msg.kuavo_msgs.srv import  SpeechSynthesis, SpeechSynthesisRequest
except:
    pass

class Audio:
    """Audio system interface for controlling audio playback functionality of Kuavo humanoid robot.
    
    Provides functionality to play music files.
    """
    
    def __init__(self):
        """Initialize the audio system."""
        self._audio_stop_publisher = rospy.Publisher('stop_music', Bool, queue_size=10)
        rospy.sleep(0.5)  # Wait for publisher initialization
    def play_audio(self, file_name: str, volume: int = 100, speed: float = 1.0) -> bool:
        """Play the specified audio file.
        
        Args:
            file_name (str): Name of the audio file to play
                
        Returns:
            bool: True if the play request was successfully sent, False otherwise
        """
        try:
            # Wait for service availability
            rospy.wait_for_service('play_music', timeout=2.0)
            # Create service client
            play_music_service = rospy.ServiceProxy('play_music', playmusic)
            # Call service
            request = playmusicRequest()
            request.music_number = file_name
            volume = min(max(volume , 0), 100)
            request.volume = volume
            response = play_music_service(request)
            SDKLogger.info(f"[Robot Audio] Requested to play audio file: {file_name}")
            return True
        except rospy.ROSException as e:
            SDKLogger.error(f"[Robot Audio] Audio playback service unavailable: {str(e)}")
            return False
        except Exception as e:
            SDKLogger.error(f"[Robot Audio] Failed to play audio file: {str(e)}")
            return False
    
    def stop_audio(self):
        """Stop the currently playing audio."""
        try:
            msg = Bool()
            msg.data = True
            self._audio_stop_publisher.publish(msg)
            SDKLogger.info("[Robot Audio] Requested to stop audio playback")
            return True 
        except Exception as e:
            SDKLogger.error(f"[Robot Audio] Failed to stop audio playback: {str(e)}")
            return False    
    
    def text_to_speech(self, text: str,volume: float = 0.5) -> bool:
        """Synthesize and play the specified text.
        
        Args:
            text (str): Text to be played
                
        Returns:
            bool: True if the play request was successfully sent, False otherwise
        """
        try:
            # Wait for service availability
            rospy.wait_for_service('play_music', timeout=2.0)
            # Create service client
            play_music_service = rospy.ServiceProxy('speech_synthesis', SpeechSynthesis)
            # Call service
            request = SpeechSynthesisRequest()
            request.data = text
            request.volume = volume
            response = play_music_service(request)
            SDKLogger.info(f"[Robot Audio] Requested to play audio text: {text}")
            return True
        except rospy.ROSException as e:
            SDKLogger.error(f"[Robot Audio] Audio playback service unavailable: {str(e)}")
            return False
        except Exception as e:
            SDKLogger.error(f"[Robot Audio] Failed to play audio text: {str(e)}")
            return False

