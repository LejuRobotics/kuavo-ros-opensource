import rospy
import collections
import threading
from kuavo_humanoid_sdk.common.logger import SDKLogger

from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import AudioReceiverData


class Microphone:
    """
    ROS-specific part for handling microphone data subscription.
    It subscribes to an audio topic and buffers the received data.
    """
    def __init__(self, subscribe_topic="/micphone_data"):
        self._buffer = collections.deque()
        self._lock = threading.Lock()
        self.subscriber = rospy.Subscriber(subscribe_topic, AudioReceiverData, self._audio_callback)
        SDKLogger.debug(f"MicrophoneROSNode subscribed to topic: {subscribe_topic}")

    def _audio_callback(self, msg: AudioReceiverData):
        """
        Callback function for the audio subscriber. Appends data to the buffer.
        """
        with self._lock:
            self._buffer.append(msg.data)

    def get_data(self):
        """
        Retrieves all data chunks from the buffer and clears it.
        This is designed to be called by the processing layer.
        """
        with self._lock:
            if not self._buffer:
                return None
            
            data_batch = b''.join(self._buffer)
            self._buffer.clear()
            return data_batch
