import rospy
from kuavo_msgs.msg import AudioReceiverData
from queue import Full


class Microphone:
    """
    订阅音频话题并将接收到的数据放入一个外部队列中。
    """
    def __init__(self, audio_queue, subscribe_topic="/micphone_data"):
        self.audio_queue = audio_queue
        self.subscriber = rospy.Subscriber(subscribe_topic, AudioReceiverData, self._audio_callback)
        rospy.loginfo(f"麦克风节点订阅话题: {subscribe_topic}")

    def _audio_callback(self, msg: AudioReceiverData):
        """
        音频数据回调函数，将数据放入队列。
        """
        if self.audio_queue.full():
            rospy.logdebug("麦克风数据积满，丢弃音频帧！")
        else:
            self.audio_queue.put_nowait(msg.data)
