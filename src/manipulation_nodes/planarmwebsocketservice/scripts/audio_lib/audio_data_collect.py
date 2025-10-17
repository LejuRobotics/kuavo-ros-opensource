import rospy
import os
from kuavo_msgs.msg import AudioReceiverData
import numpy as np # 用于将bytes转换为可处理的数值数组
import wave
import datetime

dataset_class = "negative" # "positive" or "negative"

class AudioRecorderNode:
    """
    ROS subscriber node to record audio for a fixed duration upon a keypress trigger.
    """

    def __init__(self, node_name="audio_recorder_node", subscribe_topic="/kuavo/audio_data"):
        if not rospy.get_node_uri():
            rospy.init_node(node_name, anonymous=True)

        # --- Audio Parameters (must match the publisher) ---
        self.SAMPLE_RATE = 16000
        self.CHANNELS = 1
        self.BIT_RESOLUTION = 16
        self.BYTES_PER_SAMPLE = self.BIT_RESOLUTION // 8
        self.RECORD_DURATION_SEC = 2

        # --- Recording State ---
        self.is_recording = False
        self.recorded_data = bytearray()

        # --- ROS Subscriber Setup ---
        self.subscriber = rospy.Subscriber(subscribe_topic, AudioReceiverData, self._audio_callback)

        rospy.loginfo("Audio recorder node ready.")
        rospy.loginfo(f"Audio will be recorded at {self.SAMPLE_RATE} Hz, {self.BIT_RESOLUTION}-bit.")

    def _audio_callback(self, msg: AudioReceiverData):
        """
        Callback function for incoming audio data.
        If recording is active, it appends data to the buffer.
        """
        if self.is_recording:
            self.recorded_data.extend(msg.data)

    def _save_to_wav(self):
        """
        Saves the buffered audio data into a WAV file with a timestamp.
        """
        if not self.recorded_data:
            rospy.logwarn("No data was recorded. Is the audio topic publishing?")
            return

        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        directory = dataset_class
        if not os.path.exists(directory):
            os.makedirs(directory)
            rospy.loginfo(f"Created directory: {directory}")

        filename = f"{directory}/recording_{timestamp}.wav"

        rospy.loginfo(f"Saving {len(self.recorded_data)} bytes of audio to {filename}...")
        try:
            with wave.open(filename, 'wb') as wf:
                wf.setnchannels(self.CHANNELS)
                wf.setsampwidth(self.BYTES_PER_SAMPLE)
                wf.setframerate(self.SAMPLE_RATE)
                wf.writeframes(self.recorded_data)
            rospy.loginfo(f"Successfully saved to {filename}")
        except Exception as e:
            rospy.logerr(f"Failed to save WAV file: {e}")

    def run(self):
        """
        Waits for a user's Enter key press to trigger a recording, saves the clip,
        and then waits for the next trigger.
        """
        while not rospy.is_shutdown():
            try:
                # 直接使用 Python 3 的 input 函数
                prompt_input = input

                prompt_input(f"Press Enter to start a {self.RECORD_DURATION_SEC}s recording (or Ctrl+C to exit)...")
            except (EOFError, KeyboardInterrupt):
                rospy.loginfo("Exit signal received, shutting down.")
                break

            # Start recording
            rospy.loginfo("Recording started...")
            self.recorded_data = bytearray()  # Clear any previous data
            self.is_recording = True

            try:
                # The ROS sleep function allows callbacks to continue running in the background.
                # It will raise ROSInterruptException if Ctrl+C is pressed.
                rospy.sleep(self.RECORD_DURATION_SEC)
            except rospy.ROSInterruptException:
                rospy.loginfo("\nRecording interrupted by shutdown signal. Aborting save.")
                # The while loop condition will handle the exit.
                continue

            # Stop recording
            self.is_recording = False
            rospy.loginfo("Recording stopped.")

            # Save the collected data
            self._save_to_wav()

        rospy.loginfo("Audio recorder node has shut down.")

if __name__ == '__main__':
    try:
        recorder = AudioRecorderNode()
        recorder.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception in main: {e}")