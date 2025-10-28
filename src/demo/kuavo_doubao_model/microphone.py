import os
import torch
import torch.nn as nn
import librosa
import numpy as np
import rospy
import rospkg
from lib.microphone import Microphone


class WakeWordModel(nn.Module):
    """
    Wake-up word model
    """
    def __init__(self, n_mfcc=40, max_pad_len=200):
        super(WakeWordModel, self).__init__()
        self.n_mfcc = n_mfcc
        self.max_pad_len = max_pad_len

        self.conv_layers = nn.Sequential(
            nn.Conv2d(1, 32, kernel_size=(3, 3), padding='same'),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=(3, 3), padding='same'),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=(2, 2)),
            nn.Dropout(0.25)
        )

        lstm_input_features = 64 * (self.n_mfcc // 2)

        self.lstm = nn.LSTM(
            input_size=lstm_input_features,
            hidden_size=64,
            num_layers=1,
            batch_first=True
        )

        self.dropout_lstm = nn.Dropout(0.5)
        self.fc1 = nn.Linear(64, 32)
        self.relu = nn.ReLU()
        self.output_layer = nn.Linear(32, 1)

    def forward(self, x):
        x = self.conv_layers(x)

        batch_size, num_channels, mfcc_dim, seq_len = x.shape
        x = x.permute(0, 3, 1, 2)
        x = x.reshape(batch_size, seq_len, num_channels * mfcc_dim)

        lstm_out, _ = self.lstm(x)

        out = lstm_out[:, -1, :]

        out = self.dropout_lstm(out)
        out = self.fc1(out)
        out = self.relu(out)
        out = self.output_layer(out)

        return out

class WakeWordDetector:
    """
    Provides an interface to detect the wake-up word "roban roban" using a trained model.
    """
    def __init__(self, model_path, n_mfcc=40, max_pad_len=200, target_sr=16000, threshold=0.6):
        self.device = torch.device("cpu")
        self.n_mfcc = n_mfcc
        self.max_pad_len = max_pad_len
        self.target_sr = target_sr
        self.threshold = threshold

        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model weight file not found")
        
        self.model = self._load_model(model_path)
        rospy.loginfo(f"WakeRobanDetector initialized. Using device: {self.device}")

    def _load_model(self, model_path):
        """Load the pre-trained WakeRobanModel model."""
        model = WakeWordModel(n_mfcc=self.n_mfcc, max_pad_len=self.max_pad_len).to(self.device)
        model.load_state_dict(torch.load(model_path, map_location=self.device, weights_only=True))
        model.eval()
        rospy.loginfo(f"Wake-up word model loaded successfully")
        return model
    
    def _preprocess_audio_for_model(self, audio_data_array, original_sample_rate):
        """
        Preprocesses the original audio array into MFCC features suitable for the model.
        Resampling and padding/truncation are performed.
        """
        # Ensure that audio_data_array is of type float32 for use by librosa
        audio_float = audio_data_array.astype(np.float32)

        # Resample if the sample rates are different
        if original_sample_rate != self.target_sr:
            processed_audio = librosa.resample(y=audio_float, orig_sr=original_sample_rate, target_sr=self.target_sr)
        else:
            processed_audio = audio_float

        # Calculate MFCC features
        # For 16kHz audio, hop_length=160 is usually used to get 10ms frames
        mfccs = librosa.feature.mfcc(y=processed_audio, sr=self.target_sr, n_mfcc=self.n_mfcc, hop_length=160)

        # Pad or truncate MFCC features to max_pad_len
        if mfccs.shape[1] > self.max_pad_len:
            mfccs = mfccs[:, :self.max_pad_len]
        else:
            pad_width = self.max_pad_len - mfccs.shape[1]
            mfccs = np.pad(mfccs, pad_width=((0, 0), (0, pad_width)), mode='constant')

        # Add batch and channel dimensions (batch_size, channels, n_mfcc, time_steps)
        mfccs_tensor = torch.tensor(mfccs, dtype=torch.float32).unsqueeze(0).unsqueeze(0)
        return mfccs_tensor
    
    def detect_wake_word(self, audio_data_array, original_sr):
        """
        Detect wake-up words in a given numpy array of audio data.
        Returns a tuple: (is_detected: bool, probability: float)
        """
        try:
            mfccs_tensor = self._preprocess_audio_for_model(
                audio_data_array,
                original_sample_rate=original_sr
            )
            with torch.no_grad(): # Disable gradient calculation during the inference phase
                output = self.model(mfccs_tensor.to(self.device))
                probability = torch.sigmoid(output).item() # The original output of the model is activated by sigmoid to get the probability
            
            is_detected = probability > self.threshold
            return is_detected, probability
        except Exception as e:
            rospy.logerr(f"An error occurred during the wake-up word detection process: {e}")
            return False, 0.0


class RobotMicrophoneCore:
    """
    The core logic for handling wake-up word detection using audio data provided by ROS nodes.
    """
    def __init__(self, subscribe_topic="/micphone_data"):
        self.microphone = Microphone(subscribe_topic)

        model_name = "wake_word.pth"
        model_path = os.path.dirname(os.path.abspath(__file__))
        self.model_full_path = f"{model_path}/models/{model_name}"

        # Audio parameters
        self.SAMPLE_RATE = 16000
        self.CHANNELS = 1
        self.BIT_RESOLUTION = 16
        self.BYTES_PER_SAMPLE = self.BIT_RESOLUTION // 8

        # Sliding window parameters
        self.WINDOW_DURATION_SEC = 2.0
        self.OVERLAP_STEP_SEC = 0.5

        self.WINDOW_SIZE_BYTES = int(self.WINDOW_DURATION_SEC * self.SAMPLE_RATE * self.BYTES_PER_SAMPLE * self.CHANNELS)
        self.OVERLAP_STEP_BYTES = int(self.OVERLAP_STEP_SEC * self.SAMPLE_RATE * self.BYTES_PER_SAMPLE * self.CHANNELS)

        if self.OVERLAP_STEP_BYTES <= 0:
            rospy.logwarn("The calculated overlap step is zero or negative, and defaults to half the window size.")
            self.OVERLAP_STEP_BYTES = self.WINDOW_SIZE_BYTES // 2
        elif self.OVERLAP_STEP_BYTES > self.WINDOW_SIZE_BYTES:
            rospy.logwarn("Overlap step is greater than window size. Set overlap to window size.")
            self.OVERLAP_STEP_BYTES = self.WINDOW_SIZE_BYTES

        rospy.loginfo(f"Processing window size: {self.WINDOW_DURATION_SEC} sec ({self.WINDOW_SIZE_BYTES} bytes)")
        rospy.loginfo(f"Overlap step: {self.OVERLAP_STEP_SEC} sec ({self.OVERLAP_STEP_BYTES} bytes)")

        self.processing_buffer = b''

        self.detector = WakeWordDetector(model_path=self.model_full_path, target_sr=self.SAMPLE_RATE)
        
        rospy.loginfo("The audio processor node is ready.")

    def wait_for_wake_word(self, timeout_sec=None):
        """
        Actively pull audio data, process it and wait for wake-up word detection.
        Returns True if a wake-up word is detected within the timeout period, otherwise returns False.
        """
        rospy.loginfo(f"Waiting for wake-up word to be detected for {timeout_sec if timeout_sec else 'infinite'} seconds...")
        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            if timeout_sec is not None:
                elapsed_time = rospy.get_time() - start_time
                if elapsed_time > timeout_sec:
                    rospy.loginfo("Timeout has been reached. No wake-up word was detected.")
                    return False

            new_data = self.microphone.get_data()
            if new_data:
                self.processing_buffer += new_data

            while len(self.processing_buffer) >= self.WINDOW_SIZE_BYTES:
                current_window_bytes = self.processing_buffer[:self.WINDOW_SIZE_BYTES]

                audio_array = np.frombuffer(current_window_bytes, dtype=np.int16)
                
                is_detected, probability = self.detector.detect_wake_word(audio_array, self.SAMPLE_RATE)
                
                if is_detected:
                    rospy.loginfo(f"Wake word 'roban' has been DETECTED! Probability: {probability:.4f}")
                    self.processing_buffer = b''
                    return True
                else:
                    rospy.loginfo(f"Wake word 'roban' not detected. Probability: {probability:.4f}")

                self.processing_buffer = self.processing_buffer[self.OVERLAP_STEP_BYTES:]

            rospy.sleep(0.1)

        return False
