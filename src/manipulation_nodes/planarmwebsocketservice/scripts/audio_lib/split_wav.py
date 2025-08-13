import wave
import os

def split_wav(input_file, output_dir, segment_duration_sec=2):
    """
    Splits a WAV file into smaller segments of a specified duration.

    Args:
        input_file (str): Path to the input WAV file.
        output_dir (str): Directory where the split files will be saved.
        segment_duration_sec (int): Duration of each segment in seconds.
    """
    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    with wave.open(input_file, 'rb') as wav:
        n_channels = wav.getnchannels()
        sample_width = wav.getsampwidth()
        frame_rate = wav.getframerate()
        n_frames = wav.getnframes()

        segment_frames = frame_rate * segment_duration_sec
        total_segments = n_frames // segment_frames

        for i in range(total_segments):
            output_file = os.path.join(output_dir, f"segment_{i+1}.wav")
            with wave.open(output_file, 'wb') as segment_wav:
                segment_wav.setnchannels(n_channels)
                segment_wav.setsampwidth(sample_width)
                segment_wav.setframerate(frame_rate)

                # Read frames for the current segment
                frames = wav.readframes(segment_frames)
                segment_wav.writeframes(frames)

            print(f"Segment {i+1} saved to {output_file}")

# Example usage
input_wav_file = "xxxxx.wav"
output_directory = "negative"
split_wav(input_wav_file, output_directory)
