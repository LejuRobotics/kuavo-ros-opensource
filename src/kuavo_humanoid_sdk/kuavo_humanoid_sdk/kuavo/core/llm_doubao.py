import numpy as np
import rospy
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.kuavo.core.ros.microphone import Microphone
from kuavo_humanoid_sdk.kuavo.core.ros.audio import Audio
from kuavo_humanoid_sdk.kuavo.core.llm_doubao_lib import *
from kuavo_humanoid_sdk.kuavo.core.llm_doubao_lib.audio_manager import DialogSession
from kuavo_humanoid_sdk.kuavo.core.llm_doubao_lib.realtime_dialog_client import RealtimeDialogClient
from kuavo_humanoid_sdk.kuavo.core.llm_doubao_lib import config
import os
import asyncio
import threading
import queue
import time
import struct
import uuid
from typing import Optional, Dict, Any


class ROSDialogSession(DialogSession):
    """Custom DialogSession that integrates with ROS audio system"""
    
    def __init__(self, ws_config: Dict[str, Any], audio_interface: Audio, enable_signal_handler: bool = False):
        # Initialize session ID and client without calling parent __init__
        self.session_id = str(uuid.uuid4())
        self.client = RealtimeDialogClient(config=ws_config, session_id=self.session_id)
        
        # Store reference to audio interface for ROS audio publishing
        self.audio_interface = audio_interface
        
        # Initialize session state variables
        self.is_running = True
        self.is_session_finished = False
        self.is_user_querying = False
        self.is_sending_chat_tts_text = False
        self.audio_buffer = b''
        
        # Audio chunk buffer for handling large audio chunks
        self.audio_chunk_buffer = []
        self.buffer_lock = threading.Lock()
        
        # Skip PyAudio initialization completely
        self.audio_device = None
        self.audio_queue = None
        self.output_stream = None
        self.input_stream = None
        self.player_thread = None
        self.is_recording = True
        self.is_playing = False  # We don't use PyAudio playing
        
        # Start audio chunk processing thread
        self.chunk_processor_thread = threading.Thread(target=self._process_audio_chunks)
        self.chunk_processor_thread.daemon = True
        self.chunk_processor_thread.start()
        
        # Only set signal handler if requested and in main thread
        if enable_signal_handler:
            try:
                import signal
                signal.signal(signal.SIGINT, self._keyboard_signal)
            except ValueError as e:
                SDKLogger.warning(f"Warning: Cannot set signal handler (not in main thread): {e}")

    def cleanup(self):
        """Clean up resources (no PyAudio to clean up)"""
        self.is_running = False
        self.is_recording = False
        self.is_playing = False
        
        # Clear audio buffer
        with self.buffer_lock:
            self.audio_chunk_buffer.clear()
        
        # Wait for chunk processor thread to finish
        if hasattr(self, 'chunk_processor_thread') and self.chunk_processor_thread.is_alive():
            self.chunk_processor_thread.join(timeout=2)
        
        # No audio device cleanup needed since we're using ROS

    def _keyboard_signal(self, sig, frame):
        """Handle keyboard interrupt signal"""
        SDKLogger.info(f"receive keyboard Ctrl+C")
        self.is_recording = False
        self.is_playing = False
        self.is_running = False

    def _process_audio_chunks(self):
        """Process buffered audio chunks in a separate thread"""
        while self.is_running:
            try:
                with self.buffer_lock:
                    if self.audio_chunk_buffer:
                        chunk = self.audio_chunk_buffer.pop(0)
                    else:
                        chunk = None
                
                if chunk:
                    self._publish_audio_chunk_to_ros(chunk)
                    # Small delay to prevent overwhelming the ROS system
                    time.sleep(0.01)
                else:
                    # No chunks to process, wait a bit
                    time.sleep(0.05)
                    
            except Exception as e:
                SDKLogger.error(f"[Speech] Error processing audio chunks: {e}")
                time.sleep(0.1)

    def _add_audio_chunk_to_buffer(self, audio_chunk):
        """Add audio chunk to buffer for processing"""
        with self.buffer_lock:
            self.audio_chunk_buffer.append(audio_chunk)
            # Limit buffer size to prevent memory issues
            if len(self.audio_chunk_buffer) > 50:
                SDKLogger.warn(f"[Speech] Audio buffer full, dropping oldest chunk")
                self.audio_chunk_buffer.pop(0)

    def _convert_audio_bytes_to_int_list(self, audio_bytes: bytes):
        """Convert audio bytes (PCM) to list of integers for ROS audio playback"""
        try:
            # SDKLogger.debug(f"[Speech] Converting audio bytes: length={len(audio_bytes)}")
            
            if len(audio_bytes) < 4:  # Float32 needs at least 4 bytes
                SDKLogger.warn(f"[Speech] Audio data too short: {len(audio_bytes)} bytes")
                return []
            
            # Try to detect audio format and convert accordingly
            audio_ints = self._convert_audio_with_format_detection(audio_bytes)
            
            if audio_ints:
                # Resample from 24kHz to 16kHz for ROS compatibility
                audio_ints = self._resample_audio(audio_ints, 24000, 16000)
                
                # Split large audio chunks into smaller ones for better ROS compatibility
                chunk_size = 8192
                if len(audio_ints) > chunk_size:
                    SDKLogger.debug(f"[Speech] Splitting large audio chunk ({len(audio_ints)} samples) into {len(audio_ints) // chunk_size + 1} smaller chunks")
                    # Split into multiple chunks and add to buffer
                    for i in range(0, len(audio_ints), chunk_size):
                        chunk = audio_ints[i:i + chunk_size]
                        if len(chunk) > 0:
                            self._add_audio_chunk_to_buffer(chunk)
                    return []  # Return empty since we've buffered the chunks
                else:
                    return audio_ints
            else:
                SDKLogger.warn(f"[Speech] No audio samples extracted from {len(audio_bytes)} bytes")
                return []
            
        except Exception as e:
            SDKLogger.error(f"[Speech] Error converting audio bytes to int list: {e}")
            return []

    def _convert_audio_with_format_detection(self, audio_bytes: bytes):
        """Convert audio bytes with automatic format detection"""
        import struct
        import numpy as np
        
        # Try Float32 format first (24kHz server format)
        try:
            if len(audio_bytes) % 4 == 0:  # Float32 should be divisible by 4
                float_samples = []
                for i in range(0, len(audio_bytes), 4):
                    if i + 3 < len(audio_bytes):
                        # Convert 4 bytes to float32 (little-endian)
                        sample_float = struct.unpack('<f', audio_bytes[i:i+4])[0]
                        float_samples.append(sample_float)
                
                if float_samples:
                    # Analyze float32 data range
                    min_float = min(float_samples)
                    max_float = max(float_samples)
                    abs_max = max(abs(min_float), abs(max_float))
                    
                    # SDKLogger.debug(f"[Speech] Float32 analysis: count={len(float_samples)}, min={min_float:.6f}, max={max_float:.6f}, abs_max={abs_max:.6f}")
                    
                    # Auto-detect gain based on actual float range
                    if abs_max > 0.001:  # Avoid division by zero
                        # Calculate gain to use full int16 range
                        # Leave some headroom (use 0.9 instead of 1.0)
                        target_range = 32767 * 0.9
                        auto_gain = target_range / abs_max
                        
                        # SDKLogger.debug(f"[Speech] Auto-detected gain: {auto_gain:.2f}")
                        
                        # Apply gain and convert to int16
                        samples = []
                        for sample_float in float_samples:
                            sample_int = int(sample_float * auto_gain)
                            sample_int = max(-32768, min(32767, sample_int))  # Clamp
                            samples.append(sample_int)
                        
                        # Check final result
                        min_val = min(samples)
                        max_val = max(samples)
                        variation = max_val - min_val
                        
                        # SDKLogger.debug(f"[Speech] Float32 conversion result: count={len(samples)}, variation={variation}")
                        
                        if variation > 100:  # Good variation suggests valid conversion
                            # SDKLogger.debug(f"[Speech] Using Float32 format (24kHz) with auto-gain {auto_gain:.2f}")
                            return samples
                    else:
                        SDKLogger.warn(f"[Speech] Float32 data range too small (abs_max={abs_max:.6f})")
        except Exception as e:
            SDKLogger.error(f"[Speech] Float32 conversion failed: {e}")
        
        # Try Int16 format (fallback)
        try:
            if len(audio_bytes) % 2 == 0:  # Int16 should be divisible by 2
                samples = []
                for i in range(0, len(audio_bytes), 2):
                    if i + 1 < len(audio_bytes):
                        # Convert 2 bytes to signed 16-bit integer (little-endian)
                        sample = struct.unpack('<h', audio_bytes[i:i+2])[0]
                        samples.append(sample)
                
                if samples:
                    min_val = min(samples)
                    max_val = max(samples)
                    variation = max_val - min_val
                    
                    SDKLogger.debug(f"[Speech] Int16 conversion: count={len(samples)}, variation={variation}")
                    SDKLogger.debug(f"[Speech] Using Int16 format")
                    return samples
        except Exception as e:
            SDKLogger.error(f"[Speech] Int16 conversion failed: {e}")
        
        SDKLogger.error(f"[Speech] Failed to convert audio data with any format")
        return []

    def _resample_audio(self, audio_samples, from_rate, to_rate):
        """Resample audio from one sample rate to another"""
        if from_rate == to_rate:
            return audio_samples
        
        try:
            import numpy as np
            from scipy import signal
            
            # Convert to numpy array
            audio_array = np.array(audio_samples, dtype=np.float32)
            
            # Calculate resampling ratio
            resample_ratio = to_rate / from_rate
            
            # Resample using scipy
            resampled_length = int(len(audio_array) * resample_ratio)
            resampled_audio = signal.resample(audio_array, resampled_length)
            
            # Convert back to int16 and clamp
            resampled_int = np.clip(resampled_audio, -32768, 32767).astype(np.int16)
            
            # SDKLogger.debug(f"[Speech] Resampled audio from {from_rate}Hz to {to_rate}Hz: {len(audio_samples)} -> {len(resampled_int)} samples")
            
            return resampled_int.tolist()
            
        except ImportError:
            SDKLogger.warn(f"[Speech] scipy not available, using simple decimation for resampling")
            # Simple decimation fallback
            if from_rate > to_rate:
                step = int(from_rate // to_rate)
                return audio_samples[::step]
            else:
                return audio_samples
        except Exception as e:
            SDKLogger.error(f"[Speech] Error resampling audio: {e}")
            return audio_samples

    def _publish_audio_chunk_to_ros(self, audio_int_list, gain: int = 1):
        """Publish single audio chunk directly to ROS topic using Audio interface"""
        try:
            if not audio_int_list:
                return
                
            # Use the new publish_audio_chunk method from Audio class
            success = self.audio_interface.publish_audio_chunk(audio_int_list, gain=gain)
            
            if not success:
                SDKLogger.warn(f"[Speech] Failed to publish audio chunk with {len(audio_int_list)} samples")
            
        except Exception as e:
            SDKLogger.error(f"[Speech] Error publishing audio to ROS: {e}")

    def handle_server_response(self, response: Dict[str, Any]) -> None:
        """Override to handle audio playback through ROS instead of PyAudio"""
        if response == {}:
            return
            
        # Handle audio data from server
        if response['message_type'] == 'SERVER_ACK' and isinstance(response.get('payload_msg'), bytes):
            if self.is_sending_chat_tts_text:
                return
                
            audio_data = response['payload_msg']
            self.audio_buffer += audio_data
            
            # SDKLogger.debug(f"[Speech] Received audio chunk: {len(audio_data)} bytes")
            
            # Play audio through ROS audio system instead of PyAudio
            try:
                audio_int_list = self._convert_audio_bytes_to_int_list(audio_data)
                if audio_int_list:
                    # For smaller chunks, publish immediately
                    self._publish_audio_chunk_to_ros(audio_int_list)
                # For larger chunks, they are automatically buffered in _convert_audio_bytes_to_int_list
            except Exception as e:
                SDKLogger.error(f"[Speech] Error playing server audio through ROS: {e}")
                
        elif response['message_type'] == 'SERVER_FULL_RESPONSE':
            # SDKLogger.info(f"服务器响应: {response}")
            event = response.get('event')
            payload_msg = response.get('payload_msg', {})

            # Log ASR results (user speech recognition)
            if event == 451:
                # Extract user speech text from ASR results
                results = payload_msg.get('results', [])
                if results and len(results) > 0:
                    result = results[0]
                    text = result.get('text', '')
                    is_interim = result.get('is_interim', True)
                    
                    # Only log final results (not interim)
                    if not is_interim and text:
                        SDKLogger.info(f"[Speech] 用户说话: {text}")

            # Log TTS streaming text (AI response)
            elif event == 550:
                content = payload_msg.get('content', '')
                if content:
                    # Use info level for visible logging, accumulate content for complete response
                    if not hasattr(self, '_current_ai_response'):
                        self._current_ai_response = ""
                    self._current_ai_response += content
                    # SDKLogger.info(f"[Speech] AI回复: {content}")

            if event == 450:
                SDKLogger.info(f"清空缓存音频: {response['session_id']}")
                # Clear the audio buffer
                with self.buffer_lock:
                    self.audio_chunk_buffer.clear()
                self.is_user_querying = True

            if event == 350 and self.is_sending_chat_tts_text and payload_msg.get("tts_type") == "chat_tts_text":
                # Clear the audio buffer
                with self.buffer_lock:
                    self.audio_chunk_buffer.clear()
                self.is_sending_chat_tts_text = False

            if event == 459:
                self.is_user_querying = False
                
            # Log complete AI response when TTS ends
            if event == 351:
                # TTS synthesis completed
                if hasattr(self, '_current_ai_response') and self._current_ai_response:
                    SDKLogger.info(f"[Speech] AI完整回复: {self._current_ai_response}")
                    self._current_ai_response = ""  # Reset for next response
                
        elif response['message_type'] == 'SERVER_ERROR':
            SDKLogger.error(f"服务器错误: {response['payload_msg']}")
            raise Exception("服务器错误")

    async def receive_loop(self):
        """接收服务器响应的循环"""
        try:
            while True:
                response = await self.client.receive_server_response()
                self.handle_server_response(response)
                if 'event' in response and (response['event'] == 152 or response['event'] == 153):
                    SDKLogger.info(f"receive session finished event: {response['event']}")
                    self.is_session_finished = True
                    break
        except asyncio.CancelledError:
            SDKLogger.info("接收任务已取消")
        except Exception as e:
            SDKLogger.error(f"接收消息错误: {e}")


class RobotLLMDoubaoCore:

    def __init__(self, subscribe_topic: str = "/micphone_data"):
        # Microphone interface
        self.microphone = Microphone(subscribe_topic)
        
        # ROS Audio interface for direct topic publishing
        self.ros_audio = Audio()

        # Audio parameters
        self.SAMPLE_RATE = 16000
        self.CHANNELS = 1
        self.BIT_RESOLUTION = 16
        self.BYTES_PER_SAMPLE = self.BIT_RESOLUTION // 8
        
        # Dialog session management
        self.dialog_session: Optional[ROSDialogSession] = None
        self.is_running = False
        self.event_loop = None
        self.session_thread = None
        self.ws_config = None
        
        # Audio queue for ROS microphone data
        self.audio_queue = queue.Queue()
        
        SDKLogger.info("[Speech] RobotLLMDoubaoCore initialized")

    def _setup_websocket_config(self, app_id: str, access_key: str):
        """Setup WebSocket configuration with provided credentials"""
        self.ws_config = {
            "base_url": "wss://openspeech.bytedance.com/api/v3/realtime/dialogue",
            "headers": {
                "X-Api-App-ID": app_id,
                "X-Api-Access-Key": access_key,
                "X-Api-Resource-Id": "volc.speech.dialog",
                "X-Api-App-Key": "PlgvMymc7f3tQnJ6",
                "X-Api-Connect-Id": config.ws_connect_config["headers"]["X-Api-Connect-Id"],
            }
        }

    def _run_async_session(self):
        """Run dialog session in separate thread with its own event loop"""
        self.event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.event_loop)
        
        try:
            self.event_loop.run_until_complete(self._async_session_main())
        except Exception as e:
            SDKLogger.error(f"[Speech] Dialog session error: {e}")
        finally:
            self.event_loop.close()

    async def _async_session_main(self):
        """Main async session handler"""       
        try:
            # Establish WebSocket connection first (reconnect after test)
            connection_success = await self.dialog_session.client.start_connection()
            if not connection_success:
                SDKLogger.error("[Speech] Failed to establish WebSocket connection in session")
                return

            await self.dialog_session.client.start_session()
            SDKLogger.info("[Speech] Speech session started successfully")
            
            # Start receiving responses
            receive_task = asyncio.create_task(self.dialog_session.receive_loop())
            
            # Start processing ROS microphone data
            audio_task = asyncio.create_task(self._process_ros_microphone_data())
            
            # Send hello message
            await self.dialog_session.client.say_hello()
            
            # Wait for session to finish
            while self.is_running and not self.dialog_session.is_session_finished:
                await asyncio.sleep(0.1)
                
            # Clean up tasks
            receive_task.cancel()
            audio_task.cancel()
            
            # Finish session
            await self.dialog_session.client.finish_session()
            while not self.dialog_session.is_session_finished:
                await asyncio.sleep(0.1)
            await self.dialog_session.client.finish_connection()
            await self.dialog_session.client.close()
            
            SDKLogger.info(f"[Speech] Dialog session ended, logid: {self.dialog_session.client.logid}")
            
        except Exception as e:
            SDKLogger.error(f"[Speech] Session error: {e}")
        finally:
            if self.dialog_session:
                self.dialog_session.cleanup()

    async def _process_ros_microphone_data(self):
        """Process microphone data from ROS topic"""
        SDKLogger.info("[Speech] Starting ROS microphone data processing")
        
        while self.is_running:
            try:
                # Get audio data from ROS microphone
                audio_data = self.microphone.get_data()
                
                if audio_data is not None and len(audio_data) > 0:
                    # Convert numpy array to bytes if needed
                    if isinstance(audio_data, np.ndarray):
                        audio_bytes = audio_data.tobytes()
                    else:
                        audio_bytes = audio_data
                    
                    # Send audio data to dialog service
                    await self.dialog_session.client.task_request(audio_bytes)
                    
                await asyncio.sleep(0.01)  # Small delay to prevent CPU overload
                
            except Exception as e:
                SDKLogger.warn(f"[Speech] Error processing ROS microphone data: {e}")
                await asyncio.sleep(0.1)

    def verify_connection(self, app_id: str, access_key: str) -> bool:
        """Set the app ID and access key for the speech system."""
        if not app_id or not access_key:
            SDKLogger.error("[Speech] App ID and Access Key are required")
            return False

        # Setup WebSocket configuration
        self._setup_websocket_config(app_id, access_key)
        # Use custom ROS-integrated DialogSession with Audio interface
        self.dialog_session = ROSDialogSession(self.ws_config, self.ros_audio, enable_signal_handler=False)

        # Test connection using event loop
        try:
            import asyncio
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            try:
                connection_successful = loop.run_until_complete(self.dialog_session.client.start_connection())
                if connection_successful:
                    # Close the test connection since we'll reconnect in _async_session_main
                    loop.run_until_complete(self.dialog_session.client.close())
                    SDKLogger.info("[Speech] WebSocket connected successfully")
                    return True
                else:
                    SDKLogger.error("[Speech] WebSocket connection failed")
                    self.dialog_session = None  # Clear failed session
                    return False
            finally:
                loop.close()
                
        except Exception as e:
            SDKLogger.error(f"[Speech] Failed to test WebSocket connection: {e}")
            self.dialog_session = None  # Clear failed session
            return False

    def start_speech_system(self):
        """Start the speech dialog system with Doubao service."""
        if self.is_running:
            SDKLogger.warn("[Speech] Speech system is already running")
            
        if self.dialog_session is None:
            SDKLogger.error("[Speech] Dialog session not initialized. Please call verify_connection() first with valid credentials.")
            
        try:
            SDKLogger.info(f"[Speech] Starting speech system")
            
            # Start dialog session in separate thread
            self.is_running = True
            self.session_thread = threading.Thread(target=self._run_async_session)
            self.session_thread.daemon = True
            self.session_thread.start()
            
            # Wait a bit for connection to establish
            time.sleep(2)
            
            SDKLogger.info("[Speech] Speech system started successfully")
            
        except Exception as e:
            SDKLogger.error(f"[Speech] Failed to start speech system: {e}")
            self.is_running = False

    def stop_speech_system(self):
        """Stop the Doubao speech system."""
        if not self.is_running:
            SDKLogger.warn("[Speech] Speech system is not running")
            
        try:
            SDKLogger.info("[Speech] Stopping speech system")
            
            # Signal to stop
            self.is_running = False
            
            # Stop dialog session
            if self.dialog_session:
                self.dialog_session.is_running = False
                self.dialog_session.is_recording = False
                self.dialog_session.is_playing = False
            
            # Wait for session thread to finish
            if self.session_thread and self.session_thread.is_alive():
                self.session_thread.join(timeout=5)
                
            SDKLogger.info("[Speech] Speech system stopped successfully")
            
        except Exception as e:
            SDKLogger.error(f"[Speech] Failed to stop speech system: {e}")

    def is_system_running(self) -> bool:
        """Check if the speech system is currently running."""
        return self.is_running

    def get_session_status(self) -> dict:
        """Get current session status information."""
        status = {
            "is_running": self.is_running,
            "has_session": self.dialog_session is not None,
            "session_finished": False,
            "logid": ""
        }
        
        if self.dialog_session:
            status["session_finished"] = self.dialog_session.is_session_finished
            if self.dialog_session.client:
                status["logid"] = self.dialog_session.client.logid
                
        return status