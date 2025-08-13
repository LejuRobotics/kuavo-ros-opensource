"""
Pico VR device integration core functionality.
"""

import sys
import socket
import queue
import threading
import argparse
import signal
import time
from enum import Enum
import numpy as np
import tf
from core.ros.pico import (
    KuavoPicoNodeManager,
    KuavoPicoNode,
    RobotMatrixPublisher
)
from core.ros.pico_utils import RobotInfoBroadcaster
from core.ros.robot_data import RobotDataServer
from common.logger import SDKLogger

def signal_handler(self):
    """Handle signal for graceful exit."""
    SDKLogger.info('Exiting gracefully...')
    if self.socket:
        self.socket.close()
    sys.exit(0)

def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--record_bag', type=bool, default=False, help='Record bag file')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='UDP server host')
    parser.add_argument('--port', type=int, default=12345, help='UDP server port')
    return parser.parse_args()

class PicoError(Exception):
    """Base exception class for Pico related errors."""
    pass

class ConnectionError(PicoError):
    """Exception raised for connection related errors."""
    pass

class DataProcessingError(PicoError):
    """Exception raised for data processing related errors."""
    pass

class ConfigurationError(PicoError):
    """Exception raised for configuration related errors."""
    pass

class ErrorState(Enum):
    """Enum for tracking error states."""
    NORMAL = 0
    CONNECTION_ERROR = 1
    DATA_PROCESSING_ERROR = 2
    CONFIGURATION_ERROR = 3

class KuavoPicoServer:
    """Server class for Pico VR device integration."""
    
    def __init__(self, *args, **kwargs):
        """Initialize the Pico server."""
        self.host = kwargs.get('host', '0.0.0.0')
        self.port = kwargs.get('port', 12345)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host, self.port))
        self.socket.settimeout(1)

        self.data_queue = queue.Queue(maxsize=5)
        self.robot_matrix_publisher = RobotMatrixPublisher()
        
        KuavoPicoNodeManager.get_instance('kuavo_pico_node')
        
        # Default configuration
        self.DEFAULT_CONFIG = {
            "head_motion_range": {
                "pitch": [-30, 30],  # Pitch range in degrees
                "yaw": [-60, 60]     # Yaw range in degrees
            }
        }
        self.config = self.DEFAULT_CONFIG.copy()
        
        # Error handling
        self.error_state = ErrorState.NORMAL
        self.error_count = 0
        self.max_retries = 3
        self.retry_delay = 1.0  # seconds
        
        # Initialize PicoNode
        self.pico_node = None

        # Start data processing thread
        self.process_data_thread = threading.Thread(target=self.process_data_thread, daemon=True)
        self.process_data_thread.start()
        
        # Start robot information broadcast thread
        self.broadcaster = RobotInfoBroadcaster()
        self.broadcast_thread = threading.Thread(target=self.broadcaster.broadcast, daemon=True)
        self.broadcast_thread.start()

        self.robot_data_server = RobotDataServer(self.broadcaster.eef_type)
        self.robot_data_server_thread = threading.Thread(target=self.robot_data_server.start, daemon=True)
        self.robot_data_server_thread.start()
        
    def get_head_motion_range(self):
        """Get head motion range from configuration."""
        return self.config["head_motion_range"]

    def send_initial_message(self):
        """Send initial message to establish connection."""
        message = b'hi'
        max_retries = 200
        for attempt in range(max_retries):
            try:
                self.socket.sendto(message, (self.host, self.port))
                data, addr = self.socket.recvfrom(1024)
                print(f"\033[92mAcknowledgment From PICO received on attempt {attempt + 1}, start to receiving data...\033[0m")
                return True
            except socket.timeout:
                SDKLogger.warning(f"Pico timeout: Attempt {attempt + 1} timed out. Retrying...")
            except KeyboardInterrupt:
                SDKLogger.info("Force quit by Ctrl-c.")
                return False
        SDKLogger.error("Failed to send message after 200 attempts.")
        return False

    def clean_up(self):
        """Cleanup when the object is destroyed."""
        if self.socket:
            self.socket.close()
        SDKLogger.info("Stopped UDP server")

    def handle_error(self, error, error_state):
        """Handle errors and determine if should continue."""
        self.error_state = error_state
        self.error_count += 1
        
        if self.error_count > self.max_retries:
            SDKLogger.error(f"Max retries exceeded, stopping server: {error}")
            return False
        
        SDKLogger.warning(f"Error occurred (attempt {self.error_count}/{self.max_retries}): {error}")
        time.sleep(self.retry_delay)
        return True

    def process_data(self, data_str):
        """Process data."""
        robot_urdf_matrices, current_time = self.pico_node.pico_info_transformer.get_robot_urdf_matrix(data_str)
        self.robot_matrix_publisher.publish_matrices(robot_urdf_matrices, current_time)

        # Publish TF transforms
        self.pico_node.pico_info_transformer.publish_tf_transforms(robot_urdf_matrices, current_time)

        if self.pico_node.pico_info_transformer.control_mode == 'WholeBody':
            # Publish local poses
            self.pico_node.pico_info_transformer.publish_local_poses(robot_urdf_matrices, current_time)

            # Process foot pose for stepping using parallel detection
            self.pico_node.pico_info_transformer.process_foot_poses_parallel(robot_urdf_matrices)

            # Process body pose for stepping (Another option of lower body control)
            # self.pico_node.pico_info_transformer.process_body_pose_for_stepping(robot_urdf_matrices)

        elif self.pico_node.pico_info_transformer.control_mode == 'UpperBody':
            # Process body pose for stepping 
            self.pico_node.pico_info_transformer.publish_local_poses(robot_urdf_matrices, current_time)
            
        elif self.pico_node.pico_info_transformer.control_mode == 'LowerBody':
            # Process foot pose for stepping using parallel detection
            self.pico_node.pico_info_transformer.process_foot_poses_parallel(robot_urdf_matrices)
            # self.pico_node.pico_info_transformer.publish_foot_pose(robot_urdf_matrices)

            # Process body pose for stepping (Another option of lower body control)
            # self.pico_node.pico_info_transformer.process_body_pose_for_stepping(robot_urdf_matrices)

        # # Control torso (TODO: fix this)
        # if self.pico_node.pico_info_transformer.is_running and self.pico_node.pico_info_transformer.control_torso_mode:
        #     self.pico_node.pico_info_transformer.compute_head_body_pose(robot_urdf_matrices)
        #     self.pico_node.pico_info_transformer.pub_head_body_pose_msg(self.pico_node.pico_info_transformer.head_body_pose)
    
    def process_pico_joy_data(self, data_str):
        """Process pico joy data and print to console."""
        try:
            joy_data = self.pico_node.pico_info_transformer.parse_joy_data(data_str)
            if not joy_data:
                SDKLogger.warn("no pico joy data")
                return
            self.pico_node.pico_info_transformer.publish_pico_joys(joy_data)
        except Exception as e:
            SDKLogger.error(f"Error processing pico joy data: {e}")

    def process_data_thread(self):
        """Thread function for processing data."""
        SDKLogger.info("Data processing thread started")
        while not KuavoPicoNodeManager.get_ros_is_shutdown():
            try:
                data_str = self.data_queue.get(timeout=0.1)
                self.process_data(data_str)
                self.process_pico_joy_data(data_str) # 处理pico手柄数据
            except queue.Empty:
                continue
            except Exception as e:
                SDKLogger.error(f"Error processing data: {e}")

    def start(self):
        """Start server."""
        SDKLogger.info("Starting PicoServer...")
        self.pico_node = KuavoPicoNode()
        
        self.socket.settimeout(1)
        
        SDKLogger.info(f"PICO UDP Server listening on {self.host}:{self.port}")
        
        if not self.send_initial_message():
            SDKLogger.error("Failed to establish initial connection")
            return
        
        # while not KuavoPicoNodeManager.get_ros_is_shutdown():
        #     actions = self.pico_node.pico_info_transformer.get_available_test_actions()
        #     # self.pico_node.pico_info_transformer.test_predefined_action(actions[0])
        #     for action in actions:
        #         self.pico_node.pico_info_transformer.test_predefined_action(action)
        #         time.sleep(10)
        #     # break

        SDKLogger.info("Waiting for data...")
        while not KuavoPicoNodeManager.get_ros_is_shutdown():
            try:
                data, addr = self.socket.recvfrom(4096)
                if(self.robot_data_server.target_client_addr != addr):
                    SDKLogger.debug(f"get new client: {addr}")
                    self.robot_data_server.target_client_addr = addr
                try:
                    self.data_queue.put(data.decode('utf-8'), block=False)
                except queue.Full:
                    try:
                        self.data_queue.get_nowait()
                        self.data_queue.put_nowait(data.decode('utf-8'))
                    except queue.Empty:
                        pass
            except socket.timeout:
                continue
            except Exception as e:
                SDKLogger.error(f"Error in main loop: {e}")
                if not self.handle_error(e, ErrorState.CONNECTION_ERROR):
                    break
    
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    args = parse_args()
    try:
        server = KuavoPicoServer(**vars(args))
        server.start()
    except Exception as e:
        server.clean_up()
        SDKLogger.error(f"Node interrupted: {e}")