#!/usr/bin/env python3
# coding: utf-8

"""
High-level interface for Pico VR device control.
"""

# Standard library imports
import os
import sys
import time
import signal
import socket
import argparse
from enum import Enum
from typing import Optional, Dict, Any, Tuple, List

# Third-party imports
import rospy

# Local imports
from core.pico import (
    KuavoPicoServer,
    ErrorState
)
from core.ros.pico import KuavoPicoNode, KuavoPicoNodeManager
from common.logger import SDKLogger

class RobotState(Enum):
    """Enum for tracking robot states"""
    DISCONNECTED = 0
    CONNECTING = 1
    CONNECTED = 2
    ERROR = 3

class KuavoRobotPico:
    """Interface for Pico whole body teleoperation.
    - Device connection and error handling
    - Motion control
    - State management
    - Hand pose control
    - Configuration management
    
    Example:
        >>> pico = KuavoRobotPico()
        >>> pico.connect()
        >>> pico.run()
    """
    def __init__(self):
        """Initialize Pico control interface.
        
        Creates an instance of the core Pico implementation that handles
        all the low-level details of device communication and control.
        """
        # Initialize ROS node using KuavoPicoNodeManager
        KuavoPicoNodeManager.get_instance('kuavo_pico_node')
        
        self.pico_server = None
        self.pico_node = None  # Will be initialized in connect()
        self.service_mode = True
        self._running = True
        
        # Play mode flag
        self._play_mode = False
        
        # State management
        self.robot_state = RobotState.DISCONNECTED
        self.error_state = ErrorState.NORMAL
        self.error_count = 0
        self.max_retries = 3
        self.retry_delay = 1.0  # seconds
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        
    def _signal_handler(self, signum, frame):
        """处理 Ctrl+C 信号"""
        SDKLogger.info("Quiting...")
        self._running = False
        if self.pico_server:
            self.disconnect()
        sys.exit(0)
        
    def connect(self, host: str = '0.0.0.0', port: int = 12345) -> bool:
        """Connect to Pico device.
        
        Args:
            host (str): Host address to bind to
            port (int): Port number for communication
            
        Returns:
            bool: True if connection successful, False otherwise
            
        Example:
            >>> pico.connect("0.0.0.0", 12345)
        """
        self.robot_state = RobotState.CONNECTING
        retry_count = 0
        
        while retry_count < self.max_retries:
            try:
                # Create new KuavoPicoServer instance with specified host and port
                self.pico_server = KuavoPicoServer(host=host, port=port)
                # Create KuavoPicoNode instance for configuration
                self.pico_node = KuavoPicoNode(play_mode=self._play_mode)
                self.robot_state = RobotState.CONNECTED
                self.error_state = ErrorState.NORMAL
                SDKLogger.info(f"Successfully connected to Pico at {host}:{port}")
                return True
            except socket.error as e:
                SDKLogger.error(f"PICO Connect, Socket error: {e}")
                retry_count += 1
                if retry_count < self.max_retries:
                    SDKLogger.info(f"Retrying connection (attempt {retry_count}/{self.max_retries})...")
                    time.sleep(self.retry_delay)
            except Exception as e:
                SDKLogger.error(f"Error connecting to Pico: {e}")
                retry_count += 1
                if retry_count < self.max_retries:
                    SDKLogger.info(f"Retrying connection (attempt {retry_count}/{self.max_retries})...")
                    time.sleep(self.retry_delay)
        
        self.robot_state = RobotState.ERROR
        self.error_state = ErrorState.CONNECTION_ERROR
        return False
        
    def disconnect(self):
        """Disconnect from Pico device and release resources."""
        if self.pico_server:
            try:
                self.pico_server.clean_up()
                self.robot_state = RobotState.DISCONNECTED
                SDKLogger.info("Successfully disconnected from Pico")
            except Exception as e:
                SDKLogger.error(f"Error during disconnect: {e}")
                self.robot_state = RobotState.ERROR
                self.error_state = ErrorState.CONNECTION_ERROR
        
    def run(self, callback=None, poll_interval: float = 0.01):
        """
        Start the Pico server and run the main loop.
        
        This method starts the Pico server which will begin listening for data
        and processing it automatically. The server runs in a blocking manner
        until interrupted or shutdown.
        
        Args:
            callback: User-defined processing function, takes data dict as parameter
            poll_interval: Polling interval in seconds (not used in current implementation)
        """
        if not self.pico_server:
            SDKLogger.error("Pico server not initialized. Please call connect() first.")
            return
            
        try:
            SDKLogger.info("Starting Pico server...")
            # The start() method is blocking and will run until interrupted
            self.pico_server.start()
        except KeyboardInterrupt:
            SDKLogger.info("Pico server interrupted by user")
        except Exception as e:
            SDKLogger.error(f"Error in Pico server: {e}")
            self.error_state = ErrorState.DATA_PROCESSING_ERROR
            self.error_count += 1
            if self.error_count > self.max_retries:
                raise
        finally:
            SDKLogger.info("Pico server stopped")
        
    def set_control_mode(self, mode: str):
        """Set control mode.
        
        Args:
            mode (bool): Whether to enable torso control
                - "WholeBody": Whole body control
                - "UpperBody": Upper body control
                - "LowerBody": Lower body control
                
        Example:
            >>> pico.set_control_mode("WholeBody")  # Enable whole body control
        """
        if self.pico_node:
            self.pico_node.pico_info_transformer.set_control_mode(mode)

    def set_control_torso_mode(self, mode: bool):
        """Set torso control mode.
        
        Enables or disables torso control functionality.
        When enabled, the Pico device can control the robot's
        torso movements.
        
        Args:
            mode (bool): Whether to enable torso control
                - True: Torso control is enabled
                - False: Torso control is disabled
                
        Example:
            >>> pico.set_control_torso_mode(True)  # Enable torso control
        """
        if self.pico_node:
            self.pico_node.pico_info_transformer.set_control_torso_mode(mode)

    def set_service_mode(self, enable: bool):
        """
        Set whether to automatically call service to switch control mode.
        
        Args:
            enable: True for automatic service calls, False for manual/disabled
        """
        self.service_mode = enable
        if self.pico_node:
            self.pico_node.send_srv = enable
        SDKLogger.info(f"Service mode set to {enable}")

    def set_play_mode(self, enable: bool):
        """Enable or disable play mode (controls /robot_body_matrices subscription)."""
        self._play_mode = enable
        if self.pico_node:
            if hasattr(self.pico_node, 'set_play_mode'):
                self.pico_node.set_play_mode(enable)

    def change_arm_ctrl_mode(self, mode: int):
        """Change arm control mode.
        
        Args:
            mode (int): Control mode to set
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            try:
                self.pico_node.change_arm_ctrl_mode(mode)
                SDKLogger.info(f"Arm control mode changed to {mode}")
            except Exception as e:
                SDKLogger.error(f"Error changing arm control mode: {e}")
                self.error_state = ErrorState.CONFIGURATION_ERROR

    def change_mobile_ctrl_mode(self, mode: int):
        """Change mobile control mode.
        
        Args:
            mode (int): Control mode to set
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            try:
                self.pico_node.change_mobile_ctrl_mode(mode)
                SDKLogger.info(f"Mobile control mode changed to {mode}")
            except Exception as e:
                SDKLogger.error(f"Error changing mobile control mode: {e}")
                self.error_state = ErrorState.CONFIGURATION_ERROR

    def change_mm_wbc_arm_ctrl_mode(self, mode: int):
        """Change MM WBC arm control mode.
        
        Args:
            mode (int): Control mode to set
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            try:
                self.pico_node.change_mm_wbc_arm_ctrl_mode(mode)
                SDKLogger.info(f"MM WBC arm control mode changed to {mode}")
            except Exception as e:
                SDKLogger.error(f"Error changing MM WBC arm control mode: {e}")
                self.error_state = ErrorState.CONFIGURATION_ERROR

    def configure_pico_node(self, control_mode: str = "WholeBody", 
                           control_torso: bool = False, service_mode: bool = True):
        """Configure Pico node parameters.
        
        This method should be called after connecting but before starting the server.
        
        Args:
            control_mode (str): Control mode ('WholeBody', 'UpperBody', 'LowerBody')
            control_torso (bool): Whether to enable torso control
            service_mode (bool): Whether to enable service mode
        """
        if self.pico_node:
            self.pico_node.pico_info_transformer.set_control_mode(control_mode)
            self.pico_node.pico_info_transformer.set_control_torso_mode(control_torso)
            self.pico_node.send_srv = service_mode
            SDKLogger.info("Pico node configured successfully")
        else:
            SDKLogger.warning("Pico node not available for configuration")

    def get_movement_detector_config(self) -> dict:
        """Get current movement detector configuration.
        
        Returns:
            dict: Current movement detector configuration dictionary
                - horizontal_threshold: Horizontal position threshold (meters)
                - vertical_threshold: Vertical position threshold (meters) 
                - angle_threshold: Angle threshold (degrees)
                - step_length: Step length (meters)
                - initial_left_foot_pose: Initial left foot pose [x, y, z, yaw]
                - initial_right_foot_pose: Initial right foot pose [x, y, z, yaw]
                - initial_body_pose: Initial body pose [x, y, z, yaw] or None
                
        Example:
            >>> config = pico.get_movement_detector_config()
            >>> print(f"Horizontal threshold: {config['horizontal_threshold']}m")
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            return self.pico_node.pico_info_transformer.movement_detector.copy()
        else:
            SDKLogger.warning("Pico node not available for getting movement detector config")
            return {}

    def set_use_real_foot_data(self, use_real: bool):
        """设置 complete_action_detector 的 use_real_foot_data 字段（左右脚同时设置）"""
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            try:
                detector = self.pico_node.pico_info_transformer.complete_action_detector
                for side in ['left', 'right']:
                    if side in detector:
                        detector[side]['use_real_foot_data'] = use_real
                SDKLogger.info(f"Set use_real_foot_data to {use_real} for both feet")
                return True
            except Exception as e:
                SDKLogger.error(f"Error setting use_real_foot_data: {e}")
                return False
        else:
            SDKLogger.warning("Pico node not available for setting use_real_foot_data")
            return False

    def get_use_real_foot_data(self):
        """获取 complete_action_detector 的 use_real_foot_data 字段（返回左右脚）"""
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            detector = self.pico_node.pico_info_transformer.complete_action_detector
            return {
                'left': detector.get('left', {}).get('use_real_foot_data', False),
                'right': detector.get('right', {}).get('use_real_foot_data', False)
            }
        else:
            SDKLogger.warning("Pico node not available for getting use_real_foot_data")
            return {'left': False, 'right': False}

    def set_movement_detector_config(self, config: dict) -> bool:
        """Set movement detector configuration (自动应用 use_real_foot_data)"""
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            try:
                self.pico_node.pico_info_transformer.set_movement_detector(config)
                # Handle detection mode configuration
                if 'detection_mode' in config:
                    detection_mode = config['detection_mode']
                    if detection_mode == 'complete_action':
                        self.pico_node.pico_info_transformer.enable_complete_action_detection()
                        SDKLogger.info("Complete action detection mode enabled")
                    elif detection_mode == 'threshold':
                        self.pico_node.pico_info_transformer.enable_threshold_detection()
                        SDKLogger.info("Threshold-based detection mode enabled")
                    else:
                        SDKLogger.warning(f"Unknown detection mode: {detection_mode}")
                # Handle complete action parameters
                if 'complete_action' in config:
                    complete_action_params = config['complete_action']
                    self.pico_node.pico_info_transformer.set_complete_action_parameters(complete_action_params)
                    SDKLogger.info("Complete action parameters updated")
                    # 自动设置 use_real_foot_data
                    if 'use_real_foot_data' in complete_action_params:
                        detector = self.pico_node.pico_info_transformer.complete_action_detector
                        for side in ['left', 'right']:
                            if side in detector:
                                detector[side]['use_real_foot_data'] = complete_action_params['use_real_foot_data']
                        SDKLogger.info(f"Set use_real_foot_data to {complete_action_params['use_real_foot_data']} for both feet")
                SDKLogger.info("Movement detector configuration updated successfully")
                return True
            except Exception as e:
                SDKLogger.error(f"Error setting movement detector config: {e}")
                self.error_state = ErrorState.CONFIGURATION_ERROR
                return False
        else:
            SDKLogger.warning("Pico node not available for setting movement detector config")
            return False

    def set_detection_mode(self, mode: str) -> bool:
        """Set movement detection mode.
        
        Args:
            mode (str): Detection mode
                - 'threshold': Threshold-based detection (real-time)
                - 'complete_action': Complete action detection (ground-lift-ground)
                
        Returns:
            bool: True if mode was set successfully, False otherwise
            
        Example:
            >>> pico.set_detection_mode('complete_action')  # Enable complete action detection
            >>> pico.set_detection_mode('threshold')       # Enable threshold detection
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            try:
                if mode == 'complete_action':
                    self.pico_node.pico_info_transformer.enable_complete_action_detection()
                    SDKLogger.info("Complete action detection mode enabled")
                elif mode == 'threshold':
                    self.pico_node.pico_info_transformer.enable_threshold_detection()
                    SDKLogger.info("Threshold-based detection mode enabled")
                else:
                    SDKLogger.error(f"Invalid detection mode: {mode}")
                    return False
                return True
            except Exception as e:
                SDKLogger.error(f"Error setting detection mode: {e}")
                self.error_state = ErrorState.CONFIGURATION_ERROR
                return False
        else:
            SDKLogger.warning("Pico node not available for setting detection mode")
            return False

    def get_detection_mode(self) -> str:
        """Get current detection mode.
        
        Returns:
            str: Current detection mode ('threshold' or 'complete_action')
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            return self.pico_node.pico_info_transformer.get_detection_mode()
        else:
            SDKLogger.warning("Pico node not available for getting detection mode")
            return 'threshold'

    def set_complete_action_parameters(self, parameters: dict) -> bool:
        """Set parameters for complete action detection.
        
        Args:
            parameters (dict): Complete action detection parameters
                - lift_threshold: Height threshold for lift detection (meters)
                - ground_threshold: Height threshold for ground contact (meters)
                - min_action_duration: Minimum valid action duration (seconds)
                - max_action_duration: Maximum valid action duration (seconds)
                - action_buffer_size: Size of pose history buffer
                - min_horizontal_movement: Minimum horizontal movement distance (meters)
                
        Returns:
            bool: True if parameters were set successfully, False otherwise
            
        Example:
            >>> params = {
            ...     'lift_threshold': 0.05,      # 5cm
            ...     'ground_threshold': 0.02,    # 2cm
            ...     'min_action_duration': 0.3,  # 0.3s
            ...     'max_action_duration': 10.0, # 10.0s
            ...     'min_horizontal_movement': 0.05  # 5cm
            ... }
            >>> pico.set_complete_action_parameters(params)
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            try:
                self.pico_node.pico_info_transformer.set_complete_action_parameters(parameters)
                SDKLogger.info("Complete action parameters updated successfully")
                return True
            except Exception as e:
                SDKLogger.error(f"Error setting complete action parameters: {e}")
                self.error_state = ErrorState.CONFIGURATION_ERROR
                return False
        else:
            SDKLogger.warning("Pico node not available for setting complete action parameters")
            return False

    def get_complete_action_parameters(self) -> dict:
        """Get current complete action detection parameters.
        
        Returns:
            dict: Current complete action detection parameters
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            return self.pico_node.pico_info_transformer.get_complete_action_parameters()
        else:
            SDKLogger.warning("Pico node not available for getting complete action parameters")
            return {}

    def get_detection_info(self) -> dict:
        """Get comprehensive detection information.
        
        Returns:
            dict: Detection information including current mode, parameters, and status
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            return self.pico_node.pico_info_transformer.get_detection_info()
        else:
            SDKLogger.warning("Pico node not available for getting detection info")
            return {}

    def set_parallel_detection_config(self, config: dict) -> bool:
        """Set parallel detection configuration.
        
        Args:
            config (dict): Parallel detection configuration dictionary
                - enabled: bool - Whether to enable parallel detection
                - timeout_ms: int - Detection timeout in milliseconds
                - max_workers: int - Maximum number of worker threads
                - precise_timing: bool - Whether to use precise timing
                - debug_mode: bool - Enable debug mode
                
        Returns:
            bool: True if configuration was set successfully, False otherwise
            
        Example:
            >>> config = {
            ...     'enabled': True,           # Enable parallel detection
            ...     'timeout_ms': 50,          # 50ms timeout
            ...     'max_workers': 2,          # 2 worker threads
            ...     'precise_timing': True,    # Use precise timing
            ...     'debug_mode': False        # Disable debug mode
            ... }
            >>> pico.set_parallel_detection_config(config)
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            try:
                self.pico_node.pico_info_transformer.set_parallel_detection_config(config)
                SDKLogger.info("Parallel detection configuration updated successfully")
                return True
            except Exception as e:
                SDKLogger.error(f"Error setting parallel detection config: {e}")
                self.error_state = ErrorState.CONFIGURATION_ERROR
                return False
        else:
            SDKLogger.warning("Pico node not available for setting parallel detection config")
            return False

    def get_parallel_detection_config(self) -> dict:
        """Get current parallel detection configuration.
        
        Returns:
            dict: Current parallel detection configuration
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            return self.pico_node.pico_info_transformer.get_parallel_detection_config()
        else:
            SDKLogger.warning("Pico node not available for getting parallel detection config")
            return {}

    def get_parallel_detection_status(self) -> dict:
        """Get parallel detection status.
        
        Returns:
            dict: Parallel detection status including:
                - enabled: bool - Whether parallel detection is enabled
                - executor_active: bool - Whether thread pool is active
                - active_futures: int - Number of active detection tasks
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            return self.pico_node.pico_info_transformer.get_parallel_detection_status()
        else:
            SDKLogger.warning("Pico node not available for getting parallel detection status")
            return {}

    def get_parallel_detection_performance(self) -> dict:
        """Get parallel detection performance statistics.
        
        Returns:
            dict: Performance statistics including:
                - total_detections: int - Total number of detections
                - successful_detections: int - Number of successful detections
                - timeout_count: int - Number of timeouts
                - error_count: int - Number of errors
                - avg_detection_time: float - Average detection time in seconds
                - detection_rate: float - Detections per second
                - success_rate: float - Success rate percentage
                - uptime_seconds: float - Total uptime in seconds
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            return self.pico_node.pico_info_transformer.get_parallel_detection_performance()
        else:
            SDKLogger.warning("Pico node not available for getting parallel detection performance")
            return {}

    def reset_parallel_detection_performance(self) -> bool:
        """Reset parallel detection performance statistics.
        
        Returns:
            bool: True if reset was successful, False otherwise
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            try:
                self.pico_node.pico_info_transformer.reset_parallel_detection_performance()
                SDKLogger.info("Parallel detection performance statistics reset")
                return True
            except Exception as e:
                SDKLogger.error(f"Error resetting parallel detection performance: {e}")
                return False
        else:
            SDKLogger.warning("Pico node not available for resetting parallel detection performance")
            return False

    def reset_parallel_detector(self) -> bool:
        """Reset parallel detector (shutdown and reinitialize).
        
        Returns:
            bool: True if reset was successful, False otherwise
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            try:
                self.pico_node.pico_info_transformer.reset_parallel_detector()
                SDKLogger.info("Parallel detector reset successfully")
                return True
            except Exception as e:
                SDKLogger.error(f"Error resetting parallel detector: {e}")
                return False
        else:
            SDKLogger.warning("Pico node not available for resetting parallel detector")
            return False

    def enable_parallel_detection(self) -> bool:
        """Enable parallel detection mode.
        
        Returns:
            bool: True if enabled successfully, False otherwise
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            try:
                self.pico_node.pico_info_transformer.enable_parallel_detection()
                SDKLogger.info("Parallel detection enabled")
                return True
            except Exception as e:
                SDKLogger.error(f"Error enabling parallel detection: {e}")
                return False
        else:
            SDKLogger.warning("Pico node not available for enabling parallel detection")
            return False

    def disable_parallel_detection(self) -> bool:
        """Disable parallel detection mode.
        
        Returns:
            bool: True if disabled successfully, False otherwise
        """
        if self.pico_node and self.robot_state == RobotState.CONNECTED:
            try:
                self.pico_node.pico_info_transformer.disable_parallel_detection()
                SDKLogger.info("Parallel detection disabled")
                return True
            except Exception as e:
                SDKLogger.error(f"Error disabling parallel detection: {e}")
                return False
        else:
            SDKLogger.warning("Pico node not available for disabling parallel detection")
            return False

def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Pico Robot Control')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                      help='Pico server host (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=12345,
                      help='Pico server port (default: 12345)')
    parser.add_argument('--control_mode', type=str, default='WholeBody',
                      choices=['WholeBody', 'UpperBody', 'LowerBody'],
                      help='Control mode (default: WholeBody)')
    parser.add_argument('--control_torso', type=int, default=0,
                      help='Enable torso control (0: disabled, 1: enabled)')
    parser.add_argument('--service_mode', type=int, default=1,
                      help='Enable service mode (0: disabled, 1: enabled)')
    parser.add_argument('--detection_mode', type=str, default='threshold',
                      choices=['threshold', 'complete_action'],
                      help='Movement detection mode (default: threshold)')
    
    # Parallel detection arguments
    parser.add_argument('--parallel_detection', type=int, default=1,
                      help='Enable parallel detection (0: disabled, 1: enabled, default: 1)')
    parser.add_argument('--parallel_timeout_ms', type=int, default=50,
                      help='Parallel detection timeout in milliseconds (default: 50)')
    parser.add_argument('--parallel_workers', type=int, default=2,
                      help='Number of parallel detection worker threads (default: 2)')
    parser.add_argument('--parallel_debug', type=int, default=0,
                      help='Enable parallel detection debug mode (0: disabled, 1: enabled)')
    
    parser.add_argument('--debug', action='store_true',
                      help='Enable debug output')
    return parser.parse_args()

if __name__ == "__main__":
    # Parse command line arguments
    args = parse_args()

    try:
        pico = KuavoRobotPico()
        # Connect to Pico first
        if not pico.connect(args.host, args.port):
            sys.exit(1)
            
        # Configure the pico node with the specified parameters
        pico.configure_pico_node(
            control_mode=args.control_mode,
            control_torso=bool(args.control_torso),
            service_mode=bool(args.service_mode)
        )
        
        # Configure movement detector with custom settings
        # 默认参数与 pico_utils.py 保持一致
        movement_config = {
            'horizontal_threshold': 0.2,    # 20cm position threshold
            'vertical_threshold': 0.01,    # 1cm vertical threshold
            'angle_threshold': 20.0,       # 20 degrees angle threshold
            'step_length': 0.2,            # 20cm step length
            'initial_left_foot_pose': [0.0, 0.1, 0.0, 0.0],
            'initial_right_foot_pose': [0.0, -0.1, 0.0, 0.0],
            'initial_body_pose': None,
            'detection_mode': args.detection_mode,  # 'threshold' or 'complete_action'
            'complete_action': {
                'lift_threshold': 0.03,     # 3cm 抬起阈值
                'ground_threshold': 0.01,   # 1cm 地面阈值
                'min_action_duration': 0.3, # 0.3秒
                'max_action_duration': 10.0, # 10秒
                'action_buffer_size': 50,   # 缓冲区大小
                'min_horizontal_movement': 0.05,  # 5cm
                # 自适应阈值相关
                'adaptive_threshold_enabled': True,
                'calibration_samples': 50,
                'adaptive_lift_offset': 0.02,
                'adaptive_ground_offset': 0.005,
            },
            'parallel_detection': {
                'enabled': bool(args.parallel_detection),
                'timeout_ms': args.parallel_timeout_ms,
                'max_workers': args.parallel_workers,
                'precise_timing': True,
                'debug_mode': bool(args.parallel_debug),
            }
        }

        # 兼容命令行参数覆盖部分默认值
        if args.detection_mode == 'complete_action':
            movement_config['detection_mode'] = 'complete_action'
            # 可根据需要覆盖 complete_action 子参数
            # movement_config['complete_action'].update({...})
        else:
            movement_config['detection_mode'] = 'threshold'

        SDKLogger.info(f"Using detection mode: {movement_config['detection_mode']}")

        if pico.set_movement_detector_config(movement_config):
            SDKLogger.info("Movement detector configured with custom settings")
            
            # Print current detection information
            detection_info = pico.get_detection_info()
            SDKLogger.info(f"Current detection mode: {detection_info.get('current_mode', 'unknown')}")
        else:
            SDKLogger.warning("Failed to configure movement detector with custom settings")
            
        pico.run()
    except KeyboardInterrupt:
        SDKLogger.info("\nNode interrupted")
    except Exception as e:
        SDKLogger.error(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up
        if 'pico' in locals():
            pico.disconnect()
        SDKLogger.info("Node shutdown")
