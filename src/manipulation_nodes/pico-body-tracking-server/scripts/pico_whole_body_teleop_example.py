#!/usr/bin/env python3
# coding: utf-8

"""
Pico Whole Body Teleoperation Example

This example demonstrates how to use the KuavoRobotPico interface
to perform whole body teleoperation with a Pico VR device.
Includes parallel detection functionality for improved performance.
"""

import time
import signal
import sys
import argparse
from robot_pico import KuavoRobotPico
from common.logger import SDKLogger

def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Pico Whole Body Teleoperation Example')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                      help='Pico server host (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=12345,
                      help='Pico server port (default: 12345)')
    parser.add_argument('--control_mode', type=str, default='LowerBody',
                      choices=['WholeBody', 'UpperBody', 'LowerBody'],
                      help='Control mode (default: LowerBody)')
    parser.add_argument('--control_torso', type=int, default=0,
                      help='Enable torso control (0: disabled, 1: enabled)')
    parser.add_argument('--service_mode', type=int, default=1,
                      help='Enable service mode (0: disabled, 1: enabled)')
    parser.add_argument('--detection_mode', type=str, default='complete_action',
                      choices=['threshold', 'complete_action'],
                      help='Movement detection mode (default: complete_action)')
    
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
    parser.add_argument('--use_real_foot_data', type=int, default=1,
                      help='Use real foot data for complete action detection (0: disabled, 1: enabled, default: 0)')
    # Add play mode argument
    parser.add_argument('--play_mode', type=int, default=0,
                      help='Enable play mode (0: disabled, 1: enabled, default: 0)')
    parser.add_argument('--max_step_length_x', type=float, default=0.3,
                      help='Maximum step length in x direction (default: 0.3)')
    parser.add_argument('--max_step_length_y', type=float, default=0.15,
                      help='Maximum step length in y direction (default: 0.15)')
    return parser.parse_args()

def print_status(pico: KuavoRobotPico):
    """Print current Pico status information."""
    status_info = {
        'Robot State': pico.robot_state.name,
        'Error State': pico.error_state.name if pico.error_state else 'NORMAL',
        'Detection Mode': pico.get_detection_mode(),
        'Parallel Detection': 'Enabled' if pico.get_parallel_detection_status().get('enabled', False) else 'Disabled'
    }
    
    print("\r" + " | ".join([f"{k}: {v}" for k, v in status_info.items()]), end="", flush=True)

def configure_detection_system(pico: KuavoRobotPico, args):
    """Configure detection system based on command line arguments."""

    # Configure parallel detection
    parallel_config = {
        'enabled': bool(args.parallel_detection),
        'timeout_ms': args.parallel_timeout_ms,
        'max_workers': args.parallel_workers,
        'precise_timing': True,
        'debug_mode': bool(args.parallel_debug)
    }
    if pico.set_parallel_detection_config(parallel_config):
        SDKLogger.info("Parallel detection configured successfully")
    else:
        SDKLogger.warning("Failed to configure parallel detection")

    # Configure movement detection
    movement_config = {
        'horizontal_threshold': 0.1,    # 20cm position threshold
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
            'min_action_duration': 0.0, # 0.0秒
            'max_action_duration': 2.0, # 2秒
            'action_buffer_size': 50,   # 缓冲区大小
            'min_horizontal_movement': 0.05,  # 5cm
            # 自适应阈值相关
            'adaptive_threshold_enabled': True,
            'calibration_samples': 50,
            'adaptive_lift_offset': 0.01,
            'adaptive_ground_offset': 0.005,
            'use_real_foot_data': bool(args.use_real_foot_data),
        },
        'parallel_detection': parallel_config,
        'max_step_length_x': args.max_step_length_x,
        'max_step_length_y': args.max_step_length_y,
    }

    # 兼容命令行参数覆盖部分默认值
    if args.detection_mode == 'complete_action':
        movement_config['detection_mode'] = 'complete_action'
        # 可根据需要覆盖 complete_action 子参数
        # movement_config['complete_action'].update({...})
        SDKLogger.info("Using complete action detection mode")
    else:
        movement_config['detection_mode'] = 'threshold'
        SDKLogger.info("Using threshold-based detection mode")

    if pico.set_movement_detector_config(movement_config):
        SDKLogger.info("Movement detector configured successfully")
    else:
        SDKLogger.warning("Failed to configure movement detector")

def print_system_info(pico: KuavoRobotPico):
    """Print system configuration and status information."""
    print("\n=== System Configuration ===")
    
    # Detection mode info
    detection_mode = pico.get_detection_mode()
    print(f"Detection Mode: {detection_mode}")
    
    # Parallel detection info
    parallel_status = pico.get_parallel_detection_status()
    parallel_config = pico.get_parallel_detection_config()
    
    print(f"Parallel Detection: {'Enabled' if parallel_status.get('enabled', False) else 'Disabled'}")
    if parallel_status.get('enabled', False):
        print(f"  - Timeout: {parallel_config.get('timeout_ms', 0)}ms")
        print(f"  - Workers: {parallel_config.get('max_workers', 0)}")
        print(f"  - Debug Mode: {'Enabled' if parallel_config.get('debug_mode', False) else 'Disabled'}")
        print(f"  - Active Tasks: {parallel_status.get('active_futures', 0)}")
    
    # Detection parameters
    detection_info = pico.get_detection_info()
    if detection_mode == 'complete_action':
        complete_params = detection_info.get('complete_action_parameters', {})
        print(f"Complete Action Parameters:")
        print(f"  - Lift Threshold: {complete_params.get('lift_threshold', 0):.3f}m")
        print(f"  - Ground Threshold: {complete_params.get('ground_threshold', 0):.3f}m")
        print(f"  - Min Duration: {complete_params.get('min_action_duration', 0):.1f}s")
        print(f"  - Max Duration: {complete_params.get('max_action_duration', 0):.1f}s")
    else:
        threshold_params = detection_info.get('threshold_parameters', {})
        print(f"Threshold Parameters:")
        print(f"  - Horizontal: {threshold_params.get('horizontal_threshold', 0):.3f}m")
        print(f"  - Vertical: {threshold_params.get('vertical_threshold', 0):.3f}m")
        print(f"  - Angle: {threshold_params.get('angle_threshold', 0):.1f}°")
    
    print("=" * 30)

def monitor_performance(pico: KuavoRobotPico, duration: int = 10):
    """Monitor system performance for a specified duration."""
    print(f"\n=== Performance Monitoring ({duration}s) ===")
    start_time = time.time()
    
    while time.time() - start_time < duration:
        # Get performance stats
        performance = pico.get_parallel_detection_performance()
        if performance:
            print(f"\rDetections: {performance.get('total_detections', 0)}, "
                  f"Success Rate: {performance.get('success_rate', 0):.1f}%, "
                  f"Avg Time: {performance.get('avg_detection_time', 0)*1000:.2f}ms, "
                  f"Rate: {performance.get('detection_rate', 0):.1f} Hz", end="", flush=True)
        
        time.sleep(1)
    
    print("\nPerformance monitoring completed")

def main():
    """Main function for Pico whole body teleoperation."""
    # Parse command line arguments
    args = parse_args()
    
    try:
        # Create Pico robot instance
        pico = KuavoRobotPico()

        # Set play mode if supported
        if hasattr(pico, 'set_play_mode'):
            pico.set_play_mode(bool(args.play_mode))

        # Connect to Pico device
        if not pico.connect(args.host, args.port):
            SDKLogger.error("Failed to connect to Pico device!")
            sys.exit(1)
        
        # Configure basic parameters
        pico.configure_pico_node(
            control_mode=args.control_mode,
            control_torso=bool(args.control_torso),
            service_mode=bool(args.service_mode)
        )
        
        # Configure detection system
        configure_detection_system(pico, args)
        
        # Print system information
        print_system_info(pico)
        
        # Optional: Monitor performance for 5 seconds
        if args.debug:
            monitor_performance(pico, 5)
        
        # Start the main operation
        SDKLogger.info("Starting Pico teleoperation...")
        SDKLogger.info("Press Ctrl+C to stop")
        
        # Start Pico server (this is blocking)
        pico.run()
        
    except KeyboardInterrupt:
        SDKLogger.info("\nKeyboard interrupt detected. Exiting...")
    except Exception as e:
        SDKLogger.error(f"Error during execution: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        if 'pico' in locals():
            SDKLogger.info("Disconnecting from Pico device...")
            pico.disconnect()
        SDKLogger.info("Pico whole body teleoperation example shutdown")

if __name__ == "__main__":
    main()
