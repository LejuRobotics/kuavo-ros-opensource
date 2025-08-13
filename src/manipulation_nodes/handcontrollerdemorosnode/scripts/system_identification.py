#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import argparse
import signal
import sys
import os
import numpy as np # Keep for main script if any direct numpy use remains, or for type hints
import time # Keep for main script if needed

# Modularized components
from system_id.motion_controller import MotionController
from system_id.signal_analyzer import SignalAnalyzer
from system_id.visualizer import Visualizer

class SystemIdentification:
    def __init__(self, args):
        self.args = args
        self.shutdown_requested = False
        self.use_chinese = self.args.language == 'cn' # Set based on args

        # Initialize components
        # Pass system parameters initially from args, they will be updated by MotionController during calibration
        self.analyzer = SignalAnalyzer(args, self.use_chinese, args.time_delay, 1.0, 0.0)
        self.visualizer = Visualizer(args, self.use_chinese)
        self.controller = MotionController(args, self.analyzer, self.visualizer, self.use_chinese)

        rospy.loginfo(self._get_label("系统辨识主节点已初始化组件", "SystemIdentification: Components initialized."))

    def _get_label(self, chinese_text, english_text):
        return chinese_text if self.use_chinese else english_text

    def _ros_signal_handler(self, sig, frame):
        rospy.loginfo(self._get_label("收到关闭请求！正在优雅退出...", "Shutdown requested! Gracefully exiting..."))
        self.shutdown_requested = True
        if self.controller:
            self.controller.request_shutdown() # Notify controller to stop activities
        rospy.signal_shutdown(self._get_label("用户通过Ctrl+C请求关闭", "User requested shutdown via Ctrl+C"))
        sys.exit(0)

    def run(self):
        rospy.init_node('system_identification_orchestrator', anonymous=True, disable_signals=False)
        signal.signal(signal.SIGINT, self._ros_signal_handler) # Setup signal handler for this main orchestrator

        rospy.loginfo(self._get_label("系统辨识流程开始", "System Identification Process Starting"))

        try:
            self.controller.initialize_ros_communications()

            # 1. Warmup Phase
            rospy.loginfo(self._get_label("--- 开始预热阶段 ---", "--- Starting Warmup Phase ---"))
            if self.shutdown_requested: return
            self.controller.run_warmup_phase()
            if self.shutdown_requested: return

            # 2. Calibration Phase
            rospy.loginfo(self._get_label("--- 开始校准阶段 ---", "--- Starting Calibration Phase ---"))
            calibration_successful = self.controller.run_calibration_phase()
            if self.shutdown_requested or not calibration_successful:
                rospy.logwarn(self._get_label("校准失败或被中断。", "Calibration failed or interrupted."))
                if not self.shutdown_requested: self.controller.publish_neutral_pose() # Ensure robot stops
                return # Cannot proceed without calibration
            
            # Update analyzer with calibrated parameters from controller
            self.analyzer.time_delay = self.controller.time_delay
            self.analyzer.static_gain = self.controller.static_gain
            self.analyzer.bias = self.controller.bias
            rospy.loginfo(self._get_label(f"校准参数更新: 延迟={self.analyzer.time_delay:.4f}s, 增益={self.analyzer.static_gain:.4f}, 偏置={self.analyzer.bias:.4f}",
                                        f"Calibration params updated: Delay={self.analyzer.time_delay:.4f}s, Gain={self.analyzer.static_gain:.4f}, Bias={self.analyzer.bias:.4f}"))

            rospy.loginfo(self._get_label("校准完成。暂停1秒...", "Calibration completed. Pausing for 1s..."))
            rospy.sleep(1.0)
            if self.shutdown_requested: return

            # 3. Frequency Sweep Phase
            rospy.loginfo(self._get_label("--- 开始频率扫描阶段 ---", "--- Starting Frequency Sweep Phase ---"))
            sweep_input_data, sweep_output_data, sweep_input_time, sweep_output_time = self.controller.run_sweep_phase()
            if self.shutdown_requested or len(sweep_input_data) < 10:
                rospy.logwarn(self._get_label("频率扫描数据不足或被中断。", "Frequency sweep data insufficient or interrupted."))
                if not self.shutdown_requested: self.controller.publish_neutral_pose()
                return
            
            rospy.loginfo(self._get_label(f"频率扫描收集到 {len(sweep_input_data)} 个输入点", 
                                        f"Frequency sweep collected {len(sweep_input_data)} input points"))

            # 4. Data Preprocessing for Sweep Data
            rospy.loginfo(self._get_label("--- 预处理频率扫描数据 ---", "--- Preprocessing Sweep Data ---"))
            proc_input, proc_output, proc_input_t, proc_output_t = \
                self.analyzer.preprocess_data(sweep_input_data, sweep_output_data, sweep_input_time, sweep_output_time)
            if self.shutdown_requested or len(proc_input) < 10:
                rospy.logwarn(self._get_label("预处理后数据不足。", "Insufficient data after preprocessing sweep data."))
                if not self.shutdown_requested: self.controller.publish_neutral_pose()
                return

            # 5. Align Sweep Data (after preprocessing) using calibrated delay for analysis
            rospy.loginfo(self._get_label("--- 对齐预处理后的扫描数据 ---", "--- Aligning Preprocessed Sweep Data ---"))
            # We use the calibrated time_delay for this final alignment before frequency analysis.
            # Target sample rate should be high enough, e.g., controller's update rate or specified.
            target_fs_analysis = self.args.update_rate 
            aligned_input_analysis, aligned_output_analysis, aligned_time_analysis, actual_fs_analysis = \
                self.analyzer.align_data_by_time(proc_input, proc_output, proc_input_t, proc_output_t, 
                                                 target_sample_rate=target_fs_analysis, 
                                                 estimated_delay=self.analyzer.time_delay) # Use calibrated delay
            
            if self.shutdown_requested or len(aligned_input_analysis) < 10:
                rospy.logwarn(self._get_label("对齐后扫描数据不足。", "Insufficient aligned sweep data for analysis."))
                if not self.shutdown_requested: self.controller.publish_neutral_pose()
                return
            
            rospy.loginfo(self._get_label(f"扫描数据对齐到 {actual_fs_analysis:.2f} Hz, {len(aligned_input_analysis)} 点", 
                                        f"Sweep data aligned to {actual_fs_analysis:.2f} Hz, {len(aligned_input_analysis)} points"))

            # (Optional) Visualize this final alignment before frequency analysis
            if self.visualizer and not self.args.no_plots:
                 self.visualizer.visualize_time_alignment(
                    proc_input_t, proc_input, proc_output_t, proc_output, # Original (but preprocessed) data
                    aligned_time_analysis, aligned_input_analysis, aligned_output_analysis,
                    self.analyzer.time_delay, self.analyzer.static_gain, self.analyzer.bias
                )

            # 6. Frequency Response Analysis
            rospy.loginfo(self._get_label("--- 分析频率响应 ---", "--- Analyzing Frequency Response ---"))
            # Pass the final aligned data (where output is effectively delay-compensated due to align_data_by_time)
            # and the calibrated parameters to the analyzer.
            # The analyzer's analyze_frequency_response will use the already aligned input and output.
            # The `known_time_delay` passed here is the one determined from calibration for logging/context.
            # The `static_gain_val` and `bias_val` are also from calibration.
            bandwidth, sys_params = self.analyzer.analyze_frequency_response(
                aligned_input_analysis, 
                aligned_output_analysis, 
                actual_fs_analysis, 
                self.analyzer.time_delay, 
                self.analyzer.static_gain, 
                self.analyzer.bias,
                self.visualizer # Pass visualizer for plotting within analysis if enabled
            )
            if self.shutdown_requested: return

            # Log final results
            rospy.loginfo("="*60)
            rospy.loginfo(self._get_label("最终系统辨识结果:", "FINAL SYSTEM IDENTIFICATION RESULTS:"))
            rospy.loginfo(self._get_label(f"  - 静态增益 (校准): {self.analyzer.static_gain:.4f}", f"  - Static Gain (calibrated): {self.analyzer.static_gain:.4f}"))
            rospy.loginfo(self._get_label(f"  - 偏置 (校准): {self.analyzer.bias:.4f}", f"  - Bias (calibrated): {self.analyzer.bias:.4f}"))
            rospy.loginfo(self._get_label(f"  - 时间延迟 (校准): {self.analyzer.time_delay:.4f}s", f"  - Time Delay (calibrated): {self.analyzer.time_delay:.4f}s"))
            if bandwidth is not None:
                rospy.loginfo(self._get_label(f"  - 带宽 (-3dB): {bandwidth:.2f} Hz", f"  - Bandwidth (-3dB): {bandwidth:.2f} Hz"))
            if sys_params:
                peak_f = sys_params.get('peak_frequency')
                res_gain_db = sys_params.get('resonance_gain_db')
                mean_coh = sys_params.get('mean_coherence')
                if peak_f is not None: rospy.loginfo(self._get_label(f"  - 峰值频率: {peak_f:.2f} Hz", f"  - Peak Frequency: {peak_f:.2f} Hz"))
                if res_gain_db is not None: rospy.loginfo(self._get_label(f"  - 共振增益: {res_gain_db:.2f} dB", f"  - Resonance Gain: {res_gain_db:.2f} dB"))
                if mean_coh is not None: rospy.loginfo(self._get_label(f"  - 平均相干性: {mean_coh:.4f}", f"  - Mean Coherence: {mean_coh:.4f}"))
                if mean_coh is not None and mean_coh < 0.7:
                    rospy.logwarn(self._get_label("  警告: 平均相干性较低，结果可能不可靠。", "  Warning: Low mean coherence, results may be unreliable."))
            rospy.loginfo("="*60)

        except KeyboardInterrupt:
            rospy.loginfo(self._get_label("键盘中断，停止流程。", "Keyboard interrupt, stopping process."))
            self._ros_signal_handler(signal.SIGINT, None) # Trigger cleanup
        except Exception as e:
            rospy.logerr(self._get_label(f"系统辨识主流程出错: {str(e)}", f"Error in main system ID process: {str(e)}"))
            import traceback
            traceback.print_exc()
        finally:
            rospy.loginfo(self._get_label("系统辨识流程结束。正在发送中立姿态。", "System identification process finished. Sending neutral pose."))
            if self.controller and self.controller.pub is not None: # Check if controller and pub were initialized
                self.controller.publish_neutral_pose()
            if not rospy.is_shutdown():
                rospy.signal_shutdown(self._get_label("流程完成", "Process completed"))
            rospy.loginfo("Exiting SystemIdentification.run()")

def parse_arguments():
    parser = argparse.ArgumentParser(description='System Identification Tool - Refactored')
    parser.add_argument('--language', type=str, default='en', choices=['cn', 'en'], help='Display language (cn: Chinese, en: English)')
    parser.add_argument('--dt', type=float, default=0.005, help='Sampling time interval (s) - Note: primarily for conceptual reference now as data has own timestamps')
    parser.add_argument('--update-rate', type=float, default=200.0, help='Control command update rate (Hz)')
    
    # Warmup (used by MotionController, but defined here for central config)
    parser.add_argument('--calib-cycles', type=int, default=2, help='Number of cycles for warmup/stabilization before calibration signal')
    parser.add_argument('--calib-amplitude', type=float, default=0.05, help='Amplitude for warmup signal (m)')
    parser.add_argument('--calib-freq', type=float, default=0.25, help='Frequency for warmup signal (Hz)')

    # Robot Hand Positions (used by MotionController)
    parser.add_argument('--x-center', type=float, default=0.36, help='Hand X center position (m)')
    parser.add_argument('--y-left', type=float, default=0.4, help='Left hand Y position (m)')
    parser.add_argument('--y-right', type=float, default=-0.4, help='Right hand Y position (m)')
    parser.add_argument('--z-center', type=float, default=0.12, help='Hand Z center position (m)')

    # Calibration (used by MotionController & SignalAnalyzer indirectly)
    parser.add_argument('--calibration-amplitude', type=float, default=0.1, help='Calibration signal amplitude (m)')
    parser.add_argument('--calibration-period', type=float, default=4.0, help='Calibration signal period (s) => Freq = 1/Period')
    parser.add_argument('--calibration-duration', type=float, default=8.0, help='Calibration phase signal duration (s)')
    parser.add_argument('--time-delay', type=float, default=0.1, help='Initial/Fallback time delay estimate (s)')

    # Frequency Sweep (used by MotionController & SignalAnalyzer)
    parser.add_argument('--sweep-duration', type=float, default=90.0, help='Total time for frequency sweep (s)')
    parser.add_argument('--freq-start', type=float, default=0.1, help='Start sweep frequency (Hz)')
    parser.add_argument('--freq-end', type=float, default=6.0, help='End sweep frequency (Hz)')
    parser.add_argument('--sweep-amplitude', type=float, default=0.1, help='Signal amplitude during sweep (m)')
    parser.add_argument('--sweep-type', type=str, default='logarithmic', choices=['logarithmic', 'linear'], help='Sweep type')

    # Visualization & Data Saving (used by Visualizer & SignalAnalyzer)
    parser.add_argument('--plots-dir', type=str, default='plots', help='Directory to save plots')
    parser.add_argument('--plot-results', action='store_true', help='Generate and save detailed result plots')
    parser.add_argument('--save-data', action='store_true', help='Save processed data to .npz file')
    parser.add_argument('--no-plots', action='store_true', help='Disable all plot generation and display')
    parser.add_argument('--show-plots', action='store_true', help='Show plots on screen (if GUI available, default is save only)')

    parser.add_argument('--debug', action='store_true', help='Enable debug logging (TODO: implement log level setting)')
    
    parsed_args = parser.parse_args()
    if not parsed_args.no_plots and not os.path.exists(parsed_args.plots_dir):
        os.makedirs(parsed_args.plots_dir)
    return parsed_args

if __name__ == "__main__":
    try:
        args = parse_arguments()
        # if args.debug: rospy.set_param('/rosout/logging_level', 'DEBUG') # Example, may need roslaunch for this
        
        system_id_app = SystemIdentification(args)
        system_id_app.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt. Program shutting down.")
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted by user (KeyboardInterrupt in main).")
    except Exception as e_main:
        rospy.logerr(f"Unhandled exception in main: {e_main}")
        import traceback
        traceback.print_exc()
    finally:
        rospy.loginfo("Main execution block finished.")