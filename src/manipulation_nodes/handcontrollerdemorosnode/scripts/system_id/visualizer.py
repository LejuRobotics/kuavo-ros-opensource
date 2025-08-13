import matplotlib.pyplot as plt
import numpy as np
import os
import rospy
from scipy.signal import find_peaks, medfilt # Added medfilt
import time # Added for timestamp in filename

class Visualizer:
    def __init__(self, args, use_chinese=False):
        self.args = args
        self.use_chinese = use_chinese
        # # Setting matplotlib for Chinese display (can be enabled if needed)
        # try:
        #     plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'sans-serif']
        #     plt.rcParams['axes.unicode_minus'] = False
        #     test_fig = plt.figure()
        #     plt.title('测试') # Test
        #     plt.close(test_fig)
        #     # self.use_chinese = True # Enable if fonts are set up
        #     rospy.loginfo("Visualizer: Matplotlib configured for Chinese display.")
        # except Exception as e:
        #     # self.use_chinese = False
        #     rospy.logwarn(f"Visualizer: Failed to set Chinese fonts: {e}")
        #     rospy.loginfo("Visualizer: Using English labels for display.")

        if not os.path.exists(self.args.plots_dir):
            os.makedirs(self.args.plots_dir)
            rospy.loginfo(f"Visualizer: Created plots directory: {self.args.plots_dir}")

    def _get_label(self, chinese_text, english_text):
        return chinese_text if self.use_chinese else english_text

    def _plot_calibration_results(self, input_stable, output_stable, aligned_input_compensated, 
                                aligned_output_compensated, aligned_input, aligned_output, 
                                time_delay, actual_sample_rate, static_gain, bias, r_squared,
                                aligned_time):
        """绘制校准结果图"""
        plt.figure(figsize=(12, 10))
        
        # 时域图
        plt.subplot(2, 2, 1)
        plt.plot(input_stable, label=self._get_label('输入', 'Input'))
        plt.plot(output_stable, label=self._get_label('输出', 'Output'))
        plt.xlabel(self._get_label('样本', 'Sample'))
        plt.ylabel(self._get_label('位置 (m)', 'Position (m)'))
        plt.title(self._get_label('校准数据（第一个周期后）', 'Calibration Data (After First Cycle)'))
        plt.legend()
        plt.grid(True)
        
        # 带延迟补偿的对齐数据图
        plt.subplot(2, 2, 2)
        plt.plot(aligned_input_compensated, label=self._get_label('输入（延迟补偿）', 'Input (Delay Compensated)'))
        plt.plot(aligned_output_compensated, label=self._get_label('输出', 'Output'))
        plt.xlabel(self._get_label('样本', 'Sample'))
        plt.ylabel(self._get_label('位置 (m)', 'Position (m)'))
        plt.title(self._get_label('带延迟补偿的对齐数据', 'Aligned Data with Delay Compensation'))
        plt.legend()
        plt.grid(True)
        
        # 回归图（带延迟补偿）
        plt.subplot(2, 2, 3)
        plt.scatter(aligned_input_compensated, aligned_output_compensated, alpha=0.5, s=3)
        input_range = np.linspace(min(aligned_input_compensated), max(aligned_input_compensated), 100)
        plt.plot(input_range, static_gain * input_range + bias, 'r-', linewidth=2)
        plt.xlabel(self._get_label('输入位置 (m)', 'Input Position (m)'))
        plt.ylabel(self._get_label('输出位置 (m)', 'Output Position (m)'))
        plt.title(self._get_label(f'静态关系（延迟补偿）\ny = {static_gain:.3f}x + {bias:.3f}\nR² = {r_squared:.3f}',
                               f'Static Relationship (delay comp.)\ny = {static_gain:.3f}x + {bias:.3f}\nR² = {r_squared:.3f}'))
        plt.grid(True)
        
        # 用于时间延迟可视化的互相关
        plt.subplot(2, 2, 4)
        if len(aligned_input) > 10 and len(aligned_output) > 10:
            # 归一化信号
            input_norm = aligned_input - np.mean(aligned_input)
            output_norm = aligned_output - np.mean(aligned_output)
            
            # 计算互相关
            from scipy import signal as sig # Local import for safety
            xcorr = sig.correlate(output_norm, input_norm, mode='same')
            max_lag = min(100, len(aligned_input)//4)
            lags = np.arange(-max_lag, max_lag+1)
            lag_time = lags / actual_sample_rate
            
            # 获取与lags长度匹配的xcorr的中心部分
            center = len(xcorr) // 2
            xcorr_slice = xcorr[center-max_lag:center+max_lag+1]
            
            # 确保长度匹配
            if len(lag_time) == len(xcorr_slice):
                plt.plot(lag_time, xcorr_slice)
                plt.axvline(x=time_delay, color='r', linestyle='--', 
                          label=self._get_label(f'时间延迟: {time_delay:.4f}s', f'Time Delay: {time_delay:.4f}s'))
                plt.xlabel(self._get_label('时间滞后 (s)', 'Time Lag (s)'))
                plt.ylabel(self._get_label('相关性', 'Correlation'))
                plt.title(self._get_label('用于时间延迟的互相关', 'Cross-correlation for Time Delay'))
                plt.grid(True)
                plt.legend()
        
        plt.suptitle(self._get_label('带时间延迟补偿的校准结果', 'Calibration Results with Time Delay Compensation'), fontsize=16)
        plt.tight_layout(rect=[0, 0, 1, 0.95])
        
        save_path = os.path.join(self.args.plots_dir, 'calibration_results.png')
        plt.savefig(save_path)
        rospy.loginfo(self._get_label(f"校准图保存到 '{save_path}'", f"Calibration plot saved to '{save_path}'"))
        
        # 时域中时间延迟的额外可视化
        plt.figure(figsize=(12, 6))
        
        # 获取数据的代表性部分
        if len(aligned_time) > self.args.update_rate * 2:
            # 如果可用，使用2秒的数据
            plot_len = min(int(self.args.update_rate * 2), len(aligned_input))
            start_idx = len(aligned_input) // 3  # 从1/3处开始
            end_idx = start_idx + plot_len
            
            # 绘制输入和输出信号
            plt.plot(aligned_time[start_idx:end_idx], aligned_input[start_idx:end_idx], 'b-', 
                    label=self._get_label('输入', 'Input'))
            plt.plot(aligned_time[start_idx:end_idx], aligned_output[start_idx:end_idx], 'r-', 
                    label=self._get_label('输出', 'Output'))
            
            # 标记时间延迟
            if time_delay > 0:
                # 在此部分中找到峰值
                peaks, _ = find_peaks(aligned_input[start_idx:end_idx], height=0.01, distance=20)
                
                for peak_idx in peaks[:2]:  # 使用前两个峰值
                    peak_time = aligned_time[start_idx + peak_idx]
                    peak_value = aligned_input[start_idx + peak_idx]
                    
                    # 绘制箭头来显示延迟
                    plt.annotate('', 
                                xy=(peak_time + time_delay, peak_value),
                                xytext=(peak_time, peak_value),
                                arrowprops=dict(arrowstyle='<->', color='g', lw=2))
                    
                    # 添加文本标签
                    plt.text(peak_time + time_delay/2, peak_value + 0.01, 
                            f'{time_delay*1000:.1f} ms', 
                            color='g', ha='center')
        
        plt.xlabel(self._get_label('时间 (s)', 'Time (s)'))
        plt.ylabel(self._get_label('位置 (m)', 'Position (m)'))
        plt.title(self._get_label(f'时间延迟可视化: {time_delay*1000:.1f} ms', 
                             f'Time Delay Visualization: {time_delay*1000:.1f} ms'))
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        save_path_delay = os.path.join(self.args.plots_dir, 'calibration_time_delay.png')
        plt.savefig(save_path_delay)
        rospy.loginfo(self._get_label(f"时间延迟可视化保存到 '{save_path_delay}'", f"Time delay visualization saved to '{save_path_delay}'"))

        if self.args.show_plots:
            plt.show()
        else:
            plt.close('all')


    def visualize_time_alignment(self, input_time, input_data, output_data_orig, # Renamed output_data to output_data_orig
                               output_time_orig, # Added original output time
                               aligned_time, aligned_input, aligned_output,
                               time_delay, static_gain, bias):
        """
        可视化时间对齐过程
        """
        rospy.loginfo(self._get_label(
            f"可视化数据长度 - 输入: {len(input_data)}点, 输出: {len(output_data_orig)}点",
            f"Visualization data length - Input: {len(input_data)} points, Output: {len(output_data_orig)} points"
        ))

        if len(input_time) != len(input_data):
            rospy.logwarn(self._get_label(
                f"输入时间({len(input_time)})和输入数据({len(input_data)})长度不匹配，将进行截断",
                f"Input time({len(input_time)}) and input data({len(input_data)}) lengths don't match, will truncate"
            ))
            min_len = min(len(input_time), len(input_data))
            input_time = input_time[:min_len]
            input_data = input_data[:min_len]

        # For the "Original Signals" plot, we need output_data resampled to input_time if lengths differ
        output_data_for_plot1 = output_data_orig
        if len(output_data_orig) != len(input_time):
            rospy.logwarn(self._get_label(
                f"原始输出数据({len(output_data_orig)})与输入时间({len(input_time)})长度不匹配，将为绘图重新采样",
                f"Original output data({len(output_data_orig)}) and input time({len(input_time)}) lengths don't match, will resample for plot"
            ))
            if len(output_data_orig) < 2 or len(output_time_orig) < 2:
                rospy.logerr(self._get_label("输出数据点太少，无法重新采样可视化", "Too few output data points to resample for visualization"))
                # Fallback: plot what we have, or skip
                if len(output_data_orig) > 0 and len(output_time_orig) > 0:
                     output_data_for_plot1 = output_data_orig # Will plot against its own time
                else:
                    return # Cannot proceed
            else:
                try:
                    from scipy.interpolate import interp1d
                    # Ensure output_time_orig and output_data_orig have same length before interp1d
                    min_len_oo = min(len(output_time_orig), len(output_data_orig))
                    if len(output_time_orig) != len(output_data_orig):
                        rospy.logwarn(f"visualize_time_alignment: Mismatch for interp1d input: len(output_time_orig)={len(output_time_orig)}, len(output_data_orig)={len(output_data_orig)}. Truncating.")
                    output_time_interp = output_time_orig[:min_len_oo]
                    output_data_interp = output_data_orig[:min_len_oo]

                    if len(output_time_interp) < 2: # Check again after potential truncation
                        rospy.logerr("visualize_time_alignment: Not enough points for interpolation after truncation.")
                        if len(output_data_orig) > 0 and len(output_time_orig) > 0:
                            output_data_for_plot1 = output_data_orig 
                        else:
                            return
                    else:
                        output_interp_orig = interp1d(output_time_interp, output_data_interp, kind='linear',
                                                      bounds_error=False, fill_value='extrapolate')
                        output_data_for_plot1 = output_interp_orig(input_time)
                except Exception as e:
                    rospy.logerr(self._get_label(f"重新采样原始输出数据时出错: {str(e)}", f"Error resampling original output data: {str(e)}"))
                    return


        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 1, 1)
        plt.plot(input_time, input_data, 'b-', label=self._get_label("输入信号", "Input Signal"))
        if len(output_data_for_plot1) == len(input_time):
             plt.plot(input_time, output_data_for_plot1, 'r-', label=self._get_label("输出信号 (原始, 重采样)", "Output Signal (Original, Resampled)"))
        elif len(output_data_orig) > 0 and len(output_time_orig) > 0 : # Plot against its own time if resampling failed but data exists
             plt.plot(output_time_orig, output_data_orig, 'r-', label=self._get_label("输出信号 (原始)", "Output Signal (Original)"))

        plt.title(self._get_label("原始信号 (可能已重采样用于比较)", "Original Signals (May be Resampled for Comparison)"))
        plt.xlabel(self._get_label("时间 (秒)", "Time (s)"))
        plt.ylabel(self._get_label("幅度", "Amplitude"))
        plt.grid(True)
        plt.legend()
        
        min_len_aligned = min(len(aligned_time), len(aligned_input), len(aligned_output))
        aligned_time_plot = aligned_time[:min_len_aligned]
        aligned_input_plot = aligned_input[:min_len_aligned]
        aligned_output_plot = aligned_output[:min_len_aligned]
        
        compensated_input = static_gain * aligned_input_plot + bias
        
        plt.subplot(2, 1, 2)
        plt.plot(aligned_time_plot, aligned_input_plot, 'b-', label=self._get_label("对齐输入", "Aligned Input"))
        plt.plot(aligned_time_plot, compensated_input, 'g-', label=self._get_label("补偿后输入 (增益/偏置应用到对齐输入)", "Compensated Input (Gain/Bias on Aligned Input)"))
        plt.plot(aligned_time_plot, aligned_output_plot, 'r-', label=self._get_label("对齐输出 (已延迟补偿)", "Aligned Output (Delay Compensated)"))
        plt.title(self._get_label(f"对齐与补偿后信号 (时间延迟={time_delay:.4f}秒, 增益={static_gain:.4f}, 偏置={bias:.4f})",
                               f"Aligned & Compensated Signals (Delay={time_delay:.4f}s, Gain={static_gain:.4f}, Bias={bias:.4f})"))
        plt.xlabel(self._get_label("时间 (秒)", "Time (s)"))
        plt.ylabel(self._get_label("幅度", "Amplitude"))
        plt.grid(True)
        plt.legend()
        
        if len(compensated_input) == len(aligned_output_plot) and len(compensated_input) > 0:
            mse = np.mean((compensated_input - aligned_output_plot) ** 2)
            mae = np.mean(np.abs(compensated_input - aligned_output_plot))
            rmse = np.sqrt(mse)
            plt.figtext(0.02, 0.02, self._get_label(
                f"误差统计 (补偿输入 vs 对齐输出): RMSE={rmse:.4f}, MAE={mae:.4f}", 
                f"Error metrics (Compensated Input vs Aligned Output): RMSE={rmse:.4f}, MAE={mae:.4f}"),
                fontsize=9)
        
        plt.tight_layout(rect=[0, 0.03, 1, 0.97]) # Adjusted rect for figtext
        
        save_path = os.path.join(self.args.plots_dir, "time_alignment.png")
        plt.savefig(save_path)
        rospy.loginfo(self._get_label(f"时间对齐图已保存至: {save_path}", f"Time alignment plot saved to: {save_path}"))
        
        if self.args.show_plots:
            plt.show()
        else:
            plt.close()

    def visualize_frequency_response(self, frequencies, magnitude, phase, coherence):
        plt.figure(figsize=(12, 12))
        
        plt.subplot(3, 1, 1)
        plt.semilogx(frequencies, 20 * np.log10(np.maximum(magnitude, 1e-10)), 'b-') # Added maximum for safety
        plt.title(self._get_label("幅值响应", "Magnitude Response"))
        plt.xlabel(self._get_label("频率 (Hz)", "Frequency (Hz)"))
        plt.ylabel(self._get_label("幅值 (dB)", "Magnitude (dB)"))
        plt.grid(True, which="both")
        
        plt.subplot(3, 1, 2)
        plt.semilogx(frequencies, phase, 'g-')
        plt.title(self._get_label("相位响应", "Phase Response"))
        plt.xlabel(self._get_label("频率 (Hz)", "Frequency (Hz)"))
        plt.ylabel(self._get_label("相位 (度)", "Phase (degrees)"))
        plt.grid(True, which="both")
        
        plt.subplot(3, 1, 3)
        plt.semilogx(frequencies, coherence, 'r-')
        plt.title(self._get_label("相干性", "Coherence"))
        plt.xlabel(self._get_label("频率 (Hz)", "Frequency (Hz)"))
        plt.ylabel(self._get_label("相干性", "Coherence"))
        plt.grid(True, which="both")
        
        plt.tight_layout()
        
        save_path = os.path.join(self.args.plots_dir, "frequency_response.png")
        plt.savefig(save_path)
        rospy.loginfo(self._get_label(f"频率响应图已保存至: {save_path}", f"Frequency response plot saved to: {save_path}"))
        
        if self.args.show_plots:
            plt.show()
        else:
            plt.close()

    def visualize_step_response(self, time_axis, step_response, settling_time=None, steady_state=None): # Renamed time to time_axis
        plt.figure(figsize=(10, 6))
        plt.plot(time_axis, step_response, 'b-', label=self._get_label("阶跃响应", "Step Response"))
        
        if settling_time is not None and steady_state is not None:
            plt.axhline(y=steady_state, color='r', linestyle='--', 
                       label=self._get_label(f"稳态值 = {steady_state:.4f}", f"Steady State = {steady_state:.4f}"))
            plt.axvline(x=settling_time, color='g', linestyle='--', 
                       label=self._get_label(f"稳定时间 = {settling_time:.4f}秒", f"Settling Time = {settling_time:.4f}s"))
            tolerance = 0.05
            plt.axhspan(steady_state - tolerance * steady_state, steady_state + tolerance * steady_state, 
                       alpha=0.2, color='r', label=self._get_label(f"±{tolerance*100}% 稳定区间", f"±{tolerance*100}% Tolerance Band"))
        
        plt.title(self._get_label("系统阶跃响应", "System Step Response"))
        plt.xlabel(self._get_label("时间 (秒)", "Time (s)"))
        plt.ylabel(self._get_label("幅度", "Amplitude"))
        plt.grid(True)
        plt.legend()
        
        save_path = os.path.join(self.args.plots_dir, "step_response.png")
        plt.savefig(save_path)
        rospy.loginfo(self._get_label(f"阶跃响应图已保存至: {save_path}", f"Step response plot saved to: {save_path}"))
        
        if self.args.show_plots:
            plt.show()
        else:
            plt.close()

    def _plot_frequency_response(self, input_array, normalized_output, frequencies, 
                               transfer_function, coherence, bandwidth, peak_freq,
                               sampling_rate, mean_coherence, phase, phase_delay, time_delay_sys): # Added time_delay_sys
        """绘制频率响应分析结果"""
        plt.figure(figsize=(15, 16))
        
        max_sweep_freq = self.args.freq_end
        
        plt.subplot(4, 1, 1)
        time_axis = np.arange(len(input_array)) / sampling_rate
        plt.plot(time_axis, input_array, 'b-', label=self._get_label('输入命令', 'Input Command'))
        plt.plot(time_axis, normalized_output, 'r-', label=self._get_label('输出(已去除偏置)', 'Output (bias removed)'))
        plt.xlabel(self._get_label('时间 (s)', 'Time (s)'))
        plt.ylabel(self._get_label('位置 (m)', 'Position (m)'))
        plt.title(self._get_label('时域信号（带已知时间延迟补偿）', 'Time Domain Signals (with Known Time Delay Compensation)'))
        plt.legend()
        plt.grid(True)
        
        plt.subplot(4, 1, 2)
        coherence_limited_freq = None
        coherence_threshold = 0.7
        if frequencies is not None and len(frequencies) > 0 and coherence is not None and len(coherence) > 0:
            coh_drops = np.where(coherence < coherence_threshold)[0]
            if len(coh_drops) > 0:
                first_drop_idx = coh_drops[0]
                if first_drop_idx > 3:
                    consecutive_drops = 0
                    for i in range(first_drop_idx, min(first_drop_idx + 5, len(coherence))):
                        if i < len(coherence) and coherence[i] < coherence_threshold:
                            consecutive_drops += 1
                    if consecutive_drops >= 3: # Adjusted from 0 to 3
                        coherence_limited_freq = frequencies[first_drop_idx]
        
        magnitudes_db = 20 * np.log10(np.maximum(transfer_function, 1e-10)) if transfer_function is not None else np.array([])
        
        if len(magnitudes_db) > 0:
            peak_idx_mag = np.argmax(magnitudes_db) # Renamed to avoid conflict
            peak_magnitude_db = magnitudes_db[peak_idx_mag]
            threshold_db = peak_magnitude_db - 3.0
        else: # Handle empty magnitudes_db
            peak_magnitude_db = 0 
            threshold_db = -3.0

        # Fallback for peak_freq if not properly determined
        current_peak_freq = peak_freq if peak_freq is not None and peak_freq > 0 else (frequencies[peak_idx_mag] if len(magnitudes_db) > 0 else 0.1)


        if transfer_function is not None and len(transfer_function) > 0 :
             if mean_coherence < 0.3:
                coh_threshold_display = 0.5
                reliable_mask = coherence > coh_threshold_display
                plt.semilogx(frequencies[reliable_mask], 20 * np.log10(np.maximum(transfer_function[reliable_mask], 1e-10)), 'g-', 
                            linewidth=2, label=self._get_label(f'可靠 (相干性>{coh_threshold_display})', f'Reliable (coherence>{coh_threshold_display})'))
                plt.semilogx(frequencies[~reliable_mask], 20 * np.log10(np.maximum(transfer_function[~reliable_mask], 1e-10)), 'r-', 
                            alpha=0.7, label=self._get_label(f'不可靠 (相干性<{coh_threshold_display})', f'Unreliable (coherence<{coh_threshold_display})'))
             else:
                plt.semilogx(frequencies, magnitudes_db, 'b-')
        
        if bandwidth is not None:
            plt.axvline(x=bandwidth, color='r', linestyle='--', 
                       label=self._get_label(f'最终带宽: {bandwidth:.2f} Hz', f'Final Bandwidth: {bandwidth:.2f} Hz'))
        
        plt.axvline(x=max_sweep_freq, color='purple', linestyle='-.', 
                   label=self._get_label(f'最大扫频频率: {max_sweep_freq:.2f} Hz', 
                                       f'Max Sweep Frequency: {max_sweep_freq:.2f} Hz'))
        
        if coherence_limited_freq is not None:
            plt.axvline(x=coherence_limited_freq, color='orange', linestyle='-.', 
                       label=self._get_label(f'相干性限制: {coherence_limited_freq:.2f} Hz', 
                                         f'Coherence limitation: {coherence_limited_freq:.2f} Hz'))
        
        if peak_freq is not None :
            plt.axvline(x=peak_freq, color='g', linestyle='--', 
                      label=self._get_label(f'峰值: {peak_freq:.2f} Hz', f'Peak: {peak_freq:.2f} Hz'))
        
        plt.axhline(y=threshold_db, color='b', linestyle=':', 
                   label=self._get_label('-3dB截止线', '-3dB cutoff line'))
        
        plt.xlabel(self._get_label('频率 (Hz)', 'Frequency (Hz)'))
        plt.ylabel(self._get_label('幅度 (dB)', 'Magnitude (dB)'))
        plt.title(self._get_label(f'频率响应（时间延迟: {time_delay_sys:.4f}s）', # Used passed time_delay_sys
                               f'Frequency Response (Time Delay: {time_delay_sys:.4f}s)'))
        
        if len(magnitudes_db) > 0:
            margin = 10
            min_db_plot = np.min(magnitudes_db)
            plt.ylim(min_db_plot - margin, peak_magnitude_db + margin)
        if frequencies is not None and len(frequencies) > 0:
            # Ensure frequencies[0] is positive for log scale
            min_freq_plot = frequencies[0]
            if min_freq_plot <= 0:
                # Find the first positive frequency or use a small default if all are non-positive
                positive_freqs = frequencies[frequencies > 0]
                if len(positive_freqs) > 0:
                    min_freq_plot = positive_freqs[0]
                else:
                    min_freq_plot = 0.01 # Default small positive value
            plt.xlim(min_freq_plot, max_sweep_freq * 1.2)
        
        plt.legend(loc='best')
        plt.grid(True, which='both')
        
        plt.subplot(4, 1, 3)
        if phase is not None and frequencies is not None and len(phase) == len(frequencies):
            plt.semilogx(frequencies, phase, 'g-', linewidth=1.5)
            
            if peak_freq is not None:
                peak_idx_phase = np.argmin(np.abs(frequencies - peak_freq)) # Renamed
                peak_phase_val = phase[peak_idx_phase] if peak_idx_phase < len(phase) else 0 # Renamed
                plt.plot(peak_freq, peak_phase_val, 'go', markersize=8)
                plt.text(peak_freq, peak_phase_val + 20, f"{peak_phase_val:.1f}°", ha='center')

            if bandwidth is not None:
                bw_idx_phase = np.argmin(np.abs(frequencies - bandwidth)) # Renamed
                bw_phase_val = phase[bw_idx_phase] if bw_idx_phase < len(phase) else 0 # Renamed
                plt.plot(bandwidth, bw_phase_val, 'ro', markersize=8)
                plt.text(bandwidth, bw_phase_val + 20, f"{bw_phase_val:.1f}°", ha='center')

            if peak_freq is not None:
                plt.axvline(x=peak_freq, color='g', linestyle='--', 
                           label=self._get_label(f'峰值频率: {peak_freq:.2f} Hz', f'Peak frequency: {peak_freq:.2f} Hz'))
            if bandwidth is not None:        
                plt.axvline(x=bandwidth, color='r', linestyle='--', 
                           label=self._get_label(f'带宽频率: {bandwidth:.2f} Hz', f'Bandwidth: {bandwidth:.2f} Hz'))
            
            plt.axvline(x=max_sweep_freq, color='purple', linestyle='-.', 
                       label=self._get_label(f'最大扫频频率: {max_sweep_freq:.2f} Hz', 
                                           f'Max Sweep Frequency: {max_sweep_freq:.2f} Hz'))

            if peak_freq is not None and bandwidth is not None and peak_freq > 0 and bandwidth > 0:
                 peak_delay_ms = -phase[np.argmin(np.abs(frequencies - peak_freq))] / (360 * peak_freq) * 1000 if peak_freq > 0 else 0
                 bw_delay_ms = -phase[np.argmin(np.abs(frequencies - bandwidth))] / (360 * bandwidth) * 1000 if bandwidth > 0 else 0
                 delay_text_val = self._get_label( # Renamed
                    f"峰频相位延迟: {peak_delay_ms:.2f} ms, 带宽相位延迟: {bw_delay_ms:.2f} ms",
                    f"Phase delay @ peak: {peak_delay_ms:.2f} ms, @ bandwidth: {bw_delay_ms:.2f} ms"
                 )
                 plt.figtext(0.5, 0.46, delay_text_val, ha='center', bbox={'facecolor':'white', 'alpha':0.8, 'pad':5})

        plt.title(self._get_label("相位响应", "Phase Response"))
        plt.xlabel(self._get_label("频率 (Hz)", "Frequency (Hz)"))
        plt.ylabel(self._get_label("相位 (度)", "Phase (degrees)"))
        plt.grid(True, which="both")
        plt.legend(loc='best')
        if frequencies is not None and len(frequencies) > 0:
            min_freq_plot = frequencies[0]
            if min_freq_plot <= 0:
                positive_freqs = frequencies[frequencies > 0]
                if len(positive_freqs) > 0:
                    min_freq_plot = positive_freqs[0]
                else:
                    min_freq_plot = 0.01 
            plt.xlim(min_freq_plot, max_sweep_freq * 1.2)
        
        ax2 = plt.gca().twinx()
        if phase_delay is not None and frequencies is not None and len(phase_delay) == len(frequencies):
            phase_delay_ms_val = phase_delay * 1000 # Renamed
            phase_delay_ms_val[~np.isfinite(phase_delay_ms_val)] = np.nan
            
            valid_indices = np.where(np.isfinite(phase_delay_ms_val) & (np.abs(phase_delay_ms_val) < 1000))
            if len(valid_indices[0]) > 0:
                ax2.semilogx(frequencies[valid_indices], phase_delay_ms_val[valid_indices], 'b--', alpha=0.7, 
                           label=self._get_label('相位延迟 (ms)', 'Phase Delay (ms)'))
                
                if coherence is not None:
                    reliable_mask_delay = (coherence >= coherence_threshold) & (frequencies >= 0.1) & np.isfinite(phase_delay_ms_val) # Renamed
                    if np.any(reliable_mask_delay):
                        ax2.semilogx(frequencies[reliable_mask_delay], 
                                   phase_delay_ms_val[reliable_mask_delay], 
                                   'go', markersize=4, alpha=0.7,
                                   label=self._get_label('可靠相位延迟点', 'Reliable delay points'))
                
                ax2.set_ylabel(self._get_label('相位延迟 (ms)', 'Phase Delay (ms)'), color='b')
                ax2.tick_params(axis='y', labelcolor='b')
                
                max_delay_plot = np.nanmax(phase_delay_ms_val[valid_indices]) # Renamed
                min_delay_plot = np.nanmin(phase_delay_ms_val[valid_indices]) # Renamed
                range_delay_plot = max_delay_plot - min_delay_plot # Renamed
                margin_delay = max(range_delay_plot * 0.1, 5) # Renamed
                ax2.set_ylim(min_delay_plot - margin_delay, max_delay_plot + margin_delay)
                ax2.legend(loc='upper right')

        plt.subplot(4, 1, 4)
        if coherence is not None and frequencies is not None and len(coherence) == len(frequencies):
            low_coh_threshold = 0.4
            plt.fill_between(frequencies, 0, coherence, where=coherence > coherence_threshold, color='green', alpha=0.3, label=self._get_label('良好', 'Good'))
            plt.fill_between(frequencies, 0, coherence, where=(coherence <= coherence_threshold) & (coherence > low_coh_threshold), color='yellow', alpha=0.3, label=self._get_label('中等', 'Medium'))
            plt.fill_between(frequencies, 0, coherence, where=coherence <= low_coh_threshold, color='red', alpha=0.3, label=self._get_label('低', 'Low'))
            
            plt.semilogx(frequencies, coherence, 'k-', linewidth=1.5)
            plt.axhline(y=coherence_threshold, color='r', linestyle=':', label=f'{self._get_label("阈值", "Thres.")}: {coherence_threshold}')
            plt.axhline(y=low_coh_threshold, color='orange', linestyle=':', label=f'{self._get_label("低阈值", "Low Thres.")}: {low_coh_threshold}')

            if bandwidth is not None and bandwidth < (frequencies[-1] if len(frequencies) > 0 else bandwidth) :
                plt.axvline(x=bandwidth, color='r', linestyle='--', label=self._get_label(f'带宽: {bandwidth:.2f} Hz', f'Bandwidth: {bandwidth:.2f} Hz'))
            
            plt.axvline(x=max_sweep_freq, color='purple', linestyle='-.', label=self._get_label(f'最大扫频: {max_sweep_freq:.2f} Hz', f'Max Sweep: {max_sweep_freq:.2f} Hz'))
            
            if coherence_limited_freq is not None:
                plt.axvline(x=coherence_limited_freq, color='orange', linestyle='-.', label=self._get_label(f'相干性降低: {coherence_limited_freq:.2f} Hz', f'Coherence Drop: {coherence_limited_freq:.2f} Hz'))
                coh_idx_plot = np.argmin(np.abs(frequencies - coherence_limited_freq)) # Renamed
                if coh_idx_plot < len(coherence):
                    coh_val_plot = coherence[coh_idx_plot] # Renamed
                    plt.plot(coherence_limited_freq, coh_val_plot, 'o', color='orange', markersize=8)
                    plt.text(coherence_limited_freq, coh_val_plot + 0.05, f"{coh_val_plot:.2f}", ha='center')

            if peak_freq is not None:
                plt.axvline(x=peak_freq, color='g', linestyle='--', label=self._get_label(f'峰值: {peak_freq:.2f} Hz', f'Peak: {peak_freq:.2f} Hz'))
                peak_idx_coh = np.argmin(np.abs(frequencies - peak_freq)) # Renamed
                bw_idx_coh = np.argmin(np.abs(frequencies - (bandwidth if bandwidth is not None else peak_freq))) # Renamed
                
                peak_coh_val = coherence[peak_idx_coh] if peak_idx_coh < len(coherence) else 0 # Renamed
                bw_coh_val = coherence[bw_idx_coh] if bw_idx_coh < len(coherence) else 0 # Renamed
                
                plt.plot(peak_freq, peak_coh_val, 'go', markersize=8)
                plt.text(peak_freq, peak_coh_val + 0.05, f"{peak_coh_val:.2f}", ha='center')
                if bandwidth is not None:
                    plt.plot(bandwidth, bw_coh_val, 'ro', markersize=8)
                    plt.text(bandwidth, bw_coh_val + 0.05, f"{bw_coh_val:.2f}", ha='center')
        
        plt.xlabel(self._get_label('频率 (Hz)', 'Frequency (Hz)'))
        plt.ylabel(self._get_label('相干性', 'Coherence'))
        plt.title(self._get_label(f'相干函数（平均值: {mean_coherence:.4f}）', 
                              f'Coherence Function (Mean: {mean_coherence:.4f})'))
        plt.ylim(0, 1.05)
        plt.grid(True, which='both')
        plt.legend(loc='best')
        if frequencies is not None and len(frequencies) > 0:
            min_freq_plot = frequencies[0]
            if min_freq_plot <= 0:
                positive_freqs = frequencies[frequencies > 0]
                if len(positive_freqs) > 0:
                    min_freq_plot = positive_freqs[0]
                else:
                    min_freq_plot = 0.01 
            plt.xlim(min_freq_plot, max_sweep_freq * 1.2)
        
        sweep_info = f', {self._get_label("扫频范围", "Sweep Range")}: {self.args.freq_start:.2f} - {max_sweep_freq:.2f} Hz'
        summary_text_val = f'{self._get_label("系统频率响应分析", "System Freq. Response Analysis")} ({self._get_label("带宽", "BW")}: {bandwidth if bandwidth is not None else "N/A":.2f} Hz, {self._get_label("均相干", "Avg.Coh")}: {mean_coherence:.2f}{sweep_info})' # Renamed
        if coherence_limited_freq is not None and bandwidth is not None and coherence_limited_freq <= bandwidth:
            summary_text_val += f', {self._get_label("相干性限制于", "Coh. limited at")} {coherence_limited_freq:.2f} Hz'
            
        plt.suptitle(summary_text_val, fontsize=16)
            
        if mean_coherence < 0.3:
            reliability_msg_val = self._get_label( # Renamed
                "⚠️ 警告: 平均相干性低，结果可能不可靠",
                "⚠️ Warning: Low average coherence, results may be unreliable")
            plt.figtext(0.5, 0.01, reliability_msg_val, ha='center', color='red', fontsize=12)
        
        plt.tight_layout(rect=[0, 0.02, 1, 0.95])
        
        save_path_freq = os.path.join(self.args.plots_dir, 'frequency_response_analysis.png') # Renamed
        plt.savefig(save_path_freq)
        rospy.loginfo(self._get_label("频率响应结果保存到 ", "Frequency response results saved to ") + save_path_freq)
        
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        save_path_ts = os.path.join(self.args.plots_dir, f'frequency_response_analysis_{timestamp}.png') # Renamed
        plt.savefig(save_path_ts)
        rospy.loginfo(self._get_label("带时间戳的频率响应结果保存到 ", "Timestamped frequency response results saved to ") + save_path_ts)

        if self.args.show_plots:
            try:
                plt.show()
            except Exception as e:
                rospy.logwarn(f"无法显示图: {e}。 结果已保存到文件。")
        else:
            plt.close('all') # Close all figures if not showing


    def plot_results(self, data_matrix, time_delay, static_gain, bias, folder=None):
        if self.args.no_plots:
            return
        
        t = data_matrix[:, 0]
        input_signal = data_matrix[:, 1]
        output_signal = data_matrix[:, 2]
        
        # Create time-delayed input signal
        # Ensure t is monotonically increasing for interpolation
        sort_indices = np.argsort(t)
        t_sorted = t[sort_indices]
        input_signal_sorted = input_signal[sort_indices]
        
        delayed_input = np.interp(t_sorted - time_delay, t_sorted, input_signal_sorted, left=input_signal_sorted[0], right=input_signal_sorted[-1])
        # Revert to original order if necessary, though for plotting t vs delayed_input, t_sorted should be used.
        # For simplicity, we'll plot against t_sorted.
        
        corrected_input = static_gain * delayed_input + bias
        
        fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        
        axes[0].plot(t_sorted, input_signal_sorted, 'b-', label=self._get_label('输入信号', 'Input Signal'))
        # Output signal also needs to be sorted by t for correct plotting if t wasn't already sorted
        output_signal_sorted_by_t = output_signal[sort_indices]
        axes[0].plot(t_sorted, output_signal_sorted_by_t, 'r-', label=self._get_label('输出信号', 'Output Signal'))
        axes[0].set_ylabel(self._get_label('位置 (m)', 'Position (m)'))
        axes[0].set_title(self._get_label('原始信号对比', 'Original Signals Comparison'))
        axes[0].grid(True)
        axes[0].legend()
        
        axes[1].plot(t_sorted, corrected_input, 'g-', label=self._get_label('校正后的输入信号', 'Corrected Input Signal'))
        axes[1].plot(t_sorted, output_signal_sorted_by_t, 'r-', label=self._get_label('输出信号', 'Output Signal'))
        axes[1].set_xlabel(self._get_label('时间 (s)', 'Time (s)'))
        axes[1].set_ylabel(self._get_label('位置 (m)', 'Position (m)'))
        axes[1].set_title(self._get_label(
            f'校正信号对比 (时延={time_delay:.3f}s, 增益={static_gain:.3f}, 偏置={bias:.3f})',
            f'Corrected Signals Comparison (Delay={time_delay:.3f}s, Gain={static_gain:.3f}, Bias={bias:.3f})'
        ))
        axes[1].grid(True)
        axes[1].legend()
        
        plt.tight_layout()
        
        if folder is not None:
            # Ensure folder is based on self.args.plots_dir if it's a relative path
            target_folder = folder
            if not os.path.isabs(folder):
                target_folder = os.path.join(self.args.plots_dir, folder)
            
            if not os.path.exists(target_folder) and target_folder != self.args.plots_dir : # Avoid re-creating plots_dir if folder is empty
                 os.makedirs(target_folder)
            
            filename = os.path.join(target_folder, 'system_identification_results.png')

        else: # Default save to plots_dir
            filename = os.path.join(self.args.plots_dir, 'system_identification_results.png')

        plt.savefig(filename)
        rospy.loginfo(self._get_label(f"结果图已保存至 {filename}", f"Results plot saved to {filename}"))
        
        if self.args.show_plots:
            try:
                plt.show()
            except Exception as e:
                rospy.logwarn(f"无法显示图像: {e}")
        else:
            plt.close()

    def visualize_alignment_verification(self, input_time, input_data, output_time, output_data, 
                                     aligned_input, aligned_output, time_delay, save_dir_arg=None): # Renamed save_dir to save_dir_arg
        """
        Visualizes data alignment verification.
        Uses self.args.plots_dir if save_dir_arg is None.
        """
        plt.figure(figsize=(12, 8))
        
        plt.subplot(211)
        plt.plot(input_time, input_data, 'b-', label=self._get_label('原始输入', 'Original Input'))
        plt.plot(output_time, output_data, 'r-', label=self._get_label('原始输出', 'Original Output'))
        plt.grid(True)
        plt.legend()
        plt.title(self._get_label('原始数据', 'Original Data'))
        plt.xlabel(self._get_label('时间 (秒)', 'Time (s)'))
        plt.ylabel(self._get_label('幅值', 'Amplitude'))
        
        # For aligned_input and aligned_output, they should share a common time axis.
        # Assuming the input_time for calibration phase is the common axis after alignment for this plot.
        # Or, if aligned_input/output have their own time axis, it should be passed.
        # For now, let's assume they are plotted against a common time axis, likely derived from input_time.
        # Let's make this explicit: plot against the first N points of input_time, where N is min length.
        min_len = min(len(input_time), len(aligned_input), len(aligned_output))

        plt.subplot(212)
        if min_len > 0:
            plt.plot(input_time[:min_len], aligned_input[:min_len], 'b-', label=self._get_label('对齐后输入', 'Aligned Input (from original input time base)'))
            plt.plot(input_time[:min_len], aligned_output[:min_len], 'r-', label=self._get_label('对齐后输出 (delay compensated)', 'Aligned Output (delay compensated)'))
        else:
            rospy.logwarn("Not enough data to plot aligned signals for verification.")

        plt.grid(True)
        plt.legend()
        plt.title(self._get_label(
            f'对齐后数据 (估计时间延迟: {time_delay:.3f}s)',
            f'Aligned Data (Estimated Time Delay: {time_delay:.3f}s)'
        ))
        plt.xlabel(self._get_label('时间 (秒)', 'Time (s)'))
        plt.ylabel(self._get_label('幅值', 'Amplitude'))
        
        target_save_dir = save_dir_arg if save_dir_arg is not None else self.args.plots_dir
        if not os.path.exists(target_save_dir):
            os.makedirs(target_save_dir)
        save_path = os.path.join(target_save_dir, 'alignment_verification.png')
        
        plt.tight_layout()
        plt.savefig(save_path)
        plt.close()
        
        rospy.loginfo(self._get_label(
            f"对齐验证图已保存到: {save_path}",
            f"Alignment verification plot saved to: {save_path}"
        ))

