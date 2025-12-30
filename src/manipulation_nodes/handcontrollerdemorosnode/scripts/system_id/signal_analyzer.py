import numpy as np
import rospy
from scipy import signal as sig
from scipy.interpolate import interp1d
import math
import os

class SignalAnalyzer:
    def __init__(self, args, use_chinese=False, time_delay=0.0, static_gain=1.0, bias=0.0):
        self.args = args
        self.use_chinese = use_chinese
        self.time_delay = time_delay
        self.static_gain = static_gain
        self.bias = bias
        self.bandwidth = None # Store calculated bandwidth

    def _get_label(self, chinese_text, english_text):
        return chinese_text if self.use_chinese else english_text

    def calculate_instantaneous_frequency(self, start_freq, end_freq, current_time, total_time, sweep_type='logarithmic'):
        """计算扫频过程中给定时间的瞬时频率"""
        if sweep_type == 'logarithmic':
            if start_freq <= 0:
                start_freq = 0.01  # 防止log(0)
            log_start = math.log(start_freq)
            log_end = math.log(end_freq)
            t_ratio = current_time / total_time
            return math.exp(log_start + (log_end - log_start) * t_ratio)
        else:
            return start_freq + (end_freq - start_freq) * (current_time / total_time)

    def calculate_chirp_phase(self, start_freq, end_freq, current_time, total_time, sweep_type='logarithmic'):
        """计算给定时间的啁啾信号相位"""
        if sweep_type == 'logarithmic':
            if start_freq <= 0:
                start_freq = 0.01
            k = math.log(end_freq / start_freq) / total_time
            return 2 * math.pi * start_freq * (math.exp(k * current_time) - 1) / k
        else:
            return 2 * math.pi * (start_freq * current_time + 0.5 * (end_freq - start_freq) * 
                                (current_time ** 2) / total_time)

    def estimate_static_parameters(self, input_signal, output_signal):
        A = np.vstack([input_signal, np.ones(len(input_signal))]).T
        gain, bias = np.linalg.lstsq(A, output_signal, rcond=None)[0]
        corr_coef = np.corrcoef(input_signal, output_signal)[0, 1]
        y_mean = np.mean(output_signal)
        ss_total = np.sum((output_signal - y_mean)**2)
        y_pred = gain * input_signal + bias
        ss_residual = np.sum((output_signal - y_pred)**2)
        r_squared = 1 - (ss_residual / ss_total) if ss_total > 0 else 0 # Avoid division by zero
        return gain, bias, r_squared, corr_coef

    def calculate_time_delay(self, input_data, input_time, output_data, output_time):
        try:
            input_data = np.array(input_data)
            output_data = np.array(output_data)
            
            if len(input_data) < 2 or len(output_data) < 2 or len(input_time) < 2 or len(output_time) < 2:
                rospy.logwarn(self._get_label("时间延迟计算数据不足", "Not enough data for time delay calculation"))
                return 0.0 # Return a default value
            
            input_norm = input_data - np.mean(input_data)
            output_norm = output_data - np.mean(output_data)
            
            input_fs = 1.0 / np.mean(np.diff(input_time)) if len(input_time) > 1 else self.args.update_rate
            output_fs = 1.0 / np.mean(np.diff(output_time)) if len(output_time) > 1 else self.args.update_rate
            fs = (input_fs + output_fs) / 2.0
            if fs == 0: fs = self.args.update_rate # Fallback if fs is zero
            
            xcorr = sig.correlate(output_norm, input_norm, mode='full')
            lags = np.arange(-len(input_norm)+1, len(output_norm)) # Adjusted for potentially different lengths after alignment
            if not lags.any(): # if lags is empty or all zeros
                 rospy.logwarn("Lags array is empty or all zeros in time delay calculation.")
                 return 0.0
            lag_idx = np.argmax(xcorr)
            lag_samples = lags[lag_idx]
            time_delay_val = lag_samples / fs
            
            max_corr = xcorr[lag_idx]
            # Ensure no division by zero for normalization
            std_input = np.std(input_norm)
            std_output = np.std(output_norm)
            if std_input == 0 or std_output == 0 or len(input_norm) == 0:
                normalized_max_corr = 0
            else:
                normalized_max_corr = max_corr / (std_input * std_output * len(input_norm))

            rospy.loginfo(self._get_label(
                f"互相关分析结果: 时间延迟 = {time_delay_val:.4f}s, 相关性 = {normalized_max_corr:.4f}",
                f"Cross-correlation analysis: Time delay = {time_delay_val:.4f}s, Correlation = {normalized_max_corr:.4f}"
            ))
            
            if time_delay_val < 0:
                rospy.logwarn(self._get_label(
                    f"警告: 计算得到负时间延迟 ({time_delay_val:.4f}s)。",
                    f"Warning: Negative time delay calculated ({time_delay_val:.4f}s)."
                ))
            return time_delay_val
        except Exception as e:
            rospy.logerr(self._get_label(
                f"时间延迟计算失败: {str(e)}",
                f"Time delay calculation failed: {str(e)}"
            ))
            return 0.0 # Return a default value

    def align_data(self, input_data_orig, input_time_orig, output_data_orig, output_time_orig, time_delay_comp):
        """
        Compensates output_data for time_delay_comp and resamples it to input_time_orig.
        """
        try:
            input_data = np.array(input_data_orig)
            input_time = np.array(input_time_orig)
            output_data = np.array(output_data_orig)
            output_time = np.array(output_time_orig)

            if len(input_data) == 0 or len(output_data) == 0 or len(input_time) == 0 or len(output_time) == 0:
                rospy.logwarn("Empty data arrays passed to align_data.")
                return np.array([]), np.array([])
            
            # Sort output data by its time, necessary for interpolation
            sort_indices_output = np.argsort(output_time)
            output_time_sorted = output_time[sort_indices_output]
            output_data_sorted = output_data[sort_indices_output]

            # Remove duplicates from sorted output time for interpolation
            unique_indices_output = np.concatenate(([True], np.diff(output_time_sorted) > 1e-9)) # Added tolerance
            output_time_unique = output_time_sorted[unique_indices_output]
            output_data_unique = output_data_sorted[unique_indices_output]

            if len(output_time_unique) < 2: # Need at least 2 points for interp1d
                rospy.logwarn("Not enough unique output data points for interpolation in align_data.")
                # Fallback: return input and original output, possibly truncated to match length
                min_len = min(len(input_data), len(output_data_orig)) # Use original output_data here
                return input_data[:min_len], output_data_orig[:min_len]

            output_func = interp1d(output_time_unique, output_data_unique, kind='linear', 
                                   bounds_error=False, fill_value=(output_data_unique[0], output_data_unique[-1])) # Extrapolate with edge values
            
            # Compensate time for the output data
            output_time_compensated = input_time + time_delay_comp # Target times for output are input_time shifted by delay
            
            # Interpolate output data at these compensated times
            compensated_output_resampled = output_func(output_time_compensated)
            
            rospy.loginfo(self._get_label(
                f"数据对齐 (延迟补偿输出): {len(input_data)} 个点",
                f"Data aligned (delay-compensated output): {len(input_data)} points"
            ))
            return input_data, compensated_output_resampled
            
        except Exception as e:
            rospy.logerr(self._get_label(
                f"数据对齐失败: {str(e)}",
                f"Data alignment failed: {str(e)}"
            ))
            min_len = min(len(input_data_orig), len(output_data_orig))
            return np.array(input_data_orig[:min_len]), np.array(output_data_orig[:min_len])

    def calculate_static_gain_and_bias(self, input_data_aligned, output_data_aligned):
        try:
            input_data = np.array(input_data_aligned)
            output_data = np.array(output_data_aligned)
            
            if len(input_data) < 2: # Need at least 2 points for lstsq
                rospy.logwarn("Not enough data for static gain/bias calculation.")
                return 1.0, 0.0

            A = np.vstack([input_data, np.ones(len(input_data))]).T
            gain, bias = np.linalg.lstsq(A, output_data, rcond=None)[0]
            
            y_mean = np.mean(output_data)
            y_pred = gain * input_data + bias
            ss_total = np.sum((output_data - y_mean)**2)
            ss_residual = np.sum((output_data - y_pred)**2)
            r_squared = 1 - (ss_residual / ss_total) if ss_total > 0 else 0
            
            rospy.loginfo(self._get_label(
                f"线性回归结果: 增益 = {gain:.4f}, 偏置 = {bias:.4f}, R² = {r_squared:.4f}",
                f"Linear regression: Gain = {gain:.4f}, Bias = {bias:.4f}, R² = {r_squared:.4f}"
            ))
            
            if r_squared < 0.8:
                rospy.logwarn(self._get_label(
                    f"警告: R²值较低 ({r_squared:.4f})",
                    f"Warning: Low R² value ({r_squared:.4f})"
                ))
            return gain, bias
        except Exception as e:
            rospy.logerr(self._get_label(
                f"静态参数计算失败: {str(e)}",
                f"Static parameter calculation failed: {str(e)}"
            ))
            return 1.0, 0.0

    def estimate_bandwidth(self, frequencies, magnitudes, coherence=None, coherence_threshold=0.7):
        if frequencies is None or len(frequencies) == 0 or magnitudes is None or len(magnitudes) == 0:
            rospy.logwarn("Cannot estimate bandwidth with empty frequency or magnitude data.")
            return None, None

        epsilon = 1e-10
        magnitudes_safe = np.maximum(magnitudes, epsilon)
        magnitudes_db = 20 * np.log10(magnitudes_safe)
        
        valid_indices = np.arange(len(frequencies))
        coherence_limited_freq = None
        
        if coherence is not None and len(coherence) == len(frequencies):
            # Find where coherence consistently drops below threshold
            # This logic is a bit complex, simplify or ensure it's robust
            # For now, let's use a simpler approach: find first index where coherence drops and stays low
            coh_drops_indices = np.where(coherence < coherence_threshold)[0]
            first_drop_idx = -1
            if len(coh_drops_indices) > 0:
                for idx in coh_drops_indices:
                    # Check if it's a sustained drop (e.g., next 2 points also low)
                    if idx + 2 < len(coherence):
                        if np.all(coherence[idx:idx+3] < coherence_threshold):
                            first_drop_idx = idx
                            break
                    elif idx < len(coherence): # If near the end, a single drop might be enough
                         first_drop_idx = idx
                         break
            
            if first_drop_idx != -1 and first_drop_idx > 0: # Ensure it's not the very first point
                coherence_limited_freq = frequencies[first_drop_idx]
                valid_indices = np.arange(first_drop_idx)
                rospy.loginfo(f"Coherence limits analysis to frequencies < {coherence_limited_freq:.2f} Hz (at index {first_drop_idx})")
        
        if not len(valid_indices) > 0 :
             rospy.logwarn("No valid data points after coherence filtering for bandwidth estimation.")
             # Fallback: use all data if coherence filtering removed everything, but warn
             if len(frequencies)>0:
                 valid_indices = np.arange(len(frequencies))
             else:
                 return None, None # No data at all

        # Operate on the valid range
        freq_valid = frequencies[valid_indices]
        mag_db_valid = magnitudes_db[valid_indices]

        if not len(freq_valid) > 0:
            rospy.logwarn("No data points left for bandwidth estimation after filtering.")
            return None, frequencies[-1] if len(frequencies) > 0 else None # Fallback or None

        peak_idx_valid = np.argmax(mag_db_valid)
        peak_magnitude_db_valid = mag_db_valid[peak_idx_valid]
        peak_frequency_valid = freq_valid[peak_idx_valid]
        threshold_db = peak_magnitude_db_valid - 3.0
        
        magnitude_bandwidth = None
        for i in range(peak_idx_valid + 1, len(freq_valid)):
            if mag_db_valid[i] < threshold_db:
                f1, m1 = freq_valid[i-1], mag_db_valid[i-1]
                f2, m2 = freq_valid[i], mag_db_valid[i]
                if (m2 - m1) == 0: # Avoid division by zero
                    magnitude_bandwidth = f1 
                else:
                    magnitude_bandwidth = f1 + (f2 - f1) * (threshold_db - m1) / (m2 - m1)
                rospy.loginfo(f"-3dB magnitude bandwidth estimated at: {magnitude_bandwidth:.2f} Hz (within valid range)")
                break
        
        final_bandwidth = None
        if magnitude_bandwidth is not None:
            final_bandwidth = magnitude_bandwidth
        elif coherence_limited_freq is not None:
            final_bandwidth = coherence_limited_freq
            rospy.loginfo(f"Bandwidth set by coherence limit: {final_bandwidth:.2f} Hz as -3dB point not found in valid range.")
        elif len(freq_valid) > 0:
            final_bandwidth = freq_valid[-1]
            rospy.logwarn(f"-3dB point not found. Bandwidth set to max valid frequency: {final_bandwidth:.2f} Hz.")
        else:
             rospy.logwarn("Could not determine bandwidth.")
             return None, peak_frequency_valid if 'peak_frequency_valid' in locals() else None

        # Ensure bandwidth is not greater than coherence_limited_freq if that was determined
        if coherence_limited_freq is not None and final_bandwidth > coherence_limited_freq:
            rospy.loginfo(f"Adjusting bandwidth from {final_bandwidth:.2f} to coherence limit {coherence_limited_freq:.2f} Hz.")
            final_bandwidth = coherence_limited_freq
        
        self.bandwidth = final_bandwidth # Store it
        return final_bandwidth, peak_frequency_valid

    def align_data_by_time(self, input_data_raw, output_data_raw, input_time_raw, output_time_raw, 
                           target_sample_rate=None, estimated_delay=0.0):
        input_d = np.array(input_data_raw)
        output_d = np.array(output_data_raw)
        input_t = np.array(input_time_raw)
        output_t = np.array(output_time_raw)

        if len(input_d) < 10 or len(output_d) < 10 or len(input_t) < 10 or len(output_t) < 10:
            rospy.logerr(f"Not enough data for align_data_by_time: in({len(input_d)}), out({len(output_d)})")
            min_len_fallback = min(len(input_d) if len(input_d)>0 else 10, 
                                 len(output_d) if len(output_d)>0 else 10, 
                                 len(input_t) if len(input_t)>0 else 10, 
                                 len(output_t) if len(output_t)>0 else 10)
            # Ensure we return arrays of some minimal consistent length if possible, or handle error appropriately upstream
            return input_d[:min_len_fallback], output_d[:min_len_fallback], input_t[:min_len_fallback], 1.0

        # Defensively ensure input_t and input_d have the same length before sorting
        len_it = len(input_t)
        len_id = len(input_d)
        if len_it != len_id:
            rospy.logwarn(f"align_data_by_time: Mismatch detected - len(input_t)={len_it}, len(input_d)={len_id}. Truncating to min length.")
            min_len_in = min(len_it, len_id)
            input_t = input_t[:min_len_in]
            input_d = input_d[:min_len_in]
            rospy.loginfo(f"align_data_by_time: Adjusted input lengths to {len(input_t)}")
        
        # Sort and unique ops for input
        if len(input_t) > 0: # Proceed only if there is data
            input_s_idx = np.argsort(input_t)
            input_t = input_t[input_s_idx]
            input_d = input_d[input_s_idx]
            in_uniq_idx = np.concatenate(([True], np.diff(input_t) > 1e-9)) if len(input_t) > 1 else np.array([True])
            input_t = input_t[in_uniq_idx]
            input_d = input_d[in_uniq_idx]
        
        # Defensively ensure output_t and output_d have the same length before sorting
        len_ot = len(output_t)
        len_od = len(output_d)
        if len_ot != len_od:
            rospy.logwarn(f"align_data_by_time: Mismatch detected - len(output_t)={len_ot}, len(output_d)={len_od}. Truncating to min length.")
            min_len_out = min(len_ot, len_od)
            output_t = output_t[:min_len_out]
            output_d = output_d[:min_len_out]
            rospy.loginfo(f"align_data_by_time: Adjusted output lengths to {len(output_t)}")

        # Sort and unique ops for output
        if len(output_t) > 0: # Proceed only if there is data
            output_s_idx = np.argsort(output_t)
            output_t = output_t[output_s_idx]
            output_d = output_d[output_s_idx]
            out_uniq_idx = np.concatenate(([True], np.diff(output_t) > 1e-9)) if len(output_t) > 1 else np.array([True])
            output_t = output_t[out_uniq_idx]
            output_d = output_d[out_uniq_idx]

        if len(input_t) < 2 or len(output_t) < 2:
            rospy.logerr("Not enough unique data points after cleaning for align_data_by_time.")
            min_len = min(len(input_d), len(output_d), len(input_t), len(output_t))
            return input_d[:min_len], output_d[:min_len], input_t[:min_len], 1.0

        input_fs = 1.0 / np.mean(np.diff(input_t)) if len(input_t) > 1 else self.args.update_rate
        output_fs = 1.0 / np.mean(np.diff(output_t)) if len(output_t) > 1 else self.args.update_rate
        rospy.loginfo(f"Original sample rates - Input: {input_fs:.2f} Hz, Output: {output_fs:.2f} Hz")

        if target_sample_rate is None:
            target_sample_rate = max(input_fs, output_fs, self.args.update_rate) # Ensure at least update_rate
        
        # Effective output time considering delay
        effective_output_t = output_t - estimated_delay
        
        start_common_t = max(input_t[0], effective_output_t[0])
        end_common_t = min(input_t[-1], effective_output_t[-1])

        if end_common_t <= start_common_t:
            rospy.logwarn(f"No time overlap after delay comp! Input: [{input_t[0]:.2f}-{input_t[-1]:.2f}], Eff.Output: [{effective_output_t[0]:.2f}-{effective_output_t[-1]:.2f}]")
            # Fallback to using original time ranges without delay compensation for this step
            start_common_t = min(input_t[0], output_t[0])
            end_common_t = max(input_t[-1], output_t[-1])
            effective_output_t = output_t # Revert to original output time for interpolation if no overlap with delay
            rospy.logwarn("Reverting to original output times for interpolation due to no overlap.")

        time_span = end_common_t - start_common_t
        if time_span <= 0:
            rospy.logerr("Common time span is zero or negative in align_data_by_time.")
            min_len = min(len(input_d), len(output_d), len(input_t), len(output_t))
            return input_d[:min_len], output_d[:min_len], input_t[:min_len], 1.0
            
        num_samples = int(time_span * target_sample_rate)
        if num_samples < 10:
            rospy.logwarn(f"Very few samples ({num_samples}) for aligned data. Span: {time_span:.2f}s, Rate: {target_sample_rate:.2f}Hz")
            # Fallback to original, possibly truncated
            min_len = min(len(input_d), len(output_d), len(input_t), len(output_t))
            return input_d[:min_len], output_d[:min_len], input_t[:min_len], 1.0/(np.mean(np.diff(input_t[:min_len])) if min_len > 1 else 1)

        aligned_t = np.linspace(start_common_t, end_common_t, num_samples)
        
        try:
            input_interp = interp1d(input_t, input_d, kind='linear', bounds_error=False, fill_value=(input_d[0], input_d[-1]))
            # Use effective_output_t for creating the interpolation function of output
            output_interp = interp1d(effective_output_t, output_d, kind='linear', bounds_error=False, fill_value=(output_d[0], output_d[-1]))
            
            aligned_input_d = input_interp(aligned_t)
            aligned_output_d = output_interp(aligned_t)
            
            actual_fs = 1.0 / np.mean(np.diff(aligned_t)) if len(aligned_t) > 1 else target_sample_rate
            rospy.loginfo(f"Data aligned to common timeline: {num_samples} pts, {actual_fs:.2f} Hz. Span: {time_span:.2f}s")
            return aligned_input_d, aligned_output_d, aligned_t, actual_fs
        except ValueError as ve:
            rospy.logerr(f"Interpolation error in align_data_by_time: {ve}. Check time vector monotonicity and length.")
            rospy.logerr(f"Input time: {input_t[:5]}...{input_t[-5:]} (len {len(input_t)}), Output time (effective for interp): {effective_output_t[:5]}...{effective_output_t[-5:]} (len {len(effective_output_t)})")
            min_len = min(len(input_d), len(output_d), len(input_t), len(output_t))
            return input_d[:min_len], output_d[:min_len], input_t[:min_len], 1.0
        except Exception as e:
            rospy.logerr(f"Error in align_data_by_time: {e}")
            min_len = min(len(input_d), len(output_d), len(input_t), len(output_t))
            return input_d[:min_len], output_d[:min_len], input_t[:min_len], 1.0

    def analyze_frequency_response(self, input_arr, output_arr, sample_rate_hz, known_time_delay, static_gain_val, bias_val, visualizer=None):
        rospy.loginfo("="*60)
        rospy.loginfo(f"Analyzing frequency response with known delay: {known_time_delay:.4f}s, gain: {static_gain_val:.4f}, bias: {bias_val:.4f}")
        rospy.loginfo("="*60)

        if len(input_arr) < 10 or len(output_arr) < 10:
            rospy.logerr("Not enough data points for frequency analysis.")
            return None, {}
        
        # Apply static gain and bias to input for analysis if that's the convention expected
        # OR, remove bias from output and work with gain later for TF magnitude.
        # Conventionally, TF = Output/Input. If input_arr is the command and output_arr is the response:
        # output_response = gain * (input_command_delayed) + bias
        # (output_response - bias) / gain = input_command_delayed
        # For TF analysis, we often use (output - bias) as the effective output, and input_command as input.
        # The gain then scales the TF magnitude.
        
        # Let's use output_corrected = (output_arr - bias_val)
        # And input_to_use = input_arr (assuming it's already delay compensated if needed before this function)
        # The static_gain_val will be part of the system characteristics but not directly divided out before TF estimation.

        output_corrected_for_bias = output_arr - bias_val
        input_to_analyze = input_arr # This input should ideally be aligned with output_corrected_for_bias

        data_len = min(len(input_to_analyze), len(output_corrected_for_bias))
        if data_len < 10:
             rospy.logerr("Not enough data after initial prep for freq analysis.")
             return None, {}
        
        input_to_analyze = input_to_analyze[:data_len]
        output_corrected_for_bias = output_corrected_for_bias[:data_len]

        desired_res = 0.1 # Hz
        win_size_res = int(sample_rate_hz / desired_res)
        max_win_size = data_len // 4
        win_size = min(max_win_size, max(256, win_size_res))
        if win_size > data_len : win_size = data_len // 2 # Ensure window is not too large
        if win_size < 16 : # Ensure window is not too small
            rospy.logwarn(f"Calculated window size {win_size} is too small. Data length {data_len}. Using min of 16 or data_len/2.")
            win_size = max(16, data_len//2) if data_len >=32 else data_len
            if win_size == 0 and data_len > 0 : win_size = data_len # if data_len is small, use all of it
            elif win_size == 0 and data_len == 0: 
                rospy.logerr("Zero data length for frequency analysis.")
                return None, {}

        window_hann = sig.get_window('hann', win_size)
        n_overlap = win_size // 2

        rospy.loginfo(f"Freq Analysis Params: Window Size={win_size}, Overlap={n_overlap}, Fs={sample_rate_hz:.2f}Hz")
        rospy.loginfo(f"  Expected Freq Resolution: {sample_rate_hz/win_size:.3f} Hz")

        try:
            freqs, Pxy = sig.csd(input_to_analyze, output_corrected_for_bias, fs=sample_rate_hz, 
                                  nperseg=win_size, noverlap=n_overlap, window=window_hann, detrend='linear')
            _, Pxx = sig.welch(input_to_analyze, fs=sample_rate_hz, nperseg=win_size, 
                               noverlap=n_overlap, window=window_hann, detrend='linear')
            # _, Pyy = sig.welch(output_corrected_for_bias, fs=sample_rate_hz, nperseg=win_size, 
            #                   noverlap=n_overlap, window=window_hann, detrend='linear') # For coherence if Pxx is zero anywhere

            # Avoid division by zero in Pxx. Add small epsilon or handle carefully.
            epsilon_tf = 1e-9
            H_f_complex = Pxy / (Pxx + epsilon_tf)
            tf_magnitude = np.abs(H_f_complex)
            tf_phase_rad = np.angle(H_f_complex)
            tf_phase_rad_unwrapped = np.unwrap(tf_phase_rad)
            tf_phase_deg = np.degrees(tf_phase_rad_unwrapped)

            _, coherence_vals = sig.coherence(input_to_analyze, output_corrected_for_bias, fs=sample_rate_hz, 
                                            nperseg=win_size, noverlap=n_overlap, window=window_hann, detrend='linear')
            
            # Smooth TF magnitude and phase (optional, but often helpful)
            # tf_magnitude = sig.medfilt(tf_magnitude, min(5, len(tf_magnitude)//10*2+1 if len(tf_magnitude)//10*2+1 > 0 else 1))
            # tf_phase_deg = sig.medfilt(tf_phase_deg, min(5, len(tf_phase_deg)//10*2+1 if len(tf_phase_deg)//10*2+1 > 0 else 1))
            # tf_phase_rad_unwrapped = np.radians(tf_phase_deg) # if smoothing degrees

            max_sweep_f = self.args.freq_end
            valid_f_mask = freqs <= max_sweep_f
            if not np.any(valid_f_mask):
                rospy.logwarn(f"No freqs <= max_sweep_f ({max_sweep_f} Hz). Using all.")
            else:
                freqs = freqs[valid_f_mask]
                tf_magnitude = tf_magnitude[valid_f_mask]
                tf_phase_rad_unwrapped = tf_phase_rad_unwrapped[valid_f_mask]
                tf_phase_deg = tf_phase_deg[valid_f_mask]
                coherence_vals = coherence_vals[valid_f_mask]
                rospy.loginfo(f"Freq range limited to {max_sweep_f} Hz. Points: {len(freqs)}")

            if len(freqs) == 0:
                rospy.logerr("No frequency points left after filtering by max_sweep_freq.")
                return None, {}

            mean_coh = np.mean(coherence_vals) if len(coherence_vals) > 0 else 0
            max_coh = np.max(coherence_vals) if len(coherence_vals) > 0 else 0

            if mean_coh < 0.3:
                rospy.logwarn(f"Mean coherence ({mean_coh:.4f}) is very low!")

            # Estimate bandwidth from TF magnitude (which is |(Output-Bias)/Input|)
            # This magnitude needs to be scaled by static_gain_val for physical interpretation of system output/input ratio
            # However, for -3dB bandwidth, we look at drop from DC gain of this specific TF.
            # The DC gain of H_f_complex = Pxy/Pxx is effectively gain_dynamic if input is scaled.
            # Or, if we consider H_f as (Output-Bias)/Input, then its DC value should be static_gain_val.
            # Let's use the tf_magnitude as is, find its peak (or low-freq value) and drop 3dB from that.
            
            # Bandwidth estimation using the existing estimate_bandwidth method
            # This method expects magnitudes, not scaled by system gain yet.
            # The static_gain is an overall DC gain of the system.
            # The transfer_function here is output_delta / input.
            # Its magnitude at low frequency should ideally be close to self.static_gain.
            est_bw, est_peak_f = self.estimate_bandwidth(freqs, tf_magnitude, coherence_vals, 0.7)
            self.bandwidth = est_bw # Store it in the analyzer instance

            phase_delay_s = np.full_like(freqs, np.nan, dtype=float)
            reliable_mask_pd = (coherence_vals >= 0.7) & (freqs >= 0.1) # For phase delay calc
            
            # Avoid division by zero for frequency in phase delay calculation
            non_zero_freq_mask = freqs != 0
            mask_for_pd_calc = reliable_mask_pd & non_zero_freq_mask

            if np.any(mask_for_pd_calc):
                phase_delay_s[mask_for_pd_calc] = -tf_phase_rad_unwrapped[mask_for_pd_calc] / (2 * np.pi * freqs[mask_for_pd_calc])
            
            # System parameters summary
            # The peak of tf_magnitude here is |(Output-Bias)/Input|peak
            # So, actual resonance gain in dB would be 20*log10(static_gain_val * peak_of_tf_magnitude) if tf_magnitude is normalized to 1 at DC.
            # Or, more simply, if tf_magnitude is gain_dynamic, then its peak is the resonance.
            # Let's assume tf_magnitude is the dynamic gain. Its peak is the resonance magnitude.
            peak_tf_mag_val = 0
            if est_peak_f is not None and len(tf_magnitude)>0:
                peak_idx = np.argmin(np.abs(freqs - est_peak_f))
                peak_tf_mag_val = tf_magnitude[peak_idx]

            results = {
                'frequencies': freqs,
                'tf_magnitude': tf_magnitude,
                'tf_phase_deg': tf_phase_deg,
                'coherence': coherence_vals,
                'bandwidth': est_bw,
                'peak_frequency': est_peak_f,
                'resonance_gain_abs': peak_tf_mag_val, # Absolute gain at resonance from (O-B)/I
                'resonance_gain_db': 20 * np.log10(peak_tf_mag_val) if peak_tf_mag_val > 1e-9 else -np.inf,
                'mean_coherence': mean_coh,
                'phase_delay_s': phase_delay_s,
                'static_gain_estimated_in_calib': static_gain_val, # From calibration phase
                'bias_estimated_in_calib': bias_val,             # From calibration phase
                'time_delay_estimated_in_calib': known_time_delay    # From calibration phase
            }
            rospy.loginfo(f"Bandwidth: {est_bw:.2f} Hz, Peak Freq: {est_peak_f:.2f} Hz, Resonance Gain (abs): {peak_tf_mag_val:.2f}")

            if visualizer and self.args.plot_results: # Pass self.time_delay (system time_delay from calibration)
                visualizer._plot_frequency_response(input_to_analyze, output_corrected_for_bias, freqs, 
                                                 tf_magnitude, coherence_vals, est_bw, est_peak_f,
                                                 sample_rate_hz, mean_coh, tf_phase_deg, phase_delay_s, self.time_delay)

            if self.args.save_data:
                save_dict = {
                    'input_analyzed': input_to_analyze,
                    'output_corrected_for_bias': output_corrected_for_bias,
                    'sampling_rate_hz': sample_rate_hz,
                    'static_gain_calib': static_gain_val,
                    'bias_calib': bias_val,
                    'time_delay_calib': known_time_delay,
                }
                save_dict.update(results) # Add all analysis results
                # Remove any large array objects if they are already part of results, to avoid duplication if not careful
                np.savez(os.path.join(self.args.plots_dir, 'system_identification_data.npz'), **save_dict) # Use plots_dir from args
                rospy.loginfo(f"Frequency response data saved to '{os.path.join(self.args.plots_dir, 'system_identification_data.npz')}'")
            
            return est_bw, results

        except Exception as e_freq:
            rospy.logerr(f"Error during frequency analysis: {str(e_freq)}")
            import traceback
            traceback.print_exc()
            return None, {}

    def preprocess_data(self, input_d_raw, output_d_raw, input_t_raw, output_t_raw):
        rospy.loginfo("Preprocessing sweep data...")
        if len(input_d_raw) < 10 or len(output_d_raw) < 10:
            rospy.logwarn("Too few data points for preprocessing.")
            return np.array(input_d_raw), np.array(output_d_raw), np.array(input_t_raw), np.array(output_t_raw)

        input_d = np.array(input_d_raw)
        output_d = np.array(output_d_raw)
        input_t = np.array(input_t_raw)
        output_t = np.array(output_t_raw)
        
        # Trim start and end (e.g., 10% from each side, or fixed number of samples/seconds)
        # Example: trim 1 second from start and end if data is long enough
        # sample_rate_approx = self.args.update_rate # Assuming data is roughly at this rate
        # trim_samples = int(sample_rate_approx * 1.0) # 1 second
        trim_fraction = 0.1 # Trim 10% from each side
        
        len_in = len(input_d)
        len_out = len(output_d)

        if len_in > 20: # Only trim if substantially long
            trim_count_in = int(len_in * trim_fraction)
            if trim_count_in * 2 >= len_in: # Avoid trimming everything
                trim_count_in = len_in // 4
            if trim_count_in > 0 : 
                input_d = input_d[trim_count_in:-trim_count_in]
                input_t = input_t[trim_count_in:-trim_count_in]
                rospy.loginfo(f"Trimmed {trim_count_in} samples from input ends.")
        
        if len_out > 20:
            trim_count_out = int(len_out * trim_fraction)
            if trim_count_out * 2 >= len_out:
                trim_count_out = len_out // 4
            if trim_count_out > 0:
                output_d = output_d[trim_count_out:-trim_count_out]
                output_t = output_t[trim_count_out:-trim_count_out]
                rospy.loginfo(f"Trimmed {trim_count_out} samples from output ends.")

        # Ensure final consistency after trimming
        min_len_in = min(len(input_d), len(input_t))
        input_d = input_d[:min_len_in]
        input_t = input_t[:min_len_in]

        min_len_out = min(len(output_d), len(output_t))
        output_d = output_d[:min_len_out]
        output_t = output_t[:min_len_out]

        rospy.loginfo(f"Data points after preprocessing: input={len(input_d)}, output={len(output_d)}")
        return input_d, output_d, input_t, output_t 