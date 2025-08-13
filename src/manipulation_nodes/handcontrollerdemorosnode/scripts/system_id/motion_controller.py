import rospy
import math
import numpy as np
import time
from std_msgs.msg import Float64MultiArray
from kuavo_msgs.msg import twoArmHandPoseCmd, twoArmHandPose, armHandPose,armTargetPoses
from kuavo_msgs.srv import (
                       changeArmCtrlMode, changeArmCtrlModeRequest,
                        changeTorsoCtrlMode, changeTorsoCtrlModeRequest)
class MotionController:
    def __init__(self, args, signal_analyzer, visualizer, use_chinese=False):
        self.args = args
        self.signal_analyzer = signal_analyzer # For accessing methods like calculate_instantaneous_frequency
        self.visualizer = visualizer # For calling visualization methods
        self.use_chinese = use_chinese
        self.pub = None
        self.sub = None
        self.shutdown_requested = False

        # Data storage - managed by MotionController as it acquires the data
        self.input_data = []
        self.output_data = []
        self.input_time_data = []
        self.output_time_data = []

        # System parameters - to be updated after calibration
        self.time_delay = args.time_delay
        self.static_gain = 1.0
        self.bias = 0.0

    def _get_label(self, chinese_text, english_text):
        return chinese_text if self.use_chinese else english_text

    def initialize_ros_communications(self):
        # Publisher for sending commands to the robot
        self.pub = rospy.Publisher('/mm/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
        self.kuavo_arm_target_poses = rospy.Publisher('/kuavo_arm_target_poses', armTargetPoses, queue_size=10)
        # Subscriber for receiving robot pose feedback
        self.sub = rospy.Subscriber('humanoid_eef_poses', Float64MultiArray, self.humanoid_pose_callback)
        rospy.loginfo(self._get_label("MotionController: ROS通讯已初始化", "MotionController: ROS communications initialized."))
        rospy.sleep(0.5) # Wait for publisher/subscriber to connect

    def humanoid_pose_callback(self, msg):
        if self.shutdown_requested:
            return
        z_position = msg.data[2]  # Z is the third element
        current_time = rospy.get_time()
        self.output_data.append(z_position)
        self.output_time_data.append(current_time)

    def publish_hand_pose(self, x, y_left, y_right, z_left, z_right, quat, elbow_pos_left, elbow_pos_right, joint_angles):
        if self.shutdown_requested or self.pub is None:
            return

        current_time = rospy.get_time()
        # Record input data just before publishing for better accuracy
        self.input_data.append(z_left) # Assuming z_left is the command we are tracking
        self.input_time_data.append(current_time)
        
        msg = twoArmHandPoseCmd()
        two_arm_pose = twoArmHandPose()
        
        left_pose = armHandPose()
        left_pose.pos_xyz = [x, y_left, z_left]
        left_pose.quat_xyzw = quat
        left_pose.elbow_pos_xyz = elbow_pos_left
        left_pose.joint_angles = joint_angles
        
        right_pose = armHandPose()
        right_pose.pos_xyz = [x, y_right, z_right]
        right_pose.quat_xyzw = quat
        right_pose.elbow_pos_xyz = elbow_pos_right
        right_pose.joint_angles = joint_angles
        
        two_arm_pose.left_pose = left_pose
        two_arm_pose.right_pose = right_pose
        msg.hand_poses = two_arm_pose
        self.pub.publish(msg)

    def publish_neutral_pose(self):
        rospy.loginfo(self._get_label("发送中立姿态...", "Sending neutral pose..."))
        quat = [0, -0.67566370964, 0, 0.73720997571945]
        elbow_pos_left = [0.08, 0.4, 0.18]
        elbow_pos_right = [0.08, -0.4, 0.18]
        joint_angles = [0, 0, 0, 0, 0, 0, 0]
        z_val = self.args.z_center
        self.publish_hand_pose(self.args.x_center, self.args.y_left, self.args.y_right, 
                               z_val, z_val, quat, elbow_pos_left, elbow_pos_right, joint_angles)
        # 重置MPC和手臂       
        self.arm_reset()
        rospy.loginfo(self._get_label("已发送中立姿态", "Neutral pose sent"))

    def srv_change_manipulation_mpc_ctrl_mode(self, ctrl_mode:int)->bool:
        try:
            service_name = '/mobile_manipulator_mpc_control'
            rospy.wait_for_service(service_name, timeout=2.0)
            set_mode_srv = rospy.ServiceProxy(service_name, changeTorsoCtrlMode)
            
            req = changeTorsoCtrlModeRequest()
            req.control_mode = ctrl_mode
            
            resp = set_mode_srv(req)
            if not resp.result:
                print(f"Failed to change manipulation mpc control mode to {ctrl_mode}: {resp.message}")
            return resp.result
        except rospy.ServiceException as e:
            print(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e: # For timeout from wait_for_service
            print(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            print(f"Failed to change manipulation mpc control mode: {e}")
        return False

    def arm_reset(self):
        self.srv_change_manipulation_mpc_ctrl_mode(0)
        self.srv_change_manipulation_mpc_control_flow(0)
        self.srv_change_arm_ctrl_mode(1)

    def arm_change_mpc(self):
        self.srv_change_manipulation_mpc_ctrl_mode(1)
        self.srv_change_arm_ctrl_mode(2)
        self.srv_change_manipulation_mpc_control_flow(1)

    def srv_change_manipulation_mpc_control_flow(self, ctrl_flow: int)-> bool:
        try:
            service_name = '/enable_mm_wbc_arm_trajectory_control'
            rospy.wait_for_service(service_name, timeout=2.0)
            set_mode_srv = rospy.ServiceProxy(service_name, changeArmCtrlMode)

            req = changeArmCtrlModeRequest()
            req.control_mode = ctrl_flow

            resp = set_mode_srv(req)
            if not resp.result:
                print(f"Failed to change manipulation mpc wbc arm trajectory control to {ctrl_flow}: {resp.message}")
            return resp.result
        except rospy.ServiceException as e:
            print(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e:  # For timeout from wait_for_service
            print(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            print(f"Failed to change manipulation mpc control flow: {e}")
        return False

    def srv_change_arm_ctrl_mode(self, mode: int)->bool:
        try:
            rospy.wait_for_service('/humanoid_change_arm_ctrl_mode', timeout=2.0)
            change_arm_ctrl_mode_srv = rospy.ServiceProxy('/humanoid_change_arm_ctrl_mode', changeArmCtrlMode)
            req = changeArmCtrlModeRequest()
            req.control_mode = mode
            resp = change_arm_ctrl_mode_srv(req)
            return resp.result
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
        except Exception as e:
            print(f"[Error] change arm ctrl mode: {e}")
        return False

    def change_arm_ctrl_mode(self, control_mode):
        rospy.wait_for_service('/arm_traj_change_mode', timeout=2.0)
        try:
            change_mode = rospy.ServiceProxy('/arm_traj_change_mode', changeArmCtrlMode)
            req = changeArmCtrlModeRequest()
            req.control_mode = control_mode
            res = change_mode(req)
            if res.result:
                rospy.loginfo("手臂控制模式已更改为 %d", control_mode)
            else:
                rospy.logerr("无法将手臂控制模式更改为 %d", control_mode)
        except rospy.ServiceException as e:
            rospy.logerr("服务调用失败: %s", e)    

    
    def publish_kuavo_arm_poses(self, joint_q:list, time:float=3):
        msg = armTargetPoses()
        msg.times = [time]
        msg.values = [math.degrees(angle) for angle in joint_q]
        self.kuavo_arm_target_poses.publish(msg)

    def clear_collected_data(self):
        self.input_data.clear()
        self.output_data.clear()
        self.input_time_data.clear()
        self.output_time_data.clear()
        rospy.loginfo(self._get_label("MotionController: 数据缓冲区已清除", "MotionController: Data buffers cleared."))

    def get_collected_data(self):
        return (
            np.array(self.input_data),
            np.array(self.output_data),
            np.array(self.input_time_data),
            np.array(self.output_time_data)
        )
    
    def _move_arm_start_pose(self):
        self.arm_reset()
        print("")
        time.sleep(2.0)
        self.change_arm_ctrl_mode(2)
        q = [-0.375, 0.191, 0.164, -1.278, 0.159, -0.211, 0.102, 
            -0.374, -0.191, -0.163, -1.283, -0.176, 0.209, 0.114]
        cost_time = 2.0
        self.publish_kuavo_arm_poses(q, cost_time)
        time.sleep(cost_time)
    def run_warmup_phase(self):
        rospy.loginfo("="*60)
        rospy.loginfo(self._get_label("开始预热阶段", "Starting warmup phase"))
        rospy.loginfo(f"Freq: {self.args.calib_freq} Hz, Cycles: {self.args.calib_cycles}, Amp: {self.args.calib_amplitude} m")
        rospy.loginfo("="*60)

        # Temporarily disable data collection during warmup by unsubscribing/resubscribing or using a flag
        # Simpler: clear data after warmup, and don't record input in a special publish_hand_pose_no_record
        original_sub = self.sub
        if self.sub:
            self.sub.unregister()
            self.sub = None # Mark as unregistered
        # Minimal callback that does nothing
        temp_sub = rospy.Subscriber('humanoid_eef_poses', Float64MultiArray, lambda msg: None) 

        # Store and clear current data buffers before warmup
        # We don't want warmup movements in our main data buffers
        _ = self.input_data.copy() # Not used, but shows intent
        _ = self.output_data.copy()
        _ = self.input_time_data.copy()
        _ = self.output_time_data.copy()
        self.clear_collected_data() # Clears the controller's main buffers

        # 插值移动到开始点
        self._move_arm_start_pose()

        quat = [0, -0.67566370964, 0, 0.73720997571945]
        elbow_l = [0.08, 0.4, 0.18]; elbow_r = [0.08, -0.4, 0.18]
        j_angles = [0]*7
        rate = rospy.Rate(self.args.update_rate)
        cycle_time = 1.0 / self.args.calib_freq
        total_warmup_time = self.args.calib_cycles * cycle_time
        start_time = rospy.get_time()
        last_cycle_report = 0

        while not self.shutdown_requested and not rospy.is_shutdown():
            current_elapsed_time = rospy.get_time() - start_time
            if current_elapsed_time >= total_warmup_time: break

            angle = 2 * math.pi * self.args.calib_freq * current_elapsed_time
            z_offset = self.args.calib_amplitude * math.sin(angle)
            current_cycle = int(current_elapsed_time / cycle_time)
            if current_cycle > last_cycle_report:
                rospy.loginfo(f"Warmup cycle {current_cycle + 1} / {self.args.calib_cycles}")
                last_cycle_report = current_cycle
            
            z_left_cmd = self.args.z_center + z_offset
            z_right_cmd = self.args.z_center # Keep right hand still
            
            # Publish without recording to internal buffers
            msg_cmd = twoArmHandPoseCmd()
            two_arm = twoArmHandPose()
            lp = armHandPose(); lp.pos_xyz = [self.args.x_center, self.args.y_left, z_left_cmd]; lp.quat_xyzw = quat; lp.elbow_pos_xyz = elbow_l; lp.joint_angles = j_angles
            rp = armHandPose(); rp.pos_xyz = [self.args.x_center, self.args.y_right, z_right_cmd]; rp.quat_xyzw = quat; rp.elbow_pos_xyz = elbow_r; rp.joint_angles = j_angles
            two_arm.left_pose = lp; two_arm.right_pose = rp
            msg_cmd.hand_poses = two_arm
            if self.pub: self.pub.publish(msg_cmd)

            rate.sleep()
        self.clear_collected_data() # Clear any stray data from warmup
        # Restore original subscriber for actual data collection
        if temp_sub: temp_sub.unregister()
        if original_sub is not None : # If there was an original subscriber, re-establish it
             self.sub = rospy.Subscriber('humanoid_eef_poses', Float64MultiArray, self.humanoid_pose_callback)
        elif self.sub is None: # If it was never initialized or fully cleared, re-init
             self.sub = rospy.Subscriber('humanoid_eef_poses', Float64MultiArray, self.humanoid_pose_callback)

        rospy.loginfo(self._get_label("预热阶段完成", "Warmup phase completed."))

    def run_periodic_signal(self, signal_params, duration):
        self.clear_collected_data()
        wave_type = signal_params.get("wave_type", "sine")
        offset = signal_params.get("offset", 0.0)
        amplitude = signal_params.get("amplitude", 0.05)
        frequency = signal_params.get("frequency", 0.25)

        quat = [0, -0.67566370964, 0, 0.73720997571945]
        elb_l = [0.08, 0.4, 0.18]; elb_r = [0.08, -0.4, 0.18]
        j_angs = [0]*7
        rate = rospy.Rate(self.args.update_rate)
        start_t = rospy.get_time()
        last_rep_t = start_t

        rospy.loginfo(f"Running {wave_type} signal: Amp={amplitude:.3f}m, Freq={frequency:.2f}Hz, Dur={duration:.1f}s")

        # 重置MPC和手臂       
        self.arm_reset()
        rospy.loginfo("重置MPC和手臂完成")
        # 切换到KMPC
        self.arm_change_mpc()
        rospy.loginfo("切换到KMPC完成")

        while not self.shutdown_requested and not rospy.is_shutdown():
            elap_t = rospy.get_time() - start_t
            if elap_t >= duration: break

            if wave_type == "sine":
                z_off = amplitude * math.sin(2 * math.pi * frequency * elap_t)
            elif wave_type == "square":
                z_off = amplitude if (elap_t * frequency) % 1.0 < 0.5 else -amplitude
            # Add other wave types if necessary (e.g., triangle)
            else: # Default to sine
                z_off = amplitude * math.sin(2 * math.pi * frequency * elap_t)

            z_l_cmd = self.args.z_center + offset + z_off
            z_r_cmd = self.args.z_center
            
            if rospy.get_time() - last_rep_t >= 1.0:
                rospy.loginfo(f"Progress: {elap_t/duration*100:.1f}% ({duration-elap_t:.1f}s left)")
                last_rep_t = rospy.get_time()
            self.publish_hand_pose(self.args.x_center, self.args.y_left, self.args.y_right, 
                                   z_l_cmd, z_r_cmd, quat, elb_l, elb_r, j_angs)
            rate.sleep()
        
        rospy.loginfo(f"Periodic signal finished. Duration: {rospy.get_time() - start_t:.2f}s. Collected {len(self.input_data)} input points.")
        return len(self.input_data)

    def run_calibration_phase(self):
        rospy.loginfo(self._get_label("运行标定阶段...", "Running calibration phase..."))
        self.clear_collected_data()
        
        signal_params = {
            "wave_type": "sine", 
            "offset": 0.0, 
            "amplitude": self.args.calibration_amplitude, 
            "frequency": 1.0 / self.args.calibration_period
        }
        duration = self.args.calibration_duration
        self.run_periodic_signal(signal_params, duration)
        rospy.sleep(0.5) # Allow last data points to arrive

        in_data, out_data, in_time, out_time = self.get_collected_data()

        if len(in_data) < 10 or len(out_data) < 10:
            rospy.logerr(self._get_label("标定数据不足!", "Not enough calibration data!"))
            return False # Indicate failure
        
        rospy.loginfo("Aligning data for calibration...")
        # Align raw data first to a common timeline without delay compensation
        # Target sample rate can be None to use max of originals or a fixed high rate like 200Hz
        aligned_in_raw, aligned_out_raw, aligned_t_common, actual_fs = \
            self.signal_analyzer.align_data_by_time(in_data, out_data, in_time, out_time, 
                                                    target_sample_rate=self.args.update_rate, estimated_delay=0.0)

        if len(aligned_in_raw) < 10:
            rospy.logerr("Not enough data after initial alignment for calibration.")
            return False

        rospy.loginfo(f"Data aligned to {actual_fs:.2f} Hz, {len(aligned_in_raw)} points.")

        # Now, calculate time delay using these common-timeline signals
        est_delay = self.signal_analyzer.calculate_time_delay(aligned_in_raw, aligned_t_common, 
                                                              aligned_out_raw, aligned_t_common)
        if est_delay is None: # calculate_time_delay might return None on error, ensure it returns a float
            est_delay = self.args.time_delay # Fallback to user-provided
            rospy.logwarn(f"Auto delay estimation failed. Using default/previous: {est_delay:.4f}s")
        self.time_delay = est_delay
        rospy.loginfo(f"Estimated time delay: {self.time_delay:.4f}s")

        # Compensate for this delay: shift output relative to input on the common timeline
        # The align_data method is designed to take original unaligned data and apply delay. 
        # Here, we have already aligned them to a common time. 
        # We need a way to apply delay compensation to already time-aligned data or re-align with new delay.
        # Let's re-use align_data_by_time with the new estimated_delay to get correctly delay-compensated aligned data.
        # This ensures consistency.
        aligned_in_final, aligned_out_final, aligned_t_final, actual_fs_final = \
            self.signal_analyzer.align_data_by_time(in_data, out_data, in_time, out_time, 
                                                    target_sample_rate=self.args.update_rate, estimated_delay=self.time_delay)

        if len(aligned_in_final) < 10:
            rospy.logerr("Not enough data after delay compensation and final alignment for calibration.")
            return False
        
        # Calculate static gain and bias using the *final* delay-compensated and aligned data
        est_gain, est_bias = self.signal_analyzer.calculate_static_gain_and_bias(aligned_in_final, aligned_out_final)
        self.static_gain = est_gain
        self.bias = est_bias
        rospy.loginfo(f"Est. Static Gain: {self.static_gain:.4f}, Bias: {self.bias:.4f}")

        # Visualization of calibration alignment
        if self.visualizer and not self.args.no_plots:
            # Plot 1: Raw input and output on their original time axes (or resampled for comparison)
            # Plot 2: Final aligned_in_final vs aligned_out_final (output is delay compensated by align_data_by_time)
            # For plot 2, the `aligned_input` for `visualize_time_alignment` should be `aligned_in_final`.
            # The `aligned_output` should be `aligned_out_final`.
            # The `compensated_input` in that plot is `static_gain * aligned_in_final + bias`.
            self.visualizer.visualize_time_alignment(
                in_time, in_data,          # Original input data
                out_time, out_data,        # Original output data
                aligned_t_final, aligned_in_final, aligned_out_final, # Final aligned & delay-compensated data
                self.time_delay, self.static_gain, self.bias
            )
            # Also, visualize the alignment verification
            self.visualizer.visualize_alignment_verification(
                in_time, in_data, out_time, out_data,
                aligned_in_final, aligned_out_final, # These are now the effectively aligned versions
                self.time_delay, self.args.plots_dir
            )
        return True

    def run_sweep_phase(self):
        self.clear_collected_data()
        rospy.loginfo("="*60)
        rospy.loginfo(self._get_label("开始频率扫描", "Starting frequency sweep phase"))
        rospy.loginfo(f"Type: {self.args.sweep_type}, Freq: {self.args.freq_start}-{self.args.freq_end} Hz, Dur: {self.args.sweep_duration}s, Amp: {self.args.sweep_amplitude}m")
        rospy.loginfo("="*60)

        quat = [0, -0.67566370964, 0, 0.73720997571945]
        elb_l = [0.08, 0.4, 0.18]; elb_r = [0.08, -0.4, 0.18]
        j_angs = [0]*7
        rate = rospy.Rate(self.args.update_rate)
        start_sweep_t = rospy.get_time()
        last_rep_sweep_t = start_sweep_t

        while not self.shutdown_requested and not rospy.is_shutdown():
            elap_sweep_t = rospy.get_time() - start_sweep_t
            if elap_sweep_t >= self.args.sweep_duration: break

            curr_freq = self.signal_analyzer.calculate_instantaneous_frequency(
                self.args.freq_start, self.args.freq_end, elap_sweep_t, self.args.sweep_duration, self.args.sweep_type)
            phase_val = self.signal_analyzer.calculate_chirp_phase(
                self.args.freq_start, self.args.freq_end, elap_sweep_t, self.args.sweep_duration, self.args.sweep_type)
            
            # Apply static gain compensation to the input amplitude if desired, or handle in analysis
            # For now, generate nominal amplitude, compensation will be in analysis.
            z_offset_sweep = self.args.sweep_amplitude * math.sin(phase_val)

            if rospy.get_time() - last_rep_sweep_t >= 1.0:
                rospy.loginfo(f"Sweep: {elap_sweep_t/self.args.sweep_duration*100:.1f}%, Freq: {curr_freq:.2f} Hz")
                last_rep_sweep_t = rospy.get_time()

            z_l_cmd_sweep = self.args.z_center + z_offset_sweep
            z_r_cmd_sweep = self.args.z_center
            self.publish_hand_pose(self.args.x_center, self.args.y_left, self.args.y_right, 
                                   z_l_cmd_sweep, z_r_cmd_sweep, quat, elb_l, elb_r, j_angs)
            rate.sleep()
        
        rospy.loginfo("Frequency sweep finished.")
        rospy.sleep(0.5) # Ensure last data points are captured
        return self.get_collected_data() # Return all four arrays

    def request_shutdown(self):
        self.shutdown_requested = True
        rospy.loginfo("MotionController: Shutdown requested.")
        if self.sub:
            self.sub.unregister()
            self.sub = None
            rospy.loginfo("MotionController: Subscriber unregistered.")

