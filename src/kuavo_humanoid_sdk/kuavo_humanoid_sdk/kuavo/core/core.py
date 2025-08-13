#!/usr/bin/env python3
# coding: utf-8
"""
This layer is responsible for robot state transitions.
The robot has three states:
- stance: Standing still state
- walk: Walking state 
- trot: Trotting state

State transitions are managed by a state machine that ensures valid transitions between states.
The state machine enforces the following transitions:
- stance <-> walk
- stance <-> trot
- walk <-> trot

Each state has an entry callback that handles initialization when entering that state.
"""


import time
import math
import threading
import numpy as np
from typing import Tuple
from transitions import Machine, State

from kuavo_humanoid_sdk.interfaces.data_types import KuavoArmCtrlMode, KuavoIKParams, KuavoPose, KuavoManipulationMpcFrame, KuavoManipulationMpcCtrlMode, KuavoManipulationMpcControlFlow
from kuavo_humanoid_sdk.kuavo.core.ros.control import KuavoRobotControl
from kuavo_humanoid_sdk.kuavo.core.ros.state import KuavoRobotStateCore
from kuavo_humanoid_sdk.kuavo.core.ros.param import make_robot_param
from kuavo_humanoid_sdk.common.logger import SDKLogger
# Define robot states
ROBOT_STATES = [
    State(name='stance', on_enter=['_on_enter_stance']),
    State(name='walk', on_enter=['_on_enter_walk']), 
    State(name='trot', on_enter=['_on_enter_trot']),
    State(name='custom_gait', on_enter=['_on_enter_custom_gait']),
    State(name='command_pose_world', on_enter=['_on_enter_command_pose_world']),
    State(name='command_pose', on_enter=['_on_enter_command_pose']),
]

# Define state transitions
ROBOT_TRANSITIONS = [
    {'trigger': 'to_stance', 'source': ['walk', 'trot', 'custom_gait', 'command_pose_world', 'command_pose'], 'dest': 'stance'},
    {'trigger': 'to_walk', 'source': ['stance', 'trot', 'custom_gait'], 'dest': 'walk'},
    {'trigger': 'to_trot', 'source': ['stance', 'walk', 'custom_gait'], 'dest': 'trot'},
    {'trigger': 'to_custom_gait', 'source': ['stance', 'custom_gait'], 'dest': 'custom_gait'},
    {'trigger': 'to_command_pose_world', 'source': ['stance', 'command_pose_world'], 'dest': 'command_pose_world'},
    {'trigger': 'to_command_pose', 'source': ['stance', 'command_pose'], 'dest': 'command_pose'},
]

class KuavoRobotCore:
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(KuavoRobotCore, cls).__new__(cls)
        return cls._instance
    
    def __init__(self):
        if not hasattr(self, '_initialized'):
            self.machine = Machine(
                model=self,
                states=ROBOT_STATES,
                transitions=ROBOT_TRANSITIONS,
                initial='stance',
                send_event=True
            )

            self._control = KuavoRobotControl()
            self._rb_state = KuavoRobotStateCore()
    
            # manipulation mpc
            self._manipulation_mpc_frame = KuavoManipulationMpcFrame.KeepCurrentFrame
            self._manipulation_mpc_ctrl_mode = KuavoManipulationMpcCtrlMode.NoControl
            self._manipulation_mpc_control_flow = KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
                
            self._arm_ctrl_mode = KuavoArmCtrlMode.AutoSwing
            
            # register gait changed callback
            self._rb_state.register_gait_changed_callback(self._humanoid_gait_changed)
            # initialized
            self._initialized = True

    def initialize(self, debug: bool=False)->bool:
        """
         raise RuntimeError if initialize failed.
        """
        try:
            # init state by gait_name
            gait_name = self._rb_state.gait_name()
            if gait_name is not None:
                to_method = f'to_{gait_name}'
                if hasattr(self, to_method):
                    SDKLogger.debug(f"[Core] initialize state: {gait_name}")
                    # Call the transition method if it exists
                    getattr(self, to_method)()
            else:
                SDKLogger.warn(f"[Core] gait_name is None, use default `stance`")
            # init arm control mode
            arm_ctrl_mode = self._rb_state.arm_control_mode
            if arm_ctrl_mode is not None:
                self._arm_ctrl_mode = arm_ctrl_mode
                SDKLogger.debug(f"[Core] initialize arm control mode: {arm_ctrl_mode}")
            
            # init manipulation mpc
            manipulation_mpc_frame = self._rb_state.manipulation_mpc_frame
            if manipulation_mpc_frame is not None:
                self._manipulation_mpc_frame = manipulation_mpc_frame
                SDKLogger.debug(f"[Core] initialize manipulation mpc frame: {manipulation_mpc_frame}")
            manipulation_mpc_ctrl_mode = self._rb_state.manipulation_mpc_ctrl_mode
            if manipulation_mpc_ctrl_mode is not None:
                self._manipulation_mpc_ctrl_mode = manipulation_mpc_ctrl_mode
                SDKLogger.debug(f"[Core] initialize manipulation mpc ctrl mode: {manipulation_mpc_ctrl_mode}")
            manipulation_mpc_control_flow = self._rb_state.manipulation_mpc_control_flow
            if manipulation_mpc_control_flow is not None:
                self._manipulation_mpc_control_flow = manipulation_mpc_control_flow
                SDKLogger.debug(f"[Core] initialize manipulation mpc control flow: {manipulation_mpc_control_flow}")
                
        except Exception as e:
            raise RuntimeError(f"[Core] initialize failed: \n"
                             f"{e}, please check the robot is launched, "
                             f"e.g. `roslaunch humanoid_controllers load_kuavo_real.launch`")
        rb_info = make_robot_param()
        success, err_msg = self._control.initialize(eef_type=rb_info["end_effector_type"], debug=debug)
        if not success:
            raise RuntimeError(f"[Core] initialize failed: \n{err_msg}, please check the robot is launched, "
                             f"e.g. `roslaunch humanoid_controllers load_kuavo_real.launch`")
        return True

    """ ----------------------- Machine State -----------------------"""
    def _on_enter_stance(self, event):
        previous_state = event.transition.source
        if self.state  == previous_state:
            SDKLogger.debug(f"[Core] [StateMachine] State unchanged: already in stance state")
            return
        
        SDKLogger.debug(f"[Core] [StateMachine] Entering stance state, from {previous_state}")
        if previous_state == 'walk':
            self._control.robot_walk(0.0, 0.0, 0.0) # stop walk state
            start_time = time.time()
            # slow down walk
            try:
                while time.time() - start_time < 1.5:
                    self._control.robot_walk(0.0, 0.0, 0.0)
                    # linear_x, linear_y, angular_z
                    if (abs(self._rb_state.odom_data.linear[0]) < 0.05 and abs(self._rb_state.odom_data.linear[1]) < 0.08 
                        and abs(self._rb_state.odom_data.angular[2]) < 0.05):
                        SDKLogger.debug(f"walk stop, time_cost:{time.time() - start_time}, odom_data:{self._rb_state.odom_data.linear}")
                        break
                    # SDKLogger.debug(f"kuavo robot linear: {self._rb_state.odom_data.linear}")
                    time.sleep(0.1)
            except KeyboardInterrupt:
                pass
            self._control.robot_stance()
        else:
            self._control.robot_stance() 
        time.sleep(0.5)

    def _on_enter_walk(self, event):
        previous_state = event.transition.source
        if self.state  == previous_state:
            SDKLogger.debug(f"[Core] [StateMachine] State unchanged: already in walk state")
            return
        SDKLogger.debug(f"[Core] [StateMachine] Entering walk state, from {previous_state}")

    def _on_enter_trot(self, event):
        previous_state = event.transition.source
        if self.state  == previous_state:
            SDKLogger.debug(f"[Core] [StateMachine] State unchanged: already in trot state")
            return
        SDKLogger.debug(f"[Core] [StateMachine] Entering trot state, from {previous_state}")
        self._control.robot_trot()

    def _on_enter_custom_gait(self, event):
        previous_state = event.transition.source
        if self.state  == previous_state:
            SDKLogger.debug(f"[Core] [StateMachine] State unchanged: already in custom_gait state")
            return
        SDKLogger.debug(f"[Core] [StateMachine] Entering custom_gait state, from {previous_state}")
    
    def _on_enter_command_pose_world(self, event):
        previous_state = event.transition.source
        if self.state  == previous_state:
            SDKLogger.debug(f"[Core] [StateMachine] State unchanged: already in command_pose_world state")
            return
        SDKLogger.debug(f"[Core] [StateMachine] Entering command_pose_world state, from {previous_state}")

    def _on_enter_command_pose(self, event):
        previous_state = event.transition.source
        if self.state  == previous_state:
            SDKLogger.debug(f"[Core] [StateMachine] State unchanged: already in command_pose state")
            return
        SDKLogger.debug(f"[Core] [StateMachine] Entering command_pose state, from {previous_state}")
        
    """ -------------------------------------------------------------"""

    """ --------------------------- Control -------------------------"""
    def walk(self, linear_x:float, linear_y:float, angular_z:float)-> bool:
        if self.state != 'walk':
            self.to_walk()
        
        # +-0.4, +-0.2, +-0.4 => linear_x, linear_y, angular_z
        limited_linear_x = min(0.4, abs(linear_x)) * (1 if linear_x >= 0 else -1)
        limited_linear_y = min(0.2, abs(linear_y)) * (1 if linear_y >= 0 else -1)
        limited_angular_z = min(0.4, abs(angular_z)) * (1 if angular_z >= 0 else -1)
        return self._control.robot_walk(limited_linear_x, limited_linear_y, limited_angular_z)
    
    def squat(self, height:float, pitch:float)->bool:
        if self.state != 'stance':
            SDKLogger.warn(f"[Core] control torso height failed, robot is not in stance state({self.state})!")
            return False
        
        MIN_HEIGHT = -0.35
        MAX_HEIGHT = 0.0
        MIN_PITCH = -0.4
        MAX_PITCH = 0.4
        
        # Limit height range
        limited_height = min(MAX_HEIGHT, max(MIN_HEIGHT, height))
        if height > MAX_HEIGHT or height < MIN_HEIGHT:
            SDKLogger.warn(f"[Core] height {height} exceeds limit [{MIN_HEIGHT}, {MAX_HEIGHT}], will be limited")
        
        # Limit pitch range
        limited_pitch = min(MAX_PITCH, max(MIN_PITCH, pitch))
        if abs(pitch) > MAX_PITCH:
            SDKLogger.warn(f"[Core] pitch {pitch} exceeds limit [{MIN_PITCH}, {MAX_PITCH}], will be limited")

        return self._control.control_torso_height(limited_height, limited_pitch)

    def step_control(self, target_pose:list, dt:float=0.4, is_left_first_default:bool=True, collision_check:bool=True)->bool:
        """
        Control the robot's motion by step.
        Raises:
            ValueError: If target_pose length is not 4.
            RuntimeError: If the robot is not in stance state when trying to control step motion.
        """
        if len(target_pose) != 4:
            raise ValueError(f"[Core] target_pose length must be 4, but got {len(target_pose)}")
    
        # Wait up to 1.0s for stance state
        wait_time = 0
        while self._rb_state.gait_name() != 'stance' and wait_time < 1.0:
            time.sleep(0.1)
            wait_time += 0.1
            
        if self._rb_state.gait_name() != 'stance':
            raise RuntimeError(f"[Core] control robot step failed, robot is not in stance state, {self._rb_state.gait_name()}!")

        if self.state != 'stance':
            raise RuntimeError(f"[Core] control robot step failed, robot is not in stance state({self.state})!")
        
        com_height = self._rb_state.com_height
        # print(f"[Core] Current COM height: {com_height:.2f}m")
        # Check height limits based on current COM height
        MIN_COM_HEIGHT = 0.66  # Minimum allowed COM height in meters
        MAX_COM_HEIGHT = 0.86  # Maximum allowed COM height in meters

        # Validate COM height constraints
        if target_pose[2] < 0 and com_height < MIN_COM_HEIGHT:
            SDKLogger.warn(f"[Core] Cannot squat lower: COM height {com_height:.2f}m below minimum {MIN_COM_HEIGHT}m")
            return None
        
        if target_pose[2] > 0 and com_height > MAX_COM_HEIGHT:
            SDKLogger.warn(f"[Core] Cannot stand higher: COM height {com_height:.2f}m above maximum {MAX_COM_HEIGHT}m")
            return None

        # Ensure target height is within allowed range if height change requested
        if target_pose[2] != 0:
            target_height = com_height + target_pose[2]
            if target_height < MIN_COM_HEIGHT:
                SDKLogger.warn(f"[Core] Target height {target_height:.2f}m below minimum {MIN_COM_HEIGHT}m, limiting")
                target_pose[2] = MIN_COM_HEIGHT - com_height
            elif target_height > MAX_COM_HEIGHT:
                SDKLogger.warn(f"[Core] Target height {target_height:.2f}m above maximum {MAX_COM_HEIGHT}m, limiting") 
                target_pose[2] = MAX_COM_HEIGHT - com_height
        
        # TODO(kuavo): 根据实物测试来调整....
        if com_height > 0.82:
            max_x_step = 0.20
            max_y_step = 0.20
            max_yaw_step = 90
        else:
            max_x_step = 0.15
            max_y_step = 0.15
            max_yaw_step = 45
        
        body_poses = []
        
        # 计算目标点到原点的距离和朝向
        target_dist_x = abs(target_pose[0])
        target_dist_y = abs(target_pose[1])
        target_yaw = target_pose[3] * 180 / math.pi  # Convert yaw from radians to degrees
        
        # 计算需要的步数(考虑x位移、y位移和转角)
        steps_for_x = int(np.ceil(target_dist_x / max_x_step))
        steps_for_y = int(np.ceil(target_dist_y / max_y_step))
        steps_for_yaw = int(np.ceil(abs(target_yaw) / max_yaw_step))
        steps_needed = max(steps_for_x, steps_for_y, steps_for_yaw)
        # print(f"[Core] Steps needed - X: {steps_for_x}, Y: {steps_for_y}, Yaw: {steps_for_yaw}, Total: {steps_needed}")
        
        # 计算每一步的增量
        dx = target_pose[0] / steps_needed
        dy = target_pose[1] / steps_needed
        dyaw = target_yaw / steps_needed
        
        # 分解为多个小步,沿着直线路径前进并逐步调整朝向
        for i in range(steps_needed):
            x = dx * (i+1)
            y = dy * (i+1)
            z = target_pose[2]
            yaw = dyaw * (i+1)
            body_poses.append([x, y, 0.0, yaw])
        
        # print("target_pose:", target_pose)
        # print("body_poses:", body_poses)

        if not self._control.step_control(body_poses, dt, is_left_first_default, collision_check):
            return False
        
        # # Wait for gait to switch to custom_gait
        # start_time = time.time()
        # SDKLogger.warning("-------------------STEP CONTROL -----------------------------")
        # while not self._rb_state.is_gait('custom_gait'):
        #     if time.time() - start_time > 1.0:  # 1.0s timeout
        #         SDKLogger.warn("[Core] Timeout waiting for gait to switch to custom_gait")
        #         return False
        #     time.sleep(0.01)
        # SDKLogger.warning("-------------------STEP CONTROL -----------------------------")

        return True

    def control_command_pose(self, target_pose_x:float, target_pose_y:float, target_pose_z:float, target_pose_yaw:float)->bool:
        """
        Control robot pose in base_link frame
        
        Arguments:
            - target_pose_x: x position (meters)
            - target_pose_y: y position (meters)
            - target_pose_z: z position (meters)
            - target_pose_yaw: yaw angle (radians)
        
        Returns:
            bool: True if command was sent successfully, False otherwise
            
        Raises:
            RuntimeError: If robot is not in stance state
        """
        if self.state != 'stance':
            raise RuntimeError(f"[Core] control_command_pose failed: robot must be in stance state, current state: {self.state}")
        
        # Add any parameter validation if needed
        # e.g., limit ranges for safety
        self.to_command_pose()
        return self._control.control_command_pose(target_pose_x, target_pose_y, target_pose_z, target_pose_yaw)

    def control_command_pose_world(self, target_pose_x:float, target_pose_y:float, target_pose_z:float, target_pose_yaw:float)->bool:
        """
        Control robot pose in odom (world) frame
        
        Arguments:
            - target_pose_x: x position (meters)
            - target_pose_y: y position (meters)
            - target_pose_z: z position (meters)
            - target_pose_yaw: yaw angle (radians)
        
        Returns:
            bool: True if command was sent successfully, False otherwise
            
        Raises:
            RuntimeError: If robot is not in stance state
        """
        # if self.state != 'stance':
        #     raise RuntimeError(f"[Core] control_command_pose_world failed: robot must be in stance state, current state: {self.state}")
        
        # Add any parameter validation if needed
        # e.g., limit ranges for safety
        self.to_command_pose_world()
        return self._control.control_command_pose_world(target_pose_x, target_pose_y, target_pose_z, target_pose_yaw)
    
    def execute_gesture(self, gestures:list)->bool:
        return self._control.execute_gesture(gestures)
    
    def get_gesture_names(self)->list:
        return self._control.get_gesture_names()

    def control_robot_dexhand(self, left_position:list, right_position:list)->bool:
        return self._control.control_robot_dexhand(left_position, right_position)
    
    def robot_dexhand_command(self, data, ctrl_mode, hand_side):
         return self._control.robot_dexhand_command(data, ctrl_mode, hand_side)


    def control_leju_claw(self, postions:list, velocities:list=[90, 90], torques:list=[1.0, 1.0]) ->bool:
        return self._control.control_leju_claw(postions, velocities, torques)

    def control_robot_head(self, yaw:float, pitch:float)->bool:
        # Convert radians to degrees
        yaw_deg = yaw * 180 / math.pi
        pitch_deg = pitch * 180 / math.pi
        return self._control.control_robot_head(yaw_deg, pitch_deg)
    
    def enable_head_tracking(self, target_id: int)->bool:
        return self._control.enable_head_tracking(target_id)
    
    def disable_head_tracking(self)->bool:
        return self._control.disable_head_tracking()
    
    def control_robot_arm_joint_positions(self, joint_data:list)->bool:
        if self.state != 'stance':
            raise RuntimeError(f"[Core] control_robot_arm_joint_positions failed: robot must be in stance state, current state: {self.state}")
        
        if self._control.is_arm_collision_mode() and self._control.is_arm_collision():
            raise RuntimeError(f"Arm collision detected, cannot publish arm trajectory")
        # change to external control mode  
        if self._arm_ctrl_mode != KuavoArmCtrlMode.ExternalControl:
            SDKLogger.debug("[Core] control_robot_arm_joint_positions, current arm mode != ExternalControl, change it.")
            if not self.change_robot_arm_ctrl_mode(KuavoArmCtrlMode.ExternalControl):
                SDKLogger.warn("[Core] control_robot_arm_joint_positions failed, change robot arm ctrl mode failed!")
                return False
        return self._control.control_robot_arm_joint_positions(joint_data)
    
    def control_robot_arm_joint_trajectory(self, times:list, joint_q:list)->bool:
        if self.state != 'stance':
            raise RuntimeError("[Core] control_robot_arm_joint_trajectory failed: robot must be in stance state")
            
        if self._control.is_arm_collision_mode() and self._control.is_arm_collision():
            raise RuntimeError(f"Arm collision detected, cannot publish arm trajectory")
        
        if self._arm_ctrl_mode != KuavoArmCtrlMode.ExternalControl:
            SDKLogger.debug("[Core] control_robot_arm_joint_trajectory, current arm mode != ExternalControl, change it.")
            if not self.change_robot_arm_ctrl_mode(KuavoArmCtrlMode.ExternalControl):
                SDKLogger.warn("[Core] control_robot_arm_joint_trajectory failed, change robot arm ctrl mode failed!")
                return False
            
        return self._control.control_robot_arm_joint_trajectory(times, joint_q)
    
    def control_robot_end_effector_pose(self, left_pose: KuavoPose, right_pose: KuavoPose, frame: KuavoManipulationMpcFrame)->bool:        
        if self._arm_ctrl_mode != KuavoArmCtrlMode.ExternalControl:
            SDKLogger.debug("[Core] control_robot_end_effector_pose, current arm mode != ExternalControl, change it.")
            if not self.change_robot_arm_ctrl_mode(KuavoArmCtrlMode.ExternalControl):
                SDKLogger.warn("[Core] control_robot_end_effector_pose failed, change robot arm ctrl mode failed!")
                return False

        if self._manipulation_mpc_ctrl_mode == KuavoManipulationMpcCtrlMode.NoControl:
            SDKLogger.debug("[Core] control_robot_end_effector_pose, manipulation mpc ctrl mode is NoControl, change it.")
            if not self.change_manipulation_mpc_ctrl_mode(KuavoManipulationMpcCtrlMode.ArmOnly):
                SDKLogger.warn("[Core] control_robot_end_effector_pose failed, change manipulation mpc ctrl mode failed!")
                return False
        
        return self._control.control_robot_end_effector_pose(left_pose, right_pose, frame)

    def control_hand_wrench(self, left_wrench: list, right_wrench: list) -> bool:
        return self._control.control_hand_wrench(left_wrench, right_wrench)
    
    def change_manipulation_mpc_frame(self, frame: KuavoManipulationMpcFrame)->bool:
        timeout = 1.0
        count = 0
        while self._rb_state.manipulation_mpc_frame != frame:
            SDKLogger.debug(f"[Core] Change manipulation mpc frame from {self._rb_state.manipulation_mpc_frame} to {frame}, retry: {count}")
            self._control.change_manipulation_mpc_frame(frame)
            if self._rb_state.manipulation_mpc_frame == frame:
                break
            if timeout <= 0:
                SDKLogger.warn("[Core] Change manipulation mpc frame timeout!")
                return False
            timeout -= 0.1
            time.sleep(0.1)
            count += 1
        if not hasattr(self, '_manipulation_mpc_frame_lock'):
            self._manipulation_mpc_frame_lock = threading.Lock()
        with self._manipulation_mpc_frame_lock:
            self._manipulation_mpc_frame = frame
        return True
    
    def change_manipulation_mpc_ctrl_mode(self, control_mode: KuavoManipulationMpcCtrlMode)->bool:
        timeout = 1.0
        count = 0
        while self._rb_state.manipulation_mpc_ctrl_mode != control_mode:
            SDKLogger.debug(f"[Core] Change manipulation mpc ctrl mode from {self._rb_state.manipulation_mpc_ctrl_mode} to {control_mode}, retry: {count}")
            self._control.change_manipulation_mpc_ctrl_mode(control_mode)
            if self._rb_state.manipulation_mpc_ctrl_mode == control_mode:
                break
            if timeout <= 0:
                SDKLogger.warn("[Core] Change manipulation mpc ctrl mode timeout!")
                return False
            timeout -= 0.1
            time.sleep(0.1)
            count += 1
        if not hasattr(self, '_manipulation_mpc_ctrl_mode_lock'):
            self._manipulation_mpc_ctrl_mode_lock = threading.Lock()
        with self._manipulation_mpc_ctrl_mode_lock:
            self._manipulation_mpc_ctrl_mode = control_mode
        return True
    
    def change_manipulation_mpc_control_flow(self, control_flow: KuavoManipulationMpcControlFlow)->bool:
        timeout = 1.0
        count = 0
        while self._rb_state.manipulation_mpc_control_flow != control_flow:
            SDKLogger.debug(f"[Core] Change manipulation mpc control flow from {self._rb_state.manipulation_mpc_control_flow} to {control_flow}, retry: {count}")
            self._control.change_manipulation_mpc_control_flow(control_flow)
            if self._rb_state.manipulation_mpc_control_flow == control_flow:
                break
            if timeout <= 0:
                SDKLogger.warn("[Core] Change manipulation mpc control flow timeout!")
                return False
            timeout -= 0.1
            time.sleep(0.1)
            count += 1
        if not hasattr(self, '_manipulation_mpc_control_flow_lock'):
            self._manipulation_mpc_control_flow_lock = threading.Lock()
        with self._manipulation_mpc_control_flow_lock:
            self._manipulation_mpc_control_flow = control_flow
        return True
    
    def change_robot_arm_ctrl_mode(self, mode:KuavoArmCtrlMode)->bool:

        if self._control.is_arm_collision_mode() and self.is_arm_collision():
            SDKLogger.warn("[Core] change_robot_arm_ctrl_mode failed, arm collision detected!")
            return False

        count = 0
        if self._rb_state.arm_control_mode != mode:
            SDKLogger.debug(f"[Core] Change robot arm control  from {self._rb_state.arm_control_mode} to {mode}, retry: {count}")
            self._control.change_robot_arm_ctrl_mode(mode)
        
        if not hasattr(self, '_arm_ctrl_mode_lock'):
            self._arm_ctrl_mode_lock = threading.Lock()
        with self._arm_ctrl_mode_lock:
            # 手臂控制模式切换成功，更新当前手臂控制模式
            self._arm_ctrl_mode = mode # update arm ctrl mode

        return True
    
    def robot_arm_reset(self)->bool:
        if self.state != 'stance':
            SDKLogger.warn("[Core] robot arm reset failed, robot is not in stance state!")
            return
        
        # init_pos = [0.0]*14
        # if not self.control_robot_arm_joint_trajectory([1.5], [init_pos]):
        #     SDKLogger.warn("[Core] robot arm reset failed, control robot arm traj failed!")
        #     return False
        
        return self.change_robot_arm_ctrl_mode(KuavoArmCtrlMode.AutoSwing)
        
    def robot_manipulation_mpc_reset(self)->bool:
        if self._manipulation_mpc_ctrl_mode != KuavoManipulationMpcCtrlMode.NoControl:
            SDKLogger.debug("[Core] robot manipulation mpc reset, current manipulation mpc ctrl mode != NoControl, change it.")
            if not self.change_manipulation_mpc_ctrl_mode(KuavoManipulationMpcCtrlMode.NoControl):
                SDKLogger.warn("[Core] robot manipulation mpc reset failed, change manipulation mpc ctrl mode failed!")
                return False
        if self._manipulation_mpc_control_flow != KuavoManipulationMpcControlFlow.ThroughFullBodyMpc:
            SDKLogger.debug("[Core] robot manipulation mpc reset, current manipulation mpc control flow != ThroughFullBodyMpc, change it.")
            if not self.change_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.ThroughFullBodyMpc):
                SDKLogger.warn("[Core] robot manipulation mpc reset failed, change manipulation mpc control flow failed!")
                return False
        return True
    """ ------------------------------------------------------------------------"""
    """ 电机参数设置 """
    def change_motor_param(self, motor_param:list)-> Tuple[bool, str]:
        return self._control.change_motor_param(motor_param)
    
    def get_motor_param(self)-> Tuple[bool, list]:
        success, param, _ = self._control.get_motor_param()
        return success, param

    """ ------------------------------------------------------------------------"""
    """ Arm Forward kinematics && Arm Inverse kinematics """
    def arm_ik(self, 
               l_eef_pose: KuavoPose, 
               r_eef_pose: KuavoPose, 
               l_elbow_pos_xyz: list = [0.0, 0.0, 0.0],
               r_elbow_pos_xyz: list = [0.0, 0.0, 0.0],
               arm_q0: list = None,
               params: KuavoIKParams=None) -> list:
        return self._control.arm_ik(l_eef_pose, r_eef_pose, l_elbow_pos_xyz, r_elbow_pos_xyz, arm_q0, params)
    

    def arm_fk(self, q: list) -> Tuple[KuavoPose, KuavoPose]:
        return self._control.arm_fk(q)
    
    """ ------------------------------------------------------------------------"""
    """ Base Pitch Limit Control """
    def enable_base_pitch_limit(self, enable: bool) -> Tuple[bool, str]:
        return self._control.enable_base_pitch_limit(enable)
    """ ------------------------------------------------------------------------"""
    """ Callbacks """
    def _humanoid_gait_changed(self, current_time: float, gait_name: str):
        if self.state != gait_name:
            # Check if to_$gait_name method exists
            to_method = f'to_{gait_name}'
            if hasattr(self, to_method):
                SDKLogger.debug(f"[Core] Received gait change notification: {gait_name} at time {current_time}")
                # Call the transition method if it exists
                getattr(self, to_method)()

    def is_arm_collision(self)->bool:
        return self._control.is_arm_collision()
    
    def release_arm_collision_mode(self):

        self._control.release_arm_collision_mode()
        

    def wait_arm_collision_complete(self):
        self._control.wait_arm_collision_complete()

    def set_arm_collision_mode(self, enable: bool):
        self._control.set_arm_collision_mode(enable)

if __name__ == "__main__":
    DEBUG_MODE = 0
    core = KuavoRobotCore()
    core.initialize()

    if DEBUG_MODE == 0:
        time.sleep(1.0)
        core.change_robot_arm_ctrl_mode(KuavoArmCtrlMode.ExternalControl)
        core.change_manipulation_mpc_frame(KuavoManipulationMpcFrame.VRFrame)
        core.change_manipulation_mpc_ctrl_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
        core.change_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.DirectToWbc)
        core.robot_manipulation_mpc_reset()
    elif DEBUG_MODE == 1:
        core.to_stance()
        print("state now is to_stance:", core.state)
        core.control_command_pose_world(0.0, 1.0, 0.0, 1.57)
        print("state now is control_command_pose_world:", core.state)
    elif DEBUG_MODE == 2:
        core.to_trot()
        print("state now is to_trot:", core.state)
        time.sleep(3.0)
        core.to_stance()
        print("state now is to_stance:", core.state)