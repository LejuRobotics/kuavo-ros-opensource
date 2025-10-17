#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.kuavo.core.ros_env import  KuavoROSEnvWebsocket
from kuavo_humanoid_sdk.interfaces.robot import RobotBase
from kuavo_humanoid_sdk.common.logger import SDKLogger, disable_sdk_logging
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, KuavoIKParams, KuavoManipulationMpcFrame, KuavoManipulationMpcCtrlMode, KuavoManipulationMpcControlFlow
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore

from typing import Tuple
from kuavo_humanoid_sdk.kuavo.robot_info import KuavoRobotInfo
from kuavo_humanoid_sdk.kuavo.robot_arm import KuavoRobotArm 
from kuavo_humanoid_sdk.kuavo.robot_head import KuavoRobotHead
from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK
from kuavo_humanoid_sdk.common.launch_robot_tool import LaunchRobotTool

"""
Kuavo SDK - Python Interface for Kuavo Robot Control

This module provides the main interface for controlling Kuavo robots through Python.
It includes two main classes:

KuavoSDK:
    A static class providing SDK initialization and configuration functionality.
    Handles core setup, ROS environment initialization, and logging configuration.

KuavoRobot:
    The main robot control interface class that provides access to:
    - Robot information and status (via KuavoRobotInfo)
    - Arm control capabilities (via KuavoRobotArm)
    - Head control capabilities (via KuavoRobotHead)
    - Core robot functions (via KuavoRobotCore)
    
The module requires a properly configured ROS environment to function.
"""
__all__ = ["KuavoSDK", "KuavoRobot"]


class KuavoSDK:
    class Options:
        Normal = 0x01
        WithIK = 0x02

    def __init__(self):
       pass

    @staticmethod   
    def Init(options:int=Options.Normal, log_level: str = "INFO", 
             websocket_mode: bool = False, websocket_host='127.0.0.1', websocket_port=9090, websocket_timeout=5)-> bool:
        """Initialize the SDK.
        
        Initializes the Kuavo SDK with specified options and configuration.
        
        Args:
            options (int): Configuration options for the SDK. Use KuavoSDK.Options constants.
                Defaults to Options.Normal.
            log_level (str): Logging level to use. Options are "ERROR", "WARN", "INFO", "DEBUG".
                Defaults to "INFO".
            websocket_mode (bool): Whether to use websocket mode for ROS communication.
                If True, connects to ROS through websocket instead of direct connection.
                Defaults to False.
            websocket_host (str): Host address of the rosbridge websocket server.
                Only used when websocket_mode is True. Defaults to "127.0.0.1".
            websocket_port (int): Port number of the rosbridge websocket server.
                Only used when websocket_mode is True. Defaults to 9090.
            websocket_timeout (int): Timeout in seconds for the websocket connection.
                Defaults to 5.
                
        Returns:
            bool: True if initialization succeeds, False otherwise.
            
        Raises:
            RuntimeError: If initialization fails due to missing dependencies or
                connection issues.
        """
        WebSocketKuavoSDK.use_websocket = websocket_mode
        WebSocketKuavoSDK.websocket_host = websocket_host
        WebSocketKuavoSDK.websocket_port = websocket_port
        WebSocketKuavoSDK.websocket_timeout = websocket_timeout
        SDKLogger.setLevel(log_level.upper())
        SDKLogger.debug(f" ================= Kuavo Humanoid Websocket SDK =================")

        def launch_robot()->bool:
            import time
            import os
            start_time = time.time()
            launch_robot_tool = LaunchRobotTool()
            
            # 获取机器人启动状态
            success, status = launch_robot_tool.get_robot_launch_status()
            if not success:
                # 可能websocket服务未启动或网络不通原因
                SDKLogger.error(f"[SDK] Failed to get robot launch status: {status}")
                return False

            SDKLogger.info(f"[SDK] Robot launch status: {status}, time elapsed: {time.time() - start_time:.2f}s")

            # 如果机器人已经启动，则直接返回True
            if launch_robot_tool.is_launched(status):
                return True
            
            # 未启动则启动
            if launch_robot_tool.is_unlaunch(status):
                print("\033[32m\n>_ 温馨提示: 机器人未启动，现在可以选择手动启动，如果需要请输入 y ，否则输入 n 退出\033[0m")
                try:
                    while True:
                        choice = input().lower()
                        if choice == 'y':
                            if not launch_robot_tool.robot_start():
                                SDKLogger.error("机器人启动失败，请检查机器人状态")
                                os._exit(1)
                            break
                        elif choice == 'n':
                            SDKLogger.warning("用户选择不启动机器人，程序退出。")
                            os._exit(1)
                        else:
                            SDKLogger.error("输入错误，请输入 y 或 n")
                except KeyboardInterrupt:
                    os._exit(1)
            
            SDKLogger.debug(f"等待机器人启动，已耗时: {time.time() - start_time:.2f}s")
           
            if not launch_robot_tool.robot_stand():
                SDKLogger.error("机器人站立失败，请检查机器人状态")
                os._exit(1)

            end_time = time.time()
            SDKLogger.info(f"[SDK] Robot launch time: {end_time - start_time} seconds")
            return True
        
        # Launch robot
        if not launch_robot():
            SDKLogger.error("[SDK] Failed to launch robot.")
            return False

        # Initialize core components, connect ROS Topics...
        kuavo_core = KuavoRobotCore()
        if log_level.upper() == 'DEBUG':
            debug = True
        else:
            debug = False   
        # Check if IK option is enabled
        if options & KuavoSDK.Options.WithIK:
            if not KuavoROSEnvWebsocket.check_rosnode_exists('/arms_ik_node'):
                print("\033[31m\nError:WithIK option is enabled but ik_node is not running, please run `roslaunch motion_capture_ik ik_node.launch`\033[0m")      
                exit(1)

        if not kuavo_core.initialize(debug=debug):
            SDKLogger.error("[SDK] Failed to initialize core components.")
            return False
        
        return True
    
    @staticmethod
    def DisableLogging():
        """Disable all logging output from the SDK."""
        disable_sdk_logging()

    @staticmethod
    def StopRobot()->bool:
        """Stop the robot."""
        launch_robot_tool = LaunchRobotTool()
        success, status = launch_robot_tool.get_robot_launch_status()
        if not success:
            SDKLogger.error(f"[SDK] Failed to get robot launch status: {status}")
            return False
        if launch_robot_tool.is_unlaunch(status):
            return True
        return launch_robot_tool.robot_stop()

class KuavoRobot(RobotBase):
    def __init__(self):
        super().__init__(robot_type="kuavo")
        
        kuavo_ros_env = KuavoROSEnvWebsocket()
        if not kuavo_ros_env.Init():
            raise RuntimeError("Failed to initialize ROS environment")

        self._robot_info = KuavoRobotInfo()
        self._robot_arm  = KuavoRobotArm()
        self._robot_head = KuavoRobotHead()
        self._kuavo_core = KuavoRobotCore()
    def stance(self)->bool:
        """Put the robot into 'stance' mode.
        
        Returns:
            bool: True if the robot is put into stance mode successfully, False otherwise.
            
        Note:
            You can call :meth:`KuavoRobotState.wait_for_stance` to wait until the robot enters stance mode.
        """
        return self._kuavo_core.to_stance()
        
    def trot(self)->bool:
        """Put the robot into 'trot' mode.
        
        Returns:
            bool: True if the robot is put into trot mode successfully, False otherwise.
            
        Note:
            You can call :meth:`KuavoRobotState.wait_for_walk` to wait until the robot enters trot mode.
        """
        return self._kuavo_core.to_trot()

    def walk(self, linear_x:float, linear_y:float, angular_z:float)->bool:
        """Control the robot's motion.
        
        Args:
            linear_x (float): The linear velocity along the x-axis in m/s, range [-0.4, 0.4].
            linear_y (float): The linear velocity along the y-axis in m/s, range [-0.2, 0.2].
            angular_z (float): The angular velocity around the z-axis in rad/s, range [-0.4, 0.4].
            
        Returns:
            bool: True if the motion is controlled successfully, False otherwise.
            
        Note:
            You can call :meth:`KuavoRobotState.wait_for_walk` to wait until the robot enters walk mode.
        """
        # Limit velocity ranges
        limited_linear_x = min(0.4, max(-0.4, linear_x))
        limited_linear_y = min(0.2, max(-0.2, linear_y)) 
        limited_angular_z = min(0.4, max(-0.4, angular_z))
        
        # Check if any velocity exceeds limits.
        if abs(linear_x) > 0.4:
            SDKLogger.warn(f"[Robot] linear_x velocity {linear_x} exceeds limit [-0.4, 0.4], will be limited")
        if abs(linear_y) > 0.2:
            SDKLogger.warn(f"[Robot] linear_y velocity {linear_y} exceeds limit [-0.2, 0.2], will be limited")
        if abs(angular_z) > 0.4:
            SDKLogger.warn(f"[Robot] angular_z velocity {angular_z} exceeds limit [-0.4, 0.4], will be limited")
        return self._kuavo_core.walk(limited_linear_x, limited_linear_y, limited_angular_z)

    def jump(self):
        """Jump the robot."""
        pass

    def squat(self, height: float, pitch: float=0.0)->bool:
        """Control the robot's squat height and pitch.
        Args:
                height (float): The height offset from normal standing height in meters, range [-0.35, 0.0],Negative values indicate squatting down.
                pitch (float): The pitch angle of the robot's torso in radians, range [-0.4, 0.4].
            
        Returns:
            bool: True if the squat is controlled successfully, False otherwise.
        """
        # Limit height range
        MAX_HEIGHT = 0.0
        MIN_HEIGHT = -0.35
        MAX_PITCH = 0.4
        MIN_PITCH = -0.4
        
        limited_height = min(MAX_HEIGHT, max(MIN_HEIGHT, height))
        
        # Check if height exceeds limits
        if height > MAX_HEIGHT or height < MIN_HEIGHT:
            SDKLogger.warn(f"[Robot] height {height} exceeds limit [{MIN_HEIGHT}, {MAX_HEIGHT}], will be limited")
        # Limit pitch range
        limited_pitch = min(MAX_PITCH, max(MIN_PITCH, pitch))
        
        # Check if pitch exceeds limits
        if abs(pitch) > MAX_PITCH:
            SDKLogger.warn(f"[Robot] pitch {pitch} exceeds limit [{MIN_PITCH}, {MAX_PITCH}], will be limited")
        
        return self._kuavo_core.squat(limited_height, limited_pitch)
     
    def step_by_step(self, target_pose:list, dt:float=0.4, is_left_first_default:bool=True, collision_check:bool=True)->bool:
        """Control the robot's motion by step.
        
        Args:
            target_pose (list): The target position of the robot in [x, y, z, yaw] m, rad.
            dt (float): The time interval between each step in seconds. Defaults to 0.4s.
            is_left_first_default (bool): Whether to start with the left foot. Defaults to True.
            collision_check (bool): Whether to check for collisions. Defaults to True.
            
        Returns:
            bool: True if motion is successful, False otherwise.
            
        Raises:
            RuntimeError: If the robot is not in stance state when trying to control step motion.
            ValueError: If target_pose length is not 4.

        Note:
            You can call :meth:`KuavoRobotState.wait_for_step_control` to wait until the robot enters step-control mode.
            You can call :meth:`KuavoRobotState.wait_for_stance` to wait the step-control finish.
        """    
        if len(target_pose) != 4:
            raise ValueError(f"[Robot] target_pose length must be 4 (x, y, z, yaw), but got {len(target_pose)}")

        return self._kuavo_core.step_control(target_pose, dt, is_left_first_default, collision_check)

    def control_command_pose(self, target_pose_x: float, target_pose_y: float, target_pose_z: float, target_pose_yaw: float) -> bool:
        """Control robot pose in base_link frame.
        
        Args:
            target_pose_x (float): The target x position in meters.
            target_pose_y (float): The target y position in meters.
            target_pose_z (float): The target z position in meters.
            target_pose_yaw (float): The target yaw angle in radians.
            
        Returns:
            bool: True if the command was sent successfully, False otherwise.
            
        Raises:
            RuntimeError: If the robot is not in stance state when trying to control pose.
            
        Note:
            This command changes the robot state to 'command_pose'.
        """
        return self._kuavo_core.control_command_pose(target_pose_x, target_pose_y, target_pose_z, target_pose_yaw)

    def control_command_pose_world(self, target_pose_x: float, target_pose_y: float, target_pose_z: float, target_pose_yaw: float) -> bool:
        """Control robot pose in odom (world) frame.
        
        Args:
            target_pose_x (float): The target x position in meters.
            target_pose_y (float): The target y position in meters.
            target_pose_z (float): The target z position in meters.
            target_pose_yaw (float): The target yaw angle in radians.
            
        Returns:
            bool: True if the command was sent successfully, False otherwise.
            
        Raises:
            RuntimeError: If the robot is not in stance state when trying to control pose.
            
        Note:
            This command changes the robot state to 'command_pose_world'.
        """
        return self._kuavo_core.control_command_pose_world(target_pose_x, target_pose_y, target_pose_z, target_pose_yaw)
    
    def control_head(self, yaw: float, pitch: float)->bool:
        """Control the head of the robot.
        
        Args:
            yaw (float): The yaw angle of the head in radians, range [-1.396, 1.396] (-80 to 80 degrees).
            pitch (float): The pitch angle of the head in radians, range [-0.436, 0.436] (-25 to 25 degrees).
            
        Returns:
            bool: True if the head is controlled successfully, False otherwise.
        """
        return self._robot_head.control_head(yaw=yaw, pitch=pitch)
    
    def enable_head_tracking(self, target_id: int)->bool:
        """Enable the head tracking.
        """
        return self._robot_head.enable_head_tracking(target_id)
    
    def disable_head_tracking(self)->bool:
        """Disable the head tracking.
        """
        return self._robot_head.disable_head_tracking()
    
    """ Robot Arm Control """
    def arm_reset(self)->bool:
        """Reset the robot arm.
        
        Returns:
            bool: True if the arm is reset successfully, False otherwise.
        """
        return self._robot_arm.arm_reset()
    
    def manipulation_mpc_reset(self)->bool:
        """Reset the robot arm.
        
        Returns:
            bool: True if the arm is reset successfully, False otherwise.
        """
        return self._robot_arm.manipulation_mpc_reset()
    
    def control_arm_joint_positions(self, joint_positions:list)->bool:
        """Control the position of the arm.
        
        Args:
            joint_positions (list): The target position of the arm in radians.
            
        Returns:
            bool: True if the arm is controlled successfully, False otherwise.
            
        Raises:
            ValueError: If the joint position list is not of the correct length.
            ValueError: If the joint position is outside the range of [-π, π].
            RuntimeError: If the robot is not in stance state when trying to control the arm.
        """
        if len(joint_positions) != self._robot_info.arm_joint_dof:
            print("The length of the position list must be equal to the number of DOFs of the arm.")
            return False
        
        return self._robot_arm.control_arm_joint_positions(joint_positions)
    
    def control_arm_joint_trajectory(self, times:list, q_frames:list)->bool:
        """Control the target poses of the robot arm.
        
        Args:
            times (list): List of time intervals in seconds.
            q_frames (list): List of joint positions in radians.
            
        Returns:
            bool: True if the control was successful, False otherwise.
            
        Raises:
            ValueError: If the times list is not of the correct length.
            ValueError: If the joint position list is not of the correct length.
            ValueError: If the joint position is outside the range of [-π, π].
            RuntimeError: If the robot is not in stance state when trying to control the arm.
            
        Warning:
            This is an asynchronous interface. The function returns immediately after sending the command.
            Users need to wait for the motion to complete on their own.
        """
        return self._robot_arm.control_arm_joint_trajectory(times, q_frames)

    def set_fixed_arm_mode(self) -> bool:
        """Freezes the robot arm.
        
        Returns:
            bool: True if the arm is frozen successfully, False otherwise.
        """
        return self._robot_arm.set_fixed_arm_mode()

    def set_auto_swing_arm_mode(self) -> bool:
        """Swing the robot arm.
        
        Returns:
            bool: True if the arm is swinging successfully, False otherwise.
        """
        return self._robot_arm.set_auto_swing_arm_mode()
    
    def set_external_control_arm_mode(self) -> bool:
        """External control the robot arm.
        
        Returns:
            bool: True if the arm is external controlled successfully, False otherwise.
        """
        return self._robot_arm.set_external_control_arm_mode()
    
    def set_manipulation_mpc_mode(self, ctrl_mode: KuavoManipulationMpcCtrlMode) -> bool:
        """
        Set the manipulation mpc mode.
        Returns:
            bool: True if the manipulation mpc mode is set successfully, False otherwise.
        """
        return self._robot_arm.set_manipulation_mpc_mode(ctrl_mode)
    
    def set_manipulation_mpc_control_flow(self, control_flow: KuavoManipulationMpcControlFlow) -> bool:
        """
        Set the manipulation mpc control flow.
        Returns:
            bool: True if the manipulation mpc control flow is set successfully, False otherwise.
        """ 
        return self._robot_arm.set_manipulation_mpc_control_flow(control_flow)

    def set_manipulation_mpc_frame(self, frame: KuavoManipulationMpcFrame) -> bool:
        """
        Set the manipulation mpc frame.
        Returns:
            bool: True if the manipulation mpc frame is set successfully, False otherwise.
        """
        return self._robot_arm.set_manipulation_mpc_frame(frame)
    
    """ Arm Forward kinematics && Arm Inverse kinematics """
    def arm_ik(self, 
               left_pose: KuavoPose, 
               right_pose: KuavoPose,
               left_elbow_pos_xyz: list = [0.0, 0.0, 0.0],
               right_elbow_pos_xyz: list = [0.0, 0.0, 0.0],
               arm_q0: list = None,
               params: KuavoIKParams=None) -> list:
        """Inverse kinematics for the robot arm.
        
        Args:
            left_pose (KuavoPose): Pose of the robot left arm, xyz and quat.
            right_pose (KuavoPose): Pose of the robot right arm, xyz and quat.
            left_elbow_pos_xyz (list): Position of the robot left elbow. If [0.0, 0.0, 0.0], will be ignored.
            right_elbow_pos_xyz (list): Position of the robot right elbow. If [0.0, 0.0, 0.0], will be ignored.
            arm_q0 (list, optional): Initial joint positions in radians. If None, will be ignored.
            params (KuavoIKParams, optional): Parameters for the inverse kinematics. If None, will be ignored.
                Contains:
                - major_optimality_tol: Major optimality tolerance
                - major_feasibility_tol: Major feasibility tolerance
                - minor_feasibility_tol: Minor feasibility tolerance
                - major_iterations_limit: Major iterations limit
                - oritation_constraint_tol: Orientation constraint tolerance
                - pos_constraint_tol: Position constraint tolerance, works when pos_cost_weight==0.0
                - pos_cost_weight: Position cost weight. Set to 0.0 for high accuracy
                
        Returns:
            list: List of joint positions in radians, or None if inverse kinematics failed.

        Warning:
            This function requires initializing the SDK with the :attr:`KuavoSDK.Options.WithIK`.
        """
        return self._robot_arm.arm_ik(left_pose, right_pose, left_elbow_pos_xyz, right_elbow_pos_xyz, arm_q0, params)

    def arm_fk(self, q: list) -> Tuple[KuavoPose, KuavoPose]:
        """Forward kinematics for the robot arm.
        
        Args:
            q (list): List of joint positions in radians.
            
        Returns:
            Tuple[KuavoPose, KuavoPose]: Tuple of poses for the robot left arm and right arm,
                or (None, None) if forward kinematics failed.

        Warning:
            This function requires initializing the SDK with the :attr:`KuavoSDK.Options.WithIK`.        
        """
        return self._robot_arm.arm_fk(q)

    def control_robot_end_effector_pose(self, left_pose: KuavoPose, right_pose: KuavoPose, frame: KuavoManipulationMpcFrame)->bool:
        """Control the end effector pose of the robot arm.
        
        Args:
            left_pose (KuavoPose): Pose of the robot left arm, xyz and quat.
            right_pose (KuavoPose): Pose of the robot right arm, xyz and quat.
            frame (KuavoManipulationMpcFrame): Frame of the robot arm.
            
        Returns:
            bool: True if the end effector pose is controlled successfully, False otherwise.
        """
        return self._robot_arm.control_robot_end_effector_pose(left_pose, right_pose, frame)

if __name__ == "__main__":
    robot = KuavoRobot()
    robot.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
    robot.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.DirectToWbc)
    robot.set_manipulation_mpc_frame(KuavoManipulationMpcFrame.WorldFrame)
    robot.control_robot_end_effector_pose(KuavoPose(position=[0.3, 0.4, 0.9], orientation=[0.0, 0.0, 0.0, 1.0]), KuavoPose(position=[0.3, -0.5, 1.0], orientation=[0.0, 0.0, 0.0, 1.0]), KuavoManipulationMpcFrame.WorldFrame)
