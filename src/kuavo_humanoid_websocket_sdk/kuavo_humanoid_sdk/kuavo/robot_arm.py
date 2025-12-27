#!/usr/bin/env python3
# coding: utf-8

import math
from typing import Tuple
from kuavo_humanoid_sdk.interfaces.data_types import KuavoArmCtrlMode, KuavoIKParams, KuavoPose, KuavoManipulationMpcFrame, KuavoManipulationMpcCtrlMode, KuavoManipulationMpcControlFlow
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore
from kuavo_humanoid_sdk.kuavo.robot_info import KuavoRobotInfo

class KuavoRobotArm:
    def __init__(self):
        self._kuavo_core = KuavoRobotCore()
        self._robot_info = KuavoRobotInfo(robot_type="kuavo")
    
    def arm_reset(self)-> bool:
        return self._kuavo_core.robot_arm_reset()
    
    def manipulation_mpc_reset(self)-> bool:
        return self._kuavo_core.robot_manipulation_mpc_reset()
        
    def control_arm_joint_positions(self, joint_position:list)->bool:
        """
            Control the position of the robot arm joint.
            Args:
                joint_position (list): List of joint positions in radians
            Raises:
                ValueError: If the joint position list is not of the correct length.
                RuntimeError: If the robot is not in stance state when trying to control the arm.
            Returns:
                True if the control was successful, False otherwise.
            Note:
                Joint positions that exceed physical limits will be automatically clipped to the limit values.
        """
        if len(joint_position) != self._robot_info.arm_joint_dof:
            raise ValueError("Invalid position length. Expected {}, got {}".format(self._robot_info.arm_joint_dof, len(joint_position)))
        
        # Automatically clip joint positions to physical limits
        arm_min, arm_max = self._robot_info.get_arm_joint_limits()
        joint_position_clipped = list(joint_position)  # Create a copy to avoid modifying the original list
        
        for i, pos in enumerate(joint_position):
            if pos < arm_min[i] or pos > arm_max[i]:
                # Automatically clip to limits
                joint_position_clipped[i] = max(arm_min[i], min(pos, arm_max[i]))

        return self._kuavo_core.control_robot_arm_joint_positions(joint_data=joint_position_clipped)

    def control_arm_joint_trajectory(self, times:list, joint_q:list)->bool:
        """
            Control the target poses of the robot arm.
            Args:
                times (list): List of time intervals in seconds
                joint_q (list): List of joint positions in radians
            Raises:
                ValueError: If the times list is not of the correct length.
                ValueError: If the joint position list is not of the correct length.
                RuntimeError: If the robot is not in stance state when trying to control the arm.
            Returns:
                bool: True if the control was successful, False otherwise.
            Note:
                Joint positions that exceed physical limits will be automatically clipped to the limit values.
        """
        if len(times) != len(joint_q):
            raise ValueError("Invalid input. times and joint_q must have thesame length.")
        
        # Automatically clip joint positions to physical limits
        arm_min, arm_max = self._robot_info.get_arm_joint_limits()
        q_degs = []
        for frame_idx, q in enumerate(joint_q):
            if len(q) != self._robot_info.arm_joint_dof:
                raise ValueError("Invalid position length. Expected {}, got {}".format(self._robot_info.arm_joint_dof, len(q)))
            
            # Create a copy to avoid modifying the original list
            q_clipped = list(q)
            
            # Automatically clip each joint position to physical limits
            for joint_idx, pos in enumerate(q):
                if pos < arm_min[joint_idx] or pos > arm_max[joint_idx]:
                    # Automatically clip to limits
                    q_clipped[joint_idx] = max(arm_min[joint_idx], min(pos, arm_max[joint_idx]))
            
            # Convert joint positions from radians to degrees
            q_degs.append([(p * 180.0 / math.pi) for p in q_clipped])

        return self._kuavo_core.control_robot_arm_joint_trajectory(times=times, joint_q=q_degs)

    def control_robot_end_effector_pose(self, left_pose: KuavoPose, right_pose: KuavoPose, frame: KuavoManipulationMpcFrame)->bool:
        """
            Control the end effector pose of the robot arm.
            Args:
                left_pose (KuavoPose): Pose of the robot left arm, xyz and quat.
                right_pose (KuavoPose): Pose of the robot right arm, xyz and quat.
                frame (KuavoManipulationMpcFrame): Frame of the robot end effector pose.
            Returns:
                bool: True if the control was successful, False otherwise.
        """
        return self._kuavo_core.control_robot_end_effector_pose(left_pose, right_pose, frame)

    def is_arm_collision(self)->bool:
        """Check if the arm is in collision.
        """
        return self._kuavo_core.is_arm_collision()
    
    def is_arm_collision_mode(self)->bool:
        """Check if arm collision mode is enabled.
        
        Returns:
            bool: True if collision mode is enabled, False otherwise.
        """
        return self._kuavo_core.is_arm_collision_mode()
    
    def wait_arm_collision_complete(self):
        """Wait for the arm collision to complete.
        """
        self._kuavo_core.wait_arm_collision_complete()
    
    def release_arm_collision_mode(self):
        """Release the arm collision mode.
        """
        self._kuavo_core.release_arm_collision_mode()

    def set_arm_collision_mode(self, enable: bool):
        """Set the arm collision mode.
        """
        self._kuavo_core.set_arm_collision_mode(enable)

    def set_fixed_arm_mode(self) -> bool:
        """
        Freezes the robot arm.
        Returns:
            bool: True if the arm is frozen successfully, False otherwise.
        """
        return self._kuavo_core.change_robot_arm_ctrl_mode(KuavoArmCtrlMode.ArmFixed)

    def set_auto_swing_arm_mode(self) -> bool:
        """
        Swing the robot arm.
        Returns:
            bool: True if the arm is swinging successfully, False otherwise.
        """
        return self._kuavo_core.change_robot_arm_ctrl_mode(KuavoArmCtrlMode.AutoSwing)
    
    def set_external_control_arm_mode(self) -> bool:
        """
        External control the robot arm.
        Returns:
            bool: True if the arm is external controlled successfully, False otherwise.
        """
        return self._kuavo_core.change_robot_arm_ctrl_mode(KuavoArmCtrlMode.ExternalControl)

    def set_manipulation_mpc_mode(self, ctrl_mode: KuavoManipulationMpcCtrlMode) -> bool:
        """
        Set the manipulation mpc mode.
        Returns:
            bool: True if the manipulation mpc mode is set successfully, False otherwise.
        """
        return self._kuavo_core.change_manipulation_mpc_ctrl_mode(ctrl_mode)
    
    def set_manipulation_mpc_control_flow(self, control_flow: KuavoManipulationMpcControlFlow) -> bool:
        """
        Set the manipulation mpc control flow.
        Returns:
            bool: True if the manipulation mpc control flow is set successfully, False otherwise.
        """
        return self._kuavo_core.change_manipulation_mpc_control_flow(control_flow)
    
    def set_manipulation_mpc_frame(self, frame: KuavoManipulationMpcFrame) -> bool:
        """
        Set the manipulation mpc frame.
        Returns:
            bool: True if the manipulation mpc frame is set successfully, False otherwise.
        """
        return self._kuavo_core.change_manipulation_mpc_frame(frame)
    
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
        return self._kuavo_core.arm_ik(left_pose, right_pose, left_elbow_pos_xyz, right_elbow_pos_xyz, arm_q0, params)

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
        if len(q) != self._robot_info.arm_joint_dof:
            raise ValueError("Invalid position length. Expected {}, got {}".format(self._robot_info.arm_joint_dof, len(q)))
        
        result = self._kuavo_core.arm_fk(q)
        if result is None:
            return None, None
        return result

# if __name__ == "__main__":
#     arm = KuavoRobotArm()
#     arm.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
#     arm.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.DirectToWbc)
#     arm.set_manipulation_mpc_frame(KuavoManipulationMpcFrame.WorldFrame)
#     arm.control_robot_end_effector_pose(KuavoPose(position=[0.3, 0.4, 0.9], orientation=[0.0, 0.0, 0.0, 1.0]), KuavoPose(position=[0.3, -0.5, 1.0], orientation=[0.0, 0.0, 0.0, 1.0]), KuavoManipulationMpcFrame.WorldFrame)
