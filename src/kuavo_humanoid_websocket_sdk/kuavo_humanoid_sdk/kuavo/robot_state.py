#!/usr/bin/env python3
# coding: utf-8
import time
import copy
from typing import Tuple
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoImuData, KuavoJointData, KuavoOdometry, KuavoArmCtrlMode,EndEffectorState, 
    KuavoManipulationMpcControlFlow, KuavoManipulationMpcCtrlMode, KuavoManipulationMpcFrame)
from kuavo_humanoid_sdk.kuavo.core.ros.state import KuavoRobotStateCoreWebsocket

class KuavoRobotState:
    def __init__(self, robot_type: str = "kuavo"):
        self._rs_core = KuavoRobotStateCoreWebsocket()

    @property
    def imu_data(self) -> KuavoImuData:
        """Get Robot IMU Data.

        Gets the current IMU sensor data from the robot, including gyroscope, accelerometer,
        free acceleration and orientation quaternion measurements.

        Returns:
            :class:`KuavoImuData`: IMU data containing:
                * gyro (:obj:`tuple` of :obj:`float`): Gyroscope measurements (x, y, z) in rad/s
                * acc (:obj:`tuple` of :obj:`float`): Accelerometer measurements (x, y, z) in m/s^2
                * free_acc (:obj:`tuple` of :obj:`float`): Free acceleration (x, y, z) in m/s^2
                * quat (:obj:`tuple` of :obj:`float`): Orientation quaternion (x, y, z, w)
        """
        return self._rs_core.imu_data
    
    @property
    def joint_state(self) -> KuavoJointData:
        """Get Robot Joint Data.

        Get Robot Joint Data, including joint positions, velocities, torques and accelerations.

        The data includes:
            - Joint positions (angles) in radians
            - Joint velocities in radians/second 
            - Joint torques/efforts in Newton-meters, A
            - Joint acceleration

        Returns:
            KuavoJointData: A dictionary containing joint state data with the following keys:
                position (list[float]): Joint positions, length = joint_dof(28)
                velocity (list[float]): Joint velocities, length = joint_dof(28)  
                torque (list[float]): Joint torques, length = joint_dof(28)
                acceleration (list[float]): Joint accelerations, length = joint_dof(28)
        """
        return self._rs_core.joint_data
    
    @property
    def com_height(self)->float:
        """Get the height of the robot's center of mass.

        Returns:
            float: The height of the robot's center of mass in meters.
        """
        return self._rs_core.com_height
    
    @property
    def odometry(self) -> KuavoOdometry:
        """Get Robot Odometry Data.

        Gets the current odometry data from the robot, including position, orientation,
        linear velocity and angular velocity measurements.

        Returns:
            KuavoOdometry: A dictionary containing odometry data with the following keys:
                position (tuple): Position (x, y, z) in meters
                orientation (tuple): Orientation as quaternion (x, y, z, w)
                linear (tuple): Linear velocity (x, y, z) in m/s
                angular (tuple): Angular velocity (x, y, z) in rad/s
        """
        return self._rs_core.odom_data   

    
    def robot_position(self) -> Tuple[float, float, float]:
        """Returns the robot's position in world coordinates.

        Returns:
            Tuple[float, float, float]: Position (x, y, z) in meters.
        """
        return tuple(self._rs_core.odom_data.position)

    def robot_orientation(self) -> Tuple[float, float, float, float]:
        """Returns the robot's orientation in world coordinates.

        Returns:
            Tuple[float, float, float, float]: Orientation as quaternion (x, y, z, w).
        """
        return tuple(self._rs_core.odom_data.orientation)

    def linear_velocity(self) -> Tuple[float, float, float]:
        """Returns the robot's linear velocity in world coordinates.

        Returns:
            Tuple[float, float, float]: Linear velocity (x, y, z) in m/s.
        """
        return tuple(self._rs_core.odom_data.linear)
    

    def angular_velocity(self) -> Tuple[float, float, float]:
        """Returns the robot's angular velocity in world coordinates.

        Returns:
            Tuple[float, float, float]: Angular velocity (x, y, z).
        """
        return tuple(self._rs_core.odom_data.angular)

    def arm_joint_state(self) -> KuavoJointData:
        """Get the current state of the robot arm joints.

        Get the current state of the robot arm joints, including:
            - Joint positions (angles) in radians
            - Joint velocities in radians/second 
            - Joint torques/efforts in Newton-meters, A
            - Joint acceleration

        Returns:
            KuavoJointData: Arm joint data containing:
                position: list[float] * arm_dof(14)
                velocity: list[float] * arm_dof(14)
                torque: list[float]   * arm_dof(14)
                acceleration: list[float] * arm_dof(14)
        """
        # Get arm joint states from index 12 to 25 (14 arm joints)
        arm_joint_indices = range(12, 12+14)
        return KuavoJointData(
            position=[self._rs_core.joint_data.position[i] for i in arm_joint_indices],
            velocity=[self._rs_core.joint_data.velocity[i] for i in arm_joint_indices],
            torque=[self._rs_core.joint_data.torque[i] for i in arm_joint_indices],
            acceleration=[self._rs_core.joint_data.acceleration[i] for i in arm_joint_indices]
        )

    def arm_control_mode(self) -> KuavoArmCtrlMode:
        """Get the current control mode of the robot arm.

        Returns:
            KuavoArmCtrlMode: Current arm control mode:
                ArmFixed: 0 - The robot arm is in a fixed position.
                AutoSwing: 1 - The robot arm is in automatic swing mode.
                ExternalControl: 2 - The robot arm is controlled externally.
                or None.
        """
        return KuavoArmCtrlMode(self._rs_core.arm_control_mode)
    
    def manipulation_mpc_ctrl_mode(self) -> KuavoManipulationMpcCtrlMode:
        """Get the current control mode of the robot manipulation MPC.

        Returns:
            KuavoManipulationMpcCtrlMode: Current manipulation MPC control mode.

        """
        return self._rs_core.manipulation_mpc_ctrl_mode
    
    def manipulation_mpc_control_flow(self) -> KuavoManipulationMpcControlFlow:
        """Get the current control flow of the robot manipulation.

        Returns:
            KuavoManipulationMpcControlFlow: Current manipulation control flow.
        """
        return self._rs_core.manipulation_mpc_control_flow
    
    def manipulation_mpc_frame(self) -> KuavoManipulationMpcFrame:
        """Get the current frame of the robot manipulation MPC.

        Returns:
            KuavoManipulationMpcFrame: Current manipulation MPC frame.
        """
        return self._rs_core.manipulation_mpc_frame
    
    def head_joint_state(self) -> KuavoJointData:
        """Get the current state of the robot head joints.

        Gets the current state data for the robot's head joints, including position,
        velocity, torque and acceleration values.

        Returns:
            KuavoJointData: A data structure containing the head joint states:
                position (list[float]): Joint positions in radians, length=head_dof(2)
                velocity (list[float]): Joint velocities in rad/s, length=head_dof(2) 
                torque (list[float]): Joint torques in Nm, length=head_dof(2)
                acceleration (list[float]): Joint accelerations in rad/s^2, length=head_dof(2)
                
            The joint order is [yaw, pitch].
        """
        # Get head joint states from last 2 indices
        head_joint_indices = range(len(self._rs_core.joint_data.position)-2, len(self._rs_core.joint_data.position))
        return KuavoJointData(
            position=[self._rs_core.joint_data.position[i] for i in head_joint_indices],
            velocity=[self._rs_core.joint_data.velocity[i] for i in head_joint_indices], 
            torque=[self._rs_core.joint_data.torque[i] for i in head_joint_indices],
            acceleration=[self._rs_core.joint_data.acceleration[i] for i in head_joint_indices]
        )
    
    def eef_state(self)->Tuple[EndEffectorState, EndEffectorState]:
        """Get the current state of the robot's end effectors.

        Returns:
            Tuple[EndEffectorState, EndEffectorState]: A tuple containing the state of the left and right end effectors.
                Each EndEffectorState contains:
                    - position: (float, float, float) - XYZ position in meters
                    - orientation: (float, float, float, float) - Quaternion orientation
                    - state: EndEffectorState.GraspingState - Current grasping state (UNKNOWN, OPEN, CLOSED)
        """
        return copy.deepcopy(self._rs_core.eef_state)

    def gait_name(self)->str:
        """Get the current gait name of the robot.

        Returns:
            str: The name of the current gait, e.g. 'trot', 'walk', 'stance', 'custom_gait'.
        """
        return self._rs_core.gait_name()
    

    def is_stance(self) -> bool:
        """Check if the robot is currently in stance mode.

        Returns:
            bool: True if robot is in stance mode, False otherwise.
        """
        return self._rs_core.is_gait('stance')

    def is_walk(self) -> bool:
        """Check if the robot is currently in walk mode.

        Returns:
            bool: True if robot is in walk mode, False otherwise.
        """
        return self._rs_core.is_gait('walk')

    def is_step_control(self) -> bool:
        """Check if the robot is currently in step control mode.

        Returns:
            bool: True if robot is in step control mode, False otherwise.
        """
        return self._rs_core.is_gait('custom_gait')

    def wait_for_stance(self, timeout:float=5.0)->bool:
        """Wait for the robot to enter stance state.

        Args:
            timeout (float): The maximum time to wait for the robot to enter stance state in seconds.

        Returns:
            bool: True if the robot enters stance state within the specified timeout, False otherwise.
        """
        wait_time = 0
        while not self._rs_core.is_gait('stance') and wait_time < timeout:
            time.sleep(0.1)
            wait_time += 0.1
        return self._rs_core.is_gait('stance')
    
    def wait_for_trot(self, timeout:float=5.0)->bool:
        """Wait for the robot to enter trot state.

        Args:
            timeout (float): The maximum time to wait for the robot to enter trot state in seconds.

        Returns:
            bool: True if the robot enters trot state within the specified timeout, False otherwise.
        """
        return self.wait_for_walk(timeout=timeout)

    def wait_for_walk(self, timeout:float=5.0)->bool:
        """Wait for the robot to enter walk state.

        Args:
            timeout (float): The maximum time to wait for the robot to enter walk state in seconds.

        Returns:
            bool: True if the robot enters walk state within the specified timeout, False otherwise.
        """
        wait_time = 0
        while not self._rs_core.is_gait('walk') and wait_time < timeout:
            time.sleep(0.1)
            wait_time += 0.1
        return self._rs_core.is_gait('walk')

    def wait_for_step_control(self, timeout:float=5.0)->bool:
        """Wait for the robot to enter step control state.

        Args:
            timeout (float): The maximum time to wait for the robot to enter step control state in seconds.

        Returns:
            bool: True if the robot enters step control state within the specified timeout, False otherwise.
        """
        wait_time = 0
        while not self._rs_core.is_gait('custom_gait') and wait_time < timeout:
            time.sleep(0.1)
            wait_time += 0.1
        return self._rs_core.is_gait('custom_gait')
    
# if __name__ == "__main__":
#     state = KuavoRobotState()
#     print(state.manipulation_mpc_frame())
#     print(state.manipulation_mpc_control_flow())
#     print(state.manipulation_mpc_ctrl_mode())
#     print(state.arm_control_mode())
