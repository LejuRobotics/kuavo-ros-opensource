#!/usr/bin/env python3
# coding: utf-8
import time
import copy
from typing import Tuple
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoImuData, KuavoJointData, KuavoOdometry, KuavoArmCtrlMode,EndEffectorState, 
    KuavoManipulationMpcControlFlow, KuavoManipulationMpcCtrlMode, KuavoManipulationMpcFrame)
from kuavo_humanoid_sdk.kuavo.core.ros.state import KuavoRobotStateCore

class KuavoRobotState:
    def __init__(self, robot_type: str = "kuavo"):
        self._rs_core = KuavoRobotStateCore()

    @property
    def imu_data(self) -> KuavoImuData:
        """获取 Kuavo 机器人IMU数据。

        获取机器人当前的 IMU 传感器数据，包括陀螺仪、加速度计、自由加速度和方向四元数测量值。

        Returns:
            :class:`KuavoImuData`: IMU数据包含:
                * gyro (:obj:`tuple` of :obj:`float`): 陀螺仪测量值 (x, y, z)，单位rad/s
                * acc (:obj:`tuple` of :obj:`float`): 加速度计测量值 (x, y, z)，单位m/s^2
                * free_acc (:obj:`tuple` of :obj:`float`): 自由加速度 (x, y, z)，单位m/s^2
                * quat (:obj:`tuple` of :obj:`float`): 方向四元数 (x, y, z, w)
        """
        return self._rs_core.imu_data
    
    @property
    def joint_state(self) -> KuavoJointData:
        """获取 Kuavo 机器人关节数据。

        获取机器人关节数据，包括关节位置、速度、扭矩和加速度。

        数据包括:
            - 关节位置(角度)，单位为弧度
            - 关节速度，单位为弧度/秒
            - 关节扭矩/力矩，单位为牛顿米、安培
            - 关节加速度

        Returns:
            KuavoJointData: 包含以下关节状态数据的字典:
                * position (list[float]): 关节位置，长度 = joint_dof(28)
                * velocity (list[float]): 关节速度，长度 = joint_dof(28)
                * torque (list[float]): 关节扭矩，长度 = joint_dof(28)
                * acceleration (list[float]): 关节加速度，长度 = joint_dof(28)
        """
        return self._rs_core.joint_data
    
    @property
    def com_height(self)->float:
        """获取机器人质心高度。

        Returns:
            float: 机器人质心高度，单位为米。
        """
        return self._rs_core.com_height
    
    @property
    def odometry(self) -> KuavoOdometry:
        """获取 Kuavo 机器人里程计数据。

        获取机器人当前的里程计数据，包括位置、方向、线速度和角速度测量值。

        Returns:
            KuavoOdometry: 包含以下里程计数据的字典:
                * position (tuple): 位置 (x, y, z)，单位为米
                * orientation (tuple): 方向四元数 (x, y, z, w)
                * linear (tuple): 线速度 (x, y, z)，单位为m/s
                * angular (tuple): 角速度 (x, y, z)，单位为rad/s
        """
        return self._rs_core.odom_data   

    
    def robot_position(self) -> Tuple[float, float, float]:
        """返回 Kuavo 机器人在世界坐标系中的位置。

        Returns:
            Tuple[float, float, float]: 位置 (x, y, z)，单位为米。
        """
        return tuple(self._rs_core.odom_data.position)

    def robot_orientation(self) -> Tuple[float, float, float, float]:
        """返回 Kuavo 机器人在世界坐标系中的方向。

        Returns:
            Tuple[float, float, float, float]: 方向四元数 (x, y, z, w)。
        """
        return tuple(self._rs_core.odom_data.orientation)

    def linear_velocity(self) -> Tuple[float, float, float]:
        """返回 Kuavo 机器人在世界坐标系中的线速度。

        Returns:
            Tuple[float, float, float]: 线速度 (x, y, z)，单位为m/s。
        """
        return tuple(self._rs_core.odom_data.linear)
    

    def angular_velocity(self) -> Tuple[float, float, float]:
        """返回 Kuavo 机器人在世界坐标系中的角速度。

        Returns:
            Tuple[float, float, float]: 角速度 (x, y, z)。
        """
        return tuple(self._rs_core.odom_data.angular)

    def arm_joint_state(self) -> KuavoJointData:
        """获取 Kuavo 机器人手臂关节的当前状态。

        获取 Kuavo 机器人手臂关节的当前状态，包括:
            - 关节位置(角度)，单位为弧度
            - 关节速度，单位为弧度/秒
            - 关节扭矩/力矩，单位为牛顿米、安培
            - 关节加速度

        Returns:
            KuavoJointData: 手臂关节数据包含:
                * position: list[float] * arm_dof(14)
                * velocity: list[float] * arm_dof(14)
                * torque: list[float] * arm_dof(14)
                * acceleration: list[float] * arm_dof(14)
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
        """获取 Kuavo 机器人手臂的当前控制模式。

        Returns:
            KuavoArmCtrlMode: 当前手臂控制模式:
                * ArmFixed: 0 - 机器人手臂处于固定位置。
                * AutoSwing: 1 - 机器人手臂处于自动摆动模式。
                * ExternalControl: 2 - 机器人手臂由外部控制。
                * None - 机器人手臂处于未知状态。
        """
        return KuavoArmCtrlMode(self._rs_core.arm_control_mode)
    
    def manipulation_mpc_ctrl_mode(self) -> KuavoManipulationMpcCtrlMode:
        """获取 Kuavo 机器人 Manipulation MPC 的当前控制模式。    

        Returns:
            KuavoManipulationMpcCtrlMode: 当前 Manipulation MPC 控制模式。
        """
        return self._rs_core.manipulation_mpc_ctrl_mode
    
    def manipulation_mpc_control_flow(self) -> KuavoManipulationMpcControlFlow:
        """获取 Kuavo 机器人 Manipulation MPC 的当前控制流。

        Returns:
            KuavoManipulationMpcControlFlow: 当前 Manipulation MPC 控制流。
        """
        return self._rs_core.manipulation_mpc_control_flow
    
    def manipulation_mpc_frame(self) -> KuavoManipulationMpcFrame:
        """获取机器人操作MPC的当前帧。

        Returns:
            KuavoManipulationMpcFrame: 末端执行器 Manipulation MPC 坐标系
        """
        return self._rs_core.manipulation_mpc_frame
    
    def pitch_limit_enabled(self) -> bool:
        """获取机器人 basePitch 限制状态, 如果开启则返回True，否则返回False。

        Returns:
            bool: 如果机器人 basePitch 限制开启返回True，否则返回False。
        """
        return self._rs_core.pitch_limit_enabled

    def head_joint_state(self) -> KuavoJointData:
        """获取机器人头部关节的当前状态。

        获取机器人头部关节的当前状态数据，包括位置、速度、扭矩和加速度值。

        Returns:
            KuavoJointData: 包含头部关节状态的数据结构:
                * position (list[float]): 关节位置，单位为弧度，长度=head_dof(2)
                * velocity (list[float]): 关节速度，单位为rad/s，长度=head_dof(2)
                * torque (list[float]): 关节扭矩，单位为Nm，长度=head_dof(2)
                * acceleration (list[float]): 关节加速度，单位为rad/s^2，长度=head_dof(2)
                
            关节顺序为 [偏航角, 俯仰角]。
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
        """获取机器人末端执行器的当前状态。

        Returns:
            Tuple[EndEffectorState, EndEffectorState]: 包含左右末端执行器状态的元组。
                每个EndEffectorState包含:
                    - position: (float, float, float) ，XYZ位置，单位为米
                    - orientation: (float, float, float, float) ，四元数方向
                    - state: EndEffectorState.GraspingState ，当前抓取状态 (UNKNOWN, OPEN, CLOSED)
        """
        return copy.deepcopy(self._rs_core.eef_state)

    def gait_name(self)->str:
        """获取机器人的当前步态名称。

        Returns:
            str: 当前步态的名称，例如 'trot'、'walk'、'stance'、'custom_gait'。
        """
        return self._rs_core.gait_name()
    

    def is_stance(self) -> bool:
        """检查机器人当前是否处于站立模式。

        Returns:
            bool: 如果机器人处于站立模式返回True，否则返回False。
        """
        return self._rs_core.is_gait('stance')

    def is_walk(self) -> bool:
        """检查机器人当前是否处于行走模式。

        Returns:
            bool: 如果机器人处于行走模式返回True，否则返回False。
        """
        return self._rs_core.is_gait('walk')

    def is_step_control(self) -> bool:
        """检查机器人当前是否处于单步控制模式。

        Returns:
            bool: 如果机器人处于单步控制模式返回True，否则返回False。
        """
        return self._rs_core.is_gait('custom_gait')

    def wait_for_stance(self, timeout:float=5.0)->bool:
        """等待机器人进入站立模式。

        Args:
            timeout (float): 等待机器人进入站立状态的最长时间，单位为秒。

        Returns:
            bool: 如果机器人在指定超时时间内进入站立状态返回True，否则返回False。
        """
        wait_time = 0
        while not self._rs_core.is_gait('stance') and wait_time < timeout:
            time.sleep(0.1)
            wait_time += 0.1
        return self._rs_core.is_gait('stance')
    
    def wait_for_trot(self, timeout:float=5.0)->bool:
        """等待机器人进入踏步状态。

        Args:
            timeout (float): 等待机器人进入踏步状态的最长时间，单位为秒。

        Returns:
            bool: 如果机器人在指定超时时间内进入踏步状态返回True，否则返回False。
        """
        return self.wait_for_walk(timeout=timeout)

    def wait_for_walk(self, timeout:float=5.0)->bool:
        """等待机器人进入行走模式。

        Args:
            timeout (float): 等待机器人进入行走状态的最长时间，单位为秒。

        Returns:
            bool: 如果机器人在指定超时时间内进入行走状态返回True，否则返回False。
        """
        wait_time = 0
        while not self._rs_core.is_gait('walk') and wait_time < timeout:
            time.sleep(0.1)
            wait_time += 0.1
        return self._rs_core.is_gait('walk')

    def wait_for_step_control(self, timeout:float=5.0)->bool:
        """等待机器人进入单步控制模式。

        Args:
            timeout (float): 等待机器人进入单步控制模式的最长时间，单位为秒。

        Returns:
            bool: 如果机器人在指定超时时间内进入单步控制模式返回True，否则返回False。
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
