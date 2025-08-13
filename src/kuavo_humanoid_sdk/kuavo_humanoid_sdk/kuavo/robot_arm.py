#!/usr/bin/env python3
# coding: utf-8

import math
from typing import Tuple
from kuavo_humanoid_sdk.interfaces.data_types import KuavoArmCtrlMode, KuavoIKParams, KuavoPose, KuavoManipulationMpcFrame, KuavoManipulationMpcCtrlMode, KuavoManipulationMpcControlFlow
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore
from kuavo_humanoid_sdk.kuavo.robot_info import KuavoRobotInfo

class KuavoRobotArm:
    """Kuavo机器人手臂控制类。
    
    提供了控制机器人手臂的各种接口,包括关节位置控制、轨迹控制、末端执行器姿态控制等。
    """
    def __init__(self):
        self._kuavo_core = KuavoRobotCore()
        self._robot_info = KuavoRobotInfo(robot_type="kuavo")
    
    def arm_reset(self)-> bool:
        """重置机器人手臂。
        
        Returns:
            bool: 重置成功返回True,否则返回False。
        """
        return self._kuavo_core.robot_arm_reset()
    
    def manipulation_mpc_reset(self)-> bool:
        """重置机器人 Manipulation MPC 控制器。
        
        Returns:
            bool: 重置成功返回True,否则返回False。
        """
        return self._kuavo_core.robot_manipulation_mpc_reset()
        
    def control_arm_joint_positions(self, joint_position:list)->bool:
        """控制机器人手臂关节位置。

        Args:
            joint_position (list): 关节位置列表,单位为弧度

        Raises:
            ValueError: 如果关节位置列表长度不正确
            ValueError: 如果关节位置超出[-π, π]范围
            RuntimeError: 如果在控制手臂时机器人不在站立状态

        Returns:
            bool: 控制成功返回True,否则返回False
        """
        if len(joint_position) != self._robot_info.arm_joint_dof:
            raise ValueError("Invalid position length. Expected {}, got {}".format(self._robot_info.arm_joint_dof, len(joint_position)))
        
        # Check if joint positions are within ±180 degrees (±π radians)
        for pos in joint_position:
            if abs(pos) > math.pi:
                raise ValueError(f"Joint position {pos} rad exceeds ±π rad (±180 deg) limit")

        return self._kuavo_core.control_robot_arm_joint_positions(joint_data=joint_position)

    def control_arm_joint_trajectory(self, times:list, joint_q:list)->bool:
        """控制机器人手臂关节轨迹。

        Args:
            times (list): 时间间隔列表,单位为秒
            joint_q (list): 关节位置列表,单位为弧度

        Raises:
            ValueError: 如果times列表长度不正确
            ValueError: 如果关节位置列表长度不正确
            ValueError: 如果关节位置超出[-π, π]范围
            RuntimeError: 如果在控制手臂时机器人不在站立状态

        Returns:
            bool: 控制成功返回True,否则返回False
        """
        if len(times) != len(joint_q):
            raise ValueError("Invalid input. times and joint_q must have thesame length.")
        
        # Check if joint positions are within ±180 degrees (±π radians)
        q_degs = []
        for q in joint_q:
            if any(abs(pos) > math.pi for pos in q):
                raise ValueError("Joint positions must be within ±π rad (±180 deg)")
            if len(q) != self._robot_info.arm_joint_dof:
                raise ValueError("Invalid position length. Expected {}, got {}".format(self._robot_info.arm_joint_dof, len(q)))
            # Convert joint positions from radians to degrees
            q_degs.append([(p * 180.0 / math.pi) for p in q])

        return self._kuavo_core.control_robot_arm_joint_trajectory(times=times, joint_q=q_degs)

    def control_robot_end_effector_pose(self, left_pose: KuavoPose, right_pose: KuavoPose, frame: KuavoManipulationMpcFrame)->bool:
        """控制机器人末端执行器姿态。

        Args:
            left_pose (KuavoPose): 左手臂姿态,包含xyz位置和四元数方向
            right_pose (KuavoPose): 右手臂姿态,包含xyz位置和四元数方向
            frame (KuavoManipulationMpcFrame): 末端执行器姿态的坐标系

        Returns:
            bool: 控制成功返回True,否则返回False
        """
        return self._kuavo_core.control_robot_end_effector_pose(left_pose, right_pose, frame)

    def set_fixed_arm_mode(self) -> bool:
        """固定/冻结机器人手臂。

        Returns:
            bool: 固定/冻结成功返回True,否则返回False
        """
        return self._kuavo_core.change_robot_arm_ctrl_mode(KuavoArmCtrlMode.ArmFixed)

    def set_auto_swing_arm_mode(self) -> bool:
        """设置手臂自动摆动模式。

        Returns:
            bool: 设置成功返回True,否则返回False
        """
        return self._kuavo_core.change_robot_arm_ctrl_mode(KuavoArmCtrlMode.AutoSwing)
    
    def set_external_control_arm_mode(self) -> bool:
        """设置手臂外部控制模式。

        Returns:
            bool: 设置成功返回True,否则返回False
        """
        return self._kuavo_core.change_robot_arm_ctrl_mode(KuavoArmCtrlMode.ExternalControl)

    def set_manipulation_mpc_mode(self, ctrl_mode: KuavoManipulationMpcCtrlMode) -> bool:
        """设置 Manipulation MPC 控制模式。

        Returns:
            bool: 设置成功返回True,否则返回False
        """
        return self._kuavo_core.change_manipulation_mpc_ctrl_mode(ctrl_mode)
    
    def set_manipulation_mpc_control_flow(self, control_flow: KuavoManipulationMpcControlFlow) -> bool:
        """设置 Manipulation MPC 控制流。

        Returns:
            bool: 设置成功返回True,否则返回False
        """
        return self._kuavo_core.change_manipulation_mpc_control_flow(control_flow)
    
    def set_manipulation_mpc_frame(self, frame: KuavoManipulationMpcFrame) -> bool:
        """设置 Manipulation MPC 坐标系。

        Returns:
            bool: 设置成功返回True,否则返回False
        """
        return self._kuavo_core.change_manipulation_mpc_frame(frame)
    
    """ 手臂正向运动学和逆向运动学 """
    def arm_ik(self, 
               left_pose: KuavoPose, 
               right_pose: KuavoPose,
               left_elbow_pos_xyz: list = [0.0, 0.0, 0.0],
               right_elbow_pos_xyz: list = [0.0, 0.0, 0.0],
               arm_q0: list = None,
               params: KuavoIKParams=None) -> list:
        """机器人手臂逆向运动学求解
        
        Args:
            left_pose (KuavoPose): 左手臂目标姿态,包含xyz位置和四元数方向
            right_pose (KuavoPose): 右手臂目标姿态,包含xyz位置和四元数方向
            left_elbow_pos_xyz (list): 左肘部位置。如果为[0.0, 0.0, 0.0],则忽略
            right_elbow_pos_xyz (list): 右肘部位置。如果为[0.0, 0.0, 0.0],则忽略
            arm_q0 (list, optional): 初始关节位置,单位为弧度。如果为None,则忽略
            params (KuavoIKParams, optional): 逆向运动学参数。如果为None,则忽略，包含:
                - major_optimality_tol: 主要最优性容差 \n
                - major_feasibility_tol: 主要可行性容差 \n
                - minor_feasibility_tol: 次要可行性容差 \n
                - major_iterations_limit: 主要迭代次数限制 \n
                - oritation_constraint_tol: 方向约束容差 \n
                - pos_constraint_tol: 位置约束容差,当pos_cost_weight==0.0时生效 \n
                - pos_cost_weight: 位置代价权重。设为0.0可获得高精度 \n
                
        Returns:
            list: 关节位置列表,单位为弧度。如果计算失败返回None

        Warning:
            此函数需要在初始化SDK时设置 :attr:`KuavoSDK.Options.WithIK` 选项。
        """
        return self._kuavo_core.arm_ik(left_pose, right_pose, left_elbow_pos_xyz, right_elbow_pos_xyz, arm_q0, params)

    def arm_fk(self, q: list) -> Tuple[KuavoPose, KuavoPose]:
        """机器人手臂正向运动学求解
        
        Args:
            q (list): 关节位置列表,单位为弧度
            
        Returns:
            Tuple[KuavoPose, KuavoPose]: 左右手臂姿态的元组,
                如果计算失败返回(None, None)
        
        Warning:
            此函数需要在初始化SDK时设置 :attr:`KuavoSDK.Options.WithIK` 选项。
        """
        if len(q) != self._robot_info.arm_joint_dof:
            raise ValueError("Invalid position length. Expected {}, got {}".format(self._robot_info.arm_joint_dof, len(q)))
        
        result = self._kuavo_core.arm_fk(q)
        if result is None:
            return None, None
        return result

    def control_hand_wrench(self, left_wrench: list, right_wrench: list) -> bool:
        """控制机器人末端力/力矩
        
        Args:
            left_wrench (list): 左手臂6维力控指令 [Fx, Fy, Fz, Tx, Ty, Tz]
            right_wrench (list): 右手臂6维力控指令 [Fx, Fy, Fz, Tx, Ty, Tz]
                单位:
                Fx,Fy,Fz: 牛顿(N)
                Tx,Ty,Tz: 牛·米(N·m)
        
        Returns:
            bool: 控制成功返回True, 否则返回False
        """
        return self._kuavo_core.control_hand_wrench(left_wrench, right_wrench)

    def is_arm_collision(self)->bool:
        """判断当前是否发生碰撞
        
        Returns:
            bool: 发生碰撞返回True,否则返回False
        """
        return self._kuavo_core.is_arm_collision()
    
    def release_arm_collision_mode(self):
        """释放碰撞模式
        """
        self._kuavo_core.release_arm_collision_mode()

    def wait_arm_collision_complete(self):
        """等待碰撞完成
        """
        self._kuavo_core.wait_arm_collision_complete()

    def set_arm_collision_mode(self, enable: bool):
        """设置碰撞模式
        """
        self._kuavo_core.set_arm_collision_mode(enable)

# if __name__ == "__main__":
#     arm = KuavoRobotArm()
#     arm.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
#     arm.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.DirectToWbc)
#     arm.set_manipulation_mpc_frame(KuavoManipulationMpcFrame.WorldFrame)
#     arm.control_robot_end_effector_pose(KuavoPose(position=[0.3, 0.4, 0.9], orientation=[0.0, 0.0, 0.0, 1.0]), KuavoPose(position=[0.3, -0.5, 1.0], orientation=[0.0, 0.0, 0.0, 1.0]), KuavoManipulationMpcFrame.WorldFrame)
