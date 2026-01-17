#!/usr/bin/env python3
# coding: utf-8
import time
from typing import Tuple
from kuavo_humanoid_sdk.interfaces.end_effector import EndEffector
from kuavo_humanoid_sdk.interfaces.data_types import EndEffectorSide, EndEffectorState
from kuavo_humanoid_sdk.kuavo.core.leju_claw_control import LejuClawControl
from kuavo_humanoid_sdk.kuavo.core.ros.state import KuavoRobotStateCore

class LejuClaw(EndEffector):
    def __init__(self):
        super().__init__(joint_names=['left_claw', 'right_claw'])
        self.leju_claw_control = LejuClawControl()
        self._rb_state = KuavoRobotStateCore()

    def control(self, target_positions: list, target_velocities:list=None, target_torques: list=None)->bool:
        """控制机器人夹爪抓取。

        Args:
            target_positions (list): 夹爪的目标位置。\n
            target_velocities (list, optional): 夹爪的目标速度。如果为None，将使用默认值[90, 90]。\n
            target_torques (list, optional): 夹爪的目标扭矩。如果为None，将使用默认值[1.0, 1.0]。\n

        Note:
            target_positions、target_velocities 和 target_torques 必须是长度为`self.joint_count()`的列表。\n
            调用此函数后，可以调用 :meth:`LejuClaw.wait_for_finish` 等待夹爪到达目标位置。

        Warning:
            如果夹爪仍在执行上一个命令（运动未结束），这个请求可能会被丢弃。

        Returns:
            bool: 如果夹爪成功发送命令返回True，否则返回False。
        """
        if target_positions is None or len(target_positions) != self.joint_count():
            raise ValueError("Target positions must be provided.")
        
        target_positions = [max(0.0, min(100.0, pos)) for pos in target_positions]
        if target_velocities is None:
            target_velocities = [90, 90]
        else:
           if len(target_velocities) != self.joint_count():
               raise ValueError("Target velocities must be a list of length 2.")
           target_velocities = [max(0.0, min(100.0, vel)) for vel in target_velocities]
        
        if target_torques is None:
            target_torques = [1.0, 1.0]
        else:
            if len(target_torques) != self.joint_count():
                raise ValueError("Target torques must be a list of length 2.")
            target_torques = [max(0.0, min(10.0, torque)) for torque in target_torques]
            
        return self.leju_claw_control.control(target_positions, target_velocities, target_torques, EndEffectorSide.BOTH)

    def control_left(self, target_positions:list, target_velocities:list=None, target_torques:list=None)->bool:
        """控制机器人左夹爪抓取。

        Args:
            target_positions (list): 左夹爪的目标位置。\n
            target_velocities (list, optional): 左夹爪的目标速度。如果为None，将使用默认值[90, 90]。\n
            target_torques (list, optional): 左夹爪的目标扭矩。如果为None，将使用默认值[1.0, 1.0]。\n

        Note:
            target_positions、target_velocities 和 target_torques 必须是长度为`self.joint_count()/2`的列表。\n
            调用此函数后，可以调用 :meth:`LejuClaw.wait_for_finish` 等待夹爪到达目标位置。

        Warning:
            如果夹爪仍在执行上一个命令（运动未结束），这个请求可能会被丢弃。

        Returns:
            bool: 如果夹爪成功发送命令返回True，否则返回False。
        """
        if target_positions is None:
            raise ValueError("Target positions must be provided.")
        
        for data in [target_positions, target_velocities, target_torques]:
            if data is not None and len(data) != self.joint_count()/2:
                raise ValueError(f"Target data must be a list of length {self.joint_count()/2}.")
        
        q = max(0.0, min(100.0, target_positions[0]))

        if target_velocities is not None:
            v = max(0.0, min(100.0, target_velocities[0]))
        else:
            v = 90

        if target_torques is not None:
            tau = max(0.0, min(10.0, target_torques[0]))
        else:
            tau = 1.0

        return self.leju_claw_control.control([q], [v], [tau], EndEffectorSide.LEFT)

    def control_right(self, target_positions:list, target_velocities:list=None, target_torques:list=None)->bool:
        """控制机器人右夹爪抓取。

        Args:
            target_positions (list): 右夹爪的目标位置。\n
            target_velocities (list, optional): 右夹爪的目标速度。如果为None，将使用默认值[90, 90]。\n
            target_torques (list, optional): 右夹爪的目标扭矩。如果为None，将使用默认值[1.0, 1.0]。\n

        Returns:
            bool: 如果夹爪成功发送命令返回True，否则返回False。
        
        Note:
            target_positions、target_velocities 和 target_torques 必须是长度为`self.joint_count()/2`的列表。\n
            调用此函数后，可以调用 :meth:`LejuClaw.wait_for_finish` 等待夹爪到达目标位置。

        Warning:
            If the claws are still in motion from a previous command, this request may be dropped.    
        """
        if target_positions is None:
            raise ValueError("Target positions must be provided.")
        
        for data in [target_positions, target_velocities, target_torques]:
            if data is not None and len(data) != self.joint_count()/2:
                raise ValueError(f"Target data must be a list of length {self.joint_count()/2}.")
        
        q = max(0.0, min(100.0, target_positions[0]))

        if target_velocities is not None:
            v = max(0.0, min(100.0, target_velocities[0]))
        else:
            v = 90

        if target_torques is not None:
            tau = max(0.0, min(10.0, target_torques[0]))
        else:
            tau = 1.0

        return self.leju_claw_control.control([q], [v], [tau], EndEffectorSide.RIGHT)
    
    def open(self, side:EndEffectorSide=EndEffectorSide.BOTH)->bool:
        """控制夹爪释放/打开。

        Note:
            控制夹爪打开。 \n
            调用此函数后，可以调用 :meth:`LejuClaw.wait_for_finish` 等待夹爪到达目标位置。

        Args:
            side (EndEffectorSide, optional): 要控制的夹爪侧(左/右或左右)，默认为 :attr:`EndEffectorSide.BOTH` 。

        Returns:
            bool: 如果夹爪成功发送命令返回True，否则返回False。
        """
        return self.leju_claw_control.release(side)

    def close(self, side:EndEffectorSide=EndEffectorSide.BOTH)->bool:
        """控制二指夹爪闭合/抓取。

        Note:
            控制二指夹爪闭合 \n
            调用此函数后，可以调用 :meth:`LejuClaw.wait_for_finish` 等待二指夹爪到达目标位置 \n

        Args:
            side (EndEffectorSide, optional): 要控制的二指夹爪侧(左/右或左右)，默认为 :attr:`EndEffectorState.BOTH` 

        Returns:
            bool: 如果二指夹爪闭合命令发送成功返回 True, 否则返回 False。 \n
            
        Warning:
            二指夹爪的闭合范围为 [0, 100]，其中 0 表示完全闭合，100 表示完全打开。
        """
        return self.leju_claw_control.control(position=[100, 100], velocity=[90, 90], torque=[1.0, 1.0], side=side)

    def get_grasping_state(self)->Tuple[EndEffectorState.GraspingState, EndEffectorState.GraspingState]:
        """获取夹爪的抓取状态。     

        Returns:
            Tuple[EndEffectorState.GraspingState, EndEffectorState.GraspingState]: 夹爪的抓取状态。
        """
        return self._rb_state.eef_state.state
    
    def get_position(self)->Tuple[list, list]:
        """获取夹爪的位置。

        Returns:
            Tuple[list, list]: 夹爪的位置，范围 [0.0, 100.0]。
        """
        claw_state = self._rb_state.eef_state
        return (claw_state[0].position, claw_state[1].position)
    
    def get_velocity(self)->Tuple[list, list]:
        """获取夹爪的速度。

        Returns:
            Tuple[list, list]: 夹爪的速度。
        """
        claw_state = self._rb_state.eef_state
        return (claw_state[0].velocity, claw_state[1].velocity)
    
    def get_effort(self)->Tuple[list, list]:
        """获取夹爪的力。

        Returns:
            Tuple[list, list]: 夹爪的力。
        """
        claw_state = self._rb_state.eef_state
        return (claw_state[0].effort, claw_state[1].effort)

    def get_state(self)->Tuple[EndEffectorState, EndEffectorState]:
        """获取夹爪的状态。

        Returns:
            Tuple[EndEffectorState, EndEffectorState]: 夹爪的状态。
                - position: 夹爪的位置，范围 [0.0, 100.0]。
                - velocity: 夹爪的速度。
                - effort: 夹爪的力。
                - state: 夹爪的抓取状态。
        """
        return self._rb_state.eef_state

    def wait_for_finish(self, side: EndEffectorSide=EndEffectorSide.BOTH, timeout:float=2.5):
        """等待夹爪运动完成。

        Args:
            side (EndEffectorSide, optional): 要等待的夹爪侧(左/右或左右)，默认为 :attr:`EndEffectorSide.BOTH` 。
            timeout (float, optional): 等待超时时间，默认为 2.5 秒。

        Returns:
            bool: 如果运动在超时前完成返回True，否则返回False。
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if side == EndEffectorSide.BOTH:
                if self._rb_state.eef_state[0].state >= EndEffectorState.GraspingState.REACHED and \
                   self._rb_state.eef_state[1].state >= EndEffectorState.GraspingState.REACHED:
                    return True
            elif side == EndEffectorSide.LEFT:
                if self._rb_state.eef_state[0].state >= EndEffectorState.GraspingState.REACHED:
                    return True
            elif side == EndEffectorSide.RIGHT:
                if self._rb_state.eef_state[1].state >= EndEffectorState.GraspingState.REACHED:
                    return True
            time.sleep(0.1)
        return False