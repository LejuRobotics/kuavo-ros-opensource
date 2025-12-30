#!/usr/bin/env python3
# coding: utf-8
from typing import Tuple
from kuavo_humanoid_sdk.interfaces.end_effector import EndEffector
from kuavo_humanoid_sdk.interfaces.data_types import EndEffectorSide, EndEffectorState, KuavoDexHandTouchState
from kuavo_humanoid_sdk.kuavo.core.dex_hand_control import DexHandControl
from kuavo_humanoid_sdk.kuavo.core.ros.state import KuavoRobotStateCore

class DexterousHand(EndEffector):
    """普通灵巧手控制类"""
    def __init__(self):
        joint_names = ['l_thumb', 'l_thumb_aux', 'l_index', 'l_middle', 'l_ring', 'l_pinky',
                       'r_thumb', 'r_thumb_aux', 'r_index', 'r_middle', 'r_ring', 'r_pinky',]
        super().__init__(joint_names=joint_names)
        self.dex_hand_control = DexHandControl()
        self._rb_state = KuavoRobotStateCore()

    def control(self, target_positions:list, target_velocities:list=None, target_torques:list=None)->bool:
        """控制灵巧手的位置。

        Args:
            target_positions (list): 所有手指关节的目标位置列表，长度必须为12（每只手6个手指关节），范围 => [0.0 ~ 100.0] 
            target_velocities (list, optional): 不支持。默认为None。
            target_torques (list, optional): 不支持。默认为None。

        Returns:
            bool: 如果控制成功返回 True，否则返回 False。

        Note:
            target_velocities 和 target_torques 参数暂不支持。
        """
        if len(target_positions) != self.joint_count():
            raise ValueError(f"Target positions must have the same length as joint names {len(target_positions)} != {self.joint_count()}")
        
        q = [max(0, min(100, pos if isinstance(pos, int) else int(pos))) for pos in target_positions]

        # control the hand
        return self.dex_hand_control.control(target_positions=q, side=EndEffectorSide.BOTH)

    def control_right(self, target_positions:list, target_velocities:list=None, target_torques:list=None)->bool:
        """控制右手灵巧手。

        Args:
            target_positions (list): 右手关节的目标位置 [0 ~ 100]，长度必须为6
            target_velocities (list, optional): 不支持。默认为None。
            target_torques (list, optional): 不支持。默认为None。

        Returns:
            bool: 如果控制成功返回True，否则返回False。

        Raises:
            ValueError: 如果目标位置长度与关节数不匹配或值超出[0,100]范围

        Note:
            target_velocities 和 target_torques 参数暂不支持。
        """
        if len(target_positions) != (self.joint_count()/2):
                raise ValueError(f"Target positions must have the same length as joint names {len(target_positions)} != {self.joint_count()/2}.")
        
        q = [max(0, min(100, pos if isinstance(pos, int) else int(pos))) for pos in target_positions]

        return self.dex_hand_control.control(target_positions=q, side=EndEffectorSide.RIGHT)

    def control_left(self, target_positions:list, target_velocities:list=None, target_torques:list=None)->bool:
        """控制左手灵巧手。

        Args:
            target_positions (list): 左手关节的目标位置 [0 ~ 100]，长度必须为6
            target_velocities (list, optional): 不支持。默认为None。
            target_torques (list, optional): 不支持。默认为None。

        Returns:
            bool: 如果控制成功返回True，否则返回False。

        Raises:
            ValueError: 如果目标位置长度与关节数不匹配或值超出[0,100]范围

        Note:
            target_velocities 和 target_torques 参数不支持。
        """
        if len(target_positions) != (self.joint_count()/2):
            raise ValueError(f"Target positions must have the same length as joint names {len(target_positions)} != {self.joint_count()/2}.")
        
        q = [max(0, min(100, pos if isinstance(pos, int) else int(pos))) for pos in target_positions]

        return self.dex_hand_control.control(target_positions=q, side=EndEffectorSide.LEFT)

    def open(self, side: EndEffectorSide=EndEffectorSide.BOTH)->bool:
        """通过将所有关节位置设置为 0 来张开灵巧手。

        Args:
            side (EndEffectorSide, optional): 要打开的手。默认为 :attr:`EndEffectorSide.BOTH`。 \n
                可以是 :attr:`EndEffectorSide.LEFT`、:attr:`EndEffectorSide.RIGHT` 或 :attr:`EndEffectorSide.BOTH`。

        Returns:
            bool: 如果打开命令发送成功返回True，否则返回False。
        """
        zero_pos = [0]*self.joint_count()
        if side == EndEffectorSide.LEFT:
            return self.dex_hand_control.control(target_positions=zero_pos[:len(zero_pos)//2], side=EndEffectorSide.LEFT)
        elif side == EndEffectorSide.RIGHT:
            return self.dex_hand_control.control(target_positions=zero_pos[len(zero_pos)//2:], side=EndEffectorSide.RIGHT)
        else:
            return self.dex_hand_control.control(target_positions=zero_pos, side=EndEffectorSide.BOTH)     

    def make_gesture(self, l_gesture_name: str, r_gesture_name: str)->bool:
        """为双手做预定义的手势。

        Args:
            l_gesture_name (str): 左手手势的名称。None表示跳过左手。
            r_gesture_name (str): 右手手势的名称。None表示跳过右手。

        Returns:
            bool: 如果手势命令发送成功返回True，否则返回False。

        Note:
            手势示例：'fist'、'ok'、'thumbs_up'、'666'等...
        """
        gesture = []
        if l_gesture_name is not None:
            gesture.append({'gesture_name':l_gesture_name, 'hand_side':EndEffectorSide.LEFT})
            self.dex_hand_control.make_gestures(gesture)
        if r_gesture_name is not None:
            gesture.append({'gesture_name':r_gesture_name, 'hand_side':EndEffectorSide.RIGHT})    
            self.dex_hand_control.make_gestures(gesture)
        return True
    def get_gesture_names(self)->list:
        """获取所有手势的名称。

        Returns:
            list: 手势名称列表。
                例如：['fist', 'ok', 'thumbs_up', '666', 'number_1', 'number_2', 'number_3', ... ], 如果没有手势则返回 None。
        """
        return self.dex_hand_control.get_gesture_names()
    
    def get_state(self)->Tuple[EndEffectorState, EndEffectorState]:
        """获取灵巧手的状态。

        Returns:
            Tuple[EndEffectorState, EndEffectorState]: 灵巧手的状态。
        """
        return self._rb_state.eef_state

    def get_position(self)->Tuple[list, list]:
        """获取灵巧手的位置。

        Returns:
            Tuple[list, list]: 灵巧手的位置。
        """
        state = self._rb_state.eef_state
        return (state[0].position, state[1].position)
    
    def get_velocity(self)->Tuple[list, list]:
        """获取灵巧手的速度。

        Returns:
            Tuple[list, list]: 灵巧手的速度。
        """
        state = self._rb_state.eef_state
        return (state[0].velocity, state[1].velocity)

    def get_effort(self)->Tuple[list, list]:
        """获取灵巧手的力。

        Returns:
            Tuple[list, list]: 灵巧手的力。

        Note:
            每个手指的范围为0 ~ 100。表示最大电机电流的分数，绝对数值。
            最大电机电流为600mA，换句话说，100。
        """
        state = self._rb_state.eef_state
        return (state[0].effort, state[1].effort)

    def get_grasping_state(self)->Tuple[EndEffectorState.GraspingState, EndEffectorState.GraspingState]:
        """获取灵巧手的抓取状态。

        Note:
            该功能尚未实现。

        Returns:
            Tuple[EndEffectorState.GraspingState, EndEffectorState.GraspingState]: 灵巧手的抓取状态。
        """
        raise NotImplementedError("This function is not implemented yet")


class TouchDexterousHand(DexterousHand):
    """触觉灵巧手控制类，继承自普通灵巧手控制类，可调用普通灵巧手控制类中的所有方法"""
    def __init__(self):
        super().__init__()

    def get_touch_state(self)-> Tuple[KuavoDexHandTouchState, KuavoDexHandTouchState]:
        """获取灵巧手的触觉状态。

        Warning:
            该功能仅在触觉灵巧手上可用。

        Returns:
            Tuple[KuavoDexHandTouchState, KuavoDexHandTouchState]
        """
        return self._rb_state.dexhand_touch_state