from abc import ABC, abstractmethod
from typing import Tuple
from kuavo_humanoid_sdk.interfaces.data_types import EndEffectorSide, EndEffectorState
    
class EndEffector(ABC):
    def __init__(self, joint_names: list):
        self._joint_names = joint_names

    def joint_names(self)->list:
        """Returns the joint names of the end effector.
        
        Returns:
            list: The joint names of the end effector.
        """
        return self._joint_names
    
    def joint_count(self) -> int:
        """Returns the total number of joints in the end effector.
        
        The joint_names list contains joints for both left and right end effectors.
        For a single end effector, the number of joints would be joint_names.size/2.
        
        Returns:
            int: The total number of joints in the end effector.
        """
        return len(self._joint_names)
    
    @abstractmethod
    def control(self, target_positions:list, target_velocities:list, target_torques:list)->bool:
        pass

    @abstractmethod
    def control_right(self, target_positions:list, target_velocities:list, target_torques:list)->bool:
        pass

    @abstractmethod
    def control_left(self, target_positions:list, target_velocities:list, target_torques:list)->bool:
        pass    

    @abstractmethod
    def open(self, side:EndEffectorSide)->bool:
        pass

    @abstractmethod
    def get_state(self)->Tuple[EndEffectorState, EndEffectorState]:
        pass

    @abstractmethod
    def get_position(self)->Tuple[list, list]:
        pass

    @abstractmethod
    def get_velocity(self)->Tuple[list, list]:
        pass

    @abstractmethod
    def get_effort(self)->Tuple[list, list]:
        pass

    @abstractmethod
    def get_grasping_state(self)->Tuple[EndEffectorState.GraspingState, EndEffectorState.GraspingState]:
        pass