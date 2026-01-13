from abc import ABC, abstractmethod

class RobotInfoBase(ABC):
    def __init__(self, robot_type: str = "kuavo"):
        self._robot_type = robot_type

    @property
    def robot_type(self) -> str:
        """
            Get the robot type, e.g. "kuavo"
        """
        return self._robot_type
    
    @property
    @abstractmethod
    def robot_version(self) -> str:
        """
            Get the robot version, e.g. "42", "43"
        """
        pass

    @property
    @abstractmethod
    def end_effector_type(self) -> str:
        """
            Get the end effector type, e.g. "lejuclaw"...
        """
        pass

    @property
    @abstractmethod
    def joint_names(self) -> list:
        """
            Get the joint names, e.g. ["joint1", "joint2", ...]
        """
        pass

    @property
    @abstractmethod
    def joint_dof(self) -> int:
        """
            Get the joint degrees of freedom, e.g. 28
        """
        pass

    @property
    @abstractmethod
    def arm_joint_dof(self) -> int:
        """
            Get the arm joint degrees of freedom, e.g. 14
        """
        pass

    def __str__(self) -> str:
       pass
    