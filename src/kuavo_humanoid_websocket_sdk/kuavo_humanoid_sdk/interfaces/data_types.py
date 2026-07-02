from typing import Tuple
from enum import Enum
from dataclasses import dataclass
import numpy as np

@dataclass
class KuavoJointData:
    """Data class representing joint states of the robot.

    Args:
        position (list): List of joint positions (angles) in radians
        velocity (list): List of joint velocities in radians/second
        torque (list): List of joint torques/efforts in Newton-meters or Amperes
        acceleration (list): List of joint accelerations in radians/second^2
    """
    position: list
    velocity: list
    torque: list
    acceleration:list

@dataclass
class KuavoImuData:
    """Data class representing IMU (Inertial Measurement Unit) data from the robot.

    Args:
        gyro (Tuple[float, float, float]): Angular velocity around x, y, z axes in rad/s
        acc (Tuple[float, float, float]): Linear acceleration in x, y, z axes in m/s^2
        free_acc (Tuple[float, float, float]): Free acceleration (gravity compensated) in x, y, z axes in m/s^2
        quat (Tuple[float, float, float, float]): Orientation quaternion (x, y, z, w)
    """
    gyro : Tuple[float, float, float]
    acc : Tuple[float, float, float]
    free_acc: Tuple[float, float, float]
    quat: Tuple[float, float, float, float]

@dataclass
class KuavoOdometry:
    """Data class representing odometry data from the robot.

    Args:
        position (Tuple[float, float, float]): Robot position (x, y, z) in world coordinates in meters
        orientation (Tuple[float, float, float, float]): Robot orientation as quaternion (x, y, z, w)
        linear (Tuple[float, float, float]): Linear velocity (x, y, z) in world coordinates in m/s
        angular (Tuple[float, float, float]): Angular velocity (x, y, z) in world coordinates in rad/s
    """
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]
    linear: Tuple[float, float, float]
    angular: Tuple[float, float, float]

class KuavoArmCtrlMode(Enum):
    """Enum class representing the control modes for the Kuavo robot arm.

    Attributes:
        ArmFixed: The robot arm is fixed in position (value: 0)
        AutoSwing: The robot arm is in automatic swinging mode (value: 1)
        ExternalControl: The robot arm is controlled by external commands (value: 2)
    """
    ArmFixed = 0
    AutoSwing = 1 
    ExternalControl = 2

class KuavoManipulationMpcFrame(Enum):
    """Enum class representing the manipulation mpc frame for the Kuavo robot end effector.

    Attributes:
        KeepCurrentFrame: Keep the current frame (value: 0)
        WorldFrame: World frame (value: 1)
        LocalFrame: Local frame (value: 2)
        VRFrame: VR frame (value: 3)
        ManipulationWorldFrame: Manipulation world frame (value: 4)
    """
    KeepCurrentFrame = 0
    WorldFrame = 1
    LocalFrame = 2
    VRFrame = 3
    ManipulationWorldFrame = 4
    ERROR = -1

class KuavoManipulationMpcCtrlMode(Enum):
    """Enum class representing the control mode for the Kuavo robot manipulation MPC.

    Attributes:
        NoControl: No control (value: 0)
        ArmOnly: Only control the arm (value: 1)
        BaseOnly: Only control the base (value: 2)
        BaseArm: Control both the base and the arm (value: 3)
        ERROR: Error state (value: -1)
    """
    NoControl = 0
    ArmOnly = 1
    BaseOnly = 2
    BaseArm = 3
    ERROR = -1

class KuavoManipulationMpcControlFlow(Enum):
    """Enum class representing the control data flow for the Kuavo robot manipulation.

    Attributes:
        ThroughFullBodyMpc: Control data flows through full-body MPC before entering WBC (value: 0)
        DirectToWbc: Control data flows directly to WBC without full-body MPC (value: 1)
        Error: Invalid control path (value: -1)
    """
    ThroughFullBodyMpc = 0
    DirectToWbc = 1
    Error = -1

@dataclass
class EndEffectorState:
    """Data class representing the state of the end effector.

    Args:
        position (list): float, Position of the end effector, range: [0, 100]
        velocity (list): float, ...
        effort (list): float, ...
    """
    position: list
    velocity: list
    effort: list
    class GraspingState(Enum):
        """Enum class representing the grasping states of the end effector.

        Attributes:
            ERROR: Error state (value: -1)
            UNKNOWN: Unknown state (value: 0)
            REACHED: Target position reached (value: 1)
            MOVING: Moving to target position (value: 2)
            GRABBED: Object successfully grasped (value: 3)
        """
        ERROR = -1
        UNKNOWN = 0
        MOVING = 1
        REACHED = 2
        GRABBED = 3

    state: GraspingState # gripper grasping states

class EndEffectorSide(Enum):
    """Enum class representing the sides of the end effector.

    Attributes:
        LEFT: The left side of the end effector (value: 'left')
        RIGHT: The right side of the end effector (value: 'right')
        BOTH: Both sides of the end effector (value: 'both')
    """
    LEFT = 'left'
    RIGHT = 'right'
    BOTH = 'both'

@dataclass
class KuavoPose:
    """Data class representing the pose of the robot."""
    position: Tuple[float, float, float] # x, y, z
    orientation: Tuple[float, float, float, float] # x, y, z, w

@dataclass
class KuavoIKParams:
    """Data class representing the parameters for the IK node."""
    # snopt params
    major_optimality_tol: float = 1e-3
    major_feasibility_tol: float = 1e-3  
    minor_feasibility_tol: float = 1e-3
    major_iterations_limit: float = 100
    # constraint and cost params
    oritation_constraint_tol: float = 1e-3
    pos_constraint_tol: float = 1e-3 # 0.001m, work when pos_cost_weight==0.0
    pos_cost_weight: float = 0.0 # If U need high accuracy, set this to 0.0 !!!

@dataclass
class KuavoDexHandTouchState:
    """Data class representing the touch state of the dexterous hand."""
    
    @dataclass
    class KuavoTouchState:
        """Data class representing the touch state of the dexterous hand."""
        normal_force1: int  # 法向力1
        normal_force2: int  # 法向力2
        normal_force3: int  # 法向力3
        tangential_force1: int  # 切向力1
        tangential_force2: int  # 切向力2
        tangential_force3: int  # 切向力3
        tangential_direction1: int  # 切向力方向1
        tangential_direction2: int  # 切向力方向2
        tangential_direction3: int  # 切向力方向3
        self_proximity1: int  # 自电容接近传感器1
        self_proximity2: int  # 自电容接近传感器2
        mutual_proximity: int  # 互电容接近传感器
        status: int  # 传感器状态
    # 5 fingers
    data: Tuple[KuavoTouchState, KuavoTouchState, KuavoTouchState, KuavoTouchState, KuavoTouchState]

@dataclass
class AprilTagData:
    """Represents detected AprilTag information with pose estimation.
    
    Attributes:
        id (list): List of detected AprilTag IDs (integers)
        size (list): List of tag physical sizes in meters (floats)
        pose (list): List of PoseQuaternion objects representing tag poses
    """
    id: list
    size: list
    pose: list

@dataclass
class HomogeneousMatrix:
    """4x4 homogeneous transformation matrix for 3D transformations.
    
    Represents both rotation and translation in 3D space. Can be used for
    coordinate frame transformations and pose composition.
    
    Attributes:
        matrix (np.ndarray): 4x4 numpy array of shape (4, 4) containing::
        
            [[R, t],
             [0, 1]] 
            
        where R is 3x3 rotation matrix and t is 3x1 translation
    """
    matrix: np.ndarray  # Shape (4,4) homogeneous transformation matrix

@dataclass
class PoseQuaternion:
    """3D pose representation using position and quaternion orientation.
    
    Provides a singularity-free orientation representation. Commonly used
    in robotics for smooth interpolation between orientations.
    
    Attributes:
        position (Tuple[float, float, float]): XYZ coordinates in meters
        orientation (Tuple[float, float, float, float]): Unit quaternion in
            (x, y, z, w) format following ROS convention
    """
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]


@dataclass
class KuavoJointCommand:
    """Data class representing joint command for the robot.

    Args:
        joint_q (list): List of joint position commands in radians
        joint_v (list): List of joint velocity commands in radians/second
        tau (list): List of joint torque/effort commands in Newton-meters or Amperes
        tau_max (list): List of maximum allowable torques for each joint
        tau_ratio (list): List of torque ratios (actual/maximum) for each joint
        joint_kp (list): List of position control proportional gains
        joint_kd (list): List of velocity control derivative gains
        control_modes (list): List of control mode integers for each joint
    """
    joint_q: list
    joint_v: list
    tau: list
    tau_max: list
    tau_ratio: list
    joint_kp: list
    joint_kd: list
    control_modes: list


@dataclass
class KuavoTwist:
    """Data class representing twist (velocity) data for the robot.

    Args:
        linear (Tuple[float, float, float]): Linear velocity (x, y, z) 
        angular (Tuple[float, float, float]): Angular velocity (x, y, z)
    """
    linear: Tuple[float, float, float]
    angular: Tuple[float, float, float]



@dataclass
class KuavoMotorParam:
    """表示机器人电机参数的数据类"""
    Kp: float
    """位置控制比例增益"""
    Kd: float
    """速度控制微分增益"""
    id: int
    """电机ID"""


@dataclass
class AprilTagDetection:
    """表示AprilTag检测结果的数据类"""
    
    @dataclass
    class Point:
        x: float
        y: float
        z: float

    @dataclass
    class Quaternion:
        x: float
        y: float
        z: float
        w: float

    position: Point
    orientation: Quaternion