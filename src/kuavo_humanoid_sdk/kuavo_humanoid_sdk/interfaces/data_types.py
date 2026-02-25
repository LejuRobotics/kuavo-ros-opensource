from typing import Tuple
from enum import Enum
from dataclasses import dataclass
import numpy as np

@dataclass
class KuavoJointData:
    """表示机器人关节状态的数据类"""
    position: list
    """关节位置（角度）列表，单位为弧度"""
    velocity: list
    """关节速度列表，单位为弧度/秒"""
    torque: list
    """关节扭矩/力矩列表，单位为牛顿·米或安培"""
    acceleration:list
    """关节加速度列表，单位为弧度/秒²"""

@dataclass
class KuavoImuData:
    """表示机器人IMU（惯性测量单元）数据的数据类"""
    gyro : Tuple[float, float, float]
    """绕x、y、z轴的角速度，单位为弧度/秒"""
    acc : Tuple[float, float, float]
    """x、y、z轴的线性加速度，单位为米/秒²"""
    free_acc: Tuple[float, float, float]
    """自由加速度（重力补偿），x、y、z轴，单位为米/秒²"""
    quat: Tuple[float, float, float, float]
    """方向四元数 (x, y, z, w)"""

@dataclass
class KuavoOdometry:
    """表示机器人里程计数据的数据类"""
    position: Tuple[float, float, float]
    """机器人在世界坐标系中的位置 (x, y, z)，单位为米"""
    orientation: Tuple[float, float, float, float]
    """机器人方向四元数 (x, y, z, w)"""
    linear: Tuple[float, float, float]
    """世界坐标系中的线性速度 (x, y, z)，单位为米/秒"""
    angular: Tuple[float, float, float]
    """世界坐标系中的角速度 (x, y, z)，单位为弧度/秒"""

class KuavoArmCtrlMode(Enum):
    """表示Kuavo机器人手臂控制模式的枚举类。"""
    ArmFixed = 0
    """手臂固定(冻结)模式"""
    AutoSwing = 1 
    """自动摆臂模式"""
    ExternalControl = 2
    """手臂由外部命令控制"""

class KuavoManipulationMpcFrame(Enum):
    """表示Kuavo机器人末端执行器 Manipulation MPC 坐标系的枚举类"""
    KeepCurrentFrame = 0
    """保持当前坐标系"""
    WorldFrame = 1
    """世界坐标系"""
    LocalFrame = 2
    """本地坐标系"""
    VRFrame = 3
    """VR坐标系"""
    ManipulationWorldFrame = 4
    """操作世界坐标系"""
    ERROR = -1
    """错误状态"""

class KuavoManipulationMpcCtrlMode(Enum):
    """表示Kuavo机器人 Manipulation MPC 控制模式的枚举类"""
    NoControl = 0
    """无控制"""
    ArmOnly = 1
    """仅控制手臂"""
    BaseOnly = 2
    """仅控制底座"""
    BaseArm = 3
    """同时控制底座和手臂"""
    ERROR = -1
    """错误状态"""

class KuavoManipulationMpcControlFlow(Enum):
    """表示Kuavo机器人 Manipulation MPC 控制数据流的枚举类"""
    ThroughFullBodyMpc = 0  
    """控制数据通过全身MPC后进入WBC"""
    DirectToWbc = 1
    """控制数据直接流向WBC，不经过全身MPC"""
    Error = -1
    """无效的控制路径"""

@dataclass
class EndEffectorState:
    """表示末端执行器状态的数据类。

    参数:
        position (list): 浮点数，末端执行器位置，范围: [0, 100] \n
        velocity (list): 浮点数，速度 \n
        effort (list): 浮点数，力矩 \n
        state (GraspingState): 夹爪抓取状态 \n
    """
    position: list
    velocity: list
    effort: list
    class GraspingState(Enum):
        """表示末端执行器抓取状态的枚举类。"""

        ERROR = -1
        """错误状态"""
        UNKNOWN = 0
        """未知状态"""
        MOVING = 1
        """正在移动到目标位置"""
        REACHED = 2
        """到达目标位置"""
        GRABBED = 3
        """成功抓取物体"""

    state: GraspingState

class EndEffectorSide(Enum):
    """表示末端执行器类型的枚举类。"""
    LEFT = 'left'
    """左末端执行器"""
    RIGHT = 'right'
    """右末端执行器"""
    BOTH = 'both'
    """左右末端执行器"""

@dataclass
class KuavoPose:
    """表示机器人姿态的数据类。"""
    position: Tuple[float, float, float] # x, y, z
    orientation: Tuple[float, float, float, float] # x, y, z, w

    def __str__(self) -> str:
        """Returns a formatted string representation of the pose.
        
        Returns:
            str: Formatted pose string with position and orientation
        """
        return (
            f"Position (x,y,z): ({self.position[0]:.3f}, {self.position[1]:.3f}, {self.position[2]:.3f})\n"
            f"Orientation (x,y,z,w): ({self.orientation[0]:.3f}, {self.orientation[1]:.3f}, "
            f"{self.orientation[2]:.3f}, {self.orientation[3]:.3f})"
        )

@dataclass
class KuavoIKParams:
    """表示IK节点参数的数据类。"""
    # snopt参数
    major_optimality_tol: float = 1e-3
    major_feasibility_tol: float = 1e-3  
    minor_feasibility_tol: float = 1e-3
    major_iterations_limit: float = 100
    # 约束和成本参数
    oritation_constraint_tol: float = 1e-3
    pos_constraint_tol: float = 1e-3 # 0.001m，当pos_cost_weight==0.0时生效
    pos_cost_weight: float = 0.0 # 如果需要高精度，请将此值设为0.0！！！

@dataclass
class KuavoDexHandTouchState:
    """表示灵巧手触觉状态的数据类。"""
    
    @dataclass
    class KuavoTouchState:
        """表示灵巧手触觉状态的数据类"""
        normal_force1: int  # 法向力1
        """法向力1"""
        normal_force2: int  # 法向力2
        """法向力2"""
        normal_force3: int  # 法向力3
        """法向力3"""
        tangential_force1: int  # 切向力1
        """切向力1"""
        tangential_force2: int  # 切向力2
        """切向力2"""
        tangential_force3: int  # 切向力3
        """切向力3"""
        tangential_direction1: int  # 切向力方向1
        """切向力方向1"""
        tangential_direction2: int  # 切向力方向2
        """切向力方向2"""
        tangential_direction3: int  # 切向力方向3
        """切向力方向3"""
        self_proximity1: int  # 自电容接近传感器1
        """自电容接近传感器1"""
        self_proximity2: int  # 自电容接近传感器2
        """自电容接近传感器2"""
        mutual_proximity: int  # 互电容接近传感器
        """互电容接近传感器"""
        status: int  # 传感器状态
        """传感器状态"""
    # 5个手指
    data: Tuple[KuavoTouchState, KuavoTouchState, KuavoTouchState, KuavoTouchState, KuavoTouchState]

@dataclass
class AprilTagData:
    """表示检测到的AprilTag信息及姿态估计"""
    id: list
    """检测到的AprilTag ID列表（整数）"""   
    size: list
    """标签物理尺寸列表，单位为米（浮点数）"""
    pose: list
    """表示标签姿态的PoseQuaternion对象列表"""

@dataclass
class HomogeneousMatrix:
    """用于3D变换的4x4齐次变换矩阵。
    
    表示3D空间中的旋转和平移。可用于坐标系变换和姿态合成。 \n
    
    属性:
        matrix (np.ndarray): 形状为(4, 4)的4x4 numpy数组，包含::
        
            [[R, t],
             [0, 1]] 
            
        其中R是3x3旋转矩阵，t是3x1平移向量
    """
    matrix: np.ndarray  # 形状(4,4)的齐次变换矩阵

@dataclass
class PoseQuaternion:
    """使用位置和四元数方向的3D姿态表示"""
    position: Tuple[float, float, float]
    """XYZ坐标，单位为米"""
    orientation: Tuple[float, float, float, float]
    """单位四元数，采用(x, y, z, w)格式，遵循ROS约定"""


@dataclass
class KuavoJointCommand:
    """表示机器人关节命令的数据类\n
    """
    joint_q: list
    """关节位置命令列表，单位为弧度"""
    joint_v: list
    """关节速度命令列表，单位为弧度/秒"""
    """关节扭矩/力矩命令列表，单位为牛顿·米或安培"""
    tau: list
    """关节扭矩/力矩命令列表，单位为牛顿·米或安培"""
    tau_max: list
    """每个关节的最大允许扭矩列表"""
    tau_ratio: list
    """每个关节的扭矩比率（实际/最大）列表"""
    joint_kp: list
    """位置控制比例增益列表"""
    joint_kd: list
    """速度控制微分增益列表"""
    control_modes: list
    """每个关节的控制模式整数列表"""


@dataclass
class KuavoTwist:
    """表示机器人扭转（速度）数据的数据类"""
    linear: Tuple[float, float, float]
    """线性速度 (x, y, z)，单位为米/秒"""
    angular: Tuple[float, float, float]
    """角速度 (x, y, z)，单位为弧度/秒"""


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
