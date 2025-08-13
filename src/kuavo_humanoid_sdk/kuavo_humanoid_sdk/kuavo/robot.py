#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.kuavo.core.ros_env import KuavoROSEnv
from kuavo_humanoid_sdk.interfaces.robot import RobotBase
from kuavo_humanoid_sdk.common.logger import SDKLogger, disable_sdk_logging
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, KuavoIKParams, KuavoManipulationMpcFrame, KuavoManipulationMpcCtrlMode, KuavoManipulationMpcControlFlow
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore

from typing import Tuple
from kuavo_humanoid_sdk.kuavo.robot_info import KuavoRobotInfo
from kuavo_humanoid_sdk.kuavo.robot_arm import KuavoRobotArm 
from kuavo_humanoid_sdk.kuavo.robot_head import KuavoRobotHead

"""
Kuavo SDK - Kuavo机器人控制的Python接口

本模块提供了通过Python控制Kuavo机器人的主要接口。
包含两个主要类:

KuavoSDK:
    一个静态类,提供SDK初始化和配置功能，以及处理核心设置、ROS环境初始化和日志配置。

KuavoRobot:
    主要的机器人控制接口类,提供以下功能的访问:
    - 机器人信息和状态 (通过 KuavoRobotInfo)
    - 机械臂控制功能 (通过 KuavoRobotArm)
    - 头部控制功能 (通过 KuavoRobotHead)
    - 核心机器人功能 (通过 KuavoRobotCore)
    
该模块需要正确配置的ROS环境才能运行。
"""
__all__ = ["KuavoSDK", "KuavoRobot"]


class KuavoSDK:
    class Options:
        Normal = 0x01
        WithIK = 0x02

    def __init__(self):
       pass

    @staticmethod   
    def Init(options:int=Options.Normal, log_level: str = "INFO")-> bool:
        """初始化SDK。
        
        使用指定的选项和配置初始化Kuavo SDK。
        
        Args:
            options (int): SDK的配置选项。使用: :class:`KuavoSDK.Options` 常量，默认为Options.Normal。
            log_level (str): 日志级别。可选值为"ERROR"、"WARN"、"INFO"、"DEBUG"，默认为"INFO"。
                
        Returns:
            bool: 初始化成功返回True,否则返回False。
            
        Raises:
            RuntimeError: 如果由于缺少依赖项或连接问题导致初始化失败。
        """

        SDKLogger.setLevel(log_level.upper())
        SDKLogger.debug(f" ================= Kuavo Humanoid SDK =================")
        kuavo_ros_env = KuavoROSEnv()
        if not kuavo_ros_env.Init():
            raise RuntimeError("Failed to initialize ROS environment")
        
        # Initialize core components, connect ROS Topics...
        kuavo_core = KuavoRobotCore()
        if log_level.upper() == 'DEBUG':
            debug = True
        else:
            debug = False   
        # Check if IK option is enabled

        if options & KuavoSDK.Options.WithIK:
            if not KuavoROSEnv.check_rosnode_exists('/arms_ik_node'):
                print("\033[31m\nError:WithIK option is enabled but ik_node is not running, please run `roslaunch motion_capture_ik ik_node.launch`\033[0m")      
                exit(1)


        if not kuavo_core.initialize(debug=debug):
            SDKLogger.error("[SDK] Failed to initialize core components.")
            return False
        
        return True
    
    @staticmethod
    def DisableLogging():
        """禁用SDK的所有日志输出。"""
        disable_sdk_logging()

class KuavoRobot(RobotBase):
    def __init__(self):
        super().__init__(robot_type="kuavo")
        
        self._robot_info = KuavoRobotInfo()
        self._robot_arm  = KuavoRobotArm()
        self._robot_head = KuavoRobotHead()
        self._kuavo_core = KuavoRobotCore()
    def stance(self)->bool:
        """使机器人进入'stance'站立模式。
        
        Returns:
            bool: 如果机器人成功进入站立模式返回 True,否则返回 False。
            
        Note:
            你可以调用 :meth:`KuavoRobotState.wait_for_stance` 来等待机器人进入 stance 模式。
        """
        return self._kuavo_core.to_stance()
        
    def trot(self)->bool:
        """使机器人进入'trot'踏步模式。
        
        Returns:
            bool: 如果机器人成功进入踏步模式返回 True,否则返回 False。
            
        Note:
            你可以调用 :meth:`KuavoRobotState.wait_for_walk` 来等待机器人进入踏步模式。
        """
        return self._kuavo_core.to_trot()

    def walk(self, linear_x:float, linear_y:float, angular_z:float)->bool:
        """控制机器人行走运动。
        
        Args:
            linear_x (float): x轴方向的线速度,单位m/s,范围[-0.4, 0.4]。
            linear_y (float): y轴方向的线速度,单位m/s,范围[-0.2, 0.2]。
            angular_z (float): 绕z轴的角速度,单位rad/s,范围[-0.4, 0.4]。
            
        Returns:
            bool: 如果运动控制成功返回 True,否则返回 False。
            
        Note:
            你可以调用 :meth:`KuavoRobotState.wait_for_walk` 来等待机器人进入行走模式。
        """
        # Limit velocity ranges
        limited_linear_x = min(0.4, max(-0.4, linear_x))
        limited_linear_y = min(0.2, max(-0.2, linear_y)) 
        limited_angular_z = min(0.4, max(-0.4, angular_z))
        
        # Check if any velocity exceeds limits.
        if abs(linear_x) > 0.4:
            SDKLogger.warn(f"[Robot] linear_x velocity {linear_x} exceeds limit [-0.4, 0.4], will be limited")
        if abs(linear_y) > 0.2:
            SDKLogger.warn(f"[Robot] linear_y velocity {linear_y} exceeds limit [-0.2, 0.2], will be limited")
        if abs(angular_z) > 0.4:
            SDKLogger.warn(f"[Robot] angular_z velocity {angular_z} exceeds limit [-0.4, 0.4], will be limited")
        return self._kuavo_core.walk(limited_linear_x, limited_linear_y, limited_angular_z)

    def jump(self):
        """使机器人跳跃。
        
        Warning:
            此函数暂未实现，无法调用
        """
        raise NotImplementedError("跳跃功能尚未实现")

    def squat(self, height: float, pitch: float=0.0)->bool:
        """控制机器人的蹲姿高度和俯仰角。

        Args:
                height (float): 相对于正常站立高度的高度偏移量,单位米,范围[-0.35, 0.0],负值表示下蹲。
                pitch (float): 机器人躯干的俯仰角,单位弧度,范围[-0.4, 0.4]。
            
        Returns:
            bool: 如果蹲姿控制成功返回True,否则返回False。
        """
        # Limit height range
        MAX_HEIGHT = 0.0
        MIN_HEIGHT = -0.35
        MAX_PITCH = 0.4
        MIN_PITCH = -0.4
        
        limited_height = min(MAX_HEIGHT, max(MIN_HEIGHT, height))
        
        # Check if height exceeds limits
        if height > MAX_HEIGHT or height < MIN_HEIGHT:
            SDKLogger.warn(f"[Robot] height {height} exceeds limit [{MIN_HEIGHT}, {MAX_HEIGHT}], will be limited")
        # Limit pitch range
        limited_pitch = min(MAX_PITCH, max(MIN_PITCH, pitch))
        
        # Check if pitch exceeds limits
        if abs(pitch) > MAX_PITCH:
            SDKLogger.warn(f"[Robot] pitch {pitch} exceeds limit [{MIN_PITCH}, {MAX_PITCH}], will be limited")
        
        return self._kuavo_core.squat(limited_height, limited_pitch)
     
    def step_by_step(self, target_pose:list, dt:float=0.4, is_left_first_default:bool=True, collision_check:bool=True)->bool:
        """单步控制机器人运动。
        
        Args:
            target_pose (list): 机器人的目标位姿[x, y, z, yaw],单位m,rad。
            dt (float): 每步之间的时间间隔,单位秒。默认为0.4秒。
            is_left_first_default (bool): 是否先迈左脚。默认为True。
            collision_check (bool): 是否进行碰撞检测。默认为True。
            
        Returns:
            bool: 如果运动成功返回True,否则返回False。
            
        Raises:
            RuntimeError: 如果在尝试控制步态时机器人不在stance状态。
            ValueError: 如果target_pose长度不为4。

        Note:
            你可以调用 :meth:`KuavoRobotState.wait_for_step_control` 来等待机器人进入step-control模式。
            你可以调用 :meth:`KuavoRobotState.wait_for_stance` 来等待step-control完成。
        """    
        if len(target_pose) != 4:
            raise ValueError(f"[Robot] target_pose length must be 4 (x, y, z, yaw), but got {len(target_pose)}")

        return self._kuavo_core.step_control(target_pose, dt, is_left_first_default, collision_check)

    def control_command_pose(self, target_pose_x: float, target_pose_y: float, target_pose_z: float, target_pose_yaw: float) -> bool:
        """在base_link坐标系下控制机器人姿态。
        
        Args:
            target_pose_x (float): 目标x位置,单位米。
            target_pose_y (float): 目标y位置,单位米。
            target_pose_z (float): 目标z位置,单位米。
            target_pose_yaw (float): 目标偏航角,单位弧度。
            
        Returns:
            bool: 如果命令发送成功返回True,否则返回False。
            
        Raises:
            RuntimeError: 如果在尝试控制姿态时机器人不在stance状态。
            
        Note:
            此命令会将机器人状态改变为'command_pose'。
        """
        return self._kuavo_core.control_command_pose(target_pose_x, target_pose_y, target_pose_z, target_pose_yaw)

    def control_command_pose_world(self, target_pose_x: float, target_pose_y: float, target_pose_z: float, target_pose_yaw: float) -> bool:
        """在odom(世界)坐标系下控制机器人姿态。
        
        Args:
            target_pose_x (float): 目标x位置,单位米。
            target_pose_y (float): 目标y位置,单位米。
            target_pose_z (float): 目标z位置,单位米。
            target_pose_yaw (float): 目标偏航角,单位弧度。
            
        Returns:
            bool: 如果命令发送成功返回True,否则返回False。
            
        Raises:
            RuntimeError: 如果在尝试控制姿态时机器人不在stance状态。
            
        Note:
            此命令会将机器人状态改变为'command_pose_world'。
        """
        return self._kuavo_core.control_command_pose_world(target_pose_x, target_pose_y, target_pose_z, target_pose_yaw)
    
    def control_head(self, yaw: float, pitch: float)->bool:
        """控制机器人的头部。
        
        Args:
            yaw (float): 头部的偏航角,单位弧度,范围[-1.396, 1.396](-80到80度)。
            pitch (float): 头部的俯仰角,单位弧度,范围[-0.436, 0.436](-25到25度)。
            
        Returns:
            bool: 如果头部控制成功返回True,否则返回False。
        """
        return self._robot_head.control_head(yaw=yaw, pitch=pitch)
    
    def enable_head_tracking(self, target_id: int)->bool:
        """启用头部跟踪 April Tag"""
        return self._robot_head.enable_head_tracking(target_id)
    
    def disable_head_tracking(self)->bool:
        """禁用头部跟踪。"""
        return self._robot_head.disable_head_tracking()
    
    """ Robot Arm Control """
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
        return self._robot_arm.control_hand_wrench(left_wrench, right_wrench)

    def arm_reset(self)->bool:
        """手臂归位
        
        Returns:
            bool: 如果手臂归位成功返回True,否则返回False。
        """
        return self._robot_arm.arm_reset()
    
    def manipulation_mpc_reset(self)->bool:
        """重置机器人手臂。
        
        Returns:
            bool: 如果手臂重置成功返回True,否则返回False。
        """
        return self._robot_arm.manipulation_mpc_reset()
    
    def control_arm_joint_positions(self, joint_positions:list)->bool:
        """通过关节位置角度控制手臂
        
        Args:
            joint_positions (list): 手臂的目标关节位置,单位弧度。
            
        Returns:
            bool: 如果手臂控制成功返回True,否则返回False。
            
        Raises:
            ValueError: 如果关节位置列表长度不正确。
            ValueError: 如果关节位置超出[-π, π]范围。
            RuntimeError: 如果在尝试控制手臂时机器人不在stance状态。
        """
        if len(joint_positions) != self._robot_info.arm_joint_dof:
            print("The length of the position list must be equal to the number of DOFs of the arm.")
            return False
        
        return self._robot_arm.control_arm_joint_positions(joint_positions)
    
    def control_arm_joint_trajectory(self, times:list, q_frames:list)->bool:
        """控制机器人手臂的目标轨迹。
        
        Args:
            times (list): 时间间隔列表,单位秒。
            q_frames (list): 关节位置列表,单位弧度。
            
        Returns:
            bool: 如果控制成功返回True,否则返回False。
            
        Raises:
            ValueError: 如果times列表长度不正确。
            ValueError: 如果关节位置列表长度不正确。
            ValueError: 如果关节位置超出[-π, π]范围。
            RuntimeError: 如果在尝试控制手臂时机器人不在stance状态。
            
        Warning:
            异步接口，函数在发送命令后立即返回，用户需要自行等待运动完成。
        """
        return self._robot_arm.control_arm_joint_trajectory(times, q_frames)

    def set_fixed_arm_mode(self) -> bool:
        """固定/冻结机器人手臂。
        
        Returns:
            bool: 如果手臂固定/冻结成功返回True,否则返回False。
        """
        return self._robot_arm.set_fixed_arm_mode()

    def set_auto_swing_arm_mode(self) -> bool:
        """机器人手臂自动摆动。
        
        Returns:
            bool: 如果切换手臂自动摆动模式成功返回True,否则返回False。
        """
        return self._robot_arm.set_auto_swing_arm_mode()
    
    def set_external_control_arm_mode(self) -> bool:
        """切换手臂控制模式到外部控制模式。
        
        Returns:
            bool: 如果切换手臂控制模式到外部控制模式成功返回True,否则返回False。
        """
        return self._robot_arm.set_external_control_arm_mode()
    
    def set_manipulation_mpc_mode(self, ctrl_mode: KuavoManipulationMpcCtrlMode) -> bool:
        """
        设置 Manipulation MPC 模式。
        Returns:
            bool: 如果 Manipulation MPC 模式设置成功返回True,否则返回False。
        """
        return self._robot_arm.set_manipulation_mpc_mode(ctrl_mode)
    
    def set_manipulation_mpc_control_flow(self, control_flow: KuavoManipulationMpcControlFlow) -> bool:
        """
        设置 Manipulation MPC 控制流。
        Returns:
            bool: 如果 Manipulation MPC 控制流设置成功返回True,否则返回False。
        """ 
        return self._robot_arm.set_manipulation_mpc_control_flow(control_flow)

    def set_manipulation_mpc_frame(self, frame: KuavoManipulationMpcFrame) -> bool:
        """
        设置 Manipulation MPC 坐标系。
        Returns:
            bool: 如果 Manipulation MPC 坐标系设置成功返回True,否则返回False。
        """
        return self._robot_arm.set_manipulation_mpc_frame(frame)
    
    """ Arm Forward kinematics && Arm Inverse kinematics """
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
        return self._robot_arm.arm_ik(left_pose, right_pose, left_elbow_pos_xyz, right_elbow_pos_xyz, arm_q0, params)

    def arm_fk(self, q: list) -> Tuple[KuavoPose, KuavoPose]:
        """机器人手臂的正运动学求解
        
        Args:
            q (list): 关节位置列表,单位弧度。
            
        Returns:
            Tuple[KuavoPose, KuavoPose]: 左手臂和右手臂的位姿元组,
                如果正运动学失败则返回(None, None)。

        Warning:
            此函数需要使用 :attr:`KuavoSDK.Options.WithIK` 选项初始化SDK。        
        """
        return self._robot_arm.arm_fk(q)

    def control_robot_end_effector_pose(self, left_pose: KuavoPose, right_pose: KuavoPose, frame: KuavoManipulationMpcFrame)->bool:
        """通过手臂末端执行器的位姿控制机器人手臂
        
        Args:
            left_pose (KuavoPose): 左手臂的位姿,包含xyz和四元数。
            right_pose (KuavoPose): 右手臂的位姿,包含xyz和四元数。
            frame (KuavoManipulationMpcFrame): 手臂的坐标系。
            
        Returns:
            bool: 如果控制成功返回True,否则返回False。
        """
        return self._robot_arm.control_robot_end_effector_pose(left_pose, right_pose, frame)

    def change_motor_param(self, motor_param:list)->Tuple[bool, str]:
        """更改电机参数

        Args:
            motor_param (list): :class:`kuavo_humanoid_sdk.interfaces.data_types.KuavoMotorParam` 对象列表,包含:
                - Kp (float): 位置控制比例增益
                - Kd (float): 速度控制微分增益 
                - id (int): 电机ID
        Returns:
            Tuple[bool, str]: 成功标志和消息的元组
        """
        return self._kuavo_core.change_motor_param(motor_param)
    
    def get_motor_param(self)->Tuple[bool, list]:
        """获取电机参数

        Returns:
            Tuple[bool, list]: 成功标志和 :class:`kuavo_humanoid_sdk.interfaces.data_types.KuavoMotorParam` 对象列表的元组
        """
        return self._kuavo_core.get_motor_param()

    def enable_base_pitch_limit(self, enable: bool) -> Tuple[bool, str]:
        """开启/关闭机器人 basePitch 限制
        
        Note:
             该接口用于关闭或开启机器人 basePitch 保护功能，关闭状态下可以进行比较大幅度的前后倾动作而不会触发保护导致摔倒。

        Args:
            enable (bool): 开启/关闭
        """
        return self._kuavo_core.enable_base_pitch_limit(enable)
    
    def is_arm_collision(self)->bool:
        """判断当前是否发生碰撞
        
        Returns:
            bool: 发生碰撞返回True,否则返回False
        """
        return self._robot_arm.is_arm_collision()
    
    def wait_arm_collision_complete(self):
        """等待碰撞完成
        """
        self._robot_arm.wait_arm_collision_complete()

    def release_arm_collision_mode(self):
        """释放碰撞模式
        """
        self._robot_arm.release_arm_collision_mode()

    def set_arm_collision_mode(self, enable: bool):
        """设置碰撞模式
        """
        self._robot_arm.set_arm_collision_mode(enable)

if __name__ == "__main__":
    robot = KuavoRobot()
    robot.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
    robot.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.DirectToWbc)
    robot.set_manipulation_mpc_frame(KuavoManipulationMpcFrame.WorldFrame)
    robot.control_robot_end_effector_pose(KuavoPose(position=[0.3, 0.4, 0.9], orientation=[0.0, 0.0, 0.0, 1.0]), KuavoPose(position=[0.3, -0.5, 1.0], orientation=[0.0, 0.0, 0.0, 1.0]), KuavoManipulationMpcFrame.WorldFrame)
