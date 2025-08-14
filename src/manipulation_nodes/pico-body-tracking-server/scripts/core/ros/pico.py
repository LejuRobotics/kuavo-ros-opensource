"""
Pico VR device integration with ROS.

This module provides tools for:
- ROS node management
- Pose and gesture handling
- Visualization
- Service control
"""

import os
import sys
import signal
import json
import rospy
import rospkg
import tf
import numpy as np
import enum
import time
import math
from typing import List, Optional, Any
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64MultiArray,Float64
from kuavo_msgs.msg import (
    twoArmHandPoseCmd, robotBodyMatrices, picoPoseInfoList,
    robotHeadMotionData, ikSolveParam, footPoseTargetTrajectories,
    JoySticks, sensorsData, switchGaitByName,
    lejuClawCommand,robotHandPosition,dexhandCommand
)
from kuavo_msgs.srv import changeArmCtrlMode, changeTorsoCtrlMode, changeTorsoCtrlModeRequest, changeArmCtrlMode, changeArmCtrlModeRequest
from kuavo_msgs.srv import fkSrv
from std_msgs.msg import Float32MultiArray, Int32, Bool
from geometry_msgs.msg import Twist
from .pico_utils import KuavoPicoInfoTransformer
from common.logger import SDKLogger
from .joysticks_handler import JoySticksHandler
from typing import Set
from copy import deepcopy
from scipy.spatial.transform import Rotation as R
from ..config.pico_vr_config import PicoVrConfig, HandWrenchConfig
from ..utils.toggle_switch import ToggleSwitch, ToggleEvent

class ControlMode(enum.Enum):
        NONE_MODE = 0
        MOBILE_MPC_MODE = 1    # 运动学MPC模式
        FOLLOW_MODE = 2         # 手臂跟随模式
        INCREMENTAL_MODE = 3    # 增量控制模式

# 速度控制常量定义
class CmdVelConstants:
    """命令速度控制常量"""
    # 线性速度限制 (m/s)
    MAX_FORWARD_SPEED = 0.4      # 最大前进速度
    MAX_BACKWARD_SPEED = 0.2     # 最大后退速度
    
    # 角速度限制 (度/秒)
    MAX_ANGULAR_SPEED_DEG = 10.0  # 最大角速度
    MAX_ANGULAR_SPEED_RAD = MAX_ANGULAR_SPEED_DEG * 3.14159 / 180.0  # 转换为弧度/秒
    
    # 死区阈值
    LINEAR_DEADZONE = 0.05       # 线性速度死区
    ANGULAR_DEADZONE = 0.05      # 角速度死区

# 身体倾斜控制常量定义
class BodyTiltConstants:
    """身体倾斜控制常量"""
    # 倾斜角度限制 (弧度)
    MAX_FORWARD_TILT = 0.2       # 最大前倾角度
    MAX_BACKWARD_TILT = 0.2      # 最大后仰角度
    
    # 死区阈值
    TILT_DEADZONE = 0.05         # 倾斜死区

def clamp(value: float, min_val: float, max_val: float) -> float:
    """限制值在指定范围内"""
    return max(min_val, min(value, max_val))

def apply_deadzone(value: float, deadzone: float) -> float:
    """应用死区，如果值在死区内则返回0"""
    if abs(value) < deadzone:
        return 0.0
    return value


def limit_linear_velocity(joy_x: float) -> float:
    """限制线性速度"""
    # 应用死区
    joy_x = apply_deadzone(joy_x, CmdVelConstants.LINEAR_DEADZONE)
    
    # 根据方向应用不同的速度限制
    if joy_x > 0:
        # 前进
        return clamp(joy_x * CmdVelConstants.MAX_FORWARD_SPEED, 0, CmdVelConstants.MAX_FORWARD_SPEED)
    else:
        # 后退
        return clamp(joy_x * CmdVelConstants.MAX_BACKWARD_SPEED, -CmdVelConstants.MAX_BACKWARD_SPEED, 0)

def limit_angular_velocity(joy_y: float) -> float:
    """限制角速度"""
    # 应用死区
    joy_y = apply_deadzone(joy_y, CmdVelConstants.ANGULAR_DEADZONE)
    
    # 限制角速度范围
    return clamp(joy_y * CmdVelConstants.MAX_ANGULAR_SPEED_RAD, 
                -CmdVelConstants.MAX_ANGULAR_SPEED_RAD, 
                CmdVelConstants.MAX_ANGULAR_SPEED_RAD)

def banner_echo(msg: str, color:str='green'):
    if color == 'green':
        color = "\033[32m"
    elif color == 'red':
        color = "\033[31m"
    elif color == 'yellow':
        color = "\033[33m"
    else:
        color = "\033[0m"
    SDKLogger.info(f"{color}==============================================\033[0m")
    SDKLogger.info(f"{color}{msg}\033[0m")
    SDKLogger.info(f"{color}==============================================\033[0m")


class IncrementalMpcCtrlMode(enum.Enum):
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
    

class KuavoPicoNodeManager:
    """Singleton class for managing ROS node initialization."""
    
    _instance = None
    _node_initialized = False

    @classmethod
    def get_instance(cls, node_name: str = 'kuavo_pico_node') -> 'KuavoPicoNodeManager':
        """Get singleton instance."""
        if not cls._instance:
            cls._instance = cls(node_name)
        return cls._instance

    def __init__(self, node_name: str):
        """Initialize the node manager."""
        if not KuavoPicoNodeManager._node_initialized:
            rospy.init_node(node_name)
            KuavoPicoNodeManager._node_initialized = True

    @staticmethod
    def get_ros_is_shutdown() -> bool:
        """Check if ROS is shutdown."""
        return rospy.is_shutdown()


class KuavoPicoBoneFramePublisher:
    """Class for publishing bone frame information."""
    
    def __init__(self, node_name: str = 'kuavo_pico_bone_frame_publisher'):
        """Initialize the bone frame publisher."""
        SDKLogger.info(f"[{node_name}] Initializing KuavoPicoBoneFramePublisher...")
        self.rate = rospy.Rate(100.0)
        
        # Initialize publishers
        self._init_publishers()
        
        # Create TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        # Create TF listener
        self.tf_listener = tf.TransformListener()

    def _init_publishers(self) -> None:
        """Initialize ROS publishers."""
        self.pose_info_list_pub = rospy.Publisher(
            '/leju_pico_bone_poses', 
            picoPoseInfoList, 
            queue_size=10
        )
        self.head_motion_data_pub = rospy.Publisher(
            '/robot_head_motion_data', 
            robotHeadMotionData, 
            queue_size=10
        )
        self.foot_pose_pub = rospy.Publisher(
            '/humanoid_mpc_foot_pose_world_target_trajectories',
            footPoseTargetTrajectories, 
            queue_size=10
        )

        # PICO 手柄数据
        self.pico_joy_pub = rospy.Publisher('/pico/joy', JoySticks, queue_size=10)

    def publish_pico_joys(self, joy: JoySticks) -> None:
        """Publish pico joys"""
        try:
            if not rospy.is_shutdown():
                if joy is not None:
                    self.pico_joy_pub.publish(joy)
            else:
                SDKLogger.error("ROS node is shutdown, cannot publish pico joys")
        except Exception as e:
            SDKLogger.error(f"Error publishing pico joys: {e}")

    def publish_pose_info_list(self, pose_info_list: picoPoseInfoList) -> None:
        """Publish pose information list."""
        try:
            if not rospy.is_shutdown():
                self.pose_info_list_pub.publish(pose_info_list)
            else:
                SDKLogger.error("ROS node is shutdown, cannot publish pose info list")
        except Exception as e:
            SDKLogger.error(f"Error publishing pose info list: {e}")

    def publish_foot_pose_trajectory(self, foot_pose_trajectory: footPoseTargetTrajectories) -> None:
        """Publish foot pose trajectory."""
        SDKLogger.info(f"publish_foot_pose_trajectory finished[\\\\\\\\\\\\\\\\\\\\\\\\]")
        SDKLogger.info(f"publish_foot_pose_trajectory time: {rospy.Time.now().to_sec()}")
        self.foot_pose_pub.publish(foot_pose_trajectory)

    def publish_hands_info_list(self, hands_info_list: picoPoseInfoList) -> None:
        """Publish hands information list."""
        try:
            if not rospy.is_shutdown():
                self.hands_info_list_pub.publish(hands_info_list)
            else:
                SDKLogger.error("ROS node is shutdown, cannot publish hands info list")
        except Exception as e:
            SDKLogger.error(f"Error publishing hands info list: {e}")

    def publish_tf(self, translation: List[float], rotation: List[float], 
                  child: str, parent: str) -> None:
        """Publish transform."""
        self.tf_broadcaster.sendTransform(
            translation, 
            rotation, 
            rospy.Time.now(), 
            child, 
            parent
        )

    def publish_hand_finger_tf(self, tf_msg: TFMessage) -> None:
        """Publish hand finger transform."""
        self.hand_finger_tf_pub.publish(tf_msg)

    def get_listener(self) -> tf.TransformListener:
        """Get transform listener."""
        return self.tf_listener

    def sleep(self) -> None:
        """Sleep for the configured rate."""
        self.rate.sleep()


class KuavoPicoNode:
    """Main Pico node class for handling VR device integration."""
    
    _instance = None
    
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self, play_mode: bool = False):
        """Initialize the Pico node."""
        if not hasattr(self, '_initialized'):
            KuavoPicoNodeManager.get_instance('kuavo_pico_node')
            self._play_mode = play_mode
            # Initialize parameters
            self._init_parameters()
            # Initialize toggle switch
            self._init_toggle_switch()
            # Initialize model and transformers
            self._init_model_and_transformers()
            # Initialize publishers
            self._init_publishers()
            # Initialize subscribers
            self._init_subscribers()
            # Initialize joy handler
            self._init_joy_controller()

            self.robot_matrix_publisher = RobotMatrixPublisher()

            self._initialized = True
            self.arm_joint_angles = None

            self.control_mode = ControlMode.MOBILE_MPC_MODE

            self._left_target_pose = (None, None)   # tuple(pos, quat), quat(x, y, z, w)
            self._right_target_pose = (None, None)  # tuple(pos, quat), quat(x, y, z, w) 

            self._left_anchor_pose = (None, None)
            self._right_anchor_pose = (None, None)

            self.last_control_mode = None
            self.is_in_incremental_mode = True

            self.ik_error_norm = [0.0, 0.0]

            self.interp_left_pos = None
            self.interp_right_pos = None
            self.control_mode_changing = True

        else:
            # Allow updating play_mode if already initialized
            self._play_mode = play_mode

    def set_play_mode(self, enable: bool):
        """Enable or disable play mode (controls /robot_body_matrices subscription)."""
        if hasattr(self, '_play_mode') and self._play_mode == enable:
            return
        self._play_mode = enable
        # Remove or add the subscriber as needed
        if enable:
            if not hasattr(self, '_robot_body_matrices_sub') or self._robot_body_matrices_sub is None:
                self._robot_body_matrices_sub = rospy.Subscriber(
                    "/robot_body_matrices",
                    robotBodyMatrices,
                    self.robot_body_matrices_callback
                )
        else:
            if hasattr(self, '_robot_body_matrices_sub') and self._robot_body_matrices_sub is not None:
                self._robot_body_matrices_sub.unregister()
                self._robot_body_matrices_sub = None

    def _init_parameters(self) -> None:
        """Initialize node parameters."""
        self.use_custom_ik_param = True
        self.ik_solve_param = ikSolveParam()
        self.set_ik_solver_params()
        
        self.send_srv = True
        self.last_pico_running_state = False
        self.button_y_last = False
        self.config_manager = PicoVrConfig()

        # 末端执行器类型
        self.eef_type = rospy.get_param('/end_effector_type', 'qiangnao')
        self.last_eef_command = None # 记录上一次的末端执行器命令

    def _init_toggle_switch(self) -> None:
        # hand wrench
        def on_hand_wrench_changed(event: ToggleEvent):
            SDKLogger.info(f"末端力施加状态变化: {event.old_state} -> {event.new_state}")
            if event.new_state:
                banner_echo(f"施加末端力", 'green')
                SDKLogger.info(f"末端力: {self.hand_wrench_config}")
                if self.hand_wrench_config:
                    mass_force =  {
                        'item_mass': self.hand_wrench_config.itemMass, # 物品重量 单位 kg
                        'lforce': [self.hand_wrench_config.lforceX, self.hand_wrench_config.lforceY, self.hand_wrench_config.lforceZ],
                        'rforce': [self.hand_wrench_config.lforceX, -self.hand_wrench_config.lforceY, self.hand_wrench_config.lforceZ],
                    }
                    self.publish_hand_wrench_cmd(mass_force)
                else:
                    SDKLogger.error("------------- item_mass_force is None --------------")
            else:
                banner_echo(f"释放末端力", 'yellow')
                mass_force =  {
                    'item_mass': 0.0, # 物品重量 单位 kg
                    'lforce': [0.0, 0.0, 0.0],
                    'rforce': [0.0, 0.0, 0.0],
                }
                self.publish_hand_wrench_cmd(mass_force)
        self.hand_wrench_config = self.config_manager.get_default_hand_wrench_config()
        self.toggle_hand_wrench = ToggleSwitch("hand_wrench") # 末端力施加开关
        self.toggle_hand_wrench.add_callback(on_hand_wrench_changed)
        #//////////////////////////////////
        # freeze finger
        def on_freeze_finger_changed(event: ToggleEvent):
            SDKLogger.info(f"手指锁定状态变化: {event.old_state} -> {event.new_state}")
            if event.new_state:
                banner_echo(f"手指锁定", 'green')
            else:
                banner_echo(f"手指解锁", 'yellow')
        self.toggle_freeze_finger = ToggleSwitch("freeze_finger")
        self.toggle_freeze_finger.add_callback(on_freeze_finger_changed)
        #//////////////////////////////////
        # teleop unlock
        def on_teleop_unlock_changed(event: ToggleEvent):
            SDKLogger.info(f"遥操作锁定状态变化: {event.old_state} -> {event.new_state}")
            if event.new_state:
                if self.pico_info_transformer.control_mode == "UpperBody":
                    self.toggle_auto_swing_arm.turn_off() # 手臂关闭自动摆手模式
                else:
                    self.last_pico_running_state = False
                banner_echo(f"遥操作已解锁", 'green')
            else:
                banner_echo(f"遥操作已锁定", 'yellow')
        self.toggle_teleop_unlock = ToggleSwitch("teleop_unlock")
        self.toggle_teleop_unlock.add_callback(on_teleop_unlock_changed)

        # record_bag
        def on_record_rosbag_changed(event: ToggleEvent):
            SDKLogger.info(f"录制rosbag状态变化: {event.old_state} -> {event.new_state}")
            if event.new_state:
                banner_echo(f"开始录制rosbag", 'green')
                # TODO: 实现开始录制rosbag的逻辑
            else:
                banner_echo(f"停止录制rosbag", 'yellow')
                # TODO: 实现停止录制rosbag的逻辑
        self.toggle_record_bag = ToggleSwitch("record rosbag")
        self.toggle_record_bag.add_callback(on_record_rosbag_changed)
        #//////////////////////////////////
        # switch view
        def on_switch_view_changed(event: ToggleEvent):
            SDKLogger.info(f"切换视角状态变化: {event.old_state} -> {event.new_state}")
            if event.new_state:
                banner_echo(f"切换到第一视角", 'green')
                # TODO: 切换到第一视角
            else:
                banner_echo(f"切换到第三视角", 'yellow')
                # TODO: 切换到第三视角
        self.toggle_switch_view = ToggleSwitch("switch view")
        self.toggle_switch_view.add_callback(on_switch_view_changed)
        #//////////////////////////////////
        # change arm mode
        def on_change_arm_mode_changed(event: ToggleEvent):
            SDKLogger.info(f"切换手臂模式状态变化: {event.old_state} -> {event.new_state}")
            if event.new_state:
                if self.control_mode == ControlMode.FOLLOW_MODE:
                    self.change_arm_ctrl_mode(1)
                elif self.control_mode == ControlMode.INCREMENTAL_MODE or self.control_mode == ControlMode.MOBILE_MPC_MODE:
                    self.change_mobile_ctrl_mode(0)
                    self.change_mm_wbc_arm_ctrl_mode(0)
                    self.change_arm_ctrl_mode(1)
                banner_echo(f"切换到自动摆臂模式", 'green')
            else:
                self.last_pico_running_state = False
                banner_echo(f"切换到外部控制模式", 'yellow')
        self.toggle_auto_swing_arm = ToggleSwitch("switch arm mode") # 切换手臂模式
        self.toggle_auto_swing_arm.add_callback(on_change_arm_mode_changed)
        #//////////////////////////////////
        # increase arm mode
        def on_increase_arm_mode_changed(event: ToggleEvent):
            SDKLogger.info(f"增量手臂模式状态变化: {event.old_state} -> {event.new_state}")
            if self.is_in_incremental_mode:
                banner_echo(f"关闭增量控制模式", 'yellow')
                self.set_in_incremental_mode(False)
            else:
                banner_echo(f"开启增量手臂模式", 'green')
                self.set_in_incremental_mode(True)
        self.toggle_increase_arm_mode = ToggleSwitch("switch increase arm mode")        
        self.toggle_increase_arm_mode.add_callback(on_increase_arm_mode_changed)


    def _init_model_and_transformers(self) -> None:
        """Initialize model and transformers."""
        model_path = self.set_robot_model_params()
        self.bone_frame_publisher = KuavoPicoBoneFramePublisher()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.pico_info_transformer = KuavoPicoInfoTransformer(
            model_path, 
            tf_broadcaster=self.tf_broadcaster,
            bone_frame_publisher=self.bone_frame_publisher
        )

    def _init_publishers(self) -> None:
        """Initialize ROS publishers."""
        self.pub = rospy.Publisher(
            '/mm/two_arm_hand_pose_cmd', 
            twoArmHandPoseCmd, 
            queue_size=10
        )
        self.pub_ik = rospy.Publisher(
            '/ik/two_arm_hand_pose_cmd', 
            twoArmHandPoseCmd, 
            queue_size=10
        )
        self.pub_hand_wrench_cmd = rospy.Publisher( # 发布手臂末端 6 维力
            '/hand_wrench_cmd', 
            Float64MultiArray, 
            queue_size=1)
        
        # 控制模式切换话题发布者
        self.pub_arm_ctrl_mode = rospy.Publisher(
            '/pico/arm_ctrl_mode_change', 
            Int32, 
            queue_size=10
        )
        self.pub_mobile_ctrl_mode = rospy.Publisher(
            '/pico/mobile_ctrl_mode_change', 
            Int32, 
            queue_size=10
        )
        self.pub_mm_wbc_arm_ctrl_mode = rospy.Publisher(
            '/pico/mm_wbc_arm_ctrl_mode_change', 
            Int32, 
            queue_size=10
        )
        
        if self.eef_type == "lejuclaw": # 乐聚夹爪
            self.pub_leju_claw_cmd = rospy.Publisher(
                '/leju_claw_command', 
                lejuClawCommand, 
                queue_size=10)
        elif self.eef_type.startswith('qiangnao'): # 灵巧手
            self.pub_control_robot_hand_position = rospy.Publisher(
                '/control_robot_hand_position', robotHandPosition, queue_size=10
            )
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_cmd_pose = rospy.Publisher('/cmd_pose', Twist, queue_size=10)
        self.pub_switch_gait = rospy.Publisher('/humanoid_switch_gait_by_name', switchGaitByName, queue_size=10)
        self.pub_stop_robot = rospy.Publisher('/stop_robot', Bool, queue_size=10)

    def _init_subscribers(self) -> None:
        """Initialize ROS subscribers."""
        rospy.Subscriber(
            "/leju_pico_bone_poses", 
            picoPoseInfoList, 
            # self.pico_hands_poses_callback
            # self.pico_hands_poses_callback_incremental
            # self.pico_hands_poses_callback_ik
            self.pico_hands_poses_callback_choose
        )
        # Only subscribe to /robot_body_matrices if play_mode is True
        self._robot_body_matrices_sub = None
        if getattr(self, '_play_mode', False):
            self._robot_body_matrices_sub = rospy.Subscriber(
                "/robot_body_matrices",
                robotBodyMatrices,
                self.robot_body_matrices_callback
            )
        rospy.Subscriber("/sensors_data_raw", sensorsData, self.sensors_data_raw_callback)
        rospy.Subscriber("/ik/error_norm", Float32MultiArray, self.ik_error_norm_callback)
        rospy.Subscriber("/pico/control_mode", Int32, self.set_control_mode_callback)
        rospy.Subscriber("/pico/incremental_mode", Int32, self.set_incremental_mode_callback)
    
    """//////////////////////////////////// Joy Callbacks //////////////////////////////////////////"""
    def _init_joy_controller(self) -> None:
        """Initialize joy controller"""
        self.joy_button_handler = JoySticksHandler()
        rospy.Subscriber("/pico/joy", JoySticks, self.sub_joy_callback)
        # 按键回调配置字典
        self.joy_callbacks = {
            "teleop_unlock": ({"LT_PRESSED", "LG_PRESSED"}, self._teleop_unlock_callback),  # 基础模式/全身遥操模式 - 解锁
            "teleop_lock": ({"RT_PRESSED", "RG_PRESSED"}, self._teleop_lock_callback),      # 基础模式/全身遥操模式 - 上锁
            "hand_wrench": ({"LT_PRESSED", "A_PRESSED"}, self._hand_wrench_callback),       # 基础模式/全身遥操模式 - 末端力施加开关
            "freeze_finger": ({"Y_PRESSED"}, self._freeze_finger_callback),                 # 基础模式/全身遥操模式 - 锁定/解锁手指
            # "record_rosbag": ({"X_PRESSED"}, self._record_rosbag_callback),                 # 基础模式/全身遥操模式 - 录制rosbag
            "reset_to_stance": ({"A_PRESSED"}, self._reset_to_stance_callback),             # 基础模式/全身遥操模式 - 切换到站立模式
            "stop_robot": ({"LT_PRESSED","RT_PRESSED", "A_LONG_PRESSED"}, self._stop_robot_callback),# 基础模式/全身遥操模式 - 停止机器人
            # "switch_view": ({"Y_LONG_PRESSED"}, self._switch_view_callback),                # 基础模式/全身遥操模式 - 切换视角
            "enter_walk": ({"B_PRESSED"}, self._enter_walk_callback),                       # 基础模式 - 切换到行走模式
            "auto_swing_arm": ({"LT_PRESSED","Y_PRESSED"}, self._auto_swing_arm_callback),  # 基础模式 - 切换手臂模式
            "switch_increase_arm": ({"LT_PRESSED", "X_PRESSED"}, self._switch_increase_arm_mode_callback)        # 基础模式/全身遥操模式 - 增量模式开启/关闭
        }
        for name, (key_combination, callback) in self.joy_callbacks.items():
            self.joy_button_handler.add_callback(name, key_combination, callback)
        # //////////////////////////// Debug CallBacks ////////////////////////////
        # 遥操模式切换
        dev_debug = True
        if dev_debug:
            def _debug_torso_change_callback(event: ToggleEvent):
                SDKLogger.info(f"切换躯干模式状态变化: {event.old_state} -> {event.new_state}")
                if event.new_state:
                    self.last_pico_running_state = False
                    self.pico_info_transformer.set_control_torso_mode(True)
                    banner_echo(f"切换到躯干模式", 'green')
                else:
                    self.last_pico_running_state = False
                    self.pico_info_transformer.set_control_torso_mode(False)
                    banner_echo(f"切换到非躯干模式", 'yellow')
            self.toggle_torso_change_debug = ToggleSwitch("torso change debug")
            self.toggle_torso_change_debug.add_callback(_debug_torso_change_callback)
            self.joy_debug_callbacks = {
                "torso_on": (set(['RT_PRESSED', 'Y_PRESSED']), self._torso_change_callback),
                "WholeBody_Mode": (set(['RT_PRESSED', 'X_PRESSED']), self._whole_body_callback),
                "UpperBody_Mode": (set(['RT_PRESSED', 'B_PRESSED']), self._upper_body_callback),
                "LowerBody_Mode": (set(['RT_PRESSED', 'A_PRESSED']), self._lower_body_callback)
            }
            for name, (key_combination, callback) in self.joy_debug_callbacks.items():
                self.joy_button_handler.add_callback(name, key_combination, callback)
        # //////////////////////////// Debug CallBacks ////////////////////////////
    def sub_joy_callback(self, joy: JoySticks) -> None:
        """订阅手柄回调"""

        # 更新按键状态，内部处理组合按键回调，另外返回当前按键组合和是否触发了回调函数
        key_combination, callback_triggered = self.joy_button_handler.update(joy)
        
        # 没有包含RG/LG的按键且没有触发回调时发布末端执行器控制, 否则保持上一次的末端执行器命令
        has_rg_lg_keys = any('RG' in key or 'LG' in key for key in key_combination)
        eef_freeze = (callback_triggered and has_rg_lg_keys) or self.toggle_freeze_finger
        self.publish_end_effector_control(joy, eef_freeze)
        
        # 全身遥操不处理左右摇杆控制
        if self.pico_info_transformer.control_mode == "UpperBody":
            if self.toggle_torso_change_debug:
                # SDKLogger.warning("\033[93m躯干模式开启中，无法切换左右摇杆控制\033[0m")
                return
            self.handle_joystick_movement(joy)

    def _hand_wrench_callback(self, key_combination: Set[str], joy:JoySticks)->None:
        """Callback for hand wrench."""
        # 下半身模式不响应
        if self.pico_info_transformer.control_mode == "LowerBody":
            SDKLogger.warning("\033[93m下半身遥操模式下无法切换手部力开关\033[0m")
            return
        if self.toggle_teleop_unlock:
            self.toggle_hand_wrench.toggle() # 切换手部力开关
        else:
            SDKLogger.info("\033[93m遥操未解锁，无法切换手部力开关\033[0m")

    def _freeze_finger_callback(self, key_combination: Set[str], joy:JoySticks)->None:
        """Callback for freeze finger."""
        self.toggle_freeze_finger.toggle() # 锁定/解锁手指

    def _teleop_unlock_callback(self, key_combination: Set[str], joy:JoySticks)->None:
        """Callback for teleoperation unlock."""
        self.toggle_teleop_unlock.turn_on() # 解锁遥操作

    def _teleop_lock_callback(self, key_combination: Set[str], joy:JoySticks)->None:
        """Callback for teleoperation lock."""
        self.toggle_teleop_unlock.turn_off() # 锁定遥操作

    def _record_rosbag_callback(self, key_combination: Set[str], joy:JoySticks)->None:
        """Callback for record rosbag."""
        self.toggle_record_bag.toggle() # 开始/停止录制

    def _switch_view_callback(self, key_combination: Set[str], joy:JoySticks)->None:
        """Callback for switch view."""
        self.toggle_switch_view.toggle() # 切换视角

    def _switch_increase_arm_mode_callback(self, key_combination: Set[str], joy:JoySticks)->None:
        """Callback for switch increament arm mode."""
        if self.pico_info_transformer.control_mode == "LowerBody":
            SDKLogger.warning("\033[93m下半身遥操模式下无法切换增量手臂控制模式\033[0m")
            return

        if self.control_mode != ControlMode.INCREMENTAL_MODE:
            SDKLogger.warning("\033[93m当前模式不是增量控制模式，无法切换增量手臂控制模式\033[0m")
            return
        self.toggle_increase_arm_mode.toggle() # 增量手臂控制模式开启/关闭

    def _enter_walk_callback(self, key_combination: Set[str], joy:JoySticks)->None:
        """Callback for walk mode."""
        # 只在半身遥操模式下响应
        if not self.pico_info_transformer.control_mode == "UpperBody":
            SDKLogger.warning("\033[93m下半身遥操模式下无法切换到行走模式\033[0m")
            return
        
        if self.toggle_torso_change_debug:
            SDKLogger.warning("\033[93m躯干模式开启中，无法切换到行走模式\033[0m")
            return
        try:
            banner_echo(f"切换到行走模式", 'green')
            # Create switch gait message
            gait_msg = switchGaitByName()
            gait_msg.gait_name = "walk"  # Assuming the message has a gait_name field
            
            # Publish the gait switch command once
            if not rospy.is_shutdown():
                self.pub_switch_gait.publish(gait_msg)
                SDKLogger.info("\033[92m行走模式切换命令已发布\033[0m")
        except Exception as e:
            SDKLogger.error(f"Error switching to walk mode: {e}")

    def _reset_to_stance_callback(self, key_combination: Set[str], joy:JoySticks)->None:
        """Callback for stance mode."""
        try:
            banner_echo(f"切换到站立模式", 'green')
            if not self.pico_info_transformer.control_mode == "UpperBody":
                # Create switch gait message
                gait_msg = switchGaitByName()
                gait_msg.gait_name = "walk"  # Assuming the message has a gait_name field
                
                # Publish the gait switch command once
                if not rospy.is_shutdown():
                    self.pub_switch_gait.publish(gait_msg)
                # 全身遥操/下半身遥操模式下 先踏步恢复步态然后再站立
                SDKLogger.info("下半身/全身遥操模式自动踏步调整站立")
                time.sleep(2.0) # 放宽踏步的时间尽量让机器人恢复站立
            # Create switch gait message
            gait_msg = switchGaitByName()
            gait_msg.gait_name = "stance"  # Assuming the message has a gait_name field
            
            # Publish the gait switch command once
            if not rospy.is_shutdown():
                self.pub_switch_gait.publish(gait_msg)
                SDKLogger.info("\033[92m站立模式切换命令已发布\033[0m")
        except Exception as e:
            SDKLogger.error(f"Error switching to stance mode: {e}")

    def _auto_swing_arm_callback(self, key_combination: Set[str], joy:JoySticks)->None:
        """Callback for arm reset."""
        # 下半身遥操模式下不响应
        if self.pico_info_transformer.control_mode == "LowerBody":
            SDKLogger.warning("\033[93m下半身遥操模式下无法切换手臂模式\033[0m")
            return
            
        if not self.toggle_teleop_unlock:
            SDKLogger.warning("\033[93m遥操未解锁，无法切换手臂模式\033[0m")
            return
        self.toggle_auto_swing_arm.toggle()

    def _stop_robot_callback(self, key_combination: Set[str], joy:JoySticks)->None:
        """Callback for stop robot."""
        #Squat down then publish stop robot command 3 times to ensure subscribers receive it.
        try:
            # Set squat height as a variable with default value
            squat_height = 0.05  # Default squat height in meters

            SDKLogger.info(f"\033[92m开始下蹲 {squat_height}m, 解锁机器人...\033[0m")
            squat_msg = Twist()
            squat_msg.linear.z = -squat_height  # Move down by squat_height
            
            # Publish squat command multiple times to ensure it's received
            for i in range(3):
                if not rospy.is_shutdown():
                    self.pub_cmd_pose.publish(squat_msg)
                    rospy.sleep(0.1)
                    SDKLogger.info(f"下蹲命令已发布 ({i+1}/3)")
            
            # Wait for squat motion to complete
            SDKLogger.info("等待下蹲完成...")
            rospy.sleep(1.0)  # Wait 1 second for squat motion
            
            # Then publish stop robot command
            stop_msg = Bool()
            stop_msg.data = True
            
            # Publish with small delay to ensure subscribers receive it
            for i in range(3):
                if not rospy.is_shutdown():
                    self.pub_stop_robot.publish(stop_msg)
                    rospy.sleep(0.1)  # 100ms delay between publications
                    SDKLogger.info(f"停止机器人命令已发布 ({i+1}/3)")
            SDKLogger.info("\033[92m机器人停止序列完成\033[0m")
        except Exception as e:
            SDKLogger.error(f"Error in robot stop sequence: {e}")

    def _pub_head_pose(self) -> None:
        def normalize_degree_in_180(angle: float) -> float:
            """Normalize an angle to the range [-180, 180] degrees."""
            normalized = angle % 360
            if normalized > 180:
                normalized -= 360
            elif normalized < -180:
                normalized += 360
            return normalized
        
        try:
            # Get transform from Pelvis to HEAD
            listener = self.bone_frame_publisher.get_listener()
            trans, rot = listener.lookupTransform("Pelvis", "HEAD", rospy.Time(0))
            
            # Convert quaternion to euler angles
            rpy = tf.transformations.euler_from_quaternion(rot)
            rpy_deg = [math.degrees(r) for r in rpy]
            
            # Extract and clamp pitch (X-axis) and yaw (Y-axis)
            pitch = max(-25, min(25, normalize_degree_in_180(round(rpy_deg[1], 2))))
            yaw = max(-80, min(80, normalize_degree_in_180(round(rpy_deg[2], 2))))
            
            # Publish head motion data
            msg = robotHeadMotionData()
            msg.joint_data = [yaw, pitch]
            self.bone_frame_publisher.head_motion_data_pub.publish(msg)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF lookup failed for Pelvis->HEAD transform: {e}")

    def _torso_change_callback(self, keys: Set[str], joy: JoySticks) -> None:
        self.toggle_torso_change_debug.toggle()

    def _whole_body_callback(self, keys: Set[str], joy: JoySticks) -> None:
        self.last_pico_running_state = False
        self.pico_info_transformer.set_control_mode("WholeBody")
        banner_echo("切换到全身遥操模式", 'green')

    def _upper_body_callback(self, keys: Set[str], joy: JoySticks) -> None:
        self.last_pico_running_state = False
        self.pico_info_transformer.set_control_mode("UpperBody")
        banner_echo("切换到上半身遥操模式", 'green')

    def _lower_body_callback(self, keys: Set[str], joy: JoySticks) -> None:
        self.last_pico_running_state = False
        self.pico_info_transformer.set_control_mode("LowerBody")
        banner_echo("切换到下半身遥操模式", 'green')
    """////////////////////////////////////////////////////////////////////////////////////////////////////"""

    def set_robot_model_params(self) -> str:
        """Set robot model parameters."""
        SDKLogger.info("Setting robot model parameters...")
        kuavo_assests_path = self._get_package_path("kuavo_assets")
        robot_version = os.environ.get('ROBOT_VERSION', '45')
        model_config_file = kuavo_assests_path + f"/config/kuavo_v{robot_version}/kuavo.json"
        
        with open(model_config_file, 'r') as f:
            model_config = json.load(f)
            
        rospy.set_param("/pico/upper_arm_length", model_config["upper_arm_length"])
        rospy.set_param("/pico/lower_arm_length", model_config["lower_arm_length"])
        rospy.set_param("/pico/shoulder_width", model_config["shoulder_width"])
        
        return kuavo_assests_path + f"/models/biped_s{robot_version}"

    def _get_package_path(self, package_name: str) -> Optional[str]:
        """Get the path of a ROS package."""
        try:
            rospack = rospkg.RosPack()
            return rospack.get_path(package_name)
        except rospkg.ResourceNotFound:
            SDKLogger.error(f"Package {package_name} not found")
            return None

    def set_ik_solver_params(self) -> None:
        """Set IK solver parameters."""
        SDKLogger.info("Setting IK solver parameters...")
        self.ik_solve_param.major_optimality_tol = 9e-3
        self.ik_solve_param.major_feasibility_tol = 9e-3
        self.ik_solve_param.minor_feasibility_tol = 9e-3
        self.ik_solve_param.major_iterations_limit = 50
        self.ik_solve_param.oritation_constraint_tol = 9e-3
        self.ik_solve_param.pos_constraint_tol = 9e-3
        self.ik_solve_param.pos_cost_weight = 10.0
        SDKLogger.info("IK solver parameters set successfully")

    def change_arm_ctrl_mode(self, mode: int) -> None:
        """Change arm control mode."""
        try:
            rospy.wait_for_service("/change_arm_ctrl_mode", timeout=3.0)
            changeHandTrackingMode_srv = rospy.ServiceProxy("/change_arm_ctrl_mode", changeArmCtrlMode)
            changeHandTrackingMode_srv(mode)
            # 服务调用成功后，发布模式切换话题
            self.pub_arm_ctrl_mode.publish(Int32(data=mode))
            SDKLogger.info(f"Arm control mode changed to {mode} and published to /pico/arm_ctrl_mode_change")
        except rospy.ROSException as e:
            SDKLogger.error(f"\033[31m服务 /change_arm_ctrl_mode 连接超时或连接失败, 请检查服务是否启动: {e}\033[0m")
        except Exception as e:
            SDKLogger.error(f"服务 /change_arm_ctrl_mode 调用失败: {e}")

    def change_mobile_ctrl_mode(self, mode: int) -> None:
        """Change mobile control mode."""
        try:
            rospy.wait_for_service("/mobile_manipulator_mpc_control", timeout=3.0)
            changeHandTrackingMode_srv = rospy.ServiceProxy("/mobile_manipulator_mpc_control", changeArmCtrlMode)
            changeHandTrackingMode_srv(mode)
            # 服务调用成功后，发布模式切换话题
            self.pub_mobile_ctrl_mode.publish(Int32(data=mode))
            SDKLogger.info(f"Mobile control mode changed to {mode} and published to /pico/mobile_ctrl_mode_change")
            # print(f"=================================changeHandTrackingMode_srv({mode})=============================================")
        except rospy.ROSException as e:
            SDKLogger.error(f"\033[31m服务 /mobile_manipulator_mpc_control 连接超时或连接失败, 请检查服务是否启动: {e}\033[0m")
        except Exception as e:
            SDKLogger.error(f"服务 /mobile_manipulator_mpc_control 调用失败: {e}")

    def change_mm_wbc_arm_ctrl_mode(self, mode: int) -> None:
        """Change MM WBC arm control mode."""
        try:
            rospy.wait_for_service("/enable_mm_wbc_arm_trajectory_control", timeout=3.0)
            changeHandTrackingMode_srv = rospy.ServiceProxy("/enable_mm_wbc_arm_trajectory_control", changeArmCtrlMode)
            changeHandTrackingMode_srv(mode)
            # 服务调用成功后，发布模式切换话题
            self.pub_mm_wbc_arm_ctrl_mode.publish(Int32(data=mode))
            SDKLogger.info(f"MM WBC arm control mode changed to {mode} and published to /pico/mm_wbc_arm_ctrl_mode_change")
        except rospy.ROSException as e:
            SDKLogger.error(f"\033[31m服务 /enable_mm_wbc_arm_trajectory_control 连接超时或连接失败, 请检查服务是否启动: {e}\033[0m")
        except Exception as e:
            SDKLogger.error(f"服务 /enable_mm_wbc_arm_trajectory_control 调用失败: {e}")

    def reset_mm_mpc(self):
        try:
            rospy.wait_for_service('/reset_mm_mpc', timeout=3.0)
            reset_mpc = rospy.ServiceProxy('/reset_mm_mpc', changeTorsoCtrlMode)
            req = changeTorsoCtrlModeRequest()
            res = reset_mpc(req)
            if res.result:
                rospy.loginfo("Mobile manipulator MPC reset successfully")
            else:
                rospy.logerr("Failed to reset mobile manipulator MPC")
        except rospy.ServiceException as e:
            rospy.logerr("Service call to %s failed: %s", '/reset_mm_mpc', e)
        except rospy.ROSException as e:
            rospy.logerr("Failed to connect to service %s: %s", '/reset_mm_mpc', e)
        except Exception as e:
            rospy.logerr("Failed to reset mobile manipulator MPC: %s", e)


    def robot_body_matrices_callback(self, robot_body_matrices_msg: robotBodyMatrices) -> None:
        """Callback for robot body matrices."""
        # 控制遥操解锁/锁定
        if not self.toggle_teleop_unlock:
            return
        
        # process data of hands
        robot_urdf_matrices, current_time = self.robot_matrix_publisher.get_all_matrices_from_message(robot_body_matrices_msg)
        self.pico_info_transformer.publish_local_poses(robot_urdf_matrices, current_time)

        # process data of foots
        self.pico_info_transformer.process_foot_poses_parallel(robot_urdf_matrices)

    def publish_hand_wrench_cmd(self, item_mass_force: dict) -> None:
        """Publish hand wrench command."""
        msg = Float64MultiArray()
        left_force = item_mass_force['lforce'] + [0,0,0]
        right_force = item_mass_force['rforce'] + [0,0,0]
        msg.data = left_force + right_force
        self.pub_hand_wrench_cmd.publish(msg)

    def publish_end_effector_control(self, joy: JoySticks, eef_freeze: bool = False) -> None:
        """Publish end effector control based on grip values.
        
        Args:
            joy: JoySticks message containing left_grip and right_grip values (0~1)
        """
        # 将 grip 值从 0~1 映射到 0~100
        eef_command = {
            'left': int(max(0, min(100, joy.left_grip * 100))),
            'right': int(max(0, min(100, joy.right_grip * 100))),
        }
        if self.last_eef_command is None:
            self.last_eef_command = eef_command
        # 如果eef_freeze为True，则使用上一次的末端执行器命令
        if not eef_freeze:
            self.last_eef_command = eef_command
        else:
            eef_command = self.last_eef_command
        try:  
            if self.eef_type.startswith("qiangnao"):
                # 创建末端执行器控制消息
                hand_control_msg = robotHandPosition()
                hand_control_msg.header.stamp = rospy.Time.now()
                hand_control_msg.header.frame_id = "pico_vr_control"
                hand_control_msg.left_hand_position = [eef_command['left']] * 6
                hand_control_msg.right_hand_position = [eef_command['right']] * 6
                
                # 发布末端执行器控制消息
                self.pub_control_robot_hand_position.publish(hand_control_msg)
            elif self.eef_type == "lejuclaw":
                msg = lejuClawCommand()
                msg.header.stamp = rospy.Time.now()
                msg.data.name = ['left_claw', 'right_claw']
                msg.data.position = [eef_command['left'], eef_command['right']]
                msg.data.velocity = [90, 90] # 速度默认 90%
                self.pub_leju_claw_cmd.publish(msg)
        except Exception as e:
            SDKLogger.error(f"Error publishing end effector control: {e}")

    def handle_joystick_movement(self, joy: JoySticks) -> None:
        """Handle all joystick movement controls for cmd_vel."""
        try:
            # 定义速度限制常量
            MAX_LINEAR_X_SPEED = 0.4    # 前后方向
            MAX_LINEAR_Y_SPEED = 0.2    # 左右横移
            MAX_ANGULAR_Z_SPEED = 0.4   # 旋转速度
            
            # 创建 Twist 消息
            cmd_vel_msg = Twist()
            
            # 左侧摇杆控制
            # joy.left_y 控制 linear.x (前后方向 -0.4~0.4)
            joy_y = apply_deadzone(joy.left_y, CmdVelConstants.LINEAR_DEADZONE)
            cmd_vel_msg.linear.x = clamp(joy_y * MAX_LINEAR_X_SPEED, -MAX_LINEAR_X_SPEED, MAX_LINEAR_X_SPEED)
            
            # joy.left_x 控制 linear.y (左右横移 -0.2~0.2)
            joy_x = apply_deadzone(joy.left_x, CmdVelConstants.LINEAR_DEADZONE)
            # PICO的X方向: https://developer-cn.picoxr.com/document/unreal/input-mappings/
            # cmd_vel 控制左右横移，左是正值，右是负值
            cmd_vel_msg.linear.y = clamp(-joy_x * MAX_LINEAR_Y_SPEED, -MAX_LINEAR_Y_SPEED, MAX_LINEAR_Y_SPEED)
            
            # 右侧摇杆控制
            # joy.right_x 控制 angular.z (旋转速度 -0.4~0.4)
            joy_right_x = apply_deadzone(joy.right_x, CmdVelConstants.LINEAR_DEADZONE)
            # PICO的X方向: https://developer-cn.picoxr.com/document/unreal/input-mappings/
            cmd_vel_msg.angular.z = clamp(-joy_right_x * MAX_ANGULAR_Z_SPEED, -MAX_ANGULAR_Z_SPEED, MAX_ANGULAR_Z_SPEED)
            
            # 发布 cmd_vel 消息
            self.pub_cmd_vel.publish(cmd_vel_msg)
            
        except Exception as e:
            SDKLogger.error(f"Error publishing cmd_vel: {e}")
        

    def pico_hands_poses_callback(self, pico_hands_poses_msg: picoPoseInfoList) -> None:
        """Callback for Pico hands poses."""
        self.pico_info_transformer.read_msg_hand(pico_hands_poses_msg)

        left_pose, left_elbow_pos = self.pico_info_transformer.get_hand_pose("Left")
        right_pose, right_elbow_pos = self.pico_info_transformer.get_hand_pose("Right")
        
        eef_pose_msg = twoArmHandPoseCmd()
        eef_pose_msg.hand_poses.left_pose.pos_xyz = left_pose[0]
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = left_pose[1]
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = left_elbow_pos

        eef_pose_msg.hand_poses.right_pose.pos_xyz = right_pose[0]
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = right_pose[1]
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = right_elbow_pos
        eef_pose_msg.ik_param = self.ik_solve_param
        # eef_pose_msg.use_custom_ik_param = self.use_custom_ik_param
        eef_pose_msg.use_custom_ik_param = False
        eef_pose_msg.frame = 1
        
        
        # Check for VR errors
        if self.pico_info_transformer.check_if_vr_error():
            SDKLogger.error("Detected VR ERROR!!! Please check the Pico device connection and status!")
            return
            
        if self.pico_info_transformer.is_running:
            self.pub.publish(eef_pose_msg)
        else:
            SDKLogger.warn("Not publishing to /mm/two_arm_hand_pose_cmd because is_running is False")
        
        # Handle service mode changes
        if self.send_srv and (self.last_pico_running_state != self.pico_info_transformer.is_running):
            SDKLogger.info(f"Pico running state change to: {self.pico_info_transformer.is_running}")
            mode = 2 if self.pico_info_transformer.is_running else 0
            if self.pico_info_transformer.control_torso_mode:
                mobile_mode = 3 if self.pico_info_transformer.is_running else 0
            else:
                mobile_mode = 1 if self.pico_info_transformer.is_running else 0
            wbc_mode = 1 if self.pico_info_transformer.is_running else 0
            self.change_arm_ctrl_mode(mode)
            self.change_mobile_ctrl_mode(mobile_mode)
            self.change_mm_wbc_arm_ctrl_mode(wbc_mode)
            self.last_pico_running_state = self.pico_info_transformer.is_running

    def pico_hands_poses_callback_ik(self, pico_hands_poses_msg: picoPoseInfoList) -> None:
        """Callback for Pico hands poses."""
        self.pico_info_transformer.read_msg_hand(pico_hands_poses_msg)

        left_pose, left_elbow_pos = self.pico_info_transformer.get_hand_pose("Left")
        right_pose, right_elbow_pos = self.pico_info_transformer.get_hand_pose("Right")
        
        eef_pose_msg = twoArmHandPoseCmd()
        eef_pose_msg.hand_poses.left_pose.pos_xyz = left_pose[0]
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = left_pose[1]
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = left_elbow_pos

        eef_pose_msg.hand_poses.right_pose.pos_xyz = right_pose[0]
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = right_pose[1]
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = right_elbow_pos
        eef_pose_msg.ik_param = self.ik_solve_param
        eef_pose_msg.use_custom_ik_param = self.use_custom_ik_param
        eef_pose_msg.frame = 3

        # Check for VR errors
        if self.pico_info_transformer.check_if_vr_error():
            SDKLogger.error("Detected VR ERROR!!! Please check the Pico device connection and status!")
            return
            
        if self.pico_info_transformer.is_running:
            if self.control_mode_changing:
                eef_pose_msg = self.interpolate_eef_pose(eef_pose_msg)
            self.pub_ik.publish(eef_pose_msg)
        else:
            SDKLogger.warn("Not publishing to /ik/two_arm_hand_pose_cmd because is_running is False")
            # Handle service mode changes
        if self.send_srv and (self.last_pico_running_state != self.pico_info_transformer.is_running):
            SDKLogger.info(f"Pico running state change to: {self.pico_info_transformer.is_running}")
            mode = 2 if self.pico_info_transformer.is_running else 0
            mobile_mode = 0 if self.pico_info_transformer.is_running else 0
            wbc_mode = 0 if self.pico_info_transformer.is_running else 0
            self.change_arm_ctrl_mode(mode)
            self.change_mobile_ctrl_mode(mobile_mode)
            self.change_mm_wbc_arm_ctrl_mode(wbc_mode)
            self.last_pico_running_state = self.pico_info_transformer.is_running
    def set_item_mass_force(self, case_name:str, hand_wrench_config:HandWrenchConfig)->None:
        """Set item mass force."""
        self.hand_wrench_config = hand_wrench_config
        SDKLogger.info(f"VR App端设置末端力配置: {case_name} --- {hand_wrench_config}")
        
    def set_control_mode_callback(self, control_mode: Int32) -> None:
        """Set control mode."""
        self._set_control_mode(control_mode.data)
    
    def _set_control_mode(self, control_mode: int) -> None:
        """Set control mode."""
        if control_mode == ControlMode.INCREMENTAL_MODE.value:
            self.set_control_mode("incremental")
            print("\033[93m--------------------进入增量模式-----------------\033[0m")
        elif control_mode == ControlMode.FOLLOW_MODE.value:
            self.set_control_mode("follow")
            print("\033[93m--------------------进入跟随模式-----------------\033[0m")
        elif control_mode == ControlMode.MOBILE_MPC_MODE.value:
            self.set_control_mode("mobile_mpc")
            print("\033[93m--------------------进入运动学MPC模式-----------------\033[0m")
        else:
            SDKLogger.error(f"Invalid control mode: {control_mode}")
    def set_incremental_mode_callback(self, incremental_mode: Int32) -> None:
        """Set incremental mode."""
        if incremental_mode.data == 1:
            self.set_in_incremental_mode(True)
        else:
            self.set_in_incremental_mode(False)

    def set_in_incremental_mode(self, incremental_mode: bool) -> None:
        """Set incremental mode."""
        self.is_in_incremental_mode = incremental_mode

    def set_control_mode(self, control_mode: str) -> None:
        """Set control mode."""
        if self.last_control_mode == ControlMode.INCREMENTAL_MODE and control_mode != "incremental":
            print("\033[93m--------------------退出增量模式-----------------\033[0m")
            self.change_mobile_ctrl_mode(IncrementalMpcCtrlMode.ArmOnly.value)
            self._left_target_pose = (None, None)
            self._right_target_pose = (None, None)  ## 清空上一次目标位姿
        self.last_control_mode = self.control_mode
        self.last_pico_running_state = not self.pico_info_transformer.is_running
        self.control_mode_changing = True
        if control_mode == "incremental":
            self.control_mode = ControlMode.INCREMENTAL_MODE
        elif control_mode == "follow":
            self.control_mode = ControlMode.FOLLOW_MODE
        elif control_mode == "mobile_mpc":
            self.control_mode = ControlMode.MOBILE_MPC_MODE
        else:
            SDKLogger.error(f"Invalid control mode: {control_mode}")

    def sensors_data_raw_callback(self, msg):
        if len(msg.joint_data.joint_q) >= 26:
            self.arm_joint_angles = msg.joint_data.joint_q[12:26]
    
    def ik_error_norm_callback(self, msg):
        """
        Callback for left hand error norm messages.
        """
        self.ik_error_norm = msg.data

    def fk_srv_client(self, joint_angles):
        service_name = "/ik/fk_srv"
        try:
            rospy.wait_for_service(service_name, timeout=3.0)
            fk_srv = rospy.ServiceProxy(service_name, fkSrv)
            fk_result = fk_srv(joint_angles)
            # rospy.loginfo(f"FK result: {fk_result.success}")
            return fk_result.hand_poses
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return None

    def pico_hands_poses_callback_incremental(self, pico_hands_poses_msg: picoPoseInfoList) -> None:
        """Callback for Pico hands poses."""
        self.pico_info_transformer.read_msg_hand(pico_hands_poses_msg)

        left_pose, left_elbow_pos = self.pico_info_transformer.get_hand_pose("Left")
        right_pose, right_elbow_pos = self.pico_info_transformer.get_hand_pose("Right")
        
        eef_pose_msg = twoArmHandPoseCmd()
        eef_pose_msg.hand_poses.left_pose.pos_xyz = left_pose[0]
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = left_pose[1]
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = left_elbow_pos

        eef_pose_msg.hand_poses.right_pose.pos_xyz = right_pose[0]
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = right_pose[1]
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = right_elbow_pos
        eef_pose_msg.ik_param = self.ik_solve_param
        # eef_pose_msg.use_custom_ik_param = self.use_custom_ik_param
        eef_pose_msg.use_custom_ik_param = False
        eef_pose_msg.frame = 3

        if self.is_in_incremental_mode:
            if self.last_control_mode != ControlMode.INCREMENTAL_MODE:
                print("\033[93m++++++++++++++++++++开始增量模式+++++++++++++++++\033[0m")
                # 刚开始切换到增量控制模式
                self.control_mode = ControlMode.INCREMENTAL_MODE
                
                
                self.change_mobile_ctrl_mode(IncrementalMpcCtrlMode.ArmOnly.value)
                # 设置当前VR的末端位姿为锚点
                self._left_anchor_pose = left_pose
                self._right_anchor_pose = right_pose
                
                # 只有在目标位姿为空时才重新获取FK，避免抖动
                def is_pose_empty(pose):
                    return pose[0] is None or pose[1] is None
                
                if is_pose_empty(self._left_target_pose) or is_pose_empty(self._right_target_pose):
                    self._left_target_pose = left_pose
                    self._right_target_pose = right_pose
                else:
                    print("********************************* 使用上次目标位姿 (避免抖动) *********************************")
                    print("\033[92m左手保持目标位置: {}\033[0m".format(self._left_target_pose[0]))
                    print("\033[92m左手保持目标姿态(四元数): {}\033[0m".format(self._left_target_pose[1]))
                    print("\033[92m右手保持目标位置: {}\033[0m".format(self._right_target_pose[0]))
                    print("\033[92m右手保持目标姿态(四元数): {}\033[0m".format(self._right_target_pose[1]))
                self.last_control_mode = ControlMode.INCREMENTAL_MODE
            # 计算位置增量
            l_xyz_delta = np.subtract(left_pose[0], self._left_anchor_pose[0])
            r_xyz_delta = np.subtract(right_pose[0], self._right_anchor_pose[0])
            
            # 添加位置变化阈值，减少噪声影响
            position_threshold = 0.001  # 1mm
            if np.linalg.norm(l_xyz_delta) < position_threshold:
                l_xyz_delta = np.zeros(3)
            if np.linalg.norm(r_xyz_delta) < position_threshold:
                r_xyz_delta = np.zeros(3)

            def quaternion_inverse(q):
                norm = np.sum(np.array(q)**2)
                return np.array([-q[0], -q[1], -q[2], q[3]]) / norm

            def quaternion_multiply(q1, q2):
                x1, y1, z1, w1 = q1
                x2, y2, z2, w2 = q2
                return np.array([
                    w1*x2 + x1*w2 + y1*z2 - z1*y2,
                    w1*y2 - x1*z2 + y1*w2 + z1*x2,
                    w1*z2 + x1*y2 - y1*x2 + z1*w2,
                    w1*w2 - x1*x2 - y1*y2 - z1*z2
                ])
            def normalize_quaternion(q):
                norm = np.linalg.norm(q)
                return q / norm if norm > 0 else q

            l_anchor_quat_inv = quaternion_inverse(self._left_anchor_pose[1])
            r_anchor_quat_inv = quaternion_inverse(self._right_anchor_pose[1])
            l_delta_quat = quaternion_multiply(left_pose[1], l_anchor_quat_inv)
            r_delta_quat = quaternion_multiply(right_pose[1], r_anchor_quat_inv)
            
            # 添加姿态变化阈值，减少噪声影响
            def quaternion_angle(q):
                # 计算四元数对应的旋转角度
                w = abs(q[3])  # 确保w为正
                if w > 1.0:
                    w = 1.0
                return 2.0 * np.arccos(w)
            
            orientation_threshold = 0.01  # 约0.57度
            if quaternion_angle(l_delta_quat) < orientation_threshold:
                l_delta_quat = np.array([0, 0, 0, 1])  # 单位四元数
            if quaternion_angle(r_delta_quat) < orientation_threshold:
                r_delta_quat = np.array([0, 0, 0, 1])  # 单位四元数
            
            # 只有在有显著变化时才更新锚点
            if np.linalg.norm(l_xyz_delta) > 0 or quaternion_angle(l_delta_quat) > 0:
                self._left_anchor_pose = left_pose
            if np.linalg.norm(r_xyz_delta) > 0 or quaternion_angle(r_delta_quat) > 0:
                self._right_anchor_pose = right_pose
                
            l_target_quat = quaternion_multiply(l_delta_quat, self._left_target_pose[1])
            r_target_quat = quaternion_multiply(r_delta_quat, self._right_target_pose[1])

            l_target_quat = normalize_quaternion(l_target_quat)
            r_target_quat = normalize_quaternion(r_target_quat)

            self._left_target_pose = (self._left_target_pose[0] + l_xyz_delta, l_target_quat)
            self._right_target_pose = (self._right_target_pose[0] + r_xyz_delta, r_target_quat)
            left_elbow_pos = left_elbow_pos
            right_elbow_pos = right_elbow_pos

            eef_pose_msg = twoArmHandPoseCmd()
            eef_pose_msg.hand_poses.left_pose.pos_xyz = self._left_target_pose[0]
            eef_pose_msg.hand_poses.left_pose.quat_xyzw = self._left_target_pose[1]
            eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = left_elbow_pos
            eef_pose_msg.hand_poses.right_pose.pos_xyz = self._right_target_pose[0]
            eef_pose_msg.hand_poses.right_pose.quat_xyzw = self._right_target_pose[1]
            eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = right_elbow_pos

        else:
            if self.last_control_mode == ControlMode.INCREMENTAL_MODE:
                print("\033[93m--------------------暂停增量模式-----------------\033[0m")
                self.change_mobile_ctrl_mode(IncrementalMpcCtrlMode.NoControl.value)
                # 注意：这里不清空目标位姿，保持状态以避免下次进入时的抖动
                print("\033[94m保持目标位姿以避免下次进入增量模式时的抖动\033[0m")
            # self.control_mode = ControlMode.NONE_MODE
            self.last_control_mode = ControlMode.NONE_MODE

        
        if eef_pose_msg is not None:
            eef_pose_msg.ik_param = self.ik_solve_param
            # eef_pose_msg.use_custom_ik_param = self.use_custom_ik_param
            eef_pose_msg.use_custom_ik_param = False
            eef_pose_msg.frame = 3

            if any(error > 75.0 for error in self.ik_error_norm):
                rospy.logwarn(f"Hand error norm too large: {self.ik_error_norm}, skipping arm trajectory publication")
            elif self.pico_info_transformer.is_running:
                if self.control_mode_changing:
                    eef_pose_msg = self.interpolate_eef_pose(eef_pose_msg)
                self.pub.publish(eef_pose_msg)
        
        # Handle service mode changes
        if self.send_srv and (self.last_pico_running_state != self.pico_info_transformer.is_running):
            SDKLogger.info(f"Pico running state change to: {self.pico_info_transformer.is_running}")
            mode = 2 if self.pico_info_transformer.is_running else 0
            mobile_mode = 1 if self.pico_info_transformer.is_running else 0
            wbc_mode = 1 if self.pico_info_transformer.is_running else 0
            self.change_arm_ctrl_mode(mode)
            self.change_mobile_ctrl_mode(mobile_mode)
            self.change_mm_wbc_arm_ctrl_mode(wbc_mode)
            self.last_pico_running_state = self.pico_info_transformer.is_running

    def pico_hands_poses_callback_choose(self, pico_hands_poses_msg: picoPoseInfoList) -> None:
        """Callback for Pico hands poses."""
        # 控制头部运动
        self._pub_head_pose()

        if self.toggle_auto_swing_arm:
            # LT+Y 手臂在默认位置不控制
            return 
        
        if self.control_mode == ControlMode.INCREMENTAL_MODE:
            self.pico_hands_poses_callback_incremental(pico_hands_poses_msg)
        elif self.control_mode == ControlMode.FOLLOW_MODE:
            self.change_mobile_ctrl_mode(IncrementalMpcCtrlMode.NoControl.value)
            self.pico_hands_poses_callback_ik(pico_hands_poses_msg)
        elif self.control_mode == ControlMode.MOBILE_MPC_MODE:
            self.pico_hands_poses_callback(pico_hands_poses_msg)
        else:
            SDKLogger.error(f"Invalid control mode: {self.control_mode}")

    def interpolate_eef_pose(self, eef_pose_msg: 'twoArmHandPoseCmd', max_dist_threshold: float = 0.3, interp_alpha: float = 0.1
) -> 'twoArmHandPoseCmd':
        """对双手末端位姿进行线性插值，返回插值后的消息。"""

        if self.arm_joint_angles is None:
            return None

        # 拷贝原始消息，避免副作用
        interp_msg = deepcopy(eef_pose_msg)

        # 目标位姿（VR输入）
        l_target_pos = np.array(eef_pose_msg.hand_poses.left_pose.pos_xyz)
        r_target_pos = np.array(eef_pose_msg.hand_poses.right_pose.pos_xyz)

        # 获取当前实际末端（通过FK服务）
        hand_poses = self.fk_srv_client(self.arm_joint_angles)
        if hand_poses is not None:
            l_current_pos = np.array(hand_poses.left_pose.pos_xyz)
            r_current_pos = np.array(hand_poses.right_pose.pos_xyz)

            l_dist = np.linalg.norm(l_target_pos - l_current_pos)
            r_dist = np.linalg.norm(r_target_pos - r_current_pos)

            if l_dist > max_dist_threshold or r_dist > max_dist_threshold:
                # 插值平移
                new_l_pos = (1 - interp_alpha) * l_current_pos + interp_alpha * (l_target_pos - l_current_pos)
                new_r_pos = (1 - interp_alpha) * r_current_pos + interp_alpha * (r_target_pos - r_current_pos)
            else:
                # 距离较小，不插值
                new_l_pos = l_target_pos
                new_r_pos = r_target_pos
                self.control_mode_changing = False
        else:
            # FK失败，直接使用目标
            new_l_pos = l_target_pos
            new_r_pos = r_target_pos

        # 填入新数据
        interp_msg.hand_poses.left_pose.pos_xyz = new_l_pos.tolist()
        interp_msg.hand_poses.right_pose.pos_xyz = new_r_pos.tolist()

        return interp_msg
class RobotMatrixPublisher:
    """Publisher for robot body matrices data recording."""
    
    def __init__(self, topic_name="/robot_body_matrices", queue_size=100):
        """
        Initialize the publisher.
        
        Args:
            topic_name (str): ROS topic name for publishing
            queue_size (int): Queue size for the publisher
        """
        self.publisher = rospy.Publisher(
            topic_name, 
            robotBodyMatrices, 
            queue_size=queue_size
        )
        
        # 身体部位名称列表 - 与BODY_TRACKER_ROLES对应
        self.body_parts = [
            "Pelvis", "LEFT_HIP", "RIGHT_HIP", "SPINE1", "LEFT_KNEE", 
            "RIGHT_KNEE", "SPINE2", "LEFT_ANKLE", "RIGHT_ANKLE", "SPINE3",
            "LEFT_FOOT", "RIGHT_FOOT", "NECK", "LEFT_COLLAR", "RIGHT_COLLAR",
            "HEAD", "LEFT_SHOULDER", "RIGHT_SHOULDER", "LEFT_ELBOW", 
            "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST", "LEFT_HAND", "RIGHT_HAND"
        ]
        
        self.data_source = "pico_vr_teleoperation"
        SDKLogger.info(f"RobotMatrixPublisher initialized on topic: {topic_name}")
    
    def publish_matrices(self, robot_urdf_matrices, current_time):
        """
        Publish robot body matrices data.
        
        Args:
            robot_urdf_matrices (np.ndarray): Shape [N, 4, 4] transformation matrices
            current_time (rospy.Time): Current ROS time
        """
        try:
            # 创建消息
            msg = robotBodyMatrices()
            
            # 设置header
            msg.header.stamp = current_time
            msg.header.frame_id = "world"
            
            # 设置时间戳
            msg.timestamp = current_time
            
            # 展平矩阵数据
            matrices_flat = []
            for matrix in robot_urdf_matrices:
                matrices_flat.extend(matrix.flatten())
            msg.matrices_data = matrices_flat
            
            # 设置矩阵数量
            msg.num_matrices = len(robot_urdf_matrices)
            
            # 设置身体部位名称
            msg.body_parts = self.body_parts[:msg.num_matrices]
            
            # 设置数据来源
            msg.data_source = self.data_source
            
            # 发布消息
            self.publisher.publish(msg)
            
        except Exception as e:
            SDKLogger.error(f"Error publishing robot matrices: {e}")
    
    def get_matrix_from_message(self, msg, matrix_index):
        """
        Extract a specific matrix from the message.
        
        Args:
            msg (RobotBodyMatrices): The message
            matrix_index (int): Index of the matrix to extract
            
        Returns:
            np.ndarray: 4x4 transformation matrix
        """
        if matrix_index >= msg.num_matrices:
            raise ValueError(f"Matrix index {matrix_index} out of range")
        
        start_idx = matrix_index * 16
        end_idx = start_idx + 16
        matrix_data = msg.matrices_data[start_idx:end_idx]
        timestamp = msg.timestamp
        
        return np.array(matrix_data).reshape(4, 4), timestamp
    
    def get_all_matrices_from_message(self, msg):
        """
        Extract all matrices from the message.
        
        Args:
            msg (RobotBodyMatrices): The message
            
        Returns:
            np.ndarray: Shape [N, 4, 4] transformation matrices
        """
        matrices = []
        for i in range(msg.num_matrices):
            matrix, timestamp = self.get_matrix_from_message(msg, i)
            matrices.append(matrix)
        
        return np.array(matrices), timestamp

def signal_handler(sig: int, frame: Any) -> None:
    """Handle signal for graceful exit."""
    print('Exiting gracefully...')
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    try:
        # Initialize the publisher
        publisher = KuavoPicoNode()
        
        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass