#!/usr/bin/env python3

import rospy
import copy
import json
import math
import re
import time
import threading
import numpy as np
import os
import sys
import rospkg
import subprocess
from geometry_msgs.msg import Twist
from humanoid_plan_arm_trajectory.srv import planArmTrajectoryBezierCurve, planArmTrajectoryBezierCurveRequest
    
# 使用 rospkg 获取 kuavo_common 包路径并导入 RobotVersion
try:
    kuavo_common_path = rospkg.RosPack().get_path('kuavo_common')
    kuavo_common_python_path = os.path.join(kuavo_common_path, 'python')
    if kuavo_common_python_path not in sys.path:
        sys.path.insert(0, kuavo_common_python_path)
    from robot_version import RobotVersion, is_tact_robot_type_compatible
except (rospkg.ResourceNotFound, ImportError) as e:
    # 如果 rospkg 不可用或包未找到，回退到相对路径方式
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    kuavo_common_python_path = os.path.abspath(os.path.join(current_file_dir, "../../../kuavo_common/python"))
    if kuavo_common_python_path not in sys.path:
        sys.path.insert(0, kuavo_common_python_path)
    from robot_version import RobotVersion, is_tact_robot_type_compatible
from humanoid_plan_arm_trajectory.msg import bezierCurveCubicPoint, jointBezierTrajectory
from kuavo_msgs.msg import ControllerSwitchEvent, robotHandPosition, robotHeadMotionData, sensorsData, robotWaistControl, gaitTimeName
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, getControllerList, switchController
from ocs2_msgs.msg import mpc_observation
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64MultiArray, String, Bool
from trajectory_msgs.msg import JointTrajectory
from humanoid_plan_arm_trajectory.msg import RobotActionState
from humanoid_plan_arm_trajectory.srv import ExecuteArmAction, ExecuteArmActionResponse  # Import new service type
from std_srvs.srv  import Trigger, TriggerResponse, SetBool, SetBoolResponse  # 中断服务依赖

# 根据机器人型号确定关节数据
KUAVO = "kuavo"
ROBAN = "roban"


def is_tact_playback_controller_allowed(control_scheme, controller_name):
    return get_tact_playback_controller_action(
        control_scheme, KUAVO, 5, controller_name
    ) == "allowed"


def get_tact_playback_controller_action(control_scheme, robot_class, robot_major, controller_name):
    if control_scheme != "multi":
        return "allowed"
    if robot_class == KUAVO and robot_major == 5 and controller_name == "amp_wild_controller":
        return "switch_to_amp_controller"
    return "allowed"


class ArmTrajectoryBezierDemo:
    # START_FRAME_TIME = 0
    END_FRAME_TIME = 10000
    KUAVO_TACT_LENGTH = 28
    ROBAN_TACT_LENGTH = 23

    def __init__(self):
        self.START_FRAME_TIME = 0
        self.x_shift = self.START_FRAME_TIME
        self.joint_state = JointState()
        self.hand_state = robotHandPosition()
        self.head_state = robotHeadMotionData()
        self.waist_state = robotWaistControl()
        self.running_action = False
        self.arm_flag = False
        self._timer = None
        self._timer_token = None
        # rospy service/timer callbacks run on different threads. Serialize lifecycle
        # transitions so interrupt/freeze cannot race action completion/reset.
        self._action_transition_lock = threading.RLock()
        self._execute_request_lock = threading.Lock()
        # 动作状态心跳和轨迹发布分别使用独立的停止事件。主动作切到复位动作时，
        # 必须先确认旧发布线程已经退出，避免 arm_flag 短暂 false 后又变 true
        # 导致两个 run() 线程同时发布同一条轨迹。
        self._action_state_thread = None
        self._action_state_stop_event = None
        self._action_state_lock = threading.Lock()
        self._action_state_publish_lock = threading.Lock()
        self._terminal_state_published = True
        self._trajectory_thread = None
        self._trajectory_stop_event = None
        self._controller_switch_abort_event = threading.Event()
        self._preparing_more_action = False
        self._active_action_controller = None
        self._pending_terminal_state = 2
        # 只清理本 session 真正获得过的共享控制权，避免一个在
        # preflight 阶段失败的请求误改其他控制器现有的手臂模式。
        self._action_owns_phase2 = False
        self._action_owns_external_arm_mode = False
        self.interrupt_flag  = False
        self.enable_control_state_ = True  # 软暂停状态，默认 enable=1
        self.last_published_state = None  # 记录上一次发布的状态，用于减少日志打印
        # 使用 RobotVersion 类创建版本号对象
        robot_version_int = int(os.environ.get("ROBOT_VERSION", "45"))
        self.robot_version = RobotVersion.create(robot_version_int) if RobotVersion.is_valid(robot_version_int) else RobotVersion(4, 5, 0)
        self.robot_class = KUAVO if self.robot_version.major() >= 4 else ROBAN
        self.kuavo_control_scheme = os.getenv("KUAVO_CONTROL_SCHEME", "multi")
        # KUAVO v50+ 有腰部关节
        self.has_waist = (self.robot_version.major() in (5,6)) if self.robot_class == KUAVO else False
        # KUAVO v60+ 为轮臂模型，joint_q 布局与双足版本不同
        self.is_wheeled = (self.robot_version.major() == 6) if self.robot_class == KUAVO else False
       
        if self.robot_class == KUAVO:
            # 根据是否有腰部关节确定TACT长度
            tact_length = self.KUAVO_TACT_LENGTH + (1 if self.has_waist else 0)
            current_control_mode = self.get_current_control_mode()
            if current_control_mode == "rl":
                self.INIT_ARM_POS = [int(0)] * tact_length
            else:
                # 基础28个关节 + 可选的1个腰部关节
                # 默认值（如果 ROS 参数不存在时使用）
                default_base_init = [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                
                # 从 ROS 参数读取 standJointState（手臂关节，14个）
                try:
                    stand_joint_state = None
                    while stand_joint_state is None and not rospy.is_shutdown():
                        stand_joint_state = rospy.get_param("/standJointState", None)
                        if stand_joint_state is None:
                            rospy.logwarn("ROS 参数 /standJointState 不存在，每隔 1s 检查一次，启动机器人时会自动加载")
                            rospy.sleep(1.0)
                    if stand_joint_state is not None and len(stand_joint_state) >= 14:
                        # 将弧度转换为度，并只取前14个手臂关节
                        self.arm_joints_deg = [math.degrees(rad) for rad in stand_joint_state[:14]]
                        # 构建完整的 base_init：前14个手臂关节 + 后14个默认值（手部、头部等）
                        base_init = self.arm_joints_deg + default_base_init[14:]
                        rospy.loginfo("从 ROS 参数 /standJointState 读取手臂初始位置: %s", self.arm_joints_deg)
                    else:
                        self.arm_joints_deg = default_base_init[:14]
                        base_init = default_base_init
                        rospy.logwarn("ROS 参数 /standJointState 不存在或数据不足，使用默认值")
                except Exception as e:
                    self.arm_joints_deg = default_base_init[:14]
                    base_init = default_base_init
                    rospy.logwarn("读取 ROS 参数 /standJointState 失败: %s，使用默认值", str(e))
                
                if self.has_waist:
                    self.INIT_ARM_POS = base_init + [0]  # 添加腰部初始位置
                else:
                    self.INIT_ARM_POS = base_init
            self.current_arm_joint_state = [0] * tact_length
        elif self.robot_class == ROBAN:
            self.INIT_ARM_POS = [22.91831, 10, 0, -45.83662, 22.91831, -10, 0, -45.83662, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]# task.info: shoudler_center: 0.4rad, elbow_center: -0.8rad
            self.current_arm_joint_state = [0] * self.ROBAN_TACT_LENGTH

        # rospy.spin()

        # Initialize ROS node
        rospy.init_node('autostart_arm_trajectory_bezier_demo')
        self.arm_restore_flag = rospy.get_param('~arm_restore_flag', True)
        self.keep_arm_pose = False
        
        # 从 RL/MPC 配置文件分别读取肩部 roll 默认偏移（启动时加载一次）
        self._rl_shoulder_roll, self._mpc_shoulder_roll = self._load_shoulder_roll_offsets()
        
        # 检查是否是半身模式
        self.only_half_up_body = rospy.get_param('/only_half_up_body', False)
        if self.only_half_up_body:
            rospy.loginfo("检测到半身模式（only_half_up_body=True）")
        else:
            rospy.loginfo("全身体模式（only_half_up_body=False）")

        # Subscribers and Publishers
        rospy.loginfo(
            "***************************arm_trajectory_bezier_process_start*****************************************")
        self.traj_sub = rospy.Subscriber('/bezier/arm_traj', JointTrajectory, self.traj_callback, queue_size=1,
                                         tcp_nodelay=True)
        self.kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=1, tcp_nodelay=True)
        # MoRE 离线动作使用独立输入；ArmController 在动作 session 内只接收
        # 该话题，Quest/SDK 继续发布的 /kuavo_arm_traj 不会覆盖动作目标。
        # 其他控制器仍只走已有的 /kuavo_arm_traj，不做双发布。
        self.kuavo_action_traj_pub = rospy.Publisher(
            '/kuavo_action_traj', JointState, queue_size=1, tcp_nodelay=True
        )
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1, tcp_nodelay=True)
        self.gait_name_pub = rospy.Publisher('/humanoid_mpc_gait_name_request', String, queue_size=1, tcp_nodelay=True)
        self.control_hand_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=1,
                                                tcp_nodelay=True)
        self.control_head_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=1,
                                                tcp_nodelay=True)
        self.control_waist_pub = rospy.Publisher('/robot_waist_motion_data', robotWaistControl, queue_size=1, 
                                                tcp_nodelay=True)
        # 与手臂相同，MoRE 动作腰部采用专用输入，避免 Quest torso 与 tact
        # 同时写 /robot_waist_motion_data 时最后到达者覆盖。
        self.action_waist_pub = rospy.Publisher(
            '/robot_action_waist_motion_data', robotWaistControl,
            queue_size=1, tcp_nodelay=True
        )

        self.sensor_data_sub = rospy.Subscriber('/sensors_data_raw', 
                                                sensorsData,
                                                self.sensors_data_raw_callback,   
                                                queue_size=1, 
                                                tcp_nodelay=True)

        self.robot_hand_sub = rospy.Subscriber('/dexhand/state', 
                                                JointState,
                                                self.robot_hand_callback,   
                                                queue_size=1, 
                                                tcp_nodelay=True)

        # 缓存两类手臂输入，供 create_action_data 选择正确 owner 的起始帧。
        # MoRE 新 session 的第一段轨迹从实测关节开始；进入 reset 阶段后，
        # 再使用本 session 的 action 末帧，不能误用 Quest 持续发布的 generic 目标。
        self._last_kuavo_arm_traj_msg = None
        self._last_kuavo_action_traj_msg = None
        # Full tact-space command (arm + hands + head + optional waist) last
        # produced for the active MoRE action.  /kuavo_action_traj carries only
        # the 14 arm joints, so it cannot by itself provide a safe reset start.
        self._last_more_published_servos_deg = None
        self._more_bezier_accept_after = rospy.Time(0)
        self._more_output_snapshot = None
        self._more_output_lock = threading.Lock()
        self.kuavo_arm_traj_sub = rospy.Subscriber(
            '/kuavo_arm_traj', JointState, self._kuavo_arm_traj_callback, queue_size=1, tcp_nodelay=True
        )
        self.kuavo_action_traj_sub = rospy.Subscriber(
            '/kuavo_action_traj', JointState, self._kuavo_action_traj_callback,
            queue_size=1, tcp_nodelay=True
        )

        # ===================================================================
        # [步态感知] 订阅步态切换通知，MPC/AMP 步态为 stance/walk/trot 时
        # 自动将 linker_hand 的拇指内扣（position[0]=100）。
        # 仅 linker_hand 末端生效。
        # ===================================================================
        self._last_gait_name = None
        self._gait_sub = rospy.Subscriber(
            "/humanoid_mpc_gait_name_request", String,
            self._gait_changed_callback, queue_size=10
        )

        # ===================================================================
        # [行走状态] 合并 MPC + AMP 两路信号，统一发布 /robot_walking_status (Bool)
        #   - /humanoid_mpc_gait_time_name (gaitTimeName): gait_name != "stance" 即行走
        #   - /rl_controller/InputData/command (Float64MultiArray): data[3] < 0.5 即行走
        # 任一信号为"行走"且消息在 0.5s 内 → True（兜底避免控制器切换后陈旧值）
        # ===================================================================
        self._mpc_is_walking = False
        self._rl_is_walking = False
        self._mpc_last_recv = None
        self._rl_last_recv = None
        self._walking_status_freshness = rospy.Duration(0.5)
        self._last_walking_status_pub = None
        self.walking_status_pub = rospy.Publisher(
            '/robot_walking_status', Bool, queue_size=1, tcp_nodelay=True
        )
        self._mpc_gait_time_name_sub = rospy.Subscriber(
            '/humanoid_mpc_gait_time_name', gaitTimeName,
            self._mpc_gait_time_name_callback, queue_size=10
        )
        self._rl_command_sub = rospy.Subscriber(
            '/rl_controller/InputData/command', Float64MultiArray,
            self._rl_command_callback, queue_size=10
        )

        # 软暂停：立即停止 /kuavo_arm_traj /control_robot_hand_position /robot_head_motion_data 发布
        self._enable_control_sub = rospy.Subscriber(
            '/enable_control_state', Bool, self._enable_control_callback, queue_size=1
        )
        self._controller_switch_event_sub = rospy.Subscriber(
            '/humanoid_controller/controller_switch_event',
            ControllerSwitchEvent,
            self._controller_switch_event_callback,
            queue_size=1,
        )
        self._more_action_abort_sub = rospy.Subscriber(
            '/humanoid_controller/more_arm_action_abort',
            Empty,
            self._more_action_abort_callback,
            queue_size=1,
        )

        # 添加发布者
        # 保留终态和紧随其后的新 session 心跳，避免 queue=1 在快速
        # 重入时用新的 state=1 覆盖尚未发出的上一次 terminal。
        self.robot_action_state_pub = rospy.Publisher('/robot_action_state', RobotActionState, queue_size=10)

        # Add service to execute arm actions
        self.execute_service = rospy.Service('/execute_arm_action', ExecuteArmAction, self.handle_execute_action)
        self._interrupt_service = rospy.Service('/interrupt_arm_traj', Trigger, self.handle_interrupt  )
        # 冻结：立即停止发布 /kuavo_arm_traj 且不复位（用于手柄 LB+B 将 tact 定住在当前帧）
        self._freeze_service = rospy.Service(
            '/humanoid_plan_arm_trajectory/freeze_arm_traj', Trigger, self.handle_freeze_arm_traj
        )
        self._keep_arm_pose_service = rospy.Service(
            '/humanoid_plan_arm_trajectory/keep_arm_pose', SetBool, self.handle_keep_arm_pose
        )

        # Store the file path base directory for actions
        # self.action_files_path = "/home/lab/kuavo-ros-control/src/humanoid-control/humanoid_plan_arm_trajectory/script/action_files"
        self.action_files_path = "/home/lab/.config/lejuconfig/action_files"
        rospy.loginfo("arm_trajectory_bezier_process is ready.")
        rospy.loginfo(
            "***************************arm_trajectory_bezier_process_end*****************************************")

        # self.run()
        rospy.spin()

    def _load_shoulder_roll_offsets(self):
        """分别从 RL 和 MPC 配置文件读取各自的肩部 roll 偏移值。
        
        RL:  skw_rl_param.info → defaultJointState
             索引 14(zarm_l2_joint) → l_arm_roll, 索引 21(zarm_r2_joint) → r_arm_roll
        
        MPC: task.info → swing_trajectory_config.swing_shoulder_roll_center
             单一值，左臂 +center，右臂 -center
        
        :return: (rl_offsets, mpc_offsets) 各为 (left_rad, right_rad)
        """
        rl_offsets  = (0.0, 0.0)
        mpc_offsets = (0.0, 0.0)

        humanoid_controllers_path = None
        try:
            humanoid_controllers_path = rospkg.RosPack().get_path('humanoid_controllers')
        except Exception:
            rospy.logwarn("[ShoulderRollOffset] 无法获取 humanoid_controllers 包路径")
            return (rl_offsets, mpc_offsets)

        robot_version_str = os.environ.get("ROBOT_VERSION", "45")

        rl_path = os.path.join(
            humanoid_controllers_path, "config",
            f"kuavo_v{robot_version_str}", "rl", "skw_rl_param.info"
        )
        if os.path.exists(rl_path):
            try:
                with open(rl_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                match = re.search(r'defaultJointState\s*\{([^}]+)\}', content, re.DOTALL)
                if match:
                    joint_state = {}
                    for m in re.finditer(r'\((\d+),\d+\)\s+([-0-9.]+)', match.group(1)):
                        joint_state[int(m.group(1))] = float(m.group(2))
                    rl_left  = joint_state.get(14, 0.0)
                    rl_right = joint_state.get(21, 0.0)
                    rl_offsets = (rl_left, rl_right)
                    rospy.loginfo("[ShoulderRollOffset] RL: l=%.4f r=%.4f rad", rl_left, rl_right)
            except Exception as e:
                rospy.logwarn("[ShoulderRollOffset] 读取 RL 配置失败: %s", e)
        else:
            rospy.loginfo("[ShoulderRollOffset] RL 配置文件不存在: %s", rl_path)

        mpc_path = os.path.join(
            humanoid_controllers_path, "config",
            f"kuavo_v{robot_version_str}", "mpc", "task.info"
        )
        if os.path.exists(mpc_path):
            try:
                with open(mpc_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                match = re.search(r'swing_shoulder_roll_center\s+([-0-9.]+)', content)
                if match:
                    center = float(match.group(1))
                    mpc_offsets = (center, -center)
                    rospy.loginfo("[ShoulderRollOffset] MPC: center=%.4f -> l=%.4f r=%.4f rad",
                                  center, center, -center)
            except Exception as e:
                rospy.logwarn("[ShoulderRollOffset] 读取 MPC 配置失败: %s", e)
        else:
            rospy.loginfo("[ShoulderRollOffset] MPC 配置文件不存在: %s", mpc_path)

        return (rl_offsets, mpc_offsets)


    def sensors_data_raw_callback(self, msg):
        """更新关节数据"""
        self._last_joint_msg = msg

        if not hasattr(self, "_last_hand_msg"):
            dummy_hand = JointState()
            dummy_hand.position = [0.0] * 12
            self._last_hand_msg = dummy_hand

        self._update_current_arm_joint_state(self._last_joint_msg, self._last_hand_msg)

    def robot_hand_callback(self, msg):
        """更新手部数据"""
        left = msg.position[:6] if len(msg.position) >= 6 else [0] * 6
        right = msg.position[6:12] if len(msg.position) >= 12 else [0] * 6

        self._last_hand_msg = msg

        if hasattr(self, "_last_joint_msg"):
            self._update_current_arm_joint_state(self._last_joint_msg, self._last_hand_msg)

    # --- 各机型 joint_q 提取函数 ---

    def _extract_kuavo_wheeled(self, joint_q, hand_part):
        """轮臂模型 (v60+): [knee, leg, waist_pitch, waist_yaw, arm_l*7, arm_r*7, head*2]"""
        return list(joint_q[4:18]) + hand_part + list(joint_q[-2:]) + [joint_q[3]]

    def _extract_kuavo_biped(self, joint_q, hand_part):
        """双足模型 (v4x): [leg*12, arm*14, head*2]"""
        return list(joint_q[12:26]) + hand_part + list(joint_q[-2:])

    def _extract_kuavo_biped_waist(self, joint_q, hand_part):
        """双足模型+腰部 (v50): [leg*12, waist, arm*14, head*2]"""
        return list(joint_q[13:27]) + hand_part + list(joint_q[-2:]) + [joint_q[12]]

    def _extract_roban(self, joint_q, hand_part):
        """ROBAN: [hand*12, waist, arm*8, head*2]"""
        return list(joint_q[13:21]) + hand_part + list(joint_q[21:23]) + [joint_q[12]]

    # 函数注册表：机型 key -> 提取函数
    _JOINT_EXTRACTORS = {}

    def _get_extractor_key(self):
        """根据当前机器人配置，返回对应的提取函数 key"""
        if self.robot_class == ROBAN:
            return "ROBAN"
        if self.is_wheeled:
            return "KUAVO_WHEELED"
        if self.has_waist:
            return "KUAVO_BIPED_WAIST"
        if self.robot_class == KUAVO:
            return "KUAVO_BIPED"
        raise RuntimeError(
            f"Unknown robot config: class={self.robot_class}, is_wheeled={self.is_wheeled}, "
            f"has_waist={self.has_waist}, version={self.robot_version.version_name()}"
        )

    @staticmethod
    def _extract_hand_part(hand_msg):
        """从手部话题提取手部关节数据，不足12个补零"""
        return list(hand_msg.position[:12]) if len(hand_msg.position) >= 12 else [0.0] * 12

    def _update_current_arm_joint_state(self, joint_msg, hand_msg):
        """整合 joint_msg 和 hand_msg，更新 current_arm_joint_state"""
        joint_q = joint_msg.joint_data.joint_q
        hand_part = self._extract_hand_part(hand_msg)

        extractor = self._JOINT_EXTRACTORS[self._get_extractor_key()]
        self.current_arm_joint_state = [round(v, 5) for v in extractor(self, joint_q, hand_part)]

    def _kuavo_arm_traj_callback(self, msg):
        """缓存 /kuavo_arm_traj 最新消息，供 create_action_data 使用"""
        self._last_kuavo_arm_traj_msg = msg

    def _kuavo_action_traj_callback(self, msg):
        """缓存当前 MoRE 离线动作的最新手臂目标，供 reset 起始帧使用。"""
        self._last_kuavo_action_traj_msg = msg

    def _begin_more_bezier_phase(self):
        """Discard prior-phase planner output and establish a timestamp fence."""
        with self._more_output_lock:
            self._more_output_snapshot = None
            self._more_bezier_accept_after = rospy.Time.now()

    def _gait_changed_callback(self, msg):
        """
        [步态切换回调] 当 MPC/AMP 步态变更时被调用。

        仅对 linker_hand 末端生效：步态为 stance / walk / trot 时内扣拇指
        （position[0]=100）。

        Args:
            msg (std_msgs/String): msg.data 为步态名
        """
        if rospy.get_param('/end_effector_type', '') != 'linker_hand':
            return

        new_gait = msg.data
        if new_gait == self._last_gait_name:
            return

        prev_gait = self._last_gait_name
        self._last_gait_name = new_gait
        rospy.loginfo("[GaitThumb] gait switch: %s → %s", prev_gait, new_gait)

        if new_gait in ("stance", "walk", "trot"):
            self.hand_state.left_hand_position  = [100, 0, 0, 0, 0, 0]
            self.hand_state.right_hand_position = [100, 0, 0, 0, 0, 0]
            self.control_hand_pub.publish(self.hand_state)
            rospy.loginfo("[GaitThumb] thumb retracted → left[0]=100 right[0]=100")

    def _mpc_gait_time_name_callback(self, msg):
        self._mpc_is_walking = (msg.gait_name != "stance")
        self._mpc_last_recv = rospy.Time.now()
        self._publish_walking_status()

    def _rl_command_callback(self, msg):
        if len(msg.data) < 4:
            return
        self._rl_is_walking = (msg.data[3] == 0.0)
        self._rl_last_recv = rospy.Time.now()
        self._publish_walking_status()

    def _publish_walking_status(self):
        walking = self.is_robot_walking()
        if walking != self._last_walking_status_pub:
            self.walking_status_pub.publish(Bool(data=walking))
            self._last_walking_status_pub = walking

    def is_robot_walking(self):
        now = rospy.Time.now()
        mpc_fresh = self._mpc_last_recv is not None and \
            (now - self._mpc_last_recv) < self._walking_status_freshness
        rl_fresh = self._rl_last_recv is not None and \
            (now - self._rl_last_recv) < self._walking_status_freshness
        return (mpc_fresh and self._mpc_is_walking) or (rl_fresh and self._rl_is_walking)

    def request_stance(self):
        self.cmd_vel_pub.publish(Twist())
        self.gait_name_pub.publish(String(data="stance"))

    def wait_for_stance_before_action(self, timeout=3.0):
        if not self.is_robot_walking():
            return True

        rospy.loginfo("机器人正在行走，先请求停止后再播放上肢动作")
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout and not rospy.is_shutdown():
            self.request_stance()
            if not self.is_robot_walking():
                rospy.loginfo("机器人已停止行走，继续播放上肢动作")
                return True
            rospy.sleep(0.05)

        return not self.is_robot_walking()

    def _get_servos_from_kuavo_arm_traj(self, tact_length):
        """从当前 session 的有效输入获取起始关节角（度）。

        MoRE 首段动作开始前 action cache 会被清空，因此使用实测关节；
        reset 阶段则使用当前 action 末帧。其他控制器保持原 generic 逻辑。
        """
        is_more_action = self._active_action_controller == "more_controller"
        if is_more_action:
            full_action_target = self._last_more_published_servos_deg
            if full_action_target is not None and len(full_action_target) >= tact_length:
                return list(full_action_target[:tact_length])
            msg = getattr(self, '_last_kuavo_action_traj_msg', None)
        else:
            msg = getattr(self, '_last_kuavo_arm_traj_msg', None)

        # Start from the measured complete tact state.  Arm trajectory topics
        # contain only 14 values; padding the remaining hand/head/waist values
        # with zero made reset jump the waist to zero before interpolation.
        current = [math.degrees(x) for x in self.current_arm_joint_state[:tact_length]]
        if len(current) < tact_length:
            current.extend([0.0] * (tact_length - len(current)))

        if msg is None or not getattr(msg, 'position', None):
            return current

        pos = list(msg.position)
        n_from_topic = min(len(pos), tact_length)
        current[:n_from_topic] = pos[:n_from_topic]
        return current

    def traj_callback(self, msg):
        if len(msg.points) == 0:
            return
        point = msg.points[0]

        if self.robot_class == KUAVO:
            self.joint_state.name = [
                "l_arm_pitch",
                "l_arm_roll",
                "l_arm_yaw",
                "l_forearm_pitch",
                "l_hand_yaw",
                "l_hand_pitch",
                "l_hand_roll",
                "r_arm_pitch",
                "r_arm_roll",
                "r_arm_yaw",
                "r_forearm_pitch",
                "r_hand_yaw",
                "r_hand_pitch",
                "r_hand_roll",
            ]
            self.joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
            self.joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
            self.joint_state.effort = [0] * 14

            self.hand_state.left_hand_position = [max(0, int(math.degrees(pos))) for pos in point.positions[14:20]]  # 无符号整数
            self.hand_state.right_hand_position = [max(0, int(math.degrees(pos))) for pos in point.positions[20:26]]  # 无符号整数
            
            self.head_state.joint_data = [math.degrees(pos) for pos in point.positions[26:28]]
            if self.has_waist and len(point.positions) > 28:
                # KUAVO v50+: 腰部关节在joint_q[12]位置
                self.waist_state.header.stamp = rospy.Time.now()
                self.waist_state.data.data = [math.degrees(pos) for pos in point.positions[28:29]]

            if (self._active_action_controller == "more_controller" and
                    msg.header.stamp >= self._more_bezier_accept_after):
                tact_length = self.KUAVO_TACT_LENGTH + (1 if self.has_waist else 0)
                if len(point.positions) >= tact_length:
                    # Store a new immutable-by-convention list in one
                    # assignment, so reset observes a complete frame rather
                    # than combining independently delivered ROS topics.
                    full_target = [math.degrees(pos) for pos in point.positions[:tact_length]]
                    full_target[14:20] = [
                        max(0, int(math.degrees(pos))) for pos in point.positions[14:20]
                    ]
                    full_target[20:26] = [
                        max(0, int(math.degrees(pos))) for pos in point.positions[20:26]
                    ]
                    output_snapshot = (
                        copy.deepcopy(self.joint_state),
                        copy.deepcopy(self.hand_state),
                        copy.deepcopy(self.head_state),
                        copy.deepcopy(self.waist_state),
                        full_target,
                    )
                    with self._more_output_lock:
                        # Recheck the fence after constructing the snapshot: a
                        # reset phase may have started concurrently.
                        if msg.header.stamp >= self._more_bezier_accept_after:
                            self._more_output_snapshot = output_snapshot
            
        elif self.robot_class == ROBAN:
            self.joint_state.name = [
                "l_arm_pitch",
                "l_arm_roll",
                "l_arm_yaw",
                "l_forearm_pitch",
                "r_arm_pitch",
                "r_arm_roll",
                "r_arm_yaw",
                "r_forearm_pitch",
            ]
            self.joint_state.position = [math.degrees(pos) for pos in point.positions[:8]]
            self.joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:8]]
            self.joint_state.effort = [0] * 8

            if len(point.positions) == self.ROBAN_TACT_LENGTH:

                self.hand_state.left_hand_position = [max(0, int(math.degrees(pos))) for pos in point.positions[8:14]]  # 无符号整数
                self.hand_state.right_hand_position = [max(0, int(math.degrees(pos))) for pos in
                                                    point.positions[14:20]]  # 无符号整数
                
                self.head_state.joint_data = [math.degrees(pos) for pos in point.positions[20:22]]

                self.waist_state.header.stamp = rospy.Time.now()
                self.waist_state.data.data = [math.degrees(pos) for pos in point.positions[22:]]

                if (self._active_action_controller == "more_controller" and
                        msg.header.stamp >= self._more_bezier_accept_after):
                    full_target = [math.degrees(pos) for pos in point.positions]
                    full_target[8:14] = [
                        max(0, int(math.degrees(pos))) for pos in point.positions[8:14]
                    ]
                    full_target[14:20] = [
                        max(0, int(math.degrees(pos))) for pos in point.positions[14:20]
                    ]
                    output_snapshot = (
                        copy.deepcopy(self.joint_state),
                        copy.deepcopy(self.hand_state),
                        copy.deepcopy(self.head_state),
                        copy.deepcopy(self.waist_state),
                        full_target,
                    )
                    with self._more_output_lock:
                        if msg.header.stamp >= self._more_bezier_accept_after:
                            self._more_output_snapshot = output_snapshot

    def call_change_arm_ctrl_mode_service(self, arm_ctrl_mode):
        result = True
        service_name = "/wheel_arm_change_arm_ctrl_mode" if self.is_wheeled else "arm_traj_change_mode"
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            change_arm_ctrl_mode = rospy.ServiceProxy(
                service_name, changeArmCtrlMode
            )
            resp = change_arm_ctrl_mode(control_mode=arm_ctrl_mode)
            result = bool(resp.result)
            if result:
                rospy.loginfo("Service call successful")
            else:
                rospy.logwarn(f"Service {service_name} rejected mode {arm_ctrl_mode}: {resp.message}")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)
            result = False
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")
            result = False
        finally:
            return result

    def call_enable_wbc_arm_trajectory_control_service(self, enable):
        """使能/禁用 WBC 手臂轨迹控制（走 /kuavo_arm_traj 滤波路径，与 VR 同源）。
        Roban / 轮臂跳过：走 MPC 简化自由度路径，避免锁定解锁后再进抽臂 (#3624)。"""
        if self.robot_class == ROBAN or self.is_wheeled:
            return
        service_name = "/enable_wbc_arm_trajectory_control"
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            client = rospy.ServiceProxy(service_name, changeArmCtrlMode)
            response = client(control_mode=enable)
            if response.result:
                rospy.loginfo(f"{service_name} call successful, enable={enable}")
                return True
            rospy.logwarn(
                "%s rejected enable=%s: %s",
                service_name,
                enable,
                response.message,
            )
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.loginfo(f"{service_name} not available, skipping")
            rospy.logdebug("%s call failed: %s", service_name, exc)
        return False

    def get_arm_ctrl_mode(self):
        """获取当前手臂控制模式"""
        service_name = "humanoid_get_arm_ctrl_mode"
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            get_arm_ctrl_mode = rospy.ServiceProxy(service_name, changeArmCtrlMode)
            req = changeArmCtrlModeRequest()
            req.control_mode = 0  # 查询模式时此参数不使用
            resp = get_arm_ctrl_mode(req)
            return resp.mode
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to get arm control mode: {e}")
            return -1
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")
            return -1

    def wait_for_arm_mode_change_complete(self, target_mode, timeout=2.0):
        """等待手臂控制模式切换完成"""
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            current_mode = self.get_arm_ctrl_mode()
            if current_mode == target_mode:
                rospy.loginfo(f"Arm control mode changed to {target_mode} successfully")
                return True
            # 如果获取模式失败（返回-1），继续等待
            if current_mode == -1:
                rospy.sleep(0.01)  # 10ms 检查间隔
                continue
            rospy.sleep(0.01)  # 10ms 检查间隔
        
        final_mode = self.get_arm_ctrl_mode()
        rospy.logwarn(f"Arm control mode change timeout after {timeout} seconds, current mode: {final_mode}, target: {target_mode}")
        return False

    def ensure_arm_ctrl_mode(self, target_mode, timeout=5.0):
        """Request arm trajectory mode and wait until the controller reports it."""
        start_time = rospy.Time.now()
        last_request_time = rospy.Time(0)
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            current_mode = self.get_arm_ctrl_mode()
            if current_mode == target_mode:
                rospy.loginfo(f"Arm control mode changed to {target_mode} successfully")
                return True

            now = rospy.Time.now()
            if (not last_request_time.is_zero() and
                    (now - last_request_time).to_sec() < 0.2):
                rospy.sleep(0.02)
                continue

            mode_request_accepted = self.call_change_arm_ctrl_mode_service(target_mode)
            if mode_request_accepted and target_mode == 2:
                self._action_owns_external_arm_mode = True
            if not mode_request_accepted:
                rospy.sleep(0.05)
            last_request_time = now

        final_mode = self.get_arm_ctrl_mode()
        if final_mode == target_mode:
            rospy.loginfo(f"Arm control mode changed to {target_mode} successfully")
            return True
        rospy.logwarn(f"Arm control mode change timeout after {timeout} seconds, current mode: {final_mode}, target: {target_mode}")
        return False

    def get_current_controller_name(self):
        """获取当前控制器名称（用于 multi 模式判断）
        :return: str, 当前控制器名称，如果获取失败返回 None
        """
        if self.kuavo_control_scheme != "multi":
            return None
        
        service_name = "/humanoid_controller/get_controller_list"
        # lookupService 对缺失服务返回 [-1,'no provider','']（不抛异常），故按返回码判断；
        # 服务未 advertise 时立即返回，避免 wait_for_service 阻塞 0.5s
        try:
            code, _, _ = rospy.get_master().lookupService(service_name)
            if code != 1:
                return None
        except Exception:
            pass  # master 不可达，走下方 wait_for_service 超时兜底
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            get_controller_client = rospy.ServiceProxy(service_name, getControllerList)
            response = get_controller_client()
            if response.success:
                rospy.logdebug(f"Current controller in multi mode: {response.current_controller}")
                return response.current_controller
            else:
                rospy.logwarn(f"Get controller list failed: {response.message}")
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn(f"Service '{service_name}' call failed: {e}, assuming ocs2 behavior")
        return None

    def prepare_more_arm_action(self, action_name):
        """Ask MoRE to validate and prepare an action before publishing any action state."""
        service_name = "/humanoid_controller/more_prepare_arm_action"
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            prepare_action = rospy.ServiceProxy(service_name, ExecuteArmAction)
            response = prepare_action(action_name=action_name)
            if not response.success:
                rospy.logwarn(
                    "MoRE rejected action '%s': %s", action_name, response.message
                )
            return bool(response.success), response.message
        except (rospy.ServiceException, rospy.ROSException) as e:
            message = f"MoRE action preparation service failed: {e}"
            rospy.logwarn(message)
            return False, message

    def call_switch_controller_service(self, controller_name):
        service_name = "/humanoid_controller/switch_controller"
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            switch_controller = rospy.ServiceProxy(service_name, switchController)
            resp = switch_controller(controller_name=controller_name)
            if not resp.success:
                rospy.logwarn(
                    "Switch controller to %s rejected: %s",
                    controller_name,
                    resp.message
                )
                return False
            return True
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn("Switch controller service '%s' call failed: %s", service_name, e)
            return False

    def _get_preferred_controller(self):
        """MoRE 优先，不在线则 fallback AMP"""
        try:
            resp = rospy.ServiceProxy("/humanoid_controller/get_controller_list", getControllerList)()
            if resp.success and "more_controller" in resp.controller_names:
                return "more_controller"
        except: pass
        return "amp_controller"

    def ensure_tact_playback_controller(self, timeout=3.0):
        if self.kuavo_control_scheme != "multi":
            return True

        target_controller = "amp_controller"
        start_time = rospy.Time.now()
        switch_requested = False

        while (rospy.Time.now() - start_time).to_sec() < timeout and not rospy.is_shutdown():
            current_controller = self.get_current_controller_name()
            action = get_tact_playback_controller_action(
                self.kuavo_control_scheme,
                self.robot_class,
                self.robot_version.major(),
                current_controller
            )
            if action == "allowed":
                return True

            if current_controller is None:
                rospy.sleep(0.05)
                continue

            if not switch_requested:
                rospy.loginfo(
                    "当前控制器为 %s，播放上肢动作前请求切换到 %s",
                    current_controller,
                    target_controller
                )
                self.request_stance()
                if not self.call_switch_controller_service(target_controller):
                    return False
                switch_requested = True

            rospy.sleep(0.05)

        current_controller = self.get_current_controller_name()
        return get_tact_playback_controller_action(
            self.kuavo_control_scheme,
            self.robot_class,
            self.robot_version.major(),
            current_controller
        ) == "allowed"

    def get_current_control_mode(self):
        """获取当前实际控制模式
        :return: str, 当前控制模式 ("rl" 或 "ocs2")，如果获取失败返回 "ocs2"（保守策略）
        """
        # 控制模式到控制器名称集合的映射
        mode_controllers = {
            "rl": {"amp_controller", "more_controller"},
            "ocs2": {"mpc"},
        }
        
        # 直接映射的控制方案
        control_scheme_list = ["ocs2","rl"]
        if self.kuavo_control_scheme in control_scheme_list:
            return self.kuavo_control_scheme
        
        # multi 模式需要查询当前控制器
        if self.kuavo_control_scheme == "multi":
            controller = self.get_current_controller_name()
            if controller:
                for mode, controllers in mode_controllers.items():
                    if controller.lower() in controllers:
                        return mode
                rospy.logwarn(f"Unknown controller '{controller}' in multi mode")
        
        # 默认返回 ocs2（保守策略）
        return "ocs2"

    def load_json_file(self, file_path):
        try:
            with open(file_path, "r") as f:
                return json.load(f)
        except IOError as e:
            rospy.logerr(f"Error reading file {file_path}: {e}")
            return None

    def validate_tact_file(self, data):
        """
        验证tact文件的合法性，以抱拳.tact为标准
        :param data: 加载的JSON数据
        :return: (is_valid, error_message)
        """
        errors = []
        
        # 检查必需字段
        required_fields = ["frames", "finish", "first", "robotType"]
        for field in required_fields:
            if field not in data:
                return False, f"缺少必需字段: {field}"
        
        # 通过机器人版本判断机器人类型，确定servos数组的期望长度
        if self.robot_class == ROBAN:
            expected_servos_length = self.ROBAN_TACT_LENGTH
            expected_attribute_keys = set(str(i) for i in range(1, self.ROBAN_TACT_LENGTH + 1))
        else:  # KUAVO
            tact_length = self.KUAVO_TACT_LENGTH + (1 if self.has_waist else 0)
            expected_servos_length = tact_length
            expected_attribute_keys = set(str(i) for i in range(1, tact_length + 1))
        
        frames = data.get("frames", [])
        if not frames:
            return False, "frames数组为空"
        
        # 检查每个frame
        for frame_idx, frame in enumerate(frames):
            # 检查servos数组
            if "servos" not in frame:
                errors.append(f"frame[{frame_idx}]: 缺少servos字段")
                continue
            
            servos = frame["servos"]
            if not isinstance(servos, list):
                errors.append(f"frame[{frame_idx}]: servos必须是数组")
                continue
            
            # 检查servos数组长度
            if len(servos) != expected_servos_length:
                errors.append(f"frame[{frame_idx}]: servos数组长度应为{expected_servos_length}，实际为{len(servos)}")
            
            # 检查servos数组中的值（不能为null，必须是数字）
            for servo_idx, servo_value in enumerate(servos):
                if servo_value is None:
                    errors.append(f"frame[{frame_idx}]: servos[{servo_idx}]不能为null")
                elif not isinstance(servo_value, (int, float)):
                    errors.append(f"frame[{frame_idx}]: servos[{servo_idx}]必须是数字，实际类型为{type(servo_value).__name__}")
            
            # 检查keyframe
            if "keyframe" not in frame:
                errors.append(f"frame[{frame_idx}]: 缺少keyframe字段")
            else:
                keyframe = frame["keyframe"]
                if not isinstance(keyframe, (int, float)):
                    errors.append(f"frame[{frame_idx}]: keyframe必须是数字")
                elif keyframe < 0:
                    errors.append(f"frame[{frame_idx}]: keyframe不能为负数")
            
            # 检查attribute
            if "attribute" not in frame:
                errors.append(f"frame[{frame_idx}]: 缺少attribute字段")
                continue
            
            attribute = frame["attribute"]
            if not isinstance(attribute, dict):
                errors.append(f"frame[{frame_idx}]: attribute必须是对象-f")
                continue
            
            # 检查attribute中的键
            actual_keys = set(attribute.keys())
            missing_keys = expected_attribute_keys - actual_keys
            if missing_keys:
                errors.append(f"frame[{frame_idx}]: attribute缺少键: {sorted(missing_keys)}")
            
            # 检查每个attribute项的结构
            for key in sorted(expected_attribute_keys):
                if key not in attribute:
                    continue
                
                attr_item = attribute[key]
                if not isinstance(attr_item, dict):
                    errors.append(f"frame[{frame_idx}]: attribute['{key}']必须是对象")
                    continue
                
                # 检查CP字段
                if "CP" not in attr_item:
                    errors.append(f"frame[{frame_idx}]: attribute['{key}']缺少CP字段")
                else:
                    CP = attr_item["CP"]
                    if not isinstance(CP, list) or len(CP) != 2:
                        errors.append(f"frame[{frame_idx}]: attribute['{key}'].CP必须是包含2个元素的数组")
                    else:
                        for cp_idx, cp_point in enumerate(CP):
                            if not isinstance(cp_point, list) or len(cp_point) != 2:
                                errors.append(f"frame[{frame_idx}]: attribute['{key}'].CP[{cp_idx}]必须是[x, y]格式的数组")
                            else:
                                if not all(isinstance(x, (int, float)) for x in cp_point):
                                    errors.append(f"frame[{frame_idx}]: attribute['{key}'].CP[{cp_idx}]的元素必须是数字")
                
                # 检查CPType字段
                if "CPType" not in attr_item:
                    errors.append(f"frame[{frame_idx}]: attribute['{key}']缺少CPType字段")
                else:
                    CPType = attr_item["CPType"]
                    if not isinstance(CPType, list) or len(CPType) != 2:
                        errors.append(f"frame[{frame_idx}]: attribute['{key}'].CPType必须是包含2个元素的数组")
                    else:
                        if not all(isinstance(x, str) for x in CPType):
                            errors.append(f"frame[{frame_idx}]: attribute['{key}'].CPType的元素必须是字符串")
                        # 检查CPType的值（通常是"AUTO"）
                        for cp_type_idx, cp_type_value in enumerate(CPType):
                            if cp_type_value not in ["AUTO", "MANUAL"]:
                                errors.append(f"frame[{frame_idx}]: attribute['{key}'].CPType[{cp_type_idx}]应为'AUTO'或'MANUAL'，实际为'{cp_type_value}'")
        
        # 检查finish和first字段
        numeric_fields = ["finish", "first"]
        for field in numeric_fields:
            field_value = data.get(field)
            if field_value is not None:
                if not isinstance(field_value, (int, float)):
                    errors.append(f"{field}字段必须是数字")
                elif field_value < 0:
                    errors.append(f"{field}字段不能为负数")
        
        if errors:
            error_msg = "文件合法性检查失败:\n" + "\n".join(f"  - {err}" for err in errors)
            return False, error_msg
        
        return True, None

    def create_init_stand_frame(self, frames, is_rl=False):
        """
        创建初始站立帧（0f处）
        如果0f处没有动作帧，使用指定的初始站立姿态
        :param frames: 现有的frames列表，用于获取servos长度和attribute结构
        :return: 初始站立帧字典
        """
        # 根据机器人类型确定初始站立帧的servos值
        if self.robot_class == KUAVO:
            # KUAVO的初始站立帧值（前14个关节）
            init_stand_servos = [0] * len(self.arm_joints_deg) if is_rl else self.arm_joints_deg
            tact_length = self.KUAVO_TACT_LENGTH + (1 if self.has_waist else 0)
        else:  # ROBAN
            # ROBAN的初始站立帧值（前8个关节），与抱拳.tact等标准动作文件保持一致
            init_stand_servos = [22.6, 10, 0, -54.4, 22.6, -10, 0, -54.4]
            tact_length = self.ROBAN_TACT_LENGTH
        
        # 如果frames不为空，使用第一个frame来确定servos长度和attribute结构
        if frames and len(frames) > 0:
            first_frame = frames[0]
            expected_length = len(first_frame.get("servos", []))
            # 使用第一个frame的attribute结构作为模板
            template_attribute = first_frame.get("attribute", {})
        else:
            expected_length = tact_length
            template_attribute = {}
        
        # 补全servos数组到期望长度
        if len(init_stand_servos) < expected_length:
            servos = init_stand_servos + [0] * (expected_length - len(init_stand_servos))
        else:
            servos = init_stand_servos[:expected_length]
        
        # 构造attribute，如果没有模板则创建默认结构
        if template_attribute:
            # 深拷贝模板attribute，保持与原始文件结构一致（包括是否有select字段）
            import copy
            attribute = copy.deepcopy(template_attribute)
        else:
            # 创建默认的attribute结构，与标准tact文件格式保持一致
            attribute = {}
            for i in range(1, expected_length + 1):
                attribute[str(i)] = {
                    "CP": [[0, 0], [0, 0]],  # 与标准tact文件格式一致
                    "CPType": ["AUTO", "AUTO"]
                    # 注意：不添加"select"字段，与第一个frame的格式保持一致
                }
        
        # 创建初始站立帧
        init_frame = {
            "servos": servos,
            "keyframe": 0,
            "attribute": attribute
        }

        return init_frame

    def _max_arm_pose_diff_deg(self, current_arm_joint_state_rad, target_servos_deg):
        """
        计算"当前手臂姿态(弧度)"与"目标 servos(度)"在前 14 个手臂关节上的最大角度差(度)。
        用于判断是否需要在 tact 首帧之前插入"当前姿态过渡帧"。

        :param current_arm_joint_state_rad: 当前手臂关节状态数组(弧度)
        :param target_servos_deg: 目标 servos 数组(度)
        :return: 最大角度差(度);任一数组为空时返回 0.0
        """
        if not current_arm_joint_state_rad or not target_servos_deg:
            return 0.0
        # 兼容 ROBAN(8 关节)/ KUAVO(14 关节)/ KUAVO+腰(15 关节)的不同长度,
        # 仅比较手臂部分(前 14 个),腰部不参与首帧过渡判断
        compare_len = min(len(current_arm_joint_state_rad), len(target_servos_deg), 14)
        max_diff = 0.0
        for idx in range(compare_len):
            cur_deg = math.degrees(current_arm_joint_state_rad[idx])
            diff = abs(cur_deg - target_servos_deg[idx])
            if diff > max_diff:
                max_diff = diff
        return max_diff

    def calculate_transition_time(self, source_angles, target_angles, min_keyframe=50, max_keyframe=400, default_keyframe=200):
        """
        根据两个角度数组的差值动态计算过渡时间（优化版本）
        
        优化点：
        1. 考虑不同关节的速度限制（肩膀关节速度更快）
        2. 计算每个关节所需时间，取最大值（更精确）
        3. 使用更合理的速度限制值（基于实际硬件参数）
        
        :param source_angles: 源角度数组（度）
        :param target_angles: 目标角度数组（度）
        :param min_keyframe: 最小过渡时间（keyframe单位，默认50，即0.5秒）
        :param max_keyframe: 最大过渡时间（keyframe单位，默认400，即4.0秒）
        :param default_keyframe: 默认过渡时间（keyframe单位，默认200，即2.0秒）
        :return: 过渡时间（keyframe单位）
        """
        # 整条手臂关节索引：14个手臂关节（索引0-13）
        arm_joint_indices = list(range(14))
        
        # 关节速度限制配置（度/秒）
        # 肩膀关节（索引0和7，左右臂第一个关节）速度更快，其他关节较慢
        # 基于实际硬件参数：普通关节约50度/秒，肩膀关节约120度/秒（每步限制）
        # 为安全起见，使用保守值：普通关节40度/秒，肩膀关节100度/秒
        SHOULDER_JOINT_INDICES = [0, 7]  # 左右臂肩膀关节索引
        NORMAL_JOINT_VELOCITY = 40.0  # 普通关节速度限制（度/秒）
        SHOULDER_JOINT_VELOCITY = 100.0  # 肩膀关节速度限制（度/秒）
        
        # 计算每个关节所需的时间，取最大值
        max_required_time = 0.0
        total_angle_diff = 0.0
        valid_joint_count = 0
        
        for idx in arm_joint_indices:
            if idx < len(source_angles) and idx < len(target_angles):
                source_angle = source_angles[idx]
                target_angle = target_angles[idx]
                angle_diff = abs(source_angle - target_angle)
                
                # 根据关节类型选择速度限制
                if idx in SHOULDER_JOINT_INDICES:
                    joint_velocity = SHOULDER_JOINT_VELOCITY
                else:
                    joint_velocity = NORMAL_JOINT_VELOCITY
                
                # 计算该关节所需的时间（秒）
                # 时间 = 角度差 / 速度限制
                if joint_velocity > 0:
                    required_time = angle_diff / joint_velocity
                    # 转换为keyframe（1 keyframe = 0.01秒）
                    required_keyframe = required_time * 100.0
                    if required_keyframe > max_required_time:
                        max_required_time = required_keyframe
                
                total_angle_diff += angle_diff
                valid_joint_count += 1
        
        # 根据计算结果确定过渡时间
        if valid_joint_count > 0:
            # 使用最大所需时间，并添加安全余量（20%）
            # 这样可以确保所有关节都能平滑过渡
            transition_keyframe_raw = max_required_time * 1.2
            
            # 如果最大所需时间很小，考虑平均角度差值作为补充``
            # 避免单个关节的小幅移动导致时间过短
            if max_required_time < 50:  # 如果最大时间小于0.5秒
                avg_angle_diff = total_angle_diff / valid_joint_count
                # 使用平均速度限制计算平均时间
                avg_velocity = (NORMAL_JOINT_VELOCITY * (valid_joint_count - 2) + 
                               SHOULDER_JOINT_VELOCITY * 2) / valid_joint_count if valid_joint_count > 2 else NORMAL_JOINT_VELOCITY
                avg_required_time = (avg_angle_diff / avg_velocity) * 100.0 if avg_velocity > 0 else 0
                # 取最大值和平均值的较大者
                transition_keyframe_raw = max(transition_keyframe_raw, avg_required_time * 0.8)
            
            transition_keyframe = int(transition_keyframe_raw)
            # 限制在合理范围内
            transition_keyframe = max(min_keyframe, min(max_keyframe, transition_keyframe))
        else:
            # 如果没有有效的关节数据，使用默认值
            transition_keyframe = default_keyframe
        
        return transition_keyframe

    def add_init_frame(self, frames, is_rl=False, is_first_stage=True):
        action_data = {}
        # 记录本次是否在 0f 处插入了"当前姿态→tact 首帧"的过渡帧。
        # 外层 handle_execute_action 据此决定是否重算 END_FRAME_TIME 与延时上报时机。
        self._last_inserted_init_kf = 0

        # rl 要在刚开始插入当前状态为初始值来平滑过渡，ocs2 不需要
        if is_rl:
            import copy

            # 检查当前状态和第一帧的差异
            first_frame = frames[0]

            if is_first_stage and not self.interrupt_flag:
                # 检查当前状态和第一帧的差异
                current_angles_deg = [math.degrees(pos) for pos in self.current_arm_joint_state[:len(first_frame["servos"])]]
            else:
                current_angles_deg = self.servos_start

            # 使用 calculate_transition_time 精确计算过渡时间
            # 考虑每个关节的实际差异和速度限制，比简单的平均差异更准确
            transition_keyframes = self.calculate_transition_time(
                current_angles_deg,
                first_frame["servos"],
                min_keyframe=10,   # 最小0.1秒（姿态几乎一致）
                max_keyframe=100,  # 最大1秒（姿态差异大）
                default_keyframe=50  # 默认0.5秒
            )

            rospy.loginfo(f"RL模式过渡时间：{transition_keyframes} keyframes ({transition_keyframes * 0.01:.2f}秒)")

            for frame in frames:
                frame["keyframe"] += transition_keyframes
            frame0 = copy.deepcopy(frames[0])
            # 如果原来的长度长则补全，否则就需要裁剪
            if is_first_stage and not self.interrupt_flag:
                if len(self.current_arm_joint_state) > len(frame0["servos"]):
                    # 当前状态长度更长，需要裁剪到原始长度
                    frame0["servos"] = [math.degrees(pos) for pos in self.current_arm_joint_state[:len(frame0["servos"])]]
                else:
                    # 当前状态长度更短或相等，需要补全到原始长度
                    frame0["servos"] = [math.degrees(pos) for pos in self.current_arm_joint_state] + [0] * (len(frame0["servos"]) - len(self.current_arm_joint_state))
            else:
                frame0["servos"] = self.servos_start
            frame0["keyframe"] = 0
            frames.insert(0, frame0)
        
        # 通用入场过渡(非 RL, 非 KUAVO 半身): 当前姿态与首帧差异显著时,
        # 在 0f 插入"当前姿态"作过渡起点并后移原 frames, 避免播放瞬间顺移。
        if (not is_rl
                and not (self.robot_class == KUAVO and self.only_half_up_body)
                and len(frames) > 0
                and frames[0].get("keyframe", 0) == 0
                and is_first_stage
                and not self.interrupt_flag
                and hasattr(self, 'current_arm_joint_state')
                and len(self.current_arm_joint_state) > 0):
            import copy

            first_frame = frames[0]
            max_diff = self._max_arm_pose_diff_deg(
                self.current_arm_joint_state, first_frame["servos"]
            )
            if max_diff > 3.0:  # 阈值 3°: 姿态接近时不引入额外延时
                # 仅取前 N 个关节(对齐 first_frame.servos 长度)转度作为过渡时间计算源
                source_len = min(len(self.current_arm_joint_state), len(first_frame["servos"]))
                current_angles_deg = [math.degrees(p) for p in self.current_arm_joint_state[:source_len]]
                transition_keyframe = self.calculate_transition_time(
                    current_angles_deg,
                    first_frame["servos"],
                    min_keyframe=50,    # 0.5 秒
                    max_keyframe=400,   # 4.0 秒
                    default_keyframe=200  # 2.0 秒
                )

                # 整体后移原 frames
                for frame in frames:
                    frame["keyframe"] += transition_keyframe

                # 构造"当前姿态帧"作为新的 0f 帧:沿用 first_frame 的 attribute 结构,
                # servos 替换为当前姿态(度);超长截断,不足补 0
                current_frame = copy.deepcopy(first_frame)
                current_frame["keyframe"] = 0
                if len(self.current_arm_joint_state) > len(current_frame["servos"]):
                    current_frame["servos"] = [
                        math.degrees(p) for p in self.current_arm_joint_state[:len(current_frame["servos"])]
                    ]
                else:
                    current_frame["servos"] = (
                        [math.degrees(p) for p in self.current_arm_joint_state]
                        + [0] * (len(current_frame["servos"]) - len(self.current_arm_joint_state))
                    )
                frames.insert(0, current_frame)
                self._last_inserted_init_kf = transition_keyframe
                rospy.loginfo(
                    "[首帧过渡] 当前姿态→tact首帧 max_diff=%.2f°, 过渡时间=%.2f秒",
                    max_diff, transition_keyframe * 0.01,
                )

        # ocs2 模式和半身模式：在第一帧之前插入当前手臂姿态作为第一帧
        if not is_rl and self.robot_class == KUAVO and self.only_half_up_body and len(frames) > 0:
            import copy
            
            # 获取第一帧
            first_frame = frames[0]
            
            # 检查当前姿态是否可用
            if hasattr(self, 'current_arm_joint_state') and len(self.current_arm_joint_state) > 0:
                # 将当前姿态转换为度
                current_angles_deg = [math.degrees(pos) for pos in self.current_arm_joint_state]
                # 确保 standJointState 数据可用
                if hasattr(self, 'arm_joints_deg') and len(self.arm_joints_deg) >= 14:
                    # 计算过渡时间：从当前姿态到站立帧（standJointState），最小0.5秒，最大4.0秒
                    # 这样更合理：当前位置先过渡到站立帧，然后再从站立帧过渡到第一帧
                    transition_keyframe = self.calculate_transition_time(
                        current_angles_deg, 
                        self.arm_joints_deg,
                        min_keyframe=50,  # 0.5秒
                        max_keyframe=400,  # 4.0秒
                        default_keyframe=200  # 2.0秒
                    )
                else:
                    # 如果没有站立帧数据，则使用默认值
                    transition_keyframe = 100  # 1.0秒
                
                # 先将所有现有帧的keyframe向后偏移
                for frame in frames:
                    frame["keyframe"] += transition_keyframe
                
                # 创建过渡帧，使用当前姿态
                current_frame = copy.deepcopy(first_frame)
                current_frame["keyframe"] = 0
                
                # 更新过渡帧的servos：使用当前实际姿态
                # 如果当前状态长度更长，需要裁剪到原始长度
                if len(self.current_arm_joint_state) > len(current_frame["servos"]):
                    current_frame["servos"] = [math.degrees(pos) for pos in self.current_arm_joint_state[:len(current_frame["servos"])]]
                else:
                    # 当前状态长度更短或相等，需要补全到原始长度
                    current_frame["servos"] = [math.degrees(pos) for pos in self.current_arm_joint_state] + [0] * (len(current_frame["servos"]) - len(self.current_arm_joint_state))
                
                # 将当前姿态帧插入到第一帧位置
                frames.insert(0, current_frame)
        
        # ocs2 模式和半身模式：在最后一帧后面添加一帧，整条手臂平滑过渡到 standJointState
        if not is_rl and self.robot_class == KUAVO and self.only_half_up_body:
            import copy
            
            # 找到最后一帧（keyframe 最大的帧）
            last_frame = max(frames, key=lambda f: f.get("keyframe", 0))
            last_keyframe = last_frame.get("keyframe", 0)
            
            # 确保 standJointState 数据可用
            if hasattr(self, 'arm_joints_deg') and len(self.arm_joints_deg) >= 14:
                # 计算过渡时间：从最后一帧到standJointState，最小1.0秒，最大4.0秒
                transition_keyframe = self.calculate_transition_time(
                    last_frame["servos"],
                    self.arm_joints_deg,
                    min_keyframe=100,  # 1.0秒
                    max_keyframe=400,  # 4.0秒
                    default_keyframe=200  # 2.0秒
                )
                
                # 创建新的一帧，基于最后一帧
                smooth_frame = copy.deepcopy(last_frame)
                smooth_frame["keyframe"] = last_keyframe + transition_keyframe
                
                # 更新整条手臂关节角度：从最后一帧过渡到 standJointState
                # 整条手臂关节索引：14个手臂关节（索引0-13）
                for idx in range(14):
                    if idx < len(smooth_frame["servos"]) and idx < len(self.arm_joints_deg):
                        # 使用 standJointState 的手臂关节角度
                        smooth_frame["servos"][idx] = self.arm_joints_deg[idx]
                
                # 将新帧添加到 frames 列表
                frames.append(smooth_frame)
            else:
                # 即使没有 standJointState，也创建一个默认的过渡帧（2秒），保持最后一帧的角度不变
                smooth_frame = copy.deepcopy(last_frame)
                smooth_frame["keyframe"] = last_keyframe + 200
                frames.append(smooth_frame)

        # 计算结束键值：KUAVO根据是否有腰部关节调整
        if self.robot_class == KUAVO:
            end_key = (self.KUAVO_TACT_LENGTH + (1 if self.has_waist else 0)) + 1
        else:
            end_key = self.ROBAN_TACT_LENGTH + 1

        for frame in frames:
            servos, keyframe, attribute = frame["servos"], frame["keyframe"], frame["attribute"]
            for index, value in enumerate(servos):
                key = index + 1
                if key == end_key:
                    break
                if key not in action_data:
                    action_data[key] = []
                    if keyframe != 0 and len(action_data[key]) == 0:
                        if key <= len(self.INIT_ARM_POS):
                            action_data[key].append([
                                [0, math.radians(self.INIT_ARM_POS[key - 1])],
                                [0, math.radians(self.INIT_ARM_POS[key - 1])],
                                [0, math.radians(self.INIT_ARM_POS[key - 1])],
                            ])
                if value is not None:
                    CP = attribute[str(key)]["CP"]
                    left_CP, right_CP = CP
                    action_data[key].append([
                        [round(keyframe / 100, 5), math.radians(value)],
                        [round((keyframe + left_CP[0]) / 100, 5), math.radians(value + left_CP[1])],
                        [round((keyframe + right_CP[0]) / 100, 5), math.radians(value + right_CP[1])],
                    ])
        return action_data

    def crop_frames_from_first(self, frames, first_keyframe):
        """按 tact 原始 first 裁剪动作段，并将剩余 keyframe 归零。"""
        import copy

        first_keyframe = int(first_keyframe)
        cropped_frames = []
        for frame in frames:
            keyframe = frame.get("keyframe", 0)
            if keyframe >= first_keyframe:
                new_frame = copy.deepcopy(frame)
                new_frame["keyframe"] = keyframe - first_keyframe
                cropped_frames.append(new_frame)

        if not cropped_frames and frames:
            new_frame = copy.deepcopy(frames[-1])
            new_frame["keyframe"] = 0
            cropped_frames.append(new_frame)

        return cropped_frames

    def filter_data(self, action_data):
        filtered_action_data = {}
        for key, frames in action_data.items():
            filtered_frames = []
            found_start = False
            skip_next = False
            for i in range(-1, len(frames)):
                frame = frames[i]
                if i == len(frames) - 1:
                    next_frame = frame
                else:
                    next_frame = frames[i + 1]
                end_time = next_frame[0][0]
                if not found_start and end_time >= self.START_FRAME_TIME:
                    found_start = True
                    p0 = np.array([0, self.current_arm_joint_state[key - 1]])
                    p3 = np.array([next_frame[0][0] - self.x_shift, next_frame[0][1]])

                    # 计算控制点，但使用更保守的方法避免过度摆动
                    # 使用较短的控制杆长度以减少过渡期间的运动幅度
                    curve_length = np.linalg.norm(p3 - p0)
                    p1 = p0 + curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the right
                    p2 = p3 - curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the left


                    # 创建新帧
                    frame1 = [
                        p0.tolist(),
                        p0.tolist(),
                        p1.tolist()
                    ]

                    # 修改下一帧的左控制点
                    next_frame[1] = p2.tolist()

                    filtered_frames.append(frame1)
                    skip_next = True

                if found_start:
                    if skip_next:
                        skip_next = False
                        continue
                    end_point = [round(frame[0][0] - self.x_shift, 5), round(frame[0][1], 5)]
                    left_control_point = [round(frame[1][0] - self.x_shift, 5), round(frame[1][1], 5)]
                    right_control_point = [round(frame[2][0] - self.x_shift, 5), round(frame[2][1], 5)]
                    filtered_frames.append([end_point, left_control_point, right_control_point])

            filtered_action_data[key] = filtered_frames
        return filtered_action_data

    def _start_action_state_heartbeat(self, controller_name):
        """Start one state=1 heartbeat thread for the complete play/reset session."""
        with self._action_state_lock:
            if (self.running_action or
                    self._controller_switch_abort_event.is_set() or
                    not self.enable_control_state_):
                self._preparing_more_action = False
                return None

            stop_event = threading.Event()
            state_thread = threading.Thread(
                target=self.publish_running_action_state,
                args=(stop_event,),
                name="arm_action_state_heartbeat",
                daemon=True,
            )
            self.running_action = True
            self._terminal_state_published = False
            self._active_action_controller = controller_name
            if controller_name == "more_controller":
                # 丢弃前一个动作 session 的 action 尾帧。当前新动作的第一段
                # 必须从传感器实测姿态开始，不能从旧动作缓存开始。
                self._last_kuavo_action_traj_msg = None
                self._last_more_published_servos_deg = None
                self._begin_more_bezier_phase()
            self._pending_terminal_state = 2
            self._action_owns_phase2 = False
            self._action_owns_external_arm_mode = False
            self._preparing_more_action = False
            self._action_state_stop_event = stop_event
            self._action_state_thread = state_thread
            # Commit ACTIVE synchronously before any zero-duration action timer
            # can publish its terminal state.  The background thread only
            # supplies subsequent heartbeats.
            self.publish_action_state(1)
            state_thread.start()
        return True

    def _finish_action(self, state):
        """Stop the state=1 heartbeat, then publish exactly one terminal state."""
        if state not in (0, 2):
            raise ValueError(f"Invalid terminal action state: {state}")

        with self._action_state_lock:
            if self._terminal_state_published:
                return False
            # Safety abort wins over a concurrently completing timer.  The
            # trajectory may have reached its nominal end, but ownership was
            # revoked before the session committed its terminal result.
            if (state == 2 and
                    (not self.enable_control_state_ or
                     (self._active_action_controller == "more_controller" and
                      self._controller_switch_abort_event.is_set()))):
                state = 0
            self._terminal_state_published = True
            stop_event = self._action_state_stop_event
            state_thread = self._action_state_thread
            active_controller = self._active_action_controller
            owns_phase2 = self._action_owns_phase2
            owns_external_arm_mode = self._action_owns_external_arm_mode

        # MoRE 的最终 mode/style 恢复由 C++ session 统一完成。
        # 其他控制器仅释放本 session 真正获取过的共享控制权；
        # 正常 keep-pose/freeze 仍保留原有语义。
        if state == 0 and owns_phase2:
            self.call_enable_wbc_arm_trajectory_control_service(0)
        if (state == 0 and owns_external_arm_mode
                and active_controller != "more_controller"):
            self.call_change_arm_ctrl_mode_service(1)

        if stop_event is not None:
            stop_event.set()
        if state_thread is not None and state_thread is not threading.current_thread():
            state_thread.join(timeout=1.0)
            if state_thread.is_alive():
                rospy.logwarn("Action-state heartbeat thread did not stop within 1 second")

        with self._action_state_lock:
            # Re-evaluate at the actual publication commit point.  A safety
            # callback may have arrived while the heartbeat thread was joining.
            if (state == 2 and
                    (not self.enable_control_state_ or
                     (active_controller == "more_controller" and
                      self._controller_switch_abort_event.is_set()))):
                state = 0
            self.publish_action_state(state)
            # 终态已发布后才释放 busy，防止新动作的 state=1
            # 与旧动作延迟发布的终态发生倒序。
            self.running_action = False
            if self._action_state_thread is state_thread:
                self._action_state_thread = None
                self._action_state_stop_event = None
            self._active_action_controller = None
            self._preparing_more_action = False
            self._pending_terminal_state = 2
            self._action_owns_phase2 = False
            self._action_owns_external_arm_mode = False
        return True

    def _stop_trajectory_publisher(self):
        """Stop and join the current trajectory publisher before changing phases."""
        self.arm_flag = False
        stop_event = self._trajectory_stop_event
        trajectory_thread = self._trajectory_thread
        if stop_event is not None:
            stop_event.set()

        if trajectory_thread is not None and trajectory_thread is not threading.current_thread():
            trajectory_thread.join(timeout=1.0)
            if trajectory_thread.is_alive():
                rospy.logerr("Trajectory publisher thread did not stop within 1 second")
                return False

        if self._trajectory_thread is trajectory_thread:
            self._trajectory_thread = None
            self._trajectory_stop_event = None
        return True

    def _start_trajectory_publisher(self):
        """Start one trajectory publisher for either the main action or reset phase."""
        if (self._controller_switch_abort_event.is_set() or
                not self.enable_control_state_):
            rospy.logwarn("Refuse to start arm trajectory after its controller switched away")
            return False
        if not self._stop_trajectory_publisher():
            return False
        if (self._controller_switch_abort_event.is_set() or
                not self.enable_control_state_):
            return False

        self.interrupt_flag = False
        self.arm_flag = True
        stop_event = threading.Event()
        trajectory_thread = threading.Thread(
            target=self.run,
            args=(stop_event,),
            name="arm_trajectory_publisher",
            daemon=True,
        )
        self._trajectory_stop_event = stop_event
        self._trajectory_thread = trajectory_thread
        trajectory_thread.start()
        if (self._controller_switch_abort_event.is_set() or
                not self.enable_control_state_):
            self.arm_flag = False
            stop_event.set()
            trajectory_thread.join(timeout=1.0)
            if (not trajectory_thread.is_alive() and
                    self._trajectory_thread is trajectory_thread):
                self._trajectory_thread = None
                self._trajectory_stop_event = None
            return False
        return True

    def delayed_publish_action_state(self, delay):
        """
        延时发布动作完成状态。（增加中断检查）
        :param delay: 延迟时间（秒）
        """
        rospy.loginfo(f"Delaying action completion state for {delay} seconds...")
        self._schedule_action_timer(delay, self._on_timer_trigger)

    def _schedule_action_timer(self, delay, callback):
        """Schedule a session timer whose stale callback can be identified and ignored."""
        timer_token = object()
        self._timer_token = timer_token
        self._timer = rospy.Timer(
            rospy.Duration(delay),
            lambda event: callback(event, timer_token),
            oneshot=True,
        )

    def _claim_action_timer(self, timer_token):
        """Return False when a shutdown/replaced timer callback arrives late."""
        if timer_token is not self._timer_token:
            rospy.logdebug("Ignoring stale arm-action timer callback")
            return False
        self._timer = None
        self._timer_token = None
        return True

    def reset_robot_state(self):
        # running_action 和 state=1 心跳在整个复位阶段保持，不在主动作
        # 结束与复位开始之间插入中间 state=2。
        rospy.loginfo("[RESET_START] Starting robot reset; keeping action state=1")

        # MoRE 在 prepare 时已确认为当前控制器。复位期间即使
        # get_controller_list 短暂查询失败，也不得误落入 OCS2 分支切 mode1。
        current_control_mode = (
            "rl" if self._active_action_controller == "more_controller"
            else self.get_current_control_mode()
        )
        if current_control_mode == "rl":
            if not self.rl_reset_robot_state():
                rospy.logerr("[RESET_FAILED] Failed to plan or start RL reset trajectory")
                self._stop_trajectory_publisher()
                self._finish_action(0)
        else:
            # 先禁用 Phase 2 再切 mode，避免竞态窗口内 Phase 2 stale 输出导致全关节 spike
            self.call_enable_wbc_arm_trajectory_control_service(0)
            self.call_change_arm_ctrl_mode_service(1)
            if rospy.get_param('/end_effector_type', '') == 'linker_hand':
                self.hand_state.left_hand_position  = [100, 0, 0, 0, 0, 0]
                self.hand_state.right_hand_position = [100, 0, 0, 0, 0, 0]
            else:
                self.hand_state.left_hand_position  = [0] * 6
                self.hand_state.right_hand_position = [0] * 6

            self.control_hand_pub.publish(self.hand_state)
            self.head_state.joint_data = [0] * 2
            self.control_head_pub.publish(self.head_state)
            # 复位腰部（KUAVO v50+ 或 ROBAN）
            if (self.robot_class == KUAVO and self.has_waist) or self.robot_class == ROBAN:
                self.waist_state.header.stamp = rospy.Time.now()
                self.waist_state.data.data = [0]
                self.control_waist_pub.publish(self.waist_state)

            rospy.loginfo("[RESET_COMPLETE] OCS2/MPC reset finished")
            self._finish_action(self._pending_terminal_state)

    def create_action_data(self, finish_time, is_rl=False):
        # 根据是否有腰部关节确定TACT长度
        if self.robot_class == KUAVO:
            tact_length = self.KUAVO_TACT_LENGTH + (1 if self.has_waist else 0)
        else:
            tact_length = self.ROBAN_TACT_LENGTH
        if is_rl:
            servos_end = [0] * tact_length
            if rospy.get_param('/end_effector_type', '') == 'linker_hand' and tact_length >= 26:
                servos_end[14] = 100
                servos_end[20] = 100
            if self.robot_class == KUAVO:
                left_rad, right_rad = self._rl_shoulder_roll
                servos_end[1] = math.degrees(left_rad)
                servos_end[8] = math.degrees(right_rad)
        else:
            servos_end = self.INIT_ARM_POS
        # # 起始帧从 /kuavo_arm_traj 获取（当前指令位姿）；无数据时回退到 current_arm_joint_state
        self.servos_start = self._get_servos_from_kuavo_arm_traj(tact_length)
        frames = [
            {
                "servos": self.servos_start,
                "keyframe": 0,
                "attribute": {str(i+1): {"CP": [[0,0],[0,0]]} for i in range(tact_length)}
            },
            {
                "servos": servos_end,
                "keyframe": finish_time * 100,
                "attribute": {str(i+1): {"CP": [[0,0],[0,0]]} for i in range(tact_length)}
            },
        ]
        return {"frames": frames}

    def rl_reset_robot_state(self):
        self.START_FRAME_TIME = 0
        self.x_shift = self.START_FRAME_TIME  # 动态调整 x_shift
        finish_time = 1
        data = self.create_action_data(finish_time, is_rl=True)

        # 不需要额外增加时间，add_init_frame会根据实际差异动态添加过渡帧
        self.END_FRAME_TIME = finish_time

        action_data = self.add_init_frame(data["frames"], is_rl=True, is_first_stage=False)

        # RL模式下，add_init_frame可能插入了过渡帧，需要更新END_FRAME_TIME
        current_control_mode = (
            "rl" if self._active_action_controller == "more_controller"
            else self.get_current_control_mode()
        )
        if current_control_mode == "rl":
            frames = data["frames"]
            if frames:
                last_keyframe = max(f.get("keyframe", 0) for f in frames)
                # 将 keyframe 转换为秒并更新 END_FRAME_TIME
                self.END_FRAME_TIME = last_keyframe * 0.01

        filtered_data = self.filter_data(action_data)
        bezier_request = self.create_bezier_request(filtered_data)

        if self._active_action_controller == "more_controller":
            # Do not let the reset publisher replay the main action's last
            # shared JointState while waiting for the planner's first reset
            # sample.  Keep _last_more_published_servos_deg intact: it is the
            # reset trajectory's start pose.
            self._begin_more_bezier_phase()
        success = self.plan_arm_trajectory_bezier_curve_client(bezier_request)
        if success:
            rospy.loginfo("Arm trajectory planned successfully")
            # 启动发布线程执行回归初始位的轨迹。主动作的
            # 发布线程已在 _on_timer_trigger 中停止并 join。
            if not self._start_trajectory_publisher():
                return False
            # 在复位动作完成后，仅停止发布，不再触发再次复位
            self._schedule_action_timer(self.END_FRAME_TIME, self._on_reset_timer_trigger)
            return True
        else:
            return False

    def _on_timer_trigger(self, event, timer_token):
        with self._action_transition_lock:
            try:
                if not self._claim_action_timer(timer_token):
                    return
                # 必须等主动作发布线程真正退出，再开始复位或发布终态。
                if not self._stop_trajectory_publisher():
                    self._finish_action(0)
                    return
                # 动作播放完成以后恢复机器人初始状态
                if self.arm_restore_flag and not self.keep_arm_pose:
                    self.reset_robot_state()
                    rospy.loginfo("After the action playback is complete, revert the robot initial state")
                elif self.keep_arm_pose:
                    rospy.loginfo("After the action playback is complete, keep arm pose at the last tact frame")
                    self.keep_arm_pose = False  # 一次性语义，用完即清，避免污染后续其他入口的动作
                    self._finish_action(2)
                else:
                    rospy.loginfo("After the action playback is complete, arm restore is disabled")
                    self._finish_action(2)
            except Exception as e:
                rospy.logerr("Arm action completion/reset failed: %s", e)
                self.stop_action()
                self._stop_trajectory_publisher()
                if self.running_action:
                    self._finish_action(0)

    def stop_action(self):
        with self._action_transition_lock:
            timer = self._timer
            self._timer = None
            self._timer_token = None
        if timer:
            timer.shutdown()

    def _on_reset_timer_trigger(self, event, timer_token):
        """复位发布结束后发布唯一的最终成功状态。"""
        with self._action_transition_lock:
            try:
                if not self._claim_action_timer(timer_token):
                    return
                if not self._stop_trajectory_publisher():
                    self._finish_action(0)
                    return
                rospy.loginfo(f"[RESET_COMPLETE] Reset trajectory finished at {time.time():.3f}. Stopping publishers. [DEBUG] arm_flag={self.arm_flag}, running_action={self.running_action}")
                is_more_action = self._active_action_controller == "more_controller"
                # MoRE 本 session 从未获取 Phase2；非 MoRE 复位完成后
                # 先释放共享 Phase2，再交还手臂模式。
                if not is_more_action:
                    self.call_enable_wbc_arm_trajectory_control_service(0)
                current_control_mode = (
                    "rl" if is_more_action
                    else self.get_current_control_mode()
                )
                current_controller = self.get_current_controller_name()
                if current_controller is None:
                    current_controller = self._active_action_controller
                # MoRE 由自身动作状态机恢复动作前 style/mode，Python 不得
                # 无条件将它切到 mode1。其他 RL 控制器保持原有恢复行为。
                if (current_control_mode == "rl" and not is_more_action
                        and current_controller != "more_controller"):
                    self.call_change_arm_ctrl_mode_service(1)

                self._finish_action(self._pending_terminal_state)
            except Exception as e:
                rospy.logerr("Arm reset completion failed: %s", e)
                self.stop_action()
                self._stop_trajectory_publisher()
                if self.running_action:
                    self._finish_action(0)

    def publish_running_action_state(self, stop_event):
        """持续发布 state=1"""
        while not rospy.is_shutdown():
            if stop_event.wait(0.1):
                break
            if (not self.running_action or
                    self._controller_switch_abort_event.is_set() or
                    not self.enable_control_state_):
                break
            self.publish_action_state(1)

    def publish_action_state(self, state):
        """
        发布手臂动作状态
        :param state: 动作状态 (0: 失败, 1:执行 2: 成功)
        :param message: 状态描述信息
        """
        with self._action_state_publish_lock:
            state_msg = RobotActionState()
            state_msg.state = state
            self.robot_action_state_pub.publish(state_msg)
            # 只在状态变化时打印日志，减少重复打印
            if self.last_published_state != state:
                rospy.loginfo(f"Robot action state published: state={state}")
                self.last_published_state = state

    def create_bezier_request(self, action_data):
        req = planArmTrajectoryBezierCurveRequest()
        for key, value in action_data.items():
            msg = jointBezierTrajectory()
            for frame in value:
                point = bezierCurveCubicPoint()
                point.end_point, point.left_control_point, point.right_control_point = frame
                msg.bezier_curve_points.append(point)
            req.multi_joint_bezier_trajectory.append(msg)
        req.start_frame_time = self.START_FRAME_TIME
        req.end_frame_time = self.END_FRAME_TIME
        # 基础关节名称（14个手臂关节）
        base_joint_names = [
            "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch",
            "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
            "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch",
            "r_hand_yaw", "r_hand_pitch", "r_hand_roll"
        ]
        # KUAVO v50+: 添加腰部关节
        if self.robot_class == KUAVO and self.has_waist:
            req.joint_names = base_joint_names + ["waist_yaw_joint"]
        else:
            req.joint_names = base_joint_names
        return req

    def plan_arm_trajectory_bezier_curve_client(self, req, wait_timeout=5.0):
        """Wait a bounded time for the planner, then keep the issued call synchronous."""
        service_name = '/bezier/plan_arm_trajectory'
        try:
            rospy.wait_for_service(service_name, timeout=wait_timeout)
        except rospy.ROSException as e:
            rospy.logerr("Planner service %s is unavailable: %s", service_name, e)
            return False
        try:
            plan_service = rospy.ServiceProxy(service_name, planArmTrajectoryBezierCurve)
            response = plan_service(req)
            return bool(response.success)
        except rospy.ServiceException as e:
            rospy.logerr("Planner service call failed: %s", e)
            return False

    def check_nodelet_manager_alive(self):
        """检查 nodelet_manager 节点是否真正在线且可通信
        
        Returns:
            bool: True 如果节点在线且可通信，False 否则
        """
        # 先快速检查：使用 get_num_connections 检查是否有订阅者
        # 这是最快的检查方法，可以避免不必要的子进程调用
        num_subscribers = self.kuavo_arm_traj_pub.get_num_connections()
        if num_subscribers == 0:
            rospy.logwarn("话题 /kuavo_arm_traj 没有订阅者")
            return False
        
        # 有订阅者，进一步验证 nodelet_manager 节点是否真正可通信
        # 方法1: 先检查节点是否在节点列表中
        try:
            node_result = subprocess.run(
                ['rosnode', 'list'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=2
            )
            if node_result.returncode == 0:
                node_list = node_result.stdout.decode('utf-8', errors='ignore')
                # 检查 nodelet_manager 是否在节点列表中
                if '/nodelet_manager' not in node_list:
                    # 检查是否有包含 nodelet_manager 的节点名
                    found_nodelet = False
                    for line in node_list.split('\n'):
                        if 'nodelet_manager' in line.strip():
                            found_nodelet = True
                            break
                    if not found_nodelet:
                        rospy.logwarn("nodelet_manager 节点不在节点列表中")
                        return False
            else:
                # rosnode list 失败，继续尝试 ping
                stderr_output = node_result.stderr.decode('utf-8', errors='ignore')
                rospy.logwarn(f"rosnode list 命令失败: {stderr_output.strip()}")
        except Exception as e:
            rospy.logwarn(f"检查节点列表时出错: {e}，继续尝试 ping")
        
        # 方法2: 使用 rosnode ping 验证节点是否真正可通信（最可靠）
        # 这会真正尝试与节点建立连接，即使节点在 master 中注册但已崩溃也会返回 False
        # 注意：rosnode ping 即使失败也可能返回 0，需要检查输出内容
        max_retries = 2
        for attempt in range(max_retries):
            try:
                result = subprocess.run(
                    ['rosnode', 'ping', '/nodelet_manager', '-c', '1'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,  # 将 stderr 合并到 stdout，因为错误信息可能在这里
                    timeout=3,  # 增加超时时间，匹配 rosnode ping 的默认超时
                    text=True  # 直接返回字符串而不是 bytes
                )
                
                # 当 stderr=subprocess.STDOUT 时，所有输出都在 stdout 中
                output = result.stdout or ""
                
                # 检查输出中是否包含错误信息
                output_lower = output.lower()
                has_error = 'error' in output_lower or 'connection refused' in output_lower or 'failed' in output_lower
                has_success = 'xmlrpc reply from' in output_lower
                
                # rosnode ping 成功时会显示 "xmlrpc reply from" 或类似的成功信息
                # 失败时会显示 "ERROR" 或 "connection refused"
                if not has_error and (has_success or result.returncode == 0):
                    # ping 成功，节点真正在线
                    rospy.loginfo("nodelet_manager 节点正常运行")
                    return True
                else:
                    # ping 失败，节点在 master 中注册但无法通信（可能已崩溃）
                    error_msg = output.strip() if output.strip() else "未知错误"
                    if attempt < max_retries - 1:
                        # 不是最后一次尝试，稍等再试（可能是节点刚启动）
                        rospy.logwarn(f"节点 nodelet_manager ping 失败（尝试 {attempt + 1}/{max_retries}），稍后重试: {error_msg}")
                        time.sleep(0.5)
                    else:
                        # 最后一次尝试也失败
                        rospy.logwarn(f"节点 nodelet_manager 无法通信（可能已崩溃）: {error_msg}")
                        return False
                        
            except subprocess.TimeoutExpired:
                if attempt < max_retries - 1:
                    rospy.logwarn(f"检查 nodelet_manager 节点状态超时（尝试 {attempt + 1}/{max_retries}），稍后重试")
                    time.sleep(0.5)
                else:
                    rospy.logwarn("检查 nodelet_manager 节点状态超时（最终失败）")
                    return False
            except Exception as e:
                rospy.logwarn(f"检查 nodelet_manager 节点状态时出错: {e}")
                return False
        
        return False

    def handle_interrupt(self, req):
        with self._action_transition_lock:
            try:
                return self._handle_interrupt_locked(req)
            except Exception as e:
                rospy.logerr("Failed to interrupt/reset arm action: %s", e)
                self.stop_action()
                self._stop_trajectory_publisher()
                if self.running_action:
                    self._finish_action(0)
                return TriggerResponse(success=False, message=f"中断动作失败: {e}")

    def _handle_interrupt_locked(self, req):
        """处理中断请求的服务回调"""
        rospy.loginfo("[%s]  接收到机械臂中断指令", rospy.get_time())  

        had_active_action = self.arm_flag or self.running_action
        if not had_active_action:
            return TriggerResponse(success=True, message="当前没有正在执行的手臂动作")

        self.interrupt_flag = True
        # 停止等待动作的timer
        self.stop_action()

        if not self._stop_trajectory_publisher():
            self._finish_action(0)
            return TriggerResponse(success=False, message="动作发布线程停止超时")

        # 恢复机器人初始状态
        # 中断请求处理成功不代表原动作执行成功；复位完成后应
        # 将原动作的唯一终态上报为失败/取消。
        self._pending_terminal_state = 0
        self.interrupt_flag = False
        self.reset_robot_state()
        
        # 返回标准Trigger响应 
        return TriggerResponse(
            success=True,
            message=f"动作于{time.strftime('%Y-%m-%d  %H:%M:%S')}成功中断"
        )

    def _enable_control_callback(self, msg):
        """软暂停：停 tact 播放管线 + 切手臂回自动摆臂；不解冻复位手/头/腰。
        不跟 C++ /bezier/stop_plan_arm_trajectory 交互——C++ timer 照跑，
        next action 的 planCallback 会调 C++ reset() 全量重置。"""
        prev = self.enable_control_state_
        self.enable_control_state_ = bool(msg.data)
        if prev and not self.enable_control_state_:
            # Stop producers before waiting for the lifecycle transition lock;
            # execute may currently be blocked in a synchronous planner call.
            self.arm_flag = False
            if self._trajectory_stop_event is not None:
                self._trajectory_stop_event.set()
            if self._action_state_stop_event is not None:
                self._action_state_stop_event.set()
            # 软暂停是取消而非正常完成：停发布、不复位手/头/腰，
            # 但最终状态必须是 0，不能让上层误判为动作成功。
            self._freeze_tact_pipeline(
                reason="enable_control=false",
                terminal_state=0,
            )
            # freeze 额外：把手臂控制权交回自动摆臂（freeze 服务本身不切 mode）
            try:
                self.call_enable_wbc_arm_trajectory_control_service(0)
                # 软暂停过渡窗口内 mode 切换会被 RM 拒绝（3791 守卫），如实记录结果，
                # 不再无条件打印 "switched"
                if self.call_change_arm_ctrl_mode_service(1):
                    rospy.loginfo("[%s] arm mode switched to 1 (auto swing)", rospy.get_time())
                else:
                    rospy.logwarn("[%s] arm mode switch to 1 (auto swing) rejected, arm keeps current mode", rospy.get_time())
            except Exception as e:
                rospy.logwarn("Failed to switch arm mode: %s", e)

    def _controller_switch_event_callback(self, msg):
        """Hard-abort a MoRE action when a safety switch actually leaves MoRE."""
        if (msg.from_controller != "more_controller" or
                msg.to_controller == "more_controller"):
            return

        self._request_more_action_hard_abort(
            "controller switched %s -> %s" % (
                msg.from_controller,
                msg.to_controller,
            )
        )

    def _more_action_abort_callback(self, _msg):
        """Stop the producer when C++ cancels a MoRE session for safety."""
        self._request_more_action_hard_abort("MoRE safety exit requested")

    def _request_more_action_hard_abort(self, reason):
        """Idempotently stop a MoRE trajectory and publish terminal ABORTED."""

        with self._action_state_lock:
            preparing_more_action = self._preparing_more_action
            owns_more_action = (
                self.running_action and
                self._active_action_controller == "more_controller"
            )
            if preparing_more_action or owns_more_action:
                self._pending_terminal_state = 0
                self._controller_switch_abort_event.set()
                if self._action_state_stop_event is not None:
                    self._action_state_stop_event.set()
        if not (preparing_more_action or owns_more_action):
            return

        # 先用线程安全事件阻止/停止发布；即使 execute callback 正在等待
        # planner 并持有 transition lock，也不会在返回后启动新的发布线程。
        self.interrupt_flag = True
        self.arm_flag = False
        trajectory_stop_event = self._trajectory_stop_event
        if trajectory_stop_event is not None:
            trajectory_stop_event.set()

        if owns_more_action:
            # MoRE owns neither the shared Phase2 path nor a generic mode
            # transition here, so its state machine can commit ABORTED without
            # waiting for a planner/ServiceProxy that may be blocked while
            # holding _action_transition_lock.  Stop/join the publisher first
            # to preserve the terminal-after-last-frame contract.
            self._stop_trajectory_publisher()
            self._finish_action(0)

        with self._action_transition_lock:
            rospy.logwarn("MoRE arm action aborted: %s", reason)
            self.stop_action()
            self._stop_trajectory_publisher()
            if self.running_action:
                self._finish_action(0)

    def _freeze_tact_pipeline(self, reason="freeze", terminal_state=None):
        with self._action_transition_lock:
            return self._freeze_tact_pipeline_locked(reason, terminal_state)

    def _freeze_tact_pipeline_locked(self, reason="freeze", terminal_state=None):
        """停 tact 发布管线：interrupt + 清 arm_flag/running + 发布终态 + 停 timer。
        不调用 reset_robot_state()，保持当前手/头/腰姿态。"""
        rospy.loginfo("[%s] freeze tact pipeline (%s)", rospy.get_time(), reason)
        # 仅在动作执行中才改标志；无动作时的 freeze/unfreeze 无副作用
        if not (self.arm_flag or self.running_action):
            return
        self.interrupt_flag = True
        self.stop_action()
        if self._stop_trajectory_publisher():
            final_state = self._pending_terminal_state if terminal_state is None else terminal_state
            self._finish_action(final_state)
        else:
            self._finish_action(0)

    def handle_freeze_arm_traj(self, req):
        """冻结：立即停止发布 /kuavo_arm_traj 且不复位，使手臂停在当前帧。
        与 handle_interrupt 的区别：不调用 reset_robot_state()（不切回 auto、不复位手/头/腰），
        以便配合上层将手臂控制模式置为 keep pose，把 tact 定在当前位置。"""
        with self._action_transition_lock:
            if self._active_action_controller == "more_controller":
                return TriggerResponse(
                    success=False,
                    message="MoRE 动作不支持冻结保持；请使用 /interrupt_arm_traj 取消并复位",
                )
            self._freeze_tact_pipeline_locked(reason="freeze_arm_traj service")
        return TriggerResponse(
            success=True,
            message=f"动作于{time.strftime('%Y-%m-%d  %H:%M:%S')}冻结于当前帧"
        )

    def handle_keep_arm_pose(self, req):
        with self._action_transition_lock:
            if req.data and (self._active_action_controller == "more_controller"
                             or self.get_current_controller_name() == "more_controller"):
                self.keep_arm_pose = False
                message = "MoRE 动作必须在终态交还 mode1，不支持 keep_arm_pose"
                rospy.logwarn(message)
                return SetBoolResponse(success=False, message=message)
            self.keep_arm_pose = req.data
        if self.keep_arm_pose:
            message = "keep_arm_pose enabled: action will keep arm pose at the last tact frame"
        else:
            message = "keep_arm_pose disabled: action will reset arm pose after completion"
        rospy.loginfo(message)
        return SetBoolResponse(success=True, message=message)

    def handle_execute_action(self, req):
        if not self._execute_request_lock.acquire(blocking=False):
            return ExecuteArmActionResponse(
                success=False,
                message="另一个动作请求正在准备或规划中",
            )
        try:
            # Keep planner mutation and lifecycle commit in one transition. The ROS
            # service has no session/cancel field, so abandoning an in-flight call
            # could let an old plan overwrite a newer reset/action.
            with self._action_transition_lock:
                return self._handle_execute_action_locked(req)
        except Exception as e:
            rospy.logerr("Unexpected error while executing arm action: %s", e)
            with self._action_transition_lock:
                with self._action_state_lock:
                    self._preparing_more_action = False
                self.stop_action()
                self._stop_trajectory_publisher()
                if self.running_action:
                    self._finish_action(0)
            return ExecuteArmActionResponse(
                success=False,
                message=f"执行上肢动作时发生异常: {e}",
            )
        finally:
            self._execute_request_lock.release()

    def _handle_execute_action_locked(self, req):
        action_name = req.action_name

        # 软暂停期间拒绝新动作（无 tact 残留入口）
        if not self.enable_control_state_:
            rospy.logwarn("Action '%s' rejected: enable_control is false", action_name)
            return ExecuteArmActionResponse(
                success=False,
                message="软暂停中，拒绝执行新动作",
            )

        # 检查是否有动作正在执行
        if self.arm_flag or self.running_action:
            rospy.logwarn(f"Action '{action_name}' rejected: Another action is already executing")
            return ExecuteArmActionResponse(
                success=False,
                message=f"另一个动作正在执行中，请等待当前动作完成后再试"
            )

        file_path = f"{self.action_files_path}/{action_name}.tact"
        data = self.load_json_file(file_path)
        if not data:
            return ExecuteArmActionResponse(success=False, message=f"Action file {action_name} not found")

        if not self.check_nodelet_manager_alive():
            msg = "话题 /kuavo_arm_traj 的订阅者 nodelet_manager 节点无法通信或不存在。请检查 nodelet_manager 节点是否正常运行。"
            rospy.logerr(msg)
            return ExecuteArmActionResponse(success=False, message=msg)

        robot_type_raw = data.get("robotType", None)
        
        # 验证文件合法性（以抱拳.tact为标准）
        is_valid, validation_error = self.validate_tact_file(data)
        if not is_valid:
            rospy.logerr(f"Tact file validation failed: {validation_error}")
            return ExecuteArmActionResponse(success=False, message=f"文件合法性检查失败: {validation_error}")
        
        if robot_type_raw is None:
            msg = "Action file missing required field: robotType"
            rospy.logerr(msg)
            return ExecuteArmActionResponse(success=False, message=msg)
        try:
            tact_robot_version = int(robot_type_raw)
        except (TypeError, ValueError):
            msg = f"Invalid robotType in action file: {robot_type_raw}"
            rospy.logerr(msg)
            return ExecuteArmActionResponse(success=False, message=msg)

        robot_version_number = self.robot_version.version_number()
        if not is_tact_robot_type_compatible(tact_robot_version, self.robot_version):
            msg = (
                f"Version mismatch: tact {tact_robot_version} is incompatible with robot {robot_version_number} ({self.robot_version.version_name()})"
            )
            rospy.logerr(msg)
            return ExecuteArmActionResponse(success=False, message=msg)

        current_ctrl = self.get_current_controller_name()

        # multi 模式下无法确认当前控制器时，不能猜测为 MPC/其他控制器。
        # 否则可能绕过 MoRE VR 占用门禁，或对 Depth 误切手臂模式。
        if self.kuavo_control_scheme == "multi" and current_ctrl is None:
            msg = "无法确认当前控制器，拒绝执行上肢动作"
            rospy.logwarn("Action '%s' rejected: %s", action_name, msg)
            return ExecuteArmActionResponse(success=False, message=msg)

        # Depth 没有外部手臂控制链。在发布 state 或切换任何模式前
        # 拒绝，保证请求对当前机器人状态零副作用。
        if current_ctrl == "depth_loco_controller":
            msg = "depth_loco_controller 不支持外部手臂动作"
            rospy.logwarn("Action '%s' rejected: %s", action_name, msg)
            return ExecuteArmActionResponse(success=False, message=msg)

        if current_ctrl == "more_controller" and self.keep_arm_pose:
            msg = "MoRE 动作不支持 keep_arm_pose，请先关闭该选项"
            rospy.logwarn("Action '%s' rejected: %s", action_name, msg)
            return ExecuteArmActionResponse(success=False, message=msg)

        # preflight 和控制器路由校验完成后，先确认本地没有旧的
        # timer/发布线程。此时还没有建立 MoRE session，失败不发全局状态。
        if self._timer:
            rospy.loginfo(f"Stopping old timer before executing: {action_name}")
            self.stop_action()
        if not self._stop_trajectory_publisher():
            msg = "旧动作发布线程停止超时"
            return ExecuteArmActionResponse(success=False, message=msg)

        # A safety-abort event belongs to one action session.  Clear the old
        # generation before MoRE prepare, then mark the tiny prepare->heartbeat
        # window so a C++ safety notification cannot be lost there.
        with self._action_state_lock:
            self._controller_switch_abort_event.clear()
            self._preparing_more_action = current_ctrl == "more_controller"

        # 无论之前是什么状态，执行新动作前必须清掉 interrupt_flag。
        self.interrupt_flag = False
        # 重置过渡帧状态，避免上一次播放的状态影响时长重算。
        self._last_inserted_init_kf = 0

        # MoRE 必须先原子校验 style/mode 并保存恢复快照。校验被拒绝
        # 时不发布 /robot_action_state，避免影响已有的 VR 控制。
        if current_ctrl == "more_controller":
            prepared, prepare_message = self.prepare_more_arm_action(action_name)
            if not prepared:
                with self._action_state_lock:
                    self._preparing_more_action = False
                return ExecuteArmActionResponse(success=False, message=prepare_message)
            if self._controller_switch_abort_event.is_set():
                with self._action_state_lock:
                    self._preparing_more_action = False
                msg = "MoRE 安全退出已取消本次上肢动作"
                rospy.logwarn("Action '%s' cancelled after MoRE prepare: %s", action_name, msg)
                return ExecuteArmActionResponse(success=False, message=msg)

        # prepare 成功后立即建立心跳，后续等待、模式切换和规划期间
        # 都持续发布 state=1。
        if not self._start_action_state_heartbeat(current_ctrl):
            safety_aborted = self._controller_switch_abort_event.is_set()
            msg = ("MoRE 安全退出已取消本次上肢动作"
                   if safety_aborted else "另一个动作正在执行中")
            rospy.logwarn("Action '%s' rejected after preflight: %s", action_name, msg)
            # 理论上 busy 前置检查会保证这里成功。若发生并发竞态，
            # MoRE prepare 已建立 session，必须发 0 让 C++ 恢复快照。
            if current_ctrl == "more_controller" and not safety_aborted:
                self.publish_action_state(0)
            return ExecuteArmActionResponse(success=False, message=msg)

        # MoRE 控制器支持行走中播动作，不需要强制停止。
        if current_ctrl != "more_controller":
            if not self.wait_for_stance_before_action(timeout=3.0):
                msg = "机器人停止行走超时，取消上肢动作播放"
                rospy.logwarn(msg)
                self._finish_action(0)
                return ExecuteArmActionResponse(success=False, message=msg)

        if not self.ensure_tact_playback_controller(timeout=3.0):
            current_controller = self.get_current_controller_name()
            msg = (
                f"当前控制器为 {current_controller}，不允许播放上肢动作，"
                "请先切换到 amp_controller"
            )
            rospy.logwarn(msg)
            self._finish_action(0)
            return ExecuteArmActionResponse(success=False, message=msg)

        # MoRE 控制器已自行处理手臂模式切换，跳过 MPC arm mode 检查（避免冗余 ROS service 延迟）
        if current_ctrl != "more_controller":
            if not self.ensure_arm_ctrl_mode(2, timeout=5.0):
                msg = "手臂控制模式未切换到外部控制模式，取消动作播放"
                rospy.logwarn(msg)
                self._finish_action(0)
                return ExecuteArmActionResponse(success=False, message=msg)

        # 获取初始帧时间
        first_value = data.get("first", 0)
        self.START_FRAME_TIME = round(first_value * 0.01, 2)  # 转换为秒，取两位小数
        self.x_shift = self.START_FRAME_TIME  # 动态调整 x_shift

        # 读取动作完成时间
        finish_time = data.get("finish", 0) * 0.01 # 转换为秒
        current_control_mode = (
            "rl" if current_ctrl == "more_controller"
            else self.get_current_control_mode()
        )
        # RL模式和OCS2模式的过渡时间都会在 add_init_frame 中动态添加，这里不需要额外增加时间
        # ocs2 模式的过渡时间将在 add_init_frame 中动态计算并更新
        self.END_FRAME_TIME = finish_time

        # 检查0f处是否有动作帧，如果没有则添加初始站立帧
        # 注意：RL模式下即使没有0f帧，也不需要插入站立帧（会插入当前帧）
        # 注意：半身模式下即使没有0f帧，也不需要插入站立帧（会插入当前姿态帧）
        frames = data["frames"]

        if (self.robot_class == KUAVO
                and current_control_mode in ("ocs2", "rl")):
            if current_control_mode == "rl":
                left_rad, right_rad = self._rl_shoulder_roll
            else:
                left_rad, right_rad = self._mpc_shoulder_roll
            left_deg  = math.degrees(left_rad)
            right_deg = math.degrees(right_rad)
            for frame in frames:
                servos = frame.get("servos", [])
                if len(servos) > 1:
                    servos[1] = servos[1] + left_deg
                if len(servos) > 8:
                    servos[8] = servos[8] + right_deg
            rospy.loginfo(
                "[ShoulderRollOffset] mode=%s, "
                "l=%.4f r=%.4f rad", current_control_mode, left_rad, right_rad
            )

        has_frame_at_0f = any(frame.get("keyframe", -1) == 0 for frame in frames)
        
        
        # 只有在非RL模式下，且没有0f帧时，才插入站立帧
        # 半身模式下会在 add_init_frame 中插入当前姿态帧，所以这里不需要插入站立帧
        if not has_frame_at_0f and current_control_mode == "ocs2":
            # 半身模式下不插入站立帧，因为会在 add_init_frame 中插入当前姿态帧
            if not (self.robot_class == KUAVO and self.only_half_up_body):
                # 创建初始站立帧
                init_stand_frame = self.create_init_stand_frame(frames, is_rl=current_control_mode == "rl")
                frames.insert(0, init_stand_frame)
                rospy.loginfo("0f处没有动作帧，已添加初始站立帧")

        if current_control_mode == "rl":
            frames = self.crop_frames_from_first(frames, first_value)
            self.START_FRAME_TIME = 0
            self.x_shift = 0

        min_kf_before = min((f.get("keyframe", 0) for f in frames), default=0)

        action_data = self.add_init_frame(frames, is_rl=current_control_mode == "rl")

        # 计算 add_init_frame 插入的过渡帧数，通过 ROS param 传给 handler，
        # 用于动态校正桌面端 PROGRESS_OFFSET，确保模型与机器人同步。
        min_kf_after = min((f.get("keyframe", 0) for f in frames), default=0)
        transition_kf = min_kf_after - min_kf_before
        transition_time_ms = transition_kf * 10  # 每 keyframe = 10ms
        rospy.set_param('/arm_traj_transition_time_ms', transition_time_ms)

        # 根据实际计算的过渡时间更新结束时间。
        # add_init_frame 可能插入了过渡帧，必须始终从实际 frames 的最后一帧 keyframe
        # 重新计算 END_FRAME_TIME，否则轨迹时长不足导致播放速度过快。
        # OCS2 模式额外保证不低于 finish_time + 1.0，匹配旧 handler 的 end_frame_time += 1 行为；
        # RL 模式已有自己的过渡帧逻辑，不需要此 padding。
        if frames:
            last_keyframe = max(f.get("keyframe", 0) for f in frames)
            new_end = last_keyframe * 0.01
            if current_control_mode == "ocs2":
                new_end = max(new_end, finish_time + 1.0)
            self.END_FRAME_TIME = new_end
            rospy.loginfo("[END_FRAME_TIME] updated to %.2fs (last_keyframe=%d, finish_time=%.2f, mode=%s)" % (
                self.END_FRAME_TIME, last_keyframe, finish_time, current_control_mode))
        # 注意：RL/AMP 模式同样需要走 filter_data。
        # 297038619 曾为修复 plan#1499（RL tact 首帧错位）在 RL 模式跳过 filter_data，
        # 但跳过后保持段贝塞尔控制点未经平滑（tact 原始 CP 非零），导致手指指令 ±1 跳变，
        # 在 AMP 步态抱拳等动作上表现为大拇指抖动（issue #3231）。
        # 由于 RL 分支已前置 self.START_FRAME_TIME = 0; self.x_shift = 0，
        # filter_data 内的时间平移不会破坏 RL 首帧对齐，故恢复无条件滤波。
        filtered_data = self.filter_data(action_data)
        bezier_request = self.create_bezier_request(filtered_data)

        rospy.loginfo(f"Planning arm trajectory for action: {action_name}...")
        if current_ctrl == "more_controller":
            self._begin_more_bezier_phase()
        success = self.plan_arm_trajectory_bezier_curve_client(bezier_request)
        # self.call_change_arm_ctrl_mode_service(1)
        if success:
            rospy.loginfo("Arm trajectory planned successfully")
            if not self._start_trajectory_publisher():
                msg = "Failed to start arm trajectory publisher"
                rospy.logerr(msg)
                self._finish_action(0)
                return ExecuteArmActionResponse(success=False, message=msg)
            # 使用更新后的 END_FRAME_TIME（包含过渡帧时间）
            self.delayed_publish_action_state(self.END_FRAME_TIME)
            return ExecuteArmActionResponse(success=True, message="Action executed successfully")
        else:
            rospy.logerr("Failed to plan arm trajectory")
            self._finish_action(0)
            return ExecuteArmActionResponse(success=False, message="Failed to execute action")

    def run(self, stop_event):
        rate = rospy.Rate(100)
        if (self._controller_switch_abort_event.is_set() or
                not self.enable_control_state_):
            return
        # 非 MoRE 控制器使能已有 WBC /kuavo_arm_traj 滤波路径（避免 tact 腕部抖动 #2992）。
        # 放在这里确保 arm_joint_trajectory_.pos 已有数据，避免 Phase 2 入口竞态 spike
        # MoRE ArmController 直接订阅专用 /kuavo_action_traj，不需要开启
        # humanoidController 的 MPC/WBC Phase2 共享分支。
        if self._active_action_controller != "more_controller":
            with self._action_transition_lock:
                if stop_event.is_set() or not self.arm_flag:
                    return
                if self.call_enable_wbc_arm_trajectory_control_service(1):
                    self._action_owns_phase2 = True
                else:
                    rospy.logwarn(
                        "Failed to enable shared WBC arm trajectory path; "
                        "continuing with the active controller's arm input path"
                    )
        is_more_action = self._active_action_controller == "more_controller"
        arm_traj_pub = self.kuavo_action_traj_pub if is_more_action else self.kuavo_arm_traj_pub
        waist_traj_pub = self.action_waist_pub if is_more_action else self.control_waist_pub

        while (self.arm_flag and not self.interrupt_flag
               and not self._controller_switch_abort_event.is_set()
               and self.enable_control_state_
               and not stop_event.is_set() and not rospy.is_shutdown()):
            try:
                if is_more_action:
                    with self._more_output_lock:
                        output_snapshot = self._more_output_snapshot
                    if output_snapshot is None:
                        # The planner has not delivered a complete frame for
                        # this phase yet. Publishing old shared state here would
                        # leak the previous main/reset trajectory.
                        rate.sleep()
                        continue
                    (joint_state_msg, hand_state_msg, head_state_msg,
                     waist_state_msg, published_full_target) = output_snapshot
                else:
                    joint_state_msg = self.joint_state
                    hand_state_msg = self.hand_state
                    head_state_msg = self.head_state
                    waist_state_msg = self.waist_state
                    published_full_target = None

                if len(joint_state_msg.position) != 0:
                    arm_traj_pub.publish(joint_state_msg)
                if (len(hand_state_msg.right_hand_position) != 0 or
                        len(hand_state_msg.left_hand_position) != 0):
                    self.control_hand_pub.publish(hand_state_msg)
                if len(head_state_msg.joint_data) != 0:
                    self.control_head_pub.publish(head_state_msg)
                # 发布腰部数据（KUAVO v50+ 或 ROBAN）
                if (self.robot_class == KUAVO and self.has_waist) or self.robot_class == ROBAN:
                    if len(waist_state_msg.data.data) != 0:
                        waist_traj_pub.publish(waist_state_msg)
                if is_more_action and published_full_target is not None:
                    # Assignment of a fresh list is atomic under the CPython
                    # GIL.  After _stop_trajectory_publisher() joins this
                    # thread, this is the complete last frame actually sent by
                    # the action owner and is safe to use as reset start.
                    self._last_more_published_servos_deg = published_full_target
            except Exception as e:
                rospy.logerr(f"Failed to publish arm trajectory: {e}")
            except KeyboardInterrupt:
                break
            rate.sleep()

# 在类定义完成后注册提取函数到注册表
ArmTrajectoryBezierDemo._JOINT_EXTRACTORS = {
    "KUAVO_WHEELED":     ArmTrajectoryBezierDemo._extract_kuavo_wheeled,
    "KUAVO_BIPED":       ArmTrajectoryBezierDemo._extract_kuavo_biped,
    "KUAVO_BIPED_WAIST": ArmTrajectoryBezierDemo._extract_kuavo_biped_waist,
    "ROBAN":             ArmTrajectoryBezierDemo._extract_roban,
}

if __name__ == "__main__":
    demo = ArmTrajectoryBezierDemo()
