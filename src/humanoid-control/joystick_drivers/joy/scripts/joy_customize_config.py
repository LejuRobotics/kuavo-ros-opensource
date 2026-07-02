#!/usr/bin/env python3
import rospy
import rospkg
import json
import threading
import subprocess
import time
import os
import math
import unicodedata
from typing import Dict, Any, Tuple, Optional
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Twist
from kuavo_msgs.srv import playmusic, playmusicRequest
from kuavo_msgs.srv import ExecuteArmAction, ExecuteArmActionRequest
from kuavo_msgs.srv import SetString, SetStringRequest
from kuavo_msgs.srv import getControllerList
from kuavo_msgs.msg import DanceTrajectoryState
from kuavo_msgs.msg import gaitTimeName
from kuavo_msgs.msg import robotHandPosition
from kuavo_msgs.msg import sensorsData
from std_srvs.srv import Trigger
from humanoid_plan_arm_trajectory.msg import RobotActionState

HUMANOID_ROBOT_SESSION_NAME = "humanoid_robot"
LAUNCH_HUMANOID_ROBOT_SIM_CMD = "roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch start_way:=auto"
LAUNCH_VOICE_CONTROL_REAL_CMD = os.getenv('LAUNCH_VOICE_CONTROL_REAL_CMD', "roslaunch voice_control_node voice_control.launch start_way:=auto")
ROBOT_VERSION = os.getenv('ROBOT_VERSION', "")
ROS_MASTER_URI = os.getenv("ROS_MASTER_URI", "")
ROS_IP = os.getenv("ROS_IP", "")
ROS_HOSTNAME = os.getenv("ROS_HOSTNAME", "")

# ws 导入接口维护的 展示名(中文)-> 英文控制器名 映射, 切换舞蹈时查它。
DANCE_NAME_MAP_PATH = os.path.expanduser(
    "~/.config/lejuconfig/creator_dance_upload/dance_name_map.json")


def normalize_stem(stem: str) -> str:
    """与导入脚本 / ws handler 保持一致的展示名归一化(NFC + strip)。"""
    return unicodedata.normalize("NFC", stem or "").strip()

try:
    # 使用 rospack.get_path 获取 joy 包的路径，正确处理 deb 包安装到 /opt/ros 的情况
    rospack = rospkg.RosPack()
    joy_pkg_path = rospack.get_path("joy")

    print(f"joy_pkg_path: {joy_pkg_path}")
    # 对于 deb 包安装：/opt/ros/<distro>/share/joy -> /opt/ros/<distro>
    if '/opt/ros' in joy_pkg_path:
        parts = joy_pkg_path.split('/')
        # 检查路径格式：/opt/ros/<distro>/share/joy
        if parts[1] == 'opt' and parts[2] == 'ros':
            KUAVO_ROS_CONTROL_WS_PATH = '/opt/ros/leju'

    else:
        # 对于开发工作空间，从包路径向上查找包含 devel 或 install 的目录
        current_dir = joy_pkg_path
        while current_dir != '/':
            if os.path.exists(os.path.join(current_dir, 'devel')) or \
                os.path.exists(os.path.join(current_dir, 'install')):
                KUAVO_ROS_CONTROL_WS_PATH = current_dir
                break
            parent_dir = os.path.dirname(current_dir)
            if parent_dir == current_dir:
                break
            current_dir = parent_dir
        else:
            KUAVO_ROS_CONTROL_WS_PATH = "/home/lab/kuavo-ros-opensource"
except Exception:
    # 降级方案：使用默认值
    KUAVO_ROS_CONTROL_WS_PATH = "/home/lab/kuavo-ros-opensource"
TAIJI_ACTION_SESSION_NAME = "taiji_action"

class JoyCustomizeConfigNode:
    def __init__(self) -> None:
        rospy.init_node("joy_customize_config")

        # 打印KUAVO_ROS_CONTROL_WS_PATH
        rospy.loginfo(f"KUAVO_ROS_CONTROL_WS_PATH: {KUAVO_ROS_CONTROL_WS_PATH}")

        # Params
        self.joystick_type = rospy.get_param("/joystick_type", "bt2")
        self.channel_map_path = rospy.get_param("/channel_map_path", "")
        self.joystick_sensitivity = float(rospy.get_param("/joystick_sensitivity", 20))
        self.joy_execute_action = rospy.get_param("/joy_execute_action", True)
        self.real = rospy.get_param("/real", True)
        self.start_way = rospy.get_param("/start_way", "manual")

        # Load customize config.json (within joy package)
        self.customize_config_path = self._resolve_customize_config_path()
        self.customize_config: Dict[str, Any] = {}
        self._load_customize_config()

        # 展示名 -> 英文控制器名 映射(ws 导入接口维护), 与 json 同步 reload
        self.dance_name_map: Dict[str, Any] = {}
        self._load_dance_name_map()

        # Constants for different joystick models (align with C++ node)
        self.JOYSTICK_BUTTON_NUM_BT2PRO = 16  # BEITONG
        self.JOYSTICK_BUTTON_NUM_BT2 = 11     # X-Box

        # Internal state for edge detection
        self._prev_buttons = []
        self._prev_axes = []
        
        # M1/M2 button states for combination detection
        self._m1_pressed = False
        self._m2_pressed = False

        # LT/RT axis states for combination detection
        self._lt_pressed = False
        self._rt_pressed = False
        
        # 用于避免重复打印映射切换日志
        self._last_mapping_type = None
        self._last_mapping_path = None
        # 用于限制映射切换频率（防止频繁切换）
        self._last_mapping_switch_time = 0.0
        self._mapping_switch_cooldown = 0.3  # 0.3秒内不允许再次切换

        # Default expected counts; will be adjusted by autodetect
        if self.joystick_type == "bt2pro":
            self.JOYSTICK_BUTTON_NUM = self.JOYSTICK_BUTTON_NUM_BT2PRO
        elif self.joystick_type == "bt2":
            self.JOYSTICK_BUTTON_NUM = self.JOYSTICK_BUTTON_NUM_BT2
        else:
            # Default to bt2 if joystick_type is unknown
            self.JOYSTICK_BUTTON_NUM = self.JOYSTICK_BUTTON_NUM_BT2
            rospy.logwarn(f"Unknown joystick_type '{self.joystick_type}', defaulting to bt2")
        self.JOYSTICK_AXIS_NUM = 8

        # Resolve default channel_map_path if empty
        if not self.channel_map_path:
            try:
                humanoid_controllers_path = rospkg.RosPack().get_path("humanoid_controllers")
                self.channel_map_path = f"{humanoid_controllers_path}/launch/joy/{self.joystick_type}.json"
            except Exception as e:
                rospy.logwarn(f"Failed to resolve humanoid_controllers path: {e}")

        # Load mappings
        self.joy_button_map: Dict[str, int] = {}
        self.joy_axis_map: Dict[str, int] = {}
        self._load_joy_channel_mapping(self.channel_map_path, verbose=True)  # 首次加载时打印详细信息
        # 初始化时记录当前映射类型
        self._last_mapping_type = self.joystick_type
        self._last_mapping_path = self.channel_map_path
        
        # Try to autodetect joystick type and reload mapping safely
        try:
            self._autodetect_and_set_joystick_type()
        except Exception as e:
            rospy.logwarn(f"Autodetect joystick type failed: {e}")

        rospy.loginfo(f"joy_customize_config started. joystick_type={self.joystick_type}, map={self.channel_map_path}")
        rospy.loginfo(f"customize_config={self.customize_config_path}")

        # One-time launch switch: True -> first START launches robot; False -> START triggers initialize service
        self._allow_launch_once = True
        # Only allow action combos after robot is fully launched (stand up complete)
        self._robot_launched = False
        # Polling config for launch status service
        self._status_poll_interval = float(rospy.get_param("/launch_status_poll_interval", 1.0))
        # Non-blocking launch state machine
        self._launch_phase = "idle"  # idle | waiting_ready | ready | waiting_launched | launched
        self._last_status_check_time = 0.0
        self._last_launch_status = "unknown"

        # LB+RB+X 倒地起身(仅 roban), 纯事件驱动, 与 START 自启动状态机完全解耦。两次按下
        # 跨越一次 launch 重启, 而 C++ joy 节点随主程序一起被杀, 故状态只能由本(常驻)节点持有。
        #   _fall_recovery_armed     : 第一下(带 init_fall_down 重启)后置位, 起身提交/急停后清零;
        #   _fall_recovery_standing_up: 第二下后置位, 驱动按 fall_stand_state_ 自动调两次 trigger;
        #   _standup_step            : 0=未调, 1=已请求"回初始姿态"(FALL_DOWN->READY), 2=已请求"执行起身";
        #   _fall_stand_state        : 订阅控制器阶段事件; 0=FALL_DOWN 1=READY 2=STAND_UP 3=STANDING, -1=未知。
        self._is_roban = ROBOT_VERSION.isdigit() and 10 <= int(ROBOT_VERSION) <= 20
        self._fall_recovery_armed = False
        self._fall_recovery_standing_up = False
        self._standup_step = 0
        self._fall_stand_state = -1
        self._fall_stand_state_time = 0.0   # 最近一次收到阶段事件的墙钟时刻(判断"当前确在倒地")
        # 起身前握拳、起身成功后张开: 起身开始时置 True, 起身成功(切回 MPC)张手后清零
        self._pending_hand_open = False
        # 握拳到位的非阻塞等待: 发握拳指令后等 dexhand/state.position 全 >=90 或超时再起身
        self._dexhand_positions = []        # dexhand/state.position(12 指, 0-100)
        self._hand_close_start_time = 0.0   # 发出握拳指令的墙钟时刻
        self._hands_closed_done = False     # 握拳等待是否已完成(到位或超时), 完成后才起身
        self._hand_close_wait_timeout = float(rospy.get_param("/fall_recovery_hand_close_timeout", 2.0))
        self._hand_close_threshold = int(rospy.get_param("/fall_recovery_hand_close_threshold", 90))
        # 第一下(带参重启)的使能闸: 机器人一旦进入 MPC(正常运行)即屏蔽该组合键, 只有
        # START+BACK 终止后才刷新重新允许。这样不依赖"搬运模式", 任何 MPC 运行态都屏蔽。
        self._fall_recovery_enabled = True

        # Subscribers
        self.joy_sub = rospy.Subscriber("/joy", Joy, self._joy_callback, queue_size=10)
        self.update_sub = rospy.Subscriber("/update_joy_customize_config", String, self._update_config_callback, queue_size=1)
        # 订阅动作执行状态话题，用于检测是否有动作正在执行
        self.robot_action_state_sub = rospy.Subscriber("/robot_action_state", RobotActionState, self._robot_action_state_callback, queue_size=1)
        # 订阅RL控制器状态话题，用于判断当前是否处于RL控制器模式
        self.is_rl_controller_sub = rospy.Subscriber("/humanoid_controller/is_rl_controller_", Float64, self._is_rl_controller_callback, queue_size=1)
        # 倒地起身控制器阶段事件(0/1/2/3), 驱动 LB+RB+X 第二下自动调两次 trigger
        self.fall_stand_state_sub = rospy.Subscriber(
            "/humanoid_controller/FallStandController/fall_stand_state_", Float64,
            self._fall_stand_state_callback, queue_size=1)
        # 灵巧手状态(0-100, 12 指): 起身前等握拳到位
        self.dexhand_state_sub = rospy.Subscriber(
            "dexhand/state", JointState, self._dexhand_state_callback, queue_size=1)
        self.dance_trajectory_state_sub = rospy.Subscriber(
            "/humanoid_controller/dance_trajectory_state",
            DanceTrajectoryState,
            self._dance_trajectory_state_callback,
            queue_size=1,
        )
        # 订阅 MPC 步态调度状态(仅用于日志/兼容; AMP 没有此话题输出)。
        self.gait_state_sub = rospy.Subscriber("humanoid_mpc_gait_time_name", gaitTimeName,
                                               self._gait_state_callback, queue_size=10)

        # 订阅 /cmd_vel, 用 cpp 端最终下发的速度判断"是否在走"。
        # 之所以不用 gait_name: AMP 控制器没有 walk/stance 状态机, 步态判据在 AMP 下失效。
        self._last_cmd_vel: Optional[Twist] = None
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_callback, queue_size=1)

        # 动作执行状态标志
        self.robot_action_executing = False

        # 当前 MPC 步态名(仅 MPC 有效)。用于识别踏步(trot)/行走(walk)等"在走"状态——
        # 踏步时 cmd_vel≈0, 单靠速度死区判不出, 需叠加步态判据(见 _has_motion_intent)。
        self._current_gait_name = "stance"

        # RL控制器状态标志（用于判断是否处于RL控制器模式）
        self._is_rl_controller = False
        self._last_dance_run_ids: Dict[str, int] = {}
        self._pending_dance_music: Optional[Dict[str, Any]] = None
        self._dance_music_pending_timeout = 10.0

        # 当前控制器名缓存（用于按需判定是不是 dance 控制器，避免每帧调服务）
        self._cached_controller_name: Optional[str] = None
        self._cached_controller_time: float = 0.0
        self._controller_cache_ttl: float = 0.3  # 秒

        # 最近一次触发的动作来源（"m1m2" / "ltrt" / "dance" / None）
        # 配合 robot_action_executing 决定是否发布 M1/M2 active 信号给 cpp
        self._last_action_source: Optional[str] = None

        # Publishers for robot control (align with C++ behavior)
        self.stop_pub = rospy.Publisher("/stop_robot", Bool, queue_size=10)
        self.re_start_pub = rospy.Publisher("/re_start_robot", Bool, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # 灵巧手控制(起身前握拳/起身后张开)。uint8[6]，100=握拳, 0=张开
        self.hand_position_pub = rospy.Publisher("/control_robot_hand_position", robotHandPosition, queue_size=1)
        # 告知 cpp 节点 M1/M2 动作正在执行（latched），cpp 据此屏蔽推杆走路。
        # 语义 = robot_action_executing AND 最近一次触发是 M1/M2。
        self.m1m2_action_active_pub = rospy.Publisher(
            "/joy_node/m1m2_action_active", Bool, queue_size=1, latch=True
        )
        self._publish_m1m2_action_active(False)

    def _resolve_customize_config_path(self) -> str:
        try:
            joy_pkg_path = rospkg.RosPack().get_path("joy")
            return f"{joy_pkg_path}/config/customize_config.json"
        except Exception:
            # Fallback to local package location
            try:
                local_pkg_path = rospkg.RosPack().get_path("joy")
                return f"{local_pkg_path}/config/customize_config.json"
            except Exception:
                # Final fallback: relative known repo layout
                return rospy.get_param(
                    "/customize_config_path",
                    f"{KUAVO_ROS_CONTROL_WS_PATH}/src/humanoid-control/joystick_drivers/joy/config/customize_config.json",
                )

    def _convert_button_names(self, button_map: Dict[str, int]) -> Dict[str, int]:
        """Convert standard button names to custom button names"""
        button_name_mapping = {
            "BUTTON_STANCE": "BUTTON_A",
            "BUTTON_TROT": "BUTTON_B", 
            "BUTTON_RL": "BUTTON_X",  # BUTTON_RL maps to BUTTON_X
            "BUTTON_WALK": "BUTTON_Y"
        }
        
        converted_map = {}
        for original_name, idx in button_map.items():
            # Use converted name if mapping exists, otherwise keep original
            converted_name = button_name_mapping.get(original_name, original_name)
            converted_map[converted_name] = idx
            
        return converted_map

    def _load_joy_channel_mapping(self, path: str, verbose: bool = False) -> None:
        try:
            with open(path, "r") as f:
                data = json.load(f)
            button = data.get("JoyButton", {})
            axis = data.get("JoyAxis", {})
            # Convert all values to int just in case
            raw_button_map = {str(k): int(v) for k, v in button.items()}
            self.joy_button_map = self._convert_button_names(raw_button_map)
            self.joy_axis_map = {str(k): int(v) for k, v in axis.items()}
            rospy.set_param("joystick_type", self.joystick_type)
            rospy.set_param("channel_map_path", path)
            # 只在verbose=True时打印详细信息，避免频繁打印
            if verbose:
                rospy.loginfo(f"Loaded joystick mapping from {path}")
                rospy.loginfo(f"Buttons: {self.joy_button_map}")
                rospy.loginfo(f"Axes: {self.joy_axis_map}")
            else:
                rospy.logdebug(f"Loaded joystick mapping from {path}")
        except Exception as e:
            rospy.logwarn(f"Failed to load joystick mapping from {path}: {e}")

    def _load_customize_config(self) -> None:
        try:
            with open(self.customize_config_path, "r") as f:
                self.customize_config = json.load(f)
            rospy.loginfo(f"Loaded customize_config.json with {len(self.customize_config)} entries")
        except Exception as e:
            rospy.logwarn(f"Failed to load customize_config.json: {self.customize_config_path}, error: {e}")
            self.customize_config = {}

    def _load_dance_name_map(self) -> None:
        """加载 展示名 -> 英文控制器名 映射表; 缺失/损坏当空表(不阻断, 切舞时回退原值)。"""
        try:
            with open(DANCE_NAME_MAP_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.dance_name_map = data if isinstance(data, dict) else {}
            rospy.loginfo(f"Loaded dance_name_map.json with {len(self.dance_name_map)} entries")
        except FileNotFoundError:
            self.dance_name_map = {}
        except Exception as e:
            rospy.logwarn(f"Failed to load dance_name_map.json: {DANCE_NAME_MAP_PATH}, error: {e}")
            self.dance_name_map = {}

    def _resolve_dance_controller_name(self, dance_name: str) -> str:
        """把 json 里的(可中文)dance_name 解析成英文控制器名。

        查不到映射时回退用原值直达: 支持用户手动改 json / 出厂舞蹈直接写英文控制器名,
        非法或无效名最终由控制器侧 switchToDanceControllerByName 裁决。
        """
        entry = self.dance_name_map.get(normalize_stem(dance_name))
        if isinstance(entry, dict) and entry.get("controller"):
            return entry["controller"]
        return dance_name

    def _get_device_name(self, dev_path: str) -> str:
        try:
            base = os.path.basename(dev_path)
            sysfs_path = f"/sys/class/input/{base}/device/name"
            with open(sysfs_path, "r") as f:
                name = f.read().strip()
            return name
        except Exception:
            return ""

    def _set_joystick_type_and_reload(self, new_type: str) -> None:
        if not new_type:
            return
        
        # 如果已经是目标类型，直接返回，避免不必要的切换
        if self.joystick_type == new_type:
            return
        
        # 检查冷却时间，防止频繁切换
        now = time.time()
        if now - self._last_mapping_switch_time < self._mapping_switch_cooldown:
            rospy.logdebug(f"Mapping switch cooldown active, ignoring switch to {new_type}")
            return
        
        try:
            humanoid_controllers_path = rospkg.RosPack().get_path("humanoid_controllers")
            new_map_path = f"{humanoid_controllers_path}/launch/joy/{new_type}.json"
        except Exception as e:
            rospy.logwarn(f"Resolve mapping path failed for {new_type}: {e}")
            return

        # Preserve old mapping/state
        old_button_map = dict(self.joy_button_map)
        old_prev_buttons = list(self._prev_buttons) if self._prev_buttons else []

        # Load new mapping
        self.joystick_type = new_type
        self.channel_map_path = new_map_path
        self._load_joy_channel_mapping(new_map_path)

        # Update expected button count
        if new_type == "bt2pro":
            self.JOYSTICK_BUTTON_NUM = self.JOYSTICK_BUTTON_NUM_BT2PRO
        elif new_type == "bt2":
            self.JOYSTICK_BUTTON_NUM = self.JOYSTICK_BUTTON_NUM_BT2

        # Rebuild previous buttons to match new mapping
        if old_prev_buttons:
            self._prev_buttons = self._rebuild_prev_buttons_for_new_map(
                old_prev_buttons, old_button_map, self.joy_button_map, self.JOYSTICK_BUTTON_NUM
            )
        else:
            self._prev_buttons = [0] * self.JOYSTICK_BUTTON_NUM
        self._prev_axes = [0.0] * self.JOYSTICK_AXIS_NUM
        
        # 更新切换时间和记录
        self._last_mapping_switch_time = now
        # 只在映射真正改变时打印，避免重复日志
        if self._last_mapping_type != new_type or self._last_mapping_path != new_map_path:
            # rospy.logwarn(f"Joystick mapping switched to {new_type} with safe state transfer")
            self._last_mapping_type = new_type
            self._last_mapping_path = new_map_path

    def _rebuild_prev_buttons_for_new_map(
        self,
        old_prev_buttons: list,
        old_button_map: Dict[str, int],
        new_button_map: Dict[str, int],
        new_button_count: int,
    ) -> list:
        rebuilt = [0] * new_button_count
        for name, new_idx in new_button_map.items():
            old_idx = old_button_map.get(name, -1)
            if 0 <= old_idx < len(old_prev_buttons) and 0 <= new_idx < new_button_count:
                rebuilt[new_idx] = old_prev_buttons[old_idx]
        return rebuilt

    def _autodetect_and_set_joystick_type(self) -> None:
        dev = rospy.get_param("/joy_node/dev", None)
        if not dev:
            # Try a common default
            dev = "/dev/input/js0"
        name = self._get_device_name(dev)
        if not name:
            return
        if "BEITONG" in name:
            desired = "bt2pro"
        elif "X-Box" in name or "Xbox" in name or "XBOX" in name:
            desired = "bt2"
        else:
            return
        self._set_joystick_type_and_reload(desired)
        # Still update expected count explicitly
        if desired == "bt2pro":
            self.JOYSTICK_BUTTON_NUM = self.JOYSTICK_BUTTON_NUM_BT2PRO
        elif desired == "bt2":
            self.JOYSTICK_BUTTON_NUM = self.JOYSTICK_BUTTON_NUM_BT2

    def _maybe_switch_mapping_by_msg_size(self, joy_msg: Joy) -> bool:
        try:
            btn_len = len(joy_msg.buttons)
            ax_len = len(joy_msg.axes)
            # Only consider expected axis count of 8; if not, just ignore
            if ax_len != self.JOYSTICK_AXIS_NUM:
                return False
            
            # 检查冷却时间，防止频繁切换
            now = time.time()
            if now - self._last_mapping_switch_time < self._mapping_switch_cooldown:
                return False
            
            # If we currently expect bt2pro (16) but receive 11 -> switch to bt2
            if self.JOYSTICK_BUTTON_NUM == self.JOYSTICK_BUTTON_NUM_BT2PRO and btn_len == self.JOYSTICK_BUTTON_NUM_BT2:
                # 检查是否真的需要切换（避免重复切换）
                if self.joystick_type != "bt2":
                    self._set_joystick_type_and_reload("bt2")
                    return True
                return False
            # If we currently expect bt2 (11) but receive 16 -> switch to bt2pro
            if self.JOYSTICK_BUTTON_NUM == self.JOYSTICK_BUTTON_NUM_BT2 and btn_len == self.JOYSTICK_BUTTON_NUM_BT2PRO:
                # 检查是否真的需要切换（避免重复切换）
                if self.joystick_type != "bt2pro":
                    self._set_joystick_type_and_reload("bt2pro")
                    return True
                return False
        except Exception as e:
            rospy.logwarn(f"maybe_switch_mapping_by_msg_size failed: {e}")
        return False

    def _update_config_callback(self, msg: String) -> None:
        rospy.loginfo("Received update_joy_customize_config, reloading customize_config.json ...")
        self._load_customize_config()
        # 映射表与 json 必须同步 reload: 前端先 import(写表)再替换 json 再触发本回调
        self._load_dance_name_map()

    def _execute_arm_poses(self, arm_pose_names, mode_switch_event=None):
        """执行手臂动作的线程函数"""
        for arm_pose in arm_pose_names:
            if arm_pose:  # 检查动作名称不为空
                rospy.loginfo(f"Executing arm pose: {arm_pose}")
                try:
                    success, message = self._call_execute_arm_action(arm_pose)
                    if success:
                        # 等待手臂模式切换完成（检测到动作开始执行）
                        rospy.loginfo(f"Waiting for arm mode switch to complete for action: {arm_pose}")
                        start_wait_time = time.time()
                        timeout = 5.0  # 5秒超时
                        
                        while not self.robot_action_executing and not rospy.is_shutdown():
                            if time.time() - start_wait_time > timeout:
                                rospy.logwarn(f"Timeout waiting for arm mode switch to complete for action: {arm_pose}")
                                # 超时未检测到模式切换完成，不通知音乐线程，音乐将不播放
                                return
                            time.sleep(0.01)
                        
                        if self.robot_action_executing:
                            rospy.loginfo(f"Arm mode switch completed for action: {arm_pose}, action is now executing")
                            # 只有在动作成功执行且模式切换完成后，才通知音乐线程可以开始播放
                            if mode_switch_event and not mode_switch_event.is_set():
                                mode_switch_event.set()
                        else:
                            rospy.logwarn(f"Arm mode switch not detected for action: {arm_pose}, music will not play")
                            # 未检测到模式切换完成，不通知音乐线程，音乐将不播放
                            return
                        
                        time.sleep(1.0)  # 等待动作完成
                    else:
                        rospy.logwarn(f"Failed to execute arm pose {arm_pose}: {message}")
                        # 动作执行失败，不通知音乐线程，音乐将不播放
                        return
                except Exception as e:
                    rospy.logerr(f"Failed to execute arm pose {arm_pose}: {e}")
                    # 发生异常，不通知音乐线程，音乐将不播放
                    return

    def _play_music(self, music_names, mode_switch_event=None):
        """播放音乐的线程函数"""
        # 如果有模式切换事件，等待模式切换完成后再播放音乐
        if mode_switch_event:
            rospy.loginfo("Waiting for arm mode switch to complete before playing music...")
            if not mode_switch_event.wait(timeout=10.0):  # 最多等待10秒
                rospy.logwarn("Timeout waiting for arm mode switch, action may have failed. Music will not play.")
                # 动作执行失败或超时，不播放音乐
                return
        
        # 只有在动作成功执行且模式切换完成后，才播放音乐
        for music in music_names:
            if music:  # 检查音乐名称不为空
                rospy.loginfo(f"Playing music: {music}")
                try:
                    self._set_robot_play_music(music, 100)
                except Exception as e:
                    rospy.logerr(f"Failed to play music {music}: {e}")

    def _normalize_name_list(self, names):
        """将配置中的名称字段统一成过滤空值后的列表。"""
        if names is None:
            return []
        if isinstance(names, str):
            name = names.strip()
            return [name] if name else []
        if isinstance(names, (list, tuple)):
            return [str(name).strip() for name in names if str(name).strip()]
        name = str(names).strip()
        return [name] if name else []

    def _set_robot_play_music(self, music_file_name: str, music_volume: int) -> bool:
        """机器人播放指定文件的音乐"""
        try:
            _robot_music_play_client = rospy.ServiceProxy("/play_music", playmusic)
            request = playmusicRequest()
            request.music_number = music_file_name
            request.volume = music_volume
            response = _robot_music_play_client(request)
            rospy.loginfo(f"Service call /play_music: {response.success_flag}")
            return response.success_flag
        except Exception as e:
            rospy.logerr(f"Service /play_music call failed: {e}")
            return False

    def _robot_action_state_callback(self, msg):
        """动作执行状态回调函数"""
        # state: 0=失败/未执行, 1=执行中/成功
        # 当state为1时，表示有动作正在执行
        self.robot_action_executing = (msg.state == 1)
        # 同步刷新 M1/M2 active 信号给 cpp（reset 阶段会再发 state=1，信号会自然回到 true）
        self._publish_m1m2_action_active(
            self.robot_action_executing and self._last_action_source == "m1m2"
        )

    def _publish_m1m2_action_active(self, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self.m1m2_action_active_pub.publish(msg)

    def _release_m1m2_action_active(self) -> None:
        """兜底解除推杆屏蔽信号。

        m1m2_action_active_pub 是 latch 话题: 抢发 True 后, 若空动作/服务失败/超时导致
        /robot_action_state 不发消息, _robot_action_state_callback(唯一翻回 False 入口)不触发,
        True 会被永久 latch -> cpp 侧摇杆被永久禁用(issue #3303)。
        本方法在动作流程的各结束/失败路径主动回发 False, 保证屏蔽最终必解除。
        """
        # 仅当本次触发源确实是 m1m2 时才需要回发(否则属于 LT/RT 或无操作, 不应误改 m1m2 信号)。
        if self._last_action_source == "m1m2":
            self._publish_m1m2_action_active(False)

    # 允许做自定义动作(tact)的控制器白名单：MPC / AMP 均允许 tact，
    # 但 AMP 走路时(步态非 stance) LT/RT 也会被单独拒(MPC 走路时 LT/RT 不受限,符合 PDF §4)；
    # M1/M2 任何走路状态下一律拒(不分控制器)；
    # 其它(dance_*、fall_stand_controller 等) 一律拒。
    _ACTION_ALLOWED_CONTROLLERS = ("mpc", "amp_controller", "amp_hand_controller")

    def _is_action_allowed_by_controller(self) -> bool:
        """按需查询当前控制器名是否在允许做动作的白名单内（带 TTL 缓存避免每帧调服务）。
        未取到控制器名（如启动早期）按允许处理，避免误拦。"""
        now = time.time()
        if (self._cached_controller_name is None
                or now - self._cached_controller_time >= self._controller_cache_ttl):
            self._cached_controller_name = self._get_current_controller_name() or ""
            self._cached_controller_time = now
        name = self._cached_controller_name or ""
        if not name:
            return True
        return name in self._ACTION_ALLOWED_CONTROLLERS
    
    def _is_rl_controller_callback(self, msg):
        """RL控制器状态回调函数"""
        # data: 0.0=MPC模式, 1.0=RL控制器模式
        # 当data > 0.5时，表示当前处于RL控制器模式
        self._is_rl_controller = (msg.data > 0.5)
        # 进入 MPC(基座控制器)=机器人正常运行 -> 屏蔽倒地起身自启动组合键的"第一下"。
        # 之后只有 START+BACK 终止才会重新使能(见 _joy_callback 急停分支)。
        if not self._is_rl_controller:
            self._fall_recovery_enabled = False
            # 起身成功(FallStand 完成 -> 切回 MPC)即张开灵巧手
            if self._pending_hand_open:
                rospy.loginfo("[JoyCustomize] 起身成功(已回 MPC), 灵巧手张开")
                self._set_dexterous_hand(0)
                self._pending_hand_open = False

    def _fall_stand_state_callback(self, msg):
        """倒地起身控制器阶段: 0=FALL_DOWN 1=READY_FOR_STAND_UP 2=STAND_UP 3=STANDING。
        仅在 FallStand 控制器激活时持续发布(MPC 期间无)。只做简单赋值, 由 joy 帧驱动起身。"""
        self._fall_stand_state = int(round(msg.data))
        self._fall_stand_state_time = time.time()

    def _dexhand_state_callback(self, msg):
        """灵巧手状态 dexhand/state(JointState): position 为 12 指 0-100 readback。"""
        self._dexhand_positions = list(msg.position)

    def _has_dexterous_hand(self) -> bool:
        """末端是否为灵巧手: 读 rosparam /end_effector_type, 除 none/lejuclaw 外一律视为灵巧手。
        进程常驻、param 由 roslaunch 异步设置(时间不定), 故在控制灵巧手前一刻才读, 不在 init 定死。"""
        eef = str(rospy.get_param("/end_effector_type", "qiangnao")).strip().lower()
        return eef not in ("none", "lejuclaw")

    def _gait_state_callback(self, msg: gaitTimeName) -> None:
        """记录当前 MPC 步态名。"""
        if msg.gait_name:
            self._current_gait_name = msg.gait_name

    # 平面运动死区(m/s 与 rad/s 通用阈值); 选 0.05 比 cpp DEAD_ZONE 略宽松,
    # 避免摇杆漂移在临界态把 tact 当走路拒掉。
    _MOTION_DEAD_ZONE = 0.05

    # MPC 下视为"在走"的步态名。stance=站立静止; walk=行走; trot=原地踏步。
    # 踏步时 cmd_vel≈0, 仅靠速度死区判不出, 需用步态名补判(见 _has_motion_intent)。
    # AMP 无 walk/stance 状态机, 该话题不下发, _current_gait_name 恒为 "stance", 不影响 AMP。
    _WALKING_GAIT_NAMES = ("walk", "trot")

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._last_cmd_vel = msg

    def _has_motion_intent(self) -> bool:
        """判断机器人是否处于"在走"状态, 用于走路时禁触发 M1/M2(及 AMP 下禁 LT/RT)。

        判据 = 速度死区 OR 步态名:
          - /cmd_vel 的 linear.x/y 或 angular.z 任一超死区(AMP/MPC 通用, 蹲起 linear.z 不算);
          - 或 MPC 步态名为 walk/trot(覆盖原地踏步: cmd_vel≈0 但机器人在踏步, 见 #3305)。
        """
        v = self._last_cmd_vel
        if v is not None and (abs(v.linear.x) > self._MOTION_DEAD_ZONE
                              or abs(v.linear.y) > self._MOTION_DEAD_ZONE
                              or abs(v.angular.z) > self._MOTION_DEAD_ZONE):
            return True
        return self._current_gait_name in self._WALKING_GAIT_NAMES

    def _motion_cmd_vel_str(self) -> str:
        """用于日志: 简明打印当前 /cmd_vel 的平面分量。"""
        v = self._last_cmd_vel
        if v is None:
            return "n/a"
        return (f"lx={v.linear.x:.3f},ly={v.linear.y:.3f},"
                f"wz={v.angular.z:.3f}")

    def _prepare_dance_music_pending(self, dance_name: str, music_names) -> None:
        normalized_music_names = self._normalize_name_list(music_names)
        self._pending_dance_music = {
            "dance_name": dance_name,
            "music_names": normalized_music_names,
            "request_time": rospy.Time.now(),
            "previous_run_id": int(self._last_dance_run_ids.get(dance_name, 0)),
            "played": False,
        }
        rospy.loginfo(
            f"[JoyCustomize] Pending dance music: {dance_name} "
            f"(previous_run_id={self._pending_dance_music['previous_run_id']}, "
            f"music={normalized_music_names})"
        )

    def _clear_dance_music_pending(self) -> None:
        self._pending_dance_music = None

    def _dance_trajectory_state_callback(self, msg: DanceTrajectoryState) -> None:
        self._last_dance_run_ids[msg.dance_name] = int(msg.run_id)

        pending = self._pending_dance_music
        if not pending or pending.get("played", False):
            return

        if (rospy.Time.now() - pending["request_time"]).to_sec() > self._dance_music_pending_timeout:
            rospy.logwarn(f"[JoyCustomize] Dance music pending timeout: {pending['dance_name']}")
            self._clear_dance_music_pending()
            return

        if msg.dance_name != pending["dance_name"]:
            return
        if msg.state not in ("started", "running"):
            return

        new_run = int(msg.run_id) > int(pending["previous_run_id"])
        state_after_request = msg.header.stamp >= pending["request_time"]
        if not new_run or not state_after_request:
            return

        pending["played"] = True
        music_names = pending.get("music_names", [])
        if music_names:
            rospy.loginfo(f"[JoyCustomize] Dance trajectory started, playing music: {music_names}")
            music_thread = threading.Thread(target=self._play_music, args=(music_names,))
            music_thread.daemon = True
            music_thread.start()
        else:
            rospy.loginfo(f"[JoyCustomize] Dance trajectory started, no music configured for {msg.dance_name}")
        self._clear_dance_music_pending()

    def _get_current_controller_name(self) -> Optional[str]:
        service_name = "/humanoid_controller/get_controller_list"
        try:
            client = rospy.ServiceProxy(service_name, getControllerList)
            response = client()
            if response.success:
                return response.current_controller
            rospy.logwarn(f"[JoyCustomize] Get current controller failed: {response.message}")
        except Exception as e:
            rospy.logwarn(f"[JoyCustomize] Service call to '{service_name}' failed: {e}")
        return None

    def _restart_dance_controller(self, dance_name: str) -> bool:
        service_name = f"/humanoid_controller/{dance_name}/restart_dance"
        try:
            rospy.wait_for_service(service_name, timeout=1.0)
            client = rospy.ServiceProxy(service_name, Trigger)
            response = client()
            if response.success:
                rospy.loginfo(f"[JoyCustomize] Restarted dance '{dance_name}': {response.message}")
                return True
            rospy.logwarn(f"[JoyCustomize] Restart dance '{dance_name}' failed: {response.message}")
        except Exception as e:
            rospy.logwarn(f"[JoyCustomize] Service call to '{service_name}' failed: {e}")
        return False
    
    def _call_execute_arm_action(self, action_name):
        """调用手臂动作执行服务"""
        # 检查是否有动作正在执行，如果有则不允许触发新的手臂动作
        action_executing = False
        
        # 检查ROS参数标志（用于某些动作执行场景，如太极动作）
        if rospy.has_param("/taiji_executing"):
            action_executing = rospy.get_param("/taiji_executing", False)
        
        # 检查动作状态话题（用于通过/execute_arm_action服务执行的动作）
        if not action_executing:
            action_executing = self.robot_action_executing
        
        if action_executing:
            rospy.logwarn(f"Cannot execute arm action '{action_name}': another action is currently executing")
            return False, "Another action is currently executing"
        
        try:
            _execute_arm_action_client = rospy.ServiceProxy('/execute_arm_action', ExecuteArmAction)
            request = ExecuteArmActionRequest()
            request.action_name = action_name
            response = _execute_arm_action_client(request)
            rospy.loginfo(f"ExecuteArmAction service response: success={response.success}, message={response.message}")
            return response.success, response.message
        except Exception as e:
            rospy.logerr(f"Service call to '/execute_arm_action' failed: {e}")
            return False, f"Service exception: {e}"

    def _joy_callback(self, joy_msg: Joy) -> None:
        # 仅给 roban 使用：ROBOT_VERSION 来自环境变量，是字符串，这里做一次安全解析
        robot_version_str = ROBOT_VERSION or ""
        try:
            robot_version = int(robot_version_str)
        except (TypeError, ValueError):
            rospy.logwarn(f"Invalid ROBOT_VERSION '{robot_version_str}', joy_customize_config disabled for this robot")
            return

        if robot_version < 10 or robot_version > 20:
            return

        # 根据消息尺寸动态切换映射
        if len(joy_msg.buttons) != self.JOYSTICK_BUTTON_NUM or len(joy_msg.axes) != self.JOYSTICK_AXIS_NUM:
            switched = self._maybe_switch_mapping_by_msg_size(joy_msg)

            if not switched:
                # 降低日志级别，避免频繁打印无效消息警告
                rospy.logdebug(f"Invalid joy msg. Buttons: {len(joy_msg.buttons)}, Axes: {len(joy_msg.axes)}")
                return

        # Initialize previous states on first message
        if not self._prev_buttons:
            self._prev_buttons = list(joy_msg.buttons)
        if not self._prev_axes:
            self._prev_axes = list(joy_msg.axes)

        try:
            # 检查M1和M2按键状态
            m1_idx = self.joy_button_map.get("BUTTON_M1", -1)
            m2_idx = self.joy_button_map.get("BUTTON_M2", -1)
            a_idx = self.joy_button_map.get("BUTTON_A", -1)
            b_idx = self.joy_button_map.get("BUTTON_B", -1)
            x_idx = self.joy_button_map.get("BUTTON_X", -1)
            y_idx = self.joy_button_map.get("BUTTON_Y", -1)
            start_idx = self.joy_button_map.get("BUTTON_START", -1)
            back_idx = self.joy_button_map.get("BUTTON_BACK", -1)

            # 读取 LT / RT 轴状态（按下阈值：<-0.5）
            lt_idx = self.joy_axis_map.get("AXIS_LEFT_LT", 1)
            rt_idx = self.joy_axis_map.get("AXIS_RIGHT_RT", 1)
            if 0 <= lt_idx < len(joy_msg.axes):
                self._lt_pressed = joy_msg.axes[lt_idx] < -0.5
            if 0 <= rt_idx < len(joy_msg.axes):
                self._rt_pressed = joy_msg.axes[rt_idx] < -0.5

            # 优先级最高：START + BACK 组合 -> 终止机器人并复位开关
            if 0 <= start_idx < len(joy_msg.buttons) and 0 <= back_idx < len(joy_msg.buttons):
                if joy_msg.buttons[start_idx] and joy_msg.buttons[back_idx]:
                    rospy.logerr("[JoyCustomize] Emergency stop triggered (START + BACK)")
                    self._gradually_move_right_stick_down()
                    self._call_terminate_srv()
                    self._allow_launch_once = True
                    self._robot_launched = False
                    self._launch_phase = "idle"
                    self._last_launch_status = "unknown"
                    # 急停同时中止倒地起身流程, 并刷新使能闸: 终止后重新允许 LB+RB+X 第一下
                    self._fall_recovery_armed = False
                    self._fall_recovery_standing_up = False
                    self._standup_step = 0
                    self._pending_hand_open = False
                    self._fall_recovery_enabled = True
                    # 更新前一帧，避免被下方逻辑继续处理
                    self._prev_buttons = list(joy_msg.buttons)
                    self._prev_axes = list(joy_msg.axes)
                    return

            # 次高优先级：LB+RB+X 倒地起身(roban, 纯事件驱动, 与 START 状态机解耦)。
            # armed 期间消费整帧, 屏蔽 START 误按与一切动作组合; 第二下按 fall_stand_state_ 自动起身。
            if self._handle_fall_recovery(joy_msg):
                self._prev_buttons = list(joy_msg.buttons)
                self._prev_axes = list(joy_msg.axes)
                return

            # 检查 START 按键边沿：第一次用于启动，之后用于初始化
            if 0 <= start_idx < len(joy_msg.buttons) and self.real:
                start_current = joy_msg.buttons[start_idx]
                start_prev = self._prev_buttons[start_idx] if start_idx < len(self._prev_buttons) else 0
                if start_prev == 0 and start_current == 1:
                    if self._allow_launch_once and self.start_way == "auto":
                        # 第一阶段：仅当 idle 时触发一次 tmux 启动，之后重复 START 不再重启
                        if self._launch_phase == "idle":
                            rospy.loginfo("[JoyCustomize] START pressed: launching humanoid robot (once)")
                            try:
                                self.launch_humanoid_robot()
                                if not self.real:
                                    self._robot_launched = True
                                    self._launch_phase = "launched"
                                self._launch_phase = "waiting_ready"
                            except Exception as e:
                                rospy.logerr(f"launch_humanoid_robot failed: {e}")
                        else:
                            rospy.loginfo(f"[JoyCustomize] START ignored while waiting readiness, phase={self._launch_phase}")
                        # 返回当前帧，保持非阻塞
                        self._prev_buttons = list(joy_msg.buttons)
                        self._prev_axes = list(joy_msg.axes)
                        return
                    else:
                        # 第二阶段：仅在 ready 状态下触发一次初始化服务
                        if self._launch_phase == "ready":
                            self._call_real_initialize_srv()
                            rospy.loginfo(f"[STATE] phase: {self._launch_phase} -> waiting_launched")
                            self._launch_phase = "waiting_launched"
                            
                        elif self._launch_phase == "idle":
                            self._call_real_initialize_srv()
                            rospy.loginfo(f"[STATE] phase: {self._launch_phase} -> waiting_ready")
                            self._launch_phase = "waiting_ready"
                            # idle状态下调用服务后不直接跳到waiting_launched，而是等待状态更新
                        else:
                            rospy.loginfo(f"[BUTTON] START IGNORED! not in ready state. phase={self._launch_phase}, allow_once={self._allow_launch_once}")
                        self._prev_buttons = list(joy_msg.buttons)
                        self._prev_axes = list(joy_msg.axes)
                        return

            # 非阻塞轮询一次（限频），用于推进 ready -> launched 的流程
            self._check_and_update_launch_status_nonblocking()

            # 在机器人未完全站立前，不允许按键组合动作
            if not self._robot_launched:
                # 如果到了 ready_stance 或 launched，本帧直接返回，保持 STOP 优先级与流程自然推进
                if self._launch_phase in ["waiting_ready", "ready"] and self._last_launch_status == "ready_stance":
                    # 第一次阶段完成：允许下一次 START 触发站立
                    self._allow_launch_once = False
                    self._launch_phase = "ready"
                    self._prev_buttons = list(joy_msg.buttons)
                    self._prev_axes = list(joy_msg.axes)
                    return
                
                # 处理站立失败回退到 ready_stance 的情况（硬件节点站立失败会回到下蹲状态）
                if self._launch_phase == "waiting_launched" and self._last_launch_status == "ready_stance":
                    rospy.logwarn("[JoyCustomize] Stand up failed or interrupted, falling back to ready state. Press START again to retry.")
                    self._launch_phase = "ready"
                    self._prev_buttons = list(joy_msg.buttons)
                    self._prev_axes = list(joy_msg.axes)
                    return
                
                if self._launch_phase in ["waiting_launched", "launched"] and self._last_launch_status == "launched":
                    self._robot_launched = True
                    self._launch_phase = "launched"
                    self._prev_buttons = list(joy_msg.buttons)
                    self._prev_axes = list(joy_msg.axes)
                    return

            # 检查M1按键状态
            if 0 <= m1_idx < len(joy_msg.buttons):
                m1_current = joy_msg.buttons[m1_idx]
                m1_prev = self._prev_buttons[m1_idx] if m1_idx < len(self._prev_buttons) else 0
                
                if m1_prev == 0 and m1_current == 1:
                    # M1按下
                    self._m1_pressed = True
                    rospy.loginfo("M1 button pressed")
                elif m1_prev == 1 and m1_current == 0:
                    # M1释放
                    self._m1_pressed = False
                    rospy.loginfo("M1 button released")

            # 检查M2按键状态
            if 0 <= m2_idx < len(joy_msg.buttons):
                m2_current = joy_msg.buttons[m2_idx]
                m2_prev = self._prev_buttons[m2_idx] if m2_idx < len(self._prev_buttons) else 0
                
                if m2_prev == 0 and m2_current == 1:
                    # M2按下
                    self._m2_pressed = True
                    rospy.loginfo("M2 button pressed")
                elif m2_prev == 1 and m2_current == 0:
                    # M2释放
                    self._m2_pressed = False
                    rospy.loginfo("M2 button released")

            # 检查 A/B/X/Y 按键的释放
            for button_name, button_idx in [("A", a_idx), ("B", b_idx), ("X", x_idx), ("Y", y_idx)]:
                if 0 <= button_idx < len(joy_msg.buttons):
                    button_current = joy_msg.buttons[button_idx]
                    button_prev = self._prev_buttons[button_idx] if button_idx < len(self._prev_buttons) else 0

                    if button_prev == 1 and button_current == 0:

                        # 每次按钮释放时重新加载配置文件
                        self._load_customize_config()

                        # 确定 action_key
                        action_key = None
                        # 情况 1: M1 按下、M2 未按下
                        if self._m1_pressed and not self._m2_pressed:
                            action_key = f"customize_action_M1_{button_name}"
                        # 情况 2: M2 按下、M1 未按下
                        elif self._m2_pressed and not self._m1_pressed:
                            action_key = f"customize_action_M2_{button_name}"
                        # 情况 3: M1 和 M2 同时按下
                        elif self._m1_pressed and self._m2_pressed:
                            action_key = f"customize_action_M1M2_{button_name}"
                        # 情况 4: LT（轴）按下、RT 未按下
                        elif self._lt_pressed and not self._rt_pressed and self.joy_execute_action:
                            action_key = f"customize_action_LT_{button_name}"
                        # 情况 5: RT（轴）按下、LT 未按下
                        elif self._rt_pressed and not self._lt_pressed and self.joy_execute_action:
                            action_key = f"customize_action_RT_{button_name}"

                        if action_key is None:
                            continue

                        is_m1m2 = (action_key.startswith("customize_action_M1") or
                                   action_key.startswith("customize_action_M2"))

                        # 控制器白名单(dance_* / fall_stand_controller 等直接拒)
                        # 调用同时会刷新 self._cached_controller_name, 后面的走路+控制器判定可用最新值
                        if not self._is_action_allowed_by_controller():
                            rospy.loginfo_throttle(
                                1.0,
                                f"Skipping {action_key}: controller "
                                f"'{self._cached_controller_name}' does not allow tact actions "
                                f"(only mpc / amp_controller)")
                            continue

                        # 走路时的精细化拒绝(判据见 _has_motion_intent):
                        #   - M1/M2: 任何控制器都拒(用户可自定义动作幅度过大易摔);
                        #     覆盖原地踏步(trot): cmd_vel≈0 但步态在走, 一样拒(见 #3305)。
                        #   - LT/RT: 仅 AMP 走路时拒; MPC 走路时允许(PDF §4 边走边做)。
                        # 速度死区对 AMP/MPC 通用; 步态名仅 MPC 有效(AMP 无状态机, 恒 stance)。
                        if self._has_motion_intent():
                            if is_m1m2:
                                rospy.loginfo_throttle(
                                    1.0,
                                    f"Skipping {action_key}: robot in motion "
                                    f"(cmd_vel={self._motion_cmd_vel_str()}, "
                                    f"gait={self._current_gait_name}), "
                                    f"M1/M2 disabled while walking/stepping")
                                continue
                            if self._cached_controller_name == "amp_controller":
                                rospy.loginfo_throttle(
                                    1.0,
                                    f"Skipping {action_key}: AMP 走路时禁触发 LT/RT "
                                    f"(cmd_vel={self._motion_cmd_vel_str()} over deadzone); "
                                    f"switch to MPC to walk while acting")
                                continue

                        # 已有动作在执行 → 这次必然被服务端拒绝，提前在 joy 层拒掉，
                        # 不要修改 _last_action_source / 抢发 m1m2_active 信号，
                        # 避免错误把进行中的 LT/RT 当成 M1/M2 而把走路屏蔽掉。
                        if self.robot_action_executing:
                            rospy.loginfo_throttle(
                                1.0,
                                f"Skipping {action_key}: another action is currently executing")
                            continue

                        # 记录触发源给 cpp 用于禁走信号
                        self._last_action_source = "m1m2" if is_m1m2 else "ltrt"
                        # 抢先发布一次，避免 trigger 与首个 state=1 之间的窗口
                        self._publish_m1m2_action_active(is_m1m2)

                        rospy.loginfo(f"{button_name} released, triggering {action_key}")
                        self._execute_customize_action(action_key)

            # Update previous states
            self._prev_buttons = list(joy_msg.buttons)
            self._prev_axes = list(joy_msg.axes)
            
        except Exception as e:
            rospy.logwarn(f"Error in joy callback: {e}")

    def execute_action_type(self, action_config):
        """处理action类型的自定义动作"""
        # 过滤空串: 配置里 [""] 会被误判为"有动作", 进而抢发 m1m2 屏蔽信号却又不调用
        # /execute_arm_action -> /robot_action_state 不发消息 -> 屏蔽信号 latch 卡 True -> 摇杆永久失灵(见 issue #3303)。
        # 这里把 [""] / [None] / 纯空白项视为无动作, 避免空动作触发屏蔽。
        arm_pose_names = [n for n in action_config.get("arm_pose_name", []) if n and str(n).strip()]
        music_names = [n for n in action_config.get("music_name", []) if n and str(n).strip()]
        rospy.loginfo(f"Executing regular action")
        rospy.loginfo(f"Arm poses: {arm_pose_names}")
        rospy.loginfo(f"Music: {music_names}")

        # 无动作也无音乐: 直接返回, 不发屏蔽信号, 避免空动作锁死摇杆。
        # (屏蔽信号已在调用前的 _publish_m1m2_action_active(is_m1m2) 抢发, 此处需主动回 False 兜底解除。)
        if not arm_pose_names and not music_names:
            self._release_m1m2_action_active()
            return

        # 如果同时有动作和音乐，创建事件用于同步（等待模式切换完成后再播放音乐）
        mode_switch_event = None
        if arm_pose_names and music_names:
            mode_switch_event = threading.Event()

        # 创建线程执行动作和音乐
        if arm_pose_names:
            arm_pose_thread = threading.Thread(target=self._execute_arm_poses, args=(arm_pose_names, mode_switch_event))
            arm_pose_thread.start()

        if music_names:
            music_thread = threading.Thread(target=self._play_music, args=(music_names, mode_switch_event))
            music_thread.start()

        # 等待线程完成
        if arm_pose_names:
            arm_pose_thread.join()
        if music_names:
            music_thread.join()

    def execute_shell_type(self, action_config):
        """处理shell类型的自定义动作"""
        rospy.loginfo(f"Executing shell command")
        # 从配置中获取shell命令
        shell_command = action_config.get("command", "")

        if shell_command:
            try:
                # 更安全地解析命令参数，避免索引错误
                command_parts = shell_command.split(" ")
                if len(command_parts) < 2:
                    raise ValueError("Invalid command format")

                ACTION_SESSION_NAME = command_parts[1].split("/")[-1].split(".")[0]

                subprocess.run(["tmux", "kill-session", "-t", ACTION_SESSION_NAME],
                               stderr=subprocess.DEVNULL)

                print(f"script_cmd: {shell_command}")
                print(f"If you want to check the session, please run 'tmux attach -t {ACTION_SESSION_NAME}'")
                # 仅导出存在的ROS相关环境变量，避免覆盖为空
                export_lines = [
                    f"export ROBOT_VERSION={ROBOT_VERSION}" if ROBOT_VERSION else "",
                    f"export ROS_MASTER_URI={ROS_MASTER_URI}" if ROS_MASTER_URI else "",
                    f"export ROS_IP={ROS_IP}" if ROS_IP else "",
                    f"export ROS_HOSTNAME={ROS_HOSTNAME}" if ROS_HOSTNAME else "",
                ]
                export_lines = [line for line in export_lines if line]

                session_cmd = " && ".join([
                    "source ~/.bashrc",
                    f"source {KUAVO_ROS_CONTROL_WS_PATH}/devel/setup.bash",
                    *export_lines,
                    shell_command,
                ]) + "; exec bash"

                tmux_cmd = [
                    "tmux", "new-session",
                    "-s", ACTION_SESSION_NAME,
                    "-d",
                    session_cmd
                ]

                subprocess.Popen(
                    tmux_cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )

                rospy.sleep(5.0)

                result = subprocess.run(["tmux", "has-session", "-t", ACTION_SESSION_NAME],
                                        capture_output=True)
                if result.returncode == 0:
                    print(f"Started {ACTION_SESSION_NAME} in tmux session")
                else:
                    print(f"Failed to start {ACTION_SESSION_NAME}")
                    raise Exception(f"Failed to start {ACTION_SESSION_NAME}")

            except Exception as e:
                rospy.logerr(f"Error executing shell command '{shell_command}': {e}")
                raise
        else:
            rospy.logwarn(f"No shell_command found for action")

    def execute_dance_type(self, action_config):
        """处理dance类型的自定义动作，切换到指定舞蹈控制器"""
        dance_name = action_config.get("dance_name", "")
        if not dance_name:
            rospy.logwarn("No dance_name specified for dance action")
            return
        # json 里的 dance_name 可能是中文展示名, 切换/重启/比较/run_id 都必须用映射后的
        # 英文控制器名(控制器自身发布的 dance_trajectory_state.dance_name 也是英文名)。
        controller_name = self._resolve_dance_controller_name(dance_name)
        # 动作执行期间禁止切换舞蹈控制器，避免与正在播放的手臂/tact 动作冲突
        if self.robot_action_executing:
            rospy.logwarn(f"Skipping dance switch to '{dance_name}' ({controller_name}): another action is currently executing")
            return
        music_names = action_config.get("music_name", [])
        service_name = "/humanoid_controller/switch_to_dance_controller"
        current_controller = self._get_current_controller_name()
        self._prepare_dance_music_pending(controller_name, music_names)
        if current_controller == controller_name:
            if not self._restart_dance_controller(controller_name):
                self._clear_dance_music_pending()
            return
        try:
            rospy.wait_for_service(service_name, timeout=1.0)
            switch_client = rospy.ServiceProxy(service_name, SetString)
            req = SetStringRequest()
            req.data = controller_name
            response = switch_client(req)
            if response.success:
                rospy.loginfo(f"[JoyCustomize] Dance switch success: {response.message}")
            else:
                rospy.logwarn(f"[JoyCustomize] Dance switch failed: {response.message}")
                self._clear_dance_music_pending()
        except rospy.ServiceException as e:
            rospy.logerr(f"[JoyCustomize] Service call to '{service_name}' failed: {e}")
            self._clear_dance_music_pending()
        except rospy.ROSException as e:
            rospy.logerr(f"[JoyCustomize] Service '{service_name}' not available: {e}")
            self._clear_dance_music_pending()

    def _execute_customize_action(self, action_key: str) -> None:
        """执行自定义动作"""
        try:
            if action_key in self.customize_config:
                action_config = self.customize_config[action_key]
                action_type = action_config.get("type", "")  # 获取type字段

                # 使用字典映射方式调用对应的处理函数
                action_handlers = {
                    "action": lambda: self.execute_action_type(action_config),
                    "shell": lambda: self.execute_shell_type(action_config),
                    "dance": lambda: self.execute_dance_type(action_config)
                }
                
                # 获取并调用对应的处理函数
                handler = action_handlers.get(action_type)
                if handler:
                    handler()
                else:
                    rospy.logwarn(f"Unsupported action type: {action_type}")

            else:
                rospy.logwarn(f"No configuration found for action: {action_key}")
                
        except Exception as e:
            rospy.logerr(f"Error executing customize action {action_key}: {e}")


    def _gradually_move_right_stick_down(self, time=0.1, times=10) -> None:
        while times > 0:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.z = -0.2
            self.cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(time)
            times -= 1

    def _start_humanoid_robot(self):
        # 如果按下 start 就运行launch_humanoid_robot 函数
        # 假设 "start" 是自定义配置中的一个按钮名
        if "start" in self.joy_button_map:
            start_btn_idx = self.joy_button_map["start"]
            # 获取当前按钮状态
            joy_msg = rospy.wait_for_message("/joy", Joy, timeout=1.0)
            if len(joy_msg.buttons) > start_btn_idx and joy_msg.buttons[start_btn_idx]:
                self.launch_humanoid_robot()

    def _call_real_initialize_srv(self) -> None:

        try:
            client = rospy.ServiceProxy("/humanoid_controller/real_initial_start", Trigger)
            resp = client()
            rospy.loginfo(f"[SERVICE] real_initial_start RESPONSE: success={resp.success}, message='{resp.message}'")
            if not resp.success:
                rospy.logerr(f"[SERVICE] real_initial_start FAILED: {resp.message}")
                raise RuntimeError(f"real_initial_start returned false: {resp.message}")
        except Exception as e:
            # rospy.logerr(f"[SERVICE] real_initial_start EXCEPTION: {e}")
            # rospy.logwarn(f"[SERVICE] Trying fallback: publish /re_start_robot topic")
            try:
                msg = Bool()
                msg.data = True
                for i in range(3):
                    self.re_start_pub.publish(msg)
                    rospy.loginfo(f"[SERVICE] Published /re_start_robot ({i+1}/3)")
                    rospy.sleep(0.05)
                rospy.loginfo(f"[SERVICE] Fallback completed")
            except Exception as pub_e:
                rospy.logerr(f"[SERVICE] Fallback also failed: {pub_e}")

    def _call_terminate_srv(self) -> None:
        try:
            msg = Bool()
            msg.data = True

            for _ in range(5):
                self.stop_pub.publish(msg)
                rospy.sleep(0.1)
        except Exception as e:
            rospy.logerr(f"[JoyCustomize] publish /stop_robot failed: {e}")

    def _call_trigger_fall_stand_up_srv(self) -> bool:
        """调用一次 /humanoid_controller/trigger_fall_stand_up, 仅记录日志。
        进度/时机完全由订阅的 fall_stand_state_ 事件驱动(见 _drive_stand_up),
        本函数不解析响应。服务不可达返回 False。"""
        try:
            client = rospy.ServiceProxy("/humanoid_controller/trigger_fall_stand_up", Trigger)
            resp = client()
            rospy.loginfo(f"[SERVICE] trigger_fall_stand_up: success={resp.success}, message='{resp.message}'")
            return True
        except Exception as e:
            rospy.logwarn(f"[SERVICE] trigger_fall_stand_up 暂不可用(控制器未就绪?): {e}")
            return False

    def _set_dexterous_hand(self, value: int) -> None:
        """控制双手灵巧手(6 指): value=100 握拳(闭合), value=0 张开。"""
        msg = robotHandPosition()
        msg.header.stamp = rospy.Time.now()
        msg.left_hand_position = [value] * 6
        msg.right_hand_position = [value] * 6
        self.hand_position_pub.publish(msg)

    def _begin_stand_up(self) -> None:
        """开始起身。若有灵巧手: 先发握拳(全 100), 起身插值/轨迹要等握拳到位(全>=阈值)或超时
        再开始; 起身成功后张开。若无灵巧手(从未收到 dexhand/state): 跳过握拳, 不浪费等待。"""
        self._fall_recovery_standing_up = True
        self._standup_step = 0
        if not self._has_dexterous_hand():
            rospy.loginfo("[JoyCustomize] 末端非灵巧手(none/lejuclaw), 跳过握拳直接起身")
            self._hands_closed_done = True
            self._pending_hand_open = False
            return
        rospy.loginfo("[JoyCustomize] 起身前: 灵巧手握拳(闭合, 全100), 等到位或超时再起身")
        self._set_dexterous_hand(100)
        self._pending_hand_open = True
        self._hand_close_start_time = time.time()
        self._hands_closed_done = False

    def _hands_closed_ready(self) -> bool:
        """握拳是否到位: dexhand/state.position 全部 >= 阈值, 或超时。返回 True 可开始起身。"""
        elapsed = time.time() - self._hand_close_start_time
        if elapsed >= self._hand_close_wait_timeout:
            rospy.logwarn(f"[JoyCustomize] 握拳等待超时({self._hand_close_wait_timeout:.0f}s), 直接起身")
            return True
        pos = self._dexhand_positions
        if pos and all(p >= self._hand_close_threshold for p in pos):
            rospy.loginfo(f"[JoyCustomize] 灵巧手已握拳到位(全>={self._hand_close_threshold}), 开始起身")
            return True
        return False

    def _drive_stand_up(self) -> None:
        """第二下起身: 完全按控制器阶段事件 fall_stand_state_ 自动驱动两次 trigger。
          state==0 FALL_DOWN  且 step<1 -> trigger(回初始姿态: FALL_DOWN->READY), step=1
          state==1 READY      且 step<2 -> trigger(置 request_for_stand_up_, 待插值完执行起身), step=2
          state in {2,3}                -> 起身已在执行/完成, py 不再干预, 解除待命
        只在 state∈{0,1} 调 trigger(规避 STAND_UP 时调用会 reset 的坑); step 保证每阶段只调一次。
        state==-1(未收到事件, 控制器尚未起来)则等待, 不调用。"""
        # 握拳到位(或超时/无手)之前, 不触发任何起身插值/轨迹
        if not self._hands_closed_done:
            if self._hands_closed_ready():
                self._hands_closed_done = True
            else:
                return
        s = self._fall_stand_state
        if s == 0 and self._standup_step < 1:
            rospy.logwarn("[JoyCustomize] 起身(1/2): FALL_DOWN->回起身初始姿态")
            if self._call_trigger_fall_stand_up_srv():
                self._standup_step = 1
        elif s == 1 and self._standup_step < 2:
            rospy.logwarn("[JoyCustomize] 起身(2/2): 请求执行起身(待插值完成自动起身, 起身后自动回 MPC)")
            if self._call_trigger_fall_stand_up_srv():
                self._standup_step = 2
        elif s in (2, 3):
            # STAND_UP/STANDING: 起身已交给控制器自动完成, py 解除待命
            rospy.loginfo(f"[JoyCustomize] 起身已在执行/完成(state={s}), 结束倒地起身待命")
            self._fall_recovery_armed = False
            self._fall_recovery_standing_up = False
            self._standup_step = 0

    def _launch_status_allows_standup(self) -> bool:
        """real_launch_status 为 ready_stance/launched 时才允许第二下触发起身
        (带 init_fall_down_state 启动后, 机器人先缩腿到 ready_stance 等待 real_initial_start)。
        仿真无此服务直接放行; 服务不可达(控制器尚未起来)视为未就绪。"""
        if not self.real:
            return True
        ok, status = self._query_robot_launch_status()
        if ok and status in ("ready_stance", "launched"):
            return True
        rospy.logwarn(f"[JoyCustomize] 机器人尚未就绪(real_launch_status={status}), 请稍候再按第二下 LB+RB+X")
        return False

    def _is_robot_laid_down(self) -> bool:
        """第二下起身前在「按下那一刻」检测一次姿态: 用 sensors_data_raw 的 IMU quat 算 pitch,
        判断机器人是否已放倒。第一次启动可能是吊着的, 第二下按之前才放倒, 故只在此刻即时读一帧。
        读不到数据/未放倒一律视为误操作, 返回 False。

        ★ 目前仅支持「面朝地的前倒」起身: 前倒时 pitch 为正且接近 +π/2(实测 ~1.489),
          因此只认正 pitch(pitch >= 阈值), 不能用 abs(pitch)。背朝地的后倒(pitch 为负)
          暂不支持, 待后续实机测试再补充对应逻辑。"""
        threshold = float(rospy.get_param("/fall_recovery_fallen_pitch_threshold", 1.35))
        try:
            msg = rospy.wait_for_message("/sensors_data_raw", sensorsData, timeout=1.0)
        except Exception as e:
            rospy.logwarn(f"[JoyCustomize] 读取 IMU(sensors_data_raw)失败, 暂不起身: {e}")
            return False
        q = msg.imu_data.quat
        # pitch(绕 Y): asin(2(wy - zx)), 取值 [-π/2, π/2]
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sinp)
        # 仅前倒(面朝地, pitch 为正): 用正向比较, 不用 abs; 后倒(负 pitch)暂不支持
        laid = pitch >= threshold
        rospy.loginfo(f"[JoyCustomize] 起身前姿态检测: pitch={pitch:.3f} rad, 阈值={threshold:.2f}(仅前倒/正pitch), 放倒={laid}")
        return laid

    def _initiate_stand_up(self) -> bool:
        """第二下起身前置: 必须等 real_launch_status 就绪, 且当下 IMU pitch 确认机器人已放倒,
        才触发 real_initial_start 并进入(握拳)+起身插值/轨迹。real_initial_start 之后不再看状态,
        由 fall_stand_state_ 事件驱动 _drive_stand_up。任一前置不满足返回 False(保持待命, 可再按)。"""
        if not self._launch_status_allows_standup():
            return False
        # 吊起/直立时按下第二下视为误操作: 只有确认放倒才允许起身
        if not self._is_robot_laid_down():
            rospy.logwarn("[JoyCustomize] 机器人未放倒(IMU pitch 未近垂直), 忽略本次起身; 放倒后再按 LB+RB+X")
            return False
        rospy.logwarn("[JoyCustomize] 起身前置: 触发 real_initial_start")
        self._call_real_initialize_srv()
        self._begin_stand_up()
        return True

    def _handle_fall_recovery(self, joy_msg) -> bool:
        """LB+RB+X 倒地起身(仅 roban, 纯事件驱动, 与 START 自启动状态机解耦)。
        需 LB+RB 同时按住、X 上升沿。返回 True 表示本帧已被消费(armed 期间消费整帧,
        屏蔽 START 误按与一切动作组合)。第二下按 fall_stand_state_ 自动调两次 trigger。"""
        if not self._is_roban:
            return False
        lb_idx = self.joy_button_map.get("BUTTON_LB", -1)
        rb_idx = self.joy_button_map.get("BUTTON_RB", -1)
        x_idx = self.joy_button_map.get("BUTTON_X", -1)
        if lb_idx < 0 or rb_idx < 0 or x_idx < 0:
            return False
        n = len(joy_msg.buttons)
        if lb_idx >= n or rb_idx >= n or x_idx >= n:
            return False

        lb_held = joy_msg.buttons[lb_idx]
        rb_held = joy_msg.buttons[rb_idx]
        x_now = joy_msg.buttons[x_idx]
        x_prev = self._prev_buttons[x_idx] if x_idx < len(self._prev_buttons) else 0
        pressed = bool(lb_held and rb_held and x_prev == 0 and x_now == 1)

        if not self._fall_recovery_armed:
            if not pressed:
                return False
            # 机器人当前就在倒地(命令行带 init_fall_down_state 启动, 或其它已处于 FALL_DOWN 的情况):
            # 直接起身, 跳过重启, 也不看 MPC 使能闸。判据=正在持续收到 FALL_DOWN(0) 阶段事件。
            fresh_falldown = (self._fall_stand_state == 0 and
                              (time.time() - self._fall_stand_state_time) < 1.5)
            if fresh_falldown:
                rospy.logwarn("[JoyCustomize] LB+RB+X: 机器人已倒地, 直接起身(real_initial_start->回初始姿态->执行起身), 不重启")
                self._fall_recovery_armed = True
                if self._initiate_stand_up():
                    self._drive_stand_up()
                return True
            # 机器人已进入 MPC(正常运行)则屏蔽第一下; 需先 START+BACK 终止刷新使能。
            if not self._fall_recovery_enabled:
                rospy.logwarn("[JoyCustomize] 机器人运行中(MPC), 忽略 LB+RB+X 倒地起身; 如需使用请先 START+BACK 终止")
                return False
            # 第一下: 带 init_fall_down_state:=true 重启(进入倒地)。不触碰 START 状态机。
            rospy.logwarn("[JoyCustomize] LB+RB+X (1/2): 带 init_fall_down_state:=true 重启机器人, 进入倒地状态")
            try:
                self.launch_humanoid_robot(extra_args="init_fall_down_state:=true")
            except Exception as e:
                rospy.logerr(f"[JoyCustomize] 倒地重启失败, 不进入起身待命态: {e}")
                return True
            self._fall_recovery_armed = True
            self._fall_recovery_standing_up = False
            self._standup_step = 0
            self._fall_stand_state = -1   # 丢弃旧控制器残留状态, 只认重启后的新事件
            return True

        # 已 armed: 第二下先确认 real_launch_status 就绪并触发 real_initial_start, 再起身;
        # 之后每帧按 fall_stand_state_ 事件驱动 trigger。未就绪则保持待命, 可再按。
        if pressed and not self._fall_recovery_standing_up:
            rospy.logwarn("[JoyCustomize] LB+RB+X (2/2): 请求起身(real_initial_start->回初始姿态->执行起身, 按 state 自动两次 trigger)")
            self._initiate_stand_up()
        if self._fall_recovery_standing_up:
            self._drive_stand_up()
        return True

    def launch_humanoid_robot(self, extra_args: str = ""):

        subprocess.run(["tmux", "kill-session", "-t", HUMANOID_ROBOT_SESSION_NAME],
                        stderr=subprocess.DEVNULL)

        if self.real:
            launch_cmd = f"{LAUNCH_VOICE_CONTROL_REAL_CMD} joystick_type:={self.joystick_type}"
        else:
            launch_cmd = f"{LAUNCH_HUMANOID_ROBOT_SIM_CMD} joystick_type:={self.joystick_type}"

        # 透传额外 launch 参数(如 init_fall_down_state:=true)
        if extra_args:
            launch_cmd = f"{launch_cmd} {extra_args}"

        # 普通启动(START)必须显式把 init_fall_down_state 赋值为 false: voice_control.launch
        # 不论是否带参都会被拉起, 若不主动覆盖, 上一次倒地起身(true)的 param 可能残留导致误进倒地。
        if "init_fall_down_state" not in launch_cmd:
            launch_cmd = f"{launch_cmd} init_fall_down_state:=false"

        print(f"launch_cmd: {launch_cmd}")
        print(f"If you want to check the session, please run 'tmux attach -t {HUMANOID_ROBOT_SESSION_NAME}'")

        export_lines = [
            f"export ROBOT_VERSION={ROBOT_VERSION}" if ROBOT_VERSION else "",
            f"export ROS_MASTER_URI={ROS_MASTER_URI}" if ROS_MASTER_URI else "",
            f"export ROS_IP={ROS_IP}" if ROS_IP else "",
            f"export ROS_HOSTNAME={ROS_HOSTNAME}" if ROS_HOSTNAME else "",

        ]
        export_lines = [line for line in export_lines if line]

        session_cmd = " && ".join([
            "source ~/.bashrc",
            f"source {KUAVO_ROS_CONTROL_WS_PATH}/devel/setup.bash",
            *export_lines,
            launch_cmd,
        ]) + "; exec bash"

        tmux_cmd = [
            "sudo", "tmux", "new-session",
            "-s", HUMANOID_ROBOT_SESSION_NAME, 
            "-d",  
            session_cmd
        ]

        process = subprocess.Popen(
            tmux_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        rospy.sleep(5.0)
        
        result = subprocess.run(["tmux", "has-session", "-t", HUMANOID_ROBOT_SESSION_NAME], 
                                capture_output=True)
        if result.returncode == 0:
            print(f"Started {HUMANOID_ROBOT_SESSION_NAME} in tmux session: {HUMANOID_ROBOT_SESSION_NAME}")
        else:
            print(f"Failed to start {HUMANOID_ROBOT_SESSION_NAME}")
            raise Exception(f"Failed to start {HUMANOID_ROBOT_SESSION_NAME}")

    def _query_robot_launch_status(self) -> Tuple[bool, str]:
        """Query /humanoid_controller/real_launch_status service and return (ok, status_message)."""
        try:
            client = rospy.ServiceProxy("/humanoid_controller/real_launch_status", Trigger)
            resp = client()
            # resp.success is always true in provider, message holds the status
            return True, (resp.message or "unknown")
        except Exception as e:
            rospy.logwarn(f"[JoyCustomize] query launch status failed: {e}")
            return False, "unknown"

    def _check_and_update_launch_status_nonblocking(self) -> None:
        """Rate-limited single status check without blocking loops or sleeps."""
        # 仿真模式下不需要查询真实机器人启动状态服务，直接返回以避免刷屏
        if not self.real:
            return

        if self._robot_launched or self._launch_phase == "idle":
            return
        now = time.time()
        if now - self._last_status_check_time < max(0.1, self._status_poll_interval):
            return
        self._last_status_check_time = now
        ok, status = self._query_robot_launch_status()
        if ok:
            old_status = self._last_launch_status
            self._last_launch_status = status
            if old_status != status:
                rospy.loginfo(f"[STATUS] ROBOT STATUS CHANGED: {old_status} -> {status}")
        else:
            # 保持原状态，不更新 last_status
            rospy.logwarn(f"[STATUS] Failed to query robot status")

    def spin(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    try:
        node = JoyCustomizeConfigNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
