#!/usr/bin/env python3
import rospy
import rospkg
import json
import threading
import subprocess
import time
import os
from typing import Dict, Any, Tuple, Optional
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Twist
from kuavo_msgs.srv import playmusic, playmusicRequest
from kuavo_msgs.srv import ExecuteArmAction, ExecuteArmActionRequest
from std_srvs.srv import Trigger
from humanoid_plan_arm_trajectory.msg import RobotActionState

HUMANOID_ROBOT_SESSION_NAME = "humanoid_robot"
LAUNCH_HUMANOID_ROBOT_SIM_CMD = "roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch start_way:=auto"
LAUNCH_VOICE_CONTROL_REAL_CMD = os.getenv('LAUNCH_VOICE_CONTROL_REAL_CMD', "roslaunch voice_control_node voice_control.launch start_way:=auto")
ROBOT_VERSION = os.getenv('ROBOT_VERSION', "")
ROS_MASTER_URI = os.getenv("ROS_MASTER_URI", "")
ROS_IP = os.getenv("ROS_IP", "")
ROS_HOSTNAME = os.getenv("ROS_HOSTNAME", "")

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

        # Subscribers
        self.joy_sub = rospy.Subscriber("/joy", Joy, self._joy_callback, queue_size=10)
        self.update_sub = rospy.Subscriber("/update_joy_customize_config", String, self._update_config_callback, queue_size=1)
        # 订阅动作执行状态话题，用于检测是否有动作正在执行
        self.robot_action_state_sub = rospy.Subscriber("/robot_action_state", RobotActionState, self._robot_action_state_callback, queue_size=1)
        # 订阅RL控制器状态话题，用于判断当前是否处于RL控制器模式
        self.is_rl_controller_sub = rospy.Subscriber("/humanoid_controller/is_rl_controller_", Float64, self._is_rl_controller_callback, queue_size=1)
        
        # 动作执行状态标志
        self.robot_action_executing = False
        
        # RL控制器状态标志（用于判断是否处于RL控制器模式）
        self._is_rl_controller = False

        # Publishers for robot control (align with C++ behavior)
        self.stop_pub = rospy.Publisher("/stop_robot", Bool, queue_size=10)
        self.re_start_pub = rospy.Publisher("/re_start_robot", Bool, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

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
    
    def _is_rl_controller_callback(self, msg):
        """RL控制器状态回调函数"""
        # data: 0.0=MPC模式, 1.0=RL控制器模式
        # 当data > 0.5时，表示当前处于RL控制器模式
        self._is_rl_controller = (msg.data > 0.5)
    
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

            # 优先级最高：BACK 组合 -> 终止机器人并复位开关
            if 0 <= back_idx < len(joy_msg.buttons):
                if joy_msg.buttons[back_idx]:
                    rospy.logerr("[JoyCustomize] Emergency stop triggered (BACK)")
                    self._gradually_move_right_stick_down()
                    self._call_terminate_srv()
                    self._allow_launch_once = True
                    self._robot_launched = False
                    self._launch_phase = "idle"
                    self._last_launch_status = "unknown"
                    # 更新前一帧，避免被下方逻辑继续处理
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

                        # 如果当前处于RL控制器模式，跳过组合按键动作
                        if self._is_rl_controller:
                            rospy.loginfo(f"Skipping customize action: currently in RL controller mode (is_rl_controller={self._is_rl_controller})")
                            continue

                        # 每次按钮释放时重新加载配置文件
                        self._load_customize_config()

                        # 情况 1: M1 按下、M2 未按下
                        if self._m1_pressed and not self._m2_pressed:
                            action_key = f"customize_action_M1_{button_name}"
                            rospy.loginfo(f"M1 + {button_name} released, triggering {action_key}")
                            self._execute_customize_action(action_key)

                        # 情况 2: M2 按下、M1 未按下
                        elif self._m2_pressed and not self._m1_pressed:
                            action_key = f"customize_action_M2_{button_name}"
                            rospy.loginfo(f"M2 + {button_name} released, triggering {action_key}")
                            self._execute_customize_action(action_key)

                        # 情况 3: M1 和 M2 同时按下
                        elif self._m1_pressed and self._m2_pressed:
                            action_key = f"customize_action_M1M2_{button_name}"
                            rospy.loginfo(f"M1 + M2 + {button_name} released, triggering {action_key}")
                            self._execute_customize_action(action_key)

                        # 情况 4: LT（轴）按下、RT 未按下
                        elif self._lt_pressed and not self._rt_pressed and self.joy_execute_action:
                            action_key = f"customize_action_LT_{button_name}"
                            rospy.loginfo(f"LT + {button_name} released, triggering {action_key}")
                            self._execute_customize_action(action_key)

                        # 情况 5: RT（轴）按下、LT 未按下
                        elif self._rt_pressed and not self._lt_pressed and self.joy_execute_action:
                            action_key = f"customize_action_RT_{button_name}"
                            rospy.loginfo(f"RT + {button_name} released, triggering {action_key}")
                            self._execute_customize_action(action_key)

            # Update previous states
            self._prev_buttons = list(joy_msg.buttons)
            self._prev_axes = list(joy_msg.axes)
            
        except Exception as e:
            rospy.logwarn(f"Error in joy callback: {e}")

    def execute_action_type(self, action_config):
        """处理action类型的自定义动作"""
        arm_pose_names = action_config.get("arm_pose_name", [])
        music_names = action_config.get("music_name", [])
        rospy.loginfo(f"Executing regular action")
        rospy.loginfo(f"Arm poses: {arm_pose_names}")
        rospy.loginfo(f"Music: {music_names}")

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

    def _execute_customize_action(self, action_key: str) -> None:
        """执行自定义动作"""
        try:
            if action_key in self.customize_config:
                action_config = self.customize_config[action_key]
                action_type = action_config.get("type", "")  # 获取type字段

                # 使用字典映射方式调用对应的处理函数
                action_handlers = {
                    "action": lambda: self.execute_action_type(action_config),
                    "shell": lambda: self.execute_shell_type(action_config)
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

    def launch_humanoid_robot(self):
        
        subprocess.run(["tmux", "kill-session", "-t", HUMANOID_ROBOT_SESSION_NAME], 
                        stderr=subprocess.DEVNULL) 
        
        if self.real:
            launch_cmd = f"{LAUNCH_VOICE_CONTROL_REAL_CMD} joystick_type:={self.joystick_type}"
        else:
            launch_cmd = f"{LAUNCH_HUMANOID_ROBOT_SIM_CMD} joystick_type:={self.joystick_type}"

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
