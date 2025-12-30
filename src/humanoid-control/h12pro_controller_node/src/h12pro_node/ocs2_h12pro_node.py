#!/usr/bin/env python3
from typing import Optional, Dict, Set, List, Tuple, Any
import enum
import time
import rospy
from sensor_msgs.msg import Joy
from h12pro_controller_node.msg import h12proRemoteControllerChannel
from h12pro_controller_node.msg import UpdateH12CustomizeConfig
from robot_state.robot_state_machine import robot_state_machine, RobotStateMachine, states
from transitions.core import MachineError
from utils.utils import read_json_file
import rospkg
import os
import signal
import sys
from ocs2_msgs.msg import mpc_observation
from kuavo_msgs.msg import sensorsData
from kuavo_msgs.msg import robotHandPosition, robotHeadMotionData
from sensor_msgs.msg import JointState
import math
from humanoid_plan_arm_trajectory.msg import bezierCurveCubicPoint, jointBezierTrajectory, planArmState
from trajectory_msgs.msg import JointTrajectory
from concurrent.futures import ThreadPoolExecutor
import threading

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('h12pro_controller_node')
h12pro_remote_controller_path = os.path.join(pkg_path, "src", "h12pro_node", "h12pro_remote_controller.json")


class Config:
    # Controller ranges
    MINUS_H12_AXIS_RANGE_MAX = -1722
    H12_AXIS_RANGE_MAX = 1722
    H12_AXIS_RANGE_MIN = 282
    H12_AXIS_RANGE = H12_AXIS_RANGE_MAX - H12_AXIS_RANGE_MIN
    H12_AXIS_MID_VALUE = (H12_AXIS_RANGE_MAX + H12_AXIS_RANGE_MIN) // 2
    
    # State configurations
    VALID_STATES = {"ready_stance", "stance", "walk", "trot"}
    TRIGGER_CHANNEL_MAP = {
        "stop": 8,
        # "ready_stance": 5,
        "stance": 9,
        "walk": 6,
        "trot": 7
    }
    
    # Button and axis mappings
    BUTTON_MAPPING = {
        'A': 0, 'B': 1, 'X': 2, 'Y': 3,
        'LB': 4, 'RB': 5, 'BACK': 6, 'START': 7
    }
    
    AXIS_MAPPING = {
        'LEFT_STICK_Y': 0,
        'LEFT_STICK_X': 1,
        'LEFT_LT': 2,
        'RIGHT_STICK_YAW': 3,
        'RIGHT_STICK_Z': 4,
        'RIGHT_RT': 5,
        'LEFT_RIGHT_TRIGGER': 6,
        'FORWARD_BACK_TRIGGER': 7
    }

    CALLBACK_FREQUENCY = 100
    LONG_PRESS_THRESHOLD = 1.0

    SCALE_RIGHT_STICK_Z = 0.2  # 右摇杆上下（上站下蹲）缩放比例
    SCALE_LEFT_STICK_Y = 0.25  # 左摇杆左右（左右平移）缩放比例
    
    @staticmethod
    def get_default_channels() -> List[int]:
        channels = [Config.H12_AXIS_RANGE_MIN] * 12
        channels[:4] = [Config.H12_AXIS_MID_VALUE] * 4
        return channels

class KeyType(enum.Enum):
    BUTTON = "button"
    SWITCH = "switch"
    JOYSTICK = "joystick"

class ButtonState(enum.Enum):
    RELEASE = "RELEASE"
    PRESS = "PRESS"
    LONG_PRESS = "LONG_PRESS"

class ChannelMapping:
    def __init__(self, channel, axis_index=None, button_index=None, is_button=False, reverse=False, trigger_value=None, scale=1.0):
        self.channel = channel
        self.axis_index = axis_index
        self.button_index = button_index
        self.is_button = is_button
        self.reverse = reverse
        self.trigger_value = trigger_value
        self.scale = scale
        self.previous_value = None

    def update(self, channel_value):
        if self.is_button:
            return self._update_button(channel_value)
        else:
            return self._update_axis(channel_value)

    def _update_button(self, channel_value):
        if self.previous_value is None:
            self.previous_value = channel_value
            return False  # 初次不触发

        if channel_value == self.trigger_value and self.previous_value != self.trigger_value:
            self.previous_value = channel_value
            return True  # 只有切换到目标值时才触发一次
        elif channel_value != self.trigger_value:
            self.previous_value = channel_value

        return False  # 其他情况下不触发

    def _update_axis(self, channel_value):

        value = (channel_value - Config.H12_AXIS_MID_VALUE) / (Config.H12_AXIS_RANGE//2)
        if self.reverse:
            value = -value
        value *= self.scale
        return value

    def get_current_state(self, channel_value):
        if self.is_button:
            return 1 if self.update(channel_value) else 0
        else:
            return self.update(channel_value)

class H12ToJoyControllerNode:
    def __init__(self):
        """Initialize joy controller node."""
        self.channel_mapping = self._create_channel_mapping()
        self.joy_msg = Joy(axes=[0.0] * 8, buttons=[0] * 11)
        self.channels_msg: Optional[Tuple[int, ...]] = None
        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
        self.is_stopping = False    # cd按钮下蹲标志位

    @staticmethod
    def _create_channel_mapping() -> Dict[int, ChannelMapping]:
        """Create channel mapping configuration."""
        return {
            1: ChannelMapping(1, axis_index=Config.AXIS_MAPPING['RIGHT_STICK_YAW'], reverse=True),
            2: ChannelMapping(2, axis_index=Config.AXIS_MAPPING['RIGHT_STICK_Z'], reverse=True, scale=Config.SCALE_RIGHT_STICK_Z),
            3: ChannelMapping(3, axis_index=Config.AXIS_MAPPING['LEFT_STICK_X']),
            4: ChannelMapping(4, axis_index=Config.AXIS_MAPPING['LEFT_STICK_Y'], reverse=True, scale=Config.SCALE_LEFT_STICK_Y),
            6: ChannelMapping(6, button_index=Config.BUTTON_MAPPING['START'], 
                            is_button=True, trigger_value=Config.H12_AXIS_RANGE_MAX),
            7: ChannelMapping(7, button_index=Config.BUTTON_MAPPING['Y'], 
                            is_button=True, trigger_value=Config.H12_AXIS_RANGE_MAX),
            8: ChannelMapping(8, button_index=Config.BUTTON_MAPPING['B'], 
                            is_button=True, trigger_value=Config.H12_AXIS_RANGE_MAX),
            9: ChannelMapping(9, button_index=Config.BUTTON_MAPPING['BACK'], 
                            is_button=True, trigger_value=-Config.H12_AXIS_RANGE_MAX),
            10: ChannelMapping(10, button_index=Config.BUTTON_MAPPING['A'], 
                             is_button=True, trigger_value=Config.H12_AXIS_RANGE_MAX),
        }

    def update_channels_msg(self, msg: h12proRemoteControllerChannel) -> None:
        """Update channel message data."""
        self.channels_msg = msg.channels

    def process_channels(self) -> None:
        """Process and publish channel data."""
        if self.channels_msg is None:
            self.joy_pub.publish(self.joy_msg)
            return

        # Reset messages
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 11

        # Process each channel
        for index, channel_value in enumerate(self.channels_msg):
            if mapping := self.channel_mapping.get(index + 1):
                if index + 1 == 2 and self.is_stopping:
                    mapping.scale = 1.0
                if mapping.is_button:
                    self.joy_msg.buttons[mapping.button_index] = mapping.get_current_state(channel_value)
                else:
                    self.joy_msg.axes[mapping.axis_index] = mapping.get_current_state(channel_value)
                if index + 1 == 2 and self.is_stopping:
                    mapping.scale = Config.SCALE_RIGHT_STICK_Z

        self.joy_pub.publish(self.joy_msg)

class H12PROControllerNode:
    """Main controller node for H12PRO remote controller."""
    
    def __init__(self):
        """Initialize H12PRO controller node."""

        self.robot_state_machine = robot_state_machine

        # 此if分支为命令行启动机器人: joystick_type=h12，遥控器使用和服务启动相同的逻辑。
        # manual_h12_init_state为初始状态，其值为none表示当前是用服务启动的机器人。
        # manual_h12_init_state不是none表示是命令行启动的机器人，此时机器人已经启动，需要调整状态机的初始状态为manual_h12_init_state
        self.manual_h12_init_state = rospy.get_param("manual_h12_init_state", "none")
        if self.manual_h12_init_state == "calibrate":
            self.robot_state_machine = RobotStateMachine(
                    states=states,
                    initial="calibrate",
                    send_event=True,
                    auto_transitions=False,
                )
        elif self.manual_h12_init_state == "ready_stance":
            self.robot_state_machine = RobotStateMachine(
                    states=states,
                    initial="ready_stance",
                    send_event=True,
                    auto_transitions=False,
                )
            
        print(f"[H12PROControllerNode]: robot_state_machine init state is {self.robot_state_machine.state}")
        self.h12_to_joy_node = H12ToJoyControllerNode()
        self.key_timestamp: Dict[str, float] = {}
        self._config = self._load_configuration()
        
        # ROS related initialization
        self.current_arm_joint_state = []
        self.plan_arm_is_finished = True
        self.should_pub_arm_joint_state = JointState()
        self.should_pub_hand_position = robotHandPosition()
        self.should_pub_head_motion_data = robotHeadMotionData()
        self.start_way = rospy.get_param("start_way", "auto")
        self.real_robot = rospy.get_param("real_robot", False)
        
        # 添加线程池
        self.executor = ThreadPoolExecutor(max_workers=2)
        self._state_transition_lock = threading.Lock()
        
        self._setup_ros_components()
        
    def _setup_ros_components(self) -> None:
        """Setup ROS subscribers and timers."""
        self.timer = rospy.Timer(
            rospy.Duration(0.1), 
            self._timer_callback
        )
        
        self.channel_subscriber = rospy.Subscriber(
            "/h12pro_channel",
            h12proRemoteControllerChannel,
            self._channel_callback,
            queue_size=1
        )

        self.sensor_data_sub = rospy.Subscriber(
            '/sensors_data_raw', 
            sensorsData, 
            self._sensor_data_callback, 
            queue_size=1, 
            tcp_nodelay=True
        )
        # self.mpc_obs_sub = rospy.Subscriber(
        #     '/humanoid_mpc_observation', 
        #     mpc_observation, 
        #     self._mpc_obs_callback,
        #     queue_size=1
        # )
        self.traj_sub = rospy.Subscriber(
            '/bezier/arm_traj', 
            JointTrajectory, 
            self._traj_callback, 
            queue_size=1, 
            tcp_nodelay=True
        )

        self.kuavo_arm_traj_pub = rospy.Publisher(
            '/kuavo_arm_traj', 
            JointState, 
            queue_size=1, 
            tcp_nodelay=True
        )

        self.plan_arm_state_sub = rospy.Subscriber(
            "/bezier/arm_traj_state",
            planArmState,
            self._plan_arm_state_callback,
            queue_size=1,
            tcp_nodelay=True 
        )
        self.control_hand_pub = rospy.Publisher(
            '/control_robot_hand_position', 
            robotHandPosition, 
            queue_size=1, 
            tcp_nodelay=True
        )
        self.control_head_pub = rospy.Publisher(
            '/robot_head_motion_data', 
            robotHeadMotionData, 
            queue_size=1, 
            tcp_nodelay=True
        )
        self.update_h12_customize_config_sub = rospy.Subscriber(
            "/update_h12_customize_config",
            UpdateH12CustomizeConfig,
            self._update_h12_customize_config_callback,
            queue_size=1
        )
    
    def _update_h12_customize_config_callback(self, msg):
        self.robot_state_machine.update_customize_config()
        
    def publish_arm_joint_state(self):
        if self.plan_arm_is_finished is False and len(self.should_pub_arm_joint_state.position) > 0:
            self.kuavo_arm_traj_pub.publish(self.should_pub_arm_joint_state)
            self.control_hand_pub.publish(self.should_pub_hand_position)
            self.control_head_pub.publish(self.should_pub_head_motion_data)

    def _plan_arm_state_callback(self, msg):
        self.plan_arm_is_finished = msg.is_finished

    def _traj_callback(self, msg):
        if len(msg.points) == 0:
            return
        point = msg.points[0]
        self.should_pub_arm_joint_state.name = [
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
        self.should_pub_arm_joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
        self.should_pub_arm_joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
        self.should_pub_arm_joint_state.effort = [0] * 14

        self.should_pub_hand_position.left_hand_position = [int(math.degrees(pos)) for pos in point.positions[14:20]]
        self.should_pub_hand_position.right_hand_position = [int(math.degrees(pos)) for pos in point.positions[20:26]]

        self.should_pub_head_motion_data.joint_data = [math.degrees(pos) for pos in point.positions[26:]]


    # def _mpc_obs_callback(self, msg):
    #     self.current_arm_joint_state = msg.state.value[24:]
    #     self.current_arm_joint_state = [round(pos, 2) for pos in self.current_arm_joint_state]
    #     self.current_arm_joint_state.extend([0] * 14)
    
    def _sensor_data_callback(self, msg):
        self.current_arm_joint_state = msg.joint_data.joint_q[12:26]
        self.current_arm_joint_state = [round(pos, 2) for pos in self.current_arm_joint_state]
        self.current_arm_joint_state.extend([0] * 14)

    def _load_configuration(self) -> Dict[str, Any]:
        """Load and validate configuration from JSON file.
        
        Returns:
            Dict containing validated configuration.
            
        Raises:
            ConfigError: If configuration is invalid or missing required fields.
        """
        try:
            config = read_json_file(h12pro_remote_controller_path)
            required_fields = [
                "channel_to_key_name",
                "channel_to_key_state",
                "ocs2_robot_state_transition_keycombination",
                "emergency_stop_key_combination"
            ]
            
            # Validate configuration
            for field in required_fields:
                if field not in config:
                    raise ConfigError(f"Missing required field: {field}")
            
            return {
                "channel_to_key_name": config["channel_to_key_name"],
                "channel_to_key_state": config["channel_to_key_state"],
                "state_transitions": config["ocs2_robot_state_transition_keycombination"],
                "emergency_stop_keys": set(config["emergency_stop_key_combination"])
            }
            
        except Exception as e:
            rospy.logerr(f"Failed to load configuration: {e}")
            raise ConfigError(f"Configuration error: {e}")

    def _timer_callback(self, event: rospy.Timer) -> None:
        """Update has_joy_node parameter periodically."""
        self.start_way = rospy.get_param("start_way", "auto")

    def _channel_callback(self, msg: h12proRemoteControllerChannel) -> None:
        """Process incoming channel messages.
        
        Args:
            msg: Channel message containing control data.
        """
        if self.start_way == "manual":
            return
        
        if msg.sbus_state == 0:
            rospy.logwarn("No receive h12pro channel message. Please check device `/dev/usb_remote` exist or not and re-plug the h12pro signal receiver.")
            return
        
        try:
            key_combination = self._process_channels(msg.channels)
            self._handle_state_transitions(key_combination, msg)
        except Exception as e:
            rospy.logerr(f"Error processing channel message: {e}")

    def _process_channels(self, channels: Tuple[int, ...]) -> Set[str]:
        """Process channel data and return key combination.
        
        Args:
            channels: Tuple of channel values.
            
        Returns:
            Set of active key combinations.
        """
        key_combination = set()
        
        for index, channel in enumerate(channels):
            channel_num = str(index + 1)
            if channel_num not in self._config["channel_to_key_name"]:
                continue
                
            key_info = self._config["channel_to_key_name"][channel_num]
            key = key_info["name"]
            type_ = key_info["type"]

            if type_ == KeyType.BUTTON.value:
                if state := self._handle_button(key, channel):
                    key_combination.add(state)
            elif type_ == KeyType.SWITCH.value:
                if state := self._handle_switch(key, channel):
                    key_combination.add(state)

        return key_combination

    def _handle_state_transitions(self, key_combination: Set[str], 
                                msg: h12proRemoteControllerChannel) -> None:
        """Handle state transitions based on key combinations.
        
        Args:
            key_combination: Set of active key combinations.
            msg: Original channel message.
        """
        current_state = self.robot_state_machine.state
        
        # Handle emergency stop
        if self._config["emergency_stop_keys"].issubset(key_combination):
            self._handle_emergency_stop(current_state, msg)
            return

        # Handle normal state transitions
        self._handle_normal_transitions(current_state, key_combination, msg)

    def _gradually_move_right_stick_down(self, time=0.1, times=20) -> None:
        """Gradually move right stick down to stop robot.
        
        Args:
            time: Time to wait between each step.
            times: Number of times to repeat the process.
        """
        while times > 0:
            stick_channels = Config.get_default_channels()
            stick_channels[:4] = [Config.H12_AXIS_MID_VALUE] * 4
            stick_channels[1] = Config.H12_AXIS_RANGE_MAX
            stick_msg = h12proRemoteControllerChannel()
            stick_msg.channels = tuple(stick_channels)

            self.h12_to_joy_node.update_channels_msg(msg=stick_msg)
            self.h12_to_joy_node.process_channels()
            rospy.sleep(time)
            times -= 1

    def _handle_emergency_stop(self, current_state: str, 
                             msg: h12proRemoteControllerChannel) -> None:
        """Handle emergency stop condition.
        
        Args:
            current_state: Current robot state.
            msg: Channel message for response.
        """
        try:
            if current_state in ["stance", "walk", "trot"]:
                self.h12_to_joy_node.is_stopping = True
                self._gradually_move_right_stick_down()
                self.h12_to_joy_node.is_stopping = False
                
            getattr(self.robot_state_machine, "stop")(source=current_state)
            stop_msg = h12proRemoteControllerChannel()
            channels = Config.get_default_channels()
            channels[Config.TRIGGER_CHANNEL_MAP["stop"]] = Config.MINUS_H12_AXIS_RANGE_MAX
            stop_msg.channels = tuple(channels)
            
            self.h12_to_joy_node.update_channels_msg(msg=stop_msg)
            self.h12_to_joy_node.process_channels()
            return
            
        except MachineError as e:
            return

    def _handle_normal_transitions(self, current_state: str, 
                                 key_combination: Set[str],
                                 msg: h12proRemoteControllerChannel) -> None:
        """Handle normal state transitions.
        
        Args:
            current_state: Current robot state.
            key_combination: Set of active key combinations.
            msg: Channel message for response.
        """
        triggers = self.robot_state_machine.machine.get_triggers(current_state)
        
        for trigger in triggers:
            if trigger == "trot":
                continue            # 跳过 trot 模式的状态转换
            trigger_keys = set(
                self._config["state_transitions"][current_state][trigger]
            )
            
            if not trigger_keys.issubset(key_combination):
                continue
                
            try:
                self._execute_state_transition(trigger, current_state, msg)
                return
            except Exception as e:
                rospy.logerr(f"Error during state transition: {e}")

        # Handle joystick input if no state transition
        if current_state != "vr_remote_control":
            self._handle_joystick_input(msg)

    def _execute_state_transition(self, trigger: str, source: str,
                                msg: h12proRemoteControllerChannel) -> None:
        """Execute a state transition.
        
        Args:
            trigger: Trigger name.
            source: Source state.
            msg: Channel message for response.
        """
        # 准备状态转换参数
        kwargs = {
            "trigger": trigger,
            "source": source,
            "real_robot": self.real_robot
        }
        if "arm_pose" in trigger:
            kwargs["current_arm_joint_state"] = self.current_arm_joint_state
            
        # 提交到线程池执行状态转换
        def state_transition_task():
            with self._state_transition_lock:
                try:
                    getattr(self.robot_state_machine, trigger)(**kwargs)
                    
                    # 如果是有效状态,更新消息
                    if trigger in Config.VALID_STATES:
                        new_msg = h12proRemoteControllerChannel()
                        channels = Config.get_default_channels()
                        channels[Config.TRIGGER_CHANNEL_MAP[trigger]] = Config.H12_AXIS_RANGE_MAX
                        new_msg.channels = tuple(channels)
                        
                        self.h12_to_joy_node.update_channels_msg(msg=new_msg)
                        self.h12_to_joy_node.process_channels()
                except Exception as e:
                    rospy.logerr(f"Error in state transition task: {e}")
                    
        self.executor.submit(state_transition_task)

    def _handle_joystick_input(self, msg: h12proRemoteControllerChannel) -> None:
        """Handle joystick input when no state transition occurs."""
        stick_channels = Config.get_default_channels()
        stick_channels[:4] = msg.channels[:4]
        
        stick_msg = h12proRemoteControllerChannel()
        stick_msg.channels = tuple(stick_channels)
        
        self.h12_to_joy_node.update_channels_msg(msg=stick_msg)
        self.h12_to_joy_node.process_channels()

    def _handle_button(self, key: str, channel: int) -> Optional[str]:
        """Handle button press logic."""
        try:
            state = self._config["channel_to_key_state"][key][str(channel)]
            current_time = time.time()

            if ButtonState.PRESS.value in state:
                return self._handle_button_press(key, current_time)
            elif ButtonState.RELEASE.value in state:
                return self._handle_button_release(key, current_time)
            
            return None
            
        except Exception as e:
            rospy.logwarn(f"Error handling button {key}: {e}")
            return None

    def _handle_button_press(self, key: str, current_time: float) -> Optional[str]:
        """Handle button press state."""
        if key not in self.key_timestamp:
            self.key_timestamp[key] = current_time
            return None
            
        duration = current_time - self.key_timestamp[key]
        if duration > Config.LONG_PRESS_THRESHOLD:
            return f"{key}_LONG_PRESS"
        return None

    def _handle_button_release(self, key: str, current_time: float) -> Optional[str]:
        """Handle button release state."""
        if key not in self.key_timestamp:
            return None
            
        duration = current_time - self.key_timestamp[key]
        del self.key_timestamp[key]
        
        return f"{key}_{'LONG_PRESS' if duration >= Config.LONG_PRESS_THRESHOLD else 'PRESS'}"

    def _handle_switch(self, key: str, channel: int) -> Optional[str]:
        """Handle switch press logic."""
        try:
            return self._config["channel_to_key_state"][key][str(channel)]
        except Exception as e:
            rospy.logwarn(f"Error handling switch {key}: {e}")
            return None

    def __del__(self):
        """Cleanup resources."""
        if hasattr(self, 'executor'):
            self.executor.shutdown(wait=True)

class ConfigError(Exception):
    """Custom exception for configuration errors."""
    pass

def signal_handler(signum, frame):
    """Handle interrupt signals gracefully."""
    rospy.loginfo("Received interrupt signal. Shutting down...")
    rospy.signal_shutdown("Interrupt received")
    sys.exit(0)

def main():
    """Main entry point for the node."""
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    rospy.init_node('joy_node')
    
    try:
        node = H12PROControllerNode()
        rate = rospy.Rate(Config.CALLBACK_FREQUENCY)
        
        rospy.loginfo("H12PRO Controller Node started successfully")
        
        while not rospy.is_shutdown():
            node.h12_to_joy_node.process_channels()
            node.publish_arm_joint_state()
            rate.sleep()
            
    except Exception as e:
        rospy.logerr(f"Error in main loop: {e}")
        raise
    finally:
        rospy.loginfo("Cleaning up...")
        rospy.signal_shutdown("Node shutting down")

if __name__ == '__main__':
    main()