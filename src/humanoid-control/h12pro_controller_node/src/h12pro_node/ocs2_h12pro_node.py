#!/usr/bin/env python3
from typing import Optional, Dict, Set, List, Tuple, Any
import enum
import time
import rospy
import rosnode
from sensor_msgs.msg import Joy
from h12pro_controller_node.msg import h12proRemoteControllerChannel
from h12pro_controller_node.msg import UpdateH12CustomizeConfig
from robot_state.robot_state_machine import robot_state_machine, RobotStateMachine, states
from robot_state.multi_before_callback import is_switch_controller_in_cooldown, clear_switch_controller_cooldown, is_sit_stand_in_progress, set_g11_robot_kind, set_g11_boot_force
from transitions.core import MachineError
from utils.utils import read_json_file
import rospkg
import os
import signal
import sys
from ocs2_msgs.msg import mpc_observation
from kuavo_msgs.msg import sensorsData
from kuavo_msgs.msg import robotHandPosition, robotHeadMotionData

# ===== G11 末端(夹爪/灵巧手)跟随桥消息 =====
# 遥控端只表达"使能(CH14 claw 位)+AUX 开度", 末端类型由机器人端
# /end_effector_type 决定, 遥控器不区分。
#   lejuclaw: /leju_claw_command, position 0=闭合/100=全开
#   qiangnao/linker_hand 灵巧手: /dexhand/command, 0=全开/100=全闭(方向相反),
#     6 个手指同比例联动(AUX1 左/AUX2 右), 暂不做单指/手势。
try:
    from kuavo_msgs.msg import lejuClawCommand as _G11LejuClawCmdMsg
    _G11_LEJUCLAW_MSG_OK = True
except Exception as _e:
    _G11LejuClawCmdMsg = None
    _G11_LEJUCLAW_MSG_OK = False
    rospy.logwarn(f"[G11Claw] lejuClawCommand import failed, AUX 夹爪桥禁用: {_e}")

try:
    from kuavo_msgs.msg import dexhandCommand as _G11DexhandCmdMsg
    _G11_DEXHAND_MSG_OK = True
except Exception as _e:
    _G11DexhandCmdMsg = None
    _G11_DEXHAND_MSG_OK = False
    rospy.logwarn(f"[G11Claw] dexhandCommand import failed, AUX 灵巧手桥禁用: {_e}")
from kuavo_msgs.srv import getControllerList
from sensor_msgs.msg import JointState
import math
from humanoid_plan_arm_trajectory.msg import bezierCurveCubicPoint, jointBezierTrajectory, planArmState, RobotActionState
from trajectory_msgs.msg import JointTrajectory
from concurrent.futures import ThreadPoolExecutor
import threading
from std_msgs.msg import Bool, Empty

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('h12pro_controller_node')
h12pro_remote_controller_path = os.path.join(pkg_path, "src", "h12pro_node", "h12pro_remote_controller.json")
kuavo_control_scheme = os.getenv("KUAVO_CONTROL_SCHEME", "ocs2")

# 遥控器型号: 由部署的 systemd 服务注入 REMOTE_CONTROLLER_TYPE 环境变量决定(优先)
#   h12  = H12 / G12 遥控器 (默认: 无 env / json 无字段时保持老逻辑, 不受 G11 影响)
#   g11  = G11 遥控器 (service 部署时 Environment=REMOTE_CONTROLLER_TYPE=g11)
try:
    _rcfg = read_json_file(h12pro_remote_controller_path)
    _controller_type = str(_rcfg.get("controller_type", "h12")).lower()
except Exception:
    _controller_type = "h12"
# strip().lower() 与包内 mock_h12_channel_msg.py / send_h12_trigger.py 口径一致:
# 大小写不一致会让 publisher 发 12 通道而 joy 节点按 16 通道解析 -> 每帧 IndexError
_controller_type = os.getenv("REMOTE_CONTROLLER_TYPE", _controller_type).strip().lower()
rospy.logwarn(f"[REMOTE] controller_type = {_controller_type} "
              f"(env REMOTE_CONTROLLER_TYPE={os.getenv('REMOTE_CONTROLLER_TYPE', '(unset)')})")

# ===== G11 屏幕指令解析 (协议库独立在 g11_controller_node 包) =====
# 仅 controller_type == g11 时启用; 缺库时降级为禁用(不影响实体按键端 / H12/G12)
try:
    import g11_screen_protocol as g11proto
    _G11_SCREEN_AVAILABLE = True
except Exception as _e:
    g11proto = None
    _G11_SCREEN_AVAILABLE = False
    rospy.logwarn(f"[REMOTE] g11_screen_protocol import failed, screen cmd disabled: {_e}")

try:
    from robot_version import RobotVersion
except ImportError:
    import sys
    sys.path.insert(0, os.path.join(rospack.get_path('kuavo_common'), 'python'))
    from robot_version import RobotVersion

# =====================================================
# 状态持久化常量
# =====================================================

# 合法状态列表 (来自 robot_state.json ocs2 states)
LEGAL_STATES = [
    "initial",
    "calibrate",
    "ready_stance",
    "stance",
    "vr_remote_control",
    "walk",
    "trot",
    "climb_stair",
    "sit"
]

# ROS param 名称用于持久化最后状态
LAST_STATE_PARAM = "/joy_node/last_state"

class Config:
    # Controller ranges
    MINUS_H12_AXIS_RANGE_MAX = -1722
    H12_AXIS_RANGE_MAX = 1722
    H12_AXIS_RANGE_MIN = 282
    H12_AXIS_RANGE = H12_AXIS_RANGE_MAX - H12_AXIS_RANGE_MIN
    H12_AXIS_MID_VALUE = (H12_AXIS_RANGE_MAX + H12_AXIS_RANGE_MIN) // 2
    
    # State configurations
    # VALID_STATES: 状态切换后需向 /joy 下发"状态反馈"脉冲的运动态集合。
    # ⚠️ 本集合为 H12/G12 与 G11 共用(实体键路径 _execute_state_transition 与 G11
    #    屏幕路径 _g11_state_transition_task 都读它), 改动等价于改 H12/G12 行为。
    # 注意: ready_stance 不在此集合 —— 它是过渡态, 站立由
    # ready_stance_callback 调 /humanoid_controller/real_initial_start 服务完成,
    # 无需 /joy 通道反馈(历史 ee93ed18d8 注释了 map 里的 ready_stance:5,
    # 若留在 VALID_STATES 会因 TRIGGER_CHANNEL_MAP 缺键抛 KeyError,
    # 该 KeyError 此前被外层 except 吞成日志, 行为等价但会刷日志)。
    VALID_STATES = {"rl_control", "stance", "walk", "trot"}
    TRIGGER_CHANNEL_MAP = {
        "stop": 8,
        # "ready_stance": 5,
        "rl_control": 8,
        "stance": 9,
        "walk": 6,
        "trot": 7
    }
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
    SCALE_LEFT_STICK_Y = 1.0  # 左摇杆左右（左右平移）缩放比例
    
    @staticmethod
    def get_default_channels() -> List[int]:
        # 通道数与真实发布器保持一致: g11 -> 16, 其他/默认 -> 12
        n = 16 if _controller_type == "g11" else 12
        channels = [Config.H12_AXIS_RANGE_MIN] * n
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

# G/H滚轮极值判断阈值
G12_DIAL_THRESHOLD = 50

# /joy 发布限频周期:G12 遥控器轴数据硬件上限为 50Hz(20ms),发布频率对齐硬件上限,
# 更高频率只是重复值。多路径高频调用(250Hz 通道回调 + 100Hz 主循环 + 事件路径)
# 统一在发布层去重,实测可把 /joy 从 ~850Hz 降到 50Hz。
JOY_PUB_PERIOD = 0.02

# A/B/C/D 按键边沿脉冲保持时长:与 G+H 复位(M2)同一策略,边沿触发后至少保持 200ms(约 10 帧@50Hz),
# 避免单帧脉冲被 /joy 50Hz 限频(_publish_joy)丢弃,导致 C++ 侧(MobileManipulatorJoyCommandNode)
# 的 a_just/b_just/c_just 边沿检测收不到 1 跳变而需多次按键才能切换控制模式。
G12_BUTTON_PULSE = 0.2

# G12轮臂模式JoyButton常量
G12_BUTTON_A = 0
G12_BUTTON_B = 1
G12_BUTTON_X = 2
G12_BUTTON_Y = 3
G12_BUTTON_LB = 4
G12_BUTTON_RB = 5
G12_BUTTON_BACK = 6
G12_BUTTON_START = 7
G12_BUTTON_GUIDE = 8
G12_BUTTON_M1 = 9
G12_BUTTON_M2 = 10

class H12ToJoyControllerNode:
    def __init__(self):
        """Initialize joy controller node."""
        self.channel_mapping = self._create_channel_mapping()
        self.joy_msg = Joy(axes=[0.0] * 8, buttons=[0] * 11)
        self.channels_msg: Optional[Tuple[int, ...]] = None
        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
        self._last_joy_pub_time = 0.0   # /joy 发布限频时间戳(见 JOY_PUB_PERIOD)
        self.is_stopping = False    # cd按钮下蹲标志位

        # G12轮臂模式检测
        robot_version = os.environ.get('ROBOT_VERSION', '0')
        try:
            self.is_wheel = RobotVersion.create(int(robot_version)).start_with(major=6)
        except (ValueError, TypeError):
            self.is_wheel = False

        # G12轮臂模式: C+D长按急停检测状态
        self.cd_emergency_triggered = False
        self.cd_press_start_time = None
        # G+H同时极值2秒复位：触发后持续置位 M2 200ms，避免单帧脉冲丢失导致复位偶发失效
        self.gh_press_start_time = None
        self.gh_triggered = False
        self.gh_reset_pulse_until = 0.0
        # A/B/C/D 按键脉冲保持截止时间(btn_idx -> time.time() 时间戳)
        # 边沿检测只在一帧返回 True,若该帧被 /joy 50Hz 限频丢弃,下游边沿检测会漏触发;
        # 此处把边沿展宽为持续脉冲,保证至少若干个 /joy 帧携带 button=1。
        self._button_pulse_until = {}

        # ===== G11 轮臂下肢桥(屏幕驱动 C++ MobileManipulatorJoyCommandNode) =====
        # C++ use_g12_ 分支只认 G12 语义 /joy 按钮:
        #   GUIDE=buttons[8](模拟 G 极值按住), M1=buttons[9](模拟 H 按住),
        #   M2=buttons[10](复位), 切模式需 GUIDE + A[3]/B[1]/C[2] 边沿。
        # G11 无 G/H 滚轮, 由屏幕下肢页码驱动: 362/370/378 切模式, 386 复位,
        # 394/402 选躯干组(锁存 GUIDE 或 M1 持续按下, 使摇杆持续产该组指令)。
        self.g11_torso_group = None       # None/xz(394)/yawpitch(402)
        self.g11_leg_pulse = {}           # {btn_idx: 截止时间} 一次性边沿脉冲(切模式/复位)

        if self.is_wheel:
            rospy.set_param('/joystick_type', 'h12')
            rospy.loginfo("[G12] Wheel mode enabled, ROBOT_VERSION=%s, joystick_type=h12", robot_version)

    @staticmethod
    def _create_channel_mapping() -> Dict[int, ChannelMapping]:
        """Create channel mapping configuration.

        G11 遥控器(CH5/6=SW1/SW2 开关, CH7=H, CH8/9=AUX 旋钮, CH10/11=B1/B2):
        实体键/开关全部走状态机 g11_* 映射表触发, 不映射为 G12 语义的 /joy
        按钮(避免 G12 通道布局把 SW2 右档/极值误当 START/A 等按钮)。仅保留
        摇杆 CH1-4 axes 透传, 按钮恒为 0。
        """
        # 摇杆 4 通道(G11/G12 相同): axes 映射
        stick_axes = {
            1: ChannelMapping(1, axis_index=Config.AXIS_MAPPING['RIGHT_STICK_YAW'], reverse=True),
            2: ChannelMapping(2, axis_index=Config.AXIS_MAPPING['RIGHT_STICK_Z'], reverse=True, scale=Config.SCALE_RIGHT_STICK_Z),
            3: ChannelMapping(3, axis_index=Config.AXIS_MAPPING['LEFT_STICK_X']),
            4: ChannelMapping(4, axis_index=Config.AXIS_MAPPING['LEFT_STICK_Y'], reverse=True, scale=Config.SCALE_LEFT_STICK_Y),
        }
        if _controller_type == "g11":
            return stick_axes
        if kuavo_control_scheme == "rl":
            return {
                **stick_axes,
                6: ChannelMapping(6, button_index=Config.BUTTON_MAPPING['START'], 
                                is_button=True, trigger_value=Config.H12_AXIS_RANGE_MAX),
                7: ChannelMapping(7, button_index=Config.BUTTON_MAPPING['LB'], 
                                is_button=True, trigger_value=Config.H12_AXIS_RANGE_MAX),
                8: ChannelMapping(8, button_index=Config.BUTTON_MAPPING['B'], 
                                is_button=True, trigger_value=Config.H12_AXIS_RANGE_MAX),
                9: ChannelMapping(9, button_index=Config.BUTTON_MAPPING['X'], 
                                is_button=True, trigger_value=Config.H12_AXIS_RANGE_MAX),
                10: ChannelMapping(10, button_index=Config.BUTTON_MAPPING['A'], 
                                is_button=True, trigger_value=Config.H12_AXIS_RANGE_MAX),
            }
        elif kuavo_control_scheme == "ocs2" or kuavo_control_scheme == "multi":
            return {
                **stick_axes,
                6: ChannelMapping(6, button_index=Config.BUTTON_MAPPING['START'], 
                                is_button=True, trigger_value=Config.H12_AXIS_RANGE_MAX),
                7: ChannelMapping(7, button_index=Config.BUTTON_MAPPING['Y'], 
                                is_button=True, trigger_value=Config.H12_AXIS_RANGE_MAX),
                8: ChannelMapping(8, button_index=Config.BUTTON_MAPPING['B'], 
                                is_button=True, trigger_value=Config.H12_AXIS_RANGE_MAX),
                9: ChannelMapping(9, button_index=Config.BUTTON_MAPPING['X'], 
                                is_button=True, trigger_value=Config.H12_AXIS_RANGE_MAX),
                10: ChannelMapping(10, button_index=Config.BUTTON_MAPPING['A'], 
                                is_button=True, trigger_value=Config.H12_AXIS_RANGE_MAX),
            }

    def update_channels_msg(self, msg: h12proRemoteControllerChannel) -> None:
        """Update channel message data."""
        self.channels_msg = msg.channels

    def process_channels(self, publish_immediately: bool = False) -> None:
        """Process and publish channel data.

        /joy 发布按 JOY_PUB_PERIOD 限频(G12 硬件轴数据上限 50Hz):周期路径
        (250Hz 通道回调 + 100Hz 主循环)在发布层去重,状态照常更新;
        急停、状态切换等一次性事件传 publish_immediately=True 保证即时下发。
        """
        if self.channels_msg is None:
            self._publish_joy(False)
            return

        # Reset messages
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 11

        # G11 遥控器: 无 G12 语义的 A/B/C/D 按钮与 G/H 滚轮, 全部实体键走
        # 状态机 g11_* 表; /joy 仅透传摇杆 axes, 不执行 G12 轮臂按钮/急停逻辑。
        if _controller_type == "g11":
            self._process_default_channels()
            # G11 轮臂下肢桥: 屏幕选躯干组后锁存 GUIDE/M1(模拟 G/H 按住), 驱动
            # C++ MobileManipulatorJoyCommandNode 的躯干控制; 一次性边沿也在此打。
            if self.is_wheel:
                self._apply_g11_leg_buttons()
        # G12轮臂模式特殊处理
        elif self.is_wheel:
            self._process_wheel_channels()
        else:
            self._process_default_channels()

        self._publish_joy(publish_immediately)

    def _publish_joy(self, publish_immediately: bool) -> None:
        """发布 /joy;非即时模式按 JOY_PUB_PERIOD 限频去重"""
        now = time.time()
        if not publish_immediately and now - self._last_joy_pub_time < JOY_PUB_PERIOD:
            return
        self._last_joy_pub_time = now
        try:
            self.joy_pub.publish(self.joy_msg)
        except rospy.ROSException as e:
            # 让位/关闭过程中 publisher 会被 close，此时 publish 抛
            # "publish() to a closed topic"；节点正在退出，静默丢弃，避免主循环异常退出
            if not rospy.is_shutdown():
                rospy.logwarn_throttle(5.0, 'publish /joy failed: %s' % e)

    def _process_default_channels(self) -> None:
        """默认双足模式：使用标准channel映射"""
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

    # ==================== G11 轮臂下肢桥(屏幕 → /joy → C++) ====================
    # 常数: C++ use_g12_ 分支读取的 /joy 按钮 index
    G12_BTN_GUIDE = 8   # 模拟 G 滚轮极值按住
    G12_BTN_M1 = 9      # 模拟 H 滚轮极值按住
    G12_BTN_M2 = 10     # G+H 2s 复位
    G12_BTN_C = 2       # 切 TORSO_CONTROL 用的按键(X)
    G12_BTN_B = 1       # 切 CMD_VEL_WORLD 用的按键(B)
    G12_BTN_A = 3       # 切 CMD_VEL 用的按键(Y)

    # 一次性边沿脉冲保持时长(秒): 与 G12 _button_pulse_until 同策略,
    # 保证下游 C++ 边沿检测可靠收到(50Hz 限频下不丢帧)
    G11_LEG_PULSE_DURATION = 0.2

    def release_g11_leg_latch(self, reason: str = "") -> None:
        """释放轮臂下肢桥锁存(躯干组选择 + 未过期的边沿脉冲)。

        由两个事件触发: 离开下肢页(1178) / 遥控器断连。
        否则 g11_torso_group 会一直被锁存按压 GUIDE/M1: 在 C++ 侧
        current_mode_==TORSO_CONTROL 时, 摇杆将只产躯干速度而不下发底盘速度
        (gh_combo_active 需 GUIDE+M1 同按, 单锁存不会屏蔽), 底盘将无法驾驶。
        """
        if self.g11_torso_group is not None or self.g11_leg_pulse:
            rospy.logwarn(
                f"[G11Leg] 释放下肢锁存 group={self.g11_torso_group} "
                f"pulse={sorted(self.g11_leg_pulse)} "
                f"({reason or 'unspecified'})")
        self.g11_torso_group = None
        self.g11_leg_pulse.clear()

    def _apply_g11_leg_buttons(self) -> None:
        """把 G11 轮臂下肢桥状态写进 /joy buttons(在 publish 前调用)。

        - g11_torso_group == "xz"(394): 锁存 GUIDE=1 → C++ 躯干模式读 vx/vz
        - g11_torso_group == "yawpitch"(402): 锁存 M1=1 → C++ 读 vyaw/vpitch
        - g11_leg_pulse 内的一次性边沿(切模式 GUIDE+A/B/C、复位 M2)也在此写
        """
        now = time.time()
        # 1) 锁存修饰键(躯干组选择)
        if self.g11_torso_group == "xz":
            self.joy_msg.buttons[self.G12_BTN_GUIDE] = 1
        elif self.g11_torso_group == "yawpitch":
            self.joy_msg.buttons[self.G12_BTN_M1] = 1

        # 2) 一次性边沿脉冲(切模式/复位): 时间未到就置 1
        expired = []
        for btn_idx, deadline in self.g11_leg_pulse.items():
            if now < deadline:
                self.joy_msg.buttons[btn_idx] = 1
            else:
                expired.append(btn_idx)
        for btn_idx in expired:
            del self.g11_leg_pulse[btn_idx]

    def g11_leg_action(self, target: str) -> None:
        """G11 屏幕下肢页码 → C++ MobileManipulatorJoyCommandNode 动作(经 /joy 按钮)。

        Args:
            target: 协议库下肢页的 target 值:
                cmd_vel / cmd_vel_world / torso_control / torso_reset /
                torso_group_xz(394) / torso_group_yawpitch(402)
        """
        # 进入非躯干模式: 清组锁存, C++ 只切模式
        if target in ("cmd_vel", "cmd_vel_world", "torso_control"):
            self.g11_torso_group = None
            # 切模式需 GUIDE 保持 + A/B/C 边沿; 用脉冲保证 C++ 收到边沿
            now = time.time()
            if target == "cmd_vel":
                self.g11_leg_pulse[self.G12_BTN_GUIDE] = now + self.G11_LEG_PULSE_DURATION
                self.g11_leg_pulse[self.G12_BTN_A] = now + self.G11_LEG_PULSE_DURATION
            elif target == "cmd_vel_world":
                self.g11_leg_pulse[self.G12_BTN_GUIDE] = now + self.G11_LEG_PULSE_DURATION
                self.g11_leg_pulse[self.G12_BTN_B] = now + self.G11_LEG_PULSE_DURATION
            elif target == "torso_control":
                self.g11_leg_pulse[self.G12_BTN_GUIDE] = now + self.G11_LEG_PULSE_DURATION
                self.g11_leg_pulse[self.G12_BTN_C] = now + self.G11_LEG_PULSE_DURATION
        elif target == "torso_reset":
            # 复位: M2 边沿(G+H 2s 等价), C++ 内部复位躯干
            now = time.time()
            self.g11_leg_pulse[self.G12_BTN_M2] = now + self.G11_LEG_PULSE_DURATION
        elif target == "torso_group_xz":
            # 选组1 平移/升降: 锁存 GUIDE(模拟按住 G); 需已在躯干模式才有效
            self.g11_torso_group = "xz"
        elif target == "torso_group_yawpitch":
            # 选组2 旋转/俯仰: 锁存 M1(模拟按住 H)
            self.g11_torso_group = "yawpitch"
        else:
            rospy.logwarn(f"[G11Leg] unknown target: {target}")

    def _process_wheel_channels(self) -> None:
        """Wheel-arm mode (G12) special channel processing."""
        channels = list(self.channels_msg)

        # E/F safety switch: both must be at middle position for mode switching
        e_mid = abs(channels[4] - Config.H12_AXIS_MID_VALUE) < 100
        f_mid = abs(channels[5] - Config.H12_AXIS_MID_VALUE) < 100
        safe_enabled = e_mid and f_mid

        # G/H dial extreme value detection
        g_value = channels[10]
        g_at_extreme = (g_value <= Config.H12_AXIS_RANGE_MIN + G12_DIAL_THRESHOLD or
                        g_value >= Config.H12_AXIS_RANGE_MAX - G12_DIAL_THRESHOLD)
        h_value = channels[11]
        h_at_extreme = (h_value <= Config.H12_AXIS_RANGE_MIN + G12_DIAL_THRESHOLD or
                        h_value >= Config.H12_AXIS_RANGE_MAX - G12_DIAL_THRESHOLD)

        # Joystick mapping (channel 1-4) always processed
        for index in [0, 1, 2, 3]:
            mapping = self.channel_mapping.get(index + 1)
            if mapping and mapping.axis_index is not None:
                self.joy_msg.axes[mapping.axis_index] = mapping.get_current_state(channels[index])

        # Safety switch: only controls LB/RB for mode switching
        if safe_enabled:
            self.joy_msg.buttons[G12_BUTTON_LB] = 1
            self.joy_msg.buttons[G12_BUTTON_RB] = 1

        # C+D long press emergency stop - always active
        c_pressed = channels[8] == Config.H12_AXIS_RANGE_MAX
        d_pressed = channels[9] == Config.H12_AXIS_RANGE_MAX
        if c_pressed and d_pressed:
            if self.cd_press_start_time is None:
                self.cd_press_start_time = time.time()
                rospy.loginfo("[G12] C+D emergency stop: holding, waiting 1.0s...")
            elif time.time() - self.cd_press_start_time >= 1.0 and not self.cd_emergency_triggered:
                # BACK 急停脉冲展宽:与 A/B/C/D/M2 同策略,边沿触发后保持 200ms,
                # 避免单帧脉冲被 /joy 50Hz 限频丢弃导致 C++ 侧 back_just 漏触发。
                self.cd_emergency_triggered = True
                self._button_pulse_until[G12_BUTTON_BACK] = time.time() + G12_BUTTON_PULSE
                rospy.logwarn("[G12] C+D emergency stop TRIGGERED!")
        else:
            if self.cd_emergency_triggered:
                rospy.loginfo("[G12] C+D emergency stop released")
            self.cd_press_start_time = None
            self.cd_emergency_triggered = False

        # BACK 急停脉冲保持:急停触发后持续置位 200ms,确保 C++ 侧 back_just 边沿检测可靠收到
        if time.time() < self._button_pulse_until.get(G12_BUTTON_BACK, 0.0):
            self.joy_msg.buttons[G12_BUTTON_BACK] = 1

        # Button mapping (A/B/C/D) - always active
        wheel_button_map = {
            6: G12_BUTTON_Y,    # channel 7(A) -> buttons[3](Y)
            7: G12_BUTTON_B,    # channel 8(B) -> buttons[1](B)
            8: G12_BUTTON_X,    # channel 9(C) -> buttons[2](X)
            9: G12_BUTTON_A,    # channel 10(D) -> buttons[0](A)
        }
        for ch_idx, btn_idx in wheel_button_map.items():
            mapping = self.channel_mapping.get(ch_idx + 1)
            if mapping and mapping.is_button:
                # get_current_state 仅在通道值由"未按下→按下"跳变的那一帧返回 True(边沿)。
                # 该单帧脉冲在 /joy 50Hz 限频下大概率被 _publish_joy 丢弃,导致下游
                # MobileManipulatorJoyCommandNode 的边沿检测漏触发(G12 模式切换偶发需多次按键)。
                # 边沿触发后把 button 展宽为持续 200ms 脉冲,与 G+H 复位(M2)同一策略。
                edge = mapping.get_current_state(channels[ch_idx])
                if edge:
                    self._button_pulse_until[btn_idx] = time.time() + G12_BUTTON_PULSE
                if time.time() < self._button_pulse_until.get(btn_idx, 0.0):
                    self.joy_msg.buttons[btn_idx] = 1

        # G/H dial buttons - always active
        if g_at_extreme:
            self.joy_msg.buttons[G12_BUTTON_GUIDE] = 1
        if h_at_extreme:
            self.joy_msg.buttons[G12_BUTTON_M1] = 1

        # G+H both at extreme for 2s -> torso reset - requires E/F at middle
        # 触发后持续置位 M2 200ms：单帧脉冲可能因下游回调阻塞被丢弃，导致复位偶发失效
        if safe_enabled and g_at_extreme and h_at_extreme:
            if self.gh_press_start_time is None:
                self.gh_press_start_time = time.time()
                self.gh_triggered = False
            elif time.time() - self.gh_press_start_time >= 2.0 and not self.gh_triggered:
                self.gh_triggered = True
                self.gh_reset_pulse_until = time.time() + 0.2
                rospy.logwarn("[G12] G+H torso reset TRIGGERED (200ms pulse)!")
        else:
            if self.gh_press_start_time is not None and not self.gh_triggered:
                rospy.loginfo("[G12] G+H torso reset: released, %.1fs elapsed (not triggered)",
                              time.time() - self.gh_press_start_time)
            self.gh_press_start_time = None
            if time.time() >= self.gh_reset_pulse_until:
                self.gh_triggered = False

        # M2复位脉冲持续200ms，保证C++侧边沿检测可靠收到（边沿+锁存，重复帧安全）
        if self.gh_triggered and time.time() < self.gh_reset_pulse_until:
            self.joy_msg.buttons[G12_BUTTON_M2] = 1

class H12PROControllerNode:
    """Main controller node for H12PRO remote controller."""

    def _control_stack_online(self, retries: int = 3, interval: float = 0.3) -> bool:
        """探测控制栈是否在线（= 机器人是否还站着）。

        用于 __init__ 状态恢复门：控制栈在线 -> 续 last_state（joy_node 崩溃重启，续运行态）；
        控制栈不在线 -> 回 initial（终端退出后服务 reclaim，回待命态，避免按 C 失效）。

        查询失败（rosnode 抛异常）的兜底方向是 True（保状态）：宁可退化成
        “待命态按 C 不灵”的老问题（机器人静止、无危险），也不要在查询抽风时把一个
        可能仍在运动的机器人 FSM 打回 initial。
        """
        for attempt in range(retries):
            try:
                nodes = rosnode.get_node_names()
                return "/nodelet_controller" in nodes or "/humanoid_sqp_mpc" in nodes
            except Exception as e:
                rospy.logwarn(f"[StateRecovery] rosnode query failed ({attempt + 1}/{retries}): {e}")
                if attempt < retries - 1:
                    time.sleep(interval)
        rospy.logwarn("[StateRecovery] rosnode unavailable; assume stack ONLINE to preserve state")
        return True

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
        # 先读旧 last_state（崩溃续态 / 仿真预置 stance），避免先写 initial 盖掉恢复源
        try:
            last_saved_state = rospy.get_param(LAST_STATE_PARAM, "none")
        except Exception as e:
            last_saved_state = "none"
            rospy.logwarn(
                f"[StateRecovery] Failed to read {LAST_STATE_PARAM}: {e}; "
                "treat as none (no recovery)"
            )

        # 手动模式只同步 param，不走恢复
        if self.manual_h12_init_state != "none":
            rospy.set_param(LAST_STATE_PARAM, self.robot_state_machine.state)

        self.h12_to_joy_node = H12ToJoyControllerNode()

        # 服务/仿真：按 last_state + 控制栈是否在线恢复
        if self.manual_h12_init_state == "none":
            try:
                rospy.loginfo(f"[StateRecovery] Last saved state: {last_saved_state}")

                if last_saved_state != "none" and last_saved_state in LEGAL_STATES \
                        and last_saved_state not in ["initial", "calibrate"]:

                    if self._control_stack_online():
                        # 栈在线：续运行态 / 仿真进 stance
                        self.robot_state_machine.machine.set_state(
                            last_saved_state, self.robot_state_machine
                        )
                        rospy.set_param(LAST_STATE_PARAM, last_saved_state)
                        rospy.loginfo(f"[StateRecovery] stack online, recovered to: {last_saved_state}")
                    else:
                        # 栈离线：丢弃残留 last_state，回 initial 待命
                        self.robot_state_machine.machine.set_state(
                            "initial", self.robot_state_machine
                        )
                        rospy.set_param(LAST_STATE_PARAM, "initial")
                        rospy.logwarn(
                            "[StateRecovery] control stack offline; reset FSM to initial "
                            f"(discarded stale last_state={last_saved_state}) so H12 can relaunch."
                        )
                else:
                    # 无有效旧态：冷启动同步 initial
                    rospy.set_param(LAST_STATE_PARAM, self.robot_state_machine.state)

            except Exception as e:
                rospy.logerr(f"[StateRecovery] Failed to recover: {e}")

        self.key_timestamp: Dict[str, float] = {}
        self._config = self._load_configuration()
        
        # ROS related initialization
        self.current_arm_joint_state = []
        self.plan_arm_is_finished = True
        self.should_pub_arm_joint_state = JointState()
        self.should_pub_hand_position = robotHandPosition()
        self.start_way = rospy.get_param("start_way", "auto")
        self.real_robot = rospy.get_param("real_robot", False)
        self.only_half_up_body = rospy.get_param("only_half_up_body", False)
        #zsh
        # 头部控制模式
        # 头部控制参数：摇杆 → 瞬时角速度（deg/s），不再在遥控器侧积分。
        # 开环绝对位姿由接收侧（ControlDataManager / humanoidController）积分，
        # 与躯干 vel/delta 重构（f97e109da0a）一致：速度语义不应存开环位姿。
        self.head_control_mode = False
        # 满量程角速度（deg/s），rosparam 可调
        # defaults = 旧 sensitivity × 500Hz 体感校准（与躯干 kTorsoVelCalibHz 一致）
        self.head_vel_scale_yaw = rospy.get_param("~head_vel_scale_yaw", 400.0)    # 0.8 × 500
        self.head_vel_scale_pitch = rospy.get_param("~head_vel_scale_pitch", 100.0)  # 0.2 × 500
        # 摇杆死区
        self.head_stick_deadzone = 0.1
        # True after a non-zero head vel publish; emit one zero on stick release to clear sticky latch
        self._head_vel_latched = False
        #zsh

        self.is_navigation_mode = False # 导航状态变量
        
        # 动作执行状态跟踪（用于屏蔽摇杆输入）
        self.robot_action_executing = False  # True表示有tact动作正在执行

        # ===== G11 屏幕指令解析器(协议库来自 g11_controller_node 包) =====
        self.screen_cmd_enabled = (_controller_type == "g11" and _G11_SCREEN_AVAILABLE)
        self.screen_parser = g11proto.ScreenCmdParser() if (g11proto is not None) else None
        self.screen_prev_page = -1
        # 上次已上报的计数 (cmd_drops, state_holds, unknown_cmds, hidden_cmds)
        self._g11_parser_counters = None
        self._g11_parser_counters_at = 0.0   # 上次上报时间(自管节流, 见上报方法)
        # ================================================================

        # ===== G11 末端(夹爪)跟随桥状态 =====
        self._g11_claw_was_enabled = False     # CH14 claw 位上一帧使能沿
        self._g11_claw_last_aux = None         # (left_pct, right_pct)
        self._g11_ee_type = None               # /end_effector_type 缓存
        self._g11_ee_checked_at = 0.0
        # 默认 AUX 值越大开度越大; 实机旋钮方向相反时置 True (~claw_aux_reverse)
        self._claw_aux_reverse = rospy.get_param("~claw_aux_reverse", False)
        # ================================================================

        # 添加线程池
        self.executor = ThreadPoolExecutor(max_workers=2)
        self._state_transition_lock = threading.Lock()
        self._state_transition_executing = False  # 标记是否有状态转换正在执行

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
        # /bezier/arm_traj 订阅和 /kuavo_arm_traj 发布已移除：
        # tact 执行时轨迹转发由 autostart (arm_trajectory_bezier_process.py) 统一处理，
        # joy_node 不再参与手臂轨迹数据流，避免多发布者冲突
        self.plan_arm_state_sub = rospy.Subscriber(
            "/bezier/arm_traj_state",
            planArmState,
            self._plan_arm_state_callback,
            queue_size=1,
            tcp_nodelay=True 
        )
        
        # 订阅手臂动作执行状态，用于在tact执行期间屏蔽摇杆输入
        self.robot_action_state_sub = rospy.Subscriber(
            "/robot_action_state",
            RobotActionState,
            self._robot_action_state_callback,
            queue_size=1,
            tcp_nodelay=True
        )
        
        self.control_hand_pub = rospy.Publisher(
            '/control_robot_hand_position',
            robotHandPosition,
            queue_size=1,
            tcp_nodelay=True
        )
        # 头部速度 / 相对位移接口（与躯干 /cmd_torso_vel、/cmd_torso_delta 对齐）
        # joint_data = [yaw, pitch]，vel 单位 deg/s，delta 单位 deg（oneshot）
        # 接收侧做开环积分 + 限位；本节点不再维护 current_head_yaw/pitch
        self.cmd_head_vel_pub = rospy.Publisher(
            '/cmd_head_vel',
            robotHeadMotionData,
            queue_size=1,
            tcp_nodelay=True
        )
        # 初始化全局停止话题发布者（项目统一用Bool协议，True表示停止）
        self.stop_robot_pub = rospy.Publisher('/stop_robot', Bool, queue_size=1)

        # ===== G11 末端(夹爪/灵巧手)跟随: AUX1/2 开度 → 对应命令话题 =====
        # 按 /end_effector_type 分发: lejuclaw→/leju_claw_command, 灵巧手→/dexhand/command。
        self.g11_claw_pub = None       # /leju_claw_command (二指夹爪)
        self.g11_dexhand_pub = None    # /dexhand/command (灵巧手)
        if _G11_LEJUCLAW_MSG_OK:
            self.g11_claw_pub = rospy.Publisher(
                '/leju_claw_command', _G11LejuClawCmdMsg, queue_size=1)
        if _G11_DEXHAND_MSG_OK:
            self.g11_dexhand_pub = rospy.Publisher(
                '/dexhand/command', _G11DexhandCmdMsg, queue_size=1)
        self.update_h12_customize_config_sub = rospy.Subscriber(
            "/update_h12_customize_config",
            UpdateH12CustomizeConfig,
            self._update_h12_customize_config_callback,
            queue_size=1
        )

        # 导航状态订阅者
        self.navigation_state_sub = rospy.Subscriber(
            "/navigation_control",
            Bool,
            self._navigation_state_callback,
            queue_size=1
        )
    
    def _update_h12_customize_config_callback(self, msg):
        self.robot_state_machine.update_customize_config()
        
    def _navigation_state_callback(self, msg):
        # msg: std_msgs.msg.Bool - 导航状态消息 (false: H12 控制, true: 导航控制)
        # 在导航中 joy 数据不参与控制
        try:
            self.is_navigation_mode = msg.data
            rospy.loginfo(f"[NavigationState] Navigation mode {'enabled' if msg.data else 'disabled'}")
        except Exception as e:
            rospy.logerr(f"[NavigationState] Error processing navigation state: {e}")
        
    def publish_arm_joint_state(self):
        # /kuavo_arm_traj 由 autostart 统一发布，joy_node 不再转发手臂轨迹
        pass

    def _plan_arm_state_callback(self, msg):
        self.plan_arm_is_finished = msg.is_finished


    # def _mpc_obs_callback(self, msg):
    #     self.current_arm_joint_state = msg.state.value[24:]
    #     self.current_arm_joint_state = [round(pos, 2) for pos in self.current_arm_joint_state]
    #     self.current_arm_joint_state.extend([0] * 14)
    
    def _sensor_data_callback(self, msg):
        self.current_arm_joint_state = msg.joint_data.joint_q[12:26]
        self.current_arm_joint_state = [round(pos, 2) for pos in self.current_arm_joint_state]
        self.current_arm_joint_state.extend([0] * 14)
    
    def _robot_action_state_callback(self, msg: RobotActionState) -> None:
        """处理手臂动作状态回调
        state: 0=失败, 1=执行中, 2=成功
        当state==1时，表示tact动作正在执行，需要屏蔽摇杆输入
        """
        self.robot_action_executing = (msg.state == 1)
        if self.robot_action_executing:
            rospy.logdebug("[RobotActionState] Tact action executing, joystick input will be blocked.")

    def _load_configuration(self) -> Dict[str, Any]:
        """Load and validate configuration from JSON file.

        G11 遥控器使用独立的 g11_* 映射表(实体键 SW1/SW2/H/B1/B2 + AUX 旋钮,
        通道布局/按键语义与 G12 完全不同); H12/G12 使用默认表, 零改动。
        屏幕通道(CH14/CH15/CH16)不在此表内, 由 _handle_g11_screen 单独解析。

        Returns:
            Dict containing validated configuration.
            
        Raises:
            ConfigError: If configuration is invalid or missing required fields.
        """
        try:
            config = read_json_file(h12pro_remote_controller_path)
            
            # Determine which state transition configuration to use based on control scheme
            if kuavo_control_scheme == "ocs2":
                state_transition_key = "ocs2_robot_state_transition_keycombination"
            elif kuavo_control_scheme == "rl":
                state_transition_key = "rl_robot_state_transition_keycombination"
            elif kuavo_control_scheme == "multi":
                state_transition_key = "multi_robot_state_transition_keycombination"
            else:
                raise ConfigError(f"Invalid control scheme: {kuavo_control_scheme}")

            # G11 遥控器使用 g11_ 前缀的独立映射表 (实体按键布局不同, 见 json)
            # 仅 multi scheme 提供 G11 表; 其他 scheme 回退默认表(安全: token
            # 名不匹配则不会触发, 实体键无效但不误动作)。
            if _controller_type == "g11":
                prefix = "g11_"
                if f"{prefix}{state_transition_key}" in config:
                    state_transition_key = f"{prefix}{state_transition_key}"
                else:
                    rospy.logwarn(
                        f"[Config] G11 遥控器无 {prefix}{state_transition_key} 映射表, "
                        f"回退默认表(实体键将不触发状态转换, 屏幕指令不受影响)")
            else:
                prefix = ""

            g11_key_name_key = f"{prefix}channel_to_key_name"
            g11_key_state_key = f"{prefix}channel_to_key_state"
            g11_emergency_key = f"{prefix}emergency_stop_key_combination"

            channel_to_key_name = config.get(g11_key_name_key, config["channel_to_key_name"])
            channel_to_key_state = config.get(g11_key_state_key, config["channel_to_key_state"])
            emergency_stop = config.get(g11_emergency_key, config["emergency_stop_key_combination"])

            required_fields = [
                "channel_to_key_name",
                "channel_to_key_state",
                state_transition_key,
                "emergency_stop_key_combination"
            ]
            
            # Validate configuration
            for field in required_fields:
                if field not in config:
                    raise ConfigError(f"Missing required field: {field}")
            
            rospy.loginfo(f"Loading configuration for control scheme: {kuavo_control_scheme}")
            
            return {
                "channel_to_key_name": channel_to_key_name,
                "channel_to_key_state": channel_to_key_state,
                "state_transitions": config[state_transition_key],
                "emergency_stop_keys": set(emergency_stop)
            }
            
        except Exception as e:
            rospy.logerr(f"Failed to load configuration: {e}")
            raise ConfigError(f"Configuration error: {e}")

    def _timer_callback(self, event: rospy.Timer) -> None:
        """Update has_joy_node parameter periodically."""
        self.start_way = rospy.get_param("start_way", "auto")
        self.only_half_up_body = rospy.get_param("only_half_up_body", False)

    def _channel_callback(self, msg: h12proRemoteControllerChannel) -> None:
        """Process incoming channel messages.

        Args:
            msg: Channel message containing control data.
        """
        if msg.sbus_state == 0:
            rospy.logwarn_throttle(5.0, "No receive h12pro channel message. Please check device `/dev/usb_remote` exist or not and re-plug the h12pro signal receiver.")
            # 断连保护: 驱动在 ≥100ms 无新帧时会把通道复位为默认值并置 sbus_state=0
            # (drivers_sbus.c initializeSbusRxData)。此前这里直接 return, 复位后的中性
            # 通道永远进不到 joy 节点, 而主循环仍以 100Hz 用"断连前最后一帧"重算并按
            # 50Hz 发 /joy -> 若断连时正推着行走杆, 机器人会继续按旧速度行走。
            # 这里补喂一帧中性通道(前 4 通道回中)并立即下发, 让"断连 = 摇杆回中"真正生效。
            # 注: 其余通道取 MIN(按键松开/开关归位); G12 轮臂下 GUIDE/M1 会被判为极值,
            #     而 C++ 侧 gh_combo_active 恰好会屏蔽切模式与躯干指令, 属安全态。
            try:
                neutral_msg = h12proRemoteControllerChannel()
                neutral_msg.channels = tuple(Config.get_default_channels())
                self.h12_to_joy_node.update_channels_msg(msg=neutral_msg)
                self.h12_to_joy_node.process_channels(publish_immediately=True)
                # 头部控制若处于开启态, 断连同样要停住(否则会按断连前的角速度继续转)
                if self.head_control_mode:
                    self._handle_head_control(neutral_msg)
                # 下肢桥锁存同样要释放: 断连时通道虽已回中, 但 _apply_g11_leg_buttons
                # 仍会按锁存强制写 GUIDE/M1=1, 使中性帧退化为"躯干极值按住",
                # 断连保护失效。
                self.h12_to_joy_node.release_g11_leg_latch("遥控器断连")
            except Exception as e:
                rospy.logerr(f"Error handling sbus disconnect neutral frame: {e}")
            return

        try:
            key_combination = self._process_channels(msg.channels)
            self._handle_state_transitions(key_combination, msg)
            # G11 屏幕指令解析(仅 g11 生效, 协议库在 g11_controller_node 包)
            self._handle_g11_screen(msg)
            # G11 末端(夹爪)跟随: CH14 claw 位使能 + CH8/9 AUX 开度
            self._update_g11_claw_follow(msg)
        except Exception as e:
            rospy.logerr(f"Error processing channel message: {e}")

    # ==================== G11 屏幕指令(薄引用 g11_screen_protocol) ====================
    # 协议计数上报周期(秒)
    G11_COUNTER_REPORT_PERIOD = 5.0

    def _report_g11_parser_counters(self) -> None:
        """上报协议库诊断计数(三路通道异常/未知码/未启用码), 有变化时打印。

        按通道分开报, 便于直接看出是哪条通道读到异常值:
          - CH14 异常 -> 沿用最后有效值(无害)
          - CH15 异常 -> 整帧丢弃(命令可能丢失)
          - CH16 异常 -> 沿用最后有效值(无害)
        """
        p = self.screen_parser
        if p is None:
            return
        cur = (getattr(p, "cmd_drops", 0),
               getattr(p, "state_holds", 0),
               getattr(p, "unknown_cmds", 0),
               getattr(p, "hidden_cmds", 0))
        if cur == getattr(self, "_g11_parser_counters", None):
            return
        # 未到周期先攒着(不改基线), 下次进来再一起报
        now = time.time()
        if now - getattr(self, "_g11_parser_counters_at", 0.0) < self.G11_COUNTER_REPORT_PERIOD:
            return
        old = getattr(self, "_g11_parser_counters", None) or (0, 0, 0, 0)
        self._g11_parser_counters = cur
        self._g11_parser_counters_at = now
        # 三路通道异常次数
        abn = getattr(p, "ch_abnormal", None) or {}
        abn_str = " ".join(f"CH{ch}={n}" for ch, n in sorted(abn.items()))
        last = getattr(p, "last_abnormal", None)
        rospy.logwarn(
            f"[G11Screen] 协议计数: 通道异常[{abn_str or 'CH14=0 CH15=0 CH16=0'}] "
            f"CH15越界丢帧={cur[0]}(+{cur[0] - old[0]}) "
            f"状态沿用={cur[1]}(+{cur[1] - old[1]}) "
            f"未知码={cur[2]}(+{cur[2] - old[2]}) "
            f"未启用码(842~882)={cur[3]}(+{cur[3] - old[3]})"
            + (f" 最近异常=CH{last[0]}读到{last[1]}" if last else ""))

    def _handle_g11_screen(self, msg: h12proRemoteControllerChannel) -> None:
        """解析 G11 屏幕虚拟通道指令: CH14 页面码 + CH15 功能码脉冲 + CH16 设置位。
        协议表/上升沿/页面校验全部由 g11_controller_node 包的协议库实现,
        本方法只负责: 取通道值喂解析器, 命中后映射到本进程的状态机/头部控制执行。
        """
        if not self.screen_cmd_enabled or self.screen_parser is None:
            return
        # 通道数保护: 屏幕虚拟载荷在 CH14~CH16, 需要 16 通道。若 REMOTE_CONTROLLER_TYPE
        # 只在一侧生效(publisher 发 12 / joy 节点按 16 解析), 下面取下标会 IndexError,
        # 被外层 except 吞成每帧一条日志(50Hz 刷屏)且屏幕指令静默失效。
        if len(msg.channels) < 16:
            rospy.logwarn_throttle(5.0,
                f"[G11Screen] 通道数不足({len(msg.channels)}), 需要 16 通道; 请确认 "
                f"REMOTE_CONTROLLER_TYPE=g11 在 h12pro_channel_publisher 与 joy_node 上均已生效")
            return
        try:
            # 屏幕载荷在 CH14/CH15/CH16 (msg.channels 是 0-based), CH12/CH13 不读取
            page_val = int(msg.channels[13])    # CH14 页面码(接收值)
            cmd_val = int(msg.channels[14])     # CH15 功能码(接收值)
            cfg_val = int(msg.channels[15])     # CH16 设置位(接收值)

            hit = self.screen_parser.update(page_val, cmd_val, cfg=cfg_val)
            self._report_g11_parser_counters()
            try:
                set_g11_boot_force(self.screen_parser.boot_force)
            except Exception as _e:
                rospy.logwarn_throttle(5.0, f"[G11Screen] sync boot_force failed: {_e}")

            # 页面码用协议库解析出的最后有效值(越界时沿用)
            eff_page = getattr(self.screen_parser, "effective_page", page_val)

            # 页面变化日志(节流)
            if self.screen_prev_page != eff_page:
                prev_page = self.screen_prev_page
                page_name = g11proto.SCREEN_PAGE_RECV.get(
                    eff_page, f"UNKNOWN({eff_page})")
                rospy.loginfo(f"[G11Screen] CH14 page -> {eff_page} ({page_name})")
                self.screen_prev_page = eff_page

                # 离开下肢页 -> 释放轮臂下肢锁存(躯干组选择/未过期边沿脉冲)。
                # 否则 g11_torso_group 常驻, C++ 躯干控制模式下 GUIDE/M1 一直被
                # 按住, 摇杆只产躯干速度、底盘无法驾驶。
                if (prev_page == g11proto.PAGE_LEG_RECV
                        and eff_page != g11proto.PAGE_LEG_RECV):
                    self.h12_to_joy_node.release_g11_leg_latch(
                        f"离开下肢页 {prev_page} -> {eff_page}")

                # 屏幕选型绑定: 进入人型/轮臂二级菜单 -> 记录; 回到首页/空闲 -> 清除
                # (三级页 1162+ 不改变绑定; 未选择时首页直接实体H启动走旧推断逻辑)
                try:
                    if eff_page == g11proto.PAGE_HUMANOID_RECV:
                        set_g11_robot_kind("humanoid")
                        rospy.loginfo("[G11Screen] 绑定机器人类型: humanoid (人型)")
                    elif eff_page == g11proto.PAGE_WHEEL_RECV:
                        set_g11_robot_kind("wheel")
                        rospy.loginfo("[G11Screen] 绑定机器人类型: wheel (轮臂)")
                    elif eff_page == g11proto.PAGE_IDLE_RECV:
                        set_g11_robot_kind("")
                        rospy.loginfo("[G11Screen] 回到首页, 已清除机器人类型绑定")
                except Exception as e:
                    rospy.logwarn(f"[G11Screen] update robot kind binding failed: {e}")

            if hit is None:
                return

            if hit.get("rejected"):
                rospy.logwarn(
                    f"[G11Screen] {hit['func_name']} (CH15={hit['cmd']}) rejected: "
                    f"page mismatch (CH14={hit['page']}, expect {hit['expect_page']})")
                return

            rospy.logwarn(
                f"[G11Screen] TRIGGER: {hit['func_name']} (CH15={hit['cmd']}, CH14={hit['page']})")
            self._g11_screen_exec(hit["func_name"], hit["type"], hit["target"])
        except Exception as e:
            rospy.logerr(f"[G11Screen] error: {e}")

    def _g11_screen_exec(self, func_name: str, ftype: str, target) -> None:
        """执行 G11 屏幕指令(与实体按键端共用状态机/头部控制/动作通道)。"""
        cur_state = self.robot_state_machine.state

        # 冷却期闸门: 与实体键路径 _handle_normal_transitions 口径一致 ——
        # switch_controller / depth_loco 切换后有 SWITCH_CONTROLLER_COOLDOWN(3s) 冷却,
        # 期间禁止再触发状态转换; 否则屏幕上连点两次「切换MPC/AMP」会真的切两次。
        # 例外: wheel_leg 属轮臂下肢子系统(走 /joy 桥, 有自己的 C++ 侧冷却), 不经状态机。
        if (kuavo_control_scheme == "multi" and ftype != "wheel_leg"
                and is_switch_controller_in_cooldown()):
            rospy.logwarn(
                f"[G11Screen] {func_name} rejected: switch_controller in cooldown")
            return

        # --- 头部控制: 特殊处理(复刻实体按键 toggle_head_control) ---
        if ftype == "head":
            if cur_state == "stance":
                self.head_control_mode = not self.head_control_mode
                if not self.head_control_mode:
                    self._publish_head_vel(0.0, 0.0)
                    self._head_vel_latched = False
                rospy.loginfo(
                    f"[G11Screen/Head] mode {'enabled' if self.head_control_mode else 'disabled'}")
            else:
                if self.head_control_mode:
                    self.head_control_mode = False
                    self._publish_head_vel(0.0, 0.0)
                    self._head_vel_latched = False
                    rospy.logwarn("[G11Screen/Head] exited stance, head control disabled")
            return

        # --- 下肢/轮臂页: 本期人型不处理 ---
        if ftype == "skip":
            rospy.logwarn(f"[G11Screen] {func_name} not supported on this robot")
            return

        # --- 轮臂下肢控制: 屏幕页码 → C++ MobileManipulatorJoyCommandNode ---
        # 经 /joy 伪造按钮驱动(不改 C++); 仅轮臂(is_wheel)有效, 人型忽略。
        if ftype == "wheel_leg" and target:
            if self.h12_to_joy_node.is_wheel:
                self.h12_to_joy_node.g11_leg_action(target)
                rospy.logwarn(f"[G11Leg] {func_name} -> {target}")
            else:
                rospy.logwarn(f"[G11Screen] {func_name} 仅轮臂支持, 人型忽略")
            return

        # --- 状态机 trigger / 自定义动作 / 硬件启动(boot) ---
        if ftype in ("trigger", "action", "boot") and target:
            trigger = target
            if self._state_transition_executing:
                rospy.logwarn(
                    f"[G11Screen] trigger '{trigger}' rejected: another transition executing")
                return
            if (("arm_pose" in trigger or "customize_action" in trigger)
                    and self.robot_action_executing):
                rospy.logwarn(
                    f"[G11Screen] trigger '{trigger}' rejected: arm action executing")
                return

            avail = self.robot_state_machine.machine.get_triggers(cur_state)
            if trigger not in avail:
                rospy.logwarn(
                    f"[G11Screen] trigger '{trigger}' not available from '{cur_state}' "
                    f"({func_name})")
                return

            kwargs = {
                "trigger": trigger,
                "source": cur_state,
                "real_robot": self.real_robot,
            }
            if "arm_pose" in trigger:
                kwargs["current_arm_joint_state"] = self.current_arm_joint_state
            self._g11_state_transition_task(trigger, kwargs, func_name)

    # ==================== G11 AUX 旋钮 → 末端执行器跟随桥(二指夹爪 / 灵巧手) ====================
    # 遥控端语义:  CH14 claw 位 = 末端跟踪使能开关; CH8 AUX1 / CH9 AUX2 = 左/右开度。
    # 机器人端语义: 读 /end_effector_type 自动区分末端, 遥控器不感知末端型号。
    #   lejuclaw: position 0=闭合 / 100=全开(直通)。
    #   qiangnao/linker_hand 灵巧手: /dexhand/command, 0=全开 / 100=全闭(方向相反),
    #     6 指同比例联动(AUX1 左/AUX2 右), 暂不做单指/手势。
    def _resolve_g11_ee_type(self) -> str:
        """读取 /end_effector_type(1s 缓存); 硬件节点未启动时返回 ''。"""
        now = time.time()
        if self._g11_ee_type is not None and now - self._g11_ee_checked_at < 1.0:
            return self._g11_ee_type
        self._g11_ee_checked_at = now
        try:
            ee = str(rospy.get_param("/end_effector_type", "")).strip().lower()
        except Exception:
            ee = ""
        self._g11_ee_type = ee
        return ee

    def _aux_raw_to_pct(self, raw: int) -> Optional[int]:
        """AUX 通道接收值 [282,1722] → 开度百分比 [0,100] 整数; 越界返回 None。"""
        if not (Config.H12_AXIS_RANGE_MIN <= raw <= Config.H12_AXIS_RANGE_MAX):
            return None
        pct = (raw - Config.H12_AXIS_RANGE_MIN) / \
            (Config.H12_AXIS_RANGE_MAX - Config.H12_AXIS_RANGE_MIN) * 100.0
        if self._claw_aux_reverse:
            pct = 100.0 - pct
        return int(round(max(0.0, min(100.0, pct))))

    def _update_g11_claw_follow(self, msg: h12proRemoteControllerChannel) -> None:
        """按 CH16 claw 位与 AUX 开度向夹爪发布目标位置(仅 g11 遥控器)。"""
        if _controller_type != "g11" or self.screen_parser is None:
            return
        try:
            enable = bool(self.screen_parser.claw_mode)
        except Exception:
            return

        if not enable:
            if self._g11_claw_was_enabled:
                self._g11_claw_was_enabled = False
                self._g11_claw_last_aux = None
                print("========== 末端跟随 ==========", flush=True)
                print("当前模式: 关闭(AUX 旋钮不再控制夹爪)", flush=True)
                print("==================================", flush=True)
            return

        if not self._g11_claw_was_enabled:
            self._g11_claw_was_enabled = True
            ee = self._resolve_g11_ee_type()
            print("========== 末端跟随 ==========", flush=True)
            print(f"末端类型: {ee or '(待硬件上报)'}", flush=True)
            print("当前模式: AUX1/2 旋钮控制末端执行器开度", flush=True)
            print("==================================", flush=True)

        ee = self._resolve_g11_ee_type()
        # 末端类型未上报/为 none: 机器人(硬件节点)未启动, 静默等待, 不刷日志
        if ee in ("", "none"):
            return
        # 灵巧手(qiangnao/linker_hand): AUX → /dexhand/command 全手指同比例
        # (0=全开 / 100=全闭, 与二指夹爪方向相反)
        if ee in ("qiangnao", "qiangnao_touch", "linker_hand"):
            if not _G11_DEXHAND_MSG_OK or self.g11_dexhand_pub is None:
                return
            try:
                left_raw = int(msg.channels[7])
                right_raw = int(msg.channels[8])
            except (IndexError, ValueError, TypeError):
                return
            left = self._aux_raw_to_pct(left_raw)
            right = self._aux_raw_to_pct(right_raw)
            if left is None or right is None:
                return
            # 任何开度变化立即发; 仅左右都完全未变化时跳过
            now = time.time()
            if self._g11_claw_last_aux is not None:
                if (left == self._g11_claw_last_aux[0]
                        and right == self._g11_claw_last_aux[1]):
                    return
            self._g11_claw_last_aux = (left, right)
            try:
                cmd = _G11DexhandCmdMsg()
                cmd.control_mode = 0  # 位置控制
                # 灵巧手 0=全开 / 100=全闭 → 旋钮开度取反; 双手各 6 指同比例
                l_val = 100 - left
                r_val = 100 - right
                cmd.data = [l_val] * 6 + [r_val] * 6
                self.g11_dexhand_pub.publish(cmd)
            except Exception as e:
                rospy.logerr_throttle(5.0, f"[G11Claw] dexhand publish failed: {e}")
            return
        if ee in ("revo2", "dexhand"):
            rospy.logwarn_throttle(5.0,
                f"[G11Claw] 当前末端 '{ee}' 暂未接入 AUX 同比例控制")
            return
        if ee != "lejuclaw":
            rospy.logwarn_throttle(5.0, f"[G11Claw] 未知末端类型 '{ee}', 忽略")
            return
        if not _G11_LEJUCLAW_MSG_OK or self.g11_claw_pub is None:
            return

        # AUX1=左(CH8), AUX2=右(CH9)
        try:
            left_raw = int(msg.channels[7])
            right_raw = int(msg.channels[8])
        except (IndexError, ValueError, TypeError):
            return
        left = self._aux_raw_to_pct(left_raw)
        right = self._aux_raw_to_pct(right_raw)
        if left is None or right is None:
            return

        # 任何开度变化立即发; 仅左右都完全未变化时跳过
        now = time.time()
        if self._g11_claw_last_aux is not None:
            if (left == self._g11_claw_last_aux[0]
                    and right == self._g11_claw_last_aux[1]):
                return
        self._g11_claw_last_aux = (left, right)

        try:
            cmd = _G11LejuClawCmdMsg()
            cmd.data.name = ["left_claw", "right_claw"]
            # 百分比整数下发(0 闭合 ~ 100 全开)
            cmd.data.position = [float(left), float(right)]
            cmd.data.velocity = [50.0, 50.0]
            cmd.data.effort = [1.0, 1.0]
            self.g11_claw_pub.publish(cmd)
        except Exception as e:
            rospy.logerr_throttle(5.0, f"[G11Claw] publish failed: {e}")

    def _g11_state_transition_task(self, trigger: str, kwargs: Dict[str, Any],
                                   func_name: str = "") -> None:
        """带锁线程池执行状态机 trigger + 状态持久化(不构造实体按键反馈消息)。"""
        def task():
            with self._state_transition_lock:
                before = "<unknown>"
                try:
                    before = self.robot_state_machine.state
                    self._state_transition_executing = True
                    getattr(self.robot_state_machine, trigger)(**kwargs)
                    after = self.robot_state_machine.state
                    print("========== 模式切换 ==========", flush=True)
                    print(f"调用功能: {func_name or trigger}", flush=True)
                    print(f"上一个模式: {before}", flush=True)
                    print(f"当前模式: {after}", flush=True)
                    print("==================================", flush=True)
                    if after != before:
                        rospy.loginfo(
                            f"[G11Screen] trigger '{trigger}': {before} -> {after}")
                    else:
                        rospy.logwarn(
                            f"[G11Screen] trigger '{trigger}' executed but state unchanged "
                            f"(still {after}); maybe blocked by conditions")
                    try:
                        rospy.set_param(LAST_STATE_PARAM, self.robot_state_machine.state)
                    except Exception:
                        pass
                    # 离开 stance 自动关闭头部控制并清 latch(与实体键路径口径一致;
                    # 否则屏幕切到 walk 等状态后 head_control_mode 会残留为 True)
                    if self.robot_state_machine.state != "stance" and self.head_control_mode:
                        rospy.logwarn("[HeadControl] Current state is not 'stance'. Disabling head control mode.")
                        self.head_control_mode = False
                        self._publish_head_vel(0.0, 0.0)
                        self._head_vel_latched = False
                except Exception as e:
                    print("========== 状态切换失败 ==========", flush=True)
                    print(f"调用功能: {func_name or trigger}", flush=True)
                    print(f"上一个模式: {before}", flush=True)
                    print("当前模式: 执行失败", flush=True)
                    print(f"失败原因: {e}", flush=True)
                    print("==================================", flush=True)
                    rospy.logerr(f"[G11Screen] state transition task error: {e}")
                finally:
                    self._state_transition_executing = False
        self.executor.submit(task)

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
        # 重要修复：确保在头部控制模式下也处理摇杆输入
        if self.head_control_mode and current_state == "stance":
            self._handle_head_control(msg)
    #zsh
    def _normalize_channel(self, value: int) -> float:
        """将通道值标准化到[-1.0, 1.0]范围"""
        # 确保值在有效范围内
        clamped = max(min(value, Config.H12_AXIS_RANGE_MAX), Config.H12_AXIS_RANGE_MIN)
        # 映射到[-1.0, 1.0]
        return (clamped - Config.H12_AXIS_MID_VALUE) / (Config.H12_AXIS_RANGE / 2)
    #zsh
    def _handle_head_control(self, msg: h12proRemoteControllerChannel):
        try:
            yaw_input = self._normalize_channel(msg.channels[0])  # 左右
            pitch_input = self._normalize_channel(msg.channels[1]) # 上下

            if abs(yaw_input) < self.head_stick_deadzone:
                yaw_input = 0.0
            if abs(pitch_input) < self.head_stick_deadzone:
                pitch_input = 0.0

            # 摇杆 → 瞬时角速度（deg/s）；开环积分在接收侧
            yaw_vel = yaw_input * self.head_vel_scale_yaw
            pitch_vel = pitch_input * self.head_vel_scale_pitch

            if abs(yaw_vel) < 1e-6 and abs(pitch_vel) < 1e-6:
                # Stick release: one zero to clear sticky velocity latch (then stop publishing)
                if self._head_vel_latched:
                    self._publish_head_vel(0.0, 0.0)
                    self._head_vel_latched = False
                return
            self._head_vel_latched = True
            self._publish_head_vel(yaw_vel, pitch_vel)

        except Exception as e:
            rospy.logerr(f"Head control error: {str(e)}")

    def _publish_head_vel(self, yaw_vel: float, pitch_vel: float) -> None:
        """发布瞬时头部角速度（deg/s）到 /cmd_head_vel。"""
        vel_msg = robotHeadMotionData()
        vel_msg.joint_data = [yaw_vel, pitch_vel]
        self.cmd_head_vel_pub.publish(vel_msg)



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

    def _get_current_controller_name(self) -> Optional[str]:
        """获取当前控制器名称
        
        Returns:
            当前控制器名称，如果获取失败返回 None
        """
        service_name = "/humanoid_controller/get_controller_list"
        try:
            rospy.wait_for_service(service_name, timeout=1.0)
            get_controller_client = rospy.ServiceProxy(service_name, getControllerList)
            response = get_controller_client()
            if response.success:
                current_controller = response.current_controller
                rospy.loginfo(f"Current controller: {current_controller} (index: {response.current_index})")
                return current_controller
            else:
                rospy.logwarn(f"Get controller list failed: {response.message}")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to '{service_name}' failed: {e}")
            return None
        except rospy.ROSException as e:
            rospy.logerr(f"Service '{service_name}' not available: {e}")
            return None

    def _handle_emergency_stop(self, current_state: str, 
                             msg: h12proRemoteControllerChannel) -> None:
        """Handle emergency stop condition.
        
        Args:
            current_state: Current robot state.
            msg: Channel message for response.
        """
        try:
            # 紧急停止时，清除 switch_controller 的冷却期，确保可以立即停止
            if kuavo_control_scheme == "multi":
                clear_switch_controller_cooldown()
                rospy.set_param("/sit_stand_abort", True)
                rospy.loginfo("[EmergencyStop] Cleared switch_controller cooldown, set sit_stand_abort.")

            # 检查当前控制器是否为 mpc，只有 mpc 控制器支持缓慢下降
            current_controller = None
            skip_controller_list = self.only_half_up_body or rospy.get_param("robot_type", 2) == 1
            if not skip_controller_list:
                current_controller = self._get_current_controller_name()
            if current_controller and current_controller.lower() == "mpc":
                if current_state in ["stance", "walk", "trot", "vr_remote_control"]:
                    self.h12_to_joy_node.is_stopping = True
                    self._gradually_move_right_stick_down()
                    self.h12_to_joy_node.is_stopping = False
            
            # 发布全局停止指令到/stop_robot话题（True表示停止，项目统一协议）
            self.stop_robot_pub.publish(Bool(data=True))

            # 无论当前状态是什么，急停时统一清理 hardware prep 相关参数，
            # 避免硬件节点在后续重新启动时读到残留的蹲姿关节角导致站姿启动变蹲姿。
            # hardware_node.cc L342-359: 若 /hardware_prep_joint_pos_deg 残留且 size==num_joint，
            # 会直接覆盖 squat_joint_pos_ → moving_pos → 蹲姿启动，即使 /use_sit_init=false 也无效。
            _HARDWARE_PREP_KEYS = (
                "/use_sit_init",
                "/hardware_sit_prep_ready",
                "/hardware_prep_joint_pos_deg",
                "/hardware_prep_joint_pos_offset_deg",
                "/hardware_prep_joint_pos_leg_first_deg",
                "/hardware_prep_joint_pos_arm_clear_deg",
                "/hardware_prep_moves",
                "/hardware_prep_two_phase",
                "/hardware_prep_reverse_leg_first",
                "/hardware_prep_reverse_arm_clear",
            )
            for key in _HARDWARE_PREP_KEYS:
                try:
                    rospy.delete_param(key)
                except KeyError:
                    pass
            rospy.loginfo("[EmergencyStop] Cleaned all /hardware_prep_* and /use_sit_init params.")

            # 当 current_state 为 initial 时（sit_to_stand_callback 在线程池中运行），
            # stop(source="initial") 会抛 MachineError 导致 stop_callback 不被调用。
            # 在此额外处理：杀死 tmux 会话并强制 FSM 回到 initial。
            if current_state == "initial":
                import subprocess as _subprocess_emergency
                # sit_to_stand 长按A 启动时会启动 WiFi 上报；先停止避免残留线程
                try:
                    from robot_state.multi_before_callback import stop_wifi_info_report
                    stop_wifi_info_report()
                except (ImportError, Exception):
                    pass
                _subprocess_emergency.run(
                    ["tmux", "kill-session", "-t", "humanoid_robot"],
                    stderr=_subprocess_emergency.DEVNULL,
                )
                # 强制 FSM 回到 initial，避免 sit_to_stand_callback 返回后
                # transitions 将状态改为 stance
                self.robot_state_machine.machine.set_state(
                    "initial", self.robot_state_machine,
                )
                rospy.set_param(LAST_STATE_PARAM, "initial")
                rospy.logwarn(
                    "[EmergencyStop] current_state=initial, "
                    "killed humanoid_robot tmux session, forced FSM to initial.",
                )

            getattr(self.robot_state_machine, "stop")(source=current_state)

            # ===== 紧急停止状态持久化 =====
            try:
                rospy.set_param(LAST_STATE_PARAM, self.robot_state_machine.state)
                rospy.loginfo(f"[StatePersistence] Emergency stop persisted: {self.robot_state_machine.state}")
            except Exception as e:
                rospy.logerr(f"[StatePersistence] Failed to persist: {e}")

            stop_msg = h12proRemoteControllerChannel()
            channels = Config.get_default_channels()
            channels[Config.TRIGGER_CHANNEL_MAP["stop"]] = Config.MINUS_H12_AXIS_RANGE_MAX
            stop_msg.channels = tuple(channels)
            
            self.h12_to_joy_node.update_channels_msg(msg=stop_msg)
            # 急停信号一次性下发,绕过周期限频,保证即时生效
            self.h12_to_joy_node.process_channels(publish_immediately=True)
            return
            
        except MachineError as e:
            return

    def _handle_normal_transitions(self, current_state: str, 
                                key_combination: Set[str],
                                msg: h12proRemoteControllerChannel) -> None:
        """Handle normal state transitions."""
        # 检查是否在 switch_controller 冷却期内
        if kuavo_control_scheme == "multi":
            if is_switch_controller_in_cooldown():
                rospy.logdebug("[StateTransition] Blocked: switch_controller is in cooldown period.")
                return

        triggers = self.robot_state_machine.machine.get_triggers(current_state)
        
        for trigger in self._config["state_transitions"].get(current_state, {}):
            trigger_keys = set(self._config["state_transitions"][current_state][trigger])
            
            if not trigger_keys.issubset(key_combination):
                continue

            if trigger == "toggle_head_control":
                if current_state == "stance":
                    self.head_control_mode = not self.head_control_mode
                    #zsh
                    # Exit head control: one zero to clear sticky latch (enter needs no zero)
                    if not self.head_control_mode:
                        self._publish_head_vel(0.0, 0.0)
                        self._head_vel_latched = False
                    #zsh
                    rospy.loginfo(f"[HeadControl] Head control mode {'enabled' if self.head_control_mode else 'disabled'}")
                else:
                    if self.head_control_mode:
                        rospy.logwarn("[HeadControl] Exiting stance state. Head control mode disabled.")
                        self.head_control_mode = False
                        #zsh
                        # Leave stance: zero vel to clear sticky latch
                        self._publish_head_vel(0.0, 0.0)
                        self._head_vel_latched = False
                        #zsh
                return

            
            if trigger not in triggers:
                continue

            try:
                self._execute_state_transition(trigger, current_state, msg)
                return
            except Exception as e:
                rospy.logerr(f"Error during state transition: {e}")

        if current_state != "vr_remote_control" and not self.head_control_mode and not self.is_navigation_mode:
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

        # 检查是否有状态转换正在执行，如果有则直接拒绝（不排队）
        if self._state_transition_executing:
            rospy.logwarn(f"[StateTransition] Trigger '{trigger}' rejected: Another state transition is already executing")
            return

        # 对于arm_pose和customize_action相关的trigger，检查是否有arm动作正在执行
        if ("arm_pose" in trigger or "customize_action" in trigger) and self.robot_action_executing:
            print(f"[StateTransition] Trigger '{trigger}' rejected: Arm action is currently executing (robot_action_executing={self.robot_action_executing})")
            return
        elif ("arm_pose" in trigger or "customize_action" in trigger):
            print(f"[StateTransition] Trigger '{trigger}' accepted: robot_action_executing={self.robot_action_executing}, will submit to thread pool")

        # 提交到线程池执行状态转换
        def state_transition_task():
            with self._state_transition_lock:
                try:
                    self._state_transition_executing = True  # 标记开始执行
                    getattr(self.robot_state_machine, trigger)(**kwargs)

                    # ===== 状态持久化 (新增) =====
                    try:
                        rospy.set_param(LAST_STATE_PARAM, self.robot_state_machine.state)
                        rospy.loginfo(f"[StatePersistence] Persisted state: {self.robot_state_machine.state}")
                    except Exception as e:
                        rospy.logerr(f"[StatePersistence] Failed to persist state: {e}")
                    # =====================================================

                                        # zsh如果不是stance状态，自动关闭头部控制模式
                    if self.robot_state_machine.state != "stance" and self.head_control_mode:
                        rospy.logwarn("[HeadControl] Current state is not 'stance'. Disabling head control mode.")
                        self.head_control_mode = False
                        #zsh
                        # Auto-disable outside stance: zero vel to clear sticky latch
                        self._publish_head_vel(0.0, 0.0)
                        self._head_vel_latched = False
                        #zsh

                    # 如果是有效状态,更新消息
                    current_controller_support = True
                    current_controller = None
                    skip_controller_list = self.only_half_up_body or rospy.get_param("robot_type", 2) == 1
                    if not skip_controller_list:
                        current_controller = self._get_current_controller_name()
                    if current_controller and current_controller.lower() == "mpc" and trigger in ["trot"]:
                        current_controller_support = False
                        print("mpc not support this trigger")

                    if trigger in Config.VALID_STATES and current_controller_support:
                        new_msg = h12proRemoteControllerChannel()
                        channels = Config.get_default_channels()

                        channels[Config.TRIGGER_CHANNEL_MAP[trigger]] = Config.H12_AXIS_RANGE_MAX
                        new_msg.channels = tuple(channels)
                        
                        self.h12_to_joy_node.update_channels_msg(msg=new_msg)
                        # 状态切换触发信号一次性下发,绕过周期限频,保证即时生效
                        self.h12_to_joy_node.process_channels(publish_immediately=True)
                except Exception as e:
                    rospy.logerr(f"Error in state transition task: {e}")
                finally:
                    self._state_transition_executing = False  # 清除执行标志

        self.executor.submit(state_transition_task)
#zsh
    def _handle_joystick_input(self, msg: h12proRemoteControllerChannel) -> None:
        """Handle joystick input when no state transition occurs."""
        # 如果当前状态是stance，且头部控制模式开启，则处理摇杆输入
        # rospy.loginfo(f"[JoystickInput] head_control_mode={self.head_control_mode}")#日志打印测试是否进入head

        # stick_channels = Config.get_default_channels()
        # stick_channels[:4] = msg.channels[:4]

        # stick_msg = h12proRemoteControllerChannel()
        # stick_msg.channels = tuple(stick_channels)

        # self.h12_to_joy_node.update_channels_msg(msg=stick_msg)
        # self.h12_to_joy_node.process_channels()

        if (self.only_half_up_body or is_switch_controller_in_cooldown()
                or self.robot_action_executing or self.robot_state_machine.state == "sit"
                or is_sit_stand_in_progress()):
            neutral_msg = h12proRemoteControllerChannel()
            channels = Config.get_default_channels()
            neutral_msg.channels = tuple(channels)
            self.h12_to_joy_node.update_channels_msg(msg=neutral_msg)
            self.h12_to_joy_node.process_channels()

            reasons = []
            if self.robot_state_machine.state == "sit":
                reasons.append("sit state")
            if is_switch_controller_in_cooldown():
                reasons.append("switch_controller cooldown")
            if self.robot_action_executing:
                reasons.append("tact action executing")
            rospy.logdebug(f"[JoystickInput] Blocked: {' and '.join(reasons)}. Publishing neutral joystick values.")
            return

        # 头部控制模式下处理摇杆控制
        if not self.head_control_mode:
            if self.h12_to_joy_node.is_wheel:
                # G12轮臂模式: 传递全部12通道（包含E/F安全开关、A/B/C/D按钮、G/H拨杆）
                full_msg = h12proRemoteControllerChannel()
                full_msg.channels = msg.channels
                self.h12_to_joy_node.update_channels_msg(msg=full_msg)
            else:
                # 双足模式: 只传递前4通道（摇杆），按钮由状态机直接处理
                stick_channels = Config.get_default_channels()
                stick_channels[:4] = msg.channels[:4]
                stick_msg = h12proRemoteControllerChannel()
                stick_msg.channels = tuple(stick_channels)
                self.h12_to_joy_node.update_channels_msg(msg=stick_msg)
            self.h12_to_joy_node.process_channels()
#zsh
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
            # ocs2 和 multi 模式都需要处理 channels
            if kuavo_control_scheme == "ocs2" or kuavo_control_scheme == "multi":
                node.h12_to_joy_node.process_channels()
            node.publish_arm_joint_state()
            rate.sleep()
            
    except rospy.exceptions.ROSInterruptException:
        # 正常关闭路径（rosnode kill / roslaunch 停机 / 让位），不作为错误处理
        pass
    except Exception as e:
        rospy.logerr(f"Error in main loop: {e}")
        raise
    finally:
        rospy.loginfo("Cleaning up...")
        rospy.signal_shutdown("Node shutting down")

if __name__ == '__main__':
    main()
