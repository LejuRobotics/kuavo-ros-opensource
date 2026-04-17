#!/usr/bin/env python3
import time
import os
import rospy
from sensor_msgs.msg import Joy
from h12pro_controller_node.msg import h12proRemoteControllerChannel
try:
    from robot_version import RobotVersion
except ImportError:
    import rospkg
    rospack = rospkg.RosPack()
    import sys
    sys.path.insert(0, os.path.join(rospack.get_path('kuavo_common'), 'python'))
    from robot_version import RobotVersion

H12_AXIS_RANGE_MAX = 1722
H12_AXIS_RANGE_MIN = 282
H12_AXIS_RANGE = H12_AXIS_RANGE_MAX - H12_AXIS_RANGE_MIN
H12_AXIS_MID_VALUE = (H12_AXIS_RANGE_MAX + H12_AXIS_RANGE_MIN) // 2

# G/H滚轮极值判断阈值: 只在接近极值时认为激活
G12_DIAL_THRESHOLD = 50

# JoyButton constants
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_BACK = 6
BUTTON_START = 7
BUTTON_GUIDE = 8
BUTTON_M1 = 9
BUTTON_M2 = 10

# JoyAxis constants
AXIS_LEFT_STICK_Y = 0
AXIS_LEFT_STICK_X = 1
AXIS_LEFT_LT = 2  # 1 -> (-1)
AXIS_RIGHT_STICK_YAW = 3
AXIS_RIGHT_STICK_Z = 4
AXIS_RIGHT_RT = 5  # 1 -> (-1)
AXIS_LEFT_RIGHT_TRIGGER = 6
AXIS_FORWARD_BACK_TRIGGER = 7

class ChannelMapping:
    def __init__(self, channel, axis_index=None, button_index=None, is_button=False, reverse=False, trigger_value=None):
        self.channel = channel
        self.axis_index = axis_index
        self.button_index = button_index
        self.is_button = is_button
        self.reverse = reverse
        self.trigger_value = trigger_value
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

        value = (channel_value - H12_AXIS_MID_VALUE) / (H12_AXIS_RANGE//2)
        if self.reverse:
            value = -value
        return value

    def get_current_state(self, channel_value):
        if self.is_button:
            return 1 if self.update(channel_value) else 0
        else:
            return self.update(channel_value)

class H12ToJoyControllerNode:
    def __init__(self, *args, **kwargs) -> None:
        # 判断是否为轮臂机器人(6代: major=6)
        robot_version = os.environ.get('ROBOT_VERSION', '0')
        try:
            self.is_wheel = RobotVersion.create(int(robot_version)).start_with(major=6)
        except (ValueError, TypeError):
            self.is_wheel = False

        # 定义 channel 映射（双足模式默认映射）
        self.channel_mapping = {
            1: ChannelMapping(channel=1, axis_index=AXIS_RIGHT_STICK_YAW, reverse=True),
            2: ChannelMapping(channel=2, axis_index=AXIS_RIGHT_STICK_Z,reverse=True),
            3: ChannelMapping(channel=3, axis_index=AXIS_LEFT_STICK_X),
            4: ChannelMapping(channel=4, axis_index=AXIS_LEFT_STICK_Y,reverse=True),
            5: ChannelMapping(channel=5, button_index=BUTTON_BACK, is_button=True, trigger_value=H12_AXIS_RANGE_MAX),#E
            6: ChannelMapping(channel=6, button_index=BUTTON_START, is_button=True, trigger_value=H12_AXIS_RANGE_MIN),#F
            7: ChannelMapping(channel=7, button_index=BUTTON_Y, is_button=True, trigger_value=H12_AXIS_RANGE_MAX),#A
            8: ChannelMapping(channel=8, button_index=BUTTON_B, is_button=True, trigger_value=H12_AXIS_RANGE_MAX),#B
            # 9: ChannelMapping(channel=9, button_index=BUTTON_BACK, is_button=True, trigger_value=H12_AXIS_RANGE_MAX),#C
            10: ChannelMapping(channel=10, button_index=BUTTON_A, is_button=True, trigger_value=H12_AXIS_RANGE_MAX),#D
            # 11: ChannelMapping(channel=11, axis_index=None),#G
            # 12: ChannelMapping(channel=12, axis_index=None),#H

            # 添加其他 channel 的映射
        }

        # 轮臂模式: 覆盖部分映射
        if self.is_wheel:
            self.channel_mapping[9] = ChannelMapping(channel=9, button_index=BUTTON_X, is_button=True, trigger_value=H12_AXIS_RANGE_MAX)  # C
            # self.channel_mapping[5] = ChannelMapping(channel=5, axis_index=None)  # E安全开关
            # self.channel_mapping[6] = ChannelMapping(channel=6, axis_index=None)  # F安全开关
            rospy.loginfo("轮臂模式: ROBOT_VERSION=%s, 已加载G12按键映射", robot_version)

        # C+D长按急停检测状态
        self.cd_emergency_triggered = False
        self.cd_press_start_time = None

        self.callback_frequency = 100  # Hz
        self.last_time = time.time()
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 11
        self.channels_msg = None

        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
        self.h12pro_controller_channels_sub = rospy.Subscriber(
            "/h12pro_channel",
            h12proRemoteControllerChannel,
            self.h12pro_controller_channels_callback,
            queue_size=1,
        )

    def h12pro_controller_channels_callback(self, msg):
        channels = msg.channels
        self.channels_msg = channels


    def run(self):
        rate = rospy.Rate(self.callback_frequency)
        while not rospy.is_shutdown():
            # 重置 joy_msg
            self.joy_msg.axes = [0.0] * 8
            self.joy_msg.buttons = [0] * 11
            if self.channels_msg is None:
                continue

            if self.is_wheel:
                self._process_wheel_channels()
            else:
                self._process_biped_channels()

            self.joy_pub.publish(self.joy_msg)
            rate.sleep()

    def _process_biped_channels(self):
        """双足模式：使用标准channel映射"""
        for index, channel_value in enumerate(self.channels_msg):
            mapping = self.channel_mapping.get(index + 1)
            if mapping:
                if mapping.is_button:
                    self.joy_msg.buttons[mapping.button_index] = mapping.get_current_state(channel_value)
                elif mapping.axis_index is not None:
                    self.joy_msg.axes[mapping.axis_index] = mapping.get_current_state(channel_value)

    def _process_wheel_channels(self):
        """轮臂模式(G12)特殊处理"""
        channels = list(self.channels_msg)

        # E/F安全开关: 必须都在中间位置才生效
        e_mid = abs(channels[4] - H12_AXIS_MID_VALUE) < 100
        f_mid = abs(channels[5] - H12_AXIS_MID_VALUE) < 100
        safe_enabled = e_mid and f_mid

        # G/H滚轮极值检测
        g_value = channels[10]
        g_at_extreme = (g_value <= H12_AXIS_RANGE_MIN + G12_DIAL_THRESHOLD or
                        g_value >= H12_AXIS_RANGE_MAX - G12_DIAL_THRESHOLD)
        h_value = channels[11]
        h_at_extreme = (h_value <= H12_AXIS_RANGE_MIN + G12_DIAL_THRESHOLD or
                        h_value >= H12_AXIS_RANGE_MAX - G12_DIAL_THRESHOLD)

        # 摇杆映射 (channel 1-4) 始终处理
        for index in [0, 1, 2, 3]:
            mapping = self.channel_mapping.get(index + 1)
            if mapping and mapping.axis_index is not None:
                self.joy_msg.axes[mapping.axis_index] = mapping.get_current_state(channels[index])

        if not safe_enabled:
            return

        # 安全开关已启用
        self.joy_msg.buttons[BUTTON_LB] = 1
        self.joy_msg.buttons[BUTTON_RB] = 1

        # C+D长按急停 (channel 9->index 8, channel 10->index 9)
        c_pressed = channels[8] == H12_AXIS_RANGE_MAX
        d_pressed = channels[9] == H12_AXIS_RANGE_MAX
        if c_pressed and d_pressed:
            if self.cd_press_start_time is None:
                self.cd_press_start_time = time.time()
            elif time.time() - self.cd_press_start_time >= 1.0 and not self.cd_emergency_triggered:
                self.joy_msg.buttons[BUTTON_BACK] = 1
                self.cd_emergency_triggered = True
        else:
            self.cd_press_start_time = None
            self.cd_emergency_triggered = False

        # 按键映射 (C->9, A->7, B->8, D->10)
        for ch_idx, btn_ch in [(8, 9), (6, 7), (7, 8), (9, 10)]:
            mapping = self.channel_mapping.get(btn_ch)
            if mapping and mapping.is_button:
                self.joy_msg.buttons[mapping.button_index] = mapping.get_current_state(channels[ch_idx])

        # G/H滚轮按钮
        if g_at_extreme:
            self.joy_msg.buttons[BUTTON_GUIDE] = 1
        if h_at_extreme:
            self.joy_msg.buttons[BUTTON_M1] = 1

        # G+H同时极值2秒 -> 躯干复位
        if g_at_extreme and h_at_extreme:
            if not hasattr(self, 'gh_press_start_time') or self.gh_press_start_time is None:
                self.gh_press_start_time = time.time()
            elif time.time() - self.gh_press_start_time >= 2.0:
                self.joy_msg.buttons[BUTTON_M2] = 1
        else:
            self.gh_press_start_time = None

if __name__ == '__main__':
    rospy.init_node('joy_node')
    node = H12ToJoyControllerNode()
    node.run()
