#!/usr/bin/env python3
import time
import rospy
from sensor_msgs.msg import Joy
from h12pro_controller_node.msg import h12proRemoteControllerChannel

H12_AXIS_RANGE_MAX = 1722
H12_AXIS_RANGE_MIN = 282
H12_AXIS_RANGE = H12_AXIS_RANGE_MAX - H12_AXIS_RANGE_MIN
H12_AXIS_MID_VALUE = (H12_AXIS_RANGE_MAX + H12_AXIS_RANGE_MIN) // 2

# JoyButton constants
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_BACK = 6
BUTTON_START = 7

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
        # 定义 channel 映射
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
            for index, channel_value in enumerate(self.channels_msg):
                mapping = self.channel_mapping.get(index + 1)
                if mapping:
                    if mapping.is_button:
                        # 处理按钮逻辑
                        self.joy_msg.buttons[mapping.button_index] = mapping.get_current_state(channel_value)
                    else:
                        # 处理轴逻辑
                        self.joy_msg.axes[mapping.axis_index] = mapping.get_current_state(channel_value)

            self.joy_pub.publish(self.joy_msg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('joy_node')
    node = H12ToJoyControllerNode()
    node.run()
