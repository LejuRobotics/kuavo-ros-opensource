#!/usr/bin/env python3
import rospy
import time
from h12pro_controller_node.msg import h12proRemoteControllerChannel

MIN_VALUE = 282
MID_VALUE = 1002
MAX_VALUE = 1722
DEFAULT_CHANNELS = [MIN_VALUE] * 12
DEFAULT_CHANNELS[:4] = [MID_VALUE] * 4

STATE_TO_CHANNEL = {
    "E_LEFT": {
        "index": 5,
        "value": MIN_VALUE
    },
    "E_MIDDLE": {
        "index": 5,
        "value": MID_VALUE
    },
    "E_RIGHT": {
        "index": 5,
        "value": MAX_VALUE
    },
    "F_LEFT": {
        "index": 6,
        "value": MIN_VALUE
    },
    "F_MIDDLE": {
        "index": 6,
        "value": MID_VALUE
    },
    "F_RIGHT": {
        "index": 6,
        "value": MAX_VALUE
    },
    "A": {
        "index": 7,
        "value": MAX_VALUE
    },
    "B": {
        "index": 8,
        "value": MAX_VALUE
    },
    "C": {
        "index": 9,
        "value": MAX_VALUE
    },
    "D": {
        "index": 10,
        "value": MAX_VALUE
    }
}

def publish_channel_msg(states, long_press=False):
    # 初始化 ROS 节点
    rospy.init_node('publish_channel_msg', anonymous=True)
    pub = rospy.Publisher('/h12pro_channel', h12proRemoteControllerChannel, queue_size=10)

    while pub.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscribers to connect...")
        rospy.sleep(0.5)

    # 创建一个新的消息
    msg = h12proRemoteControllerChannel()

    # 发布默认值
    msg.channels = DEFAULT_CHANNELS.copy()  # 使用默认值
    msg.sbus_state = 1
    pub.publish(msg)
    rospy.loginfo(f"Published default channel message: {msg.channels}")



    # 将输入状态映射到通道值
    for state in states:
        if state in STATE_TO_CHANNEL:
            index = STATE_TO_CHANNEL[state]["index"] - 1
            print(index)
            msg.channels[index] = STATE_TO_CHANNEL[state]["value"]

    # 保持 E 和 F 的值不变
    e_value = msg.channels[4]
    f_value = msg.channels[5]
    # 发布触发的消息
    pub.publish(msg)
    rospy.loginfo(f"Published triggered channel message: {msg.channels}")

    # 如果是长按，添加延时
    if long_press:
        time.sleep(1.1)
    else:
        time.sleep(0.1)

    # 发布回到默认值，但保持 E 和 F 的值不变
    reset_msg = DEFAULT_CHANNELS.copy()  # 创建一个新的重置消息
    # 保持 E 和 F 的值不变
    reset_msg[4] = e_value  # E
    reset_msg[5] = f_value  # F
    msg.channels = reset_msg  # 更新消息通道
    pub.publish(msg)
    rospy.loginfo(f"Published reset channel message: {msg.channels}")

if __name__ == '__main__':
    try:
        # 示例调用
        publish_channel_msg(["A", "B"], long_press=True)
    except rospy.ROSInterruptException:
        pass