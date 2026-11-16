#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dhclaw_pub.py -- DH 夹爪 ROS 测试客户端

配合 dhclaw_ros_node 使用（独立 /dh_claw_* 话题）:
  发布  /dh_claw_command  (kuavo_msgs/dhClawCommand)
  订阅  /dh_claw_state    (kuavo_msgs/dhClawState)

position: 0=全开 .. 100=全关
velocity: 1~100  (对应 DH 速度%)
effort:   建议 1.0~2.0  (ros_node 映射到力 20~100；也兼容直接传 20~100)

用法:
  # 终端1
  roscore
  # 终端2 (共总线默认 canbus0 ID=17 @1M)
  rosrun hardware_node dhclaw_ros_node
  # 或
  rosrun hardware_node dhclaw_ros_node _use_config:=true _device_name:=Llejuclaw
  # 终端3
  python3 dhclaw_pub.py
  python3 dhclaw_pub.py --open 10 --close 90 --period 2.0 --speed 50 --effort 2.0
"""

from __future__ import print_function

import argparse
import sys
import time

import rospy
from kuavo_msgs.msg import dhClawState, dhClawCommand

STATE_NAME = {
    dhClawState.kError: "Error",
    dhClawState.kUnknown: "Unknown",
    dhClawState.kMoving: "Moving",
    dhClawState.kReached: "Reached",
    dhClawState.kGrabbed: "Grabbed",
    dhClawState.kDropped: "Dropped",
}

claw_state = [dhClawState.kUnknown, dhClawState.kUnknown]
claw_pos = [0.0, 0.0]


def state_callback(msg):
    global claw_state, claw_pos
    if msg.state:
        claw_state = list(msg.state)
        while len(claw_state) < 2:
            claw_state.append(claw_state[-1])
    if hasattr(msg, "data") and msg.data.position:
        claw_pos = list(msg.data.position)
        while len(claw_pos) < 2:
            claw_pos.append(claw_pos[-1])


def publish_cmd(pub, pos, vel, effort):
    msg = dhClawCommand()
    msg.data.name = ["left_claw", "right_claw"]
    # 单爪 ros_node 使用 position[1](right)；左右同值即可
    if isinstance(pos, (int, float)):
        pos = [float(pos), float(pos)]
    if isinstance(vel, (int, float)):
        vel = [float(vel), float(vel)]
    if isinstance(effort, (int, float)):
        effort = [float(effort), float(effort)]
    msg.data.position = pos
    msg.data.velocity = vel
    msg.data.effort = effort
    pub.publish(msg)


def fmt_state():
    names = [STATE_NAME.get(s, str(s)) for s in claw_state[:2]]
    poss = ["{:.0f}%".format(p) for p in claw_pos[:2]]
    return "state={} pos={}".format(names, poss)


def main():
    parser = argparse.ArgumentParser(description="DH claw ROS pub test client")
    parser.add_argument("--open", type=float, default=10.0, help="open position 0~100")
    parser.add_argument("--close", type=float, default=90.0, help="close position 0~100")
    parser.add_argument("--mid", type=float, default=50.0, help="initial mid position")
    parser.add_argument("--speed", type=float, default=50.0, help="velocity 1~100")
    parser.add_argument(
        "--effort",
        type=float,
        default=2.0,
        help="effort (1.0~2.0 maps to force; or 20~100 raw)",
    )
    parser.add_argument("--period", type=float, default=2.5, help="cycle period seconds")
    parser.add_argument("--once", action="store_true", help="only go to mid then exit")
    parser.add_argument("--cycles", type=int, default=0, help="0=infinite")
    args = parser.parse_args()

    rospy.init_node("dhclaw_pub_client", anonymous=True)
    pub = rospy.Publisher("/dh_claw_command", dhClawCommand, queue_size=10)
    rospy.Subscriber("/dh_claw_state", dhClawState, state_callback)

    rospy.loginfo(
        "dhclaw_pub: wait for /dh_claw_command subscriber (dhclaw_ros_node) ..."
    )
    timeout = rospy.Time.now() + rospy.Duration(15.0)
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        if rospy.Time.now() > timeout:
            rospy.logerr(
                "No subscriber on /dh_claw_command. "
                "Please start: rosrun hardware_node dhclaw_ros_node"
            )
            return 1
        rospy.sleep(0.1)

    rospy.loginfo(
        "subscriber online. mid=%.0f open=%.0f close=%.0f speed=%.0f effort=%.2f",
        args.mid, args.open, args.close, args.speed, args.effort,
    )

    publish_cmd(pub, args.mid, args.speed, args.effort)
    rospy.sleep(1.0)
    print("[mid] ", fmt_state())

    if args.once or rospy.is_shutdown():
        return 0

    option_open = False
    cycle = 0
    rate_sleep = max(0.1, float(args.period))

    while not rospy.is_shutdown():
        if args.cycles > 0 and cycle >= args.cycles:
            break
        cycle += 1

        if option_open:
            target = args.open
            label = "open"
            option_open = False
        else:
            target = args.close
            label = "close"
            option_open = True

        print("[cycle {}] {} -> {:.0f}% | {}".format(cycle, label, target, fmt_state()))
        publish_cmd(pub, target, args.speed, args.effort)
        time.sleep(rate_sleep)

    if not rospy.is_shutdown():
        print("[exit] mid {:.0f}%".format(args.mid))
        publish_cmd(pub, args.mid, args.speed, args.effort)
        rospy.sleep(0.5)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main() or 0)
    except rospy.ROSInterruptException:
        pass
