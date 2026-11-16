#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DH 夹爪 topic 控制示例（整机 EndEffectorType=dhclaw）。

独立 /dh_claw_* 话题：
  command: /dh_claw_command
  state:   /dh_claw_state

量纲（DH 百分比）：
  position: 0~100
  velocity: 1~100   （速度 %）
  effort:   20~100  （力 %；>=20 直接按 DH 力下发）

单侧 / 双侧：
  改 TARGET_SIDE，或运行时：
    rosrun ... dh_claw_pub.py _side:=left
    rosrun ... dh_claw_pub.py _side:=right
    rosrun ... dh_claw_pub.py _side:=both
  也可用命令行：python3 dh_claw_pub.py left
"""
import sys
import time
import rospy
from kuavo_msgs.msg import dhClawState, dhClawCommand

# DH 默认速度/力（百分比）
DH_SPEED = 100     # 速度 1~100
DH_FORCE = 100     # 力 20~100
# 开合位置（0~100）
POS_OPEN = 10
POS_CLOSE = 90
# topic 连续开合周期；过小会刷指令，建议 >=1.0s
TOGGLE_PERIOD_S = 1.5

# 默认只发一侧： "left" | "right" | "both"
TARGET_SIDE = "both"

claw_state = [dhClawState.kUnknown, dhClawState.kUnknown]
pub = rospy.Publisher("/dh_claw_command", dhClawCommand, queue_size=10)


def _normalize_side(side):
    s = str(side).strip().lower()
    aliases = {
        "l": "left",
        "left": "left",
        "left_claw": "left",
        "r": "right",
        "right": "right",
        "right_claw": "right",
        "both": "both",
        "all": "both",
        "lr": "both",
    }
    if s not in aliases:
        raise ValueError("side must be left/right/both, got: %r" % side)
    return aliases[s]


def pub_dh_claw_command(pos, side=None, vel=None, effort=None):
    """
    pos:
      - side=left/right: 标量或单元素列表
      - side=both: [left_pos, right_pos]
    """
    side = _normalize_side(side if side is not None else TARGET_SIDE)

    if side == "both":
        if isinstance(pos, (int, float)):
            positions = [float(pos), float(pos)]
        else:
            positions = list(map(float, pos))
            if len(positions) == 1:
                positions = [positions[0], positions[0]]
            if len(positions) < 2:
                raise ValueError("both 模式需要 2 个 position")
            positions = positions[:2]
        names = ["left_claw", "right_claw"]
        velocities = [DH_SPEED, DH_SPEED] if vel is None else list(map(float, vel))
        efforts = [DH_FORCE, DH_FORCE] if effort is None else list(map(float, effort))
        if len(velocities) < 2:
            velocities = (velocities + [DH_SPEED, DH_SPEED])[:2]
        if len(efforts) < 2:
            efforts = (efforts + [DH_FORCE, DH_FORCE])[:2]
    else:
        # 单侧：消息里只带这一侧 name，plant 不会驱动另一侧
        p = float(pos[0] if isinstance(pos, (list, tuple)) else pos)
        name = "left_claw" if side == "left" else "right_claw"
        names = [name]
        positions = [p]
        velocities = [float(vel[0])] if isinstance(vel, (list, tuple)) else [
            float(DH_SPEED if vel is None else vel)
        ]
        efforts = [float(effort[0])] if isinstance(effort, (list, tuple)) else [
            float(DH_FORCE if effort is None else effort)
        ]

    msg = dhClawCommand()
    msg.data.name = names
    msg.data.position = positions
    msg.data.velocity = velocities
    msg.data.effort = efforts
    pub.publish(msg)
    rospy.loginfo(
        "publish /dh_claw_command side=%s name=%s pos=%s vel=%s force=%s",
        side,
        names,
        positions,
        velocities,
        efforts,
    )


def claw_state_callback(msg):
    global claw_state
    claw_state = list(msg.state)


if __name__ == "__main__":
    rospy.init_node("dh_claw_pub_node")

    # 优先级：命令行参数 > ROS param ~side > 文件内 TARGET_SIDE
    side = TARGET_SIDE
    argv = [a for a in sys.argv[1:] if not a.startswith("__") and not a.startswith("_")]
    if argv:
        side = argv[0]
    side = rospy.get_param("~side", side)
    side = _normalize_side(side)

    rospy.Subscriber("/dh_claw_state", dhClawState, claw_state_callback)
    time.sleep(1.0)

    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("Waiting for /dh_claw_command subscriber...")
        rospy.sleep(0.1)

    rospy.loginfo(
        "DH claw topic demo: side=%s open/close=%.0f/%.0f, speed=%.0f%%, force=%.0f%%, period=%.1fs",
        side,
        POS_OPEN,
        POS_CLOSE,
        DH_SPEED,
        DH_FORCE,
        TOGGLE_PERIOD_S,
    )
    pub_dh_claw_command(POS_OPEN, side=side)

    option_open = False
    while not rospy.is_shutdown():
        if option_open:
            rospy.loginfo("open claw (%s)", side)
            pub_dh_claw_command(POS_OPEN, side=side)
            option_open = False
        else:
            rospy.loginfo("close claw (%s)", side)
            pub_dh_claw_command(POS_CLOSE, side=side)
            option_open = True
        time.sleep(TOGGLE_PERIOD_S)
