#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DH 夹爪 service 控制示例（整机 EndEffectorType=dhclaw）。

独立 /dh_claw_* 话题和服务：
  service: /control_robot_dh_claw
  state:   /dh_claw_state

量纲（与 DhClawController 一致，直接用 DH 百分比，不走 leju 电流映射）：
  position: 0~100   （开→合）
  velocity: 1~100   （DH 速度 %）
  effort:   20~100  （DH 力 %；>=20 时按力百分比下发）

侧别（name 决定 enable，未出现的侧不会被驱动）：
  both  / all  : left_claw + right_claw
  left  / l    : 仅 left_claw
  right / r    : 仅 right_claw

示例：
  python3 dh_claw_client.py              # 默认双侧开合 demo
  python3 dh_claw_client.py --side left
  python3 dh_claw_client.py --side right
  python3 dh_claw_client.py --side both --once open
  python3 dh_claw_client.py --side left --once close --pos-close 85
"""
import argparse
import time
import rospy
from kuavo_msgs.srv import controlLejuClaw, controlLejuClawRequest
from kuavo_msgs.msg import dhClawState

# DH 默认速度/力（百分比）
DH_SPEED = 100     # 速度 1~100
DH_FORCE = 100     # 力 20~100
# 开合位置（0~100）
POS_OPEN = 10
POS_CLOSE = 90
# 连续开合周期；过小会刷指令，建议 >=1.0s
TOGGLE_PERIOD_S = 1.5

LEFT_NAME = "left_claw"
RIGHT_NAME = "right_claw"

claw_state = [dhClawState.kUnknown, dhClawState.kUnknown]


def normalize_side(side):
    s = (side or "both").strip().lower()
    if s in ("both", "all", "lr", "l+r", "dual"):
        return "both"
    if s in ("left", "l", "left_claw"):
        return "left"
    if s in ("right", "r", "right_claw"):
        return "right"
    raise ValueError("side must be both|left|right, got {!r}".format(side))


def side_names(side):
    side = normalize_side(side)
    if side == "left":
        return [LEFT_NAME]
    if side == "right":
        return [RIGHT_NAME]
    return [LEFT_NAME, RIGHT_NAME]


def side_indices(side):
    """state/position 双槽下标：0=left, 1=right。"""
    side = normalize_side(side)
    if side == "left":
        return [0]
    if side == "right":
        return [1]
    return [0, 1]


def _as_list_for_names(value, names, default_scalar):
    """把标量/列表对齐到 name 长度。"""
    n = len(names)
    if value is None:
        return [float(default_scalar)] * n
    if isinstance(value, (int, float)):
        return [float(value)] * n
    vals = list(value)
    if len(vals) == 1 and n > 1:
        return [float(vals[0])] * n
    if len(vals) < n:
        raise ValueError("expect {} values for {}, got {}".format(n, names, vals))
    return [float(v) for v in vals[:n]]


def call_dh_claw(pos, vel=None, effort=None, side="both"):
    """按 side 下发：both / left / right。

    pos/vel/effort:
      - 标量：该侧（或双侧）同一值
      - 列表：长度与 name 一致；both 时为 [left, right]
    底层只 enable 消息里出现的 name，未出现侧保持不动。
    """
    names = side_names(side)
    try:
        positions = _as_list_for_names(pos, names, POS_OPEN)
        velocities = _as_list_for_names(vel, names, DH_SPEED)
        efforts = _as_list_for_names(effort, names, DH_FORCE)
    except ValueError as e:
        rospy.logerr("bad cmd args: %s", e)
        return False

    try:
        req = controlLejuClawRequest()
        req.data.name = list(names)
        req.data.position = positions
        req.data.velocity = velocities
        req.data.effort = efforts
        rospy.wait_for_service("/control_robot_dh_claw")
        proxy = rospy.ServiceProxy("/control_robot_dh_claw", controlLejuClaw)
        res = proxy(req)
        rospy.loginfo(
            "dhclaw side=%s name=%s pos=%s vel=%s force=%s -> success=%s msg=%s",
            normalize_side(side),
            names,
            positions,
            velocities,
            efforts,
            res.success,
            res.message,
        )
        return bool(res.success)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False


def claw_state_callback(msg):
    global claw_state
    claw_state = list(msg.state)


def motion_done(side="both"):
    """只看本次下发涉及的侧是否到位。

    - 目标侧仍在 kMoving → 未完成
    - 目标侧至少有一侧 >= kReached，且目标侧无 kMoving → 完成
    - 目标侧全是 kUnknown → 未完成（无反馈/离线）
    """
    idxs = side_indices(side)
    if not idxs or len(claw_state) < 2:
        return False
    states = [claw_state[i] for i in idxs]
    if any(s == dhClawState.kMoving for s in states):
        return False
    return any(s >= dhClawState.kReached for s in states)


def wait_motion_done(side="both", timeout_s=15.0, poll_s=0.05):
    """等到目标侧到位或超时；超时仍返回，避免 demo 死等。"""
    t0 = time.time()
    last_log = 0.0
    while not rospy.is_shutdown():
        if motion_done(side):
            return True
        now = time.time()
        if now - t0 >= timeout_s:
            rospy.logwarn(
                "wait motion timeout (%.1fs), side=%s state=%s — continue anyway",
                timeout_s,
                normalize_side(side),
                claw_state,
            )
            return False
        if now - last_log >= 2.0:
            rospy.loginfo(
                "waiting side=%s state=%s ...",
                normalize_side(side),
                claw_state,
            )
            last_log = now
        rospy.sleep(poll_s)
    return False


def parse_args():
    p = argparse.ArgumentParser(description="DH claw service demo (left/right/both)")
    p.add_argument(
        "--side",
        default="both",
        choices=["both", "all", "left", "l", "right", "r"],
        help="which claw(s) to command (default: both)",
    )
    p.add_argument(
        "--once",
        choices=["open", "close", "none"],
        default="none",
        help="send one open/close then exit; default loops open/close",
    )
    p.add_argument("--pos-open", type=float, default=POS_OPEN)
    p.add_argument("--pos-close", type=float, default=POS_CLOSE)
    p.add_argument("--speed", type=float, default=DH_SPEED)
    p.add_argument("--force", type=float, default=DH_FORCE)
    p.add_argument("--period", type=float, default=TOGGLE_PERIOD_S)
    p.add_argument("--timeout", type=float, default=15.0)
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    side = normalize_side(args.side)
    pos_open = float(args.pos_open)
    pos_close = float(args.pos_close)
    speed = float(args.speed)
    force = float(args.force)
    period = float(args.period)
    timeout = float(args.timeout)

    rospy.init_node("dh_claw_client_node")
    rospy.Subscriber("/dh_claw_state", dhClawState, claw_state_callback)
    time.sleep(1.0)

    rospy.loginfo(
        "DH claw demo: side=%s pos open/close=%.0f/%.0f speed=%.0f%% force=%.0f%%",
        side,
        pos_open,
        pos_close,
        speed,
        force,
    )

    if args.once in ("open", "close"):
        target = pos_open if args.once == "open" else pos_close
        call_dh_claw(target, vel=speed, effort=force, side=side)
        wait_motion_done(side=side, timeout_s=timeout)
        rospy.loginfo("done once=%s state=%s", args.once, claw_state)
        raise SystemExit(0)

    call_dh_claw(pos_open, vel=speed, effort=force, side=side)

    option_open = False
    while not rospy.is_shutdown():
        wait_motion_done(side=side, timeout_s=timeout)
        rospy.loginfo("claw state: %s", claw_state)
        if option_open:
            rospy.loginfo("open claw (%s)", side)
            call_dh_claw(pos_open, vel=speed, effort=force, side=side)
            option_open = False
        else:
            rospy.loginfo("close claw (%s)", side)
            call_dh_claw(pos_close, vel=speed, effort=force, side=side)
            option_open = True
        time.sleep(period)
