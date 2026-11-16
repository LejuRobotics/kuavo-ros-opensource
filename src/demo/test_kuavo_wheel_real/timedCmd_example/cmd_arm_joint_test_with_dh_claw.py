#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双臂关节定时指令 + DH 夹爪串行动作 demo（轮臂 ROBOT_VERSION 62/63）

一个脚本串行编排：手臂到点停稳 -> 调夹爪并等到位 -> 下一帧
可调参数在下面「可调参数」区；启动后输入轮数，按帧循环执行到结束：
  python3 cmd_arm_joint_test_dh_claw.py

前置：load_kuavo_real_wheel.launch + EndEffectorType=dhclaw；不要再起 dhclaw_ros_node。
夹爪：0=全开 / 100=全关（以 dhclaw_can.h 为准）。
"""
import argparse
import math
import os
import sys
import time

# 复用同目录上级的轮臂 API（与 cmd_arm_joint_test.py 相同）
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
from kuavo_msgs.msg import dhClawCommand, dhClawState
import lb_ctrl_api as ct

# =============================================================================
# 可调参数（改此处参数）
# =============================================================================
# 夹爪侧："both" / "left" / "right"（离线侧会自动跳过）
CLAW_SIDE = "both"
# 夹爪目标位（% ，0=全开 100=全关）
CLAW_CLOSE = 100.0          # 展开双臂后
CLAW_OPEN = 2.0             # 弯曲收回后
CLAW_AFTER_HANG = 50.0      # 手臂下垂/回零后
# 夹爪运动参数
CLAW_SPEED = 100.0          # 1~100 %
CLAW_FORCE = 100.0          # 20~100 %
CLAW_TOL = 8.0              # 到位容差 %
CLAW_TIMEOUT = 4.0          # 单次等待上限 s
# 手臂：轨迹 actualTime 结束后再多停多久再调夹爪
ARM_SETTLE = 0.3            # s
# 轮间额外停顿 s（一轮全部帧跑完后、下一轮开始前）
ROUND_PAUSE = 0.2
# 默认轮数；命令行 --rounds N 可覆盖；都不给则启动时 input 询问
DEFAULT_ROUNDS = 1
# 各步手臂期望时间 s（服务还会返 actualTime，以实际为准）
ARM_TIME_EXPAND = 2.0
ARM_TIME_BEND = 2.0
ARM_TIME_HANG = 2.0
# 各步双臂关节角（度：前7左臂 + 后7右臂）
ARM_POSE_EXPAND = [-30, 20, 15, -45, 25, 10, -35,
                   -30, -20, -15, -45, -25, -10, -35]
ARM_POSE_BEND = [-20, 30, -25, -20, 40, -15, 25,
                 -20, -30, 25, -20, -40, 15, 25]
ARM_POSE_HANG = [0.0] * 14  # 下垂/回零
# =============================================================================

CLAW_CMD_TOPIC = "/dh_claw_command"
CLAW_STATE_TOPIC = "/dh_claw_state"
LEFT_CLAW = "left_claw"
RIGHT_CLAW = "right_claw"

# 重发周期 < DhClawController VR 流式检测间隔(200ms)，任务槽只保最新一条
CLAW_RESEND_PERIOD = 0.1

_claw_state = {LEFT_CLAW: dhClawState.kUnknown, RIGHT_CLAW: dhClawState.kUnknown}
_claw_pos = {LEFT_CLAW: 0.0, RIGHT_CLAW: 0.0}
_claw_seen_moving = {LEFT_CLAW: False, RIGHT_CLAW: False}
_claw_state_received = False


def _claw_state_cb(msg):
    """按 name 对齐左右槽位；position 是硬件实时位置反馈。"""
    global _claw_state_received
    _claw_state_received = True
    for i, name in enumerate(msg.data.name):
        if name not in _claw_state:
            continue
        if i < len(msg.state):
            _claw_state[name] = msg.state[i]
            if msg.state[i] == dhClawState.kMoving:
                _claw_seen_moving[name] = True
        if i < len(msg.data.position):
            _claw_pos[name] = msg.data.position[i]


def _side_names(side):
    if side == "left":
        return [LEFT_CLAW]
    if side == "right":
        return [RIGHT_CLAW]
    return [LEFT_CLAW, RIGHT_CLAW]


def _fmt_claw(names):
    return " ".join(
        f"{n}(state={_claw_state[n]}, pos={_claw_pos[n]:.0f})" for n in names
    )


def _claw_arrived(name, target, tol):
    """到位判据：位置到目标且非 Moving；夹住物体时位置停在中途，用夹持状态兜底。"""
    if _claw_state[name] == dhClawState.kMoving:
        return False
    if abs(_claw_pos[name] - target) <= tol:
        return True
    return _claw_seen_moving[name] and _claw_state[name] in (
        dhClawState.kReached,
        dhClawState.kGrabbed,
    )


def move_claw(pub, target, args):
    """下发目标位置并按流式频率重发，直到到位/夹住或超时。返回是否达成。"""
    names = _side_names(args.side)
    for n in names:
        _claw_seen_moving[n] = False

    msg = dhClawCommand()
    msg.data.name = list(names)
    msg.data.position = [float(target)] * len(names)
    msg.data.velocity = [float(args.claw_speed)] * len(names)
    msg.data.effort = [float(args.claw_force)] * len(names)
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)

    # 先等一帧反馈：据此判断哪些侧真正在线（离线侧 position 恒为 0、state=kUnknown，
    # 若把它算进到位判据，目标 0 会被误判成"已到位"）
    rospy.sleep(0.3)
    drive = [n for n in names if _claw_state[n] != dhClawState.kUnknown]
    if not drive:
        rospy.logwarn(f"⚠️ 无在线夹爪，跳过本次动作: {_fmt_claw(names)}")
        return False

    rospy.loginfo(f"  夹爪 -> {target:.0f}%  {_fmt_claw(drive)}")
    t0 = time.time()
    last_log = t0
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        if all(_claw_arrived(n, target, args.claw_tol) for n in drive):
            rospy.loginfo(f"✅ 夹爪到位 target={target:.0f}%  {_fmt_claw(drive)}")
            return True
        now = time.time()
        if now - t0 >= args.claw_timeout:
            rospy.logwarn(
                f"⚠️ 夹爪等待超时 {args.claw_timeout:.1f}s target={target:.0f}%  {_fmt_claw(drive)}"
            )
            return False
        if now - last_log >= 2.0:
            rospy.loginfo(f"  等待夹爪 target={target:.0f}%  {_fmt_claw(drive)}")
            last_log = now
        rospy.sleep(CLAW_RESEND_PERIOD)
    return False


def send_arm_step(name, desire_time, joint_angles_deg):
    """下发一组双臂关节定时指令。服务返回即表示已下发，轨迹由控制器按 actualTime 执行。"""
    joint_angles_rad = [math.radians(a) for a in joint_angles_deg]
    left_arm_angles = joint_angles_rad[:7]
    right_arm_angles = joint_angles_rad[7:]

    # send_timed_single_command 异常路径返回 3 元组，这里统一按 (success, time) 取用
    res_l = ct.send_timed_single_command(8, desire_time, left_arm_angles)
    res_r = ct.send_timed_single_command(9, desire_time, right_arm_angles)
    ok_l, t_l = bool(res_l[0]), float(res_l[1] or desire_time)
    ok_r, t_r = bool(res_r[0]), float(res_r[1] or desire_time)

    if not (ok_l and ok_r):
        rospy.logwarn(f"⚠️ {name} 命令下发失败: 左臂{'成功' if ok_l else '失败'}, "
                      f"右臂{'成功' if ok_r else '失败'}")
    return max(t_l, t_r)


def wait_arm_done(t_start, actual_time, settle):
    """等手臂轨迹跑完 actualTime，再额外 settle 秒让关节停稳。

    timed 服务本身不阻塞、也不提供到位回调，只能以服务返回的 actualTime 为轨迹时长。
    """
    remain = t_start + actual_time + settle - time.time()
    if remain > 0:
        rospy.sleep(remain)


def preflight(pub):
    """整机没按 dhclaw 配置时早失败，避免 demo 静默只动手臂。"""
    eef = rospy.get_param("/end_effector_type", "")
    if eef != "dhclaw":
        rospy.logerr(f"❌ /end_effector_type={eef!r}，本 demo 需要 kuavo.json 配置 EndEffectorType=dhclaw")
        return False

    rospy.loginfo(f"等待 {CLAW_CMD_TOPIC} 订阅者（hardware_node）...")
    t0 = time.time()
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        if time.time() - t0 > 10.0:
            rospy.logerr(f"❌ {CLAW_CMD_TOPIC} 无订阅者：hardware_node 未启动或未初始化 dhclaw")
            return False
        rospy.sleep(0.1)

    t0 = time.time()
    while not _claw_state_received and not rospy.is_shutdown():
        if time.time() - t0 > 5.0:
            rospy.logerr(f"❌ 未收到 {CLAW_STATE_TOPIC}：无法判断夹爪是否到位")
            return False
        rospy.sleep(0.1)
    rospy.loginfo(f"夹爪反馈在线: {_fmt_claw([LEFT_CLAW, RIGHT_CLAW])}")
    return True


def build_steps(args):
    """一轮内的帧序列：(名称, 期望时间s, 关节角deg×14, 夹爪目标%|None)。"""
    return [
        ("展开双臂", ARM_TIME_EXPAND, list(ARM_POSE_EXPAND), args.claw_close),
        ("弯曲收回", ARM_TIME_BEND, list(ARM_POSE_BEND), args.claw_open),
        ("手臂下垂", ARM_TIME_HANG, list(ARM_POSE_HANG), args.claw_after_hang),
    ]


def estimate_round_time(steps, settle, round_pause):
    """粗估一轮时长（desire_time + settle×帧数 + 轮间停顿；夹爪实际时长另计）。"""
    return sum(t for _, t, _, _ in steps) + settle * len(steps) + round_pause


def ask_num_rounds(default_rounds):
    """启动时输入轮数；空回车用 default；非法输入重问。"""
    while True:
        raw = input(f"请输入要执行的轮数 [默认: {default_rounds}]: ").strip()
        if not raw:
            return default_rounds
        try:
            n = int(raw)
            if n >= 1:
                return n
        except ValueError:
            pass
        print("请输入 >= 1 的整数")


def execute_round(steps, pub, args, round_num, num_rounds):
    """执行一轮：按帧串行 手臂到点停稳 -> 夹爪到位 -> 下一帧。"""
    rospy.loginfo(f"\n========== 第 {round_num}/{num_rounds} 轮 ==========")

    for idx, (name, desire_time, joint_angles_deg, claw_target) in enumerate(steps, 1):
        if rospy.is_shutdown():
            return False

        rospy.loginfo(f"\n--- 帧{idx}/{len(steps)}: {name} ---")
        rospy.loginfo(f"  期望时间: {desire_time:.1f}s  关节(度): {joint_angles_deg}")

        t_start = time.time()
        actual_time = send_arm_step(name, desire_time, joint_angles_deg)
        rospy.loginfo(f"  双臂已下发，规划 {actual_time:.2f}s，等待到点停稳...")
        wait_arm_done(t_start, actual_time, args.arm_settle)
        rospy.loginfo(f"  手臂已停稳（+{args.arm_settle:.2f}s settle）")

        if claw_target is not None:
            rospy.loginfo(f"  手臂到位后调用夹爪 -> {claw_target:.0f}%")
            move_claw(pub, claw_target, args)

        rospy.loginfo(f"  {name} 到达!")

    return True


def execute(steps, pub, args, num_rounds):
    """按轮循环整组帧，直到 num_rounds 结束或 ROS 中断。"""
    rospy.loginfo(
        f"开始循环：共 {num_rounds} 轮 × {len(steps)} 帧；"
        f"每帧=手臂到点停稳 -> 夹爪到位 -> 下一帧"
    )
    t_all = time.time()

    for round_num in range(1, num_rounds + 1):
        if rospy.is_shutdown():
            break
        if not execute_round(steps, pub, args, round_num, num_rounds):
            break
        if round_num < num_rounds and args.round_pause > 0:
            rospy.loginfo(f"  轮间停顿 {args.round_pause:.1f}s...")
            rospy.sleep(args.round_pause)

    rospy.loginfo(f"\n全部完成：{num_rounds} 轮，耗时 {time.time() - t_all:.1f}s")


def main():
    # 命令行可选覆盖顶部常量；平时直接改文件顶部「可调参数」即可
    parser = argparse.ArgumentParser(
        description="双臂关节 + DH 夹爪串行 demo（循环帧；参数默认见文件顶部）")
    parser.add_argument("--side", choices=["both", "left", "right"], default=CLAW_SIDE)
    parser.add_argument("--claw-close", type=float, default=CLAW_CLOSE)
    parser.add_argument("--claw-open", type=float, default=CLAW_OPEN)
    parser.add_argument("--claw-after-hang", type=float, default=CLAW_AFTER_HANG)
    parser.add_argument("--claw-speed", type=float, default=CLAW_SPEED)
    parser.add_argument("--claw-force", type=float, default=CLAW_FORCE)
    parser.add_argument("--claw-tol", type=float, default=CLAW_TOL)
    parser.add_argument("--claw-timeout", type=float, default=CLAW_TIMEOUT)
    parser.add_argument("--arm-settle", type=float, default=ARM_SETTLE)
    parser.add_argument("--round-pause", type=float, default=ROUND_PAUSE,
                        help="一轮结束后、下一轮开始前的停顿 s")
    parser.add_argument("-n", "--rounds", type=int, default=None,
                        help="执行轮数；不给则启动时交互输入")
    args = parser.parse_args()

    steps = build_steps(args)
    rough = estimate_round_time(steps, args.arm_settle, 0.0)

    if args.rounds is not None:
        if args.rounds < 1:
            print("--rounds 需 >= 1", file=sys.stderr)
            return 1
        num_rounds = args.rounds
    else:
        num_rounds = ask_num_rounds(DEFAULT_ROUNDS)

    print(f"将执行 {num_rounds} 轮动作序列")

    rospy.init_node("arm_joint_dh_claw_demo", anonymous=True)
    pub = rospy.Publisher(CLAW_CMD_TOPIC, dhClawCommand, queue_size=10)
    rospy.Subscriber(CLAW_STATE_TOPIC, dhClawState, _claw_state_cb)

    if not preflight(pub):
        return 1

    if not ct.set_arm_control_mode(2):
        rospy.logerr("❌ 手臂控制模式切换失败，终止")
        return 1

    try:
        execute(steps, pub, args, num_rounds)
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS 中断异常")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
