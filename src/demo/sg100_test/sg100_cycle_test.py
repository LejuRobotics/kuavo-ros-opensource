#!/usr/bin/env python3
"""在 kuavo-ros-opensource 根目录运行，连接整机 ROS 测试 SG100 双手。"""

import itertools
import math
from pathlib import Path
import signal
import sys
import time

# 其余关节沿用手势库关键点，J4 缩小行程以避开机械阻挡。
OPEN = (0.2, 0, 0.05, -0.16, 0, 0, 0, 0, 0.55, 0, 0)
FIST = (1.3, -1.7, 1.15, -0.06, 1.55, 1.25, 1.55, 1.25, 0.15, 1.55, 1.25)
GRIP = tuple(start + 0.65 * (end - start) for start, end in zip(OPEN, FIST))
SIDES = ("left", "right")
PERIOD = 0.05
TOLERANCE = 0.10


def positions(state):
    result = {}
    for side in SIDES:
        error = getattr(state, side + "_error_code")
        valid = getattr(state, side + "_feedback_valid")
        angles = getattr(state, side + "_hand_positions")
        if error or not getattr(state, side + "_hand_connected") or not valid:
            raise RuntimeError("{} 手反馈异常，error_code={}".format(side, error))
        if len(angles) != 11 or not all(map(math.isfinite, angles)):
            raise RuntimeError("{} 手关节反馈无效".format(side))
        result[side] = tuple(angles)
    return result


def check_target(actual, target):
    for side in SIDES:
        for joint, (angle, goal) in enumerate(zip(actual[side], target), start=1):
            if abs(angle - goal) > TOLERANCE:
                raise RuntimeError(
                    "{} 手 J{} 未到位，实际 {:.3f} rad，目标 {:.3f} rad".format(
                        side, joint, angle, goal
                    )
                )


def run(rospy, command_type, state_type):
    latest = None
    active = False
    pub = rospy.Publisher("/sg100_hand_command", command_type, queue_size=1)

    def receive(msg):
        nonlocal latest
        latest = (time.monotonic(), msg)

    def read():
        if rospy.is_shutdown() or pub.get_num_connections() == 0:
            raise RuntimeError("SG100 命令订阅已断开")
        sample = latest
        if sample is None or time.monotonic() - sample[0] > 0.5:
            raise RuntimeError("SG100 状态超过 0.5 秒未更新")
        return positions(sample[1])

    def send(targets=None):
        msg = command_type()
        msg.header.stamp = rospy.Time.now()
        msg.source_id = 3
        if targets is not None:
            msg.control_mode = command_type.MODE_JOINT_POSITION
            msg.left_enable_mask = msg.right_enable_mask = 0x07FF
            msg.left_hand_positions, msg.right_hand_positions = targets
        pub.publish(msg)

    def move(start, target, ramp=1.0):
        nonlocal active
        began = time.monotonic()
        while True:
            actual = read()
            elapsed = time.monotonic() - began
            if elapsed >= ramp + 1.0:
                check_target(actual, target)
                return
            fraction = min(1.0, elapsed / ramp)
            targets = [
                [begin + fraction * (goal - begin) for begin, goal in zip(start[side], target)]
                for side in SIDES
            ]
            active = True
            send(targets)
            time.sleep(PERIOD)

    sub = rospy.Subscriber("/sg100_hand_state", state_type, receive, queue_size=1)
    try:
        deadline = time.monotonic() + 10.0
        while True:
            try:
                initial = read()
                break
            except RuntimeError as exc:
                if rospy.is_shutdown() or time.monotonic() >= deadline:
                    raise RuntimeError("双手未就绪：{}".format(exc)) from exc
                time.sleep(PERIOD)

        rospy.loginfo("SG100 双手就绪，缓慢进入张开姿态。")
        move(initial, OPEN, ramp=2.0)
        rospy.loginfo("SG100 开始持续开合，按 Ctrl+C 退出。")
        for start, target in itertools.cycle(((OPEN, GRIP), (GRIP, OPEN))):
            move(dict.fromkeys(SIDES, start), target)
    finally:
        try:
            if active and not rospy.is_shutdown() and pub.get_num_connections():
                for _ in range(3):
                    send()
                    time.sleep(PERIOD)
            elif active:
                rospy.logerr("SG100 命令链路不可用，无法发送释放指令。")
        finally:
            sub.unregister()
            pub.unregister()


def message_type(root, name):
    from genpy.dynamic import generate_dynamic
    from std_msgs.msg import Header

    kind = "kuavo_msgs/" + name
    definition = (root / "src/kuavo_msgs/msg" / (name + ".msg")).read_text(encoding="utf-8")
    definition += "\n" + "=" * 80 + "\nMSG: std_msgs/Header\n" + Header._full_text
    return generate_dynamic(kind, definition)[kind]


def interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    try:
        import rosgraph
        import rospy
    except ImportError as exc:
        print("请先加载整机使用的 ROS 环境：{}".format(exc), file=sys.stderr)
        return 1

    try:
        if not rosgraph.is_master_online():
            raise RuntimeError("未连接到整机 ROS Master，请先启动整机并检查 ROS_MASTER_URI")
        root = Path.cwd()
        command_type = message_type(root, "SG100HandCommand")
        state_type = message_type(root, "SG100HandState")
        signal.signal(signal.SIGTERM, interrupt)
        rospy.init_node("sg100_cycle_test", disable_signals=True)
        run(rospy, command_type, state_type)
    except KeyboardInterrupt:
        print("SG100 测试已停止。")
        return 130
    except (OSError, ImportError, RuntimeError, rospy.ROSException) as exc:
        print("SG100 测试失败：{}".format(exc), file=sys.stderr)
        return 1
    finally:
        rospy.signal_shutdown("SG100 test finished")


if __name__ == "__main__":
    raise SystemExit(main())
