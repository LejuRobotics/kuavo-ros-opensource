#!/usr/bin/env python3
"""仿真里按你给的开合量驱动左右夹爪。

0 全开，50 行程中间，100 全闭。先起 600062 的轮臂 MuJoCo，再运行：

    python3 cmd_claw_random.py 0 100
    python3 cmd_claw_random.py

带两个数时，左、右一直保持这个开合，Ctrl+C 结束。
不带参数时，反复输入「左 右」，只输入一个数则两边相同。
"""
import sys
import threading

import rospy
from kuavo_msgs.msg import lejuClawCommand


def clamp(value):
    return max(0.0, min(100.0, float(value)))


def claw_message(left, right):
    msg = lejuClawCommand()
    msg.header.stamp = rospy.Time.now()
    msg.data.name = ["left_claw", "right_claw"]
    msg.data.position = [float(left), float(right)]
    msg.data.velocity = [50.0, 50.0]
    msg.data.effort = [1.0, 1.0]
    return msg


def parse_pair(text):
    parts = text.split()
    if len(parts) == 1:
        value = clamp(parts[0])
        return value, value
    if len(parts) == 2:
        return clamp(parts[0]), clamp(parts[1])
    raise ValueError("需要 1 个或 2 个数，范围 0~100")


def publish_loop(pub, target, stop):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not stop.is_set():
        if target["ready"]:
            pub.publish(claw_message(target["left"], target["right"]))
        rate.sleep()


def main():
    rospy.init_node("claw_set", anonymous=True)
    pub = rospy.Publisher("/leju_claw_command", lejuClawCommand, queue_size=10)
    target = {"left": 0.0, "right": 0.0, "ready": False}
    stop = threading.Event()
    thread = threading.Thread(target=publish_loop, args=(pub, target, stop))
    thread.daemon = True
    thread.start()
    rospy.sleep(0.3)

    if len(sys.argv) > 1:
        left, right = parse_pair(" ".join(sys.argv[1:]))
        target["left"] = left
        target["right"] = right
        target["ready"] = True
        print("夹爪 左/右 {:.0f}/{:.0f}".format(left, right))
        rospy.spin()
        return

    print("输入开合量：左 右，或一个数表示两边相同。空行退出。")
    while not rospy.is_shutdown():
        try:
            line = input("开合> ").strip()
        except EOFError:
            break
        if not line:
            break
        try:
            left, right = parse_pair(line)
        except ValueError as exc:
            print(exc)
            continue
        target["left"] = left
        target["right"] = right
        target["ready"] = True
        print("夹爪 左/右 {:.0f}/{:.0f}".format(left, right))

    stop.set()


if __name__ == "__main__":
    main()
