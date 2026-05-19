#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
底盘急停脚本（常驻 100 Hz 急停）

在底盘失控时运行此脚本，向 /cmd_vel 与 /cmd_vel_world 以 100Hz 持续发送零速度，
覆盖当前速度指令，使底盘一直处于急停状态，直到脚本被 Ctrl+C 终止。

用法（在已启动 ROS 与轮臂接口的环境下）：
    rosrun humanoid_wheel_interface_ros chassis_emergency_stop.py
  或直接：
    python src/humanoid-wheel-control/humanoid_wheel_interface_ros/scripts/chassis_emergency_stop.py
"""

import sys
import time

try:
    import rospy
    from geometry_msgs.msg import Twist
except ImportError:
    print("请确保在 ROS 环境下运行（source /opt/ros/.../setup.bash 且安装 geometry_msgs）")
    sys.exit(1)


# 零速度消息（局部系与世界系共用）
ZERO_TWIST = Twist()
ZERO_TWIST.linear.x = 0.0
ZERO_TWIST.linear.y = 0.0
ZERO_TWIST.linear.z = 0.0
ZERO_TWIST.angular.x = 0.0
ZERO_TWIST.angular.y = 0.0
ZERO_TWIST.angular.z = 0.0

# 急停发布频率（Hz），固定 100Hz，持续覆盖控制指令
PUB_RATE = 100


def run_emergency_stop(rate_hz: float = PUB_RATE) -> None:
    rospy.init_node("chassis_emergency_stop", anonymous=True)
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)
    pub_cmd_vel_world = rospy.Publisher("/cmd_vel_world", Twist, queue_size=1, latch=False)

    # 让订阅者连接，避免首条消息丢失
    time.sleep(0.2)
    r = rospy.Rate(rate_hz)
    count = 0
    print(f"[底盘急停] 持续以 {rate_hz} Hz 发送零速度到 /cmd_vel 与 /cmd_vel_world，Ctrl+C 退出")
    try:
        while not rospy.is_shutdown():
            pub_cmd_vel.publish(ZERO_TWIST)
            pub_cmd_vel_world.publish(ZERO_TWIST)
            count += 1
            r.sleep()
    except KeyboardInterrupt:
        pass
    print(f"[底盘急停] 已发送 {count} 次零速度，脚本退出。")


def main():
    # 固定 100Hz 一直发布零速度，直到 Ctrl+C 或节点关闭
    run_emergency_stop(rate_hz=PUB_RATE)


if __name__ == "__main__":
    main()
