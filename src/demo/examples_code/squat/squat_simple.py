#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
简单测试：通过 /cmd_pose 话题控制机器人上下蹲。
/cmd_pose 为 geometry_msgs/Twist，其中 linear.z 表示【相对于标称站立高度的偏移量】（米），
而不是在上一条 /cmd_pose 命令基础上的“累积增量”；负值表示比标称高度更低（下蹲）。

注意：/cmd_pose 话题本身不会对数值范围做任何限制（既不限制速度也不限制高度），参数过大会导致仿真或实物摔倒。
本脚本内部对 linear.z 做了范围限制，数值来源见下方注释。
"""
import rospy
from geometry_msgs.msg import Twist
import os
import sys
import time

import rospkg

try:
    kuavo_common_path = rospkg.RosPack().get_path("kuavo_common")
    kuavo_common_python_path = os.path.join(kuavo_common_path, "python")
    if kuavo_common_python_path not in sys.path:
        sys.path.insert(0, kuavo_common_python_path)
    from robot_version import RobotVersion
except (rospkg.ResourceNotFound, ImportError):
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    kuavo_common_python_path = os.path.abspath(
        os.path.join(current_file_dir, "../../../kuavo_common/python")
    )
    if kuavo_common_python_path not in sys.path:
        sys.path.insert(0, kuavo_common_python_path)
    from robot_version import RobotVersion

# 仅允许 5 代机器人运行：版本号 50-59，判断条件 (version % 100) // 10 == 5
GEN5_NUMBER = 5

# 安全限制：linear.z 为相对于标称站立高度的偏移量（米），非时间上的累积增量
Z_DELTA_MIN = -0.2   # 下蹲方向：最多约 20cm
Z_DELTA_MAX = 0.00    # 抬升方向：最多约 0cm


def clamp_z_delta(z):
    """将高度增量限制在安全范围内。"""
    return max(Z_DELTA_MIN, min(Z_DELTA_MAX, z))


def get_robot_version():
    """从 ROS 参数或环境变量获取机器人版本号。"""
    try:
        return int(rospy.get_param("/robot_version", 0))
    except (rospy.exceptions.ROSException, ValueError):
        pass
    try:
        return int(os.environ.get("ROBOT_VERSION", "0"))
    except ValueError:
        pass
    return 0


def ensure_gen5_or_shutdown():
    version = get_robot_version()
    if not RobotVersion.is_valid(version):
        rospy.logerr("无法解析 robot_version=%r，禁止运行。", version)
        rospy.signal_shutdown("invalid robot_version")
        return False

    rv = RobotVersion.create(version)
    if not rv.start_with(GEN5_NUMBER):
        rospy.logerr(
            "当前 robot_version=%s（解析为 %s），禁止运行。本脚本仅支持 5 代机器人（robot_version 50-59）。",
            version,
            rv.version_name(),
        )
        rospy.signal_shutdown("robot_version not allowed")
        return False

    rospy.loginfo(
        "当前 robot_version=%s（解析为 %s），允许运行", version, rv.version_name()
    )
    return True


def main():
    rospy.init_node("squat_simple", anonymous=True)
    if not ensure_gen5_or_shutdown():
        return
    pub = rospy.Publisher("/cmd_pose", Twist, queue_size=10)
    time.sleep(0.5)  # 等待订阅者连接

    # linear.z：相对于标称站立高度的偏移量(m)，负=下蹲，0=恢复（会经安全限制截断）
    squat_delta_z = -0.2  # 下蹲 5cm，建议不超过 |Z_DELTA_MIN|
    duration = 5.0          # 每个姿态保持时间（秒）

    def publish_cmd_pose(linear_z):
        z_safe = clamp_z_delta(linear_z)
        if z_safe != linear_z:
            rospy.logwarn("高度增量 %.3f 已限制为 %.3f m（安全范围 [%.3f, %.3f]）",
                          linear_z, z_safe, Z_DELTA_MIN, Z_DELTA_MAX)
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = z_safe
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        pub.publish(msg)
        rospy.loginfo("cmd_pose 发布: linear.z=%.3f m (相对标称高度的偏移量)", z_safe)

    # 顺序：当前高度 -> 下蹲 -> 恢复
    rospy.loginfo("开始上下蹲测试")
    publish_cmd_pose(0.0)
    time.sleep(1.0)

    rospy.loginfo("下蹲")
    publish_cmd_pose(squat_delta_z)
    time.sleep(duration)

    rospy.loginfo("恢复站立")
    publish_cmd_pose(0.0)
    time.sleep(duration)


    rospy.loginfo("上下蹲测试结束")


if __name__ == "__main__":
    main()
