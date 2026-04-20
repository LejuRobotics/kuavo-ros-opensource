#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
简单测试：通过 /robot_waist_motion_data 话题控制机器人转腰。
消息类型：kuavo_msgs/robotWaistControl，data.data 为腰部目标角度（度）。
仅支持 5 代机器人运行（robot_version 50-59）。
"""
import os
import sys
import time

import rospy
import rospkg
from kuavo_msgs.msg import robotWaistControl

try:
    kuavo_common_path = rospkg.RosPack().get_path("kuavo_common")
    kuavo_common_python_path = os.path.join(kuavo_common_path, "python")
    if kuavo_common_python_path not in sys.path:
        sys.path.insert(0, kuavo_common_python_path)
    from robot_version import RobotVersion  # type: ignore[import]
except (rospkg.ResourceNotFound, ImportError):
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    kuavo_common_python_path = os.path.abspath(
        os.path.join(current_file_dir, "../../../kuavo_common/python")
    )
    if kuavo_common_python_path not in sys.path:
        sys.path.insert(0, kuavo_common_python_path)
    from robot_version import RobotVersion  # type: ignore[import]

# 5 代机器人：版本号 50-59
GEN5_NUMBER = 5
# 腰部转动限位（度）
WAIST_LIMIT_DEG = 90.0
# 转腰幅度（度）、每个姿态保持时间（秒）、发布前等待订阅连接（秒）
TURN_ANGLE_DEG = 45.0
POSE_HOLD_DURATION = 3.0
PUB_WAIT_SEC = 0.5


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
    rospy.init_node("waist_turn_simple", anonymous=True)
    if not ensure_gen5_or_shutdown():
        return

    pub = rospy.Publisher("/robot_waist_motion_data", robotWaistControl, queue_size=10)
    time.sleep(PUB_WAIT_SEC)

    # 依次：回正 -> 右转 -> 回正 -> 左转 -> 回正（幅度 TURN_ANGLE_DEG）
    angles_deg = [0.0, TURN_ANGLE_DEG, 0.0, -TURN_ANGLE_DEG, 0.0]
    for i, angle in enumerate(angles_deg):
        # 对腰部目标角度进行限位（±WAIST_LIMIT_DEG）
        clamped_angle = max(-WAIST_LIMIT_DEG, min(WAIST_LIMIT_DEG, float(angle)))
        if clamped_angle != angle:
            rospy.logwarn(
                "腰部目标角度 %.1f 超出限位，将被裁剪到 %.1f deg", angle, clamped_angle
            )
        msg = robotWaistControl()
        msg.header.stamp = rospy.Time.now()
        msg.data.data = [clamped_angle]
        pub.publish(msg)
        rospy.loginfo("腰部目标角度: %.1f deg", clamped_angle)
        if i < len(angles_deg) - 1:
            time.sleep(POSE_HOLD_DURATION)

    rospy.loginfo("转腰测试结束")


if __name__ == "__main__":
    main()
