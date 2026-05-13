#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""轮臂末端力控制命令行案例。"""

import rospy

from lb_force_ctrl import LBForceController



# 配置参数
HAND = "both"  # 可选: "left" / "right" / "both"
LEFT_FORCE = (0.0, 0.0, -29.4)      # 左手期望力，单位 N
RIGHT_FORCE = (0.0, 0.0, -29.4)     # 右手期望力，单位 N
ENABLE_EXTERNAL_FORCE = False  # True 时发送与期望力相同大小的额外力，由于是瞬间施加，仿真有抖动现象
DISABLE_FORCE_EMPTY_DETACT = True  # True: 施力前关闭挥空检测，结束后恢复，False: 不关闭挥空检测


def parse_vector(values, name):
    if values is None:
        return None
    if len(values) != 3:
        raise ValueError("%s requires exactly 3 values" % name)
    return tuple(float(v) for v in values)


def main():
    rospy.init_node("cmd_arm_force_test", anonymous=True)
    force_ctrl = LBForceController(wait_for_connection=True, connection_delay=0.2)

    if HAND not in ("left", "right", "both"):
        raise ValueError("HAND must be one of: left, right, both")

    left_force = parse_vector(LEFT_FORCE, "LEFT_FORCE")
    right_force = parse_vector(RIGHT_FORCE, "RIGHT_FORCE")

    try:
        if DISABLE_FORCE_EMPTY_DETACT:
            force_ctrl.enable_force_empty_detact(False)
            rospy.loginfo("已关闭挥空检测")

        if HAND == "both":
            force_ctrl.set_desired_ee_force_both(left_force=left_force, right_force=right_force)
            rospy.loginfo("已施加双手期望力: left=%s, right=%s", left_force, right_force)
            if ENABLE_EXTERNAL_FORCE:
                force_ctrl.set_external_wrench_both(
                    left_force=left_force,
                    right_force=right_force,
                )
                rospy.loginfo("已施加双手额外外力: left=%s, right=%s", left_force, right_force)
        else:
            selected_force = left_force if HAND == "left" else right_force
            force_ctrl.set_desired_ee_force(HAND, force=selected_force)
            rospy.loginfo("已施加%s手期望力: %s", "左" if HAND == "left" else "右", selected_force)
            if ENABLE_EXTERNAL_FORCE:
                force_ctrl.set_external_wrench(HAND, force=selected_force)
                rospy.loginfo("已施加%s手额外外力: %s", "左" if HAND == "left" else "右", selected_force)

        input("已施加末端力，按回车撤销...\n")

    finally:
        force_ctrl.clear_desired_ee_force(HAND if HAND != "both" else None)
        if ENABLE_EXTERNAL_FORCE:
            force_ctrl.clear_external_wrench(HAND if HAND != "both" else None)
        if DISABLE_FORCE_EMPTY_DETACT:
            force_ctrl.enable_force_empty_detact(True)
            rospy.loginfo("已恢复挥空检测")
        rospy.sleep(0.1)


if __name__ == "__main__":
    main()
