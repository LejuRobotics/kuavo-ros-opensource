#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Case00：给定期望双臂末端位姿（欧拉角 RPY），按 --mode 选择一种 IK 方式求解。

末端期望位置/姿态参考系：waist_yaw_link（位置 m；姿态 RPY deg 或四元数 xyzw）。
"""

import argparse
import math
import sys
import time

from arms_ik_api import ArmsIKApi

# topic | srv | muli_refer
IK_IFACE = {
    "topic": "/ik/two_arm_hand_pose_cmd",
    "srv": "/ik/two_arm_hand_pose_cmd_srv",
    "muli_refer": "/ik/two_arm_hand_pose_cmd_srv_muli_refer",
}


def rpy_deg_to_quat(roll, pitch, yaw):
    """欧拉角 (deg, ZYX) -> 四元数 xyzw。"""
    r, p, y = [math.radians(v) for v in (roll, pitch, yaw)]
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def main():
    parser = argparse.ArgumentParser(description="Case00：期望位姿 IK（--mode topic|srv|muli_refer）")
    parser.add_argument("--mode", required=True, choices=sorted(IK_IFACE))
    mode = parser.parse_args().mode

    api = ArmsIKApi("case_00_desired_pose_ik")

    # ===== 在此修改期望位姿（参考系: waist_yaw_link，单位 m / deg）=====
    left_pos, right_pos = [0.45, 0.3, -0.1], [0.45, -0.3, -0.1]
    left_rpy, right_rpy = [0.0, -90.0, 0.0], [0.0, -90.0, 0.0]
    cmd = api.build_two_arm_cmd(
        left_pos, rpy_deg_to_quat(*left_rpy),
        right_pos, rpy_deg_to_quat(*right_rpy),
        ik_param=api.default_ik_param(constraint_mode=2),
        joint_angles_as_q0=True,
    )

    tag = "Case0_{}".format(mode)
    print("[Case0] mode={}  iface={}".format(mode, IK_IFACE[mode]))
    print("left_pos:", list(cmd.hand_poses.left_pose.pos_xyz))
    print("right_pos:", list(cmd.hand_poses.right_pose.pos_xyz))

    q_exec = None
    if mode == "topic":
        api.reset_ik_result_topic_buffer()
        api.publish_ik_topic(cmd)  # /ik/two_arm_hand_pose_cmd
        ik_msg = api.wait_ik_result_topic(timeout_sec=2.0)
        ok = ik_msg is not None
        err = api.calc_pose_error(cmd.hand_poses, ik_msg) if ok else None
        api.print_ik_summary(tag, ok, None, err)
        if ok:
            q_exec = list(ik_msg.left_pose.joint_angles) + list(ik_msg.right_pose.joint_angles)
    else:
        call = api.call_ik_srv if mode == "srv" else api.call_ik_multi_ref_srv
        res = call(cmd)  # srv 或 muli_refer 服务
        err = api.calc_pose_error(cmd.hand_poses, res.hand_poses) if res.success else None
        api.print_ik_summary(tag, res.success, res.time_cost, err)
        if not res.success and res.error_reason:
            print("[{}] error_reason: {}".format(tag, res.error_reason))
        ok = res.success
        if ok and len(res.q_arm) >= 14:
            q_exec = res.q_arm[:14]

    if q_exec is not None:
        api.publish_arm_traj(q_exec)
        time.sleep(0.2)

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
