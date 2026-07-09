#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Case04：带肘部约束 IK。末端/肘部目标参考系: waist_yaw_link。"""

import time

from arms_ik_api import ArmsIKApi


def main():
    api = ArmsIKApi("case_04_srv_with_elbow_constraint")

    # 期望末端位置（m），参考系: waist_yaw_link
    left_pos = [0.45, 0.25, 0.12]
    right_pos = [0.45, -0.25, 0.12]
    left_quat = [0.0, -0.70682518, 0.0, 0.70738827]
    right_quat = [0.0, -0.70682518, 0.0, 0.70738827]

    # 肘部约束点（m），参考系: waist_yaw_link
    left_elbow = [0.24, 0.30, 0.28]
    right_elbow = [0.24, -0.30, 0.28]

    cmd = api.build_two_arm_cmd(
        left_pos_xyz=left_pos,
        left_quat_xyzw=left_quat,
        right_pos_xyz=right_pos,
        right_quat_xyzw=right_quat,
        ik_param=api.default_ik_param(constraint_mode=3),
        left_elbow_pos_xyz=left_elbow,
        right_elbow_pos_xyz=right_elbow,
    )
    # 调用 IK 服务 /ik/two_arm_hand_pose_cmd_srv_muli_refer 进行多参考初值求解
    res = api.call_ik_multi_ref_srv(cmd)
    err = api.calc_pose_error(cmd.hand_poses, res.hand_poses)
    api.print_ik_summary("Case4_srv_elbow", res.success, res.time_cost, err)

    if res.success and len(res.q_arm) >= 14:
        api.publish_arm_traj(res.q_arm[:14])
        time.sleep(0.2)


if __name__ == "__main__":
    main()
