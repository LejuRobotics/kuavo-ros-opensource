#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

from arms_ik_api import ArmsIKApi


def euler_zyx_deg_to_quat_xyzw(roll_deg, pitch_deg, yaw_deg):
    """欧拉角(roll, pitch, yaw, deg) -> 四元数(x, y, z, w)，按 ZYX 顺序。"""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return [qx, qy, qz, qw]


def main():
    api = ArmsIKApi("case_03_multi_ref_constraint_compare")

    # 期望末端位置（m）
    left_pos = [0.45, 0.25, 0.22]       #以腰部waist_yaw_link为参考系
    right_pos = [0.45, -0.25, 0.22]
    # 期望末端姿态用欧拉角配置（deg），更方便用户改参数
    left_rpy_deg = [0.0, -90, 0.0]
    right_rpy_deg = [0.0, -90, 0.0]
    left_quat = euler_zyx_deg_to_quat_xyzw(*left_rpy_deg)
    right_quat = euler_zyx_deg_to_quat_xyzw(*right_rpy_deg)

    mode_desc = {
        0: "PosSoft_OriSoft",
        1: "PosSoft_OriHard",
        2: "PosHard_OriSoft",
        3: "PosHard_OriHard",
        4: "ThreePoint_Soft",
        6: "ThreePoint_Mixed",
    }

    results = []
    for mode in [0, 1, 2, 3, 4, 6]:
        ik_param = api.default_ik_param(constraint_mode=mode)
        cmd = api.build_two_arm_cmd(left_pos, left_quat, right_pos, right_quat, ik_param=ik_param)
        # 调用 IK 服务 /ik/two_arm_hand_pose_cmd_srv_muli_refer 进行多参考初值求解
        res = api.call_ik_multi_ref_srv(cmd)
        err = api.calc_pose_error(cmd.hand_poses, res.hand_poses)
        tag = "Case3_mode_{}_{}".format(mode, mode_desc[mode])
        api.print_ik_summary(tag, res.success, res.time_cost, err)
        results.append((mode, mode_desc[mode], res.success, res.time_cost, err["max_pos_mm"], err["max_ori_deg"]))

    print("\n[Case3] constraint_mode 对比汇总")
    for mode, desc, success, tms, max_pos, max_ori in results:
        print(
            "  mode={} ({}) success={} time={:.2f}ms pos_error_max={:.2f}mm ori_error_max={:.3f}deg".format(
                mode, desc, success, float(tms), max_pos, max_ori
            )
        )


if __name__ == "__main__":
    main()
