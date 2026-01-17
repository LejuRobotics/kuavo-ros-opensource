#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
根据采样结果(txt)计算 POE/URDF/Mocap 误差：
- 输入关节角文件（度）
- 使用标定旋量 + pe0 计算 POE 末端位置
- 调用 /ik/fk_srv 计算 URDF FK 末端位置
- 可选读取动捕末端位置（已对齐 base_link）
- 输出多类误差：POE_vs_FK、POE_vs_Mocap、URDF_vs_Mocap

用法示例：
python3 compare_errors.py \
  --arm-side left \
  --joints-file ./scripts/measured_joints_left_arm.txt \
  --kesi-file ./config/theory_kesi.txt \
  --pe0-file ./config/target_flange_matrix.txt \
  --mocap-file ./scripts/measured_mocap_left_arm.txt \
  --output-dir ./scripts/output_compare
"""

import argparse
from pathlib import Path
import numpy as np
import rospy

from function.screw_utils import forward_kinematics
from function.data_utils import (
    load_joints_deg,
    load_positions,
    load_pe0,
    call_fk_service,
    build_dual_arm,
    norm_err,
    stats,
)


def main():
    parser = argparse.ArgumentParser(description="计算 POE/URDF/Mocap 误差")
    parser.add_argument("--arm-side", choices=["left", "right"], default="left")
    parser.add_argument("--joints-file", type=Path, required=True, help="关节角文件（度），每行7个")
    parser.add_argument("--kesi-file", type=Path, required=True, help="标定/理论旋量文件 6x7 (m)")
    parser.add_argument("--pe0-file", type=Path, required=True, help="工具矩阵文件 4x4，取第4列")
    parser.add_argument("--mocap-file", type=Path, help="可选，动捕末端位置（米），与关节行数一致")
    parser.add_argument("--output-dir", type=Path, default=Path("./output_compare"))
    args = parser.parse_args()

    rospy.init_node("compare_errors", anonymous=True)

    joints_deg = load_joints_deg(args.joints_file)
    joints_rad = np.deg2rad(joints_deg)
    kesi = np.loadtxt(args.kesi_file)
    pe0 = load_pe0(args.pe0_file)

    mocap_data = load_positions(args.mocap_file) if args.mocap_file and args.mocap_file.exists() else None

    poe_list, urdf_list = [], []
    for i in range(joints_rad.shape[0]):
        theta_row = joints_rad[i, :]
        poe_pos = forward_kinematics(kesi, theta_row, pe0).flatten()
        poe_list.append(poe_pos)

        fk_input = build_dual_arm(theta_row, args.arm_side)
        urdf_pos = call_fk_service(fk_input, args.arm_side)
        if urdf_pos is None:
            urdf_pos = np.full(3, np.nan)
        urdf_list.append(urdf_pos)

    poe_arr = np.vstack(poe_list)
    urdf_arr = np.vstack(urdf_list)

    poe_vs_fk = norm_err(poe_arr, urdf_arr)
    poe_vs_mocap = norm_err(poe_arr, mocap_data) if mocap_data is not None else None
    urdf_vs_mocap = norm_err(urdf_arr, mocap_data) if mocap_data is not None else None

    args.output_dir.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.output_dir / "poe_vs_fk.txt", poe_vs_fk, fmt="%.9f")
    if poe_vs_mocap is not None:
        np.savetxt(args.output_dir / "poe_vs_mocap.txt", poe_vs_mocap, fmt="%.9f")
    if urdf_vs_mocap is not None:
        np.savetxt(args.output_dir / "urdf_vs_mocap.txt", urdf_vs_mocap, fmt="%.9f")

    stats("POE vs FK", poe_vs_fk)
    if poe_vs_mocap is not None:
        stats("POE vs Mocap", poe_vs_mocap)
    if urdf_vs_mocap is not None:
        stats("URDF vs Mocap", urdf_vs_mocap)


if __name__ == "__main__":
    main()

