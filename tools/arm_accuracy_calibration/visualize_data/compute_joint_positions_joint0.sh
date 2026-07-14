#!/usr/bin/env bash
set -e

# 计算 joint_0 标定结果下各关节在 joint1 坐标系中的位置
python3 /root/kuavo_ws/tools/arm_accuracy_calibration/urdf_tools/compute_all_joint_positions_in_joint1.py \
  --target-dir /root/kuavo_ws/tools/arm_accuracy_calibration/calibration_output/0302_output/calibration_output/fitted_results/joint_0

