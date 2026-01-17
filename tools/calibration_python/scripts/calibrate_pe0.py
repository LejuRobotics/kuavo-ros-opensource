#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
只标定工具坐标系平移 Pe0 的脚本（不优化旋量）。

思路：
- 已知：旋量参数 kesi（6×7，米）、关节角 theta（N×7，度）、末端实测位置 pos（N×3，毫米）
- 固定旋量，逐条计算 base->末端的齐次变换 g_i（假设 Pe0 初值为零向量或已有值）
- 利用线性最小二乘求解 Pe0 满足：g_i * [Pe0;1] ≈ pos_meas_i
  展开：R_i * Pe0 + t_i = p_meas_i  ->  A x = b
- 输出：更新后的 Pe0 写回 `target_flange_matrix.txt`（只替换平移列，旋转保持不变）

注意：
- 仅标定平移 3 维，不估计姿态
- 单位转换：theta 度->弧度，pos 毫米->米
- 依赖 `screw_utils.py`
"""

import numpy as np
from pathlib import Path
from function.screw_utils import forward_g_sequence

def calibrate_pe0_only(kesi_file, theta_file, pos_file, pe0_file, output_file=None, verbose=True):
    """
    仅标定工具坐标系平移 Pe0，并写回指定文件。

    Args:
        kesi_file: 旋量文件路径（6×7，米）
        theta_file: 关节角文件路径（N×7，度，文件内可为列向量，代码会自动转置）
        pos_file: 末端位置文件路径（N×3，毫米，文件内可为列向量，代码会自动转置）
        pe0_file: 工具坐标系 4×4 矩阵文件路径（读取并更新第4列平移）
        output_file: 可选，若提供则将更新后的矩阵写入此文件，否则覆盖 pe0_file
        verbose: 是否打印调试信息
    Returns:
        pe0_new: 4×1 齐次坐标（米）
    """
    # ========== 加载数据 ==========
    kesi = np.loadtxt(kesi_file)  # 6×7
    theta = np.loadtxt(theta_file)
    pos = np.loadtxt(pos_file)

    # 若文件为列存储，转置成 N×7 / N×3
    if theta.shape[0] == 7 and theta.ndim == 2:
        theta = theta.T
    if pos.shape[0] == 3 and pos.ndim == 2:
        pos = pos.T

    # 单位转换
    theta_rad = np.deg2rad(theta)          # 度->弧度
    pos_m = pos * 0.001                    # 毫米->米

    num_samples = theta_rad.shape[0]
    if verbose:
        print(f"加载数据: 样本数={num_samples}")

    # ========== 构建线性方程 A x = b ==========
    # 对每个样本：R_i * pe0 + t_i = p_meas_i
    A_list = []
    b_list = []
    for i in range(num_samples):
        g = forward_g_sequence(kesi, theta_rad[i, :])
        R = g[0:3, 0:3]
        t = g[0:3, 3]
        A_list.append(R)
        b_list.append(pos_m[i, :] - t)

    A = np.vstack(A_list)        # (3N, 3)
    b = np.hstack(b_list)        # (3N,)

    # ========== 最小二乘求解 pe0 ==========
    pe0_xyz, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    pe0_xyz = pe0_xyz.reshape(3)

    if verbose:
        print(f"Pe0 求解完成：pe0_xyz = {pe0_xyz}, 残差范数 = {np.sqrt(np.sum(residuals)) if residuals.size>0 else 0:.6e}")

    # ========== 写回 pe0 矩阵 ==========
    pe0_mat = np.loadtxt(pe0_file)
    pe0_mat[0:3, 3] = pe0_xyz
    pe0_mat[3, 3] = 1.0  # 保持齐次

    save_path = output_file if output_file is not None else pe0_file
    np.savetxt(save_path, pe0_mat, fmt="%.10e")

    if verbose:
        print(f"已写入新的工具坐标系平移到: {save_path}")

    # 返回 4×1 齐次坐标
    pe0_new = np.zeros((4, 1))
    pe0_new[0:3, 0] = pe0_xyz
    pe0_new[3, 0] = 1.0
    return pe0_new


if __name__ == "__main__":
    # 默认路径，可按需修改
    base_dir = Path(__file__).parent
    config_dir = base_dir / "config"
    input_dir = base_dir / "input"

    kesi_file = config_dir / "theory_kesi.txt"
    theta_file = input_dir / "calibration_joints.txt"
    pos_file = input_dir / "calibration_points.txt"
    pe0_file = config_dir / "target_flange_matrix.txt"

    calibrate_pe0_only(kesi_file, theta_file, pos_file, pe0_file, verbose=True)

