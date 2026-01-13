#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
7自由度机器人旋量标定代码（Python版本）

基于等距离约束的旋量标定方法：
- 优化变量：7个关节的旋量增量（42维）+ 工具坐标系平移（3维）= 45维
- 约束：任意两个标定位形的末端实测距离 = 理论距离
- 方法：正则化伪逆求解

作者：基于MATLAB代码转换
日期：2025
"""

import numpy as np
from pathlib import Path
import os

# 从本地 function 目录导入工具函数（执行时请在 tools/robot_calibration_python 目录下）
from function.screw_utils import (
    operator_w,
    exp_se3_r,
    ad_g,
    ad_kesi,
    yita_exp_diff,
    coe_dif_yita,
    jacobi_g,
    jacobi_pos,
    forward_kinematics,
)


# ==================== 主标定函数 ====================

def calibrate_robot_screw(
    kesi_init,           # 初始旋量参数（6×7），米
    theta_data,          # 关节角数据（N×7），度
    pos_data,            # 末端位置数据（N×3），毫米
    pe0_init,            # 初始工具坐标系平移（4×1），齐次坐标，米
    num_train=80,        # 训练集数量
    num_test=40,         # 测试集数量
    lambda_reg=1e-6,     # 正则化系数
    max_iter=100,        # 最大迭代次数
    tol=1e-10,           # 收敛容差
    verbose=True
):
    """
    7自由度机器人旋量标定主函数
    
    Args:
        kesi_init: 初始旋量参数（6×7），米
        theta_data: 关节角数据（N×7），度
        pos_data: 末端位置数据（N×3），毫米
        pe0_init: 初始工具坐标系平移（4×1），米
        num_train: 训练集数量
        num_test: 测试集数量
        lambda_reg: 正则化系数
        max_iter: 最大迭代次数
        tol: 收敛容差
        verbose: 是否打印详细信息
    
    Returns:
        dict: 包含标定结果的字典
            - iden_kesi: 标定后的旋量参数（6×7）
            - iden_pe0: 标定后的工具坐标系平移（4×1）
            - train_errors: 训练集误差（mm）
            - test_errors: 测试集误差（mm）
            - opt_R: 最优旋转矩阵（3×3）
            - opt_t: 最优平移向量（3×1）
    """
    # ========== 数据预处理 ==========
    num_total = theta_data.shape[0]
    if num_train + num_test > num_total:
        raise ValueError(f"训练集+测试集数量({num_train}+{num_test})超过总数据量({num_total})")
    
    # 单位转换
    theta_rad = np.deg2rad(theta_data)  # 度转弧度
    pos_m = pos_data * 0.001  # 毫米转米
    
    # 随机划分训练集和测试集
    np.random.seed(42)  # 固定随机种子，便于复现
    indices = np.random.permutation(num_total)
    index_train = indices[:num_train]
    index_test = indices[num_train:num_train + num_test]
    
    # 提取训练集和测试集
    theta_train = theta_rad[index_train, :]  # (num_train, 7)
    pos_train = pos_m[index_train, :].T  # (3, num_train)
    theta_test = theta_rad[index_test, :]  # (num_test, 7)
    pos_test = pos_m[index_test, :].T  # (3, num_test)
    
    # 初始化
    iden_kesi = kesi_init.copy()  # 使用初始旋量
    iden_pe0 = pe0_init.copy()  # 使用初始工具坐标系平移
    iden_yita = np.zeros_like(kesi_init)  # 旋量增量初始为0
    
    if verbose:
        print(f"开始旋量标定...")
        print(f"训练集: {num_train}组, 测试集: {num_test}组")
        print(f"正则化系数: {lambda_reg}")
    
    # ========== 主优化循环 ==========
    for count in range(max_iter):
        # 计算所有训练集位形的末端位置和雅可比
        iden_pe = np.zeros((3, num_train))  # 理论末端位置
        J_pos_list = []  # 存储每个位形的雅可比（3×45）
        
        for i in range(num_train):
            # 正运动学计算
            g = np.eye(4)
            num_joint = kesi_init.shape[1]  # 7自由度
            for k in range(num_joint - 1, -1, -1):  # 从第7关节到第1关节
                g = exp_se3_r(iden_kesi[0:3, k], iden_kesi[3:6, k], 
                             theta_train[i, k]) @ g
            iden_pe[:, i] = (g @ iden_pe0)[0:3]
            
            # 计算位置雅可比（3×45）
            J_pos_i = jacobi_pos(iden_yita, kesi_init, theta_train[i, :],
                                iden_pe[:, i], g)
            J_pos_list.append(J_pos_i)
        
        # 构建等距离约束的雅可比和误差
        J_dist = []
        error_dist = []
        
        for i in range(num_train):
            for j in range(i + 1, num_train):
                # 距离误差（平方形式）
                dist_meas_sq = np.sum((pos_train[:, j] - pos_train[:, i])**2)
                dist_theo_sq = np.sum((iden_pe[:, j] - iden_pe[:, i])**2)
                error_ij = dist_meas_sq - dist_theo_sq
                error_dist.append(error_ij)
                
                # 距离约束的雅可比
                # J_dist_ij = 2 * (pos_j - pos_i)^T * (J_pos_j - J_pos_i)
                pos_diff = iden_pe[:, j] - iden_pe[:, i]
                J_pos_diff = J_pos_list[j] - J_pos_list[i]
                J_dist_ij = 2 * pos_diff.T @ J_pos_diff  # (1, 45)
                J_dist.append(J_dist_ij)
        
        J_dist = np.array(J_dist)  # (num_constraints, 45)
        error_dist = np.array(error_dist)  # (num_constraints,)
        
        # 去除零列（数值稳定性）
        select_column = np.where(np.sum(J_dist**2, axis=0) > 0)[0]
        J_dist_selected = J_dist[:, select_column]
        
        # 正则化伪逆求解
        # inc = (J^T J + lambda*I)^{-1} J^T * error
        JtJ = J_dist_selected.T @ J_dist_selected
        JtJ_reg = JtJ + lambda_reg * np.eye(JtJ.shape[0])
        inc_yita_pe0_selected = np.linalg.solve(JtJ_reg, 
                                                J_dist_selected.T @ error_dist)
        
        # 恢复完整增量向量
        num_vars = num_joint * 6 + 3  # 7×6+3=45（旋量增量+工具坐标系平移）
        inc_yita_pe0 = np.zeros(num_vars)
        inc_yita_pe0[select_column] = inc_yita_pe0_selected
        
        # 更新旋量增量（前num_joint*6个）
        num_yita_vars = num_joint * 6  # 7×6=42
        inc_yita_flat = inc_yita_pe0[:num_yita_vars]
        iden_yita_flat = iden_yita.flatten('F')  # 按列展开
        iden_yita_flat[select_column[select_column < num_yita_vars]] += \
            inc_yita_flat[select_column[select_column < num_yita_vars]]
        iden_yita = iden_yita_flat.reshape(iden_yita.shape, order='F')
        
        # 更新工具坐标系平移（后3个）
        pe0_indices = select_column[select_column >= num_yita_vars] - num_yita_vars
        if len(pe0_indices) > 0:
            iden_pe0[pe0_indices] += inc_yita_pe0[select_column[select_column >= num_yita_vars]]
        
        # 根据旋量增量更新旋量参数
        for i in range(num_joint):
            iden_kesi[:, i] = ad_g(yita_exp_diff(iden_yita[:, i])) @ kesi_init[:, i]
        
        # 检查收敛
        error_norm = np.linalg.norm(error_dist)
        inc_norm = np.linalg.norm(inc_yita_pe0_selected)
        
        if verbose and (count % 10 == 0 or count == max_iter - 1):
            print(f"迭代 {count+1}/{max_iter}: 误差范数={error_norm:.6e}, "
                  f"增量范数={inc_norm:.6e}")
        
        if inc_norm < tol:
            if verbose:
                print(f"收敛于第 {count+1} 次迭代")
            break
    
    if verbose:
        print(f"\n标定完成！")
        print(f"迭代次数: {count + 1}")
    
    return {
        'iden_kesi': iden_kesi,
        'iden_pe0': iden_pe0,
        'num_iterations': count + 1
    }


# ==================== 主程序 ====================

if __name__ == "__main__":
    # ========== 配置参数 ==========
    # 文件路径（需要根据实际情况修改）
    base_dir = Path(__file__).parent
    config_dir = base_dir / "config"
    input_dir = base_dir / "input"
    output_dir = base_dir / "output"
    
    # 确保输出目录存在
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # ========== 加载数据 ==========
    print("加载数据...")
    
    # 加载初始旋量参数（6×7，单位：米）
    kesi_file = config_dir / "theory_kesi.txt"
    if kesi_file.exists():
        kesi_init = np.loadtxt(kesi_file)
    else:
        raise FileNotFoundError(f"找不到旋量文件: {kesi_file}")
    
    # 加载关节角数据（N×7，单位：度）
    theta_file = input_dir / "calibration_joints.txt"
    if theta_file.exists():
        theta_data = np.loadtxt(theta_file).T  # 转置为N×7
    else:
        raise FileNotFoundError(f"找不到关节角文件: {theta_file}")
    
    # 加载末端位置数据（N×3，单位：毫米）
    pos_file = input_dir / "calibration_points.txt"
    if pos_file.exists():
        pos_data = np.loadtxt(pos_file).T  # 转置为N×3
    else:
        raise FileNotFoundError(f"找不到位置文件: {pos_file}")
    
    # 加载工具坐标系平移（4×4矩阵，取第4列，单位：米）
    pe0_file = config_dir / "target_flange_matrix.txt"
    if pe0_file.exists():
        pe0_matrix = np.loadtxt(pe0_file)
        pe0_init = pe0_matrix[0:4, 3:4]  # 取第4列
    else:
        raise FileNotFoundError(f"找不到工具坐标系文件: {pe0_file}")
    
    print(f"数据加载完成:")
    print(f"  旋量参数: {kesi_init.shape}")
    print(f"  关节角数据: {theta_data.shape}")
    print(f"  位置数据: {pos_data.shape}")
    print(f"  工具坐标系: {pe0_init.shape}")
    
    # ========== 执行标定 ==========
    result = calibrate_robot_screw(
        kesi_init=kesi_init,
        theta_data=theta_data,
        pos_data=pos_data,
        pe0_init=pe0_init,
        num_train=80,
        num_test=40,
        lambda_reg=1e-6,
        max_iter=100,
        verbose=True
    )
    
    # ========== 保存结果 ==========
    print("\n保存结果...")
    
    # 保存标定后的旋量参数
    iden_kesi_file = output_dir / "iden_kesi.txt"
    np.savetxt(iden_kesi_file, result['iden_kesi'], fmt='%.10e')
    print(f"标定后的旋量参数已保存: {iden_kesi_file}")
    
    # 保存工具坐标系平移
    iden_pe0_file = output_dir / "iden_pe0.txt"
    np.savetxt(iden_pe0_file, result['iden_pe0'], fmt='%.10e')
    print(f"标定后的工具坐标系平移已保存: {iden_pe0_file}")
    
    print(f"\n迭代次数: {result['num_iterations']}")
    print("\n标定完成！")
    
    # ========== 询问是否进行验证 ==========
    print("\n" + "="*60)
    print("是否现在进行标定结果验证？")
    print("="*60)
    user_input = input("输入 'y' 或 'yes' 进行验证，其他任意键跳过: ").strip().lower()
    
    if user_input in ['y', 'yes']:
        print("\n开始验证...")
        try:
            from calibration_verify import verify_calibration
            
            verify_calibration(
                iden_kesi=result['iden_kesi'],
                iden_pe0=result['iden_pe0'],
                theta_data=theta_data,
                pos_data=pos_data,
                num_train=80,
                num_test=40,
                output_dir=output_dir,
                num_iterations=result['num_iterations'],
                verbose=True
            )
        except Exception as e:
            print(f"\n验证过程中出现错误: {e}")
            print("您可以稍后单独运行 calibration_verify.py 进行验证")
    else:
        print("\n跳过验证。")
        print("您可以稍后运行以下命令进行验证：")
        print(f"  python {Path(__file__).parent / 'calibration_verify.py'}")

