#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
7自由度机器人旋量标定结果验证脚本

功能：
- 加载标定后的旋量参数和工具坐标系平移
- 计算训练集和测试集的位置误差
- 生成误差统计报告和可视化图表

使用方法：
1. 单独运行：python calibration_verify.py
2. 从标定脚本调用：在标定完成后选择验证

作者：基于MATLAB代码转换
日期：2025
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# 从本地 function 目录导入工具函数
from function.screw_utils import forward_kinematics
from function.data_utils import svd_alignment


def verify_calibration(
    iden_kesi,          # 标定后的旋量参数（6×7），米
    iden_pe0,           # 标定后的工具坐标系平移（4×1），米
    theta_data,         # 关节角数据（N×7），度
    pos_data,           # 末端位置数据（N×3），毫米
    num_train=80,       # 训练集数量
    num_test=40,        # 测试集数量
    output_dir=None,    # 输出目录，如果提供则保存结果
    num_iterations=None, # 迭代次数（用于保存到报告）
    verbose=True
):
    """
    验证标定结果
    
    Args:
        iden_kesi: 标定后的旋量参数（6×7），米
        iden_pe0: 标定后的工具坐标系平移（4×1），米
        theta_data: 关节角数据（N×7），度
        pos_data: 末端位置数据（N×3），毫米
        num_train: 训练集数量
        num_test: 测试集数量
        output_dir: 输出目录，如果提供则自动保存误差统计和绘图
        num_iterations: 迭代次数（可选，用于保存到报告）
        verbose: 是否打印详细信息
    
    Returns:
        dict: 包含验证结果的字典
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
    
    # 使用与标定相同的随机种子划分训练集和测试集
    np.random.seed(42)  # 固定随机种子，与标定脚本一致
    indices = np.random.permutation(num_total)
    index_train = indices[:num_train]
    index_test = indices[num_train:num_train + num_test]
    
    # 提取训练集和测试集
    theta_train = theta_rad[index_train, :]  # (num_train, 7)
    pos_train = pos_m[index_train, :].T  # (3, num_train)
    theta_test = theta_rad[index_test, :]  # (num_test, 7)
    pos_test = pos_m[index_test, :].T  # (3, num_test)
    
    if verbose:
        print(f"开始验证标定结果...")
        print(f"训练集: {num_train}组, 测试集: {num_test}组")
    
    # ========== 计算训练集误差 ==========
    iden_pe_train = np.zeros((3, num_train))
    for i in range(num_train):
        iden_pe_train[:, i] = forward_kinematics(iden_kesi, theta_train[i, :], iden_pe0)
    
    # SVD配准（消除坐标系偏差）
    alignment_result = svd_alignment(pos_train, iden_pe_train)
    opt_R = alignment_result['R']
    opt_t = alignment_result['t']
    
    # 训练集位置误差
    train_errors = alignment_result['errors'] * 1000  # 转毫米
    
    # ========== 计算测试集误差 ==========
    iden_pe_test = np.zeros((3, num_test))
    for i in range(num_test):
        iden_pe_test[:, i] = forward_kinematics(iden_kesi, theta_test[i, :], iden_pe0)
    
    # 使用相同的配准参数计算测试集误差
    aligned_test = opt_R @ iden_pe_test + opt_t
    test_errors = np.linalg.norm(pos_test - aligned_test, axis=0) * 1000  # 转毫米
    
    if verbose:
        print(f"\n验证完成！")
        print(f"训练集误差: 平均={np.mean(train_errors):.3f}mm, "
              f"最大={np.max(train_errors):.3f}mm, "
              f"最小={np.min(train_errors):.3f}mm, "
              f"标准差={np.std(train_errors):.3f}mm")
        print(f"测试集误差: 平均={np.mean(test_errors):.3f}mm, "
              f"最大={np.max(test_errors):.3f}mm, "
              f"最小={np.min(test_errors):.3f}mm, "
              f"标准差={np.std(test_errors):.3f}mm")
    
    result = {
        'train_errors': train_errors,
        'test_errors': test_errors,
        'opt_R': opt_R,
        'opt_t': opt_t
    }
    
    # 如果提供了输出目录，自动保存结果和绘图
    if output_dir is not None:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # 保存误差统计
        error_file = output_dir / "calibration_errors.txt"
        with open(error_file, 'w') as f:
            f.write(f"训练集误差 (mm):\n")
            f.write(f"  平均: {np.mean(train_errors):.6f}\n")
            f.write(f"  最大: {np.max(train_errors):.6f}\n")
            f.write(f"  最小: {np.min(train_errors):.6f}\n")
            f.write(f"  标准差: {np.std(train_errors):.6f}\n")
            f.write(f"\n测试集误差 (mm):\n")
            f.write(f"  平均: {np.mean(test_errors):.6f}\n")
            f.write(f"  最大: {np.max(test_errors):.6f}\n")
            f.write(f"  最小: {np.min(test_errors):.6f}\n")
            f.write(f"  标准差: {np.std(test_errors):.6f}\n")
            if num_iterations is not None:
                f.write(f"\n迭代次数: {num_iterations}\n")
        if verbose:
            print(f"\n误差统计已保存: {error_file}")
        
        # 绘制结果
        if verbose:
            print("绘制误差曲线...")
        plt.figure(figsize=(12, 5))
        
        # 训练集误差
        plt.subplot(1, 2, 1)
        plt.plot(train_errors, 'bo-', markersize=4)
        plt.axhline(y=np.mean(train_errors), color='r', linestyle='--', 
                    label=f'平均={np.mean(train_errors):.3f}mm')
        plt.xlabel('样本索引')
        plt.ylabel('位置误差 (mm)')
        plt.title('训练集误差')
        plt.legend()
        plt.grid(True)
        
        # 测试集误差
        plt.subplot(1, 2, 2)
        plt.plot(test_errors, 'ro-', markersize=4)
        plt.axhline(y=np.mean(test_errors), color='r', linestyle='--',
                    label=f'平均={np.mean(test_errors):.3f}mm')
        plt.xlabel('样本索引')
        plt.ylabel('位置误差 (mm)')
        plt.title('测试集误差')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plot_file = output_dir / "calibration_errors.png"
        plt.savefig(plot_file, dpi=150)
        if verbose:
            print(f"误差曲线已保存: {plot_file}")
    
    return result


# ==================== 主程序 ====================

if __name__ == "__main__":
    # ========== 配置参数 ==========
    base_dir = Path(__file__).parent
    config_dir = base_dir / "config"
    input_dir = base_dir / "input"
    output_dir = base_dir / "output"
    
    # 确保输出目录存在
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # ========== 加载标定结果 ==========
    print("加载标定结果...")
    
    # 加载标定后的旋量参数（6×7，单位：米）
    iden_kesi_file = output_dir / "iden_kesi.txt"
    if iden_kesi_file.exists():
        iden_kesi = np.loadtxt(iden_kesi_file)
    else:
        raise FileNotFoundError(f"找不到标定后的旋量文件: {iden_kesi_file}\n"
                                f"请先运行 calibrate_screw.py 进行标定")
    
    # 加载标定后的工具坐标系平移（4×1或3×1，单位：米）
    iden_pe0_file = output_dir / "iden_pe0.txt"
    if iden_pe0_file.exists():
        pe0_data = np.loadtxt(iden_pe0_file)
        if pe0_data.ndim == 1:
            iden_pe0 = pe0_data.reshape(-1, 1)
        else:
            iden_pe0 = pe0_data
        # 确保是4×1格式
        if iden_pe0.shape[0] == 3:
            iden_pe0 = np.vstack([iden_pe0, np.array([[1.0]])])
    else:
        raise FileNotFoundError(f"找不到标定后的工具坐标系文件: {iden_pe0_file}\n"
                                f"请先运行 calibrate_screw.py 进行标定")
    
    # ========== 加载数据 ==========
    print("加载数据...")
    
    # 加载关节角数据（N×7，单位：度）
    theta_file = input_dir / "calibration_joints.txt"
    if theta_file.exists():
        theta_data = np.loadtxt(theta_file)
        # 如果是列存储格式，转置
        if theta_data.shape[0] == 7:
            theta_data = theta_data.T
    else:
        raise FileNotFoundError(f"找不到关节角文件: {theta_file}")
    
    # 加载末端位置数据（N×3，单位：毫米）
    pos_file = input_dir / "calibration_points.txt"
    if pos_file.exists():
        pos_data = np.loadtxt(pos_file)
        # 如果是列存储格式，转置
        if pos_data.shape[0] == 3:
            pos_data = pos_data.T
    else:
        raise FileNotFoundError(f"找不到位置文件: {pos_file}")
    
    print(f"数据加载完成:")
    print(f"  标定旋量参数: {iden_kesi.shape}")
    print(f"  工具坐标系: {iden_pe0.shape}")
    print(f"  关节角数据: {theta_data.shape}")
    print(f"  位置数据: {pos_data.shape}")
    
    # ========== 执行验证 ==========
    verify_calibration(
        iden_kesi=iden_kesi,
        iden_pe0=iden_pe0,
        theta_data=theta_data,
        pos_data=pos_data,
        num_train=80,
        num_test=40,
        output_dir=output_dir,
        verbose=True
    )
    
    print("\n验证完成！")

