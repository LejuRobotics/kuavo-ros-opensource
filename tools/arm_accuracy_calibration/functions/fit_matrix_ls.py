#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
最小二乘法拟合齐次变换矩阵：从CSV读取多个矩阵，筛选后拟合为一个最优矩阵。
"""

import numpy as np
import pandas as pd
import argparse
import os
from scipy.linalg import svd


def load_matrices_from_csv(csv_path):
    """
    从CSV文件加载齐次变换矩阵。
    
    Args:
        csv_path: CSV文件路径，格式为 timestamp, m00, m01, ..., m33
        
    Returns:
        matrices: (N, 4, 4) 数组，N个4x4齐次变换矩阵
        timestamps: (N,) 数组，对应的时间戳
    """
    df = pd.read_csv(csv_path)
    
    # 检查必需的列
    required_cols = [f"m{r}{c}" for r in range(4) for c in range(4)]
    missing_cols = [c for c in required_cols if c not in df.columns]
    if missing_cols:
        raise ValueError(f"CSV文件缺少必需的列: {missing_cols}")
    
    # 提取时间戳
    timestamps = df["timestamp"].values if "timestamp" in df.columns else None
    
    # 提取矩阵数据（行优先）
    matrices = []
    for idx, row in df.iterrows():
        matrix_flat = [row[f"m{r}{c}"] for r in range(4) for c in range(4)]
        matrix = np.array(matrix_flat).reshape(4, 4)
        matrices.append(matrix)
    
    matrices = np.array(matrices)
    return matrices, timestamps


def filter_matrices_by_x_position(matrices, x_threshold=999.0):
    """
    根据x位置（m03，矩阵的平移x分量）筛选矩阵。
    
    Args:
        matrices: (N, 4, 4) 数组
        x_threshold: x位置阈值，大于此值的矩阵将被筛掉
        
    Returns:
        filtered_matrices: 筛选后的矩阵数组
        mask: (N,) 布尔数组，True表示保留
    """
    x_positions = matrices[:, 0, 3]  # m03是x方向的平移
    mask = np.abs(x_positions) <= x_threshold
    filtered_matrices = matrices[mask]
    return filtered_matrices, mask


def fit_transform_matrix_ls(matrices):
    """
    使用最小二乘法拟合多个齐次变换矩阵为一个最优矩阵。
    
    方法：
    1. 旋转部分：对所有旋转矩阵求平均，然后用SVD分解得到最接近的正交矩阵
    2. 平移部分：直接求均值
    
    Args:
        matrices: (N, 4, 4) 数组，N个齐次变换矩阵
        
    Returns:
        T_fit: (4, 4) 拟合得到的齐次变换矩阵
    """
    if len(matrices) == 0:
        raise ValueError("矩阵数量为0，无法拟合")
    
    # 提取旋转部分 (3x3) 和平移部分 (3,)
    rotations = matrices[:, :3, :3]  # (N, 3, 3)
    translations = matrices[:, :3, 3]  # (N, 3)
    
    # 平移部分：直接求均值
    translation_mean = np.mean(translations, axis=0)
    
    # 旋转部分：对所有旋转矩阵求平均，然后用SVD分解得到正交矩阵
    # 方法：R_mean = mean(R_i)，然后用SVD分解 R_mean = U * S * V^T
    # 最优旋转矩阵为 R_opt = U * V^T（去除缩放）
    rotation_mean = np.mean(rotations, axis=0)
    
    # SVD分解
    U, s, Vt = svd(rotation_mean)
    R_opt = U @ Vt
    
    # 确保行列式为1（保证是旋转矩阵，不是反射）
    if np.linalg.det(R_opt) < 0:
        U[:, -1] *= -1
        R_opt = U @ Vt
    
    # 组合成齐次变换矩阵
    T_fit = np.eye(4)
    T_fit[:3, :3] = R_opt
    T_fit[:3, 3] = translation_mean
    
    return T_fit


def print_matrix(T, name="矩阵"):
    """打印4x4矩阵，格式化输出"""
    print(f"\n{name}:")
    print("=" * 60)
    for i in range(4):
        print(f"  [{T[i,0]:12.6f} {T[i,1]:12.6f} {T[i,2]:12.6f} {T[i,3]:12.6f}]")
    print("=" * 60)


def save_matrix_to_csv(T, output_path, verbose=False):
    """将矩阵保存为CSV文件"""
    os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else ".", exist_ok=True)
    
    # 展平为行优先
    matrix_flat = [T[r, c] for r in range(4) for c in range(4)]
    
    df = pd.DataFrame({
        "m00": [T[0, 0]], "m01": [T[0, 1]], "m02": [T[0, 2]], "m03": [T[0, 3]],
        "m10": [T[1, 0]], "m11": [T[1, 1]], "m12": [T[1, 2]], "m13": [T[1, 3]],
        "m20": [T[2, 0]], "m21": [T[2, 1]], "m22": [T[2, 2]], "m23": [T[2, 3]],
        "m30": [T[3, 0]], "m31": [T[3, 1]], "m32": [T[3, 2]], "m33": [T[3, 3]],
    })
    
    df.to_csv(output_path, index=False)
    if verbose:
        print(f"\n拟合矩阵已保存到: {output_path}")


def fit_matrix_from_csv(csv_path, x_threshold=999.0, verbose=True):
    """
    从CSV文件加载矩阵并拟合，返回拟合结果和统计信息。
    
    Args:
        csv_path: CSV文件路径（包含齐次变换矩阵）
        x_threshold: x位置阈值，大于此值的矩阵将被筛掉（默认: 999.0）
        verbose: 是否打印详细信息（默认: True）
        
    Returns:
        dict: 包含以下键的字典
            - 'T_fit': (4, 4) 拟合得到的齐次变换矩阵（平移部分单位为mm）
            - 'matrices_filtered': 筛选后的矩阵数组
            - 'n_total': 总矩阵数量
            - 'n_filtered': 筛选后保留的矩阵数量
            - 'n_removed': 被剔除的矩阵数量
            - 'errors': dict包含平移误差、旋转误差等统计信息
            - 'x_stats': dict包含x位置统计信息
            - 'unit': 返回值的单位（'mm'）
            
    注意：
        - T_fit可以直接传递给calibrate_dh_parameters作为T_meas参数
        - T_fit的平移部分单位为mm
    """
    if verbose:
        print("=" * 60)
        print("最小二乘法拟合齐次变换矩阵")
        print("=" * 60)
        print(f"输入文件: {csv_path}")
        print(f"X位置阈值: {x_threshold}")
        print("=" * 60)
    
    # 加载矩阵
    if verbose:
        print("\n正在加载矩阵...")
    matrices, timestamps = load_matrices_from_csv(csv_path)
    n_total = len(matrices)
    if verbose:
        print(f"成功加载 {n_total} 个矩阵")
    
    # 筛选矩阵（x位置 <= threshold）
    if verbose:
        print(f"\n正在筛选矩阵（x位置 <= {x_threshold}）...")
    matrices_filtered, mask = filter_matrices_by_x_position(matrices, x_threshold)
    n_filtered = len(matrices_filtered)
    n_removed = np.sum(~mask)
    if verbose:
        print(f"筛选结果: 保留 {n_filtered} 个矩阵，剔除 {n_removed} 个矩阵")
    
    if n_filtered == 0:
        raise ValueError(f"筛选后没有剩余矩阵，请检查x_threshold参数（当前: {x_threshold}）")
    
    # 统计x位置信息
    x_positions = matrices[:, 0, 3]
    x_positions_filtered = matrices_filtered[:, 0, 3]
    x_stats = {
        'before': {
            'min': float(np.min(x_positions)),
            'max': float(np.max(x_positions)),
            'mean': float(np.mean(x_positions)),
            'std': float(np.std(x_positions))
        },
        'after': {
            'min': float(np.min(x_positions_filtered)),
            'max': float(np.max(x_positions_filtered)),
            'mean': float(np.mean(x_positions_filtered)),
            'std': float(np.std(x_positions_filtered))
        }
    }
    
    if verbose:
        print(f"\nX位置统计（筛选前）:")
        print(f"  最小值: {x_stats['before']['min']:.6f}")
        print(f"  最大值: {x_stats['before']['max']:.6f}")
        print(f"  均值: {x_stats['before']['mean']:.6f}")
        print(f"  标准差: {x_stats['before']['std']:.6f}")
        print(f"\nX位置统计（筛选后）:")
        print(f"  最小值: {x_stats['after']['min']:.6f}")
        print(f"  最大值: {x_stats['after']['max']:.6f}")
        print(f"  均值: {x_stats['after']['mean']:.6f}")
        print(f"  标准差: {x_stats['after']['std']:.6f}")
    
    # 拟合矩阵
    if verbose:
        print("\n正在使用最小二乘法拟合矩阵...")
    T_fit = fit_transform_matrix_ls(matrices_filtered)
    
    if verbose:
        print_matrix(T_fit, "拟合得到的齐次变换矩阵")
    
    # 计算拟合误差统计
    errors_translation = []
    errors_rotation = []
    errors_rotation_angle = []
    
    for T in matrices_filtered:
        # 平移误差（欧氏距离）
        err_t = np.linalg.norm(T[:3, 3] - T_fit[:3, 3])
        errors_translation.append(err_t)
        
        # 旋转误差（Frobenius范数）
        err_R_fro = np.linalg.norm(T[:3, :3] - T_fit[:3, :3], 'fro')
        errors_rotation.append(err_R_fro)
        
        # 旋转角度误差（通过相对旋转矩阵的迹计算角度）
        R_rel = T_fit[:3, :3].T @ T[:3, :3]
        trace = np.trace(R_rel)
        trace = np.clip(trace, -1.0, 3.0)
        angle_rad = np.arccos((trace - 1.0) / 2.0)
        angle_deg = np.degrees(angle_rad)
        errors_rotation_angle.append(angle_deg)
    
    errors_translation = np.array(errors_translation)
    errors_rotation = np.array(errors_rotation)
    errors_rotation_angle = np.array(errors_rotation_angle)
    
    errors = {
        'translation': {
            'mean': float(np.mean(errors_translation)),
            'std': float(np.std(errors_translation)),
            'max': float(np.max(errors_translation)),
            'min': float(np.min(errors_translation))
        },
        'rotation_frobenius': {
            'mean': float(np.mean(errors_rotation)),
            'std': float(np.std(errors_rotation)),
            'max': float(np.max(errors_rotation))
        },
        'rotation_angle': {
            'mean': float(np.mean(errors_rotation_angle)),
            'std': float(np.std(errors_rotation_angle)),
            'max': float(np.max(errors_rotation_angle))
        }
    }
    
    if verbose:
        print("\n拟合误差统计:")
        print(f"  平移误差（欧氏距离，单位mm）:")
        print(f"    均值: {errors['translation']['mean']:.6f}")
        print(f"    标准差: {errors['translation']['std']:.6f}")
        print(f"    最大值: {errors['translation']['max']:.6f}")
        print(f"    最小值: {errors['translation']['min']:.6f}")
        print(f"  旋转误差（Frobenius范数）:")
        print(f"    均值: {errors['rotation_frobenius']['mean']:.6f}")
        print(f"    标准差: {errors['rotation_frobenius']['std']:.6f}")
        print(f"    最大值: {errors['rotation_frobenius']['max']:.6f}")
        print(f"  旋转角度误差（度）:")
        print(f"    均值: {errors['rotation_angle']['mean']:.6f}")
        print(f"    标准差: {errors['rotation_angle']['std']:.6f}")
        print(f"    最大值: {errors['rotation_angle']['max']:.6f}")
    
    return {
        'T_fit': T_fit,  # (4, 4) numpy array，平移部分单位为mm，可直接传递给calibrate_dh_parameters
        'matrices_filtered': matrices_filtered,  # (N, 4, 4) numpy array
        'n_total': n_total,
        'n_filtered': n_filtered,
        'n_removed': n_removed,
        'errors': errors,
        'x_stats': x_stats,
        'unit': 'mm'  # 返回值的单位
    }


def calibrate_dh_from_csv_files(matrix_csv_path, axis_csv_path, x_threshold=999.0, 
                                 reference_axis_name='y', x_filter_threshold=900.0,
                                 epsilon_theta=None, epsilon_d=None, max_iterations=100,
                                 verbose=True):
    """
    从CSV文件拟合矩阵和旋转轴，然后进行DH参数标定。
    
    这是一个便捷函数，将fit_matrix_from_csv和fit_rotation_axis_from_csv的结果
    直接传递给calibrate_dh_parameters。
    
    Args:
        matrix_csv_path: 矩阵CSV文件路径（包含齐次变换矩阵）
        axis_csv_path: 轴线CSV文件路径（包含位置点）
        x_threshold: 矩阵筛选的x位置阈值（默认: 999.0）
        reference_axis_name: 参考轴名称（'x', 'y', 'z'），默认: 'y'
        x_filter_threshold: 轴线点筛选的X坐标阈值（默认: 900.0）
        epsilon_theta: 方向偏差收敛阈值（弧度），默认: None（使用默认值）
        epsilon_d: 位置偏差收敛阈值（长度单位，mm），默认: None（使用默认值）
        max_iterations: 最大迭代次数（默认: 100）
        verbose: 是否打印详细信息（默认: True）
        
    Returns:
        dict: calibrate_dh_parameters的返回值，包含dh_final等字段
    """
    from .dh_calibration import calibrate_dh_parameters
    
    # 拟合矩阵
    if verbose:
        print("=" * 70)
        print("步骤1: 拟合齐次变换矩阵")
        print("=" * 70)
    matrix_result = fit_matrix_from_csv(matrix_csv_path, x_threshold=x_threshold, verbose=verbose)
    T_fit = matrix_result['T_fit']
    
    # 拟合旋转轴（单位设为mm，与矩阵保持一致）
    if verbose:
        print("\n" + "=" * 70)
        print("步骤2: 拟合旋转轴")
        print("=" * 70)
    axis_result = fit_rotation_axis_from_csv(
        axis_csv_path, 
        reference_axis_name=reference_axis_name,
        x_filter_threshold=x_filter_threshold,
        verbose=verbose,
        unit='mm'  # 使用mm单位，与矩阵保持一致
    )
    P_meas = axis_result['center']  # (3,) numpy array，单位mm
    n_meas = axis_result['axis']    # (3,) numpy array，单位向量
    
    # 调用DH参数标定
    if verbose:
        print("\n" + "=" * 70)
        print("步骤3: DH参数迭代标定")
        print("=" * 70)
        print(f"输入:")
        print(f"  T_meas: 4x4矩阵（平移单位: mm）")
        print(f"  P_meas: {P_meas} (单位: mm)")
        print(f"  n_meas: {n_meas} (单位向量)")
    
    dh_result = calibrate_dh_parameters(
        T_meas=T_fit,
        P_meas=P_meas,
        n_meas=n_meas,
        epsilon_theta=epsilon_theta,
        epsilon_d=epsilon_d,
        max_iterations=max_iterations,
        verbose=verbose
    )
    
    return dh_result


def main():
    parser = argparse.ArgumentParser(
        description="最小二乘法拟合齐次变换矩阵",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python fit_matrix_ls.py matrix_data/link1/T_joint2_in_joint1.csv
  python fit_matrix_ls.py matrix_data/link1/T_joint2_in_joint1.csv --x_threshold 500 --output fitted_matrix.csv
        """
    )
    
    parser.add_argument("csv_path", type=str, help="输入的CSV文件路径（包含齐次变换矩阵）")
    parser.add_argument("--x_threshold", type=float, default=999.0,
                        help="x位置阈值，大于此值的矩阵将被筛掉（默认: 999.0）")
    parser.add_argument("--output", "-o", type=str, default=None,
                        help="输出CSV文件路径（可选，默认不保存）")
    
    args = parser.parse_args()
    
    # 调用主函数
    result = fit_matrix_from_csv(args.csv_path, args.x_threshold, verbose=True)
    
    # 保存结果
    if args.output:
        save_matrix_to_csv(result['T_fit'], args.output)
    
    print("\n完成！")


if __name__ == "__main__":
    main()
