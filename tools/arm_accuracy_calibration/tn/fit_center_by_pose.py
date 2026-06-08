#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
点云筛选与中心拟合脚本

功能：
1. 从 CSV 文件读取点云数据
2. 使用 3-sigma 规则筛选异常点
3. 计算筛选后点的几何中心
4. 可视化：绿色=筛选后的点，红色=被筛除的点，黄色大球=中心
5. 保存筛选后的 CSV 和中心坐标

用法：
    python3 filter_and_fit_center.py
"""

import os
import sys
import argparse
import re
import yaml
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  # for 3D projection

# ============================================================================
# 配置区域 - 在这里修改参数
# ============================================================================

# Sigma 倍数，用于筛选离群点（3.0 表示 3-sigma 规则）
SIGMA_MULTIPLIER = 3.0

# 是否保存筛选后的 CSV 文件
SAVE_FILTERED = True

# 是否显示可视化
SHOW_PLOT = True

# 是否更新 fitting_data.yaml
UPDATE_FITTING_YAML = True

# 要处理的 CSV 文件列表（相对于脚本所在目录的路径）
CSV_PATHS = [
    "pose_data/belly_pose.csv",
    "pose_data/shoulder_pose.csv",
    "pose_data/effector_pose.csv",
    "pose_data/elbow_pose.csv",
]

# ============================================================================


def quaternion_mean(quats):
    """
    计算四元数的平均值（使用特征值方法）。

    参数:
        quats: numpy array, shape (N, 4)，每行是一个四元数 [x, y, z, w]

    返回:
        mean_quat: numpy array, shape (4,)，平均四元数 [x, y, z, w]
    """
    if len(quats) == 0:
        return np.array([0, 0, 0, 1])
    
    # 归一化所有四元数
    Q = np.array(quats, dtype=float)
    norms = np.linalg.norm(Q, axis=1, keepdims=True)
    norms[norms == 0] = 1.0
    Q = Q / norms
    
    # 使用特征值方法计算平均四元数
    A = Q.T @ Q
    eigvals, eigvecs = np.linalg.eigh(A)
    mean_q = eigvecs[:, np.argmax(eigvals)]
    
    # 确保 w 分量为正（四元数的双倍覆盖）
    if mean_q[3] < 0:
        mean_q = -mean_q
    
    # 归一化
    mean_q = mean_q / np.linalg.norm(mean_q)
    return mean_q


def quaternion_angular_distance(q1, q2):
    """
    计算两个四元数之间的角度距离（弧度）。

    参数:
        q1, q2: numpy array, shape (4,)，四元数 [x, y, z, w]

    返回:
        angle: float，角度距离（弧度）
    """
    # 归一化
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    
    # 计算点积（考虑四元数的双倍覆盖，取绝对值）
    dot_product = np.abs(np.dot(q1, q2))
    dot_product = min(dot_product, 1.0)  # 数值稳定性
    
    # 计算角度
    angle = 2 * np.arccos(dot_product)
    return angle


def filter_poses_by_sigma(points, quats, sigma_multiplier=3.0):
    """
    使用 3-sigma 规则筛选点云，优先按位置筛选，再按旋转筛选。

    参数:
        points: numpy array, shape (N, 3)，每行是一个点的 [x, y, z]
        quats: numpy array, shape (N, 4)，每行是一个四元数 [x, y, z, w]
        sigma_multiplier: float，sigma 倍数，默认 3.0（即 3-sigma 规则）

    返回:
        filtered_points: numpy array，筛选后的点
        filtered_quats: numpy array，筛选后的四元数
        pos_removed_points: numpy array，因位置被筛除的点
        pos_removed_quats: numpy array，因位置被筛除的四元数
        rot_removed_points: numpy array，因旋转被筛除的点
        rot_removed_quats: numpy array，因旋转被筛除的四元数
        center: numpy array, shape (3,)，几何中心
        mean_quat: numpy array, shape (4,)，平均四元数
        pos_mean_dist: float，位置平均距离
        pos_std_dist: float，位置距离标准差
        rot_mean_dist: float，旋转平均角度距离
        rot_std_dist: float，旋转角度距离标准差
    """
    # 第一步：按位置筛选
    center = np.mean(points, axis=0)
    pos_distances = np.linalg.norm(points - center, axis=1)
    pos_mean_dist = np.mean(pos_distances)
    pos_std_dist = np.std(pos_distances)
    pos_threshold = pos_mean_dist + sigma_multiplier * pos_std_dist
    
    # 位置筛选掩码
    pos_mask = pos_distances <= pos_threshold
    pos_filtered_points = points[pos_mask]
    pos_filtered_quats = quats[pos_mask]
    pos_removed_points = points[~pos_mask]
    pos_removed_quats = quats[~pos_mask]
    
    # 第二步：在位置筛选后的点中，按旋转筛选
    if len(pos_filtered_points) > 0:
        mean_quat = quaternion_mean(pos_filtered_quats)
        # 向量化计算旋转距离（优化性能）
        # 归一化所有四元数
        Q = pos_filtered_quats.copy()
        norms = np.linalg.norm(Q, axis=1, keepdims=True)
        norms[norms == 0] = 1.0
        Q = Q / norms
        mean_q_norm = mean_quat / np.linalg.norm(mean_quat)
        # 计算点积（考虑四元数的双倍覆盖，取绝对值）
        dot_products = np.abs(np.sum(Q * mean_q_norm, axis=1))
        dot_products = np.clip(dot_products, 0, 1.0)  # 数值稳定性
        # 计算角度距离
        rot_distances = 2 * np.arccos(dot_products)
        rot_mean_dist = np.mean(rot_distances)
        rot_std_dist = np.std(rot_distances)
        rot_threshold = rot_mean_dist + sigma_multiplier * rot_std_dist
        
        # 旋转筛选掩码（在位置筛选后的点中）
        rot_mask = rot_distances <= rot_threshold
        filtered_points = pos_filtered_points[rot_mask]
        filtered_quats = pos_filtered_quats[rot_mask]
        rot_removed_points = pos_filtered_points[~rot_mask]
        rot_removed_quats = pos_filtered_quats[~rot_mask]
    else:
        # 如果位置筛选后没有点，则旋转筛选也没有点
        mean_quat = quaternion_mean(quats)  # 使用原始数据的平均
        filtered_points = np.array([]).reshape(0, 3)
        filtered_quats = np.array([]).reshape(0, 4)
        rot_removed_points = np.array([]).reshape(0, 3)
        rot_removed_quats = np.array([]).reshape(0, 4)
        rot_mean_dist = 0.0
        rot_std_dist = 0.0

    # 计算最终筛选掩码（用于获取原始索引）
    if len(pos_filtered_points) > 0:
        # 最终掩码 = 位置掩码 AND 旋转掩码（在位置筛选后的点中）
        final_mask = np.zeros(len(points), dtype=bool)
        final_mask[pos_mask] = rot_mask  # 在位置筛选后的点中应用旋转掩码
    else:
        final_mask = np.zeros(len(points), dtype=bool)
    
    return (filtered_points, filtered_quats, 
            pos_removed_points, pos_removed_quats,
            rot_removed_points, rot_removed_quats,
            center, mean_quat, 
            pos_mean_dist, pos_std_dist, rot_mean_dist, rot_std_dist,
            final_mask)


def fit_geometric_center(points):
    """
    计算点的几何中心（均值）。

    参数:
        points: numpy array, shape (N, 3)

    返回:
        center: numpy array, shape (3,)，几何中心 [x, y, z]
    """
    return np.mean(points, axis=0)


def fit_rotation_center(quats):
    """
    计算旋转的平均值（四元数平均）。

    参数:
        quats: numpy array, shape (N, 4)，每行是一个四元数 [x, y, z, w]

    返回:
        mean_quat: numpy array, shape (4,)，平均四元数 [x, y, z, w]
    """
    return quaternion_mean(quats)


def process_csv(csv_path, sigma_multiplier=3.0, save_filtered=True, show_plot=True):
    """
    处理单个 CSV 文件：筛选点云并拟合中心。

    参数:
        csv_path: str，CSV 文件路径
        sigma_multiplier: float，sigma 倍数
        save_filtered: bool，是否保存筛选后的 CSV
        show_plot: bool，是否显示可视化

    返回:
        dict: 包含处理结果的字典
    """
    # 读取 CSV
    if not os.path.isfile(csv_path):
        print(f"[WARN] 文件不存在: {csv_path}，跳过")
        return None

    print(f"\n[INFO] 处理: {csv_path}")
    df = pd.read_csv(csv_path)

    # 检查必需的列
    required_cols = ["pos_x", "pos_y", "pos_z"]
    for c in required_cols:
        if c not in df.columns:
            raise ValueError(f"{csv_path} 缺少列: {c}")

    # 提取点云数据
    points = df[["pos_x", "pos_y", "pos_z"]].values
    n_original = len(points)

    if n_original == 0:
        print(f"[WARN] {csv_path} 空文件，跳过")
        return None

    print(f"[INFO] 原始点数: {n_original}")

    # 检查是否有旋转数据
    has_rotation = all(col in df.columns for col in ["ori_x", "ori_y", "ori_z", "ori_w"])
    if has_rotation:
        quats = df[["ori_x", "ori_y", "ori_z", "ori_w"]].values
        print(f"[INFO] 检测到旋转数据，将同时考虑位置和旋转进行筛选")
    else:
        # 如果没有旋转数据，使用单位四元数
        quats = np.tile([0, 0, 0, 1], (n_original, 1))
        print(f"[WARN] 未检测到旋转数据，仅使用位置进行筛选")

    # 筛选点云（优先位置筛选，再旋转筛选）
    (filtered_points, filtered_quats, 
     pos_removed_points, pos_removed_quats,
     rot_removed_points, rot_removed_quats,
     initial_center, initial_mean_quat,
     pos_mean_dist, pos_std_dist, rot_mean_dist, rot_std_dist,
     final_mask) = filter_poses_by_sigma(
        points, quats, sigma_multiplier
    )
    n_filtered = len(filtered_points)
    n_pos_removed = len(pos_removed_points)
    n_rot_removed = len(rot_removed_points)

    # 获取筛选后的索引（用于保存 CSV）- 使用掩码高效获取
    filtered_indices = np.where(final_mask)[0]

    print(f"[INFO] 筛选后点数: {n_filtered} ({n_filtered/n_original*100:.2f}%)")
    print(f"[INFO] 位置筛除点数: {n_pos_removed} ({n_pos_removed/n_original*100:.2f}%)")
    if has_rotation:
        print(f"[INFO] 旋转筛除点数: {n_rot_removed} ({n_rot_removed/n_original*100:.2f}%)")
    print(f"[INFO] 位置距离统计: 均值={pos_mean_dist:.6f}, 标准差={pos_std_dist:.6f}, 阈值={pos_mean_dist + sigma_multiplier * pos_std_dist:.6f}")
    if has_rotation:
        print(f"[INFO] 旋转角度统计: 均值={rot_mean_dist:.6f}, 标准差={rot_std_dist:.6f}, 阈值={rot_mean_dist + sigma_multiplier * rot_std_dist:.6f}")

    # 拟合中心（使用筛选后的点）
    center = fit_geometric_center(filtered_points)
    mean_quat = fit_rotation_center(filtered_quats)
    print(f"[INFO] 拟合位置中心: ({center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f})")
    if has_rotation:
        print(f"[INFO] 拟合旋转中心: ({mean_quat[0]:.6f}, {mean_quat[1]:.6f}, {mean_quat[2]:.6f}, {mean_quat[3]:.6f})")

    # 保存筛选后的 CSV
    if save_filtered:
        base, ext = os.path.splitext(csv_path)
        out_path = base + "_filtered" + ext

        # 使用索引创建筛选后的 DataFrame
        filtered_df = df.iloc[filtered_indices].copy()
        filtered_df.to_csv(out_path, index=False)
        print(f"[INFO] 筛选后的点云已保存: {out_path}")

    # 保存中心坐标
    base, ext = os.path.splitext(csv_path)
    center_path = base + "_center.txt"
    with open(center_path, "w") as f:
        f.write(f"# 点云中心坐标和旋转（基于筛选后的点）\n")
        f.write(f"# 文件: {csv_path}\n")
        f.write(f"# 原始点数: {n_original}\n")
        f.write(f"# 筛选后点数: {n_filtered}\n")
        f.write(f"# Sigma 倍数: {sigma_multiplier}\n")
        f.write(f"# 位置中心坐标 (mm):\n")
        f.write(f"{center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f}\n")
        if has_rotation:
            f.write(f"# 旋转中心四元数 (x, y, z, w):\n")
            f.write(f"{mean_quat[0]:.6f}, {mean_quat[1]:.6f}, {mean_quat[2]:.6f}, {mean_quat[3]:.6f}\n")
    print(f"[INFO] 中心坐标已保存: {center_path}")

    # 可视化
    if show_plot:
        label = os.path.splitext(os.path.basename(csv_path))[0]

        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')

        # 绘制筛选后的点（绿色，size增大一倍）
        if n_filtered > 0:
            ax.scatter(
                filtered_points[:, 0],
                filtered_points[:, 1],
                filtered_points[:, 2],
                c='green',
                s=6,  # 从3增大到6
                alpha=0.6,
                label=f'Filtered points ({n_filtered})',
                edgecolors='none'
            )

        # 绘制因位置被筛除的点（红色，size增大一倍）
        if n_pos_removed > 0:
            ax.scatter(
                pos_removed_points[:, 0],
                pos_removed_points[:, 1],
                pos_removed_points[:, 2],
                c='red',
                s=4,  # 从2增大到4
                alpha=0.3,
                label=f'Position removed ({n_pos_removed})',
                edgecolors='none'
            )

        # 绘制因旋转被筛除的点（蓝色，size增大一倍）
        if has_rotation and n_rot_removed > 0:
            ax.scatter(
                rot_removed_points[:, 0],
                rot_removed_points[:, 1],
                rot_removed_points[:, 2],
                c='blue',
                s=4,  # 从2增大到4
                alpha=0.3,
                label=f'Rotation removed ({n_rot_removed})',
                edgecolors='none'
            )

        # 绘制中心（黄色大球）
        ax.scatter(
            [center[0]],
            [center[1]],
            [center[2]],
            c='yellow',
            s=200,
            alpha=0.9,
            marker='o',
            edgecolors='black',
            linewidths=2,
            label='Fitted center'
        )

        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        if has_rotation and n_rot_removed > 0:
            ax.set_title(f"{label} - Point Cloud Filtering and Center Fitting\n(green=filtered, red=position removed, blue=rotation removed, yellow=center)")
        else:
            ax.set_title(f"{label} - Point Cloud Filtering and Center Fitting\n(green=filtered, red=position removed, yellow=center)")
        ax.legend()

        # 保存图片
        image_dir = "image"
        os.makedirs(image_dir, exist_ok=True)
        png_path = os.path.join(image_dir, f"{label}_filtered_center.png")
        fig.tight_layout()
        fig.savefig(png_path, dpi=300, bbox_inches='tight', facecolor='white')
        print(f"[INFO] 可视化图片已保存: {png_path}")

        # 询问是否显示交互窗口
        resp = input("是否显示交互 3D 窗口？[y/N]: ").strip().lower()
        if resp in ("y", "yes", "1"):
            plt.show()
        plt.close(fig)

    return {
        "csv_path": csv_path,
        "n_original": n_original,
        "n_filtered": n_filtered,
        "n_pos_removed": n_pos_removed,
        "n_rot_removed": n_rot_removed,
        "center": center,
        "mean_quat": mean_quat,
        "has_rotation": has_rotation,
        "pos_mean_dist": pos_mean_dist,
        "pos_std_dist": pos_std_dist,
        "rot_mean_dist": rot_mean_dist,
        "rot_std_dist": rot_std_dist,
    }


def update_fitting_yaml(results, script_dir):
    """
    将拟合结果更新到 fitting_data.yaml
    
    参数:
        results: process_csv 返回的结果列表
        script_dir: 脚本所在目录
    """
    fitting_yaml_path = os.path.join(script_dir, "offline_data", "fitting_data.yaml")
    
    # 读取现有的 fitting_data.yaml
    if os.path.exists(fitting_yaml_path):
        with open(fitting_yaml_path, 'r') as f:
            data = yaml.safe_load(f) or {}
    else:
        data = {}
    
    # 确保有 rigid_body_poses 字段
    if 'rigid_body_poses' not in data:
        data['rigid_body_poses'] = {}
    
    # 映射：CSV文件名 -> YAML中的键名
    name_mapping = {
        'belly_pose': 'belly',
        'shoulder_pose': 'shoulder',
        'effector_pose': 'effector',
        'elbow_pose': 'elbow',
    }
    
    # 从 results 更新 YAML
    updated_count = 0
    for result in results:
        csv_path = result['csv_path']
        csv_name = os.path.splitext(os.path.basename(csv_path))[0]
        
        if csv_name not in name_mapping:
            continue
        
        yaml_key = name_mapping[csv_name]
        center = result['center']
        mean_quat = result['mean_quat']
        has_rotation = result['has_rotation']
        
        # 更新 YAML 数据
        if yaml_key not in data['rigid_body_poses']:
            data['rigid_body_poses'][yaml_key] = {}
        
        data['rigid_body_poses'][yaml_key]['pos'] = [float(center[0]), float(center[1]), float(center[2])]
        
        if has_rotation:
            data['rigid_body_poses'][yaml_key]['quat'] = [float(mean_quat[0]), float(mean_quat[1]), 
                                                          float(mean_quat[2]), float(mean_quat[3])]
        else:
            # 如果没有旋转数据，使用单位四元数
            data['rigid_body_poses'][yaml_key]['quat'] = [0.0, 0.0, 0.0, 1.0]
        
        print(f"[INFO] 更新 {yaml_key}: pos={data['rigid_body_poses'][yaml_key]['pos']}, quat={data['rigid_body_poses'][yaml_key]['quat']}")
        updated_count += 1
    
    # 保持 arm_joint_angles_14dof 不变（如果存在）
    if 'arm_joint_angles_14dof' not in data:
        data['arm_joint_angles_14dof'] = [0.0] * 14
    
    # 保存更新后的 YAML
    os.makedirs(os.path.dirname(fitting_yaml_path), exist_ok=True)
    with open(fitting_yaml_path, 'w') as f:
        f.write("# Arm Calibration Data\n")
        f.write("# Position: mm, Rotation: quaternion (x,y,z,w), Joint angles: rad\n\n")
        yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
    
    print(f"[INFO] 已更新 {updated_count} 个刚体位姿到 {fitting_yaml_path}")


def parse_args():
    """解析命令行参数（可选，用于覆盖脚本中的配置）"""
    parser = argparse.ArgumentParser(
        description="点云筛选与中心拟合：使用 3-sigma 规则筛选异常点，计算几何中心"
    )
    parser.add_argument(
        "-s",
        "--sigma",
        type=float,
        default=None,
        help="覆盖配置中的 Sigma 倍数",
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="覆盖配置，不保存筛选后的 CSV 文件",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="覆盖配置，不显示可视化",
    )
    parser.add_argument(
        "--no-update-yaml",
        action="store_true",
        help="覆盖配置，不更新 fitting_data.yaml",
    )
    parser.add_argument(
        "csv_paths",
        nargs="*",
        help="覆盖配置中的 CSV 文件列表（可选）",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # 从脚本配置读取默认值
    sigma_multiplier = SIGMA_MULTIPLIER
    save_filtered = SAVE_FILTERED
    show_plot = SHOW_PLOT
    update_fitting_yaml = UPDATE_FITTING_YAML
    csv_paths = CSV_PATHS.copy()

    # 命令行参数可以覆盖配置
    if args.sigma is not None:
        sigma_multiplier = args.sigma
    if args.no_save:
        save_filtered = False
    if args.no_plot:
        show_plot = False
    if args.no_update_yaml:
        update_fitting_yaml = False
    if args.csv_paths:
        csv_paths = args.csv_paths

    if not csv_paths:
        print("[ERROR] CSV 文件列表为空，请在脚本开头的配置区域设置 CSV_PATHS，")
        print("       或在命令行中指定 CSV 文件路径。")
        sys.exit(1)

    # 将相对路径转换为绝对路径（相对于脚本所在目录）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_paths = [os.path.join(script_dir, p) if not os.path.isabs(p) else p for p in csv_paths]

    print("=" * 80)
    print("点云筛选与中心拟合")
    print("=" * 80)
    print(f"Sigma 倍数: {sigma_multiplier}")
    print(f"保存筛选后的 CSV: {save_filtered}")
    print(f"显示可视化: {show_plot}")
    print(f"CSV 文件数量: {len(csv_paths)}")
    for i, p in enumerate(csv_paths, 1):
        print(f"  [{i}] {p}")
    print("=" * 80)

    results = []
    for csv_path in csv_paths:
        result = process_csv(csv_path, sigma_multiplier, save_filtered, show_plot)
        if result is not None:
            results.append(result)

    # 打印总结
    if results:
        print("\n" + "=" * 80)
        print("处理总结")
        print("=" * 80)
        for r in results:
            print(f"\n文件: {os.path.basename(r['csv_path'])}")
            print(f"  原始点数: {r['n_original']}")
            print(f"  筛选后点数: {r['n_filtered']} ({r['n_filtered']/r['n_original']*100:.2f}%)")
            print(f"  位置筛除点数: {r['n_pos_removed']} ({r['n_pos_removed']/r['n_original']*100:.2f}%) [红色]")
            if r['has_rotation']:
                print(f"  旋转筛除点数: {r['n_rot_removed']} ({r['n_rot_removed']/r['n_original']*100:.2f}%) [蓝色]")
            print(f"  位置中心: ({r['center'][0]:.6f}, {r['center'][1]:.6f}, {r['center'][2]:.6f}) mm")
            if r['has_rotation']:
                q = r['mean_quat']
                print(f"  旋转中心: ({q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}) (x,y,z,w)")
            else:
                print(f"  旋转中心: (未提供旋转数据)")
        print("=" * 80)
        
        # 单独输出每个CSV的拟合中心点（位置和旋转）
        print("\n" + "=" * 80)
        print("拟合中心点汇总")
        print("=" * 80)
        for r in results:
            csv_name = os.path.splitext(os.path.basename(r['csv_path']))[0]
            center = r['center']
            mean_quat = r['mean_quat']
            has_rotation = r['has_rotation']
            print(f"{csv_name:30s} :")
            print(f"  位置 (mm)     : ({center[0]:12.6f}, {center[1]:12.6f}, {center[2]:12.6f})")
            if has_rotation:
                print(f"  旋转 (x,y,z,w) : ({mean_quat[0]:12.6f}, {mean_quat[1]:12.6f}, {mean_quat[2]:12.6f}, {mean_quat[3]:12.6f})")
            else:
                print(f"  旋转 (x,y,z,w) : (未提供旋转数据)")
        print("=" * 80)
        
        # 更新 fitting_data.yaml
        if update_fitting_yaml:
            print("\n" + "=" * 80)
            print("更新 fitting_data.yaml")
            print("=" * 80)
            update_fitting_yaml(results, script_dir)


if __name__ == "__main__":
    main()

