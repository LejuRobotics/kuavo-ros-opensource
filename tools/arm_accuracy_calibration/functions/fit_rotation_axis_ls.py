#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
使用最小二乘法拟合3D旋转圆并计算转轴偏差
"""

import numpy as np
import pandas as pd
import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize
import os


def configure_matplotlib_plot_fonts():
    """
    Prefer DejaVu Sans (bundled with matplotlib) for text/legend/axes.

    On Linux, the default sans-serif can resolve to a CJK or partial font that lacks
    basic Latin punctuation; each missing glyph emits RuntimeWarning from backend_agg.
    """
    plt.rcParams["font.sans-serif"] = [
        "DejaVu Sans",
        "Liberation Sans",
        "Bitstream Vera Sans",
        "sans-serif",
    ]
    plt.rcParams["axes.unicode_minus"] = False


def fit_circle_2d(projected_points):
    """
    在2D平面上拟合圆
    返回：圆心(2D), 半径, 拟合误差
    """
    n_points = len(projected_points)
    A = np.column_stack([
        -2 * projected_points[:, 0],
        -2 * projected_points[:, 1],
        np.ones(n_points)
    ])
    b = -(projected_points[:, 0]**2 + projected_points[:, 1]**2)
    
    params, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    a, b, c = params
    
    center_2d = np.array([a, b])
    radius_2d = np.sqrt(a**2 + b**2 - c)
    
    # 计算拟合误差（所有点到圆的距离平方和）
    if len(residuals) > 0:
        fit_error = residuals[0] / n_points  # 平均误差
    else:
        # 手动计算误差
        distances = np.sqrt((projected_points[:, 0] - center_2d[0])**2 + 
                           (projected_points[:, 1] - center_2d[1])**2)
        fit_error = np.mean((distances - radius_2d)**2)
    
    return center_2d, radius_2d, fit_error


def fit_circle_3d_ls(points, max_iterations=10, tolerance=1e-6):
    """
    使用迭代最小二乘法拟合3D圆
    参数:
        points: 3D点云
        max_iterations: 最大迭代次数
        tolerance: 收敛容差
    返回：center (圆心), radius (半径), axis (旋转轴方向，单位向量)
    """
    n_points = len(points)
    if n_points < 3:
        raise ValueError("至少需要3个点来拟合圆")
    
    # 步骤1：使用SVD估计初始旋转轴方向
    center_guess = np.mean(points, axis=0)
    centered = points - center_guess
    
    # SVD分解
    U, s, Vt = np.linalg.svd(centered, full_matrices=False)
    # 最小奇异值对应的右奇异向量是旋转轴方向
    axis = Vt[-1, :].copy()
    axis = axis / np.linalg.norm(axis)
    
    # 确保方向一致（主要分量为正）
    if axis[1] < 0:
        axis = -axis
    
    best_axis = axis.copy()
    best_error = float('inf')
    best_center = None
    best_radius = None
    prev_error = float('inf')
    
    # 迭代优化
    for iteration in range(max_iterations):
        # 步骤2：投影点到垂直于旋转轴的平面
        if abs(axis[0]) < 0.9:
            ref_vec = np.array([1, 0, 0])
        else:
            ref_vec = np.array([0, 0, 1])
        
        u = np.cross(axis, ref_vec)
        u = u / (np.linalg.norm(u) + 1e-10)
        v = np.cross(axis, u)
        v = v / (np.linalg.norm(v) + 1e-10)
        
        # 将3D点投影到平面上
        projected_points = []
        for p in points:
            vec = p - center_guess
            proj = vec - np.dot(vec, axis) * axis
            x_2d = np.dot(proj, u)
            y_2d = np.dot(proj, v)
            projected_points.append([x_2d, y_2d])
        projected_points = np.array(projected_points)
        
        # 步骤3：在2D平面上拟合圆
        center_2d, radius_2d, fit_error = fit_circle_2d(projected_points)
        
        # 将2D圆心转换回3D空间
        center_3d = center_guess + center_2d[0] * u + center_2d[1] * v
        
        # 保存最佳结果
        if fit_error < best_error:
            best_error = fit_error
            best_axis = axis.copy()
            best_center = center_3d.copy()
            best_radius = radius_2d
        
        # 检查收敛
        error_change = abs(fit_error - prev_error)
        
        if iteration > 0 and error_change < tolerance:
            break
        
        prev_error = fit_error
        
        # 如果误差已经很小，提前退出
        if fit_error < tolerance:
            break
        
        # 改进转轴估计：基于拟合误差优化转轴方向
        if iteration < max_iterations - 1 and fit_error > tolerance:
            # 使用非线性优化微调转轴方向
            def objective_axis(params):
                """优化目标：给定转轴方向，计算拟合误差"""
                nx, ny, nz = params
                test_axis = np.array([nx, ny, nz])
                test_axis = test_axis / (np.linalg.norm(test_axis) + 1e-10)
                
                # 构建垂直平面
                if abs(test_axis[0]) < 0.9:
                    ref_vec = np.array([1, 0, 0])
                else:
                    ref_vec = np.array([0, 0, 1])
                
                u = np.cross(test_axis, ref_vec)
                u = u / (np.linalg.norm(u) + 1e-10)
                v = np.cross(test_axis, u)
                v = v / (np.linalg.norm(v) + 1e-10)
                
                # 投影点
                projected = []
                for p in points:
                    vec = p - center_guess
                    proj = vec - np.dot(vec, test_axis) * test_axis
                    x_2d = np.dot(proj, u)
                    y_2d = np.dot(proj, v)
                    projected.append([x_2d, y_2d])
                projected = np.array(projected)
                
                # 拟合圆并返回误差
                try:
                    _, _, error = fit_circle_2d(projected)
                    return error
                except:
                    return 1e10
            
            # 使用当前转轴作为初始值，进行局部优化
            try:
                init_params = axis.copy()
                bounds = [(-1, 1), (-1, 1), (-1, 1)]
                result = minimize(
                    objective_axis,
                    init_params,
                    method='L-BFGS-B',
                    bounds=bounds,
                    options={'maxiter': 5, 'ftol': 1e-9}
                )
                
                if result.success:
                    new_axis = result.x / np.linalg.norm(result.x)
                    new_error = objective_axis(result.x)
                    # 如果新转轴使误差更小，使用它
                    if new_error < fit_error:
                        axis = new_axis
                        if axis[1] < 0:
                            axis = -axis
            except:
                pass  # 如果优化失败，保持当前转轴
    
    return best_center, best_radius, best_axis


def get_reference_axis(axis_name):
    """
    根据轴名称获取参考轴向量
    参数:
        axis_name: 'x', 'y', 'z' 之一
    返回:
        参考轴向量
    """
    axis_map = {
        'x': np.array([1, 0, 0]),
        'y': np.array([0, 1, 0]),
        'z': np.array([0, 0, 1])
    }
    return axis_map.get(axis_name.lower(), np.array([0, 1, 0]))


def calculate_axis_deviation(axis, reference_axis):
    """
    计算转轴与参考轴的偏差
    参数:
        axis: 拟合得到的转轴方向
        reference_axis: 参考轴方向（如X/Y/Z轴）
    返回：总偏差角度（度），Y方向偏差角度（度），Z方向偏差角度（度）
    """
    reference_axis = reference_axis / np.linalg.norm(reference_axis)
    
    # 计算总夹角
    cos_angle = np.clip(np.dot(axis, reference_axis), -1.0, 1.0)
    angle_rad = np.arccos(cos_angle)
    angle_deg = np.degrees(angle_rad)
    
    # 计算转轴在各个方向的分量偏差
    # 偏差1：转轴在参考轴垂直平面上的投影与某个标准方向的夹角
    # 偏差2：转轴在另一个垂直平面上的投影与某个标准方向的夹角
    
    if np.allclose(reference_axis, [1, 0, 0]):  # X轴参考
        # 偏差1：转轴在YZ平面的投影角度（转轴与X轴在YZ平面的投影夹角）
        # 转轴在YZ平面的投影向量
        axis_yz = np.array([0, axis[1], axis[2]])
        axis_yz_norm = np.linalg.norm(axis_yz)
        if axis_yz_norm > 1e-6:
            # 转轴与X轴的夹角在YZ平面的分量
            # 使用反正切计算：atan2(|axis_yz|, axis[0])
            deviation1 = np.degrees(np.arctan2(axis_yz_norm, abs(axis[0])))
        else:
            deviation1 = 0.0
        
        # 偏差2：转轴在XY平面的投影角度（转轴与X轴在XY平面的投影夹角）
        axis_xy = np.array([axis[0], axis[1], 0])
        axis_xy_norm = np.linalg.norm(axis_xy)
        if axis_xy_norm > 1e-6:
            # 转轴与X轴的夹角在XY平面的分量
            deviation2 = np.degrees(np.arctan2(abs(axis[1]), abs(axis[0])))
        else:
            deviation2 = 0.0
            
    elif np.allclose(reference_axis, [0, 1, 0]):  # Y轴参考
        # 偏差1：转轴在XY平面的投影角度（转轴与Y轴在XY平面的投影夹角）
        axis_xy = np.array([axis[0], axis[1], 0])
        axis_xy_norm = np.linalg.norm(axis_xy)
        if axis_xy_norm > 1e-6:
            # 转轴与Y轴的夹角在XY平面的分量
            deviation1 = np.degrees(np.arctan2(abs(axis[0]), abs(axis[1])))
        else:
            deviation1 = 0.0
        
        # 偏差2：转轴在YZ平面的投影角度（转轴与Y轴在YZ平面的投影夹角）
        axis_yz = np.array([0, axis[1], axis[2]])
        axis_yz_norm = np.linalg.norm(axis_yz)
        if axis_yz_norm > 1e-6:
            # 转轴与Y轴的夹角在YZ平面的分量
            deviation2 = np.degrees(np.arctan2(abs(axis[2]), abs(axis[1])))
        else:
            deviation2 = 0.0
            
    else:  # Z轴参考
        # 偏差1：转轴在XZ平面的投影角度（转轴与Z轴在XZ平面的投影夹角）
        axis_xz = np.array([axis[0], 0, axis[2]])
        axis_xz_norm = np.linalg.norm(axis_xz)
        if axis_xz_norm > 1e-6:
            # 转轴与Z轴的夹角在XZ平面的分量
            deviation1 = np.degrees(np.arctan2(abs(axis[0]), abs(axis[2])))
        else:
            deviation1 = 0.0
        
        # 偏差2：转轴在YZ平面的投影角度（转轴与Z轴在YZ平面的投影夹角）
        axis_yz = np.array([0, axis[1], axis[2]])
        axis_yz_norm = np.linalg.norm(axis_yz)
        if axis_yz_norm > 1e-6:
            # 转轴与Z轴的夹角在YZ平面的分量
            deviation2 = np.degrees(np.arctan2(abs(axis[1]), abs(axis[2])))
        else:
            deviation2 = 0.0
    
    return angle_deg, deviation1, deviation2


def compute_axis_misalignment_perp_to_ideal_deg(axis, reference_axis_name):
    """
    Misalignment of fitted axis vs ideal +X / +Y / +Z.

    - spatial_deg: angle between the two unit directions (what most people mean by
      \"how much they differ\").
    - tilt_u_deg, tilt_v_deg: signed angles atan2(a·u, a·r) and atan2(a·v, a·r) where
      (u, v, r) is right-handed orthonormal with r along the ideal axis. This describes
      the fitted direction in the plane *perpendicular* to the ideal axis (intrinsic to
      the joint axis), unlike world-frame atan2(ax, ay).
    """
    a = np.asarray(axis, dtype=float)
    a = a / (np.linalg.norm(a) + 1e-12)
    ref_name = (reference_axis_name or "y").lower()
    r = get_reference_axis(ref_name)
    r = r / (np.linalg.norm(r) + 1e-12)
    if abs(r[0]) < 0.9:
        aux = np.array([1.0, 0.0, 0.0])
    else:
        aux = np.array([0.0, 1.0, 0.0])
    u = np.cross(r, aux)
    nu = np.linalg.norm(u)
    if nu < 1e-10:
        aux = np.array([0.0, 0.0, 1.0])
        u = np.cross(r, aux)
        nu = np.linalg.norm(u)
    u = u / (nu + 1e-12)
    v = np.cross(r, u)
    v = v / (np.linalg.norm(v) + 1e-12)

    ar = float(np.dot(a, r))
    au = float(np.dot(a, u))
    av = float(np.dot(a, v))
    spatial_deg = float(np.degrees(np.arccos(np.clip(ar, -1.0, 1.0))))
    tilt_u_deg = float(np.degrees(np.arctan2(au, ar)))
    tilt_v_deg = float(np.degrees(np.arctan2(av, ar)))
    return {
        "spatial_deg": spatial_deg,
        "tilt_u_deg": tilt_u_deg,
        "tilt_v_deg": tilt_v_deg,
        "ideal_axis": ref_name.upper(),
        "u_world": [float(u[0]), float(u[1]), float(u[2])],
        "v_world": [float(v[0]), float(v[1]), float(v[2])],
    }


def _format_axis_misalignment_caption_en(info):
    """English-only caption for matplotlib (avoids missing CJK glyphs on headless systems)."""
    return (
        "Fitted axis vs ideal +%s\n"
        "Spatial angle (3D): %.4f deg\n"
        "Tilts in plane perp. to ideal:\n"
        "  atan2(a*u, a*r) = %.4f deg\n"
        "  atan2(a*v, a*r) = %.4f deg"
    ) % (
        info["ideal_axis"],
        info["spatial_deg"],
        info["tilt_u_deg"],
        info["tilt_v_deg"],
    )


def visualize_3d_circle(
    points,
    center,
    radius,
    axis,
    reference_axis,
    output_path=None,
    reference_axis_name="y",
):
    """
    可视化3D圆拟合结果
    参数:
        points: 原始点云
        center: 圆心
        radius: 半径
        axis: 拟合的转轴方向
        reference_axis: 参考轴方向
        output_path: 保存图像的路径（可选）
    """
    configure_matplotlib_plot_fonts()
    fig = plt.figure(figsize=(14, 11))
    ax = fig.add_subplot(111, projection='3d')
    _label_fs = 13
    _title_fs = 15
    _caption_fs = 15
    
    # 绘制原始点
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
               c='blue', label='Data Points', s=20, alpha=0.6)
    
    # 绘制圆心
    ax.scatter(center[0], center[1], center[2], 
              c='red', marker='*', s=300, label='Circle Center')
    
    # 绘制拟合的旋转轴
    axis_length = radius * 3
    ax.quiver(center[0], center[1], center[2],
              axis[0] * axis_length, axis[1] * axis_length, axis[2] * axis_length,
              color='green', linewidth=3, label='Fitted Rotation Axis', arrow_length_ratio=0.1)
    
    # 绘制参考轴
    ax.quiver(center[0], center[1], center[2],
              reference_axis[0] * axis_length, reference_axis[1] * axis_length, reference_axis[2] * axis_length,
              color='orange', linewidth=2, linestyle='--', label='Reference Axis', arrow_length_ratio=0.1)
    
    # 生成并绘制拟合圆
    theta = np.linspace(0, 2*np.pi, 100)
    if np.linalg.norm(np.cross(axis, np.array([1, 0, 0]))) > 1e-6:
        u = np.cross(axis, np.array([1, 0, 0]))
    else:
        u = np.cross(axis, np.array([0, 1, 0]))
    u = u - np.dot(u, axis) * axis
    u = u / (np.linalg.norm(u) + 1e-10)
    v = np.cross(axis, u)
    v = v / (np.linalg.norm(v) + 1e-10)
    
    circle_points = center + radius * (np.outer(np.cos(theta), u) + np.outer(np.sin(theta), v))
    ax.plot(circle_points[:, 0], circle_points[:, 1], circle_points[:, 2],
            c='purple', linewidth=2, label='Fitted Circle')

    mis_info = compute_axis_misalignment_perp_to_ideal_deg(axis, reference_axis_name)
    caption = _format_axis_misalignment_caption_en(mis_info)
    ax.text2D(
        0.02,
        0.98,
        caption,
        transform=ax.transAxes,
        fontsize=_caption_fs,
        verticalalignment="top",
        bbox=dict(
            boxstyle="round,pad=0.55",
            facecolor="wheat",
            alpha=0.88,
            linewidth=1.2,
        ),
    )
    
    ax.set_xlabel('X (m)', fontsize=_label_fs, labelpad=12)
    ax.set_ylabel('Y (m)', fontsize=_label_fs, labelpad=12)
    ax.set_zlabel('Z (m)', fontsize=_label_fs, labelpad=12)
    ax.tick_params(axis="both", labelsize=_label_fs - 1)
    ax.legend(loc="upper right", fontsize=_label_fs)
    ax.set_title('Rotation Axis Calibration (Least Squares)', fontsize=_title_fs, pad=14)
    
    # 设置相等的坐标轴比例
    max_range = np.array([points[:, 0].max() - points[:, 0].min(),
                          points[:, 1].max() - points[:, 1].min(),
                          points[:, 2].max() - points[:, 2].min()]).max() / 2.0
    mid_x = (points[:, 0].max() + points[:, 0].min()) * 0.5
    mid_y = (points[:, 1].max() + points[:, 1].min()) * 0.5
    mid_z = (points[:, 2].max() + points[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    plt.tight_layout()
    
    # 保存图像
    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"\n3D视图已保存到: {output_path}")
    
    plt.show()


def load_points_from_csv(csv_path):
    """
    从 CSV 加载 3D 点用于拟合圆。
    支持两种格式：
    1) matrix_data 格式：列 timestamp, m00..m33（4x4 齐次变换），位置取 m03,m13,m23（单位 mm，会转为 m）
    2) 旧 pose 格式：列 pos_x, pos_y, pos_z（单位一般为 m）
    """
    df = pd.read_csv(csv_path)
    required_matrix_cols = ["m03", "m13", "m23"]
    required_pose_cols = ["pos_x", "pos_y", "pos_z"]
    if all(c in df.columns for c in required_matrix_cols):
        # matrix_data 齐次变换矩阵格式：平移列 m03, m13, m23，单位 mm
        df = df.dropna(subset=required_matrix_cols).reset_index(drop=True)
        points = df[["m03", "m13", "m23"]].values.astype(float) / 1000.0  # mm -> m
    elif all(c in df.columns for c in required_pose_cols):
        df = df.dropna(subset=required_pose_cols).reset_index(drop=True)
        points = df[["pos_x", "pos_y", "pos_z"]].values.astype(float)
    else:
        raise ValueError(
            f"CSV 需包含齐次矩阵列 {required_matrix_cols} 或位姿列 {required_pose_cols}，当前列: {list(df.columns)}"
        )
    return points


def fit_rotation_axis_from_csv(csv_path, reference_axis_name='y', x_filter_threshold=900.0, 
                                max_iterations=5, tolerance=1e-8, verbose=True, unit='mm',
                                output_image_path=None):
    """
    从CSV文件加载点云并拟合旋转轴，返回拟合结果和统计信息。
    
    Args:
        csv_path: CSV文件路径（支持matrix_data格式或pose格式）
        reference_axis_name: 参考轴名称（'x', 'y', 'z'），默认: 'y'
        x_filter_threshold: X坐标过滤阈值，绝对值大于此值的点将被筛掉（默认: 900.0）
        max_iterations: 最大迭代次数（默认: 5）
        tolerance: 收敛容差（默认: 1e-8）
        verbose: 是否打印详细信息（默认: True）
        unit: 返回值的单位，'mm' 或 'm'（默认: 'mm'，与fit_matrix_from_csv保持一致）
        output_image_path: 3D可视化图片保存路径（可选，默认: None，不生成图片）
        
    Returns:
        dict: 包含以下键的字典
            - 'center': (3,) 圆心坐标（单位由unit参数决定）
            - 'radius': float 圆半径（单位由unit参数决定）
            - 'axis': (3,) 转轴方向向量（单位向量，无单位）
            - 'reference_axis': (3,) 参考轴方向向量
            - 'points_filtered': 筛选后的点云（单位由unit参数决定）
            - 'n_total': 总点数
            - 'n_filtered': 筛选后保留的点数
            - 'n_removed': 被剔除的点数
            - 'deviation': dict，含 total_angle_deg、deviation1/2_deg（旧平面分解）、
              perp_tilt_u/v_deg（相对理想轴垂直平面内 atan2(a*u,a*r)、atan2(a*v,a*r)，度）
            - 'max_distance_info': dict包含距离圆心最远的点信息
            - 'unit': 返回值的单位（'mm' 或 'm'）
    """
    if verbose:
        print("读取 CSV 数据...")
    
    # 读取 CSV 数据（自动识别 matrix_data 矩阵格式或旧 pose 格式）
    points = load_points_from_csv(csv_path)
    n_total = len(points)
    
    if n_total < 3:
        raise ValueError(f"数据点不足（仅{n_total}个），至少需要3个点")
    
    if verbose:
        print(f"成功读取 {n_total} 个数据点")
    
    # 筛选X坐标大于阈值的点（明显异常值）
    n_before_filter = len(points)
    x_filter_mask = np.abs(points[:, 0]) <= x_filter_threshold
    points_filtered = points[x_filter_mask]
    n_filtered = len(points_filtered)
    n_removed = n_before_filter - n_filtered
    
    if verbose and n_removed > 0:
        print(f"X坐标过滤: 剔除 {n_removed} 个X坐标绝对值>{x_filter_threshold}的点，保留 {n_filtered} 个点")
    
    if n_filtered < 3:
        raise ValueError(f"X坐标过滤后数据点不足（仅{n_filtered}个），至少需要3个点")
    
    # 获取参考轴
    reference_axis = get_reference_axis(reference_axis_name)
    if verbose:
        print(f"\n基准轴: {reference_axis_name.upper()}轴 {reference_axis}")
    
    # 拟合圆和转轴（迭代优化）
    if verbose:
        print("\n拟合圆和旋转轴（迭代最小二乘法）...")
    center, radius, axis = fit_circle_3d_ls(points_filtered, max_iterations=max_iterations, tolerance=tolerance)
    
    # 单位转换：如果要求返回mm单位，将m转换为mm
    if unit == 'mm':
        center = center * 1000.0  # m -> mm
        radius = radius * 1000.0  # m -> mm
        points_filtered = points_filtered * 1000.0  # m -> mm
    
    if verbose:
        unit_str = unit if unit == 'mm' else 'm'
        print(f"\n拟合结果:")
        print(f"  圆心坐标: ({center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f}) {unit_str}")
        print(f"  圆半径: {radius:.6f} {unit_str}")
        print(f"  转轴方向: ({axis[0]:.6f}, {axis[1]:.6f}, {axis[2]:.6f})")
    
    # 计算每个点到圆心的距离，找出距离最大的点
    distances_to_center = np.linalg.norm(points_filtered - center, axis=1)
    max_dist_idx = np.argmax(distances_to_center)
    max_dist_point = points_filtered[max_dist_idx]
    max_distance = distances_to_center[max_dist_idx]
    
    max_distance_info = {
        'point': max_dist_point.tolist(),
        'distance': float(max_distance),
        'deviation_from_radius': float(max_distance - radius) if radius > 1e-10 else 0.0,
        'deviation_percent': float(100.0 * (max_distance - radius) / radius) if radius > 1e-10 else 0.0
    }
    
    if verbose:
        unit_str = unit if unit == 'mm' else 'm'
        print(f"\n保留下来的点中，距离圆心最大的点:")
        print(f"  位置: ({max_dist_point[0]:.6f}, {max_dist_point[1]:.6f}, {max_dist_point[2]:.6f}) {unit_str}")
        print(f"  距离: {max_distance:.6f} {unit_str}")
        if radius > 1e-10:
            print(f"  与半径偏差: {max_distance - radius:.6f} {unit_str} ({100.0 * (max_distance - radius) / radius:.2f}%)")
    
    # 计算转轴偏差
    if verbose:
        print(f"\n计算转轴与{reference_axis_name.upper()}轴的偏差...")
    total_deviation, deviation1, deviation2 = calculate_axis_deviation(axis, reference_axis)
    perp_ideal = compute_axis_misalignment_perp_to_ideal_deg(axis, reference_axis_name)

    deviation = {
        'total_angle_deg': float(total_deviation),
        'deviation1_deg': float(deviation1),
        'deviation2_deg': float(deviation2),
        'perp_tilt_u_deg': perp_ideal['tilt_u_deg'],
        'perp_tilt_v_deg': perp_ideal['tilt_v_deg'],
        'perp_spatial_deg': perp_ideal['spatial_deg'],
        'perp_u_world': perp_ideal['u_world'],
        'perp_v_world': perp_ideal['v_world'],
        'axis': axis.tolist(),
        'reference_axis': reference_axis.tolist()
    }
    
    if verbose:
        print(f"\n转轴偏差分析:")
        print(f"  总偏差角度: {total_deviation:.4f} 度")
        if reference_axis_name.lower() == 'x':
            print(f"  YZ平面偏差: {deviation1:.4f} 度 (转轴与X轴在YZ平面的投影夹角)")
            print(f"  XY平面偏差: {deviation2:.4f} 度 (转轴与X轴在XY平面的投影夹角)")
        elif reference_axis_name.lower() == 'y':
            print(f"  XY平面偏差: {deviation1:.4f} 度 (转轴与Y轴在XY平面的投影夹角)")
            print(f"  YZ平面偏差: {deviation2:.4f} 度 (转轴与Y轴在YZ平面的投影夹角)")
        else:  # z
            print(f"  XZ平面偏差: {deviation1:.4f} 度 (转轴与Z轴在XZ平面的投影夹角)")
            print(f"  YZ平面偏差: {deviation2:.4f} 度 (转轴与Z轴在YZ平面的投影夹角)")
        print(f"\n转轴方向向量: [{axis[0]:.6f}, {axis[1]:.6f}, {axis[2]:.6f}]")
        print(f"参考{reference_axis_name.upper()}轴方向: [{reference_axis[0]:.6f}, {reference_axis[1]:.6f}, {reference_axis[2]:.6f}]")
        print(
            "  Perp.-to-ideal tilts (deg): atan2(a*u,a*r)=%.4f, atan2(a*v,a*r)=%.4f "
            "(spatial %.4f)"
            % (
                perp_ideal["tilt_u_deg"],
                perp_ideal["tilt_v_deg"],
                perp_ideal["spatial_deg"],
            )
        )
    
    # 生成3D可视化图片（如果指定了输出路径）
    if output_image_path is not None:
        try:
            if verbose:
                print(f"\n生成3D可视化图片: {output_image_path}")
            visualize_3d_circle(
                points_filtered,
                center,
                radius,
                axis,
                reference_axis,
                output_path=output_image_path,
                reference_axis_name=reference_axis_name,
            )
        except Exception as e:
            if verbose:
                print(f"警告: 图片生成失败: {e}")
    
    return {
        'center': center,  # (3,) numpy array，单位由unit参数决定
        'radius': float(radius),  # float，单位由unit参数决定
        'axis': axis,  # (3,) numpy array，单位向量（无单位）
        'reference_axis': reference_axis,  # (3,) numpy array
        'points_filtered': points_filtered,  # (N, 3) numpy array，单位由unit参数决定
        'n_total': n_total,
        'n_filtered': n_filtered,
        'n_removed': n_removed,
        'deviation': deviation,
        'max_distance_info': max_distance_info,
        'unit': unit  # 返回值的单位
    }


def main():
    parser = argparse.ArgumentParser(description="使用最小二乘法拟合旋转轴")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calibration_dir = os.path.dirname(script_dir)
    default_csv = os.path.join(
        calibration_dir,
        "matrix_data",
        "T_joint4_in_joint3.csv",
    )
    parser.add_argument(
        "--csv", "-c", type=str, default=default_csv,
        help=f"CSV 路径：matrix_data 下齐次变换矩阵文件（含 m03,m13,m23）或旧格式（pos_x,pos_y,pos_z），默认: {default_csv}",
    )
    parser.add_argument("--axis", "-a", type=str, default="y", choices=['x', 'y', 'z'],
                       help="基准轴（x/y/z），默认: y")
    parser.add_argument("--output", "-o", type=str, default=None,
                       help="3D视图保存路径（可选）")
    args = parser.parse_args()

    # 调用主函数
    result = fit_rotation_axis_from_csv(
        args.csv, 
        reference_axis_name=args.axis,
        verbose=True
    )
    
    # 生成3D视图
    print("\n生成3D视图...")
    if args.output is None:
        # 默认保存路径
        script_dir = os.path.dirname(os.path.abspath(__file__))
        calibration_dir = os.path.dirname(script_dir)
        image_dir = os.path.join(calibration_dir, "image")
        os.makedirs(image_dir, exist_ok=True)
        output_path = os.path.join(image_dir, "rotation_axis_calibration_ls.png")
    else:
        output_path = args.output
    
    visualize_3d_circle(
        result['points_filtered'], 
        result['center'], 
        result['radius'], 
        result['axis'], 
        result['reference_axis'], 
        output_path,
        reference_axis_name=args.axis,
    )


if __name__ == "__main__":
    main()
