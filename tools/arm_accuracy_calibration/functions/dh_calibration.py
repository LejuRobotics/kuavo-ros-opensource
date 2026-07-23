"""
DH参数迭代标定算法

实现基于关节实测轴线的DH参数精校准迭代算法。
通过迭代优化，规避误差链式放大，获得稳定精准的DH参数。
"""

import numpy as np
from typing import Tuple, Dict, Optional


def _wrap_to_pi(angle: float) -> float:
    """将角度归一化到 [-pi, pi)。"""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def _project_to_rotation_matrix(R: np.ndarray) -> np.ndarray:
    """
    将 3x3 矩阵投影到最近的合法旋转矩阵（正交且 det(R)=1）。
    SVD: R = U S V^T，取 R_proj = U V^T，并保证 det(R_proj)=1。
    """
    R = np.asarray(R, dtype=float)
    U, _, Vt = np.linalg.svd(R)
    R_proj = U @ Vt
    if np.linalg.det(R_proj) < 0:
        U = U.copy()
        U[:, -1] *= -1.0
        R_proj = U @ Vt
    return R_proj


def initialize_dh_from_transform_candidates(T_meas: np.ndarray):
    """
    从相邻连杆实测变换矩阵反解DH参数：解析解有两支，本函数返回两支候选。

    先用 _project_to_rotation_matrix 保证 R 正交且 det=1，再在投影后的 T 上反解，
    避免非正交 R 导致解析式不稳定。正解中 (theta, alpha, a, d) 与 (theta+pi, -alpha, -a, d)
    给出相同平移但旋转不同，故反解有两支代数解。

    参数:
        T_meas: 4x4 变换矩阵，(k-1)^k T^meas

    返回:
        list of (dh, rot_err, trans_err): 两支 DH 及各自相对投影后 T 的重建误差
        dh 为 [a_{k-1}, alpha_{k-1}, d_k, theta_k]，角度已归一化到 (-pi, pi]
    """
    R = np.asarray(T_meas[:3, :3], dtype=float)
    p = np.asarray(T_meas[:3, 3], dtype=float)
    R = _project_to_rotation_matrix(R)
    r11, r21 = R[0, 0], R[1, 0]
    r32, r33 = R[2, 1], R[2, 2]
    px, py, pz = float(p[0]), float(p[1]), float(p[2])

    # 第一支：theta = arctan2(r21, r11), alpha = arctan2(r32, r33)
    theta1 = np.arctan2(r21, r11)
    alpha1 = np.arctan2(r32, r33)
    a1 = px * np.cos(theta1) + py * np.sin(theta1)
    d1 = pz
    dh1 = np.array([a1, _wrap_to_pi(alpha1), d1, _wrap_to_pi(theta1)])

    # 第二支：(theta2, alpha2, a2, d2) = (theta1+pi, -alpha1, -a1, d1)
    theta2 = _wrap_to_pi(theta1 + np.pi)
    alpha2 = _wrap_to_pi(-alpha1)
    a2 = -a1
    d2 = d1
    dh2 = np.array([a2, alpha2, d2, theta2])

    T_proj = np.eye(4)
    T_proj[:3, :3] = R
    T_proj[:3, 3] = p
    out = []
    for dh in (dh1, dh2):
        rot_err, trans_err = _compute_transform_consistency(dh, T_proj)
        out.append((dh, float(rot_err), float(trans_err)))
    return out


def initialize_dh_from_transform(T_meas: np.ndarray) -> np.ndarray:
    """
    从相邻连杆实测变换矩阵反解DH参数初始值（单支）。

    解析解有两支（见 initialize_dh_from_transform_candidates）。反解前先将 R 投影到
    最近的正交矩阵（保证正交且 det=1），再在两支中返回与投影后 T 重建误差更小的一支。

    参数:
        T_meas: 4x4变换矩阵，表示相邻连杆的实测变换 (k-1)^k T^meas

    返回:
        DH参数数组 [a_{k-1}, alpha_{k-1}, d_k, theta_k]
    """
    candidates = initialize_dh_from_transform_candidates(T_meas)
    # 选旋转误差 + 平移误差 更小的一支（平移单位与旋转弧度可同量级时简单相加）
    best = min(candidates, key=lambda x: x[1] + x[2])
    return best[0]


def extract_axis_from_transform_in_z(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    从变换矩阵提取理论关节轴线

    参数:
        T: 4x4变换矩阵

    返回:
        (P, n): 轴线上一点P和轴线方向向量n
    """
    P = T[:3, 3]  # 平移部分作为轴线上一点
    R = T[:3, :3]
    n = R[:, 2]  # z轴方向作为轴线方向
    return P, n


def compute_axis_deviation(
    P_pred: np.ndarray,
    n_pred: np.ndarray,
    P_meas: np.ndarray,
    n_meas: np.ndarray,
) -> Tuple[float, float]:
    """
    计算关节轴线的方向偏差和位置偏差

    参数:
        P_pred: 理论轴线上一点
        n_pred: 理论轴线方向向量
        P_meas: 实测轴线上一点
        n_meas: 实测轴线方向向量

    返回:
        (delta_theta, delta_d): 方向偏差（弧度）和位置偏差（长度单位）
    """
    # 归一化方向向量
    n_pred_norm = n_pred / (np.linalg.norm(n_pred) + 1e-10)
    n_meas_norm = n_meas / (np.linalg.norm(n_meas) + 1e-10)

    # 方向偏差：两方向向量的夹角
    dot_product = np.clip(np.dot(n_pred_norm, n_meas_norm), -1.0, 1.0)
    delta_theta = np.arccos(dot_product)

    # 位置偏差：计算两轴线之间的最短距离
    # 使用叉积计算垂直于两轴线的方向
    cross_product = np.cross(n_pred_norm, n_meas_norm)
    cross_norm = np.linalg.norm(cross_product)

    if cross_norm < 1e-8:
        # 两轴线平行或反平行，使用点到直线的距离公式
        # 计算从P_meas到理论轴线的距离
        vec_to_pred = P_meas - P_pred
        # 投影到垂直于n_pred的平面
        proj = vec_to_pred - np.dot(vec_to_pred, n_pred_norm) * n_pred_norm
        delta_d = np.linalg.norm(proj)
    else:
        # 使用公式：|(P_meas - P_pred) · (n_pred × n_meas)| / ||n_pred × n_meas||
        vec_diff = P_meas - P_pred
        delta_d = np.abs(np.dot(vec_diff, cross_product)) / cross_norm

    return delta_theta, delta_d


def update_dh_parameters(
    dh_current: np.ndarray, delta_theta: float, delta_d: float
) -> np.ndarray:
    """
    根据轴线偏差更新DH参数

    参数:
        dh_current: 当前DH参数 [a_{k-1}, alpha_{k-1}, d_k, theta_k]
        delta_theta: 方向偏差（弧度）
        delta_d: 位置偏差（长度单位）

    返回:
        更新后的DH参数 [a_{k-1}, alpha_{k-1}, d_k, theta_k]
    """
    a_k1, alpha_k1, d_k, theta_k = dh_current

    # 核心修正公式
    a_k1_new = a_k1 - delta_d * np.cos(delta_theta)
    alpha_k1_new = alpha_k1 - delta_theta
    d_k_new = d_k - delta_d * np.sin(delta_theta)
    theta_k_new = theta_k  # theta保持不变

    return np.array([a_k1_new, alpha_k1_new, d_k_new, theta_k_new])


def build_transform_from_dh(dh: np.ndarray) -> np.ndarray:
    """
    根据DH参数构建变换矩阵

    参数:
        dh: DH参数 [a_{k-1}, alpha_{k-1}, d_k, theta_k]

    返回:
        4x4变换矩阵
    """
    a_k1, alpha_k1, d_k, theta_k = dh

    # 构建DH变换矩阵
    cos_theta = np.cos(theta_k)
    sin_theta = np.sin(theta_k)
    cos_alpha = np.cos(alpha_k1)
    sin_alpha = np.sin(alpha_k1)

    T = np.eye(4)
    T[0, 0] = cos_theta
    T[0, 1] = -sin_theta * cos_alpha
    T[0, 2] = sin_theta * sin_alpha
    T[0, 3] = a_k1 * cos_theta

    T[1, 0] = sin_theta
    T[1, 1] = cos_theta * cos_alpha
    T[1, 2] = -cos_theta * sin_alpha
    T[1, 3] = a_k1 * sin_theta

    T[2, 0] = 0
    T[2, 1] = sin_alpha
    T[2, 2] = cos_alpha
    T[2, 3] = d_k

    return T


def compute_theoretical_axis_from_dh(dh: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    根据当前DH参数计算理论关节轴线

    参数:
        dh: DH参数 [a_{k-1}, alpha_{k-1}, d_k, theta_k]

    返回:
        (P_pred, n_pred): 理论轴线上一点和方向向量
    """
    # 根据DH参数构建变换矩阵
    T_dh = build_transform_from_dh(dh)

    # 按文档定义，理论轴线应完整由当前DH参数生成（位置+方向同时更新）
    P_pred, n_pred = extract_axis_from_transform_in_z(T_dh)

    return P_pred, n_pred


def _compute_transform_consistency(dh: np.ndarray, T_meas: np.ndarray) -> Tuple[float, float]:
    """
    计算DH变换与单帧实测变换的一致性

    返回:
        (rotation_error_rad, translation_error)
    """
    T_dh = build_transform_from_dh(dh)
    R_dh = T_dh[:3, :3]
    p_dh = T_dh[:3, 3]
    R_meas = T_meas[:3, :3]
    p_meas = T_meas[:3, 3]
    R_rel = R_dh @ R_meas.T
    cos_angle = np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0)
    rot_err = float(np.arccos(cos_angle))
    trans_err = float(np.linalg.norm(p_dh - p_meas))
    return rot_err, trans_err


def calibrate_dh_parameters(
    T_meas: np.ndarray,
    P_meas: np.ndarray,
    n_meas: np.ndarray,
    epsilon: Optional[float] = 0.001,
    epsilon_theta: Optional[float] = None,
    epsilon_d: Optional[float] = None,
    max_iterations: int = 100,
    verbose: bool = False,
) -> Dict:
    """
    DH参数迭代标定主函数（单帧 T + 轴线）

    参数:
        T_meas: 4x4 变换矩阵，相邻连杆实测变换 (k-1)^k T^meas
        P_meas: 实测轴线上一点（与 T_meas 同一坐标系）
        n_meas: 实测轴线方向向量
        epsilon: 兼容旧接口的统一收敛阈值
        epsilon_theta: 方向偏差收敛阈值（弧度）
        epsilon_d: 位置偏差收敛阈值（长度单位）
        max_iterations: 最大迭代次数
        verbose: 是否打印迭代过程信息

    返回:
        包含 dh_final, converged, iterations, final_delta_theta, final_delta_d,
        final_transform_rotation_error, final_transform_translation_error, [history]
    """
    T_meas = np.asarray(T_meas, dtype=float)
    if T_meas.shape != (4, 4):
        raise ValueError("T_meas must have shape (4, 4)")
    P_meas = np.asarray(P_meas, dtype=float)
    n_meas = np.asarray(n_meas, dtype=float)
    if P_meas.shape != (3,) or n_meas.shape != (3,):
        raise ValueError("P_meas and n_meas must have shape (3,)")

    if epsilon_theta is None and epsilon_d is None:
        epsilon_theta = float(epsilon if epsilon is not None else 0.001)
        epsilon_d = float(epsilon if epsilon is not None else 0.001)
    else:
        if epsilon_theta is None:
            epsilon_theta = float(epsilon if epsilon is not None else 0.001)
        if epsilon_d is None:
            epsilon_d = float(epsilon if epsilon is not None else 0.001)

    # 步骤1：从单帧 T 初始化 DH
    dh = initialize_dh_from_transform(T_meas)
    P_pred, n_pred = compute_theoretical_axis_from_dh(dh)
    trans_scale = max(1.0, float(np.linalg.norm(T_meas[:3, 3])))
    param_bound = 100.0 * trans_scale

    history = []
    converged = False
    prev_metric = None
    growth_streak = 0
    step_size = 0.2

    # 迭代循环
    for iteration in range(max_iterations):
        # 步骤2：计算轴线偏差
        delta_theta, delta_d = compute_axis_deviation(P_pred, n_pred, P_meas, n_meas)

        # 记录历史
        if verbose:
            history.append(
                {
                    "iteration": iteration,
                    "dh": dh.copy(),
                    "delta_theta": delta_theta,
                    "delta_d": delta_d,
                }
            )

        # 步骤3：收敛判断
        if delta_theta <= epsilon_theta and delta_d <= epsilon_d:
            converged = True
            if verbose:
                print(
                    f"收敛于第 {iteration} 次迭代: "
                    f"delta_theta={np.degrees(delta_theta):.6f}°, "
                    f"delta_d={delta_d:.6f}"
                )
            break

        # 计算带符号的位置修正，避免参数单向漂移
        n_pred_norm = n_pred / (np.linalg.norm(n_pred) + 1e-12)
        n_meas_norm = n_meas / (np.linalg.norm(n_meas) + 1e-12)
        cross_product = np.cross(n_pred_norm, n_meas_norm)
        cross_norm = np.linalg.norm(cross_product)
        if cross_norm >= 1e-8:
            signed_delta_d = np.dot(P_meas - P_pred, cross_product) / cross_norm
        else:
            signed_delta_d = delta_d

        # 限幅 + 阻尼，避免一步过冲导致发散
        # 不再使用过小的固定角截断，避免累计产生系统性90°漂移
        delta_theta_step = min(delta_theta * step_size, np.deg2rad(15.0))
        delta_d_step = np.clip(signed_delta_d, -0.05 * trans_scale, 0.05 * trans_scale)
        delta_d_step *= step_size

        # 角度符号未知时，双向试探，选择使误差更小的更新方向
        def _project_dh(dh_vec: np.ndarray) -> np.ndarray:
            dh_vec = dh_vec.copy()
            dh_vec[1] = _wrap_to_pi(dh_vec[1])  # alpha
            dh_vec[3] = _wrap_to_pi(dh_vec[3])  # theta
            dh_vec[0] = float(np.clip(dh_vec[0], -param_bound, param_bound))  # a
            dh_vec[2] = float(np.clip(dh_vec[2], -param_bound, param_bound))  # d
            return dh_vec

        dh_plus = _project_dh(update_dh_parameters(dh, +delta_theta_step, delta_d_step))
        P_plus, n_plus = compute_theoretical_axis_from_dh(dh_plus)
        dt_plus, dd_plus = compute_axis_deviation(P_plus, n_plus, P_meas, n_meas)
        metric_plus = float(dt_plus + dd_plus)

        dh_minus = _project_dh(
            update_dh_parameters(dh, -delta_theta_step, delta_d_step)
        )
        P_minus, n_minus = compute_theoretical_axis_from_dh(dh_minus)
        dt_minus, dd_minus = compute_axis_deviation(P_minus, n_minus, P_meas, n_meas)
        metric_minus = float(dt_minus + dd_minus)

        current_metric = float(delta_theta + delta_d)
        if metric_plus <= metric_minus:
            candidate_dh, candidate_P, candidate_n, candidate_metric = (
                dh_plus,
                P_plus,
                n_plus,
                metric_plus,
            )
        else:
            candidate_dh, candidate_P, candidate_n, candidate_metric = (
                dh_minus,
                P_minus,
                n_minus,
                metric_minus,
            )

        # 若本轮无改进，减小步长重试下一轮
        if candidate_metric > current_metric:
            step_size = max(0.05, step_size * 0.7)
        else:
            dh = candidate_dh
            P_pred, n_pred = candidate_P, candidate_n

        # 发散保护：误差连续恶化时提前终止
        metric = float(min(candidate_metric, param_bound))
        if prev_metric is not None and metric > prev_metric * 1.2:
            growth_streak += 1
        else:
            growth_streak = 0
        prev_metric = metric
        if growth_streak >= 8:
            if verbose:
                print("警告: 检测到连续误差恶化，提前停止迭代以避免发散。")
            break

    if not converged:
        if verbose:
            print(
                f"警告: 达到最大迭代次数 {max_iterations}，未收敛。"
                f"最终偏差: delta_theta={np.degrees(delta_theta):.6f}°, "
                f"delta_d={delta_d:.6f}"
            )

    final_rot_err, final_trans_err = _compute_transform_consistency(dh, T_meas)

    result = {
        "dh_final": dh,
        "converged": converged,
        "iterations": iteration + 1,
        "final_delta_theta": delta_theta,
        "final_delta_d": delta_d,
        "final_transform_rotation_error": final_rot_err,
        "final_transform_translation_error": final_trans_err,
    }

    if verbose:
        result["history"] = history

    return result
