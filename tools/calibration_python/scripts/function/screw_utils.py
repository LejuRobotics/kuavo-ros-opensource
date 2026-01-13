#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
旋量标定工具函数模块

包含所有用于旋量标定的数学计算函数：
- SE(3)指数映射
- 反对称矩阵
- Adjoint变换
- 雅可比矩阵计算
- 正运动学计算

这些函数对应MATLAB代码中的function目录下的函数
"""

import numpy as np


def operator_w(w):
    """
    计算向量w的反对称矩阵（用于叉乘）
    
    对应MATLAB: operator_w.m
    
    Args:
        w: 3维向量 [wx, wy, wz]
    
    Returns:
        3×3反对称矩阵
        [[0,  -wz,  wy],
         [wz,  0,  -wx],
         [-wy, wx,  0]]
    """
    return np.array([
        [0,    -w[2],  w[1]],
        [w[2],  0,    -w[0]],
        [-w[1], w[0],  0]
    ])


def exp_se3_r(w, v, theta):
    """
    SE(3)指数映射：旋量到齐次变换矩阵
    
    对应MATLAB: exp_se3_r.m
    
    旋量形式：ξ = [v; w]，其中v是线速度分量，w是角速度分量
    
    Args:
        w: 角速度分量（3维向量）
        v: 线速度分量（3维向量）
        theta: 关节角（标量，弧度）
    
    Returns:
        4×4齐次变换矩阵
    """
    w_hat = operator_w(w)
    I = np.eye(3)
    
    # 旋转部分（Rodrigues公式）
    R = I + w_hat * np.sin(theta) + w_hat @ w_hat * (1 - np.cos(theta))
    
    # 平移部分
    t = (I - R) @ np.cross(w, v) + theta * w * (w @ v)
    
    # 组合为齐次变换矩阵
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = t
    
    return T


def ad_g(g):
    """
    计算齐次变换矩阵g的adjoint变换（6×6矩阵）
    
    对应MATLAB: ad_g.m
    
    Args:
        g: 4×4齐次变换矩阵
    
    Returns:
        6×6 adjoint矩阵
    """
    R = g[0:3, 0:3]
    b = g[0:3, 3]
    b_hat = operator_w(b)
    
    Z = np.zeros((6, 6))
    Z[0:3, 0:3] = R
    Z[3:6, 0:3] = b_hat @ R
    Z[3:6, 3:6] = R
    
    return Z


def ad_kesi(x):
    """
    计算旋量x的adjoint矩阵（6×6）
    
    对应MATLAB: ad_kesi.m
    
    Args:
        x: 6维旋量 [v; w]，v是线速度分量，w是角速度分量
    
    Returns:
        6×6 adjoint矩阵
    """
    w = x[0:3]
    v = x[3:6]
    W = operator_w(w)
    V = operator_w(v)
    
    A = np.zeros((6, 6))
    A[0:3, 0:3] = W
    A[3:6, 0:3] = V
    A[3:6, 3:6] = W
    
    return A


def yita_exp_diff(yita):
    """
    计算旋量增量yita对应的齐次变换矩阵
    
    对应MATLAB: yita_exp_diff.m
    
    Args:
        yita: 6维旋量增量 [w; v]
    
    Returns:
        4×4齐次变换矩阵
    """
    w = yita[0:3]
    v = yita[3:6]
    q = np.linalg.norm(w)
    
    if q < 1e-15:  # 防止q=0
        return np.eye(4)
    
    w_hat = operator_w(w)
    I = np.eye(3)
    
    # 旋转部分
    T1 = I + (np.sin(q) / q) * w_hat + w_hat @ w_hat * (1 - np.cos(q)) / (q**2)
    
    # 平移部分
    T2 = (I + (1 - np.cos(q)) / (q**2) * w_hat + 
          (q - np.sin(q)) / (q**3) * w_hat @ w_hat) @ v
    
    T = np.eye(4)
    T[0:3, 0:3] = T1
    T[0:3, 3] = T2
    
    return T


def coe_dif_yita(yita):
    """
    计算旋量增量yita的微分系数矩阵（6×6）
    
    对应MATLAB: coe_dif_yita.m
    
    用于构建雅可比矩阵
    
    Args:
        yita: 6维旋量增量 [w; v]
    
    Returns:
        6×6系数矩阵
    """
    w = yita[0:3]
    q = np.linalg.norm(w)
    
    A0 = np.eye(6)
    if q <= 1e-15:
        return A0
    
    Z = ad_kesi(yita)
    si = np.sin(q)
    co = np.cos(q)
    
    A1 = 1 / (2 * q**2) * (4 - q * si - 4 * co) * Z
    A2 = 1 / (2 * q**3) * (4 * q - 5 * si + q * co) * (Z @ Z)
    A3 = 1 / (2 * q**4) * (2 - q * si - 2 * co) * (Z @ Z @ Z)
    A4 = 1 / (2 * q**5) * (2 * q - 3 * si + q * co) * (Z @ Z @ Z @ Z)
    
    return A0 + A1 + A2 + A3 + A4


def jacobi_g(yita, kesi, theta):
    """
    计算旋量增量对变换矩阵的雅可比（6×42，7自由度）
    
    对应MATLAB: jacobi_g.m
    
    Args:
        yita: 旋量增量（6×7），每个关节6维
        kesi: 旋量参数（6×7），每个关节6维
        theta: 关节角（7×1），弧度
    
    Returns:
        6×42雅可比矩阵（6维旋量 × 7个关节 × 6维增量）
    """
    num_dof, num_joint = kesi.shape  # num_dof=6, num_joint=7
    J_g = np.zeros((num_dof, num_joint * num_dof))
    I = np.eye(num_dof)
    A = I.copy()
    
    # 第一个关节
    B = (ad_g(yita_exp_diff(yita[:, 0])) @ 
         ad_g(exp_se3_r(kesi[0:3, 0], kesi[3:6, 0], theta[0])) @ 
         ad_g(yita_exp_diff(-yita[:, 0])))
    J_g[:, 0:num_dof] = A @ (I - B) @ coe_dif_yita(yita[:, 0])
    
    # 后续关节
    for i in range(1, num_joint):
        A = A @ B
        B = (ad_g(yita_exp_diff(yita[:, i])) @ 
             ad_g(exp_se3_r(kesi[0:3, i], kesi[3:6, i], theta[i])) @ 
             ad_g(yita_exp_diff(-yita[:, i])))
        start_col = i * num_dof
        end_col = (i + 1) * num_dof
        J_g[:, start_col:end_col] = A @ (I - B) @ coe_dif_yita(yita[:, i])
    
    return J_g


def jacobi_pos(yita, kesi, theta, pos, g):
    """
    计算位置对旋量增量和工具坐标系平移的雅可比（3×45）
    
    对应MATLAB: jacobi_pos.m
    
    Args:
        yita: 旋量增量（6×7）
        kesi: 旋量参数（6×7）
        theta: 关节角（7×1），弧度
        pos: 末端位置（3×1），米
        g: 最后一个关节的齐次变换矩阵（4×4）
    
    Returns:
        3×45雅可比矩阵
        - 前42列：位置对7个关节旋量增量的雅可比（7×6=42）
        - 后3列：位置对工具坐标系平移的雅可比（3）
    """
    J_g = jacobi_g(yita, kesi, theta)  # 6×42
    
    # 构建位置雅可比：[-operator_w(pos), I] @ J_g 得到 3×42
    pos_w_hat = -operator_w(pos)
    I3 = np.eye(3)
    J_pos_part1 = np.hstack([pos_w_hat, I3]) @ J_g  # 3×42
    
    # 工具坐标系平移的雅可比：旋转矩阵（3×3）
    J_pos_part2 = g[0:3, 0:3]  # 3×3
    
    # 拼接：3×(42+3) = 3×45
    J_pos = np.hstack([J_pos_part1, J_pos_part2])
    
    return J_pos


def forward_kinematics(kesi, theta, pe0):
    """
    计算正运动学：根据旋量参数和关节角计算末端位置
    
    Args:
        kesi: 旋量参数（6×7）
        theta: 关节角（7×1），弧度
        pe0: 工具坐标系平移（4×1），齐次坐标，米
    
    Returns:
        末端位置（3×1），米
    """
    g = np.eye(4)
    num_joint = kesi.shape[1]
    
    # 从最后一个关节到第一个关节（注意顺序）
    for k in range(num_joint - 1, -1, -1):
        g = exp_se3_r(kesi[0:3, k], kesi[3:6, k], theta[k]) @ g
    
    # 计算末端位置
    pe = g @ pe0
    return pe[0:3]


def forward_g_sequence(kesi, theta_row):
    """
    给定旋量和一行关节角，计算 base->末端（法兰前）齐次变换 g
    
    用于标定工具坐标系平移时，需要计算不含工具坐标系的齐次变换矩阵
    
    Args:
        kesi: 6×7 旋量参数（米）
        theta_row: 7维关节角（弧度），可以是数组或列表
    
    Returns:
        4×4 齐次变换矩阵 g
    """
    g = np.eye(4)
    num_joint = kesi.shape[1]
    # 从第7个关节到第1个关节（从末端到基座）
    for k in range(num_joint - 1, -1, -1):
        g = exp_se3_r(kesi[0:3, k], kesi[3:6, k], theta_row[k]) @ g
    return g

