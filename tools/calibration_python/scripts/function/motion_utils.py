#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
通用运动/坐标工具函数：
- 关节角生成（含安全边界）
- 关节空间贝塞尔插值轨迹
- 工作空间判定
- Pose <-> 4x4 齐次矩阵转换
- 动捕坐标系到 base_link 的变换更新
- 四元数与欧拉角转换
- 姿态误差计算
"""
import time
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_matrix


# ========== 关节角生成 ==========

def generate_random_joint_angles(num_points, left_limit, right_limit, arm="left", seed=None):
    if seed is not None:
        np.random.seed(seed)

    if arm == "left":
        joint_limit = left_limit
        num_joints = 7
    elif arm == "right":
        joint_limit = right_limit
        num_joints = 7
    else:  # both
        joint_limit = np.vstack([left_limit, right_limit])
        num_joints = 14

    joint_angles_rad = np.zeros((num_points, num_joints))
    for i in range(num_points):
        for j in range(num_joints):
            low, high = joint_limit[j]
            joint_angles_rad[i, j] = np.random.uniform(low, high)

    joint_angles_deg = np.degrees(joint_angles_rad)
    return joint_angles_rad, joint_angles_deg


def generate_joint_angles_with_margin(num_points, left_limit, right_limit, margin_ratio=0.1, arm="left", seed=None):
    if seed is not None:
        np.random.seed(seed)

    if arm == "left":
        joint_limit = left_limit.copy()
        num_joints = 7
    elif arm == "right":
        joint_limit = right_limit.copy()
        num_joints = 7
    else:  # both
        joint_limit = np.vstack([left_limit, right_limit])
        num_joints = 14

    joint_angles_rad = np.zeros((num_points, num_joints))
    for i in range(num_points):
        for j in range(num_joints):
            low, high = joint_limit[j]
            span = high - low
            safe_low = low + span * margin_ratio
            safe_high = high - span * margin_ratio
            joint_angles_rad[i, j] = np.random.uniform(safe_low, safe_high)

    joint_angles_deg = np.degrees(joint_angles_rad)
    return joint_angles_rad, joint_angles_deg


# ========== 关节空间贝塞尔插值 ==========

def interpolate_joint_positions_bezier(start_joints, end_joints, num_points=20):
    start_joints = np.array(start_joints, dtype=float)
    end_joints = np.array(end_joints, dtype=float)
    t = np.linspace(0, 1, num_points)
    interp = []
    for ti in t:
        mid_point = (start_joints + end_joints) / 2.0
        offset = (end_joints - start_joints) * 0.1
        control_point = mid_point + offset
        joint_pos = (1 - ti) ** 2 * start_joints + 2 * (1 - ti) * ti * control_point + ti**2 * end_joints
        interp.append(joint_pos.tolist())
    return interp


def build_jointspace_bezier_trajectory(target_positions_deg, init_pos_deg, time_per_point, publish_hz,
                                       num_points_per_segment=None, dwell_time_per_point=None):
    points_per_segment = num_points_per_segment or max(5, int(time_per_point * publish_hz))
    dwell_time = dwell_time_per_point if dwell_time_per_point is not None else 0
    dwell_steps = max(0, int(dwell_time * publish_hz))
    trajectory_deg = []

    current = init_pos_deg
    for idx, target in enumerate(target_positions_deg):
        segment = interpolate_joint_positions_bezier(current, target, num_points=points_per_segment)
        trajectory_deg.extend(segment)
        current = target
        if dwell_steps > 0 and idx < len(target_positions_deg) - 1:
            trajectory_deg.extend([target] * dwell_steps)
    return trajectory_deg


# ========== 工作空间检查 ==========

def check_workspace(pos, workspace):
    if pos is None or len(pos) < 3:
        return False
    x, y, z = pos[0], pos[1], pos[2]
    return (
        workspace["x_min"] <= x <= workspace["x_max"]
        and workspace["y_min"] <= y <= workspace["y_max"]
        and workspace["z_min"] <= z <= workspace["z_max"]
    )


# ========== 动捕/坐标系工具 ==========

def pose_to_mat(pose_msg):
    T = np.eye(4)
    q = pose_msg.pose.orientation
    p = pose_msg.pose.position
    T[:3, :3] = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    T[0, 3] = p.x
    T[1, 3] = p.y
    T[2, 3] = p.z
    return T


def mat_to_pose(T):
    quat = quaternion_from_matrix(T)
    pos = T[:3, 3]
    return pos, quat


def generate_filtered_joint_targets(
    num_points,
    left_limit,
    right_limit,
    init_arm_pos_deg,
    left_workspace,
    right_workspace,
    arm_side='right',
    margin_ratio=0.1,
    timeout_s=5.0,
    fk_service_callback=None,
):
    """
    生成随机关节角并过滤工作空间
    
    该函数会：
    1. 根据关节限位随机生成目标关节角（留有安全边界）
    2. 调用FK服务检查末端位置是否在工作空间内
    3. 过滤掉超出工作空间的点，仅保留有效目标
    4. 支持超时机制，避免无限循环
    
    Args:
        num_points: 需要生成的有效目标点数
        left_limit: 左臂关节限位矩阵 (7×2, rad)
        right_limit: 右臂关节限位矩阵 (7×2, rad)
        init_arm_pos_deg: 初始手臂位置 (14,) 度
        left_workspace: 左臂工作空间字典 {'x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max'}
        right_workspace: 右臂工作空间字典 {'x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max'}
        arm_side: 手臂侧，'left'、'right' 或 'both'
        margin_ratio: 安全边界比例，默认0.1（10%）
        timeout_s: 超时时间（秒），默认5.0
        fk_service_callback: FK服务回调函数，接收14关节角（弧度列表），返回 (left_pos, right_pos) 或 (None, None)
    
    Returns:
        list: 过滤后的目标关节角列表，每个元素是 (14,) 度的numpy数组
    """
    filtered_targets = []
    seed_counter = 0
    start_time = time.time()
    
    if fk_service_callback is None:
        raise ValueError("fk_service_callback 不能为 None")
    
    while len(filtered_targets) < num_points:
        if time.time() - start_time > timeout_s:
            raise TimeoutError(
                f"生成超时：过滤后仅 {len(filtered_targets)}/{num_points}，超出 {timeout_s}s"
            )
        
        # 逐点生成
        if arm_side == 'left':
            _, left_deg = generate_joint_angles_with_margin(
                1,
                left_limit=left_limit,
                right_limit=right_limit,
                margin_ratio=margin_ratio,
                arm='left',
                seed=42 + seed_counter,
            )
            target_deg = np.zeros(14)
            target_deg[:7] = left_deg[0]
            target_deg[7:] = np.array(init_arm_pos_deg[7:])
        elif arm_side == 'right':
            _, right_deg = generate_joint_angles_with_margin(
                1,
                left_limit=left_limit,
                right_limit=right_limit,
                margin_ratio=margin_ratio,
                arm='right',
                seed=42 + seed_counter,
            )
            target_deg = np.zeros(14)
            target_deg[:7] = np.array(init_arm_pos_deg[:7])
            target_deg[7:] = right_deg[0]
        elif arm_side == 'both':
            _, left_deg = generate_joint_angles_with_margin(
                1,
                left_limit=left_limit,
                right_limit=right_limit,
                margin_ratio=margin_ratio,
                arm='left',
                seed=42 + seed_counter,
            )
            _, right_deg = generate_joint_angles_with_margin(
                1,
                left_limit=left_limit,
                right_limit=right_limit,
                margin_ratio=margin_ratio,
                arm='right',
                seed=43 + seed_counter,
            )
            target_deg = np.zeros(14)
            target_deg[:7] = left_deg[0]
            target_deg[7:] = right_deg[0]
        else:
            raise ValueError(f"ARM_SIDE 配置无效: {arm_side}, 请选择 left/right/both")
        
        seed_counter += 1
        
        # 调用FK服务检查工作空间
        target_rad = np.radians(target_deg).tolist()
        left_pos, right_pos = fk_service_callback(target_rad)
        if left_pos is None or right_pos is None:
            continue
        
        # 工作空间过滤
        if arm_side == 'left' and check_workspace(left_pos, left_workspace):
            filtered_targets.append(target_deg)
        elif arm_side == 'right' and check_workspace(right_pos, right_workspace):
            filtered_targets.append(target_deg)
        elif arm_side == 'both' and check_workspace(left_pos, left_workspace) and check_workspace(right_pos, right_workspace):
            filtered_targets.append(target_deg)
        
        # 进度日志（每10个点或达到目标数量时输出）
        if len(filtered_targets) % 10 == 0 or len(filtered_targets) == num_points:
            try:
                import rospy
                rospy.loginfo(f"已累积有效点 {len(filtered_targets)}/{num_points}")
            except:
                pass  # 如果不在ROS环境中，忽略日志输出
    
    # 截断到刚好所需数量
    return filtered_targets[:num_points]


def quaternion_to_euler(q):
    """
    将四元数转换为欧拉角（ZYX顺序，即roll, pitch, yaw）
    
    Args:
        q: 四元数 (4,) [x, y, z, w]，可以是列表或numpy数组
    
    Returns:
        tuple: (roll, pitch, yaw) 弧度制
    """
    q = np.array(q)
    x, y, z, w = q
    
    # 归一化四元数
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if norm < 1e-10:
        return 0.0, 0.0, 0.0
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    
    # 计算旋转矩阵的元素
    r11 = 1.0 - 2.0 * (y*y + z*z)
    r12 = 2.0 * (x*y - w*z)
    r21 = 2.0 * (x*y + w*z)
    r22 = 1.0 - 2.0 * (x*x + z*z)
    r31 = 2.0 * (x*z - w*y)
    r32 = 2.0 * (y*z + w*x)
    r33 = 1.0 - 2.0 * (x*x + y*y)
    
    # 从旋转矩阵提取欧拉角（ZYX顺序）
    yaw = np.arctan2(r21, r11)
    pitch = np.arcsin(-r31)
    roll = np.arctan2(r32, r33)
    
    return roll, pitch, yaw


def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """
    将欧拉角（度）转换为四元数 (xyzw)
    使用 ZYX 顺序（roll, pitch, yaw）
    
    Args:
        roll_deg: 滚转角（度）
        pitch_deg: 俯仰角（度）
        yaw_deg: 偏航角（度）
    
    Returns:
        numpy.ndarray: 四元数 [x, y, z, w]
    """
    roll = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)
    yaw = np.radians(yaw_deg)
    
    # 计算半角
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    # ZYX 顺序（roll, pitch, yaw）
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return np.array([x, y, z, w])


def quaternion_angle_error(q1, q2):
    """
    计算两个四元数之间的角度误差（弧度）
    
    Args:
        q1: 四元数1 (4,) [x, y, z, w]，可以是列表或numpy数组
        q2: 四元数2 (4,) [x, y, z, w]，可以是列表或numpy数组
    
    Returns:
        float: 角度误差（弧度），范围 [0, π]
    """
    q1 = np.array(q1)
    q2 = np.array(q2)
    
    # 归一化四元数
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    
    # 确保两个四元数在同一个半球上（避免符号翻转导致的跳变）
    # 如果 w 分量的符号相反，将其中一个取反（q 和 -q 表示相同的旋转）
    if q1[3] * q2[3] < 0:
        q2 = -q2
    
    # 计算点积（内积）
    # 点积 = q1 · q2 = x1*x2 + y1*y2 + z1*z2 + w1*w2
    dot_product = np.dot(q1, q2)
    
    # 限制点积范围，防止数值误差
    dot_product = np.clip(dot_product, -1.0, 1.0)
    
    # 角度误差 = 2 * arccos(|dot_product|)
    # 由于已经确保了符号一致性，可以直接使用绝对值
    angle = 2 * np.arccos(np.abs(dot_product))
    
    return angle


def quaternion_multiply(q1, q2):
    """
    四元数乘法：q1 * q2
    四元数格式：[x, y, z, w]
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    return np.array([x, y, z, w])


def quaternion_conjugate(q):
    """
    四元数共轭（逆旋转）
    四元数格式：[x, y, z, w]
    """
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quaternion_relative_rotation(q1, q2):
    """
    计算从 q1 到 q2 的相对旋转四元数
    返回 q_relative，使得 q2 = q_relative * q1
    
    Args:
        q1: 初始四元数 [x, y, z, w]
        q2: 目标四元数 [x, y, z, w]
    
    Returns:
        numpy.ndarray: 相对旋转四元数 [x, y, z, w]
    """
    q1 = np.array(q1)
    q2 = np.array(q2)
    
    # 归一化
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    
    # 计算相对旋转：q_relative = q2 * q1^(-1)
    # q1^(-1) = conjugate(q1) / |q1|^2，由于已归一化，所以就是 conjugate(q1)
    q1_inv = quaternion_conjugate(q1)
    q_relative = quaternion_multiply(q2, q1_inv)
    
    # 归一化相对旋转四元数
    q_relative = q_relative / np.linalg.norm(q_relative)
    
    return q_relative


def pose_difference(pos1, quat1, pos2, quat2, use_quaternion_method=True):
    """
    计算两个姿态之间的详细差值
    
    Args:
        pos1: 姿态1的位置 (3,)，可以是列表或numpy数组
        quat1: 姿态1的四元数 (4,) [x, y, z, w]，可以是列表或numpy数组
        pos2: 姿态2的位置 (3,)，可以是列表或numpy数组
        quat2: 姿态2的四元数 (4,) [x, y, z, w]，可以是列表或numpy数组
        use_quaternion_method: 如果为True，使用四元数相对旋转方法计算角度误差（避免万向锁）
                               如果为False，使用传统欧拉角差值方法
    
    Returns:
        dict: 包含详细差值的字典
            - pos_norm: 位置误差的模（米）
            - pos_diff_x: x方向差值（米）
            - pos_diff_y: y方向差值（米）
            - pos_diff_z: z方向差值（米）
            - roll_diff: roll角度差值（弧度）
            - pitch_diff: pitch角度差值（弧度）
            - yaw_diff: yaw角度差值（弧度）
            - total_angle_error: 总角度误差（弧度，使用四元数方法）
    """
    pos1 = np.array(pos1)
    pos2 = np.array(pos2)
    quat1 = np.array(quat1)
    quat2 = np.array(quat2)
    
    # 位置差值
    pos_diff = pos2 - pos1
    pos_norm = np.linalg.norm(pos_diff)
    
    # 直接计算两个姿态的欧拉角差值（更直观，避免相对旋转转换时的万向锁问题）
    roll1, pitch1, yaw1 = quaternion_to_euler(quat1)
    roll2, pitch2, yaw2 = quaternion_to_euler(quat2)
    
    # 计算角度差值（处理角度环绕问题）
    def angle_diff(a1, a2):
        """计算两个角度之间的差值，处理-π到π的环绕"""
        diff = a2 - a1
        # 将差值归一化到[-π, π]
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff
    
    roll_diff = angle_diff(roll1, roll2)
    pitch_diff = angle_diff(pitch1, pitch2)
    yaw_diff = angle_diff(yaw1, yaw2)
    
    # 计算总角度误差（使用四元数方法，更准确）
    total_angle_error = quaternion_angle_error(quat1, quat2)
    
    return {
        'pos_norm': pos_norm,
        'pos_diff_x': pos_diff[0],
        'pos_diff_y': pos_diff[1],
        'pos_diff_z': pos_diff[2],
        'roll_diff': roll_diff,
        'pitch_diff': pitch_diff,
        'yaw_diff': yaw_diff,
        'total_angle_error': total_angle_error,
    }
    


def update_mocap_transforms(mocap_latest, T_mocap_to_baselink):
    """
    根据 mocap/base + 左/右手 PoseStamped，更新:
    - base_link 在 mocap 下的 4x4
    - 左/右手在 base_link 下的 4x4
    """
    if mocap_latest.get("base_mocap") is None:
        return

    T_base_mocap = pose_to_mat(mocap_latest["base_mocap"])
    T_baselink_in_mocap = T_mocap_to_baselink @ T_base_mocap
    mocap_latest["base_link_in_mocap"] = T_baselink_in_mocap
    T_baselink_inv = np.linalg.inv(T_baselink_in_mocap)

    for key_mocap, key_out in (("left_mocap", "left_in_base"), ("right_mocap", "right_in_base")):
        if mocap_latest.get(key_mocap) is None:
            continue
        T_hand_mocap = pose_to_mat(mocap_latest[key_mocap])
        mocap_latest[key_out] = T_baselink_inv @ T_hand_mocap


# ========== 左右臂镜像转换 ==========

def map_left_to_right_joints(left_joints_deg):
    """
    将左臂关节角映射到右臂关节角（对称映射）
    
    Args:
        left_joints_deg: 左臂7个关节角（度）
    
    Returns:
        numpy.ndarray: 右臂7个关节角（度）
    """
    left_joints_deg = np.array(left_joints_deg)
    right_joints_deg = np.zeros(7)
    
    # 根据关节限位矩阵的对称关系进行映射
    right_joints_deg[0] = left_joints_deg[0]  # pitch: 相同
    right_joints_deg[1] = -left_joints_deg[1]  # roll: 取反（左右对称）
    right_joints_deg[2] = -left_joints_deg[2]  # yaw: 取反（左右对称）
    right_joints_deg[3] = left_joints_deg[3]  # forearm_pitch: 相同
    right_joints_deg[4] = left_joints_deg[4]  # hand_yaw: 相同
    right_joints_deg[5] = -left_joints_deg[5]  # hand_pitch: 取反（左右对称）
    right_joints_deg[6] = left_joints_deg[6]  # hand_roll: 相同
    
    return right_joints_deg


def mirror_quaternion_for_right_arm(quat_left):
    """
    将左臂的四元数姿态镜像到右臂（左右对称，YZ平面镜像）
    
    在右手定则坐标系下，对于左右对称（YZ平面镜像）的姿态变换：
    - roll（绕X轴）：取反（镜像后绕X轴的旋转方向相反）
    - pitch（绕Y轴）：保持不变（Y轴在镜像平面内，旋转方向相同）
    - yaw（绕Z轴）：取反（镜像后绕Z轴的旋转方向相反）
    
    Args:
        quat_left: 左臂四元数 [x, y, z, w]
    
    Returns:
        numpy.ndarray: 右臂四元数 [x, y, z, w]
    """
    # 将四元数转换为欧拉角
    roll, pitch, yaw = quaternion_to_euler(quat_left)
    
    # 左右对称（YZ平面镜像）：roll和yaw取反，pitch保持不变
    roll_mirrored = -roll
    pitch_mirrored = pitch
    yaw_mirrored = -yaw
    
    # 转换回四元数
    quat_right = euler_to_quaternion(
        roll_deg=np.degrees(roll_mirrored),
        pitch_deg=np.degrees(pitch_mirrored),
        yaw_deg=np.degrees(yaw_mirrored)
    )
    return quat_right


def mirror_position_for_right_arm(pos_left):
    """
    将左臂位置镜像到右臂位置（y轴关于x轴对称，x和z保持不变）
    
    Args:
        pos_left: 左臂位置 [x, y, z]
    
    Returns:
        numpy.ndarray: 右臂位置 [x, -y, z]
    """
    pos_left = np.array(pos_left)
    pos_right = np.array([pos_left[0], -pos_left[1], pos_left[2]])
    return pos_right


