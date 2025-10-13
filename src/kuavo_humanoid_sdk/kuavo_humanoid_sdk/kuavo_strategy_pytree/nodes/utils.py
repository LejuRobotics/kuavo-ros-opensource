import numpy as np
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoPose,
    KuavoManipulationMpcCtrlMode,
    KuavoArmCtrlMode,
    KuavoManipulationMpcFrame)


def generate_bezier_control_points_c1_continuous(key_points, smoothness_factor=0.3):
    """
    根据关键点生成C1连续的分段贝塞尔曲线控制点

    参数：
        key_points: 关键点列表，每个点为3D位置 (x, y, z)
        smoothness_factor: 平滑因子，控制曲线的弯曲程度，范围[0, 1]

    返回：
        List[numpy.ndarray]: 每段贝塞尔曲线的控制点列表
    """
    key_points = np.array(key_points)
    n_keys = len(key_points)

    if n_keys < 2:
        return [key_points]

    if n_keys == 2:
        # 只有2个关键点，生成一段3阶贝塞尔曲线
        p0, p3 = key_points[0], key_points[1]
        direction = p3 - p0

        p1 = p0 + direction * smoothness_factor
        p2 = p3 - direction * smoothness_factor

        return [np.array([p0, p1, p2, p3])]

    # 多个关键点：生成C1连续的分段贝塞尔曲线
    segments = []

    # 首先计算所有关键点的切线方向和长度
    in_tangents = []  # 进入该点的切线
    out_tangents = []  # 离开该点的切线

    for i in range(n_keys):
        if i == 0:
            # 起点：只有出切线，方向指向下一个点
            out_direction = (key_points[1] - key_points[0]).astype(float)
            out_length = np.linalg.norm(out_direction)
            if out_length > 0:
                out_direction = out_direction / out_length
            distance = out_length * smoothness_factor

            in_tangents.append(np.zeros(3))  # 起点没有入切线
            out_tangents.append(out_direction * distance)

        elif i == n_keys - 1:
            # 终点：只有入切线，方向来自上一个点
            in_direction = (key_points[i] - key_points[i - 1]).astype(float)
            in_length = np.linalg.norm(in_direction)
            if in_length > 0:
                in_direction = in_direction / in_length
            distance = in_length * smoothness_factor

            in_tangents.append(in_direction * distance)
            out_tangents.append(np.zeros(3))  # 终点没有出切线

        else:
            # 中间点：计算平滑的切线方向
            prev_direction = (key_points[i] - key_points[i - 1]).astype(float)
            next_direction = (key_points[i + 1] - key_points[i]).astype(float)

            # 归一化方向向量
            prev_length = np.linalg.norm(prev_direction)
            next_length = np.linalg.norm(next_direction)

            if prev_length > 0:
                prev_direction = prev_direction / prev_length
            if next_length > 0:
                next_direction = next_direction / next_length

            # 计算平均切线方向（保证C1连续性）
            avg_direction = (prev_direction + next_direction) / 2.0
            avg_length = np.linalg.norm(avg_direction)
            if avg_length > 0:
                avg_direction = avg_direction / avg_length

            # 使用较小的距离来控制切线长度，避免过度弯曲
            distance = min(prev_length, next_length) * smoothness_factor * 0.5

            in_tangents.append(avg_direction * distance)
            out_tangents.append(avg_direction * distance)

    # 生成每段的控制点
    for i in range(n_keys - 1):
        p0 = key_points[i]  # 当前段起点
        p3 = key_points[i + 1]  # 当前段终点

        # 使用出切线和入切线计算控制点
        p1 = p0 + out_tangents[i]  # 起点 + 出切线
        p2 = p3 - in_tangents[i + 1]  # 终点 - 入切线

        segments.append(np.array([p0, p1, p2, p3]))

    return segments


def generate_bezier_control_points(key_points, smoothness_factor=0.3):
    """
    根据关键点生成贝塞尔曲线的控制点（兼容旧接口）

    参数：
        key_points: 关键点列表，每个点为3D位置 (x, y, z)
        smoothness_factor: 平滑因子，控制曲线的弯曲程度，范围[0, 1]

    返回：
        numpy.ndarray: 控制点数组
    """
    segments = generate_bezier_control_points_c1_continuous(key_points, smoothness_factor)

    # 将所有段的控制点合并为一个数组（为了兼容旧代码）
    all_control_points = []
    for i, segment in enumerate(segments):
        if i == 0:
            # 第一段：添加所有4个控制点
            all_control_points.extend(segment)
        else:
            # 后续段：跳过第一个控制点（因为它与前一段的最后一个控制点相同）
            all_control_points.extend(segment[1:])

    return np.array(all_control_points)


def bezier_interpolate_poses_full_trajectory(key_poses_list, num_points=100):
    """
    对完整轨迹进行贝塞尔插值

    参数：
        key_poses_list: 关键姿态列表，每个元素为KuavoPose或Pose
        num_points: 总的插值点数

    返回：
        List[KuavoPose]: 插值后的完整轨迹
    """
    if len(key_poses_list) < 2:
        return key_poses_list

    # 提取位置和四元数
    positions = []
    quaternions = []

    for pose in key_poses_list:
        if hasattr(pose, 'position') and hasattr(pose, 'orientation'):
            # KuavoPose
            positions.append(pose.position)
            quaternions.append(pose.orientation)
        elif hasattr(pose, 'pos') and hasattr(pose, 'quat'):
            # Pose
            positions.append(pose.pos)
            if hasattr(pose.quat, 'tolist'):
                quaternions.append(pose.quat.tolist())
            else:
                quaternions.append(pose.quat)

    positions = np.array(positions)
    quaternions = np.array(quaternions)

    # 确保四元数方向一致
    for i in range(1, len(quaternions)):
        if np.dot(quaternions[i - 1], quaternions[i]) < 0:
            quaternions[i] = -quaternions[i]

    # 生成位置的贝塞尔控制点
    position_control_points = generate_bezier_control_points(positions, smoothness_factor=0.2)

    # 生成插值轨迹
    interpolated_poses = []
    t_values = np.linspace(0, 1, num_points)

    # 生成C1连续的分段贝塞尔曲线（降低平滑因子避免过度弯曲）
    position_segments = generate_bezier_control_points_c1_continuous(positions, smoothness_factor=0.15)

    # 验证C1连续性和控制点合理性
    for i, segment in enumerate(position_segments):
        p0, p1, p2, p3 = segment

        # 检查控制点是否偏离过远
        control_distance_1 = np.linalg.norm(p1 - p0)
        control_distance_2 = np.linalg.norm(p3 - p2)
        segment_length = np.linalg.norm(p3 - p0)

        if i < len(position_segments) - 1:
            current_end = position_segments[i][3]  # 当前段终点
            next_start = position_segments[i + 1][0]  # 下一段起点
            connection_error = np.linalg.norm(current_end - next_start)

    for t in t_values:
        # 确定当前t值属于哪个段
        num_segments = len(position_segments)
        segment_index = min(int(t * num_segments), num_segments - 1)

        # 计算在当前段内的局部参数
        if num_segments == 1:
            local_t = t
        else:
            segment_start = segment_index / num_segments
            segment_end = (segment_index + 1) / num_segments
            local_t = (t - segment_start) / (segment_end - segment_start) if segment_end > segment_start else 0.0
            local_t = np.clip(local_t, 0.0, 1.0)

        # 使用当前段的控制点进行贝塞尔插值
        segment_control_points = position_segments[segment_index]
        interp_pos = bezier_curve(segment_control_points, local_t)

        # 四元数插值 - 使用SLERP
        # 找到当前t值对应的段
        if len(quaternions) == 2:
            start_quat = quaternions[0]
            end_quat = quaternions[1]
            quat_t = t
        else:
            segment_length = 1.0 / (len(quaternions) - 1)
            segment_index = min(int(t / segment_length), len(quaternions) - 2)
            quat_t = (t - segment_index * segment_length) / segment_length
            start_quat = quaternions[segment_index]
            end_quat = quaternions[segment_index + 1]

        # SLERP插值
        cos_half_theta = np.dot(start_quat, end_quat)
        cos_half_theta = np.clip(cos_half_theta, -1.0, 1.0)

        if abs(cos_half_theta) >= 1.0:
            interp_quat = start_quat
        else:
            half_theta = np.arccos(cos_half_theta)
            sin_half_theta = np.sqrt(1.0 - cos_half_theta * cos_half_theta)

            if abs(sin_half_theta) < 0.001:
                interp_quat = start_quat * (1 - quat_t) + end_quat * quat_t
                interp_quat = interp_quat / np.linalg.norm(interp_quat)
            else:
                ratio_a = np.sin((1 - quat_t) * half_theta) / sin_half_theta
                ratio_b = np.sin(quat_t * half_theta) / sin_half_theta
                interp_quat = start_quat * ratio_a + end_quat * ratio_b
                interp_quat = interp_quat / np.linalg.norm(interp_quat)

        # 创建插值后的姿态
        interpolated_poses.append(KuavoPose(
            position=interp_pos.tolist(),
            orientation=interp_quat.tolist()
        ))

    return interpolated_poses


def generate_full_bezier_trajectory(
        current_left_pose,
        current_right_pose,
        left_keypoints_list,
        right_keypoints_list,
):
    """
    生成完整的贝塞尔轨迹（包含所有关键点）
    """

    # left_target_list, right_target_list = self.target

    # 获取当前手臂位置作为起始点

    # 构建包含起始位置的完整关键点列表
    left_key_poses = [KuavoPose(
        position=current_left_pose.pos,
        orientation=current_left_pose.quat.tolist()
    )]
    right_key_poses = [KuavoPose(
        position=current_right_pose.pos,
        orientation=current_right_pose.quat.tolist()
    )]

    # 添加所有目标关键点
    for left_pose, right_pose in zip(left_keypoints_list, right_keypoints_list):
        left_key_poses.append(KuavoPose(
            position=left_pose.pos,
            orientation=left_pose.quat.tolist()
        ))
        right_key_poses.append(KuavoPose(
            position=right_pose.pos,
            orientation=right_pose.quat.tolist()
        ))

    # 生成完整的贝塞尔轨迹
    total_points = len(left_keypoints_list) * 50  # 每个关键点段分配50个插值点
    total_points = max(total_points, 100)  # 至少100个点

    left_full_trajectory = bezier_interpolate_poses_full_trajectory(
        left_key_poses, num_points=total_points
    )
    right_full_trajectory = bezier_interpolate_poses_full_trajectory(
        right_key_poses, num_points=total_points
    )

    print(
        f"✅ 生成完整贝塞尔轨迹: 左手{len(left_full_trajectory)}点, 右手{len(right_full_trajectory)}点")

    return left_full_trajectory, right_full_trajectory


def bezier_curve(control_points, t):
    """
    计算n阶贝塞尔曲线上的点

    参数：
        control_points: 控制点列表，shape为(n+1, 3)，其中n为贝塞尔曲线的阶数
        t: 参数值，范围[0, 1]

    返回：
        numpy.ndarray: 贝塞尔曲线上对应t值的点
    """
    n = len(control_points) - 1
    result = np.zeros(3)

    for i in range(n + 1):
        # 计算二项式系数
        binomial_coeff = np.math.comb(n, i)
        # 计算贝塞尔基函数
        basis = binomial_coeff * ((1 - t) ** (n - i)) * (t ** i)
        result += basis * control_points[i]

    return result
