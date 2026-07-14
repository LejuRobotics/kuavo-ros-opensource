#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tag位置转换脚本

功能：
    订阅seeker_markers话题获取marker位置，计算动捕坐标系到base坐标系的变换矩阵，
    并将tag位置从动捕坐标系转换到base坐标系

使用方法:
    python3 transform_tag_to_base.py
"""

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from collections import defaultdict
from kuavo_msgs.msg import sensorsData
from kuavo_msgs.srv import fkSrv

# 数据结构：{frame_id: {marker_id: [x, y, z]}}
# frame_id: 'shoulder' 或 'effector'
# marker_id: marker编号
# [x, y, z]: 位置坐标（单位：毫米）
marker_positions = defaultdict(dict)

# Base位置数据结构：{'position': [x, y, z], 'orientation': [x, y, z, w]}
base_position = None

# sensor_data_raw 最新数据（关节角等）
sensor_data_raw = None

# 动捕标号到默认标号的映射字典，按frame_id区分，默认不映射
# 格式：{'shoulder': {mocap_id: default_id}, 'effector': {mocap_id: default_id}}
# shoulder: 4个marker(1-4), effector: 5个marker(1-5)
mocap_marker_mapping = {'shoulder': {}, 'effector': {}}



def quaternion_to_euler(qx, qy, qz, qw):
    """
    将四元数转换为欧拉角（ZYX顺序）
    
    参数:
        qx, qy, qz, qw: 四元数的x, y, z, w分量
        
    返回:
        yaw, pitch, roll: 欧拉角（度）
    """
    # 构建旋转矩阵
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])
    
    # 计算欧拉角（ZYX顺序）
    yaw = np.arctan2(R[1, 0], R[0, 0]) * 180 / np.pi
    pitch = np.arcsin(-R[2, 0]) * 180 / np.pi
    roll = np.arctan2(R[2, 1], R[2, 2]) * 180 / np.pi
    
    return yaw, pitch, roll


def _normalize(v, eps=1e-9):
    """向量归一化"""
    n = float(np.linalg.norm(v))
    if n < eps:
        return None
    return v / n


# 结构定义：marker相对于marker1的位置（在base坐标系下，单位：毫米）
SHOULDER_RELATIVE_TO_1_MM = {
    1: np.array([0.0, 0.0, 0.0]),
    2: np.array([70.0, 0.0, 60.0]),
    3: np.array([95.0, 0.0, -20.0]),
    4: np.array([70.0, -50.0, 0.0]),
}

# base_link到shoulder_array（marker1）的变换矩阵（4x4齐次变换，单位：毫米）
T_BASE_LINK_TO_MARKER1 = np.array([
    [1, 0, 0, -71.5],
    [0, 1, 0, 199.5],
    [0, 0, 1, 499.5],
    [0, 0, 0, 1],
])

# 手臂阵列（effector）中球相对于1号球的位置（单位：毫米）
# base_ball=1, 2nd_ball=2, 3rd_ball=3, 4th_ball=4, 5th_ball=5
EFFECTOR_RELATIVE_TO_1_MM = {
    1: np.array([0.0, 0.0, 0.0]),
    2: np.array([-70.0, 0.0525, -49.9475]),
    3: np.array([-70.0, 60.0, 0.0]),
    4: np.array([-120.0, 0.0525, -19.9475]),
    5: np.array([-52.5, -35.0, 0.0]),
}

# link7(r_hand_pitch)到base_ball（effector阵列1号球）的变换矩阵（4x4齐次变换，单位：毫米）
T_LINK7_TO_EFFECTOR_ARRAY = np.array([
    [1, 0, 0, 70.0],
    [0, 1, 0, 0.5],
    [0, 0, 1, -79.6525],
    [0, 0, 0, 1],
])

T_LINK7_TO_TOOL_OFFSET = np.array([
    [1, 0, 0, 0.0],
    [0, 1, 0, 30.0],
    [0, 0, 1, -170.0],
    [0, 0, 0, 1],
])

def compute_mocap_to_base_by_vectors(marker1_mocap, marker2_mocap, marker3_mocap, marker2_id, marker3_id,
                                     relative_mm=None):
    """
    使用三个marker点（两个向量）求解动捕到marker1坐标系的齐次变换矩阵
    
    算法思路：结构参数(relative_mm)定义期望坐标系，marker2在结构下为(70,0,60)等；
    动捕实测两个向量与结构期望向量的偏差用于求解旋转矩阵。
    输出坐标系为结构坐标系，marker2在该系下坐标为(70,0,60)等，而非(|v12|,0,0)。
    
    参数:
        marker1_mocap: marker1在动捕坐标系下的位置 [x, y, z]（单位：毫米）
        marker2_mocap: marker2在动捕坐标系下的位置 [x, y, z]（单位：毫米）
        marker3_mocap: marker3在动捕坐标系下的位置 [x, y, z]（单位：毫米）
        marker2_id: marker2的ID
        marker3_id: marker3的ID
        relative_mm: {marker_id: np.array([x,y,z])} 结构参数，定义marker在结构系下的位置，默认SHOULDER_RELATIVE_TO_1_MM
        
    返回:
        T_mocap_to_marker1: 4x4齐次变换矩阵，失败返回None
    """
    if relative_mm is None:
        relative_mm = SHOULDER_RELATIVE_TO_1_MM
    if marker2_id not in relative_mm or marker3_id not in relative_mm:
        rospy.logerr(f"无效的marker ID {marker2_id}/{marker3_id}，relative_mm中需包含对应键")
        return None
    
    p1_mocap = np.array(marker1_mocap, dtype=float)
    p2_mocap = np.array(marker2_mocap, dtype=float)
    p3_mocap = np.array(marker3_mocap, dtype=float)
    
    # 1. 动捕实测向量（测量值）
    v12_mocap = p2_mocap - p1_mocap
    v13_mocap = p3_mocap - p1_mocap
    
    # 2. 结构参数定义的期望向量（结构系下，如 marker2=(70,0,60) 即 X前70mm, Z上60mm）
    v12_struct = relative_mm[marker2_id] - relative_mm[1]
    v13_struct = relative_mm[marker3_id] - relative_mm[1]
    
    v12_mocap_norm = _normalize(v12_mocap)
    v13_mocap_norm = _normalize(v13_mocap)
    v12_struct_norm = _normalize(v12_struct)
    v13_struct_norm = _normalize(v13_struct)
    
    if v12_mocap_norm is None or v13_mocap_norm is None or v12_struct_norm is None or v13_struct_norm is None:
        rospy.logerr(f"无法归一化向量（Marker1与Marker{marker2_id}、Marker{marker3_id}）")
        return None
    
    # 检查两向量是否共线（共线则无法唯一确定绕该轴的旋转）
    cross_mocap = np.cross(v12_mocap_norm, v13_mocap_norm)
    cross_struct = np.cross(v12_struct_norm, v13_struct_norm)
    
    if np.linalg.norm(cross_mocap) < 1e-6 or np.linalg.norm(cross_struct) < 1e-6:
        rospy.logerr(f"向量共线，无法构建坐标系（Marker1与Marker{marker2_id}、Marker{marker3_id}）")
        return None
    
    # 3. 在动捕系下由实测向量构建正交基（表示刚体当前朝向）
    x_mocap = v12_mocap_norm
    z_mocap = _normalize(cross_mocap)
    if z_mocap is None:
        return None
    y_mocap = _normalize(np.cross(z_mocap, x_mocap))
    if y_mocap is None:
        return None
    
    # 4. 在结构系下由期望向量构建正交基（与结构参数同一套约定）
    x_struct = v12_struct_norm
    z_struct = _normalize(cross_struct)
    if z_struct is None:
        return None
    y_struct = _normalize(np.cross(z_struct, x_struct))
    if y_struct is None:
        return None
    
    # 5. 旋转矩阵：将动捕系下的点变换到结构系
    #    R 满足 R @ v12_mocap = v12_struct, R @ v13_mocap = v13_struct
    #    从而输出系下 marker2 在 (70,0,60) 等结构定义位置
    R_mocap = np.column_stack([x_mocap, y_mocap, z_mocap])   # 动捕系下的刚体基
    R_struct = np.column_stack([x_struct, y_struct, z_struct])  # 结构系下的刚体基
    R_mocap_to_base = R_struct @ R_mocap.T
    
    # 6. 平移：结构系原点在 marker1
    t_mocap_to_base = -R_mocap_to_base @ p1_mocap
    
    T_mocap_to_base = np.eye(4)
    T_mocap_to_base[:3, :3] = R_mocap_to_base
    T_mocap_to_base[:3, 3] = t_mocap_to_base
    
    return T_mocap_to_base


def _set_mocap_marker_order_for_frame(frame_id, order_list, n_expected, valid_sets, frame_name):
    """内部函数：为指定frame设置映射"""
    global mocap_marker_mapping
    if frame_id not in mocap_marker_mapping:
        mocap_marker_mapping[frame_id] = {}
    if not isinstance(order_list, list) or len(order_list) != n_expected:
        rospy.logerr(f"{frame_name} order_list必须是包含{n_expected}个元素的列表")
        return False
    order_set = set(order_list)
    if order_set not in valid_sets:
        rospy.logerr(f"{frame_name} order_list标号无效，当前输入: {order_list}")
        return False
    mocap_marker_mapping[frame_id] = {order_list[i]: i + 1 for i in range(n_expected)}
    rospy.loginfo(f"{frame_name} 动捕标号映射已设置: {mocap_marker_mapping[frame_id]}")
    return True


def set_mocap_marker_order(shoulder_order=None, effector_order=None):
    """
    设置动捕标号的映射顺序
    
    参数:
        shoulder_order: list - 肩部阵列(4个marker)的标号顺序，例如 [3, 2, 1, 0] 或 [4, 1, 2, 3]
                       表示：列表第i位的动捕标号 -> 默认标号i+1
                       接受0-3或1-4的标号
        effector_order: list - 手臂阵列(5个marker)的标号顺序，例如 [4, 3, 2, 1, 0] 或 [5, 1, 2, 3, 4]
                       接受0-4或1-5的标号
    
    示例:
        set_mocap_marker_order(shoulder_order=[3, 2, 1, 0])
        set_mocap_marker_order(shoulder_order=[3, 2, 1, 0], effector_order=[4, 3, 2, 1, 0])
    """
    global mocap_marker_mapping
    ok = True
    if shoulder_order is not None:
        ok = _set_mocap_marker_order_for_frame(
            'shoulder', shoulder_order, 4,
            [{0, 1, 2, 3}, {1, 2, 3, 4}], "肩部(shoulder)"
        ) and ok
    if effector_order is not None:
        ok = _set_mocap_marker_order_for_frame(
            'effector', effector_order, 5,
            [{0, 1, 2, 3, 4}, {1, 2, 3, 4, 5}], "手臂(effector)"
        ) and ok
    return ok


def marker_callback(msg: Marker):
    """Marker消息回调函数"""
    frame_id = (msg.header.frame_id or "").strip().lower()
    mocap_marker_id = int(msg.id)
    
    # 按frame_id选用对应映射；若未设置则使用原标号
    mapping = mocap_marker_mapping.get(frame_id, {})
    if mapping:
        default_marker_id = mapping.get(mocap_marker_id, mocap_marker_id)
    else:
        default_marker_id = mocap_marker_id
    
    marker_positions[frame_id][default_marker_id] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]


def _average_rotation_matrices(R_list):
    """
    对多个旋转矩阵求平均
    
    参数:
        R_list: list of np.ndarray - 旋转矩阵列表（每个都是3x3）
        
    返回:
        np.ndarray: 平均后的旋转矩阵（3x3），如果失败则返回None
    
    方法：
        对旋转矩阵的元素求平均，然后使用SVD分解进行正交化
    """
    if not R_list:
        return None
    
    # 对旋转矩阵的元素求平均
    R_avg = np.mean(R_list, axis=0)
    
    # 使用SVD分解进行正交化
    U, S, Vt = np.linalg.svd(R_avg)
    
    # 重新构建正交旋转矩阵
    R_orthogonal = U @ Vt
    
    # 确保是右手系（行列式为1）
    if np.linalg.det(R_orthogonal) < 0:
        U[:, -1] *= -1
        R_orthogonal = U @ Vt
    
    return R_orthogonal


def compute_shoulder_frame(marker_dict):
    """
    使用4个marker点计算从动捕坐标系到base_link坐标系的齐次变换矩阵
    
    通过marker1与其他三个marker之间的三个向量分别求解三个变换矩阵，然后求平均以提高稳定性。
    最后应用base_link到shoulder_array（marker1）的变换矩阵，将结果转换到base_link坐标系。
    
    参数:
        marker_dict: {marker_id: [x, y, z]} 字典，包含marker 1, 2, 3, 4的位置（单位：毫米）
        
    返回:
        T_base_to_mocap: 4x4齐次变换矩阵，从base_link到mocap（逆变换），便于显示base在mocap下的位置/欧拉角，失败返回None
    """
    if not all(i in marker_dict for i in (1, 2, 3, 4)):
        rospy.logerr("缺少必需的marker点（需要1, 2, 3, 4）")
        return None
    
    marker1_pos = marker_dict[1]
    T_list = []
    
    # 使用不同的向量组合求解变换矩阵（需要两个不共线的向量）
    # 组合1: marker1->marker2 和 marker1->marker3
    T = compute_mocap_to_base_by_vectors(marker1_pos, marker_dict[2], marker_dict[3], 2, 3)
    if T is not None:
        T_list.append(T)
    
    # 组合2: marker1->marker2 和 marker1->marker4
    T = compute_mocap_to_base_by_vectors(marker1_pos, marker_dict[2], marker_dict[4], 2, 4)
    if T is not None:
        T_list.append(T)
    
    # 组合3: marker1->marker3 和 marker1->marker4
    T = compute_mocap_to_base_by_vectors(marker1_pos, marker_dict[3], marker_dict[4], 3, 4)
    if T is not None:
        T_list.append(T)
    
    if len(T_list) == 0:
        rospy.logerr("所有向量求解都失败")
        return None
    
    # 提取旋转矩阵和平移向量并求平均
    R_list = [T[:3, :3] for T in T_list]
    t_list = [T[:3, 3] for T in T_list]
    
    R_avg = _average_rotation_matrices(R_list)
    if R_avg is None:
        rospy.logerr("旋转矩阵平均失败")
        return None
    
    t_avg = np.mean(t_list, axis=0)
    
    # 构建从mocap到marker1坐标系的齐次变换矩阵
    T_mocap_to_marker1 = np.eye(4)
    T_mocap_to_marker1[:3, :3] = R_avg
    T_mocap_to_marker1[:3, 3] = t_avg
    
    # 应用base_link到shoulder_array（marker1）的变换
    T_marker1_to_base_link = T_BASE_LINK_TO_MARKER1
    T_mocap_to_base_link = T_marker1_to_base_link @ T_mocap_to_marker1
    # 返回T_base_to_mocap（逆变换），便于显示base在mocap下的位置和欧拉角
    return np.linalg.inv(T_mocap_to_base_link)


def compute_effector_frame(marker_dict):
    """
    使用5个marker点计算从动捕坐标系到effector阵列(marker1)的齐次变换矩阵
    
    先通过手臂mark阵列求出阵列的位置和朝向（以marker1为原点）。
    
    参数:
        marker_dict: {marker_id: [x, y, z]} 字典，包含marker 1, 2, 3, 4, 5的位置（单位：毫米）
        
    返回:
        T_mocap_to_effector_array: 4x4齐次变换矩阵，从mocap到effector阵列坐标系，失败返回None
    """
    if not all(i in marker_dict for i in (1, 2, 3, 4, 5)):
        rospy.logerr("缺少必需的marker点（需要1, 2, 3, 4, 5）")
        return None
    
    marker1_pos = marker_dict[1]
    T_list = []
    pairs = [(2, 3), (2, 4), (2, 5), (3, 4), (3, 5), (4, 5)]
    
    for id2, id3 in pairs:
        T = compute_mocap_to_base_by_vectors(
            marker1_pos, marker_dict[id2], marker_dict[id3], id2, id3,
            relative_mm=EFFECTOR_RELATIVE_TO_1_MM
        )
        if T is not None:
            T_list.append(T)
    
    if len(T_list) == 0:
        rospy.logerr("所有向量求解都失败")
        return None
    
    R_list = [T[:3, :3] for T in T_list]
    t_list = [T[:3, 3] for T in T_list]
    R_avg = _average_rotation_matrices(R_list)
    if R_avg is None:
        return None
    
    T_mocap_to_effector_array = np.eye(4)
    T_mocap_to_effector_array[:3, :3] = R_avg
    T_mocap_to_effector_array[:3, 3] = np.mean(t_list, axis=0)

    T_effector_to_link7 = T_LINK7_TO_EFFECTOR_ARRAY
    T_mocap_to_link7 = T_effector_to_link7 @ T_mocap_to_effector_array
    return np.linalg.inv(T_mocap_to_link7)


def base_pose_callback(msg: Pose):
    global base_position
    base_position = {
        'position': [msg.position.x, msg.position.y, msg.position.z],
        'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    }


def sensor_data_raw_callback(msg: sensorsData):
    """订阅 /sensor_data_raw 或 /sensors_data_raw 的关节数据"""
    global sensor_data_raw
    sensor_data_raw = msg


def fk_srv_client(joint_angles):
    """
    调用 /ik/fk_srv 服务（与 example_fk_srv.py 一致）
    joint_angles: 14维手臂关节角（弧度），[左臂7, 右臂7]
    返回: hand_poses 或 None
    """
    try:
        rospy.wait_for_service('/ik/fk_srv', timeout=1.0)
        fk_srv = rospy.ServiceProxy('/ik/fk_srv', fkSrv)
        fk_result = fk_srv(list(joint_angles))
        return fk_result.hand_poses if fk_result.success else None
    except rospy.ServiceException as e:
        rospy.logwarn_throttle(5, "FK服务调用失败: %s" % e)
        return None


def call_fk_for_left_arm(joint_q):
    """
    从 sensor_data_raw.joint_q 取 12-25（双臂14轴），调用 FK 服务，返回左臂末端位姿。
    返回: (pos, quat) 或 (None, None)，pos 单位米，quat [x,y,z,w]
    """
    if joint_q is None or len(joint_q) < 26:
        return None, None
    arm_joints = joint_q[12:26]  # 左臂12-18，右臂19-25，共14个
    hand_poses = fk_srv_client(arm_joints)
    if hand_poses is None:
        return None, None
    left_pose = hand_poses.left_pose
    pos = np.array(left_pose.pos_xyz, dtype=float)
    quat = np.array(left_pose.quat_xyzw, dtype=float)
    return pos, quat


def main():
    """主函数"""
    rospy.init_node('marker_position_subscriber', anonymous=True)
    
    # ========== 动捕标号映射设置 ==========
    # shoulder: 4个marker; effector: 5个marker
    set_mocap_marker_order(shoulder_order=[0, 2, 3, 1], effector_order=[1, 3, 0, 2, 4])
    
    # 订阅seeker_markers话题
    rospy.Subscriber('seeker_markers', Marker, marker_callback, queue_size=1000)
    

    # 订阅base在动捕坐标系的话题
    rospy.Subscriber('base_pose', Pose, base_pose_callback, queue_size=1000)
    
    # 订阅关节数据（humanoid_controller 发布 /share_memory/sensor_data_raw，硬件/仿真发布 /sensors_data_raw）
    rospy.Subscriber('/sensor_data_raw', sensorsData, sensor_data_raw_callback, queue_size=10)
    rospy.Subscriber('/sensors_data_raw', sensorsData, sensor_data_raw_callback, queue_size=10)
    rospy.Subscriber('/share_memory/sensor_data_raw', sensorsData, sensor_data_raw_callback, queue_size=10)
    
    rospy.loginfo("开始订阅marker位置，等待数据...")
    rospy.sleep(2.0)
    
    rate = rospy.Rate(1.0)
    
    while not rospy.is_shutdown():
        try:
            # 获取shoulder marker位置
            shoulder_markers = marker_positions.get('shoulder', {})
            # 获取手臂阵列（effector）位置
            effector_markers = marker_positions.get('effector', {})
            
            # 计算base在动捕坐标系下的变换矩阵（T_base_to_mocap）
            T_base_to_mocap = compute_shoulder_frame(shoulder_markers)
            if T_base_to_mocap is None:
                rospy.logwarn("计算base变换矩阵失败，跳过本次循环")
                rate.sleep()
                continue
            # 计算link7在动捕坐标系下的变换矩阵（T_link7_to_mocap）
            T_link7_to_mocap = compute_effector_frame(effector_markers)
            if T_link7_to_mocap is None:
                rospy.logwarn("计算link7变换矩阵失败，跳过本次循环")
                rate.sleep()
                continue
                  
            # 每5秒打印一次变换矩阵和验证信息
            # 使用T_base_to_mocap的旋转矩阵计算欧拉角，这样可以得到正确的偏航角符号
            R = T_base_to_mocap[:3, :3]
            # 计算欧拉角（ZYX顺序）
            yaw = np.arctan2(R[1, 0], R[0, 0]) * 180 / np.pi
            pitch = np.arcsin(-R[2, 0]) * 180 / np.pi
            roll = np.arctan2(R[2, 1], R[2, 2]) * 180 / np.pi
            
            # 打印转换后的位置和欧拉角
            pos = T_base_to_mocap[:3, 3]
            rospy.loginfo_throttle(5, 
                f"转换后位置: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] mm, "
                f"欧拉角: Yaw={yaw:.2f}°, Pitch={pitch:.2f}°, Roll={roll:.2f}°")
            
            # 打印base_pose话题的位置和欧拉角
            if base_position is not None:
                base_pos = np.array(base_position['position'])
                base_orient = base_position['orientation']
                base_yaw, base_pitch, base_roll = quaternion_to_euler(
                    base_orient[0], base_orient[1], base_orient[2], base_orient[3]
                )
                rospy.loginfo_throttle(5,
                    f"base_pose位置: [{base_pos[0]:.2f}, {base_pos[1]:.2f}, {base_pos[2]:.2f}] mm, "
                    f"欧拉角: Yaw={base_yaw:.2f}°, Pitch={base_pitch:.2f}°, Roll={base_roll:.2f}°")
            
            T_link7_to_base = np.linalg.inv(T_base_to_mocap)@T_link7_to_mocap
            T_TOOL_to_base = T_link7_to_base @ T_LINK7_TO_TOOL_OFFSET
            R = T_TOOL_to_base[:3, :3]
            # 计算欧拉角（ZYX顺序）
            yaw_l7 = np.arctan2(R[1, 0], R[0, 0]) * 180 / np.pi
            pitch_l7 = np.arcsin(-R[2, 0]) * 180 / np.pi
            roll_l7 = np.arctan2(R[2, 1], R[2, 2]) * 180 / np.pi
            rospy.loginfo_throttle(5,
                f"TOOL在base下: 位置[{T_TOOL_to_base[:3, 3][0]:.2f}, {T_TOOL_to_base[:3, 3][1]:.2f}, {T_TOOL_to_base[:3, 3][2]:.2f}] mm, "
                f"欧拉角: Yaw={yaw_l7:.2f}°, Pitch={pitch_l7:.2f}°, Roll={roll_l7:.2f}°")
            
            # 从 sensor_data_raw 取 12-25 电机角度，调用 FK 算左臂末端位姿（与 example_fk_srv.py 同逻辑）
            if sensor_data_raw is not None and hasattr(sensor_data_raw, 'joint_data') and hasattr(sensor_data_raw.joint_data, 'joint_q'):
                joint_q = list(sensor_data_raw.joint_data.joint_q)
                pos_fk, quat_fk = call_fk_for_left_arm(joint_q)
                if pos_fk is not None:
                    pos_fk_mm = pos_fk * 1000.0
                    yaw_fk, pitch_fk, roll_fk = quaternion_to_euler(quat_fk[0], quat_fk[1], quat_fk[2], quat_fk[3])
                    rospy.loginfo_throttle(5,
                        f"FK左臂link7(12-18): 位置[{pos_fk_mm[0]:.2f}, {pos_fk_mm[1]:.2f}, {pos_fk_mm[2]:.2f}] mm, "
                        f"欧拉角: Yaw={yaw_fk:.2f}°, Pitch={pitch_fk:.2f}°, Roll={roll_fk:.2f}°")
                else:
                    rospy.logwarn_throttle(5, "FK服务返回失败，请确认 /ik/fk_srv 可用且 motion_capture_ik 已启动")
            else:
                rospy.logwarn_throttle(5, "无 sensor_data_raw 数据，FK 未计算。请确认 /sensors_data_raw 或 /share_memory/sensor_data_raw 在发布")
            
        except Exception as e:
            rospy.logerr(f"处理过程中出错: {e}")
            import traceback
            traceback.print_exc()
        
        rate.sleep()


if __name__ == "__main__":
    main()
