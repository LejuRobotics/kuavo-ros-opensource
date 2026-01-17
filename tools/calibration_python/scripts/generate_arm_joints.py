# 控制台颜色
COLOR_PURPLE = "\033[95m"
COLOR_RESET = "\033[0m"
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人手臂标定关节角生成与贝塞尔曲线轨迹规划脚本

脚本作用：
    该脚本用于自动化生成机器人手臂标定所需的关节角数据，主要功能包括：
    1. 自动生成标定位形：根据关节限位随机生成多组目标关节角，覆盖工作空间
    2. 工作空间验证：通过正运动学服务检查每个目标点的末端位置是否在安全工作空间内，自动过滤无效位形
    3. 平滑轨迹执行：使用本地贝塞尔曲线插值生成平滑轨迹，控制机器人依次运动到各个目标点
    4. 自动数据采集：在每个目标点到位后，自动采集实际关节角和计算末端位置，保存为标定数据文件

主要设计点：
    1. 本地关节空间贝塞尔插值：不依赖外部轨迹规划服务，脚本内部实现二次贝塞尔曲线插值，生成平滑轨迹
    2. 工作空间自动过滤：生成关节角后，调用正运动学服务检查末端位置，仅执行在工作空间内的目标点
    3. 自动数据采集：每个目标点到位后，延迟1s，以100ms间隔采集10次关节角求均值，并调用FK服务保存末端位置
    4. 双臂独立控制：支持左臂、右臂或双臂同时运动

使用方法:
    1. 启动机器人控制节点：
       cd kuavo-ros-opensource
       sudo su
       source devel/setup.bash
       roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch joystick_type:=sim
       # 或实物模式: roslaunch humanoid_controllers load_kuavo_real.launch joystick_type:=h12
    
    2. 运行脚本：
       cd docs/3调试教程/scripts
       python3 calibration_generate_arm_joints.py
       # 或测试模式: python3 calibration_generate_arm_joints.py --test

输出文件（保存在 config 目录）：
    - measured_joints_<arm_side>_arm.txt：采集的实际关节角（度），每行7个或14个关节
    - measured_fk_<arm_side>_arm.txt：通过FK服务计算的末端位置（米），每行3个坐标（x, y, z）
    - measured_mocap_<arm_side>_arm.txt：动捕系统采集的末端位置（米），每行3个坐标（x, y, z）

工作流程：
    1. 根据关节限位随机生成NUM_DEBUG_PTS组目标关节角（留有10%安全边界）
    2. 对每个目标点调用FK服务，检查末端位置是否在工作空间内
    3. 过滤掉超出工作空间的点，仅保留有效目标
    4. 使用本地贝塞尔插值生成完整轨迹（不依赖外部服务）
    5. 按100Hz频率发布轨迹到/kuavo_arm_traj
    6. 每个目标点到位后，延迟1s，采集10次关节角（100ms间隔）求均值
    7. 调用FK服务计算末端位置，将关节角和末端位置分别保存到文件
"""

import os
import time
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from kuavo_msgs.msg import sensorsData

from function.motion_utils import (
    generate_joint_angles_with_margin,
    build_jointspace_bezier_trajectory,
    check_workspace,
    update_mocap_transforms,
    generate_filtered_joint_targets,
)
from function.file_utils import get_config_dir
from function.data_utils import (
    call_fk_service_both_arms,
    call_change_arm_ctrl_mode_service,
    save_position_to_file,
)


# ==================== 配置参数 ====================

# 左臂关节限位矩阵 (7×2, rad)
# 说明：左臂7个关节的限位矩阵，格式为[下限, 上限]
# 作用：用于限制随机生成的关节角范围，确保在机械限位内
LEFT_ARM_JOINT_LIMIT = np.array([
    [-2.4,    0.6],    # l1: l_arm_pitch
    [-0.349,  1.46],   # l2: l_arm_roll
    [-1.57,   0.46],   # l3: l_arm_yaw
    [-2.618,  0.0],    # l4: l_forearm_pitch
    [-1.57,   1.57],   # l5: l_hand_yaw
    [-1.309,  0.698],  # l6: l_hand_pitch
    [-0.698,  0.698]   # l7: l_hand_roll
])

# 右臂关节限位矩阵 (7×2, rad)
# 说明：右臂7个关节的限位矩阵，与左臂对称
# 作用：用于限制随机生成的关节角范围，确保在机械限位内
RIGHT_ARM_JOINT_LIMIT = np.array([
    [-2.4,    0.6],    # r1: r_arm_pitch
    [-1.46,   0.349],  # r2: r_arm_roll (左右对称)
    [-0.46,   1.57],   # r3: r_arm_yaw (左右对称)
    [-2.618,  0.0],    # r4: r_forearm_pitch
    [-1.57,   1.57],   # r5: r_hand_yaw
    [-0.698,  1.309],  # r6: r_hand_pitch (左右对称)
    [-0.698,  0.698]   # r7: r_hand_roll
])

# 生成调试用关节角的数量
# 说明：生成的目标关节角组数（经过工作空间过滤后的点数）
NUM_DEBUG_PTS = 100

# 每个目标点之间的运动时间 (秒)
# 说明：控制运动速度，值越大运动越慢，轨迹越平滑
TIME_PER_POINT = 3.0

# 每段路径末尾的停顿时间 (秒)
# 说明：用于在关键点停留以便采集数据，确保机器人到位稳定后再采集
# 建议值：2-5秒
DWELL_TIME_PER_POINT = 3

# 选择生成和发布的手臂
# 说明：'left'（左臂）、'right'（右臂）、'both'（双臂）
ARM_SIDE = 'left'

# Mocap 话题配置（PoseStamped，单位默认米；如为毫米请在回调内调整）
MOCAP_TOPICS = {
    'base': '/mocap/base',
    'left': '/mocap/lefthand',
    'right': '/mocap/righthand',
}

# Mocap 坐标系到机器人 base_link 的静态变换（4×4），暂用单位阵，可替换为标定外参
T_MOCAP_TO_BASELINK = np.eye(4)

# 是否启用动捕采样（启动后探测3s决定）
USE_MOCAP_SAMPLING = False

# 初始手臂位置 (度)
# 说明：14个关节的初始角度，格式：[左7关节, 右7关节]
INIT_ARM_POS_DEG = [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]

# 发布频率（Hz）
# 说明：结合TIME_PER_POINT控制轨迹平滑程度
# 建议值：100Hz，保证轨迹连续性
PUBLISH_HZ = 100

# 右手工作空间边界（单位：米）
# 说明：基于biped_s45.xml定义，pos="0.45 -0.32 0.35", size="0.3 0.3 0.35"
# 作用：用于过滤超出工作空间的目标点，确保安全执行
RIGHT_ARM_WORKSPACE = {
    'x_min': 0.15, 'x_max': 0.75,  # 0.45 ± 0.3
    'y_min': -0.62, 'y_max': -0.02,  # -0.32 ± 0.3
    'z_min': 0.0, 'z_max': 0.7  # 0.35 ± 0.35
}

# 左手工作空间边界（单位：米）
# 说明：基于biped_s45.xml定义，pos="0.45 0.32 0.35", size="0.3 0.3 0.35"
# 作用：用于过滤超出工作空间的目标点，确保安全执行
LEFT_ARM_WORKSPACE = {
    'x_min': 0.15, 'x_max': 0.75,  # 0.45 ± 0.3
    'y_min': 0.02, 'y_max': 0.62,  # 0.32 ± 0.3
    'z_min': 0.0, 'z_max': 0.7  # 0.35 ± 0.35
}


# ==================== 全局变量 ====================
joint_state = JointState()
current_arm_joint_state = []
mocap_latest = {
    'base_mocap': None,          # PoseStamped in mocap frame
    'left_mocap': None,
    'right_mocap': None,
    'base_link_in_mocap': None,  # 4x4
    'left_in_base': None,        # 4x4
    'right_in_base': None,       # 4x4
}


# ==================== 回调函数 ====================

def sensors_data_callback(msg):
    """
    传感器数据回调函数,获取当前手臂关节状态
    """
    global current_arm_joint_state
    current_arm_joint_state = msg.joint_data.joint_q[12:26]
    current_arm_joint_state = [round(pos, 2) for pos in current_arm_joint_state]


def mocap_base_cb(msg):
    mocap_latest['base_mocap'] = msg
    update_mocap_transforms(mocap_latest, T_MOCAP_TO_BASELINK)


def mocap_left_cb(msg):
    mocap_latest['left_mocap'] = msg
    update_mocap_transforms(mocap_latest, T_MOCAP_TO_BASELINK)


def mocap_right_cb(msg):
    mocap_latest['right_mocap'] = msg
    update_mocap_transforms(mocap_latest, T_MOCAP_TO_BASELINK)



def is_mocap_available_for_arm(arm_side):
    """
    启动后检测动捕是否可用：等待回调填充 base/hand 后，检查对应 arm 的转换是否存在。
    """
    if arm_side == 'left':
        return mocap_latest.get('left_in_base') is not None
    if arm_side == 'right':
        return mocap_latest.get('right_in_base') is not None
    return (
        mocap_latest.get('left_in_base') is not None and
        mocap_latest.get('right_in_base') is not None
    )


def sample_and_save(arm_side, joints_file, fk_file, mocap_file, mocap_thresh_m=0.0002):
    """
    段末采样与保存（关节/FK/动捕），并检测动捕稳定性。
    - 先延迟1s，随后以100ms间隔采10帧关节角；同步抓取动捕末端位姿（base_link）。
    - 若动捕10帧的最大两两距离 > mocap_thresh_m（默认0.2mm），判定不稳定，当前段数据全部不保存。
    - 若动捕不可用（无数据），仅保存关节与FK。
    """
    global current_arm_joint_state
    if not current_arm_joint_state or len(current_arm_joint_state) < 14:
        rospy.logwarn("跳过采样：current_arm_joint_state 为空")
        return

    rospy.sleep(1.0)

    joint_samples = []
    mocap_samples = []
    if USE_MOCAP_SAMPLING:
        mocap_key = 'left_in_base' if arm_side == 'left' else ('right_in_base' if arm_side == 'right' else None)
    else:
        mocap_key = None

    for _ in range(10):
        if current_arm_joint_state and len(current_arm_joint_state) >= 14:
            joint_samples.append(current_arm_joint_state[:14])
        if mocap_key and mocap_latest.get(mocap_key) is not None:
            T = mocap_latest[mocap_key]
            mocap_samples.append(T[0:3, 3].copy())
        rospy.sleep(0.1)

    if len(joint_samples) == 0:
        rospy.logwarn("采样失败：未获取关节角样本")
        return

    # 检查动捕稳定性（如果有并且要求）
    mocap_mean = None
    if mocap_key:
        if len(mocap_samples) == 0:
            rospy.logwarn("动捕缺失，本段数据不保存")
            return
        mocap_arr = np.vstack(mocap_samples)
        diff = mocap_arr[:, None, :] - mocap_arr[None, :, :]
        max_pairwise = np.linalg.norm(diff, axis=-1).max()
        if max_pairwise > mocap_thresh_m:
            rospy.logwarn(
                f"动捕不稳定，最大位移 {max_pairwise*1000:.3f} mm > 0.2 mm，本段数据不保存"
            )
            return
        mocap_mean = mocap_arr.mean(axis=0)

    # 计算关节均值并落盘
    samples_np = np.array(joint_samples)
    avg_rad = np.mean(samples_np, axis=0)
    avg_deg = np.degrees(avg_rad)

    if arm_side == 'left':
        row = avg_deg[:7]
    elif arm_side == 'right':
        row = avg_deg[7:]
    else:
        row = avg_deg

    joints_path = os.path.join(get_config_dir(), joints_file)
    with open(joints_path, "a") as f:
        f.write(",".join([f"{v:.4f}" for v in row]) + "\n")
    rospy.loginfo(f"关节角保存: {joints_path}，均值(度)={np.round(row, 2)}")

    # FK 保存
    call_fk_and_save(avg_rad, arm_side, fk_file)

    # 动捕保存（若存在且稳定）
    if mocap_mean is not None and mocap_file:
        mocap_path = os.path.join(get_config_dir(), mocap_file)
        save_position_to_file(mocap_mean, mocap_path, append=True, precision=4)
        rospy.loginfo(f"动捕(base_link)保存: {mocap_path}，均值={np.round(mocap_mean, 4)}")


def call_fk_and_save(joint_angles_rad, arm_side, filename):
    """
    调用 /ik/fk_srv 计算末端位置，并将选定手的 pos_xyz 保存到文件。
    - joint_angles_rad: 长度14的rad列表
    """
    if joint_angles_rad is None or len(joint_angles_rad) < 14:
        rospy.logwarn("FK跳过：joint_angles_rad 无效")
        return

    left_pos, right_pos = call_fk_service_both_arms(joint_angles_rad)
    if left_pos is None or right_pos is None:
        rospy.logwarn("FK服务调用失败，无法保存末端位置")
        return

    # 选择手臂
    if arm_side == 'left':
        pos = left_pos
    elif arm_side == 'right':
        pos = right_pos
    else:
        pos = right_pos  # 默认右手

    out_path = os.path.join(get_config_dir(), filename)
    save_position_to_file(pos, out_path, append=True, precision=4)
    rospy.loginfo(f"末端位置保存: {out_path}，pos={np.round(pos, 4)}")


def save_mocap_in_base(arm_side, filename):
    """
    将动捕坐标转换到 base_link 后的末端位置保存到文件（与关节采样同节奏）。
    """
    key = 'left_in_base' if arm_side == 'left' else 'right_in_base'
    if mocap_latest.get(key) is None:
        return
    T = mocap_latest[key]
    pos = T[0:3, 3]
    out_path = os.path.join(get_config_dir(), filename)
    save_position_to_file(pos, out_path, append=True, precision=4)
    rospy.loginfo(f"动捕(base_link)位置保存: {out_path}，pos={np.round(pos, 4)}")


# ==================== 主函数 ====================

def main():
    """
    主函数: 生成随机关节角并通过本地贝塞尔插值发送到机器人
    """

    global USE_MOCAP_SAMPLING
    
    rospy.init_node('arm_calibration_joint_generator')
    
    # 仅订阅传感器数据（当前未使用，可扩展为起点跟随当前关节）
    rospy.Subscriber('/sensors_data_raw', sensorsData, sensors_data_callback, queue_size=1, tcp_nodelay=True)
    # 订阅动捕数据（PoseStamped，mocap坐标系）
    rospy.Subscriber(MOCAP_TOPICS['base'], PoseStamped, mocap_base_cb, queue_size=1, tcp_nodelay=True)
    rospy.Subscriber(MOCAP_TOPICS['left'], PoseStamped, mocap_left_cb, queue_size=1, tcp_nodelay=True)
    rospy.Subscriber(MOCAP_TOPICS['right'], PoseStamped, mocap_right_cb, queue_size=1, tcp_nodelay=True)
    kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=1, tcp_nodelay=True)

    # 探测动捕可用性（等待3s）
    rospy.loginfo(f"{COLOR_PURPLE}等待动捕数据 3s 用于可用性检测...{COLOR_RESET}")
    rospy.sleep(3.0)
    USE_MOCAP_SAMPLING = is_mocap_available_for_arm(ARM_SIDE)
    rospy.loginfo(f"{COLOR_PURPLE}动捕可用性: {USE_MOCAP_SAMPLING}（ARM_SIDE={ARM_SIDE}）{COLOR_RESET}")
    
    # 切换到手臂控制模式
    rospy.loginfo(f"{COLOR_PURPLE}切换手臂控制模式...{COLOR_RESET}")
    call_change_arm_ctrl_mode_service(2)
    rospy.sleep(0.5)
    
    # =============== 生成随机关节角 ===============
    rospy.loginfo(f"{COLOR_PURPLE}目标有效点数 = {NUM_DEBUG_PTS} ... (ARM_SIDE={ARM_SIDE}){COLOR_RESET}")
    
    try:
        filtered_targets = generate_filtered_joint_targets(
            num_points=NUM_DEBUG_PTS,
            left_limit=LEFT_ARM_JOINT_LIMIT,
            right_limit=RIGHT_ARM_JOINT_LIMIT,
            init_arm_pos_deg=INIT_ARM_POS_DEG,
            left_workspace=LEFT_ARM_WORKSPACE,
            right_workspace=RIGHT_ARM_WORKSPACE,
            arm_side=ARM_SIDE,
            margin_ratio=0.1,
            timeout_s=5.0,
            fk_service_callback=call_fk_service_both_arms,
        )
    except TimeoutError as e:
        rospy.logerr(str(e))
        return
    rospy.loginfo(f"{COLOR_PURPLE}工作空间过滤完成: {len(filtered_targets)}/{NUM_DEBUG_PTS} 个有效点（已截断为目标数量）{COLOR_RESET}")
    
    # =============== 本地关节空间贝塞尔插值 ===============
    rospy.loginfo("本地关节空间贝塞尔插值生成完整轨迹...")
    points_per_segment = max(5, int(TIME_PER_POINT * PUBLISH_HZ))
    full_traj_deg = build_jointspace_bezier_trajectory(
        filtered_targets,
        INIT_ARM_POS_DEG,
        TIME_PER_POINT,
        PUBLISH_HZ,
        num_points_per_segment=points_per_segment,
        dwell_time_per_point=0.0,  # 不在轨迹内插停顿
    )
    
    # 等待订阅者
    while kuavo_arm_traj_pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("等待 kuavo_arm_traj 订阅者连接...")
        rospy.sleep(0.5)
    
    # =============== 发布轨迹到humanoid_control ===============
    rospy.loginfo("开始发布轨迹到 /kuavo_arm_traj（本地插值）...")
    rate = rospy.Rate(PUBLISH_HZ)
    
    try:
        num_valid_targets = len(filtered_targets)
        segment_lengths = [points_per_segment] * num_valid_targets
        seg_idx = 0
        seg_step = 0
        seg_len = segment_lengths[seg_idx] if segment_lengths else 0

        # 采集结果文件名
        measure_file = {
            'left': 'measured_joints_left_arm.txt',
            'right': 'measured_joints_right_arm.txt',
            'both': 'measured_joints_both_arms.txt',
        }.get(ARM_SIDE, 'measured_joints.txt')
        fk_file = {
            'left': 'measured_fk_left_arm.txt',
            'right': 'measured_fk_right_arm.txt',
            'both': 'measured_fk_both_arms.txt',
        }.get(ARM_SIDE, 'measured_fk.txt')
        mocap_file = {
            'left': 'measured_mocap_left_arm.txt',
            'right': 'measured_mocap_right_arm.txt',
            'both': 'measured_mocap_both_arms.txt',
        }.get(ARM_SIDE, 'measured_mocap.txt')

        for i, q in enumerate(full_traj_deg):
            if rospy.is_shutdown():
                break

            # 每个目标段开始时打印一次
            if seg_step == 0 and seg_idx < len(filtered_targets):
                current_target = filtered_targets[seg_idx]
                if ARM_SIDE == 'left':
                    target_view = current_target[:7]
                elif ARM_SIDE == 'right':
                    target_view = current_target[7:]
                else:
                    target_view = current_target
                progress = i / float(len(full_traj_deg)) * 100.0 if len(full_traj_deg) > 0 else 0.0
                rospy.loginfo(
                    f"[轨迹进度] 段 {seg_idx + 1}/{num_valid_targets}, 段内步 {seg_step + 1}/{seg_len}, "
                    f"整体 {progress:.1f}%, 目标(度)={np.round(target_view, 2)}"
                )

            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = [
                "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch",
                "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
                "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch",
                "r_hand_yaw", "r_hand_pitch", "r_hand_roll",
            ]
            msg.position = q
            msg.velocity = [0.0] * 14
            msg.effort = [0.0] * 14
            kuavo_arm_traj_pub.publish(msg)

            # 段计数
            seg_step += 1
            finished_segment = seg_step >= seg_len
            last_segment = seg_idx >= len(segment_lengths) - 1

            # 段结束后执行采样保存（包括最后一段）
            if finished_segment:
                if DWELL_TIME_PER_POINT > 0:
                    rospy.sleep(DWELL_TIME_PER_POINT)
                sample_and_save(ARM_SIDE, measure_file, fk_file, mocap_file)

            if finished_segment and not last_segment:
                seg_idx += 1
                seg_step = 0
                seg_len = segment_lengths[seg_idx]

            rate.sleep()
        rospy.loginfo("轨迹发布完成")
    except KeyboardInterrupt:
        rospy.loginfo("用户中断,退出程序")


def test_single_point():
    """
    测试模式: 只运动到一个随机点（本地关节空间插值）
    """
    rospy.init_node('arm_calibration_test')
    
    # 创建发布者
    kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=1, tcp_nodelay=True)
    
    # 切换控制模式
    call_change_arm_ctrl_mode_service(2)
    rospy.sleep(0.5)
    
    # 生成单个测试点
    left_angles_rad, left_angles_deg = generate_joint_angles_with_margin(
        1,
        left_limit=LEFT_ARM_JOINT_LIMIT,
        right_limit=RIGHT_ARM_JOINT_LIMIT,
        margin_ratio=0.2,
        arm='left',
    )
    
    rospy.loginfo(f"测试目标关节角 (度): {left_angles_deg[0]}")
    
    # 合并成14关节
    target_pos = np.concatenate([left_angles_deg[0], np.array(INIT_ARM_POS_DEG[7:])])
    target_positions_deg = target_pos.reshape(1, -1)
    
    # 使用本地插值生成轨迹
    full_traj_deg = build_jointspace_bezier_trajectory(
        target_positions_deg.tolist(),
        INIT_ARM_POS_DEG,
        time_per_point=5.0,
        publish_hz=PUBLISH_HZ,
        num_points_per_segment=max(5, int(5.0 * PUBLISH_HZ)),
        dwell_time_per_point=0.0,  # 轨迹内不插停顿，停顿在段尾处理
    )
    
    # 发布
    while kuavo_arm_traj_pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    rate = rospy.Rate(100)
    for q in full_traj_deg:
        if rospy.is_shutdown():
            break
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [
            "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch",
            "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
            "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch",
            "r_hand_yaw", "r_hand_pitch", "r_hand_roll",
        ]
        msg.position = q
        msg.velocity = [0.0] * 14
        msg.effort = [0.0] * 14
        kuavo_arm_traj_pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == '--test':
        test_single_point()
    else:
        main()

