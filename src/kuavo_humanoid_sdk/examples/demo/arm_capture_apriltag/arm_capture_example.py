#!/usr/bin/env python3
import time 
import math
import numpy as np  # 引入numpy库用于数值计算
from enum import Enum, auto
import argparse
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState, KuavoRobotTools, KuavoRobotVision, KuavoRobotInfo, DexterousHand
from kuavo_humanoid_sdk.interfaces.data_types import AprilTagData, PoseQuaternion
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, KuavoManipulationMpcFrame
# 先不使用策略模块,直接通过SDK实现
from kuavo_humanoid_sdk.common.logger import SDKLogger

############################## 角度计算工具函数 #######################################

# 角度转弧度
def degrees_to_radians(degrees_list):
    return [math.radians(angle) for angle in degrees_list]

# 通过角度（弧度制）计算四元数
class Quaternion:
    def __init__(self):
        self.w = 0    
        self.x = 0    
        self.y = 0     
        self.z = 0

# yaw (Z), pitch (Y), roll (X)
# 欧拉角(Z-Y-X顺序) → 旋转矩阵 → 四元数
def euler_to_rotation_matrix(yaw_adaptive=0, pitch_adaptive=0, roll_adaptive=0,
                            yaw_manual=0, pitch_manual=0, roll_manual=0):
    """
    欧拉角(Z-Y-X顺序) → 旋转矩阵
    参数:
        yaw (float):   绕Z轴旋转角度（弧度）
        pitch (float): 绕Y轴旋转角度（弧度）
        roll (float):  绕X轴旋转角度（弧度）
    返回:
        np.ndarray: 3x3旋转矩阵
    """
    # 计算三角函数值
    cy, sy = np.cos(yaw_adaptive), np.sin(yaw_adaptive)
    cp, sp = np.cos(pitch_adaptive), np.sin(pitch_adaptive)
    
    R = np.array([
        [cy * cp,   -sy,        cy * sp],
        [sy * cp,    cy,        sy * sp],
        [-sp,        0,         cp     ]
    ])

    # 存在自定义参数 需要二次旋转
    if yaw_manual or pitch_manual or roll_manual:

        # 初始化为单位矩阵
        R_manual = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        if abs(yaw_manual) > 0.01:
            print("yaw_manual=",yaw_manual)
            c, s = np.cos(yaw_manual), np.sin(yaw_manual)
            R_manual = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ R_manual

        if abs(pitch_manual) > 0.01:
            print("pitch_manual=",pitch_manual)
            c, s = np.cos(pitch_manual), np.sin(pitch_manual)
            R_manual = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) @ R_manual

        if abs(roll_manual) > 0.01:
            print("roll_manual=",roll_manual)
            c, s = np.cos(roll_manual), np.sin(roll_manual)
            R_manual = np.array([[1, 0, 0], [0, c, -s], [0, s, c]]) @ R_manual

        return R @ R_manual
    # 不存在自定义参数,直接输出旋转矩阵
    else :
        return R

def rotation_matrix_to_quaternion(R):
    """
    旋转矩阵 → 四元数
    参数:
        R (np.ndarray): 3x3旋转矩阵
    返回:
        np.ndarray: 四元数 [x, y, z, w]
    """
    # 计算四元数分量
    trace = np.trace(R)

    q = Quaternion()

    if trace > 0:
        q.w = math.sqrt(trace + 1.0) / 2
        q.x = (R[2, 1] - R[1, 2]) / (4 * q.w)
        q.y = (R[0, 2] - R[2, 0]) / (4 * q.w)
        q.z = (R[1, 0] - R[0, 1]) / (4 * q.w)
    else:
        # 处理w接近零的情况
        i = np.argmax([R[0, 0], R[1, 1], R[2, 2]])
        j = (i + 1) % 3
        k = (j + 1) % 3
        t = np.zeros(4)
        t[i] = math.sqrt(R[i, i] - R[j, j] - R[k, k] + 1) / 2
        t[j] = (R[i, j] + R[j, i]) / (4 * t[i])
        t[k] = (R[i, k] + R[k, i]) / (4 * t[i])
        t[3] = (R[k, j] - R[j, k]) / (4 * t[i])

        q.x, q.y, q.z, q.w = t  # 重排序为[x, y, z, w]

    # 归一化（防止数值误差）
    norm = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
    if norm > 0:
        q.w /= norm
        q.x /= norm
        q.y /= norm
        q.z /= norm
    return q

def euler_to_quaternion_via_matrix(yaw_adaptive=0, pitch_adaptive=0, roll_adaptive=0,
                                    yaw_manual=0, pitch_manual=0, roll_manual=0):
    """
    欧拉角 → 旋转矩阵 → 四元数
    参数:
        yaw (float):   绕Z轴旋转角度(弧度)
        pitch (float): 绕Y轴旋转角度(弧度)
        roll (float):  绕X轴旋转角度(弧度)
    返回:
        np.ndarray: 四元数 [x, y, z, w]
    """
    R = euler_to_rotation_matrix(yaw_adaptive, pitch_adaptive, roll_adaptive,
                                yaw_manual, pitch_manual, roll_manual)
    return rotation_matrix_to_quaternion(R)

############################## 手臂控制工具函数 #######################################

def control_arm_traj(robot , q0 , q1 , duration):
    """
    控制机械臂从起始关节角度平滑移动到目标关节角度

    参数:
        robot: 机器人控制对象
        q0: 起始关节角度列表 (弧度)
        q1: 目标关节角度列表 (弧度)
        duration: 运动持续时间 (秒)
    """
    q_list = []
    interval = 0.1  # 控制间隔(秒)
    steps = int(duration / interval)  # 计算整数步数
    for i in range(steps):
        q_tmp = [0.0]*14
        for j in range(14):
            q_tmp[j] = q0[j] + i/float(steps)*(q1[j] - q0[j])
        q_list.append(q_tmp)
    for q in q_list:
        # !!! Convert degrees to radians
        robot.control_arm_joint_positions(q)
        time.sleep(0.1)
    #robot.set_fixed_arm_mode()
    #time.sleep(1.0)

def execute_arm_trajectory(robot, robot_state, target_poses):
    """
    执行手臂轨迹列表
    
    参数:
        robot: 机器人控制对象
        robot_state: 机器人状态对象
        target_poses: 目标位姿序列 [[duration_s, [joint_angles_deg]], ...]
    """
    for pose in target_poses:
        duration, angles_deg = pose
        curr_q = robot_state.arm_joint_state().position
        target_rad = degrees_to_radians(angles_deg)
        control_arm_traj(robot, curr_q, target_rad, duration)
        time.sleep(1)  # 等待当前动作完成

############################## 手部控制工具函数 #######################################

def control_dex_hand(dex_hand, hand: str, pose: str):
    """
    控制灵巧手姿态
    :param dex_hand: DexterousHand 对象
    :param hand: 'left' 或 'right'
    :param pose: 'open'、'close'、'zero'
    """
    # 预定义角度组
    poses = {
        "open":  [0, 100, 0, 0, 0, 0],
        "close": [80, 80, 80, 80, 80, 80],
        "zero":  [0, 0, 0, 0, 0, 0]
    }

    if pose not in poses:
        raise ValueError(f"未知的姿态名称: {pose}, 可选: {list(poses.keys())}")

    angles = poses[pose]

    if hand == "left":
        dex_hand.control_left(angles)
    elif hand == "right":
        dex_hand.control_right(angles)
    else:
        raise ValueError("hand 参数必须是 'left' 或 'right'")

    # 给手一点执行时间
    time.sleep(0.5)

############################## 主函数 #######################################

def main():
# ############################## 初始化SDK ####################################
    if not KuavoSDK.Init(options=KuavoSDK.Options.Normal, log_level="DEBUG"):
        print("Failed to initialize Kuavo SDK")
        return
    print("初始化机器人...")
   
    # 初始化机器人及相关组件
    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    robot_vision = KuavoRobotVision()
    robot_info = KuavoRobotInfo()
    # 初始化灵巧手
    dex_hand = DexterousHand()

    # Stance
    robot.stance()
    if not robot_state.wait_for_stance(timeout=100.0):
        print("change to stance fail!")
        return
    print("Robot is in stance state")
    
# ############################## 输入参数获取 ####################################

    parser = argparse.ArgumentParser(description="是否启用偏移量")
    parser.add_argument("--offset_start", type=str, choices=["False", "True"], 
                        default="False", help="选择 offset_start = True or Flase (默认: False)")
    args = parser.parse_args()
    # offset_start="True"表示启用偏移量 否则不启用偏移量
    if args.offset_start == "True":
        offset_z=-0.10  # 抓取点位于标签正下方
        temp_x_l=-0.035  # 偏向侧后边一点
        temp_y_l=0.035
        temp_x_r=-0.045  # 偏向侧后边一点
        temp_y_r=0.035
    else :
        offset_z, temp_x_l, temp_y_l, temp_x_r, temp_y_r = 0.00, 0.00, 0.00, 0.00, 0.00
    # 角度偏移量（修正绕z轴的偏移角度,默认为不变）
    offset_angle=1.00

# ############################## 预定义参数获取 ####################################

    print("Robot Version:", robot_info.robot_version)
    robot_version = robot_info.robot_version
    #不同型号机器人的初始位置 (机器人坐标系)
    if robot_version in (45, 48, 49):
        robot_zero_x, robot_zero_y, robot_zero_z = -0.0173, -0.2927, -0.2837
    elif robot_version == 42:
        robot_zero_x, robot_zero_y, robot_zero_z = -0.0175, -0.25886, -0.20115
    else :
        print("机器人版本号错误, 仅支持42 45 48 49")
        return

# ############################## Apriltag二维码位置获取 #################################
    # 低头
    robot.control_head(yaw=0, pitch=20)
    # 获取指定 tag=0 的数据
    time.sleep(0.1)
    print("tag 0 data:")
    data = robot_vision.get_data_by_id(0, "base")
    print(data)
    if data and 'poses' in data and len(data['poses']) > 0:
        detection = data['poses'][0]
        position = [detection.position.x, detection.position.y, detection.position.z]
        print("Position (x, y, z):", position)
    else:
        print("No valid data for tag 0.")
        return
    
# ############################## 抓取位姿设定(右手) ####################################

    set_x=position[0] + temp_x_r
    set_y=position[1] - temp_y_r
    set_z=position[2] + offset_z
    print("抓取点x y z:",  set_x ," , ", set_y ," , ", set_z )
    relative_angle=math.atan((set_y-robot_zero_y)/(set_x-robot_zero_x))
    quat=euler_to_quaternion_via_matrix(relative_angle*offset_angle, -1.57 , 0)
    print(f"relative_angle: {relative_angle}")

# ############################## ik参数赋值 #######################################

    left_front_position = [0.45,0.25,0.11988012 ]#+0.813]   # Left arm front position
    left_front_orientation = [0.0,-0.70682518,0.0,0.70738827]
    # right_front_position = [0.45,-0.25,0.11988012 ]#+0.813]  # Right arm front position
    # right_front_orientation = [0.0,-0.70682518,0.0,0.70738827]
    right_front_position = np.array([set_x,set_y,set_z])
    right_front_orientation = [quat.x,quat.y,quat.z,quat.w]

# ############################## ik求解 #######################################

    res = robot.arm_ik(
        KuavoPose(position=left_front_position, orientation=left_front_orientation),
        KuavoPose(position=right_front_position, orientation=right_front_orientation)
    )
    if res:
        # 执行 ik 结果
        curr_q = robot_state.arm_joint_state().position
        # 只执行右手的值
        new_res = tuple(curr_q[:7]) + res[7:]
        print("curr_q:",curr_q)
        print("修改后的 res:", new_res)

# ############################## 准备姿态 #######################################
        
        control_dex_hand(dex_hand, "right", "zero") # 右手归零
        
        target_poses = [
            [2.5, [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                    20.0, -60.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
            [2.5, [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                    -10.0, -60.0, 0.0, -90.0, 35.0, 20.0, 0.0]],
        ]
        execute_arm_trajectory(robot, robot_state, target_poses)
        
        control_dex_hand(dex_hand, "right", "open") # 张开右手
# ############################## ik结果执行 ####################################
        
        curr_q = robot_state.arm_joint_state().position
        control_arm_traj(robot , curr_q , new_res, 5)
        time.sleep(2)
        control_dex_hand(dex_hand, "right", "close") # 闭合右手
        time.sleep(4)
        # 通过 fk 评估结果
        curr_q = robot_state.arm_joint_state().position
        l_pose, r_pose = robot.arm_fk(curr_q)
        print(l_pose)
        print(r_pose)
        
# ############################## 递水动作 #######################################

        target_poses = [
            [2, [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                    -60.0, 0.0, 0.0, -30.0, 20.0, 0.0, 0.0]],
        ]
        execute_arm_trajectory(robot, robot_state, target_poses)

        control_dex_hand(dex_hand, "right", "open")  # 张开右手

# ############################## 回到初始位置 #####################################
        
        target_poses = [
            [2.5, [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                    -10.0, -60.0, 0.0, -90.0, 35.0, 20.0, 0.0]],
            [2.5, [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                    20.0, -60.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
        ]
        execute_arm_trajectory(robot, robot_state, target_poses)
        
        robot.control_head(yaw=0, pitch=0) # 抬头
        control_dex_hand(dex_hand, "right", "zero") # 右手归零
        
# ############################## 手臂复位 #######################################
    robot.manipulation_mpc_reset()
    robot.arm_reset()
    time.sleep(1)

if __name__ == "__main__":
    main() 

