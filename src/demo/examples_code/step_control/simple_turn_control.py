#!/usr/bin/env python3

import rospy
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories  # 导入自定义消息类型
import numpy as np
from utils.sat import RotatingRectangle  # 导入用于碰撞检测的工具类


def get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj):
    """
    创建并返回一个 footPoseTargetTrajectories 消息对象。
    
    参数：
    - time_traj: 时间轨迹列表
    - foot_idx_traj: 脚索引轨迹列表
    - foot_traj: 脚姿态轨迹列表
    - torso_traj: 躯干姿态轨迹列表
    
    返回：
    - footPoseTargetTrajectories 消息对象
    """
    num = len(time_traj)
    msg = footPoseTargetTrajectories()  # 创建消息实例
    msg.timeTrajectory = time_traj  # 设置时间轨迹
    msg.footIndexTrajectory = foot_idx_traj  # 设置脚索引
    msg.footPoseTrajectory = []  # 初始化脚姿态轨迹

    for i in range(num):
        foot_pose_msg = footPose()  # 创建脚姿态信息
        foot_pose_msg.footPose = foot_traj[i]  # 设置脚姿态
        foot_pose_msg.torsoPose = torso_traj[i]  # 设置躯干姿态
        msg.footPoseTrajectory.append(foot_pose_msg)  # 将脚姿态添加到消息中

    return msg


def generate_steps(torso_pos, torso_yaw, foot_bias):
    """
    根据躯干位置和偏航角生成左右脚的位置。
    
    参数：
    - torso_pos: 躯干位置
    - torso_yaw: 躯干偏航角（弧度）
    - foot_bias: 脚偏移量
    
    返回：
    - 左脚和右脚的位置
    """
    l_foot_bias = np.array([0, foot_bias, -torso_pos[2]])
    r_foot_bias = np.array([0, -foot_bias, -torso_pos[2]])
    R_z = np.array([
        [np.cos(torso_yaw), -np.sin(torso_yaw), 0],
        [np.sin(torso_yaw), np.cos(torso_yaw), 0],
        [0, 0, 1]
    ])
    l_foot = torso_pos + R_z.dot(l_foot_bias)
    r_foot = torso_pos + R_z.dot(r_foot_bias)
    return l_foot, r_foot


def simple_turn_msg(turn_angle_deg, dt=0.4, is_left_first=None, torso_height=0.0):
    """
    生成简单两步转向消息：左脚一步，右脚一步完成转向。
    
    参数：
    - turn_angle_deg: 转向角度（度），正数为顺时针，负数为逆时针
    - dt: 每步时间间隔（秒）
    - is_left_first: 是否左脚先踏步（None时自动判断：顺时针左脚先，逆时针右脚先）
    - torso_height: 躯干高度
    
    返回：
    - footPoseTargetTrajectories 消息对象
    """
    # 自动判断先踏哪只脚：顺时针（正数）时左脚先，逆时针（负数）时右脚先
    if is_left_first is None:
        is_left_first = turn_angle_deg > 0  # 顺时针时左脚先
    
    # 将角度转换为弧度
    turn_angle_rad = np.radians(turn_angle_deg)
    
    # 定义身体姿态：只有一个目标位置，转向指定角度
    body_pose = [0.0, 0.0, torso_height, turn_angle_deg]
    
    # 固定两步：左脚一步，右脚一步
    time_traj = [dt, 2 * dt]  # 时间轨迹
    foot_idx_traj = []  # 脚索引轨迹
    foot_traj = []  # 脚姿态轨迹
    torso_traj = []  # 躯干姿态轨迹
    
    # 生成目标位置的左右脚位置
    torso_pos = np.asarray(body_pose[:3])  # 躯干位置
    torso_yaw = turn_angle_rad  # 躯干偏航角（弧度）
    
    l_foot, r_foot = generate_steps(torso_pos, torso_yaw, 0.1)  # 生成左右脚位置
    l_foot = [*l_foot[:3], torso_yaw]  # 左脚姿态
    r_foot = [*r_foot[:3], torso_yaw]  # 右脚姿态
    
    # 第一步和第二步
    if is_left_first:
        # 第一步：左脚
        foot_idx_traj.append(0)  # 左脚
        foot_traj.append(l_foot)
        torso_traj.append([*body_pose[:3], torso_yaw])
        
        # 第二步：右脚
        foot_idx_traj.append(1)  # 右脚
        foot_traj.append(r_foot)
        torso_traj.append([*body_pose[:3], torso_yaw])
    else:
        # 第一步：右脚
        foot_idx_traj.append(1)  # 右脚
        foot_traj.append(r_foot)
        torso_traj.append([*body_pose[:3], torso_yaw])
        
        # 第二步：左脚
        foot_idx_traj.append(0)  # 左脚
        foot_traj.append(l_foot)
        torso_traj.append([*body_pose[:3], torso_yaw])
    
    print(f"\033[92m[Info] Simple turn: {turn_angle_deg}° ({'Clockwise' if turn_angle_deg > 0 else 'Counter-clockwise'})\033[0m")
    print(f"\033[92m[Info] Two steps: {'Left->Right' if is_left_first else 'Right->Left'}\033[0m")
    print(f"\033[92m[Info] Time per step: {dt}s, Total time: {2*dt}s\033[0m")
    
    return get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj)


def execute_simple_turn(turn_angle_deg, dt=0.4, is_left_first=None, torso_height=0.0):
    """
    执行简单的两步转向。
    
    参数：
    - turn_angle_deg: 转向角度（度），正数为顺时针，负数为逆时针
    - dt: 每步时间间隔（秒）
    - is_left_first: 是否左脚先踏步（None时自动判断：顺时针左脚先，逆时针右脚先）
    - torso_height: 躯干高度
    """
    # 初始化 ROS 节点
    rospy.init_node('simple_turn_controller', anonymous=True)

    # 创建发布者，话题为 /humanoid_mpc_foot_pose_target_trajectories
    pub = rospy.Publisher('/humanoid_mpc_foot_pose_target_trajectories', footPoseTargetTrajectories, queue_size=10)

    # 等待一定时间以确保订阅者已经准备好
    rospy.sleep(1)

    print(f"\033[94m=== Simple Turn Controller ===\033[0m")
    print(f"\033[94mTurn angle: {turn_angle_deg}°\033[0m")
    print(f"\033[94mStep sequence: {'Left->Right' if is_left_first else 'Right->Left'}\033[0m")
    print(f"\033[94mTime per step: {dt}s\033[0m")

    # 生成步态消息并发布
    msg = simple_turn_msg(turn_angle_deg, dt, is_left_first, torso_height)
    
    if msg is not None:
        pub.publish(msg)
        print(f"\033[92m[Success] Published simple turn trajectory\033[0m")
        print(f"\033[92m[Info] Robot will turn {turn_angle_deg}° in 2 steps\033[0m")
    else:
        print(f"\033[91m[Error] Failed to generate turn trajectory\033[0m")

    return msg


if __name__ == '__main__':
    import argparse
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(
        description='Simple Two-Step Turn Controller',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # 30度顺时针转向
  python simple_turn_control.py --angle 30
  
  # 45度逆时针转向  
  python simple_turn_control.py --angle -45
  
  # 90度转向，右脚先踏步，慢速
  python simple_turn_control.py --angle 90 --right-first --time 0.6
        """
    )
    
    parser.add_argument('--angle', '-a', type=float, default=30,
                       help='Turn angle in degrees (positive=clockwise, negative=counter-clockwise, default: 30)')
    parser.add_argument('--time', '-t', type=float, default=0.4,
                       help='Time per step in seconds (default: 0.4)')
    parser.add_argument('--right-first', action='store_true',
                       help='Force right foot to start first (default: auto-select based on direction)')
    parser.add_argument('--left-first', action='store_true',
                       help='Force left foot to start first (default: auto-select based on direction)')
    parser.add_argument('--height', type=float, default=0.0,
                       help='Torso height (default: 0.0)')
    
    args = parser.parse_args()
    
    # 检查冲突参数
    if args.right_first and args.left_first:
        print(f"\033[91m[Error] Cannot specify both --right-first and --left-first\033[0m")
        exit(1)
    
    # 确定踏步顺序：如果用户指定了，使用用户指定；否则自动判断
    if args.right_first:
        is_left_first = False
    elif args.left_first:
        is_left_first = True
    else:
        is_left_first = None  # 自动判断
    
    # 参数验证
    if abs(args.angle) > 90:
        print(f"\033[93m[Warning] Large angle ({args.angle}°) may be unstable for two-step turn\033[0m")
        confirm = input("Continue? (y/N): ").lower().strip()
        if confirm != 'y':
            print("Cancelled.")
            exit(0)
    
    if args.time < 0.2 or args.time > 1.0:
        print(f"\033[93m[Warning] Time per step ({args.time}s) is outside recommended range (0.2-1.0s)\033[0m")
    
    # 显示自动选择的踏步顺序
    if is_left_first is None:
        auto_foot = "Left" if args.angle > 0 else "Right"
        print(f"\033[96m[Auto] {auto_foot} foot will step first ({'clockwise' if args.angle > 0 else 'counter-clockwise'} turn)\033[0m")
    
    try:
        # 执行简单转向
        execute_simple_turn(
            turn_angle_deg=args.angle,
            dt=args.time,
            is_left_first=is_left_first,
            torso_height=args.height
        )
        
        print(f"\033[92m[Info] Turn command sent successfully!\033[0m")
        print(f"\033[94m[Info] Expected completion time: {2 * args.time}s\033[0m")
        
    except rospy.ROSInterruptException:
        print("\033[91m[Info] Program interrupted\033[0m")
    except KeyboardInterrupt:
        print("\033[91m[Info] Program interrupted by user\033[0m")
    except Exception as e:
        print(f"\033[91m[Error] {str(e)}\033[0m")