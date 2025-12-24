#!/usr/bin/env python3

import rospy
import argparse
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories
import numpy as np
from utils.sat import RotatingRectangle
from simple_turn_config import get_simple_config, list_simple_configs, validate_simple_config


def get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj):
    """创建并返回一个 footPoseTargetTrajectories 消息对象。"""
    num = len(time_traj)
    msg = footPoseTargetTrajectories()
    msg.timeTrajectory = time_traj
    msg.footIndexTrajectory = foot_idx_traj
    msg.footPoseTrajectory = []

    for i in range(num):
        foot_pose_msg = footPose()
        foot_pose_msg.footPose = foot_traj[i]
        foot_pose_msg.torsoPose = torso_traj[i]
        msg.footPoseTrajectory.append(foot_pose_msg)

    return msg


def generate_steps(torso_pos, torso_yaw, foot_bias):
    """根据躯干位置和偏航角生成左右脚的位置。"""
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


def simple_turn_msg(config):
    """
    生成简单两步转向消息。
    
    参数：
    - config: 配置字典，包含angle, dt, is_left_first, torso_height
    
    返回：
    - footPoseTargetTrajectories 消息对象
    """
    turn_angle_deg = config['angle']
    dt = config['dt']
    is_left_first = config.get('is_left_first', None)
    torso_height = config['torso_height']
    
    # 自动判断先踏哪只脚：顺时针（正数）时左脚先，逆时针（负数）时右脚先
    if is_left_first is None:
        is_left_first = turn_angle_deg > 0  # 顺时针时左脚先
    
    # 验证配置
    is_valid, message = validate_simple_config(turn_angle_deg, dt)
    if not is_valid:
        print(f"\033[91m[Error] {message}\033[0m")
        return None
    
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


def execute_simple_turn_from_config(config):
    """
    根据配置执行简单转向。
    
    参数：
    - config: 配置字典
    """
    # 初始化 ROS 节点
    if not rospy.get_node_uri():
        rospy.init_node('advanced_simple_turn_controller', anonymous=True)

    # 创建发布者
    pub = rospy.Publisher('/humanoid_mpc_foot_pose_target_trajectories', footPoseTargetTrajectories, queue_size=10)

    # 等待订阅者准备好
    rospy.sleep(1)

    print(f"\033[94m=== Advanced Simple Turn Controller ===\033[0m")
    if 'description' in config:
        print(f"\033[94mDescription: {config['description']}\033[0m")
    print(f"\033[94mTurn angle: {config['angle']}°\033[0m")
    print(f"\033[94mStep sequence: {'Left->Right' if config['is_left_first'] else 'Right->Left'}\033[0m")
    print(f"\033[94mTime per step: {config['dt']}s\033[0m")

    # 生成并发布步态消息
    msg = simple_turn_msg(config)
    
    if msg is not None:
        pub.publish(msg)
        print(f"\033[92m[Success] Published simple turn trajectory\033[0m")
        print(f"\033[92m[Info] Robot will turn {config['angle']}° in 2 steps\033[0m")
        return True
    else:
        print(f"\033[91m[Error] Failed to generate turn trajectory\033[0m")
        return False


def main():
    """主函数：处理命令行参数并执行转向。"""
    parser = argparse.ArgumentParser(
        description='Advanced Simple Two-Step Turn Controller',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # 使用预设配置
  python advanced_simple_turn.py --config turn_30
  python advanced_simple_turn.py --config turn_left_45
  
  # 自定义转向
  python advanced_simple_turn.py --angle 30
  python advanced_simple_turn.py --angle -45 --time 0.6 --right-first
  
  # 列出所有预设配置
  python advanced_simple_turn.py --list
        """
    )
    
    parser.add_argument('--config', '-c', type=str,
                       help='Use predefined configuration')
    parser.add_argument('--angle', '-a', type=float,
                       help='Turn angle in degrees (positive=clockwise, negative=counter-clockwise)')
    parser.add_argument('--time', '-t', type=float, default=0.4,
                       help='Time per step in seconds (default: 0.4)')
    parser.add_argument('--right-first', action='store_true',
                       help='Force right foot to start first (default: auto-select based on direction)')
    parser.add_argument('--left-first', action='store_true',
                       help='Force left foot to start first (default: auto-select based on direction)')
    parser.add_argument('--height', type=float, default=0.0,
                       help='Torso height (default: 0.0)')
    parser.add_argument('--list', '-l', action='store_true',
                       help='List all available configurations')
    
    args = parser.parse_args()
    
    # 检查冲突参数
    if args.right_first and args.left_first:
        print(f"\033[91m[Error] Cannot specify both --right-first and --left-first\033[0m")
        return
    
    # 如果请求列出配置
    if args.list:
        list_simple_configs()
        return
    
    config = None
    
    # 优先使用预设配置
    if args.config:
        config = get_simple_config(args.config)
        if config is None:
            print(f"\033[91m[Error] Configuration '{args.config}' not found\033[0m")
            print("\033[93mAvailable configurations:\033[0m")
            list_simple_configs()
            return
        print(f"\033[92m[Info] Using predefined configuration: {args.config}\033[0m")
    
    # 如果没有预设配置，使用命令行参数创建配置
    elif args.angle is not None:
        # 确定踏步顺序
        if args.right_first:
            is_left_first = False
        elif args.left_first:
            is_left_first = True
        else:
            is_left_first = None  # 自动判断
        
        config = {
            'angle': args.angle,
            'dt': args.time,
            'is_left_first': is_left_first,
            'torso_height': args.height,
            'description': f"自定义{args.angle}°转向"
        }
        print(f"\033[92m[Info] Using custom configuration\033[0m")
    
    # 如果没有指定任何配置，使用默认的30度转向
    else:
        print("\033[93m[Warning] No configuration specified, using default 30° turn\033[0m")
        config = get_simple_config("turn_30")
    
    # 应用命令行参数覆盖
    if args.time != 0.4:
        config["dt"] = args.time
    if args.right_first:
        config["is_left_first"] = False
    elif args.left_first:
        config["is_left_first"] = True
    if args.height != 0.0:
        config["torso_height"] = args.height
    
    # 显示自动选择的踏步顺序
    if config.get('is_left_first', None) is None and 'angle' in config:
        auto_foot = "Left" if config['angle'] > 0 else "Right"
        print(f"\033[96m[Auto] {auto_foot} foot will step first ({'clockwise' if config['angle'] > 0 else 'counter-clockwise'} turn)\033[0m")
    
    # 执行转向
    try:
        success = execute_simple_turn_from_config(config)
        if success:
            print(f"\033[92m[Info] Turn command sent successfully!\033[0m")
            print(f"\033[94m[Info] Expected completion time: {2 * config['dt']}s\033[0m")
        else:
            print(f"\033[91m[Error] Failed to execute turn\033[0m")
            
    except rospy.ROSInterruptException:
        print("\033[91m[Info] Program interrupted\033[0m")
    except KeyboardInterrupt:
        print("\033[91m[Info] Program interrupted by user\033[0m")
    except Exception as e:
        print(f"\033[91m[Error] {str(e)}\033[0m")


if __name__ == '__main__':
    main()