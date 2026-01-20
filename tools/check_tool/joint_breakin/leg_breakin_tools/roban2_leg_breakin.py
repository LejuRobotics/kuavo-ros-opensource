#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
from EcMasterConfig import EcMasterConfig

# 设置Python为无缓冲模式，确保实时输出
os.environ['PYTHONUNBUFFERED'] = '1'
sys.stdout.flush()
sys.stderr.flush()

# 检查是否有root权限
if os.geteuid() != 0:
    print("\033[31merror: 请使用root权限运行\033[0m")
    sys.exit(1)

# 获取当前脚本所在目录（而非工作目录）
script_dir = os.path.dirname(os.path.abspath(__file__))
relative_path = "build_lib/roban2"
target_path = os.path.join(script_dir, relative_path)
sys.path.append(target_path)

# print(sys.path)

import ec_master_wrap

def get_robot_version_from_bashrc():
    """从.bashrc文件中读取ROBOT_VERSION环境变量"""
    home_dir = os.path.expanduser('/home/lab/')
    bashrc_path = os.path.join(home_dir, '.bashrc')
    
    if os.path.exists(bashrc_path):
        with open(bashrc_path, 'r') as file:
            lines = file.readlines()
        for line in reversed(lines):
            line = line.strip()
            if line.startswith("export ROBOT_VERSION=") and "#" not in line:
                version = line.split("=")[1].strip()
                print(f"---------- 检测到 ROBOT_VERSION = {version} ----------")
                return version
    print("警告：ROBOT_VERSION 未找到或无效")
    return None

# 在创建EcMasterConfig之前，先设置环境变量
robot_version = get_robot_version_from_bashrc()
if robot_version:
    os.environ['ROBOT_VERSION'] = robot_version
    print(f"已设置环境变量 ROBOT_VERSION = {robot_version}")
else:
    print("错误：无法获取ROBOT_VERSION，程序退出")
    sys.exit(1)

g_EcMasterConfig = EcMasterConfig()

def menu():
    print("\n\033[1;34m====== EC_Master_tools ======\033[0m")
    print("\033[1;33m1\033[0m. 全身电机CSP正弦运动\033[1;31m（1943工厂专用）\033[0m")
    print("\033[1;33m2\033[0m. 电机磨线测试\033[1;31m（前6个电机往复运动）\033[0m")
    print("\033[1;33mq\033[0m. 退出程序")
    print("\033[1;34m-----------------------------\033[0m")

def option1():
    # 调用函数1：CSP正弦运动
    # A = float(input("请输入振幅: "))
    # T = float(input("请输入周期: "))
    # time_total = float(input("请输入总时间: "))
    print("正在执行EC通信的全部电机做CSP正弦运动...")
    success = ec_master_wrap.MotorCspSin(g_EcMasterConfig.slave_num, 5, 2, 10)
    if not success:
        print("\033[1;31m✘ 运动失败\033[0m")

def option2():
    # Roban2机器人腿部磨线测试：腰部电机1 + 左腿电机2-7 + 右腿电机8-13
    print("正在执行Roban2机器人腿部磨线测试...")
    sys.stdout.flush()
    
    # 获取用户输入的时间参数
    while True:
        try:
            time_input = input("请输入磨线运行时长（秒），最少15秒: ").strip()
            if time_input.lower() == 'q':
                print("已取消操作")
                return
            time_total = float(time_input)
            if time_total < 15:
                print("错误：运行时长必须至少15秒，请重新输入")
                continue
            break
        except (ValueError, EOFError):
            print("输入无效，请输入一个有效的数字")
            continue
    
    # Roban2机器人的动作序列配置
    # 腰部电机1的动作序列
    # waist_action = [0, 0, 0, 0, 0, 0, 0, 20, 0, -20, 0]
    waist_action = [0, 0, 0, 0, 0, 0, 0, 80, 0, -80, 0]
    
    # 左腿电机2-7的动作序列
    # left_leg_actions = [
    #     [0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 0],          # 电机2（左腿关节1）
    #     [0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 0],          # 电机3（左腿关节2）
    #     [0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 0],          # 电机4（左腿关节3）
    #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],            # 电机5（左腿关节4）
    #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],            # 电机6（左腿关节5）
    #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]             # 电机7（左腿关节6）
    # ]

    left_leg_actions = [
        [0, 30, 90, 0, -100, 20, 0, 0, 0, 0, 0],     # 电机2（左腿关节1）
        [0, 20, 60, 90, 0, 50, 0, 0, 0, 0, 0],       # 电机3（左腿关节2）
        [0, 15, -15, 0, 0, -25, 0, 0, 0, 0, 0],      # 电机4（左腿关节3）
        [0, 20, 100, 70, 0, 130, 0, 0, 0, 0, 0],     # 电机5（左腿关节4）
        [0, -15, 0, -15, 15, -15, 0, 0, 0, 0, 0],    # 电机6（左腿关节5）
        [0, 15, 0, -15, 15, -15, 0, 0, 0, 0, 0]      # 电机7（左腿关节6）
    ]
    
    # 构建完整的电机ID列表和动作序列
    motor_ids = []
    motor_actions = []
    
    # 腰部电机 (1) - 独立控制
    motor_ids.append(1)
    motor_actions.append(waist_action)
    
    # 添加左腿电机 (2-7)
    for i in range(6):  # 左腿电机2-7 (索引0-5)
        left_motor_id = i + 2  # 左腿电机ID (2-7)
        motor_ids.append(left_motor_id)
        motor_actions.append(left_leg_actions[i])
    
    # 添加右腿电机 (8-13) - 简单映射关系：2-8, 3-9, 4-10, 5-11, 6-12, 7-13
    for i in range(6):  # 左腿电机2-7 (索引0-5)
        left_motor_id = i + 2  # 左腿电机ID (2-7)
        right_motor_id = i + 8  # 右腿电机ID (8-13)
        
        # 根据电机ID决定是取反还是同向
        if left_motor_id <= 4:  # 电机2,3,4取反
            right_leg_action = [-action for action in left_leg_actions[i]]
        else:  # 电机5,6,7同向
            right_leg_action = left_leg_actions[i].copy()
        
        motor_ids.append(right_motor_id)
        motor_actions.append(right_leg_action)
    
    motion_duration = 1.27  # 每个动作持续时间
    
    print(f"电机ID: {motor_ids}")
    print(f"每个动作持续时间: {motion_duration}秒")
    print(f"总运行时间: {time_total}秒")
    sys.stdout.flush()
    
    success = ec_master_wrap.MotorMultiAction(motor_ids, motor_actions, motion_duration, time_total)
    if not success:
        print("\033[1;31m✘ 磨线运动失败\033[0m")
        sys.stdout.flush()
    else:
        print("\033[1;32m✓ 磨线运动完成\033[0m")
        sys.stdout.flush()

# 选项与函数的映射字典
FUNCTION_MAP = {
    '1': option1,
    '2': option2,
}

def main():
    for i in range(1, g_EcMasterConfig.slave_num+1):
        encoder_range = g_EcMasterConfig.get_encoder_range(i)
        if encoder_range is not None:
            ec_master_wrap.setEncoderRange(i, encoder_range)
    ec_master_wrap.set_command_args(g_EcMasterConfig.command_args)

    for slave_id, joint_id in g_EcMasterConfig.slave2joint.items():
        ec_master_wrap.setSlave2Joint(slave_id, joint_id)
    
    # 直接执行Roban2腿部磨线功能，不显示菜单
    option2()

if __name__ == "__main__":
    main()
