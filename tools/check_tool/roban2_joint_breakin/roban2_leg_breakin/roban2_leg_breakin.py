#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import threading
import time
import signal
from EcMasterConfig import EcMasterConfig

if os.geteuid() != 0:
    print("\033[31merror: 请使用root权限运行\033[0m")
    sys.exit(1)

script_dir = os.path.dirname(os.path.abspath(__file__))
relative_path = "build"
target_path = os.path.join(script_dir, relative_path)
sys.path.append(target_path)

import leg_breakin
g_EcMasterConfig = EcMasterConfig()

early_exit_requested = False
exit_after_cycle = False

def listen_for_exit():
    """监听用户输入，检测是否按了'q'键"""
    global early_exit_requested, exit_after_cycle
    while not early_exit_requested:
        try:
            user_input = input()
            if user_input.strip().lower() == 'q':
                print(f"\033[92m用户请求提前结束程序，将在当前完整动作周期完成后停止。\033[0m")
                early_exit_requested = True
                exit_after_cycle = True
                break
        except (EOFError, KeyboardInterrupt):
            break

def option_leg_breakin():
    print("\033[1;34m=== Roban2腿部磨线 ===\033[0m")
    print("\033[1;33m=== 请将机器人吊起，将电机摆正到零点，包括腰部（大腿电机则直接自然垂直向下） ===\033[0m \n")
    
    left_leg_actions = [
        [0, 0, 0, 0, 0, 0, 0, 80, 0, -80, 0],       # 电机1（腰部）
        [0, 30, 90, 0, -100, 20, 0, 0, 0, 0, 0],    # 电机2（左腿关节1）
        [0, 20, 60, 90, 0, 50, 0, 0, 0, 0, 0],       # 电机3（左腿关节2）
        [0, 15, -15, 0, 0, -25, 0, 0, 0, 0, 0],     # 电机4（左腿关节3）
        [0, 20, 100, 70, 0, 130, 0, 0, 0, 0, 0],    # 电机5（左腿关节4）
        [0, -15, 0, -15, 15, -15, 0, 0, 0, 0, 0],    # 电机6（左腿关节5）
        [0, 15, 0, -15, 15, -15, 0, 0, 0, 0, 0]     # 电机7（左腿关节6）
    ]
    
    motor_actions = left_leg_actions.copy()
    
    right_leg_actions = []
    for i in range(1, 7):  # 左腿电机2-7 (索引1-6)
        left_motor_id = i + 1  # 左腿电机ID (2-7)
        right_motor_id = 15 - left_motor_id  # 右腿电机ID (13-8)
        left_leg_joint = i  # 左腿关节编号 (1-6)
        right_leg_joint = 7 - i  # 右腿关节编号 (6-1)
        
        # 根据电机ID决定是取反还是同向
        if left_motor_id <= 4:  # 电机2,3,4取反
            right_leg_action = [-action for action in left_leg_actions[i]]
            motion_type = "取反"
        else:  # 电机5,6,7同向
            right_leg_action = left_leg_actions[i].copy()
            motion_type = "同向"
        
        right_leg_actions.append(right_leg_action)
    
    for i in range(6):
        left_motor_index = 5 - i
        right_motor_id = 8 + i
        motor_actions.append(right_leg_actions[left_motor_index])
    motor_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
    motion_duration = 1.27
    
    cycle_time = len(left_leg_actions[0]) * motion_duration
    
    while True:
        try:
            total_time_input = input(f"请输入总运行时间（秒），最少{cycle_time}秒: ").strip()
            if total_time_input.lower() == 'q':
                print("已取消操作")
                return
            total_time = float(total_time_input)
            if total_time <= 0:
                print("总运行时间必须大于0，请重新输入")
                continue
            if total_time < cycle_time:
                print(f"错误：总运行时间必须至少{cycle_time}秒才能完成一个完整的动作周期，请重新输入")
                continue
            break
        except ValueError:
            print("输入无效，请输入一个有效的数字")
        except KeyboardInterrupt:
            print("\n已取消操作")
            return
    
    print(f"开始启动腿部控制，将运行磨线时间: {total_time}秒")
    
    print("\033[92m提示：在执行过程中，输入 'q' 并回车，可以在当前完整动作周期完成后提前结束程序。\033[0m")
    
    listen_thread = threading.Thread(target=listen_for_exit)
    listen_thread.daemon = True
    listen_thread.start()
    print(f"开始执行腿部磨线动作，每个周期 {cycle_time} 秒")
    print(f"提示：输入 'q' 并回车，可以在当前完整动作周期完成后提前结束程序")
    
    def run_motor_control():
        """在单独线程中运行电机控制"""
        global motor_control_success
        try:
            motor_control_success = leg_breakin.MotorMultiActionFrame(motor_ids, motor_actions, motion_duration, total_time)
        except Exception as e:
            print(f"\033[1;31m电机控制出现异常: {e}\033[0m")
            motor_control_success = False
    
    motor_control_success = None
    motor_thread = threading.Thread(target=run_motor_control)
    motor_thread.daemon = True
    motor_thread.start()
    
    print("电机控制已启动，开始执行动作序列...")
    
    while motor_thread.is_alive():
        time.sleep(0.1)
        
        if exit_after_cycle:
            print("用户请求退出，等待当前周期完成...")
            break
    
    motor_thread.join(timeout=2.0)
    
    if motor_control_success is False:
        print("\033[1;31m✘ 运动失败\033[0m")
    elif motor_control_success is True:
        print("\033[1;32m✓ 运动完成\033[0m")
    else:
        print("\033[1;33m⚠ 运动被提前终止\033[0m")
    
    print("\033[1;32m✓ 腿部磨线程序结束\033[0m")

def main():
    for i in range(1, g_EcMasterConfig.slave_num+1):
        encoder_range = g_EcMasterConfig.get_encoder_range(i)
        if encoder_range is not None:
            leg_breakin.setEncoderRange(i, encoder_range)
    leg_breakin.set_command_args(g_EcMasterConfig.command_args)

    option_leg_breakin()

if __name__ == "__main__":
    main()
