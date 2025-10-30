#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import time
import signal
import threading
from EcMasterConfig import EcMasterConfig

# 设置Python为无缓冲模式，确保实时输出
os.environ['PYTHONUNBUFFERED'] = '1'
sys.stdout.flush()
sys.stderr.flush()

# 心跳文件路径
HEARTBEAT_FILE = "/tmp/leg_heartbeat"

# 全局停止标志
stop_program = threading.Event()

# 心跳写入函数
def write_heartbeat():
    """写入心跳文件"""
    # 如果收到停止信号，不再写入心跳
    if stop_program.is_set():
        return
    try:
        with open(HEARTBEAT_FILE, 'w') as f:
            f.write(f"{time.time()}\n")
            f.write(f"{time.strftime('%H:%M:%S', time.localtime())}\n")
            f.write(f"leg_moving\n")
    except Exception as e:
        print(f"写入心跳文件失败: {e}")

# 信号处理函数
def signal_handler(signum, frame):
    """处理中断信号"""
    print(f"\n{time.strftime('%H:%M:%S', time.localtime())} 收到停止信号，正在安全停止腿部磨线程序...")
    stop_program.set()
    # 创建停止信号文件，通知主程序停止
    try:
        with open("/tmp/leg_stop_signal", "w") as f:
            f.write(f"stop_signal_{time.time()}")
    except Exception as e:
        print(f"创建停止信号文件失败: {e}")
    
    # 立即退出程序
    print(f"{time.strftime('%H:%M:%S', time.localtime())} 腿部磨线程序正在退出...")
    sys.exit(0)

# 注册信号处理器
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# 检查是否有root权限
if os.geteuid() != 0:
    print("\033[31merror: 请使用root权限运行\033[0m")
    sys.exit(1)

# 获取当前脚本所在目录（而非工作目录）
script_dir = os.path.dirname(os.path.abspath(__file__))
relative_path = "build_lib/kuavo4"
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
    # 电机磨线测试：对称控制左脚、右脚、左肩膀、右肩膀电机进行往复运动
    print("正在执行电机磨线测试...")
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
    
    left_foot_actions = [
        # [0, 1, 2, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],  # 左脚电机1的动作序列
        # [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 左脚电机2的动作序列
        # [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 左脚电机3的动作序列
        # [0, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 左脚电机4的动作序列
        # [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 左脚电机5的动作序列
        # [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 左脚电机6的动作序列

        [0, 15, 20, 55, 17, 0, 17, 40, 20, 10, 0, 0, 0, 0],      # 左脚电机1的动作序列
        [0, 10, 50, 90, 20, 0, -35, -60, -40, -20, 0, 0, 0, 0],  # 左脚电机2的动作序列
        [0, 0, -90, -40, -110, 0, 50, 70, 40, 0, 0, 0, 0, 0],    # 左脚电机3的动作序列
        [0, 0, 80, 40, 80, 0, 70, 95, 40, 0, 0, 0, 0, 0],        # 左脚电机4的动作序列
        [0, 10, 20, 0, -10, -20, 0, 15, 20, 0, 0, 0, 0, 0],      # 左脚电机5的动作序列
        [0, -10, -20, 0, 10, 20, 0, -10, -20, 0, 0, 0, 0, 0]     # 左脚电机6的动作序列
    ]
    
    # 左肩膀电机13的动作序列
    # left_shoulder_action = [0, 1, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # 左肩膀电机13的动作序列
    left_shoulder_action = [0, 50, 90, 50, 20, -50, -110, -50, 0, 0, 0, 0, 0, 0]  # 左肩膀电机13的动作序列
    
    # 构建完整的电机ID列表和动作序列
    motor_ids = []
    motor_actions = []
    
    # 左脚电机 (1-6)
    for i in range(6):
        motor_ids.append(i + 1)
        motor_actions.append(left_foot_actions[i])
    
    # 右脚电机 (7-12) - 镜像对称映射左脚的动作（左脚加右脚减，左脚减右脚加）
    for i in range(6):
        motor_ids.append(i + 7)
        # 只有1和7、2和8、5和11、6和12是镜像对称，其他相同
        if i in [0, 1, 4, 5]:  # 对应电机1和7、2和8、5和11、6和12
            right_foot_action = [-x for x in left_foot_actions[i]]
            motor_actions.append(right_foot_action)
        else:  # 电机3和9、4和10使用相同动作
            motor_actions.append(left_foot_actions[i])
    
    # 左肩膀电机 (13)
    motor_ids.append(13)
    motor_actions.append(left_shoulder_action)
    
    # 右肩膀电机 (14) - 对称映射左肩膀的动作
    motor_ids.append(14)
    motor_actions.append(left_shoulder_action)
    
    motion_duration = 1.0  # 每个动作持续时间
    
    print(f"电机ID: {motor_ids}")
    print(f"每个动作持续时间: {motion_duration}秒")
    print(f"总运行时间: {time_total}秒")
    sys.stdout.flush()
    
    # 创建心跳监控线程
    heartbeat_stop = threading.Event()
    
    def heartbeat_thread():
        """心跳线程，每秒写入心跳文件"""
        while not heartbeat_stop.is_set() and not stop_program.is_set():
            write_heartbeat()
            time.sleep(1.0)
    
    # 启动心跳线程
    heartbeat_thread_obj = threading.Thread(target=heartbeat_thread, daemon=True)
    heartbeat_thread_obj.start()
    
    try:
        # 直接调用C++函数，不进行停止信号检查
        # 因为C++函数内部没有停止信号检查机制，只能通过强制终止进程来停止
        print(f"\n{time.strftime('%H:%M:%S', time.localtime())} 开始执行磨线运动...")
        print("注意：如需停止，请使用Ctrl+C或主程序的安全停止功能")
        
        success = ec_master_wrap.MotorMultiAction(motor_ids, motor_actions, motion_duration, time_total)
        if not success:
            print("\033[1;31m✘ 磨线运动失败\033[0m")
            sys.stdout.flush()
        else:
            print("\033[1;32m✓ 磨线运动完成\033[0m")
            sys.stdout.flush()
    except KeyboardInterrupt:
        print(f"\n{time.strftime('%H:%M:%S', time.localtime())} 用户中断，正在安全停止...")
        stop_program.set()
    finally:
        # 停止心跳线程
        heartbeat_stop.set()
        heartbeat_thread_obj.join(timeout=2.0)
        
        # 清理停止信号文件
        try:
            if os.path.exists("/tmp/leg_stop_signal"):
                os.remove("/tmp/leg_stop_signal")
        except Exception as e:
            print(f"清理停止信号文件失败: {e}")

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
    
    # 直接执行磨线功能，不显示菜单
    option2()

if __name__ == "__main__":
    main()
