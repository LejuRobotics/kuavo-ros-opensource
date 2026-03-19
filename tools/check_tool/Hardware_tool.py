#!/usr/bin/env python3
# coding=utf-8
'''
Author: dongdongmingming
Date: 2024-04-17 17:31:40
LastEditors: Please set LastEditors
LastEditTime: 2024-04-25 14:41:52
FilePath: /kuavo/tools/check_tool/Hardware_tool.py
Description: 硬件测试脚本
'''

import time
import os,sys
import subprocess
import re
import shutil
import random
import string
import pwd
import grp

if sys.version_info[0] == 2:
    print("你正在使用 Python 2.x , 请更换运行指令为：$ sudo python3 tools/check_tool/Hardware_tool.py ")
    exit()


folder_path = os.path.dirname(os.path.abspath(__file__))    # check_tool/

# 导入 robot_version 模块
robot_version_path = os.path.join(os.path.dirname(os.path.dirname(folder_path)), 'src/kuavo_common/python')
sys.path.insert(0, robot_version_path)
try:
    from robot_version import RobotVersion
except ImportError:
    RobotVersion = None
    # 只在需要时打印警告，避免在导入时就打印

sys.path.append('/home/lab/.local/lib/python3.8/site-packages/')
sys.path.append(os.path.join(folder_path,"Ruierman"))

import yaml
import serial
import serial.tools.list_ports


import ruierman


servo_usb_path = "/dev/usb_servo"
claw_usb_path = "/dev/claw_serial"
handL_usb_path = "/dev/stark_serial_L"
handR_usb_path = "/dev/stark_serial_R"
handTouchL_usb_path = "/dev/stark_serial_touch_L"
handTouchR_usb_path = "/dev/stark_serial_touch_R"

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# print(bcolors.HEADER + "This is a header" + bcolors.ENDC)
# print(bcolors.OKBLUE + "This is a blue text" + bcolors.ENDC)
# print(bcolors.OKCYAN + "This is a cyan text" + bcolors.ENDC)
# print(bcolors.OKGREEN + "This is a green text" + bcolors.ENDC)
# print(bcolors.WARNING + "This is a warning text" + bcolors.ENDC)
# print(bcolors.FAIL + "This is an error text" + bcolors.ENDC)
# print(bcolors.BOLD + "This is a bold text" + bcolors.ENDC)
# print(bcolors.UNDERLINE + "This is an underlined text" + bcolors.ENDC)


def get_robot_version():
    # 获取用户的主目录
    home_dir = os.path.expanduser('/home/lab/')
    bashrc_path = os.path.join(home_dir, '.bashrc')

    # 初始化变量
    robot_version = None

    # 读取 .bashrc 文件
    try:
        with open(bashrc_path, 'r') as file:
            lines = file.readlines()
            for line in reversed(lines):
                # 查找 export ROBOT_VERSION= 行
                if line.startswith('export ROBOT_VERSION='):
                    # 提取变量值
                    robot_version = line.split('=')[1].strip()
                    break
    except FileNotFoundError:
        print("文件 {} 不存在".format(bashrc_path))

    return robot_version


def get_core_count():
    try:
        # Execute the 'nproc' command
        result = subprocess.run(['nproc'], capture_output=True, text=True, check=True)
        core_count = result.stdout.strip()
        return int(core_count)
    except subprocess.CalledProcessError as e:
        print("Error executing nproc: {}".format(e))
        return None


def usb_port():
    print("Hardware_tool begin")
    sudo_user = os.getenv("SUDO_USER")
    if(sudo_user):
        print(sudo_user)
    else:
        print(bcolors.FAIL + "请使用 sudo 权限运行".format(servo_usb_path) + bcolors.ENDC)
        exit(0)
    # 获取所有连接的串口设备
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        # print(port)
        print("设备: {}".format(port.device))
        print("描述: {}".format(port.description))
        print("hwid: {}".format(port.hwid))
        print("------------------------------")


    # hand L RS485 USB
    if os.path.exists(handL_usb_path):
        print(bcolors.OKGREEN + "{} 左灵巧手(普通)串口设备存在".format(handL_usb_path) + bcolors.ENDC)
    else:
        print(bcolors.FAIL + "{} 左灵巧手(普通)串口设备不存在".format(handL_usb_path) + bcolors.ENDC)
    # hand R RS485 USB
    if os.path.exists(handR_usb_path):
        print(bcolors.OKGREEN + "{} 右灵巧手(普通)串口设备存在".format(handR_usb_path) + bcolors.ENDC)
    else:
        print(bcolors.FAIL + "{} 右灵巧手(普通)串口设备不存在".format(handR_usb_path) + bcolors.ENDC)


    if os.path.exists(handTouchL_usb_path):
        print(bcolors.OKGREEN + "{} 左灵巧手（触觉）串口设备存在".format(handTouchL_usb_path) + bcolors.ENDC)
    else:
        print(bcolors.FAIL + "{} 左灵巧手（触觉）串口设备不存在".format(handTouchL_usb_path) + bcolors.ENDC)
    if os.path.exists(handTouchR_usb_path):
        print(bcolors.OKGREEN + "{} 右灵巧手（触觉）串口设备存在".format(handTouchR_usb_path) + bcolors.ENDC)
    else:
        print(bcolors.FAIL + "{} 右灵巧手（触觉）串口设备不存在".format(handTouchR_usb_path) + bcolors.ENDC)


    result,canBus = ruierman.canbus_open()
    if(result == True):
        print(bcolors.OKGREEN + "canbus open success 设备存在" + bcolors.ENDC)
    else:
        print(bcolors.FAIL + "canbus open fail 设备不存在" + bcolors.ENDC)
    time.sleep(2)
    canBus.close_canbus()


def imu_software():
    # 定义要运行的命令
    command = "/home/lab/mtmanager/linux-x64/bin/mtmanager" 

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)


def imu_test():
    # 定义要运行的命令
    command = "sudo "+ folder_path +"/bin/imu_test"

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)

def get_canbus_wiring_type():
    """
    获取CAN总线接线类型
    返回: 'single_bus' 或 'dual_bus'
    如果配置文件不存在或读取失败，将打印警告并返回 'single_bus'
    """
    canbus_wiring_file = os.path.expanduser('~/.config/lejuconfig/CanbusWiringType.ini')

    if not os.path.exists(canbus_wiring_file):
        print(bcolors.WARNING + f"警告: CAN总线配置文件不存在: {canbus_wiring_file}" + bcolors.ENDC)
        print(bcolors.WARNING + "默认使用单CAN模式 (single_bus)" + bcolors.ENDC)
        return "single_bus"
    
    try:
        with open(canbus_wiring_file, 'r') as f:
            wiring_type = f.read().strip()
            if wiring_type == "dual_bus":
                return "dual_bus"
            elif wiring_type == "single_bus":
                return "single_bus"
            else:
                print(bcolors.FAIL + f"错误: CAN总线配置值无效: '{wiring_type}'" + bcolors.ENDC)
                print(bcolors.FAIL + f"配置文件: {canbus_wiring_file}" + bcolors.ENDC)
                print(bcolors.FAIL + "有效值应为: 'single_bus' 或 'dual_bus'" + bcolors.ENDC)
                exit(1)
    except Exception as e:
        print(bcolors.FAIL + f"错误: 读取CAN总线配置文件失败: {e}" + bcolors.ENDC)
        print(bcolors.FAIL + f"配置文件: {canbus_wiring_file}" + bcolors.ENDC)
        exit(1)


def get_leju_claw_script_path():
    """
    根据CAN总线配置类型返回对应的夹爪测试脚本路径
    返回: 脚本的完整路径
    """
    wiring_type = get_canbus_wiring_type()
    
    if wiring_type == "dual_bus":
        script_name = "lejuclaw_can_test.sh"
    else:
        script_name = "lejuclaw_test.sh"
    
    script_path = os.path.join(folder_path, "leju_claw_driver", script_name)
    return script_path


def leju_claw_test():
    """
    主菜单使用的简单测试函数
    """
    uid = pwd.getpwnam('lab').pw_uid
    gid = grp.getgrnam('lab').gr_gid
    source_file = folder_path + '/config.yaml'
    target_file = '/home/lab/.config/lejuconfig/config.yaml'
    if not os.path.exists(source_file):
        print("kuavo_opensource 手臂电机 config.yaml 文件丢失")
    elif not os.path.exists(target_file):
        # 如果不存在，则复制源文件到目标位置
        shutil.copy2(source_file, target_file)
        print("Copied {} to {}".format(source_file, target_file))
        os.chown(target_file, uid, gid)
    else:
        print("{} already exists.".format(target_file))

    # 读取配置
    with open(target_file, 'r') as file:
        config = yaml.safe_load(file)

    
    claw_left = config['address'].get('Claw_joint_left', None)
    claw_right = config['address'].get('Claw_joint_right', None)
    if claw_left is None or claw_right is None:
        print("你正在使用二指夹爪config.yaml缺少配置，你需要更新 config.yaml 文件，请保存需要的内容后删除该文件再重新运行程序即可")
        # 提问用户是否删除目标文件
        user_input = input(f"确定要删除 {target_file} 吗？会自动备份当前文件（yes/no）：").strip().lower()    
        if user_input == "yes":
            # 备份目标文件
            backup_file = target_file.replace('.yaml', '_back.yaml')
            shutil.copy2(target_file, backup_file)
            print(f"已备份 {target_file} 为 {backup_file}")

            # 删除目标文件
            try:
                os.remove(target_file)
                print(f"{target_file} 已删除")
            except PermissionError:
                print(f"没有权限删除 {target_file}，请检查文件权限")
                exit(1)

            # 拷贝源文件到目标位置
            if os.path.exists(source_file):
                shutil.copy2(source_file, target_file)
                print(f"已将 {source_file} 拷贝到 {target_file}")
                os.chown(target_file, uid, gid)
            else:
                print(f"源文件 {source_file} 不存在，无法拷贝")
        else:
            print("操作已取消，文件未更新")
            exit(0)

    # 根据CAN总线配置选择脚本
    script_path = get_leju_claw_script_path()
    wiring_type = get_canbus_wiring_type()
    print(bcolors.OKCYAN + f"检测到CAN总线配置: {wiring_type}，使用脚本: {os.path.basename(script_path)}" + bcolors.ENDC)
    
    command = "sudo bash " + script_path

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)


def leju_claw_test_with_menu():
    """
    开发者工具菜单使用的带子菜单的测试函数
    """
    uid = pwd.getpwnam('lab').pw_uid
    gid = grp.getgrnam('lab').gr_gid
    source_file = folder_path + '/config.yaml'
    target_file = '/home/lab/.config/lejuconfig/config.yaml'
    if not os.path.exists(source_file):
        print("kuavo_opensource 手臂电机 config.yaml 文件丢失")
    elif not os.path.exists(target_file):
        # 如果不存在，则复制源文件到目标位置
        shutil.copy2(source_file, target_file)
        print("Copied {} to {}".format(source_file, target_file))
        os.chown(target_file, uid, gid)
    else:
        print("{} already exists.".format(target_file))

    # 读取配置
    with open(target_file, 'r') as file:
        config = yaml.safe_load(file)

    
    claw_left = config['address'].get('Claw_joint_left', None)
    claw_right = config['address'].get('Claw_joint_right', None)
    if claw_left is None or claw_right is None:
        print("你正在使用二指夹爪config.yaml缺少配置，你需要更新 config.yaml 文件，请保存需要的内容后删除该文件再重新运行程序即可")
        # 提问用户是否删除目标文件
        user_input = input(f"确定要删除 {target_file} 吗？会自动备份当前文件（yes/no）：").strip().lower()    
        if user_input == "yes":
            # 备份目标文件
            backup_file = target_file.replace('.yaml', '_back.yaml')
            shutil.copy2(target_file, backup_file)
            print(f"已备份 {target_file} 为 {backup_file}")

            # 删除目标文件
            try:
                os.remove(target_file)
                print(f"{target_file} 已删除")
            except PermissionError:
                print(f"没有权限删除 {target_file}，请检查文件权限")
                exit(1)

            # 拷贝源文件到目标位置
            if os.path.exists(source_file):
                shutil.copy2(source_file, target_file)
                print(f"已将 {source_file} 拷贝到 {target_file}")
                os.chown(target_file, uid, gid)
            else:
                print(f"源文件 {source_file} 不存在，无法拷贝")
        else:
            print("操作已取消，文件未更新")
            exit(0)

    # 先判断CAN总线配置类型
    wiring_type = get_canbus_wiring_type()
    
    if wiring_type == "dual_bus":
        # dual_bus 直接执行 lejuclaw_can_test.sh，不显示菜单
        script_path = os.path.join(folder_path, "leju_claw_driver", "lejuclaw_can_test.sh")
        print(bcolors.OKCYAN + f"检测到CAN总线配置: {wiring_type}，直接执行测试脚本" + bcolors.ENDC)
        if not os.path.exists(script_path):
            print(bcolors.FAIL + f"未找到脚本: {script_path}" + bcolors.ENDC)
            return
        command = "sudo bash " + script_path
        subprocess.run(command, shell=True)
        return
    
    # single_bus 显示测试模式选择菜单
    while True:
        print("\n*------------二指夹爪测试模式------------*")
        print(bcolors.BOLD + "请选择测试模式（q回车退出）：" + bcolors.ENDC)
        print("1. 夹爪持续发布0、100位置开合测试")
        print("2. 发布单次目标位置控制夹爪运动-启动夹爪")
        print("3. 发布单次目标位置控制夹爪运动-发布目标位置")
        
        choice = input("请输入选项编号：").strip()
        
        if choice == 'q':
            print("\n*-------------退出测试-------------*")
            break
        elif choice == "1":
            print(bcolors.HEADER + "###开始，夹爪持续开合测试（Ctrl + C 退出）###" + bcolors.ENDC)
            leju_claw_continuous_test()
            print(bcolors.HEADER + "###结束，夹爪持续开合测试###" + bcolors.ENDC)
            break
        elif choice == "2":
            print(bcolors.HEADER + "###开始，启动夹爪服务器###" + bcolors.ENDC)
            leju_claw_start_server()
            print(bcolors.HEADER + "###结束，启动夹爪服务器###" + bcolors.ENDC)
            break
        elif choice == "3":
            print(bcolors.HEADER + "###开始，发布目标位置控制测试###" + bcolors.ENDC)
            leju_claw_send_position()
            print(bcolors.HEADER + "###结束，发布目标位置控制测试###" + bcolors.ENDC)
            break
        else:
            print(bcolors.FAIL + "无效的选项编号，请重新输入！\n" + bcolors.ENDC)


def leju_claw_continuous_test():
    """
    夹爪持续发布0、100位置开合测试（仅用于single_bus配置）
    """
    script_path = os.path.join(folder_path, "leju_claw_driver", "lejuclaw_test.sh")
    command = "sudo bash " + script_path + " --continuous"
    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)


def leju_claw_start_server():
    """
    启动夹爪服务器（仅用于single_bus配置）
    """
    script_path = os.path.join(folder_path, "leju_claw_driver", "lejuclaw_test.sh")
    
    if not os.path.exists(script_path):
        print(bcolors.FAIL + f"未找到脚本: {script_path}" + bcolors.ENDC)
        return
    
    print(bcolors.OKCYAN + "正在启动夹爪服务器..." + bcolors.ENDC)
    print(bcolors.WARNING + "提示：服务器启动后，请在另一个终端运行选项3来发布目标位置" + bcolors.ENDC)
    
    command = "sudo bash " + script_path + " --start-server"
    subprocess.run(command, shell=True)


def leju_claw_send_position():
    """
    发布单次目标位置控制夹爪运动（仅用于single_bus配置）
    """
    script_path = os.path.join(folder_path, "leju_claw_driver", "lejuclaw_test.sh")
    
    if not os.path.exists(script_path):
        print(bcolors.FAIL + f"未找到脚本: {script_path}" + bcolors.ENDC)
        return
    
    try:
        print(bcolors.OKCYAN + "请输入左右夹爪的目标位置（0-100，0为开爪，100为关爪）" + bcolors.ENDC)
        left_position = input("左夹爪位置（0-100）：").strip()
        right_position = input("右夹爪位置（0-100）：").strip()
        
        left_pos = float(left_position)
        right_pos = float(right_position)
        
        if left_pos < 0 or left_pos > 100 or right_pos < 0 or right_pos > 100:
            print(bcolors.FAIL + "位置值必须在0-100之间！" + bcolors.ENDC)
            return
        
        command = "bash " + script_path + " --send-position " + str(left_pos) + " " + str(right_pos)
        print(bcolors.OKCYAN + f"正在控制夹爪移动到位置: 左={left_pos}%, 右={right_pos}%" + bcolors.ENDC)
        # 使用 subprocess.run() 运行命令
        subprocess.run(command, shell=True)
    except ValueError:
        print(bcolors.FAIL + "输入无效，请输入0-100之间的数字！" + bcolors.ENDC)
    except KeyboardInterrupt:
        print(bcolors.WARNING + "\n操作已取消" + bcolors.ENDC)


def dxl_zero():
    # 定义要运行的命令
    if(folder_path.startswith("/home/lab/kuavo_opensource/")):
        command = "sudo bash /home/lab/kuavo_opensource/bin/start_tools.sh /home/lab/kuavo_opensource/bin/dynamixel_calibrate_servos  --record"
    else:
        command =  folder_path +"/../../build/lib/DynamixelSDK/dynamixel_calibrate_servos --record" 

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)


def delete_except_preserved(target_path, preserve):
    # 列出目标路径下的所有文件和文件夹
    for item in os.listdir(target_path):
        item_path = os.path.join(target_path, item)
        
        # 如果当前项不在保留列表中，且不是以"."开头的隐藏文件，删除它
        if item not in preserve and not item.startswith('.'):
            if os.path.isfile(item_path) or os.path.islink(item_path):
                os.remove(item_path)
                print("Deleted file: {}".format(item_path))
            elif os.path.isdir(item_path):
                shutil.rmtree(item_path)
                print("Deleted folder: {}".format(item_path))


def folder_backup():
    # 定义要运行的命令
    command = "bash "+ folder_path +"/folder_backups.sh"  

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)

def control_H12():
    # 定义要运行的命令
    command = "bash "+ folder_path +"/creat_remote_udev_rule.sh"  

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)

def change_imu_usb():
    file_path = '/etc/udev/rules.d/imu.rules'
    new_rule = 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="d388", MODE:="0777", ATTR{latency_timer}="1"'
    if not os.path.exists(file_path) or not os.access(file_path, os.W_OK):
        print("Error: {} does not exist or you do not have write permission.".format(file_path))
        exit(1)

    with open(file_path, 'r') as file:
        lines = file.readlines()

    found = False
    with open(file_path, 'w') as file:
        for line in lines:
            # 使用正则表达式匹配规则（假设整行都是规则）
            if re.match(r'^\s*KERNEL=="ttyUSB.*', line):
                # 替换现有规则（如果需要）
                file.write(new_rule)
                found = True
            else:
                # 保留其他行
                file.write(line)

        # 如果未找到匹配项，则在文件末尾添加新规则
        if not found:
            file.write(new_rule)

    print("Rule has been added or replaced in {}".format(file_path))

def elmo_position_read():
    almoZR_path = folder_path + "/elmoZeroRead.py"
    print("1.复制运行该行命令修改内容进行零点数据转换：code " + almoZR_path)
    print("2.复制运行指令，将运行结果复制粘贴到零点文件中保存：python3 " + almoZR_path)
    print("3.复制运行该行命令打开零点文件进行修改：code /home/lab/.config/lejuconfig/offset.csv")

def claw_usb(choice):

    device_list = []
    
    # 获取所有连接的串口设备
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if(port.description == "LJ485A - LJ485A"):
            hwid_string = port.hwid
            # 编写正则表达式
            pattern = re.compile(r"SER=([0-9A-Z]+)")
            # 使用findall方法查找所有匹配项，这将返回一个包含所有匹配结果的列表
            matches = pattern.findall(hwid_string)
            # 输出SER值
            for match in matches:
                print("串口：", port.device,"SER", match)  # 输出: DB81ASSB

                device_list.append(port.device)
    
        if(port.description == "LJ485B - LJ485B"):
            hwid_string = port.hwid
            # 编写正则表达式
            pattern = re.compile(r"SER=([0-9A-Z]+)")
            # 使用findall方法查找所有匹配项，这将返回一个包含所有匹配结果的列表
            matches = pattern.findall(hwid_string)
            # 输出SER值
            for match in matches:
                print("串口：", port.device,"SER", match)  # 输出: DB81ASSB

                device_list.append(port.device)

    if len(device_list) == 2:
        # 定义脚本路径和参数
        arg1 = device_list[choice]
        arg2 = "claw_serial"
        # 定义要运行的命令
        command = "sudo bash "+ folder_path +"/generate_serial.sh "  +  arg1  + " " + arg2
        print(command)
        # 使用 subprocess.run() 运行命令
        subprocess.run(command, shell=True)
    else:
        print(bcolors.WARNING + "失败，请确认kuavo电源板是否正常" + bcolors.ENDC)



def hand_usb():

    device_list = []
    
    # 获取所有连接的串口设备
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if(port.description == "LJ485A - LJ485A"):
            hwid_string = port.hwid
            # 编写正则表达式
            pattern = re.compile(r"SER=([0-9A-Z]+)")
            # 使用findall方法查找所有匹配项，这将返回一个包含所有匹配结果的列表
            matches = pattern.findall(hwid_string)
            # 输出SER值
            for match in matches:
                print("串口：", port.device,"SER", match)  # 输出: DB81ASSB

                device_list.append(port.device)

        if(port.description == "LJ485B - LJ485B"):
            hwid_string = port.hwid
            # 编写正则表达式
            pattern = re.compile(r"SER=([0-9A-Z]+)")
            # 使用findall方法查找所有匹配项，这将返回一个包含所有匹配结果的列表
            matches = pattern.findall(hwid_string)
            # 输出SER值
            for match in matches:
                print("串口：", port.device,"SER", match)  # 输出: DB81ASSB

                device_list.append(port.device)

    swap_str = input("是否交换左右手(no/yes)：")
    swap_flag = 0
    if(swap_str[0].lower() == 'y'):
        swap_flag = 1
    if len(device_list) == 2:
        # 定义脚本路径和参数
        if(swap_flag):
            arg1 = device_list[0]
        else:
            arg1 = device_list[1]
        arg2 = "stark_serial_R"
        # 定义要运行的命令
        command = "sudo bash "+ folder_path +"/generate_serial.sh "  +  arg1  + " " + arg2
        print(command)
        # 使用 subprocess.run() 运行命令
        subprocess.run(command, shell=True)

        
        # 定义脚本路径和参数
        if(swap_flag):
            arg1 = device_list[1]
        else:
            arg1 = device_list[0]
        arg2 = "stark_serial_L"
        # 定义要运行的命令
        command = "sudo bash "+ folder_path +"/generate_serial.sh "  +  arg1  + " " + arg2
        print(command)
        # 使用 subprocess.run() 运行命令
        subprocess.run(command, shell=True)
    else:
        print(bcolors.WARNING + "失败，kuavo电源板485设备连接异常" + bcolors.ENDC)


def handTouch_usb():

    device_list = []
    
    # 获取所有连接的串口设备
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if(port.description == "LJ485A - LJ485A"):
            hwid_string = port.hwid
            # 编写正则表达式
            pattern = re.compile(r"SER=([0-9A-Z]+)")
            # 使用findall方法查找所有匹配项，这将返回一个包含所有匹配结果的列表
            matches = pattern.findall(hwid_string)
            # 输出SER值
            for match in matches:
                print("串口：", port.device,"SER", match)  # 输出: DB81ASSB

                device_list.append(port.device)

        if(port.description == "LJ485B - LJ485B"):
            hwid_string = port.hwid
            # 编写正则表达式
            pattern = re.compile(r"SER=([0-9A-Z]+)")
            # 使用findall方法查找所有匹配项，这将返回一个包含所有匹配结果的列表
            matches = pattern.findall(hwid_string)
            # 输出SER值
            for match in matches:
                print("串口：", port.device,"SER", match)  # 输出: DB81ASSB

                device_list.append(port.device)

    swap_str = input("是否交换左右手(no/yes)：")
    swap_flag = 0
    if(swap_str[0].lower() == 'y'):
        swap_flag = 1
    if len(device_list) == 2:
        # 定义脚本路径和参数
        if(swap_flag):
            arg1 = device_list[0]
        else:
            arg1 = device_list[1]
        arg2 = "stark_serial_touch_R"
        # 定义要运行的命令
        command = "sudo bash "+ folder_path +"/generate_serial.sh "  +  arg1  + " " + arg2
        print(command)
        # 使用 subprocess.run() 运行命令
        subprocess.run(command, shell=True)

        
        # 定义脚本路径和参数
        if(swap_flag):
            arg1 = device_list[1]
        else:
            arg1 = device_list[0]
        arg2 = "stark_serial_touch_L"
        # 定义要运行的命令
        command = "sudo bash "+ folder_path +"/generate_serial.sh "  +  arg1  + " " + arg2
        print(command)
        # 使用 subprocess.run() 运行命令
        subprocess.run(command, shell=True)
    else:
        print(bcolors.WARNING + "失败，kuavo电源板485设备连接异常" + bcolors.ENDC)


def ruiwo_zero():
    # 检查CAN总线接线方式
    canbus_wiring_file = os.path.expanduser('~/.config/lejuconfig/CanbusWiringType.ini')
    
    use_motorevo_tool = False
    if os.path.exists(canbus_wiring_file):
        with open(canbus_wiring_file, 'r') as f:
            wiring_type = f.read().strip()
            if wiring_type == "dual_bus":
                use_motorevo_tool = True
    
    if use_motorevo_tool:
        print(bcolors.OKGREEN + "检测到 Roban2 双CAN配置，使用 motorevo_tool.sh 工具" + bcolors.ENDC)
        # 使用新的双CAN工具
        command = "bash " + folder_path + "/motorevo_tool.sh --cali"

    else:
        kuavo_ros_file_path = folder_path +"/ruiwo_zero_set.sh" 
        kuavo_open_file_path = folder_path +"../../installed/share/hardware_plant/lib/ruiwo_controller/setZero.sh" 
        

        if os.path.exists(kuavo_ros_file_path):
            command = "bash "+ kuavo_ros_file_path
        elif os.path.exists(kuavo_open_file_path):
            command = "bash "+ kuavo_open_file_path
        else:
            print(f"The file {file_path} does not exist.")
            return
            
    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)

def ruiwo_zero_selective():
    """
    选择性设置指定电机硬件零点（仅支持双CAN配置）
    """
    # 检查CAN总线接线方式
    canbus_wiring_file = os.path.expanduser('~/.config/lejuconfig/CanbusWiringType.ini')
    
    use_motorevo_tool = False
    if os.path.exists(canbus_wiring_file):
        with open(canbus_wiring_file, 'r') as f:
            wiring_type = f.read().strip()
            if wiring_type == "dual_bus":
                use_motorevo_tool = True
    
    if not use_motorevo_tool:
        print(bcolors.FAIL + "错误：此功能仅支持双CAN配置" + bcolors.ENDC)
        print(bcolors.WARNING + "当前为单CAN配置，请使用选项 'd' 进行零点设置" + bcolors.ENDC)
        return
    
    print(bcolors.OKGREEN + "检测到双CAN配置" + bcolors.ENDC)
    print(bcolors.OKCYAN + "请输入要设置零点的电机ID（多个ID用逗号分隔，如：1,8）" + bcolors.ENDC)
    print(bcolors.OKCYAN + "或直接按回车设置所有电机零点" + bcolors.ENDC)
    
    user_input = input("电机ID: ").strip()
    
    motorevo_tool = os.path.join(folder_path, "motorevo_tool.sh")
    if not os.path.exists(motorevo_tool):
        print(bcolors.FAIL + f"错误：未找到 {motorevo_tool}" + bcolors.ENDC)
        return
    
    if user_input:
        # 解析用户输入的电机ID
        try:
            # 去除空格并按逗号分割
            motor_ids = [id.strip() for id in user_input.split(',') if id.strip()]
            if not motor_ids:
                print(bcolors.FAIL + "错误：未输入有效的电机ID" + bcolors.ENDC)
                return
            
            # 验证所有ID都是数字
            for id in motor_ids:
                if not id.isdigit():
                    print(bcolors.FAIL + f"错误：无效的电机ID '{id}'，请输入数字" + bcolors.ENDC)
                    return
            
            motor_ids_str = ','.join(motor_ids)
            print(bcolors.OKCYAN + f"将为以下电机设置零点: {motor_ids_str}" + bcolors.ENDC)
            
            # 执行设置零点命令，传递电机ID参数
            command = f"bash {motorevo_tool} --set-zero {motor_ids_str}"
            subprocess.run(command, shell=True)
            
        except Exception as e:
            print(bcolors.FAIL + f"错误：{e}" + bcolors.ENDC)
            return
    else:
        # 未输入ID，设置所有电机零点
        print(bcolors.WARNING + "未指定电机ID，将设置所有电机零点" + bcolors.ENDC)
        confirm = input("确定要设置所有电机零点吗？(yes/no): ").strip().lower()
        if confirm == "yes":
            command = f"bash {motorevo_tool} --set-zero"
            subprocess.run(command, shell=True)
        else:
            print(bcolors.OKGREEN + "操作已取消" + bcolors.ENDC)

def ruiwo_negtive():
    while True:
        print("请选择手臂总线类型：")
        print("1. 单CAN")
        print("2. 双CAN（ROBAN2.1）")
        choice = input("请输入选择 (1 或 2): ").strip()

        if choice == "1":
            kuavo_ros_file_path = folder_path + "/ruiwo_zero_set.sh"
            kuavo_open_file_path = folder_path + "../../installed/share/hardware_plant/lib/ruiwo_controller/setZero.sh"

            if os.path.exists(kuavo_ros_file_path):
                command = "bash " + kuavo_ros_file_path
            elif os.path.exists(kuavo_open_file_path):
                command = "bash " + kuavo_open_file_path
            else:
                print(f"The file {file_path} does not exist.")
                return

            subprocess.run(command, shell=True)
            break

        elif choice == "2":
            motorevo_tool = os.path.join(folder_path, "motorevo_tool.sh")
            if os.path.exists(motorevo_tool):
                command = f"bash {motorevo_tool} --cali"
                subprocess.run(command, shell=True)
            else:
                print(bcolors.FAIL + f"错误：未找到 {motorevo_tool}" + bcolors.ENDC)
            break

        else:
            print(bcolors.FAIL + "无效选择，请重新选择！" + bcolors.ENDC)

def ruiwo_negtive():
    while True:
        print("请选择手臂总线类型：")
        print("1. 单CAN")
        print("2. 双CAN（ROBAN2.1）")
        can_choice = input("请输入选择 (1 或 2): ").strip()

        if can_choice == "2":
            motorevo_tool = os.path.join(folder_path, "motorevo_tool.sh")
            if os.path.exists(motorevo_tool):
                command = f"bash {motorevo_tool} --negative"
                subprocess.run(command, shell=True)
            else:
                print(bcolors.FAIL + f"错误：未找到 {motorevo_tool}" + bcolors.ENDC)
            break

        elif can_choice == "1":
            while True:
                print("请选择机器人类型：")
                print("1. 4Pro型（14个电机）")
                print("2. Roban2型（10个电机）")
                choice = input("请输入选择 (1 或 2): ").strip()
                if choice == "1":
                    robot_type = "4pro"
                    break
                elif choice == "2":
                    robot_type = "roban2"
                    break
                else:
                    print(bcolors.FAIL + "无效选择，请重新选择！" + bcolors.ENDC)
                    continue

            command = "bash " + folder_path + "/ruiwo_negtive_set.sh " + robot_type
            subprocess.run(command, shell=True)
            break
        else:
            print(bcolors.FAIL + "无效选择，请重新选择！" + bcolors.ENDC)
            continue


def qiangnao_hand():
    """
    测试灵巧手（一代手/二代手）
    """
    # 让用户选择手类型
    print(bcolors.BOLD + "请选择手类型：" + bcolors.ENDC)
    print("1. 一代手 (Revo1)")
    print("2. 二代手 (Revo2) 注：灵心巧手属于二代手")
    hand_type = input("请输入选项：")
    if hand_type not in ["1", "2"]:
        print(bcolors.FAIL + "无效选项" + bcolors.ENDC)
        return

    # 检查CAN总线配置类型
    canbus_wiring_file = os.path.expanduser('~/.config/lejuconfig/CanbusWiringType.ini')

    is_dual_bus = False
    if os.path.exists(canbus_wiring_file):
        with open(canbus_wiring_file, 'r') as f:
            wiring_type = f.read().strip()
            if wiring_type == "dual_bus":
                is_dual_bus = True

    # 根据手类型和CAN总线配置选择不同的命令
    command = None

    if hand_type == "2":
        # 二代手 (Revo2)
        if is_dual_bus:
            command = "bash " + folder_path + "/dexhand_test.sh --revo2can --test 3"
            print(bcolors.OKGREEN + "检测到二代手双CAN配置，使用 Revo2Can 测试命令" + bcolors.ENDC)
        else:
            command = "bash " + folder_path + "/dexhand_test.sh --revo2 --test 3"
            print(bcolors.OKGREEN + "检测到二代手单CAN配置，使用 Revo2 测试命令" + bcolors.ENDC)
    else:
        # 一代手 (Revo1)
        if is_dual_bus:
            command = "bash " + folder_path + "/dexhand_test.sh --revo1can --test 3"
            print(bcolors.OKGREEN + "检测到一代手双CAN配置，使用 Revo1Can 测试命令" + bcolors.ENDC)
        else:
            command = "bash " + folder_path + "/dexhand_test.sh --normal --test 3"
            print(bcolors.OKGREEN + "检测到一代手单CAN配置，使用 normal 测试命令" + bcolors.ENDC)

    if command:
        subprocess.run(command, shell=True)
    else:
        print(bcolors.FAIL + "错误: 无法确定测试命令" + bcolors.ENDC)


def touch_dexhand():
    # 定义要运行的命令
    choice = input("请输入你要对触觉灵巧手的操作(Ctrl+C退出)  1. 配置usb 2. 测试 : ")
    if choice == "1":
        handTouch_usb()
    elif choice == "2":
        command = "bash "+ folder_path +"/dexhand_test.sh --touch --test" 
        # 使用 subprocess.run() 运行命令
        subprocess.run(command, shell=True)
    else:
        print("输入无效，请重新运行程序~")


def update_kuavo():
    # 定义要运行的命令
    command = "bash "+ folder_path +"/update_kuavo.sh"  

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)

def arm_setzero():

    kuavo_ros_file_path = folder_path +"/arm_setzero.sh" 
    kuavo_open_file_path = folder_path +"../../installed/share/hardware_plant/lib/ruiwo_controller/arm_setzero.sh" 
    
    if os.path.exists(kuavo_ros_file_path):
        command = "bash "+ kuavo_ros_file_path
    elif os.path.exists(kuavo_open_file_path):
        command = "bash "+ kuavo_open_file_path
    else:
        print(f"The file {file_path} does not exist.")
        return
        
    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)

def arm_breakin():

    kuavo_ros_file_path = folder_path + "/arm_breakin.sh" 
    kuavo_open_file_path = folder_path + "../../installed/share/hardware_plant/lib/ruiwo_controller/arm_breakin.sh" 
    
    if os.path.exists(kuavo_ros_file_path):
        command = "bash "+ kuavo_ros_file_path
    elif os.path.exists(kuavo_open_file_path):
        command = "bash "+ kuavo_open_file_path
    else:
        print(f"The file {file_path} does not exist.")
        return
        
    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)
    
def fix_ros_key():
    # 脚本路径
    script_paths = [
        os.path.join(folder_path, "fix_ros_key.sh"),
        os.path.join(folder_path, "../../tools/check_tool/fix_ros_key.sh")
    ]
    
    script_path = None
    # 检查脚本是否存在
    for path in script_paths:
        if os.path.exists(path):
            script_path = path
            break
    
    if not script_path:
        print(f"{bcolors.FAIL}错误：未找到fix_ros_key.sh脚本{bcolors.ENDC}")
        print("请检查脚本路径是否正确或是否存在该脚本")
        return
    
    try:
        print(f"{bcolors.OKCYAN}正在执行ROS密钥更新脚本...{bcolors.ENDC}")
        # 使用subprocess运行脚本，设置stdout和stderr为None（默认继承父进程的终端输出）
        result = subprocess.run(
            f"bash {script_path}",
            shell=True,
            stdout=None,  # 直接输出到终端
            stderr=None,  # 直接输出到终端
            check=True  # 若脚本返回非0状态码则抛出异常
        )
        print(f"{bcolors.OKGREEN}ROS密钥更新成功！{bcolors.ENDC}")
        
    except subprocess.CalledProcessError as e:
        print(f"{bcolors.FAIL}ROS密钥更新失败，返回码：{e.returncode}{bcolors.ENDC}")
        print(f"{bcolors.FAIL}脚本执行异常，请检查脚本内容或权限{bcolors.ENDC}")
    except FileNotFoundError:
        print(f"{bcolors.FAIL}错误：未找到bash解释器，请确保系统已安装bash{bcolors.ENDC}")
    except Exception as e:
        print(f"{bcolors.FAIL}执行脚本时发生未知错误：{str(e)}{bcolors.ENDC}")

def hip_imu_serial_set():
    kuavo_ros_file_path = folder_path + "/hip_imu_serial_set.sh"
    kuavo_open_file_path = folder_path + "../../installed/share/hardware_plant/lib/hipnuc_imu/scripts/hip_imu_serial_set.sh" 
    
    if os.path.exists(kuavo_ros_file_path):
        command = "bash "+ kuavo_ros_file_path
    elif os.path.exists(kuavo_open_file_path):
        command = "bash "+ kuavo_open_file_path
    else:
        print(f"The file {file_path} does not exist.")
        return
        
    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)

def hip_imu_test():
    kuavo_ros_file_path = folder_path + "/hip_imu_test.sh"
    kuavo_open_file_path = folder_path + "../../installed/share/hardware_plant/lib/hipnuc_imu/scripts/hip_imu_test.sh" 
    
    if os.path.exists(kuavo_ros_file_path):
        command = "bash "+ kuavo_ros_file_path
    elif os.path.exists(kuavo_open_file_path):
        command = "bash "+ kuavo_open_file_path
    else:
        print(f"The file {file_path} does not exist.")
        return
        
    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)


def stress_test_all_cores():
    """
    启动 CPU 压力测试，检查散热
    """
    stress_test_script = os.path.join(folder_path, "stress_test", "stress_test_all_cores.sh")
    
    if not os.path.exists(stress_test_script):
        print(bcolors.FAIL + f"错误：压力测试脚本不存在: {stress_test_script}" + bcolors.ENDC)
        return
    
    print(bcolors.OKCYAN + f"启动 CPU 压力测试脚本: {stress_test_script}" + bcolors.ENDC)
    print()
    
    # 使用 subprocess.run() 运行命令
    command = "bash " + stress_test_script
    subprocess.run(command, shell=True)


def isolate_cores():
    kuavo_ros_file_path = folder_path + "/isolate_cores.sh"
    kuavo_open_file_path = folder_path + "../../installed/share/hardware_plant/lib/hipnuc_imu/scripts/isolate_cores.sh" 
    
    if os.path.exists(kuavo_ros_file_path):
        command = "bash "+ kuavo_ros_file_path
    elif os.path.exists(kuavo_open_file_path):
        command = "bash "+ kuavo_open_file_path
    else:
        print(f"The file {file_path} does not exist.")
        return
        
    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)

def license_sign():
    FILE = "/home/lab/.config/lejuconfig/ec_master.key"
    # 检查文件是否存在
    if os.path.exists(FILE):
        # 打开文件并读取内容
        with open(FILE, 'r') as file:
            content = file.read()
            first_20_chars = content[:20]
            print(f"license key 存在，内容为: {first_20_chars}")
            print(bcolors.OKCYAN + "如需要重新设置key请继续，如不需要，请 Ctrl+C 退出" + bcolors.ENDC)
    else:
        print(bcolors.OKCYAN + "License key 不存在 " + bcolors.ENDC)
        

    license_str = input("请输入提供的License：")
    # 定义要运行的命令
    command = "bash "+ folder_path +"/EtherCAT_license.sh "  + license_str

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)
    print("请检查机器人程序运行后有显示 EtherCAT license is verified! 为license注册成功。")

def MAC_get():
    print("请等待  10s 。。。")
    # 定义要运行的命令
    command = "bash "+ folder_path +"/ecMAC/ecMACget.sh"  

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)

    time.sleep(2)
    # 文件路径
    file_path = folder_path + "/ecMAC/output_file.txt"

    # 检查文件是否存在
    if os.path.exists(file_path):
        # 打开文件并读取内容
        with open(file_path, "r") as file:
            content = file.readlines()

        # 正则表达式匹配 MAC 地址
        mac_pattern = r"network adapter MAC:\s([0-9A-Fa-f-]+)"
        mac_addresses = []

        for line in content:
            matches = re.findall(mac_pattern, line)
            if matches:
                mac_addresses.extend(matches)

        # 打印提取到的 MAC 地址
        if mac_addresses:
            print("MAC 地址如下:")
            for mac in mac_addresses:
                print(mac)
        else:
            print("No MAC addresses found in the file. 请检查设备状况。")
    else:
        print(f"File not found: {file_path} 请检查设备状况。")

def reset_folder():
    
    # 定义要保留的文件和文件夹的名称
    preserve = {
        "confirm_backups.zip",
        "kuavo_opensource",
        "mtmanager",
        "pybind11",
        "snap",
        "gdb",
        "wifi",
        "rosbag",
        "craic_code_repo",
        "kuavo-ros-opensource",
        "Documents",
        "Downloads",
        "xfolder"
    }

    # 指定目标路径
    target_path = "/home/lab/"

    license_str = input("请谨慎，此操作将会删除文件和文件夹内容不可恢复！！！（回车继续）：")

    random_char = random.choice(string.ascii_letters)
    print("Please enter the following character（请输入该字符）: {}".format(random_char))
    user_input = input("Your input: ")

    # 检查用户输入是否正确（区分大小写）
    if user_input == random_char:
        delete_except_preserved(target_path, preserve)
        delete_except_preserved("/home/lab/.ssh/", {})
        print("文件夹清除成功。")
    else:
        print("您输入的字符不符，请重试。")

def read_and_edit_env_file(file_path, target_variable, new_value):
    try:
        # 读取 .env 文件内容
        with open(file_path, 'r') as file:
            lines = file.readlines()

        # 存储编辑后的内容
        new_lines = []
        variable_found = False

        # 遍历文件的每一行
        for line in lines:
            # 如果行是以 'target_variable=' 开头，说明找到了要编辑的变量
            if line.startswith(f"{target_variable}="):
                # 将目标变量的值更新为 new_value
                new_lines.append(f"{target_variable}={new_value}\n")
                variable_found = True
            else:
                # 保留原行
                new_lines.append(line)

        # 如果没有找到目标变量，添加到文件的末尾
        if not variable_found:
            new_lines.append(f"{target_variable}={new_value}\n")

        # 将修改后的内容写回文件
        with open(file_path, 'w') as file:
            file.writelines(new_lines)

        print(f"'{target_variable}' has been updated to '{new_value}' in {file_path}")

    except FileNotFoundError:
        print(f"Error: The file {file_path} was not found.")
    except Exception as e:
        print(f"Error occurred: {e}")

def get_env_variable(file_path, variable_name):
    try:
        # 读取 .env 文件内容
        with open(file_path, 'r') as file:
            for line in file:
                # 去除左右空白字符并忽略空行和注释
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                
                # 拆分 KEY=VALUE 格式的行
                key, value = line.split("=", 1)
                if key == variable_name:
                    return value.strip()

        print(f"'{variable_name}' not found in {file_path}")
        return None
    except FileNotFoundError:
        print(f"Error: The file {file_path} was not found.")
    except Exception as e:
        print(f"Error occurred: {e}")
        return None

def robot_login():
    env_file_path = folder_path + "/report_robot_network_info_service/RRNIS.env"  # .env 文件的路径
    variable_name = "EC_MASTER_MAC"  # 目标变量的名称

    # 获取并打印变量的值
    variable_value = get_env_variable(env_file_path, variable_name)
    robot_name_value = get_env_variable(env_file_path, "ROBOT_SERIAL_NUMBER")
    if variable_value:
        print(f"机器人编号： {robot_name_value}   {variable_name}={variable_value}  ")

    # 设置 MAC 地址
    new_mac_address = input("请输入新的 MAC 地址：")
    pattern = r"^([0-9A-Fa-f]{2}-){5}[0-9A-Fa-f]{2}$"
    if(bool(re.match(pattern, new_mac_address))):
        print(f"{new_mac_address} 已变更.")
    else:
        print(f"{new_mac_address} 不合法.")
        return False

    env_file_path = folder_path + "/report_robot_network_info_service/RRNIS.env"  # 文件路径
    variable_to_edit = "EC_MASTER_MAC"  # 需要修改的变量名
    new_variable_value = new_mac_address  # 变量的新值
    read_and_edit_env_file(env_file_path, variable_to_edit, new_variable_value)

    variable_to_edit = "ROBOT_SERIAL_NUMBER"  # 需要修改的变量名
    new_mac_address = input("请输入新的 机器人编号：")
    new_variable_value = new_mac_address  # 变量的新值
    read_and_edit_env_file(env_file_path, variable_to_edit, new_variable_value)
    

    # 定义要运行的命令
    command = "bash "+ folder_path +"/report_robot_network_info_service/setup.sh"  

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)
    
    # sudo systemctl start report_robot_network_info.service

def get_git_info():
    """
    获取 Git 信息:
    1. 如果存在 .git 文件夹，则调用 git log 获取
    2. 否则读取 .version 目录下的 GIT_* 文件
    """
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # 回退两级目录，得到仓库根目录
    repo_root = os.path.abspath(os.path.join(script_dir, "..", ".."))


    # 拼接 .git 和 .version 路径
    git_dir = os.path.join(repo_root, ".git")
    version_dir = os.path.join(repo_root, ".version")

    # 情况1: 存在 .git 文件夹
    if os.path.exists(git_dir):
        try:
            result = subprocess.run(
                ['git', 'log', '-1', '--format=%H%n%ci%n%s'],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5
            )
            output = result.stdout.decode('utf-8').strip().split('\n')

            if len(output) < 3:
                raise ValueError("Unexpected git output: " + repr(output))

            return {
                'commit_hash': output[0],
                'commit_date': output[1],
                'commit_message': output[2],
                'branch': subprocess.run(
                    ['git', 'rev-parse', '--abbrev-ref', 'HEAD'],
                    check=True, stdout=subprocess.PIPE
                ).stdout.decode('utf-8').strip()
            }

        except Exception as e:
            print("❌ Git command failed:", str(e))
            return None

    # 情况2: 没有 .git，尝试读取 .version 目录
    elif os.path.exists(version_dir):
        def read_file(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    return f.read().strip()
            except FileNotFoundError:
                return None

        return {
            'commit_hash': read_file(os.path.join(version_dir, "GIT_COMMIT")),
            'branch': read_file(os.path.join(version_dir, "GIT_BRANCH")),
            'remote': read_file(os.path.join(version_dir, "GIT_REMOTE")),
            'tag': read_file(os.path.join(version_dir, "GIT_TAG")),
        }

    else:
        print("⚠️ Neither .git nor .version found, cannot get commit info.")
        return None

def secondary_menu():
    while True:
        print("\n*------------开发者工具------------*")
        print(bcolors.BOLD + "请输入一个选项按回车(q回车退出)：" + bcolors.ENDC)
        print("0. 打开零点文件")
        print("1. 硬件连接检查")
        print("2. 打开imu上位机软件(接屏幕)")
        print("3. 测试imu(先编译)")
        print("a. 测试二指夹爪（Ctrl + C 退出）")
        print("b. 配置灵巧手（普通）usb")
        print("c. 测试灵巧手")
        print("d. 手臂电机设置软件零点（仅将电机偏置值保存到零点文件）")
        print("dd. 手臂电机设置硬件零点（将电机当前位置设为电机零点）")
        print("e. 手臂电机辨识方向（注意电机限位不要堵转）")    
        print("f. 零点文件备份")
        print("g. 遥控器配置usb")
        print("h. 新款IMU通信板配置usb")
        print("i. 电机零点数据转换")
        print("j. 触觉灵巧手操作")
        print("k. 更新当前目录程序(注意：会重置文件内容，建议备份文件)")
        # print("m. MAC 地址")
        print("l. license导入")
        print("m. 执行机器人磨线")
        print("n. 更新ros密钥")
        print("o. 国产IMU配置udev规则")
        print("p. 国产IMU测试")
        print("r. 隔离CPU核心 ")
        print("u. 配置robot上线提醒")
        print("t. 恢复出厂文件夹")
        print("v. 执行CPU压力测试，检查散热")

        option = input("请输入你的选择：")
        if option == 'q':
            print("\n*-------------退出程序-------------*")
            break
        if option == "0":
            print(bcolors.HEADER + "###开始，打开零点文件###" + bcolors.ENDC)
            print("复制运行该行命令打开：code /home/lab/.config/lejuconfig/offset.csv")
            print(bcolors.HEADER + "###结束，打开零点文件###" + bcolors.ENDC)
            break
        elif option == "1":
            print(bcolors.HEADER + "###开始，硬件连接检查###" + bcolors.ENDC)
            usb_port()
            print(bcolors.HEADER + "###结束，硬件连接检查###" + bcolors.ENDC)
            break
        elif option == "2":
            print(bcolors.HEADER + "###正在打开imu上位机软件###" + bcolors.ENDC)
            imu_software()
            print(bcolors.HEADER + "###已打开imu上位机软件###" + bcolors.ENDC)
            break
        elif option == "3":
            print(bcolors.HEADER + "###开始，测试imu(Ctrl+C退出)###" + bcolors.ENDC)
            imu_test()
            print(bcolors.HEADER + "###结束，测试imu###" + bcolors.ENDC)
            break
        elif option == "a":
            print(bcolors.HEADER + "###开始，测试夹爪（Ctrl + C 退出）###" + bcolors.ENDC)
            leju_claw_test_with_menu()
            print(bcolors.HEADER + "###结束，测试夹爪###" + bcolors.ENDC)
            break
        elif option == "b":
            print(bcolors.HEADER + "###开始，配置灵巧手（普通）usb###" + bcolors.ENDC)
            hand_usb()
            print(bcolors.HEADER + "###结束，配置灵巧手（普通）usb###" + bcolors.ENDC)
            break
        elif option == "c":
            print(bcolors.HEADER + "###开始，测试灵巧手###" + bcolors.ENDC)
            print(bcolors.OKCYAN + "先左右手一起握，然后依次握左手，握右手" + bcolors.ENDC)
            qiangnao_hand()
            print(bcolors.HEADER + "###结束，测试灵巧手###" + bcolors.ENDC)
            break
        elif option == "d":
            print(bcolors.HEADER + "###开始，手臂电机设置软件零点（仅将电机偏置值保存到零点文件）###" + bcolors.ENDC)
            ruiwo_zero()
            print(bcolors.HEADER + "###结束，手臂电机设置软件零点（仅将电机偏置值保存到零点文件）###" + bcolors.ENDC)
            break
        elif option == "dd":
            print(bcolors.HEADER + "###开始，手臂电机设置硬件零点（将电机当前位置设为电机零点）###" + bcolors.ENDC)
            ruiwo_zero_selective()
            print(bcolors.HEADER + "###结束，手臂电机设置硬件零点（将电机当前位置设为电机零点）###" + bcolors.ENDC)
            print(bcolors.HEADER + "###【注意，还需要执行 d, 来设置电机软件零点】###" + bcolors.ENDC)
            break
        elif option == "e":
            print(bcolors.HEADER + "###开始，手臂电机辨识方向###" + bcolors.ENDC)
            ruiwo_negtive()
            print(bcolors.HEADER + "###结束，手臂电机辨识方向###" + bcolors.ENDC)
            break
        elif option == "f":
            print(bcolors.HEADER + "###开始，文件备份###" + bcolors.ENDC)
            folder_backup()
            print(bcolors.HEADER + "###结束，文件备份###" + bcolors.ENDC)
            break
        elif option == "g":
            print(bcolors.HEADER + "###开始，遥控器配置usb###" + bcolors.ENDC)
            control_H12()
            print(bcolors.HEADER + "###结束，遥控器配置usb###" + bcolors.ENDC)
            break
        elif option == "h":
            print(bcolors.HEADER + "###开始，新IMU模块配置usb###" + bcolors.ENDC)
            change_imu_usb()
            print(bcolors.HEADER + "###结束，新IMU模块配置usb###" + bcolors.ENDC)
            break
        elif option == "i":
            print(bcolors.HEADER + "###开始，电机零点数据转换###" + bcolors.ENDC)
            elmo_position_read()
            print(bcolors.HEADER + "###结束，电机零点数据转换###" + bcolors.ENDC)
            break
        elif option == "k":
            print(bcolors.HEADER + "###开始，更新当前目录程序###" + bcolors.ENDC)
            update_kuavo()
            print(bcolors.HEADER + "###结束，更新当前目录程序###" + bcolors.ENDC)
            break
        elif option == "j":
            print(bcolors.HEADER + "###开始，触觉灵巧手操作###" + bcolors.ENDC)
            touch_dexhand()
            print(bcolors.HEADER + "###结束，触觉灵巧手操作###" + bcolors.ENDC)
            break
        # elif option == "m":
        #     print(bcolors.HEADER + "###开始，获取 MAC 地址（等待执行完成）###" + bcolors.ENDC)
        #     MAC_get()
        #     print(bcolors.HEADER + "###结束，获取 MAC 地址###" + bcolors.ENDC)   
        #     break  
        elif option == "l":
            print(bcolors.HEADER + "###开始，license导入###" + bcolors.ENDC)
            license_sign()
            print(bcolors.HEADER + "###结束，license已导入，请确认验证###" + bcolors.ENDC)   
            break  
        elif option == "m":
            print(bcolors.HEADER + "###开始，执行机器人磨线###" + bcolors.ENDC)
            
            # 获取机器人版本
            robot_version = get_robot_version()
            kuavo_breakin_script = None
            script_description = ""
            
            if robot_version:
                try:
                    version_num = int(robot_version)

                    if 13 <= version_num <= 14:
                        kuavo_breakin_script = os.path.join(folder_path, "joint_breakin_ros", "src", "breakin_control", "scripts", "breakin_main_controller.py")
                        script_description = f"roban2磨线脚本 joint_breakin_ros/src/breakin_control/scripts/breakin_main_controller.py (版本 {robot_version})"

                    elif version_num == 17:
                        kuavo_breakin_script = os.path.join(folder_path, "joint_breakin_ros", "src", "breakin_control", "scripts", "breakin_main_controller.py")
                        script_description = f"roban2磨线脚本 joint_breakin_ros/src/breakin_control/scripts/breakin_main_controller.py (版本 {robot_version})"

                    elif 40 <= version_num <= 49:
                        kuavo_breakin_script = os.path.join(folder_path, "joint_breakin", "joint_breakin.py")
                        script_description = f"Kuavo4磨线脚本 joint_breakin/joint_breakin.py (版本 {robot_version})"

                    elif 50 <= version_num <= 52:
                        kuavo_breakin_script = os.path.join(folder_path, "joint_breakin_ros", "src", "breakin_control", "scripts", "breakin_main_controller.py")
                        script_description = f"Kuavo5磨线脚本 joint_breakin_ros/src/breakin_control/scripts/breakin_main_controller.py (版本 {robot_version})"

                    
                    elif version_num == 53:
                        kuavo_breakin_script = os.path.join(folder_path, "joint_breakin_ros", "src", "breakin_control", "scripts", "breakin_main_controller.py")
                        script_description = f"Kuavo5磨线脚本 joint_breakin_ros/src/breakin_control/scripts/breakin_main_controller.py (版本 {robot_version})"

                    else:
                        print(bcolors.WARNING + f"警告：版本 {robot_version} 不在支持的范围内（13-14、17、40-49、50-52、53），当前不支持自动选择磨线脚本" + bcolors.ENDC)
                except (ValueError, TypeError):
                    print(bcolors.WARNING + f"警告：无法解析版本号 {robot_version}，请检查 ~/.bashrc 中的 ROBOT_VERSION 设置" + bcolors.ENDC)
            else:
                print(bcolors.WARNING + "警告：未找到 ROBOT_VERSION，无法自动选择磨线脚本" + bcolors.ENDC)
            
            if kuavo_breakin_script:
                if os.path.exists(kuavo_breakin_script):
                    print(bcolors.OKGREEN + f"\n使用{script_description}" + bcolors.ENDC)
                    # 如果需要以 root 运行，请直接使用 root 终端启动本工具
                    command = "python3 " + kuavo_breakin_script
                    subprocess.run(command, shell=True)
                else:
                    print(bcolors.FAIL + f"错误：磨线脚本不存在: {kuavo_breakin_script}" + bcolors.ENDC)
            print(bcolors.HEADER + "###结束，执行机器人磨线###" + bcolors.ENDC)
            break
        elif option == "n":
            print(bcolors.HEADER + "###开始，更新ros密钥###" + bcolors.ENDC)
            fix_ros_key()
            print(bcolors.HEADER + "###结束，更新ros密钥###" + bcolors.ENDC)   
            break
        elif option == "o":
            print(bcolors.HEADER + "###开始，国产IMU配置udev规则###" + bcolors.ENDC)
            hip_imu_serial_set()
            print(bcolors.HEADER + "###结束，国产IMU配置udev规则###" + bcolors.ENDC)   
            break
        elif option == "p":
            print(bcolors.HEADER + "###开始，国产IMU测试###" + bcolors.ENDC)
            hip_imu_test()
            print(bcolors.HEADER + "###结束，国产IMU测试###" + bcolors.ENDC)
            break
        elif option == "r":
            print(bcolors.HEADER + "###开始，隔离CPU核心###" + bcolors.ENDC)
            isolate_cores()
            print(bcolors.HEADER + "###结束，隔离CPU核心###" + bcolors.ENDC)
            break
        elif option == "u":
            print(bcolors.HEADER + "###开始，robot上线提醒配置###" + bcolors.ENDC)
            robot_login()
            print("运行指令将触发提示：sudo systemctl start report_robot_network_info.service")
            print(bcolors.HEADER + "###结束，robot上线提醒配置###" + bcolors.ENDC)   
            break
        elif option == "t":
            print(bcolors.HEADER + "###开始，恢复出厂文件夹###" + bcolors.ENDC)
            reset_folder()
            print(bcolors.HEADER + "###结束，恢复出厂文件夹###" + bcolors.ENDC) 
            break
        elif option == "v":
            print(bcolors.HEADER + "###开始，执行CPU压力测试，检查散热###" + bcolors.ENDC)
            stress_test_all_cores()
            print(bcolors.HEADER + "###结束，执行CPU压力测试，检查散热###" + bcolors.ENDC)
            break
        else:
            print(bcolors.FAIL + "无效选项，请重新选择！\n" + bcolors.ENDC)


if __name__ == '__main__':
    
    # 获取 ROBOT_VERSION 变量值
    robot_version = get_robot_version()
    if robot_version:
        print("ROBOT_VERSION={}".format(robot_version))
        mass_file_path = os.path.expanduser(f"~/.config/lejuconfig/TotalMassV{robot_version}")
        # 检查文件是否存在
        if os.path.exists(mass_file_path):
            # 读取文件内容
            with open(mass_file_path, 'r') as file:
                content = file.read()
                print(f"Total MASS 质量为: {content}")
        else:
            print(bcolors.FAIL + "质量文件不存在 !!" + bcolors.ENDC)
    else:
        print("未找到 ROBOT_VERSION 变量")

    core_count = get_core_count()
    if core_count is not None:
        print("8 Number of CPU cores: {}".format(core_count))

    git_info = get_git_info()
    if git_info:
        # 公共信息
        if 'commit_hash' in git_info:
            print(f"程序提交版本: {git_info['commit_hash']}")

        # Git 仓库模式
        if 'commit_date' in git_info:
            print(f"程序提交日期: {git_info['commit_date']}")
        if 'commit_message' in git_info:
            print(f"程序提交信息: {git_info['commit_message']}")
        if 'branch' in git_info and git_info['branch']:
            print(f"程序所在分支: {git_info['branch']}")

        # .version 模式下的额外信息
        if 'tag' in git_info and git_info['tag']:
            print(f"程序 Tag: {git_info['tag']}")
        if 'remote' in git_info and git_info['remote']:
            print(f"程序远程仓库: {git_info['remote']}")

    else:
        print("❌ Failed to retrieve Git information.")
    
    dev_flag = 0
    if len(sys.argv) > 1:  # 至少有一个参数
        if "o" in sys.argv:
            dev_flag = 1
    
    while True:
        # 提示用户选择
        print(bcolors.BOLD + "请输入一个选项按回车(q回车退出)：" + bcolors.ENDC)
        print("0. 打开零点文件")
        print("1. 硬件连接检查")
        print("2. 打开imu上位机软件(接屏幕)")
        print("3. 测试imu(先编译)")
        print("a. 测试二指夹爪（Ctrl + C 退出）")
        print("c. 测试灵巧手") 
        print("f. 零点文件备份")
        print("k. 更新当前目录程序(注意：会重置文件内容，建议备份文件)")
        print("o. 打开开发者工具")

        # 获取用户输入的选项
        if(dev_flag):
            option = 'o'
            dev_flag = 0
        else:
            option = input("请输入选项编号：")

        # 如果用户选择退出，则退出循环
        if option == "q":
            print("已退出")
            exit()

        # 根据用户选择执行相应的操作
        if option == "o":
            secondary_menu()
            break
        elif option == "0":
            print(bcolors.HEADER + "###开始，打开零点文件###" + bcolors.ENDC)
            print("复制运行该行命令打开：code /home/lab/.config/lejuconfig/offset.csv")
            print(bcolors.HEADER + "###结束，打开零点文件###" + bcolors.ENDC)
            break
        elif option == "1":
            print(bcolors.HEADER + "###开始，硬件连接检查###" + bcolors.ENDC)
            usb_port()
            print(bcolors.HEADER + "###结束，硬件连接检查###" + bcolors.ENDC)
            break
        elif option == "2":
            print(bcolors.HEADER + "###正在打开imu上位机软件###" + bcolors.ENDC)
            imu_software()
            print(bcolors.HEADER + "###已打开imu上位机软件###" + bcolors.ENDC)
            break
        elif option == "3":
            print(bcolors.HEADER + "###开始，测试imu(Ctrl+C退出)###" + bcolors.ENDC)
            imu_test()
            print(bcolors.HEADER + "###结束，测试imu###" + bcolors.ENDC)
            break
        elif option == "a":
            print(bcolors.HEADER + "###开始，测试夹爪（Ctrl + C 退出）###" + bcolors.ENDC)
            leju_claw_test()
            print(bcolors.HEADER + "###结束，测试夹爪###" + bcolors.ENDC)
            break
        elif option == "c":
            print(bcolors.HEADER + "###开始，测试灵巧手###" + bcolors.ENDC)
            print(bcolors.OKCYAN + "先左右手一起握，然后依次握左手，握右手" + bcolors.ENDC)
            qiangnao_hand()
            print(bcolors.HEADER + "###结束，测试灵巧手###" + bcolors.ENDC)
            break
        elif option == "f":
            print(bcolors.HEADER + "###开始，文件备份###" + bcolors.ENDC)
            folder_backup()
            print(bcolors.HEADER + "###结束，文件备份###" + bcolors.ENDC)  
            break  
        elif option == "k":
            print(bcolors.HEADER + "###开始，更新当前目录程序###" + bcolors.ENDC)
            update_kuavo()
            print(bcolors.HEADER + "###结束，更新当前目录程序###" + bcolors.ENDC)
            break

        else:
            print(bcolors.FAIL + "无效的选项编号，请重新输入！\n" + bcolors.ENDC)
