import os
import sys
import time
import signal
import os
import pwd
from SimpleSDK import RUIWOTools

# Roban2机器人电机配置
def get_motor_config():
    return {
        "total_motors": 10,
        "motor_ids": [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A],
        "description": "Roban2 - 10个电机"
    }

ruiwo = RUIWOTools()
def get_home_path():
    sudo_user = os.getenv("SUDO_USER")
    if sudo_user:
        try:
            pw = pwd.getpwnam(sudo_user)
            path = os.path.join(pw.pw_dir, ".config/lejuconfig")
            return path
        except KeyError:
            pass
    else:
        uid = os.getuid()
        try:
            pw = pwd.getpwuid(uid)
            path = os.path.join(pw.pw_dir, ".config/lejuconfig")
            return path
        except KeyError:
            pass
    return ""

def get_dev_id():
    dev_id = input("\n请输入RUIWO电机ID号(注意电机限位不要堵转): ")
    try:
        return int(dev_id)
    except ValueError:
        print("无效的RUIWO电机ID，请输入有效的整数。")
        return get_dev_id()
def check_dev_id_in_config(dev_id):
    # 读取 config.yaml 文件
    if not os.path.exists(config_path):
        print(f"Config file {config_path} does not exist.")
        return
    with open(config_path, 'r') as file:
        config_lines = file.readlines()
    if not config_lines:
        print(f"Failed to read the config file or file is empty.")
        return
    dev_id_str = f'0x{dev_id:02X}'
    for i, line in enumerate(config_lines):
        if 'negtive_address:' in line:
            found_negtive_address = True
            # 查找下一个列表项的位置
            next_line_index = i + 1
            while next_line_index < len(config_lines) and not config_lines[next_line_index].strip().startswith('-'):
                next_line_index += 1
    if dev_id_str in config_lines[next_line_index]:
        return True

def modify_config_file(dev_id,action):
    # 读取 config.yaml 文件
    if not os.path.exists(config_path):
        print(f"Config file {config_path} does not exist.")
        return
    with open(config_path, 'r') as file:
        config_lines = file.readlines()
    if not config_lines:
        print(f"Failed to read the config file or file is empty.")
        return
    found_negtive_address = False
    dev_id_str = f'0x{dev_id:02X}'
    for i, line in enumerate(config_lines):
        if 'negtive_address:' in line:
            found_negtive_address = True
            next_line_index = i + 1
            while next_line_index < len(config_lines) and not config_lines[next_line_index].strip().startswith('-'):
                next_line_index += 1
            if action == "save":
                if dev_id_str not in config_lines[next_line_index]:
                    if config_lines[next_line_index].strip() == '- []':
                        config_lines[next_line_index] = f' - [{dev_id_str}]\n'
                    else:
                        config_lines[next_line_index] = ' ' + config_lines[next_line_index].strip()[:-1] + f', {dev_id_str}]\n'
                else:
                    print(f"RUIWO motor ID: {dev_id_str} is already in the negtive_address list.")
                    return
            elif action == "del":
                if dev_id_str in config_lines[next_line_index]:
                    new_line = config_lines[next_line_index].replace(f', {dev_id_str}', '').replace(f'{dev_id_str}, ', '').replace(f'{dev_id_str}', '')
                    config_lines[next_line_index] = new_line
            break
    if not found_negtive_address:
        print(f"negtive_address not found in {config_path}")
        return
    with open(config_path, 'w') as file:
        file.writelines(config_lines)

def signal_handler(sig, frame):
    print("\nMotor reset: ", DEV_ID)
    print("Close Canbus:", ruiwo.close_canbus())
    print('Exiting gracefully.')
    exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    
    path = get_home_path()
    print(f"[RUIWO motor]:config.yaml path: {path}")
    if not path:
        print("Failed to get home path.")
        exit(1)
    config_file = 'config.yaml'
    config_path = os.path.join(path,config_file)
    
    # 先尝试打开CAN总线
    open_canbus = ruiwo.open_canbus()
    if not open_canbus:
        print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")
        exit(1)
    print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")
    
    # 获取Roban2机器人电机配置
    motor_config = get_motor_config()
    
    print(f"\n=== Roban2机器人配置 ===")
    print(f"机器人类型: {motor_config['description']}")
    print(f"电机位置分布:")
    print(f"  左手: 电机ID 0x01-0x04 (1-4号)")
    print(f"  右手: 电机ID 0x05-0x08 (5-8号)")
    print(f"  头部: 电机ID 0x09-0x0A (9-10号)")
    print(f"可用电机ID: {[f'0x{id:02X}' for id in motor_config['motor_ids']]}")
    print(f"=======================")
    DEV_ID = 0
    while True:
        DEV_ID = get_dev_id()
        
        # 验证电机ID是否在配置范围内
        if DEV_ID not in motor_config['motor_ids']:
            print(f"\033[91m错误: 电机ID 0x{DEV_ID:02X} 不在Roban2机器人配置范围内!\033[0m")
            print(f"Roban2支持的电机ID: {[f'0x{id:02X}' for id in motor_config['motor_ids']]}")
            print("\033[91m请重新输入有效的电机ID。\033[0m")
            continue
        
        print(f"RUIWO motor ID: 0x{DEV_ID:02X}")
        print("Motor reset: ", ruiwo.enter_reset_state(DEV_ID))
        time.sleep(0.05)
        state = ruiwo.enter_motor_state(DEV_ID)
        print("Motor enter: ", state)
        time.sleep(0.05)
        x = state[1]
        for i in range(100):
            x = x + 0.003
            state = ruiwo.run_ptm_mode(DEV_ID, x, 0, 10, 3, 0)
            if state == False:
                print("Motor run failed.")
                exit(1)
            time.sleep(0.05)
        print("Motor position:", state[1])
        # 等待用户输入
        user_input = input(f"\n您想要将当前的电机ID 0x{DEV_ID:02X} 加入 config.yaml 的反转列表吗？电机当前如果反转请输入 'yes'，正转请输入 'no': ")
        if user_input.strip().lower() in ['yes', 'y']:
            # 用户输入 yes，表示电机反转
            if check_dev_id_in_config(DEV_ID):
                # 电机已经在反转列表中，无需操作
                print(f"\033[93m电机ID 0x{DEV_ID:02X} 已经在 negtive_address 列表中，保持反转设置\033[0m")
            else:
                # 电机不在反转列表中，添加到反转列表
                modify_config_file(DEV_ID, 'save')
                print(f"\033[92m电机ID 0x{DEV_ID:02X} 已添加到 negtive_address 列表，设置为反转\033[0m")
        else:
            # 用户输入 no，表示电机正转
            if check_dev_id_in_config(DEV_ID):
                # 电机在反转列表中，需要删除以恢复正转
                print(f"\033[93m电机ID 0x{DEV_ID:02X} 当前在 negtive_address 列表中，需要删除以恢复正转\033[0m")
                confirm = input("确认删除此电机ID以恢复正转方向? (y/n): ").strip().lower()
                if confirm in ['y', 'yes']:
                    modify_config_file(DEV_ID, 'del')
                    print(f"\033[92m电机ID 0x{DEV_ID:02X} 已从反转列表中删除，现在使用正转方向\033[0m")
                else:
                    print("取消删除操作，保持反转设置。")
            else:
                # 电机不在反转列表中，已经是正转状态
                print(f"\033[92m电机ID 0x{DEV_ID:02X} 当前使用正转方向，无需修改配置\033[0m")
        
        # 恢复位置
        print(f"\033[94m开始恢复电机位置...\033[0m")
        for i in range(100):
            x = x - 0.001
            state = ruiwo.run_ptm_mode(DEV_ID, x, 0, 10, 3, 0)
            if state == False:
                print("Motor run failed.")
                exit(1)
            time.sleep(0.05)
        print("Motor position:", state[1])
        print("Motor reset: ", ruiwo.enter_reset_state(DEV_ID))
        
        print(f"\033[92m=== 电机ID 0x{DEV_ID:02X} 配置完成 ===\033[0m")
