import os
import sys
import time
import signal
import os
import pwd
from SimpleSDK import RUIWOTools
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
    dev_id = input("Please enter the RUIWO motor ID number(注意电机限位不要堵转): ")
    try:
        return int(dev_id)
    except ValueError:
        print("Invalid RUIWO motor ID. Please enter a valid int number.")
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
    open_canbus = ruiwo.open_canbus()
    if not open_canbus:
        print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")
        exit(1)
    print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")
    DEV_ID = 0
    while True:
        DEV_ID = get_dev_id()
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
        print("Motor position:",state[1])
        # 等待用户输入
        user_input = input("Do you want to add 'RUIWO motor ID' to negtive_address in config.yaml? 电机当前如果反转请输入yes,正转输入no: ")
        if user_input.strip().lower() in ['yes', 'y']:
            modify_config_file(DEV_ID, 'save')
        else:
            if check_dev_id_in_config(DEV_ID):
                print(f"RUIWO motor ID: 0x{DEV_ID:02X} already exists in negtive_address.")
                user_action = input("Enter 'del' to delete this ID, or 'save' to keep it: ").strip().lower()
                if user_action == 'del':
                    modify_config_file(DEV_ID, 'del')
                elif user_action == 'save':
                    print("ID has been kept.")
                else:
                    print("Invalid input. No changes made.")
            else:
                print(f"RUIWO motor ID: 0x{DEV_ID:02X} not found in negtive_address. No changes made.")
        for i in range(100):
            x = x - 0.001
            state = ruiwo.run_ptm_mode(DEV_ID, x, 0, 10, 3, 0)
            if state == False:
                print("Motor run failed.")
                exit(1)
            time.sleep(0.05)
        print("Motor position:",state[1])
        print("Motor reset: ", ruiwo.enter_reset_state(DEV_ID))
        
