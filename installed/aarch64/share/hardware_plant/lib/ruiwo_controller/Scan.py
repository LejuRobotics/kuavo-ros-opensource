from SimpleSDK import RUIWOTools
import yaml
import os
import signal
import pwd
import time
ruiwo = RUIWOTools()
def get_dev_id(config):
    DEV_ID1 = config['address']['Left_joint_arm_1']
    DEV_ID2 = config['address']['Left_joint_arm_2']
    DEV_ID3 = config['address']['Left_joint_arm_3']
    DEV_ID4 = config['address']['Left_joint_arm_4']
    DEV_ID5 = config['address']['Left_joint_arm_5']
    DEV_ID6 = config['address']['Left_joint_arm_6']
    DEV_ID7 = config['address']['Right_joint_arm_1']
    DEV_ID8 = config['address']['Right_joint_arm_2']
    DEV_ID9 = config['address']['Right_joint_arm_3']
    DEV_ID10 = config['address']['Right_joint_arm_4']
    DEV_ID11 = config['address']['Right_joint_arm_5']
    DEV_ID12 = config['address']['Right_joint_arm_6']
    DEV_ID13 = config['address']['Head_joint_low']
    DEV_ID14 = config['address']['Head_joint_high']
    joint_address_list = [DEV_ID1, DEV_ID2, DEV_ID3, DEV_ID4, DEV_ID5, DEV_ID6, 
                        DEV_ID7, DEV_ID8, DEV_ID9, DEV_ID10, DEV_ID11, DEV_ID12, 
                        DEV_ID13, DEV_ID14]
    return joint_address_list

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

def signal_handler(sig, frame):
    print("\n[RUIWO motor]:Close Canbus:", ruiwo.close_canbus())
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
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
    except FileNotFoundError:
        print("Config file not found.")
        exit(1)
    except yaml.YAMLError as e:
        print("Error reading config file:", e)
        exit(1)
    joint_address_list = get_dev_id(config)
    open_canbus = ruiwo.open_canbus()
    if not open_canbus:
        print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")
        exit(1)
    print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")
    for address, dev_id in enumerate(joint_address_list, start=1):
        if dev_id == 0:
            print(f"[RUIWO motor]:ID: {address} Has no motor")
            continue
        else:
            state = ruiwo.enter_reset_state(dev_id)
        if isinstance(state, list):
            print(f"[RUIWO motor]:ID: {dev_id} Connection")
        else:
            print(f"[RUIWO motor]:ID: {dev_id} Disconnected")
        time.sleep(0.05)
    print("[RUIWO motor]:Close Canbus:", ruiwo.close_canbus())
    exit(0)