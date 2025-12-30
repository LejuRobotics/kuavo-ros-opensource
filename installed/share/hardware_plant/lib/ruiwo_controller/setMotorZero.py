from SimpleSDK import RUIWOTools
import yaml
import time
import os
import signal
import pwd
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

def get_head_id(config):
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
    joint_address_list = [DEV_ID13, DEV_ID14]
    return joint_address_list

def signal_handler(sig, frame):
    motor_set(1, joint_address_list)
    print("\n[RUIWO motor]:Close Canbus:", ruiwo.close_canbus())
    print('Exiting gracefully.')
    exit(0)

def motor_set(mode, joint_address_list):
    if mode == 0:
        for address in range(len(joint_address_list)):
            state_mode = ruiwo.enter_motor_state(joint_address_list[address])
            if isinstance(state_mode, list):
                print("[RUIWO motor]:ID:",joint_address_list[address], "Enable: [Succeed]")
            else:
                print("[RUIWO motor]:ID:",joint_address_list[address], "Enable:","[",state_mode,"]")
            time.sleep(0.05)
    else:
        for address in range(len(joint_address_list)):
            state = ruiwo.enter_reset_state(joint_address_list[address])
            if isinstance(state, list):
                print("[RUIWO motor]:ID:",joint_address_list[address], "Disable: [Succeed]")
            else:
                print("[RUIWO motor]:ID:",joint_address_list[address], "Disable:","[",state,"]")
            time.sleep(0.05)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    path = get_home_path()
    print(f"[RUIWO motor]:config.yaml path: {path}")
    if not path:
        print("Failed to get home path.")
        exit(1)
    config_file = 'config.yaml'
    config_path = os.path.join(path,config_file)
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    # 多圈编码器模式
    try:
        multi_turn_encoder_mode = config['Multi-turn_Encoder_mode']
    except KeyError:
        multi_turn_encoder_mode = False
    if not multi_turn_encoder_mode:
        joint_address_list = get_dev_id(config)
    else:
        joint_address_list = get_head_id(config)
    open_canbus = ruiwo.open_canbus()
    if not open_canbus:
        print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")
        exit(1)
    print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")
    joint_address_list_reload = []
    for address in range(len(joint_address_list)):
        state = ruiwo.enter_reset_state(joint_address_list[address])
        if isinstance(state, list):
            joint_address_list_reload.append(joint_address_list[address])
            print("[RUIWO motor]:ID:",joint_address_list[address], "Disable: [Succeed]")
        else:
            print("[RUIWO motor]:ID:",joint_address_list[address], "Disable:","[",state,"]")
        time.sleep(0.05)
    del joint_address_list[len(joint_address_list_reload):]
    joint_address_list[:] = joint_address_list_reload
    # motor_set(0, joint_address_list)
    # for i in range(5):
    for address in range(len(joint_address_list)):
        for j in range(5):
            state_zero = ruiwo.set_zero_positon(joint_address_list[address])
            time.sleep(0.05)
        print("[RUIWO motor]:ID:",joint_address_list[address],"Set zero positon:",state_zero)
        time.sleep(0.05)
        state_ptm = ruiwo.enter_reset_state(joint_address_list[address])
        print("[RUIWO motor]:ID:",joint_address_list[address],"Motor state:",state_ptm)
        if abs(state_zero[1] - state_ptm[1]) > 0.1:
            motor_set(1, joint_address_list)
            print("[RUIWO motor]:Close Canbus:",ruiwo.close_canbus())
            print(f"[ERROR]: State difference for ID {joint_address_list[address]} exceeds '0.1' Exiting.")
            exit(1)
    time.sleep(0.05)
    motor_set(1, joint_address_list)
    print("[RUIWO motor]:Close Canbus:",ruiwo.close_canbus())
    exit(0)