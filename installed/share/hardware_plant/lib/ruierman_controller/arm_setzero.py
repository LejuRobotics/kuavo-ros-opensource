import os
import yaml
import time
from SimpleSDK import RUIWOTools

# 定义零点文件路径
def get_zero_path():
    return '/home/lab/.config/lejuconfig/arms_zero.yaml'

# 读取零点位置
def read_zero_positions():
    zeros_path = get_zero_path()
    if os.path.exists(zeros_path):
        with open(zeros_path, 'r') as file:
            zeros_config = yaml.safe_load(file)
        return zeros_config['arms_zero_position']
    else:
        print("[RUIWO motor]:Warning: zero_position file does not exist, will use 0 as zero value.")
        return [0.0] * 14

# 读取电机地址配置
def read_motor_address_config():
    config_path = '/home/lab/.config/lejuconfig/config.yaml'
    if os.path.exists(config_path):
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        return config.get('address', {})
    else:
        print("[RUIWO motor]:Warning: config.yaml file does not exist, no motor address config will be applied.")
        return {}

# 创建对象
ruiwo = RUIWOTools()

# 打开CAN总线
open_canbus = ruiwo.open_canbus()
if not open_canbus:
    print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")
    exit(1)
print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")

# 读取电机地址配置
motor_addresses = read_motor_address_config()

# 遍历每个电机地址
for joint_name, dev_id in motor_addresses.items():
    if 'arm' in joint_name.lower():
        print(f"正在处理关节: {joint_name}, 电机ID: {hex(dev_id)}")

        # 设置电机零点
        state = ruiwo.set_zero_positon(dev_id)
        if isinstance(state, list):
            print(f"[RUIWO motor]:ID: {dev_id} Set zero position:  [Succeed], Motor info: {state}")
            zero_position = state[1]
            print("已设置好当前位置为零点：", zero_position)
        else:
            print(f"[RUIWO motor]:ID: {dev_id} Set zero position:  [{state}]")

        # 使能电机
        state = ruiwo.enter_motor_state(dev_id)
        if isinstance(state, list):
            print(f"[RUIWO motor]:ID: {dev_id} Enable:  [Succeed]")
        else:
            print(f"[RUIWO motor]:ID: {dev_id} Enable:  [{state}]")

        # 失能电机
        state = ruiwo.enter_reset_state(dev_id)
        if isinstance(state, list):
            print(f"[RUIWO motor]:ID: {dev_id} Disable:  [Succeed]")
        else:
            print(f"[RUIWO motor]:ID: {dev_id} Disable:  [{state}]")

# 关闭CAN总线
close_canbus = ruiwo.close_canbus()
if close_canbus:
    print("[RUIWO motor]:Canbus status:","[ Close ]")