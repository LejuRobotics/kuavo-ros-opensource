import os
import yaml
import time
from SimpleSDK import RUIWOTools
import sys
import threading

# 缺失的电机ID
MISSING_MOTOR_IDS = {5, 6, 11, 12}

# 相关参数
MOTION_DURATION = 2  # 每个动作的执行时间（秒）
POS_KP = 30
POS_KD = 5

# 更新频率
UPDATE_FREQUENCY = 50
UPDATE_INTERVAL = 1 / UPDATE_FREQUENCY

# 定义零点文件路径
def get_zero_path():
    return '/home/lab/.config/lejuconfig/arms_zero.yaml'

# 读取零点位置
def read_zero_positions():
    zeros_path = get_zero_path()
    if os.path.exists(zeros_path):
        with open(zeros_path, 'r') as file:
            zeros_config = yaml.safe_load(file)
        return zeros_config['arms_zero_position'][:12]  # 只取前 12 个值
    else:
        print("[RUIWO motor]:Warning: zero_position file does not exist, will use 0 as zero value.")
        return [0.0] * 12

# 获取用户输入的测试时长
def get_test_duration(cycle_time):
    while True:
        try:
            duration = int(input(f"\n请输入测试时长（大于 {cycle_time:.1f} 秒）："))
            if duration >= cycle_time:
                return duration
            else:
                print(f"输入的时长小于一个完整动作周期的时间（大于 {cycle_time:.1f} 秒），请重新输入。")
        except ValueError:
            print("输入无效，请输入一个整数。")

# 获取当前时间戳
def get_timestamp():
    return time.strftime("%H:%M:%S", time.localtime()) + f".{int(time.time() % 1 * 1000):03d}"

# 读取电机正反转配置
def read_motor_reverse_config():
    config_path = '/home/lab/.config/lejuconfig/config.yaml'
    if os.path.exists(config_path):
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        return config.get('negtive_address', [])
    else:
        print("[RUIWO motor]:Warning: config.yaml file does not exist, no motor reverse config will be applied.")
        return []

# 读取关节 ID 列表
def read_joint_ids():
    config_path = '/home/lab/.config/lejuconfig/config.yaml'
    if os.path.exists(config_path):
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        return list(config.get('address', {}).values())[:12]  # 只取前 12 个值
    else:
        print("[RUIWO motor]:Warning: config.yaml file does not exist, using default joint IDs.")
        return [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C]

# 读取机器人的版本号
def get_robot_version():
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
                if version in ROBOT_VERSION_MAPPING:
                    return version
                else:
                    print(f"[RUIWO motor]:Warning: ROBOT_VERSION '{version}' 不在映射表中。")
                    break
    print("[RUIWO motor]:Warning: ROBOT_VERSION 未找到或无效，需要手动选择模式。")
    return None

# ROBOT_VERSION 到机器人模式的映射
ROBOT_VERSION_MAPPING = {
    # 短手版本
    "40": "short",
    "41": "short",
    "42": "short",
    
    # 长手版本
    "43": "long",
    "44": "long",
    "45": "long",
    "46": "long",
    "47": "long",
    "48": "long",
    "49": "long",
    
    # Roban2机器人版本
    "13": "roban2",
    "14": "roban2",
    
    # 特殊版本号（长手）
    "100045": "long",
    "100049": "long",
}

# 获取机器人模式（自动映射 + 电机检测）
def get_robot_mode(robot_version, enable_results):
    # 检查是否为roban2机器人
    if robot_version in ["13", "14"]:
        print(f"检测到Roban2机器人（ROBOT_VERSION={robot_version}），将执行对应的动作序列")
        return "roban2"
    
    # 通过电机使能结果检测是否为4Pro标准版
    failed_ids = [dev_id for dev_id, success in enable_results if not success]
    is_FourPro_Standard = set(failed_ids) == MISSING_MOTOR_IDS
    
    if is_FourPro_Standard:
        print("检测到4Pro标准版，假手（缺少5、6、11、12号电机），将执行对应的动作序列")
        return "FourPro_Standard"
    
    # 若ROBOT_VERSION存在且在映射表中，自动选择对应模式
    if robot_version and robot_version in ROBOT_VERSION_MAPPING:
        auto_mode = ROBOT_VERSION_MAPPING[robot_version]
        print(f"根据 ROBOT_VERSION = {robot_version}，自动选择动作：{auto_mode}")
        
        # 根据模式返回对应的描述
        if auto_mode == "long":
            print("执行长手版动作序列。")
        elif auto_mode == "short":
            print("执行短手版动作序列。")
        elif auto_mode == "FourPro_Standard":
            print("执行4Pro假手机器人动作序列。")
        elif auto_mode == "roban2":
            print("执行Roban2机器人动作序列。")
        
        return auto_mode
    
    # 若无法自动确定，则手动选择
    print(f"\n无法根据 ROBOT_VERSION = {robot_version} 自动确定模式，请手动选择：")
    print("1. 长手模式")
    print("2. 短手模式")
    print("3. Roban2模式")
    while True:
        choice = input("请输入选项（1/2/3）：").strip()
        if choice == '1':
            return "long"
        elif choice == '2':
            return "short"
        elif choice == '3':
            return "roban2"
        else:
            print("输入无效，请重新选择！")
    return None

# 定义长手和短手的动作
long_arm_actions = [
    [0.23, 0.00, 0.00, 0.00, 0.00, -0.24],
    [1.30, 1.00, -1.40, 1.30, 0.40, -0.60],
    [2.00, -0.50, -0.30, 1.50, 0.80, 0.70],
    [1.10, -2.00, -2.30, -1.50, 0.00, -0.24],
    [0.23, -0.40, -1.30, 1.00, -0.80, 0.70],
    [1.25, 0.00, -1.90, 0.70, 0.00, -0.60],
    [0.23, 0.00, 0.00, 0.00, 0.00, -0.24]
]

short_arm_actions = [
    [0.00, 0.00, 0.00, 0.00, -0.10, 0.00],
    [0.85, -1.30, -0.20, 1.40, 1.30, -1.30],
    [1.90, 0.40, -0.50, -1.40, 0.52, -0.80],
    [1.31, 1.30, -0.90, 0.00, -0.10, 1.00],
    [1.90, 0.31, -1.30, -1.40, -0.72, 0.00],
    [1.40, 0.90, -0.60, 0.90, -1.50, -1.00],
    [0.40, 0.20, 0.00, 0.20, -0.10, 0.00]
]

# 4Pro标准版假手动作
FourPro_Standard_actions = [
    [0.23, 0.00, 0.00, 0.00, 0.00, 0.00],
    [1.30, 1.00, -1.40, 1.30, 0.00, 0.00],
    [2.00, -0.50, -0.30, 1.50, 0.00, 0.00],
    [1.10, -2.00, -2.30, -1.50, 0.00, 0.00],
    [0.23, -0.40, -1.30, 1.00, 0.00, 0.00],
    [1.25, 0.00, -1.90, 0.70, 0.00, 0.00],
    [0.23, 0.00, 0.00, 0.00, 0.00, 0.00]
]

# Roban2左手动作
roban2_left_arm_actions = [
    [0.00, 0.00, 0.00, 0.00],
    [1.00, 1.00, -1.00, 1.50],
    [1.40, 1.80, -1.50, 0.20],
    [0.00, 2.50, 0.00, 1.60],
    [-0.60, 1.30, 1.50, 2.00],
    [-1.40, 0.80, 0.90, 1.00],
    [0.00, 0.00, 0.00, 0.00]
]

# 读取机器人版本号
robot_version = get_robot_version()
input("请确认机器型号无误后，按回车键继续...")

# 读取关节配置
joint_ids = read_joint_ids()
zero_positions = read_zero_positions()
reverse_addresses = read_motor_reverse_config()

# 初始化硬件
ruiwo = RUIWOTools()
open_canbus = ruiwo.open_canbus()
if not open_canbus:
    print("[RUIWO motor]:Canbus状态:", "[", open_canbus, "]")
    exit(1)
print("[RUIWO motor]:Canbus状态:", "[", open_canbus, "]")

# 使能电机并检测模式
enable_results = []
enable_all_success = True

# 根据机器人版本决定使能哪些电机
if robot_version in ["13", "14"]:
    motors_to_enable = joint_ids[:8]
    print(f"Roban2模式：使能1-8号电机 {motors_to_enable}")
else:
    # 其他模式：使能所有电机
    motors_to_enable = joint_ids

for dev_id in motors_to_enable:
    state = ruiwo.enter_motor_state(dev_id)
    success = isinstance(state, list)
    enable_results.append((dev_id, success))
    print(f"[RUIWO motor]:ID: {dev_id} 使能:  [{state if success else '失败'}]")
    enable_all_success = enable_all_success and success

# 确定机器人模式
robot_mode = get_robot_mode(robot_version, enable_results)
if robot_mode is None:
    print("[RUIWO motor]:错误：无法确定机器人模式，程序退出。")
    exit(1)

# 根据机器人模式选择动作序列
if robot_mode == "FourPro_Standard":
    base_actions = FourPro_Standard_actions
elif robot_mode == "long":
    base_actions = long_arm_actions
elif robot_mode == "short":
    base_actions = short_arm_actions
elif robot_mode == "roban2":
    base_actions = roban2_left_arm_actions
else:
    pass

# 生成左右手完整动作
full_base_actions = []
if robot_mode == "roban2":
    left_joint_ids = joint_ids[:4]  # 取前4个电机ID
    right_joint_ids = joint_ids[4:8]  # 取5-8号电机ID
    
    for action in base_actions:
        # 处理左手动作
        left_action = []
        for i in range(len(action)):
            left_id = left_joint_ids[i]
            # 根据电机配置应用反转逻辑
            if left_id in reverse_addresses:
                left_action.append(-action[i])
            else:
                left_action.append(action[i])
        
        # 处理右手动作
        right_action = []
        for i in range(len(action)):
            right_id = right_joint_ids[i]
            mirrored_value = -action[i]
            if right_id in reverse_addresses:
                right_action.append(-mirrored_value)
            else:
                right_action.append(mirrored_value)
        
        full_action = left_action + right_action
        full_base_actions.append(full_action)
else:
    # 其他模式：使用完整的左右手动作
    left_joint_ids = joint_ids[:6]
    right_joint_ids = joint_ids[6:]
    for action in base_actions:
        left_action = action
        right_action = []
        for i in range(len(left_action)):
            left_id = left_joint_ids[i]
            right_id = right_joint_ids[i]
            # 镜像逻辑：根据电机配置取反
            if (left_id in reverse_addresses) ^ (right_id in reverse_addresses):
                right_action.append(-action[i])
            elif (left_id in reverse_addresses) and (right_id in reverse_addresses):
                right_action.append(-action[i])
            else:
                right_action.append(-action[i])
        full_action = left_action + right_action
        full_base_actions.append(full_action)

# 计算动作周期
cycle_time = len(base_actions) * MOTION_DURATION
test_duration = get_test_duration(cycle_time)
start_time = time.perf_counter()

# 标记是否检测到电机失能
motor_disabled = False

# 标记是否提前结束程序
early_exit = False

# 标记是否在当前周期结束后退出
exit_after_cycle = False

# 电机丢帧统计
motor_frame_loss_stats = {}  # 记录每个电机的丢帧次数
motor_total_commands = {}    # 记录每个电机发送的总命令数

# 提示用户可以输入 'q' 提前结束程序
print("\033[92m提示：在执行过程中，输入 'q' 并回车，可以在当前完整动作周期完成后提前结束程序。\033[0m")

# 创建一个线程安全的标志变量
early_exit_event = threading.Event()

# 监听键盘输入
def listen_for_exit():
    global early_exit, exit_after_cycle
    while not early_exit:
        user_input = input()
        if user_input.strip().lower() == 'q':
            print(f"{get_timestamp()} 用户请求提前结束程序，将在当前完整动作周期完成后停止。")
            early_exit_event.set()
            exit_after_cycle = True
            early_exit = True

# 启动监听线程
listen_thread = threading.Thread(target=listen_for_exit)
listen_thread.daemon = True  # 设置为守护线程，主线程结束时自动退出
listen_thread.start()

# 电机状态检查（跳过缺失电机）
def check_motor_status(joint_ids, robot_mode):
    disabled_motors = []  # 记录失能的电机ID
    # 根据机器人模式确定需要检查的电机ID（跳过缺失电机）
    if robot_mode == "FourPro_Standard":
        motors_to_check = [dev_id for dev_id in joint_ids if dev_id not in MISSING_MOTOR_IDS]
    elif robot_mode == "roban2":
        motors_to_check = joint_ids[:8]
    else:
        motors_to_check = joint_ids
    
    for dev_id in motors_to_check:
        state = ruiwo.enter_motor_state(dev_id)
        if isinstance(state, list):
            # 检查故障码是否为15（失能状态）
            if state[-2] == 15:  # 倒数第二个元素为故障码
                disabled_motors.append(dev_id)  # 记录失能的电机ID
        else:
            print(f"\033[91m电机 {dev_id} 状态获取失败！\033[0m")
            return False, disabled_motors  # 状态获取失败，直接返回False
    
    # 若有失能的电机，输出失能的电机列表
    if disabled_motors:
        print(f"\033[91m以下电机失能：{disabled_motors}\033[0m")
        return False, disabled_motors  # 返回False表示有电机失能
    
    return True, []  # 所有电机状态正常

while True:
    current_time = time.perf_counter()
    elapsed_time = current_time - start_time
    remaining_time = test_duration - elapsed_time

    # 输出剩余时间
    print(f"{get_timestamp()} 总剩余时间：{remaining_time:.2f} 秒")

    # 检查是否开始新的完整周期
    if elapsed_time % cycle_time < MOTION_DURATION:
        # 判断剩余时间是否足够完成一个完整周期
        if remaining_time < cycle_time:
            print(f"{get_timestamp()} 剩余时间不足完成一个动作周期，提前结束。")
            break
            
        # 检查是否需要在当前周期结束后退出
        if exit_after_cycle:
            print(f"{get_timestamp()} 完成当前周期后将退出程序...")
            
        # 检查所有电机状态，是否出现失能情况
        if not motor_disabled:
            status, disabled_motors = check_motor_status(joint_ids, robot_mode)
            if not status:
                motor_disabled = True
                print(f"\033[91m检测到电机失能，停止当前动作！请检查以下电机：{disabled_motors}。检查完毕后，输入两次 ['c' + 回车] 失能所有电机并退出程序。\033[0m")
                while True:
                    user_input = input().strip().lower()
                    if user_input == 'c':
                        # 失能所有关节电机（跳过缺失电机）
                        for dev_id in joint_ids:
                            if robot_mode == "FourPro_Standard" and dev_id in MISSING_MOTOR_IDS:
                                continue
                            state = ruiwo.enter_reset_state(dev_id)
                            if isinstance(state, list):
                                print(f"{get_timestamp()} [RUIWO motor]:ID: {dev_id} Disable:  [Succeed]")
                            else:
                                print(f"{get_timestamp()} [RUIWO motor]:ID: {dev_id} Disable:  [{state}]")
                        # 关闭CAN总线
                        close_canbus = ruiwo.close_canbus()
                        if close_canbus:
                            print(f"{get_timestamp()} [RUIWO motor]:Canbus status: [Close]")
                        sys.exit(0)
        else:
            print(f"{get_timestamp()} 等待用户检查电机...")

    # 检查是否应该退出：时间到或用户请求且完成了当前周期
    if (elapsed_time >= test_duration) or (exit_after_cycle and (elapsed_time % cycle_time >= cycle_time - MOTION_DURATION)):
        break

    # 根据实际时间计算当前应该执行的关键帧索引
    current_frame_index = int(elapsed_time // MOTION_DURATION) % len(full_base_actions)
    next_frame_index = (current_frame_index + 1) % len(full_base_actions)

    current_positions = full_base_actions[current_frame_index]
    target_positions = full_base_actions[next_frame_index]

    # 检查长度是否匹配
    expected_length = len(joint_ids[:8]) if robot_mode == "roban2" else len(joint_ids)
    if len(current_positions) != expected_length or len(target_positions) != expected_length:
        raise ValueError(f"动作 {current_frame_index + 1} 的位置列表长度不匹配。当前长度: {len(current_positions)}，目标长度: {len(target_positions)}，期望长度: {expected_length}")
    steps = MOTION_DURATION * UPDATE_FREQUENCY  # MOTION_DURATION 秒内发送的步数
    step_start_time = elapsed_time % MOTION_DURATION

    if not motor_disabled:
        for step in range(int(steps)):
            loop_start = time.perf_counter()
            if robot_mode == "roban2":
                for joint_index, dev_id in enumerate(joint_ids[:8]):
                    # 计算当前位置到目标位置的插值
                    interpolated_pos = current_positions[joint_index] + (target_positions[joint_index] - current_positions[joint_index]) * (step / steps)
                    zero_position = zero_positions[joint_index]  # 电机对应零点位置索引
                    compensated_pos = interpolated_pos + zero_position  # 应用零点补偿
                    state = ruiwo.run_ptm_mode(dev_id, compensated_pos, 0, POS_KP, POS_KD, 0)
                    
                    # 统计电机命令和丢帧情况
                    if dev_id not in motor_total_commands:
                        motor_total_commands[dev_id] = 0
                        motor_frame_loss_stats[dev_id] = 0
                    
                    motor_total_commands[dev_id] += 1
                    
                    if isinstance(state, list):
                        pass
                    else:
                        # 记录丢帧
                        motor_frame_loss_stats[dev_id] += 1
                        print(f"{get_timestamp()} ID: {dev_id} Run ptm mode:  [{state}]")
            else:
                for joint_index, dev_id in enumerate(joint_ids):
                    # 跳过缺失的电机
                    if robot_mode == "FourPro_Standard" and dev_id in MISSING_MOTOR_IDS:
                        continue
                    # 计算当前位置到目标位置的插值
                    interpolated_pos = current_positions[joint_index] + (target_positions[joint_index] - current_positions[joint_index]) * (step / steps)
                    zero_position = zero_positions[joint_index]
                    compensated_pos = interpolated_pos + zero_position  # 应用零点补偿
                    state = ruiwo.run_ptm_mode(dev_id, compensated_pos, 0, POS_KP, POS_KD, 0)
                    
                    # 统计电机命令和丢帧情况
                    if dev_id not in motor_total_commands:
                        motor_total_commands[dev_id] = 0
                        motor_frame_loss_stats[dev_id] = 0
                    
                    motor_total_commands[dev_id] += 1
                    
                    if isinstance(state, list):
                        pass
                    else:
                        # 记录丢帧
                        motor_frame_loss_stats[dev_id] += 1
                        print(f"{get_timestamp()} ID: {dev_id} Run ptm mode:  [{state}]")

            loop_end = time.perf_counter()
            elapsed_time = loop_end - loop_start
            remaining_time = UPDATE_INTERVAL - elapsed_time
            if remaining_time > 0:
                time.sleep(remaining_time)

        print(f"{get_timestamp()} 动作 {current_frame_index + 1} 执行完成，开始向位置 {next_frame_index + 1} 运动")

# 在程序结束时返回到零点位置（跳过缺失电机）
print(f"{get_timestamp()} 正在返回到零点位置...")
if robot_mode == "roban2":
    for joint_index, dev_id in enumerate(joint_ids[:8]):
        zero_position = zero_positions[joint_index]  # 电机对应零点位置索引
        state = ruiwo.run_ptm_mode(dev_id, zero_position, 0, POS_KP, POS_KD, 0)
        
        # 统计返回零点时的命令和丢帧情况
        if dev_id not in motor_total_commands:
            motor_total_commands[dev_id] = 0
            motor_frame_loss_stats[dev_id] = 0
        
        motor_total_commands[dev_id] += 1
        
        if isinstance(state, list):
            pass
        else:
            # 记录丢帧
            motor_frame_loss_stats[dev_id] += 1
            print(f"{get_timestamp()} ID: {dev_id} 返回零点：[{state}]")
else:
    # 其他模式：使用完整的电机控制
    for joint_index, dev_id in enumerate(joint_ids):
        if robot_mode == "FourPro_Standard" and dev_id in MISSING_MOTOR_IDS:
            continue
        zero_position = zero_positions[joint_index]
        state = ruiwo.run_ptm_mode(dev_id, zero_position, 0, POS_KP, POS_KD, 0)
        
        # 统计返回零点时的命令和丢帧情况
        if dev_id not in motor_total_commands:
            motor_total_commands[dev_id] = 0
            motor_frame_loss_stats[dev_id] = 0
        
        motor_total_commands[dev_id] += 1
        
        if isinstance(state, list):
            pass
        else:
            # 记录丢帧
            motor_frame_loss_stats[dev_id] += 1
            print(f"{get_timestamp()} ID: {dev_id} 返回零点：[{state}]")

# 等待返回零点动作完成
time.sleep(MOTION_DURATION)

# 失能所有关节电机（跳过缺失电机）
print(f"{get_timestamp()} 失能所有关节电机...")
if robot_mode == "roban2":
    for dev_id in joint_ids[:8]:
        state = ruiwo.enter_reset_state(dev_id)
        if isinstance(state, list):
            print(f"{get_timestamp()} [RUIWO motor]:ID: {dev_id} Disable:  [Succeed]")
        else:
            print(f"{get_timestamp()} [RUIWO motor]:ID: {dev_id} Disable:  [{state}]")
else:
    for dev_id in joint_ids:
        if robot_mode == "FourPro_Standard" and dev_id in MISSING_MOTOR_IDS:
            continue
        state = ruiwo.enter_reset_state(dev_id)
        if isinstance(state, list):
            print(f"{get_timestamp()} [RUIWO motor]:ID: {dev_id} Disable:  [Succeed]")
        else:
            print(f"{get_timestamp()} [RUIWO motor]:ID: {dev_id} Disable:  [{state}]")

# 统计报告
def generate_motor_report():
    print(f"\n{'='*30}")
    print("电机丢帧统计")
    print(f"{'='*30}")
    
    # 计算总执行时间
    total_test_time = time.perf_counter() - start_time
    print(f"总执行时间: {total_test_time:.2f} 秒")
    print()
    
    # 统计有丢帧的电机
    motors_with_frame_loss = []
    total_frame_loss = 0
    total_commands = 0
    
    # 先统计所有数据
    for dev_id in sorted(motor_total_commands.keys()):
        total_cmd = motor_total_commands[dev_id]
        frame_loss = motor_frame_loss_stats[dev_id]
        
        total_commands += total_cmd
        total_frame_loss += frame_loss
        
        if frame_loss > 0:
            motors_with_frame_loss.append(dev_id)
    
    # 只输出有丢帧的电机
    if motors_with_frame_loss:
        print("电机丢帧统计:")
        print("-" * 40)
        print(f"{'电机ID':<8} {'总命令数':<10} {'丢帧次数':<10} {'丢帧率':<10}")
        print("-" * 40)
        
        for dev_id in sorted(motors_with_frame_loss):
            total_cmd = motor_total_commands[dev_id]
            frame_loss = motor_frame_loss_stats[dev_id]
            loss_rate = (frame_loss / total_cmd * 100) if total_cmd > 0 else 0
            print(f"{dev_id:<8} {total_cmd:<10} {frame_loss:<10} {loss_rate:<9.2f}%")
        
        print("-" * 40)
        print(f"总计: {total_commands} 个命令, {total_frame_loss} 次丢帧")
        if total_commands > 0:
            overall_loss_rate = (total_frame_loss / total_commands * 100)
            print(f"总体丢帧率: {overall_loss_rate:.2f}%")
        
        print()
        print(f"出现丢帧的电机: {sorted(motors_with_frame_loss)}")
        print(f"丢帧电机数量: {len(motors_with_frame_loss)} 个")
    else:
        print("\033[92m所有电机运行正常，无丢帧现象\033[0m")
    
    print(f"{'='*50}")

# 生成统计报告
generate_motor_report()

# 关闭CAN总线
close_canbus = ruiwo.close_canbus()
if close_canbus:
    print(f"{get_timestamp()} [RUIWO motor]:Canbus status: [Close]")