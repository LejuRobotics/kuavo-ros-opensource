#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import time
import signal
import threading
import math
import yaml
import pwd
import shutil  
import numpy as np
import select
import termios
import tty
from SimpleSDK import RUIWOTools
from enum import Enum

current_path =os.path.dirname(os.path.abspath(__file__))
sys.path.append('/usr/lib/python3/dist-packages')
# 控制周期
dt=0.004
# 插值规划的速度
max_speed = 4
velocity_factor = 0.01
DISABLE_ADDRESS = 0x00

class RuiwoErrCode(Enum):
    """
    Revo电机故障码
    一旦驱动板检测到故障，将会从 Motor State 自动切回 Rest State 以保护驱动器和电机
    在排除异常情况后，发送 Enter Rest State 命令清除故障码，
    再发送 Enter Motor State 命令让电机重新恢复运行
    See Details:《Motorevo Driver User Guide v0.2.2》- 6.5.1 反馈帧格式 - 故障码
    """
    NO_FAULT = 0x00                                    # 无故障
    DC_BUS_OVER_VOLTAGE = 0x01                         # 直流母线电压过压
    DC_BUS_UNDER_VOLTAGE = 0x02                         # 直流母线电压欠压
    ENCODER_ANGLE_FAULT = 0x03                         # 编码器电角度故障
    DRV_DRIVER_FAULT = 0x04                             # DRV 驱动器故障
    DC_BUS_CURRENT_OVERLOAD = 0x05                      # 直流母线电流过流
    MOTOR_A_PHASE_CURRENT_OVERLOAD = 0x06               # 电机 A 相电流过载
    MOTOR_B_PHASE_CURRENT_OVERLOAD = 0x07               # 电机 B 相电流过载
    MOTOR_C_PHASE_CURRENT_OVERLOAD = 0x08               # 电机 C 相电流过载
    DRIVER_BOARD_OVERHEAT = 0x09                        # 驱动板温度过高
    MOTOR_WINDING_OVERHEAT = 0x0A                       # 电机线圈过温
    ENCODER_FAILURE = 0x0B                              # 编码器故障
    CURRENT_SENSOR_FAILURE = 0x0C                       # 电流传感器故障
    OUTPUT_ANGLE_OUT_OF_RANGE = 0x0D                    # 输出轴实际角度超过通信范围：CAN COM Theta MIN ~ CAN COM Theta MAX
    OUTPUT_SPEED_OUT_OF_RANGE = 0x0E                    # 输出轴速度超过通信范围 CAN COM Velocity MIN ~ CAN COM Velocity MAX

    # WARNING: 大于 128 的故障码为开机自检检测出的故障码，无法用该方法清除
    ABS_ENCODER_OFFSET_VERIFICATION_FAILURE = 0x81     # 离轴/对心多圈绝对值编码器接口帧头校验失败
    ABSOLUTE_ENCODER_MULTI_TURN_FAILURE = 0x82         # 对心多圈绝对值编编码器多圈接口故障
    ABSOLUTE_ENCODER_EXTERNAL_INPUT_FAILURE = 0x83     # 对心多圈绝对值编码器外部输入故障
    ABSOLUTE_ENCODER_SYSTEM_ANOMALY = 0x84             # 对心多圈绝对值编码器读值故障
    ERR_OFFS = 0x85                                     # 对心多圈绝对值编码器ERR_OFFS
    ERR_CFG = 0x86                                      # 对心多圈绝对值编码器ERR_CFG
    ILLEGAL_FIRMWARE_DETECTED = 0x88                    # 检测到非法固件
    INTEGRATED_STATOR_DRIVER_DAMAGED = 0x89             # 集成式栅极驱动器初始化失败

def RuiwoErrCode2string(errcode):
    """
    @brief 将RuiwoErrCode枚举值转换为可读的字符串描述

    @param[in] errcode RuiwoErrCode枚举值，表示具体的电机故障码
    @return str 故障码的中文描述
    """
    try:
        # 如果传入的是枚举类型，直接获取其value
        if isinstance(errcode, RuiwoErrCode):
            errcode = errcode.value

        # 确保是整数类型
        if not isinstance(errcode, int):
            errcode = int(errcode)

        # 故障码映射字典 - 使用整数键，避免魔数，通过枚举值定义
        error_map = {
            RuiwoErrCode.NO_FAULT.value: "无故障",
            RuiwoErrCode.DC_BUS_OVER_VOLTAGE.value: "直流母线电压过压",
            RuiwoErrCode.DC_BUS_UNDER_VOLTAGE.value: "直流母线电压欠压",
            RuiwoErrCode.ENCODER_ANGLE_FAULT.value: "编码器电角度故障",
            RuiwoErrCode.DRV_DRIVER_FAULT.value: "DRV 驱动器故障",
            RuiwoErrCode.DC_BUS_CURRENT_OVERLOAD.value: "直流母线电流过流",
            RuiwoErrCode.MOTOR_A_PHASE_CURRENT_OVERLOAD.value: "电机 A 相电流过载",
            RuiwoErrCode.MOTOR_B_PHASE_CURRENT_OVERLOAD.value: "电机 B 相电流过载",
            RuiwoErrCode.MOTOR_C_PHASE_CURRENT_OVERLOAD.value: "电机 C 相电流过载",
            RuiwoErrCode.DRIVER_BOARD_OVERHEAT.value: "驱动板温度过高",
            RuiwoErrCode.MOTOR_WINDING_OVERHEAT.value: "电机线圈过温",
            RuiwoErrCode.ENCODER_FAILURE.value: "编码器故障",
            RuiwoErrCode.CURRENT_SENSOR_FAILURE.value: "电流传感器故障",
            RuiwoErrCode.OUTPUT_ANGLE_OUT_OF_RANGE.value: "输出轴实际角度超过通信范围：CAN COM Theta MIN ~ CAN COM Theta MAX",
            RuiwoErrCode.OUTPUT_SPEED_OUT_OF_RANGE.value: "输出轴速度超过通信范围 CAN COM Velocity MIN ~ CAN COM Velocity MAX",
            # 开机自检故障码（大于128，无法用该方法清除）
            RuiwoErrCode.ABS_ENCODER_OFFSET_VERIFICATION_FAILURE.value: "离轴/对心多圈绝对值编码器接口帧头校验失败，重启，若还失败则请联系售后工程师",
            RuiwoErrCode.ABSOLUTE_ENCODER_MULTI_TURN_FAILURE.value: "对心多圈绝对值编编码器多圈接口故障，重启，若还失败则请联系售后工程师",
            RuiwoErrCode.ABSOLUTE_ENCODER_EXTERNAL_INPUT_FAILURE.value: "对心多圈绝对值编码器外部输入故障，重启，若还失败则请联系售后工程师",
            RuiwoErrCode.ABSOLUTE_ENCODER_SYSTEM_ANOMALY.value: "对心多圈绝对值编码器读值故障，重启，若还失败则请联系售后工程师",
            RuiwoErrCode.ERR_OFFS.value: "对心多圈绝对值编码器ERR_OFFS，重启，若还失败则请联系售后工程师",
            RuiwoErrCode.ERR_CFG.value: "对心多圈绝对值编码器ERR_CFG，重启，若还失败则请联系售后工程师",
            RuiwoErrCode.ILLEGAL_FIRMWARE_DETECTED.value: "检测到非法固件，重启，若还失败则请联系售后工程师",
            RuiwoErrCode.INTEGRATED_STATOR_DRIVER_DAMAGED.value: "集成式栅极驱动器初始化失败，重启，若还失败则请联系售后工程师"
        }

        return error_map.get(errcode, f"未知故障码: 0x{errcode:02X}")

    except (ValueError, TypeError) as e:
        return f"故障码格式错误: {errcode}"

def is_position_invalid(position):
    """
    根据RUIWO电机的机械限制，检查关节位置是否超出合理范围
    通常关节位置应在±12.5rad范围内

    Args:
        position (float): 关节位置值

    Returns:
        bool: True表示位置无效，False表示位置有效
    """
    POSITION_MAX_LIMIT = 12.5  # 硬件最大位置限制
    EPSILON = 1e-10          # 浮点数比较精度

    # 检查是否为NaN或无穷大
    if np.isnan(position) or np.isinf(position):
        return True

    # 如果位置绝对值大于等于最大限制（考虑浮点精度），则非法
    if abs(position) - POSITION_MAX_LIMIT > EPSILON:
        return True

    return False

def has_position_jump(previous_pos, current_pos):
    """
    根据RUIWO电机特性，检查位置是否存在异常跳变
    100RPM = 10.47 rad/sec，考虑CAN通讯可能丢几帧，设置安全的跳变阈值

    Args:
        previous_pos (float): 前一个位置
        current_pos (float): 当前位置

    Returns:
        bool: True表示存在异常跳变，False表示正常
    """
    POSITION_JUMP_THRESHOLD = 0.3  # 位置跳变阈值（弧度）
    EPSILON = 1e-10                # 浮点数比较精度

    # 检查输入值的有效性
    if (np.isnan(current_pos) or np.isinf(current_pos) or
        np.isnan(previous_pos) or np.isinf(previous_pos)):
        return True  # 无效输入认为是异常

    # 计算位置变化的绝对值
    position_change = abs(current_pos - previous_pos)

    # 如果位置变化超过阈值，则认为存在异常跳变
    return position_change > POSITION_JUMP_THRESHOLD + EPSILON

def execute_setzero_script():
    script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"setZero.sh")
    print("Executing setzero script:", script_path)
    if not os.path.exists(script_path):
        raise FileNotFoundError(f"The script {script_path} does not exist.")
    command = f"bash {script_path}"
    return os.system(command)

def getKey():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key

class RuiWoActuator():
    def __init__(self, disable_joint_ids=[], setZero=False):
        self.disable_joint_ids = disable_joint_ids

        config_path = self.get_config_path()
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        self.get_prarameter_name()
        self.running = True
        self.RUIWOTools = RUIWOTools()

        # 在 0-torque 模式下, 手动拖摆动机器人手臂
        self.teach_pendant_mode = 0
       
        print("---------------INTIALIZED START---------------")
        open_canbus = self.RUIWOTools.open_canbus()
        if not open_canbus:
            print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")
            exit(1)
        print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")
        self.get_config(config)

        # cali_arm 模式下进行多圈清零
        if setZero:
            all_joint_addresses = self.Left_joint_address + self.Right_joint_address
            active_dev_ids = [addr for addr in all_joint_addresses if addr not in disable_joint_ids]
            self.RUIWOTools.multi_turn_zeroing(active_dev_ids)
            print("[RUIWO motor]: Calibration mode detected, performing multi-turn zeroing...")
        else:
            print("[RUIWO motor]: Normal mode, skipping multi-turn zeroing")

        self.sendposlock = threading.Lock()
        self.recvposlock = threading.Lock()
        self.sendvellock = threading.Lock()
        self.recvvellock = threading.Lock()
        self.sendtorlock = threading.Lock()
        self.recvtorlock = threading.Lock()
        self.statelock = threading.Lock()
        self.updatelock = threading.Lock()
        self.target_update = False
        self.zero_position = [0.0] * 6 + [0.0] * 6 + [0.0] * 2
         # 零点位置
        zeros_path = self.get_zero_path()
        if os.path.exists(zeros_path) and not setZero:
            with open(zeros_path, 'r') as file:
                zeros_config = yaml.safe_load(file)
            self.zero_position = zeros_config['arms_zero_position']
            ret = self.enable()
            if ret != 0:
                print(f"\033[31m[RUIWO motor]: 使能失败，错误码: {ret}，程序退出\033[0m")
                exit(1)  # 使能失败
        else:
            ret = self.enable()
            if ret != 0:
                print(f"\033[31m[RUIWO motor]: 使能失败，错误码: {ret}，程序退出\033[0m")
                exit(1)  # 使能失败
            if not setZero:
                print("[RUIWO motor]:Warning: zero_position file does not exist, will use current position as zero value.")
            self.set_as_zero()# 保存当前为零点位置


        print("[RUIWO motor]:Control mode:",self.control_mode)
        print("[RUIWO motor]:Negtive joint ID:",self.negtive_joint_address_list)
        print("[RUIWO motor]:Multi-turn_Encoder_mode:",self.multi_turn_encoder_mode)
        print("[RUIWO motor]:Multi-turn_Encoder_modeZero:",self.zero_position)    
        
        # time.sleep(0.1)
        print("[RUIWO motor]原始手臂关节位置:")
        zero_pos = [state[1] for state in self.origin_joint_status if type(state) is list]
        print(zero_pos)

        # 检查是否存在无效位置
        has_invalid_pos = False
        for position in zero_pos:
            if is_position_invalid(position):
                has_invalid_pos = True
                break

        if has_invalid_pos:
            print("\033[33m[RUIWO motor]: Error: 检测到接近或超过12.5rad的关节位置，可能存在位置异常\033[0m")
            exit(3)  # 位置异常退出码

        print("[RUIWO motor]加上零点后的手臂关节位置:")
        zero_pos = [state[1] for state in self.joint_status if type(state) is list]
        print(zero_pos)
        # while True:
        #     time.sleep(0.1)
        self.go_to_zero()
        
        zero_state = self.get_joint_state()
        print("[RUIWO motor]:Moved to zero succeed")
        print("\n\n")

        print("[RUIWO motor]回零之后，手臂关节位置:")
        zero_pos = [state[1] for state in self.joint_status if type(state) is list]
        print(zero_pos)
        maybe_nagative_joint_ids = [self.joint_address_list[i] for i,pos in enumerate(zero_pos) if abs(pos)>0.1]
        if len(maybe_nagative_joint_ids) > 0:
            print("\033[31m[RUIWO motor]:Warning: 下列关节方向可能反了:\n",maybe_nagative_joint_ids)
            print("请检查并修改config.yaml\033[0m")
        print("\n\n")
        # print("[RUIWO motor]:Joint zero state:")
        # for i in range(len(self.joint_address_list)):
        #     print(self.joint_status[i])
        print("---------------INTIALIZED DONE---------------")
        self.control_thread = threading.Thread(target=self.control_thread)
        self.control_thread.start()
        
    def get_config_path(self):
        path = self.get_home_path()
        if not path:
            print("Failed to get home path.")
            exit(1)
        config_file = 'config.yaml'
        config_path = os.path.join(path,config_file)
        return config_path
    
    def get_zero_path(self):
        path = self.get_home_path()
        if not path:
            print("Failed to get home path.")
            exit(1)
        config_file = 'arms_zero.yaml'
        config_path = os.path.join(path,config_file)
        return config_path
    
    def _get_joint_addresses(self, config, joint_type, count):
        return [config['address'][f'{joint_type}_{i+1}'] for i in range(count)]

    def _get_joint_online_status(self, config, joint_type, count):
        return [config['online'][f'{joint_type}_{i+1}'] for i in range(count)]

    def _get_joint_parameters(self, config, joint_type, count):
        return [config['parameter'][f'{joint_type}_{i+1}'] for i in range(count)]

    def get_prarameter_name(self):
        self.Left_joint_address = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]
        self.Right_joint_address = [0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C]
        self.Head_joint_address = [0x0D, 0x0E]
        self.Left_joint_online = [False, False, False, False, False, False]
        self.Right_joint_online = [False, False, False, False, False, False]
        self.Head_joint_online = [False, False]
        self.Left_joint_parameter = [[0, 25, 8, 0, 0, 0, 0],
                                    [0, 20, 6, 0, 0, 0, 0],
                                    [0, 20, 6, 0, 0, 0, 0],
                                    [0, 10, 3, 0, 0, 0, 0],
                                    [0, 10, 3, 0, 0, 0, 0],
                                    [0, 10, 3, 0, 0, 0, 0]]
        self.Right_joint_parameter = [[0, 25, 8, 0, 0, 0, 0],
                                    [0, 20, 6, 0, 0, 0, 0],
                                    [0, 20, 6, 0, 0, 0, 0],
                                    [0, 10, 3, 0, 0, 0, 0],
                                    [0, 10, 3, 0, 0, 0, 0],
                                    [0, 10, 3, 0, 0, 0, 0]]
        self.Head_joint_parameter = [[0, 4, 3, 0, 0, 0, 0],
                                    [0, 10, 6, 0, 0, 0, 0]]
        self.Joint_parameter_list = self.Left_joint_parameter + self.Left_joint_parameter + self.Head_joint_address
        self.joint_address_list = self.Left_joint_address + self.Right_joint_address + self.Head_joint_address
        self.joint_online_list = self.Left_joint_online + self.Right_joint_online + self.Head_joint_online
        self.negtive_joint_address_list = []
        #self.unnecessary_go_zero_list = [0x04, 0x05, 0x06, 0x0A, 0x0B, 0x0C]
        self.unnecessary_go_zero_list = [0x0, 0x0, 0x0, 0x0, 0x0, 0x0]
        self.control_mode = "ptm"
        self.multi_turn_encoder_mode = False
        self.Head_joint_zero_position = [0.0] * 2
        self.ratio = [36,36,36,10,10,10,36,36,36,10,10,10,36,36]
        
    def get_config(self,config):
        # 关节电机地址
        self.Left_joint_address = self._get_joint_addresses(config, 'Left_joint_arm', 6)
        self.Right_joint_address = self._get_joint_addresses(config, 'Right_joint_arm', 6)
        self.Head_joint_address = [config['address']['Head_joint_low'], config['address']['Head_joint_high']]
        # 关节电机状态标志位
        self.Left_joint_online = self._get_joint_online_status(config, 'Left_joint_arm', 6)
        self.Right_joint_online = self._get_joint_online_status(config, 'Right_joint_arm', 6)
        self.Head_joint_online = [config['online']['Head_joint_low'], config['online']['Head_joint_high']]
        # 关节参数[vel, kp_pos, kd_pos, tor, kp_vel, kd_vel, ki_vel]
        self.Left_joint_parameter = self._get_joint_parameters(config, 'Left_joint_arm', 6)
        self.Right_joint_parameter = self._get_joint_parameters(config, 'Right_joint_arm', 6)
        self.Head_joint_parameter = [config['parameter']['Head_joint_low'], config['parameter']['Head_joint_high']]
        # 关节参数列表
        self.Joint_parameter_list = self.Left_joint_parameter + self.Right_joint_parameter + self.Head_joint_parameter
        # 汇总地址和在线状态列表
        self.joint_address_list = self.Left_joint_address + self.Right_joint_address + self.Head_joint_address
        self.joint_online_list = self.Left_joint_online + self.Right_joint_online + self.Head_joint_online
        # 其他配置项
        self.negtive_joint_address_list = config['negtive_address'][0]
        self.unnecessary_go_zero_list = config['low_arm_address'][0]
        # 控制模式
        self.control_mode = config['control_mode']
        # 多圈编码器模式
        try:
            self.multi_turn_encoder_mode = config['Multi-turn_Encoder_mode']
        except KeyError:
            self.multi_turn_encoder_mode = False
        try:
            config_ratio = config['ratio']
            for i in range(len(config_ratio)):
                self.ratio[i] = config_ratio[i]
        except KeyError:    
            self.ratio = [36,36,36,10,10,10,36,36,36,10,10,10,36,36]
        # if self.multi_turn_encoder_mode:
        
        for id in self.disable_joint_ids:
            self.joint_address_list[id - 1] = DISABLE_ADDRESS
     
    def get_home_path(self):
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
    
    def control_thread(self):
        print("[RUIWO motor]:Threadstart succeed")
        target_positions = self.target_positions
        target_torque = self.target_torque
        target_velocity = self.target_velocity
        try:
            
            while self.running:
                self.update_status_asynchronous()
                self.update_status()
                index = range(len(self.joint_address_list))
                time.sleep(dt)

                if 1 == self.teach_pendant_mode:
                    self.send_positions_No_response(index, list(target_positions), list(target_torque), list(target_velocity))
                    continue
                if self.target_update == False:
                    continue
                with self.sendposlock:
                    target_positions = self.target_positions
                    target_torque = self.target_torque
                    target_velocity = self.target_velocity
                    self.target_update = False
                self.send_positions_No_response(index, list(target_positions), list(target_torque), list(target_velocity))
                
                
        except Exception as e:
            print(e)
        print("[RUIWO motor]:Threadend succeed")
            
    def join(self):
        self.control_thread.join()

    def interpolate_positions_with_speed(self,a, b, speed, dt):
        """
        根据速度插值函数，从位置a插值到位置b。

        Parameters:
            a (list or numpy.ndarray): 起始位置。
            b (list or numpy.ndarray): 目标位置。
            speed (float): 插值速度，每秒移动的距离。
            dt (float): 时间步长，默认为0.02秒。

        Returns:
            list of numpy.ndarray: 插值结果，每个元素为从a到b每个维度的插值序列。
        """
        a = np.array(a)
        b = np.array(b)
        # 计算总时间
        total_time = np.linalg.norm(b - a) / speed
        # 根据总时间和时间步长计算实际的时间步数
        steps = int(total_time / dt) + 1
        # 使用NumPy的linspace进行插值
        interpolation = np.linspace(a, b, steps)
        max_length = max(steps, interpolation.shape[0])
        # 将插值结果按维度拆分成列表
        interpolation_list = [[interpolation[i, j] for j in range(interpolation.shape[1])] for i in range(interpolation.shape[0])]
        return interpolation_list
            
    def go_to_zero(self):
        print("[RUIWO motor]:Start moving to zero")
        state = self.get_joint_state()
        current_positions = [0]*len(self.joint_address_list)
        target_positions = [0]*len(self.joint_address_list)
        for i, address in enumerate(self.joint_address_list):
            if self.joint_online_list[i] == True:
                motor = state[i]
                current_positions[i] = motor[1]
                if address in self.unnecessary_go_zero_list:
                    pass

        self.interpolate_move(current_positions, target_positions,max_speed,dt)
    
    def go_to_target(self, target_positions):
        print("[RUIWO motor]:Start moving to target....")
        state = self.get_joint_state()
        current_positions = [0]*len(self.joint_address_list)
        for i, address in enumerate(self.joint_address_list):
            if self.joint_online_list[i] == True:
                motor = state[i]
                current_positions[i] = motor[1]

        self.interpolate_move(current_positions, target_positions,max_speed,dt)
        
    def interpolate_move(self,start_positions, target_positions, speed, dt):
        interpolation_list = self.interpolate_positions_with_speed(start_positions, target_positions, speed, dt)
        print ("target_positions:",target_positions)
        print ("start_positions:",start_positions)
        # last = interpolation_list[-1]
        # print([last[i]+self.zero_position[i] for i in range(len(last))])
        for target_position in interpolation_list:
            self.send_positions(range(len(self.joint_address_list)), target_position,self.target_torque,self.target_velocity)
            self.old_target_positions = target_position
            state = self.get_joint_state()
            for i, address in enumerate(self.joint_address_list):
                if self.joint_online_list[i]:
                    motor = state[i]
                    self.current_positions[i] = motor[1]
                    self.current_velocity[i] = motor[2]
                    self.current_torque[i] = motor[3]
            time.sleep(dt)
        
    def enable(self):
        """
        @brief 使能所有电机，对全部电机执行 Enter Motor State 进入 Motor State 运行模式

        @return int 成功返回 0，失败返回其他错误码
                 1: 通信问题，包括：CAN 消息发送/接收失败或者超时
                 2: 严重错误!!! 严重错误!!! 返回 2 时说明有电机存在严重故障码(RuiwoErrorCode 中大于 128 的故障码)
        """
        # reset motors
        ret = self.disable()
        if ret != 0:
            print(f"\033[31m[RUIWO motor]: 电机使能前置步骤（清除电机故障码）失败，无法继续使能，返回码:\033[0m{ret}")
            return ret

        # ////////////// 初始化变量 //////////////
        self.target_positions = [0]*len(self.joint_address_list)
        self.target_velocity = [0]*len(self.joint_address_list)
        self.target_torque = [0]*len(self.joint_address_list)
        self.target_pos_kp = [param[1] for param in self.Joint_parameter_list]
        self.target_pos_kd = [param[2] for param in self.Joint_parameter_list]
        self.target_vel_kp = [param[4] for param in self.Joint_parameter_list]
        self.target_vel_kd = [param[5] for param in self.Joint_parameter_list]
        self.target_vel_ki = [param[6] for param in self.Joint_parameter_list]
        self.old_target_positions = [0]*len(self.joint_address_list)
        self.current_positions = [0]*len(self.joint_address_list)
        self.current_torque = [0]*len(self.joint_address_list)
        self.current_velocity = [0]*len(self.joint_address_list)
        self.joint_status = [0]*len(self.joint_address_list)
        self.origin_joint_status = [0]*len(self.joint_address_list)
        self.head_low_torque = 0
        self.head_high_torque = 0
        # ////////////// 初始化变量 end //////////////

        fail_count = 0
        for index, address in enumerate(self.joint_address_list):
            if address == DISABLE_ADDRESS:
                self.joint_online_list[index] = False
                print("[RUIWO motor]:ID:", address, "Enable: [Ignored]")
                continue

            # 先 Enter Reset 擦除电机故障码
            reset_state = self.RUIWOTools.enter_reset_state(address)
            if isinstance(reset_state, list):
                errcode = reset_state[5] if len(reset_state) > 5 else 0
                if errcode >= RuiwoErrCode.ABS_ENCODER_OFFSET_VERIFICATION_FAILURE.value:
                    motor_err = RuiwoErrCode2string(errcode)
                    print(f"\033[31m[RUIWO motor]:ID:{address} Enable: [Failed], Errorcode: [0x{errcode:X}] {motor_err}\033[0m")
                    return 2  # 直接返回 2 表示存在电机有严重故障码

            # 使能电机
            time.sleep(0.02)
            state = self.RUIWOTools.enter_motor_state(address)
            if isinstance(state, list):
                errcode = state[5] if len(state) > 5 else 0
                if errcode >= RuiwoErrCode.ABS_ENCODER_OFFSET_VERIFICATION_FAILURE.value:
                    motor_err = RuiwoErrCode2string(errcode)
                    print(f"\033[31m[RUIWO motor]:ID:{address} Enable: [Failed], Errorcode: [0x{errcode:X}] {motor_err}\033[0m")
                    return 2  # 直接返回 2 表示存在电机有严重故障码

                self.RUIWOTools.run_ptm_mode(address, state[1], 0, self.target_pos_kp[index], self.target_pos_kd[index], 0)
                self.set_joint_state(index, state)
                self.joint_online_list[index] = True
                print("[RUIWO motor]:ID:", address, "Enable: [Succeed]")
            else:
                fail_count += 1
                print("[RUIWO motor]:ID:", address, "Enable: [Failed], Errorcode:", state)

        if fail_count > 0:
            return 1  # 全部失败 ==> 通讯问题

        return 0  # 成功返回0

    def disable(self):
        """
        @brief 对全部电机执行 Enter Reset State 进入 Reset State 运行模式

        @note: enter reset state 可用于清除故障码（大于 128 的故障码无法清除）
        @return int 成功返回 0，失败返回其他错误码
                 1: 通信问题，包括：CAN 消息发送/接收失败或者超时
                 2: 严重错误!!! 严重错误!!! 返回 2 时说明有电机存在严重故障码(RuiwoErrorCode 中大于 128 的故障码)
        """
        fail_count = 0
        motor_error_count = 0  # 错误计数

        for index, address in enumerate(self.joint_address_list):
            if address == DISABLE_ADDRESS:
                print("[RUIWO motor]:ID:", self.joint_address_list[index], "Disable: [Ignored]")
                continue

            state = self.RUIWOTools.enter_reset_state(self.joint_address_list[index])
            if isinstance(state, list):
                errcode = state[5] if len(state) > 5 else 0
                # 出现大于 128 无法清除的故障码
                if errcode >= RuiwoErrCode.ABS_ENCODER_OFFSET_VERIFICATION_FAILURE.value:
                    print(f"\033[31m[RUIWO motor]:ID:{address} Disable: [Failed], Errorcode: [0x{errcode:X}],错误: {RuiwoErrCode2string(errcode)}\033[0m")
                    motor_error_count += 1
                    continue

                self.joint_online_list[index] = False
                print("[RUIWO motor]:ID:", address, "Disable: [Succeed]")
                # for i in range(5):
                #     self.RUIWOTools.run_ptm_mode(self.joint_address_list[index],0,0,0,0,0)
                #     time.sleep(0.01)
            else:
                fail_count += 1
                print("[RUIWO motor]:ID:", address, "Disable: [Failed], Retcode:", state)

        # 有电机存在大于 128 的故障码(无法清除), 严重
        if motor_error_count > 0:
            return 2  # 优先返回更严重的错误
        elif fail_count == len(self.joint_address_list):
            return 1  # 全部失败 ==> 通讯问题

        return 0  # 成功返回0
    
    def measure_head_torque(self,pos):
        torque_coefficient = 1
        sin_coefficient = -0.45
        torque = (torque_coefficient / math.sin(sin_coefficient)) * math.sin(pos)
        return torque

    def send_positions(self, index, pos, torque, velocity):
        target_torque = torque
        for i in (index):
            if self.joint_online_list[i] is False:
                continue

            address = self.joint_address_list[i]
            if (address) in self.negtive_joint_address_list:
                pos[i] = -(pos[i] + self.zero_position[i]) #减去零点偏移
                torque[i] = max(-10.0, -torque[i])
                velocity[i] = velocity_factor * -velocity[i]
            else:
                pos[i] = pos[i] + self.zero_position[i] #减去零点偏移
                torque[i] = min(10.0, torque[i])
                velocity[i] = velocity_factor * velocity[i]
            if self.joint_address_list[i] == self.Head_joint_address[1]:
                target_torque[i] = self.head_high_torque
            # print("current dev id: ", self.joint_address_list[i])
            # print("target pos: ", pos[i])
            if self.control_mode == "ptm":
                if self.teach_pendant_mode == 1:
                    state = self.RUIWOTools.run_ptm_mode(self.joint_address_list[i], 0, 0, 0, 0, 0)
                else:
                    state = self.RUIWOTools.run_ptm_mode(self.joint_address_list[i], pos[i], velocity[i], self.target_pos_kp[i], self.target_pos_kd[i], target_torque[i])
            
            elif self.control_mode == "servo":
                state = self.RUIWOTools.run_servo_mode(self.joint_address_list[i], pos[i], velocity[i], self.target_pos_kp[i], self.target_pos_kd[i], self.target_vel_kp[i], self.target_vel_kd[i], self.target_vel_ki[i])
            if isinstance(state, list):
                self.set_joint_state(i,state)
                if self.joint_address_list[i] == self.Head_joint_address[1]:
                    self.head_high_torque = self.measure_head_torque(state[1])
        # print(pos)

    def set_teach_pendant_mode(self, mode):

        if self.teach_pendant_mode == 1:
            print("[RUIWO motor]进入teach_pendant模式")
        self.teach_pendant_mode = mode

    def send_positions_No_response(self, index, pos, torque, velocity):
        target_torque = torque
        for i in (index):
            if self.joint_online_list[i] is False:
                continue

            address = self.joint_address_list[i]
            if (address) in self.negtive_joint_address_list:
                pos[i] = -(pos[i] + self.zero_position[i]) #减去零点偏移
                torque[i] = max(-10.0, -torque[i])
                velocity[i] = velocity_factor * -velocity[i]
            else:
                pos[i] = pos[i] + self.zero_position[i] #减去零点偏移
                torque[i] = min(10.0, torque[i])
                velocity[i] = velocity_factor * velocity[i]
            if self.joint_address_list[i] == self.Head_joint_address[1]:
                target_torque[i] = self.head_high_torque
            if self.control_mode == "ptm":
                if self.teach_pendant_mode == 1:
                    self.RUIWOTools.run_ptm_mode_No_response(self.joint_address_list[i], 0, 0, 0, 0, 0)
                else:
                    self.RUIWOTools.run_ptm_mode_No_response(self.joint_address_list[i], pos[i], velocity[i], self.target_pos_kp[i], self.target_pos_kd[i], target_torque[i])
            
            elif self.control_mode == "servo":
                self.RUIWOTools.run_servo_mode(self.joint_address_list[i], pos[i], velocity[i], self.target_pos_kp[i], self.target_pos_kd[i], self.target_vel_kp[i], self.target_vel_kd[i], self.target_vel_ki[i])
            self.update_status_asynchronous()
            self.update_status()
            # time.sleep(dt)
            # if isinstance(state, list):
            #     self.set_joint_state(i,state)
            #     if self.joint_address_list[i] == self.Head_joint_address[1]:
            #         self.head_high_torque = self.measure_head_torque(state[1])
        
    def update_status(self):
        current_possitions = [0]*len(self.joint_address_list)
        current_torque = [0]*len(self.joint_address_list)
        current_velocity = [0]*len(self.joint_address_list)
        with self.statelock:
            joint_state = self.joint_status
        index = range(len(self.joint_address_list))
        for i in index:
            if self.joint_online_list[i] == True:
                motor = joint_state[i]
                current_possitions[i] = motor[1]
                current_velocity[i] = motor[2]
                current_torque[i] = motor[3]
        with self.recvposlock:
            self.current_positions = current_possitions
        with self.recvtorlock:
            self.current_torque = current_torque
        with self.recvvellock:
            self.current_velocity = current_velocity

    def update_status_asynchronous(self):
        try:
            rx_msg = self.RUIWOTools.dev.recv(self.RUIWOTools.dev_info["notimeout"])
            if rx_msg is not None:
                    # print("rec:",rx_msg)
                    # 如果接收的 arbitration_id 和 data[0] 都在 sender_ids 中
                if rx_msg.arbitration_id in self.joint_address_list:
                    if len(rx_msg.data) > 0:
                        rx_id = rx_msg.data[0]
                            # 双重判断: arbitration_id == rx_id
                            # 确保是我们发送出去并返回的那个 ID
                        if rx_id == rx_msg.arbitration_id and rx_id in self.joint_address_list:
                            motor_state = self.RUIWOTools.return_motor_state(rx_msg.data)

                            # 处理errcode
                            errcode = motor_state[5] if len(motor_state) > 5 else 0
                            if errcode != 0:
                                print(f"\033[31m[RUIWO motor] RX_ID:{rx_id} Error code: 0x{errcode:X} {RuiwoErrCode2string(errcode)}\033[0m")
                            else:
                                # 检查位置跳变
                                # 判断电机是否为反向
                                is_negative = rx_id in self.negtive_joint_address_list
                                current_raw_pos = -motor_state[1] if is_negative else motor_state[1]  # origin state 是带方向的

                                # 获取之前的位置
                                previous_state = self.origin_joint_status[rx_id-1]
                                if isinstance(previous_state, list) and len(previous_state) > 1:
                                    previous_raw_pos = previous_state[1]  # previous state

                                if has_position_jump(previous_raw_pos, current_raw_pos):
                                    print(f"\033[33m[RUIWO motor] RX_ID:{rx_id} 检测到位置异常跳变: {previous_raw_pos} -> {current_raw_pos}\033[0m")
                                else:
                                    self.set_joint_state(rx_id-1, motor_state)
                                    self.update_status()
                        else:
                            print(f"[RUIWO motor] rx_id mismatch {rx_id} : rx_msg.id.SID : {rx_msg.arbitration_id}")

        except Exception as e:
            print(f"接收线程异常: {e}")
            
    def set_positions(self,index, positions, torque, velocity):
        if not self.running:
            return None 
        with self.sendposlock:
            new_positions = self.target_positions
            new_torque = self.target_torque
            new_velocity = self.target_velocity
        num_input_joints = len(index)    
        for i, address in enumerate(self.joint_address_list):
            if address == DISABLE_ADDRESS:
                continue
            for j, id in enumerate(index):
                if address - 1 == id:
                    new_positions[i] = positions[j]
                    new_torque[i] = torque[j]
                    new_velocity[i] = velocity[j]
            #         if (address) in self.negtive_joint_address_list:
            #             new_positions[i] = -positions[j]
            #             new_torque[i] = max(-10.0, -torque[j])
            #             new_velocity[i] = velocity_factor * -velocity[j]
            #         else:
            #             new_positions[i] = positions[j]
            #             new_torque[i] = min(10.0, torque[j])
            #             new_velocity[i] = velocity_factor * velocity[j]
            if i >= num_input_joints:
                new_positions[i] = self.target_positions[i]
                new_torque[i] = 0
                new_velocity[i] = 0

        with self.sendposlock:
            self.target_positions = new_positions
            self.target_torque = new_torque
            self.target_velocity = new_velocity
            self.target_update = True

    def set_torgue(self,index, torque):
        if not self.running:
            return None
        with self.sendtorlock:
            new_torque = self.target_torque
        for i, address in enumerate(self.joint_address_list):
            if address == DISABLE_ADDRESS:
                continue
            for j, id in enumerate(index):
                if address - 1 == id:
                    if (address) in self.negtive_joint_address_list:
                        new_torque[i] = -torque[j]
                    else:
                        new_torque[i] =  torque[j]
        with self.sendtorlock:
            self.target_torque = new_torque
        with self.updatelock:
            self.target_update = True

    def set_velocity(self,index, velocity):
        if not self.running:
            return None
        with self.sendvellock:
            new_velocity = self.target_velocity
        for i, address in enumerate(self.joint_address_list):
            if address == DISABLE_ADDRESS:
                continue
            for j, id in enumerate(index):
                if address - 1 == id:
                    if (address) in self.negtive_joint_address_list:
                        new_velocity[i] = -velocity[j]
                    else:
                        new_velocity[i] =  velocity[j]
        with self.sendvellock:
            self.target_velocity = new_velocity
        with self.updatelock:
            self.target_update = True

    def get_positions(self):
        if not self.running:
            return None 
        with self.recvposlock:
            position_list = self.current_positions
        return position_list

    def get_torque(self):
        if not self.running:
            return None 
        with self.recvtorlock:
            torque_list = self.current_torque
        return torque_list
    
    def get_velocity(self):
        if not self.running:
            return None 
        with self.recvvellock:
            velocity_list = self.current_velocity
        return velocity_list
    
    def set_joint_state(self,idex,state):
        if not self.running:
            return None 
        new_state = state
        
        if new_state[0] in self.negtive_joint_address_list:
            new_state[1] = -new_state[1]
            new_state[2] = -new_state[2]
            new_state[3] = -new_state[3]
        origin_states = list(new_state) # 构造list保存原始state,去除方向
        new_state[1] -= self.zero_position[idex] #减去零点偏移
        
        # 添加小的容差范围，避免浮点数误差
        if abs(new_state[1]) < 1e-10:
            new_state[1] = 0.0
        if abs(new_state[2]) < 1e-10:
            new_state[2] = 0.0
        if abs(new_state[3]) < 1e-10:
            new_state[3] = 0.0
        with self.statelock:
            self.joint_status[idex] = new_state
            self.origin_joint_status[idex] = origin_states
            
    def get_joint_origin_state(self):
        if not self.running:
            return None 
        with self.statelock:
            state_list = self.origin_joint_status
        return state_list
    
    def get_joint_state(self):
        if not self.running:
            return None 
        # 成功返回电机信息（电机ID, 电机位置，电机速度，电机力矩，故障码,驱动板温度），失败返回False
        with self.statelock:
            state_list = self.joint_status
        return state_list
    
    def close(self):
        self.disable()
        self.running = False
        close_canbus = self.RUIWOTools.close_canbus()
        if close_canbus:
            print("[RUIWO motor]:Canbus status:","[ Close ]")
        exit(0)
    
    def check_state(self):
       motors = [[],[]] # motors[0] => enable motor
                        # motors[1] => disable motor
       joint_addrs = self.Left_joint_address + self.Right_joint_address + self.Head_joint_address
                 
       try:
        for index, address in enumerate(joint_addrs):
            # 检测电机状态
            if self.joint_address_list.count(address) > 0:
                print("[RUIWO motor] Check State ID:", address, "State: [Enable]")
                motors[0].append(address)
            else:
                motors[1].append(address)
                print("[RUIWO motor] Check State ID:",address, "State: [Disable]")            
        
        print("[RUIWO motor] motors:", motors)              
        return  motors
       except Exception as e:
            print(f"Failed to check state due to an error: {e}")
            return motors
    
    # 把当前的位置设置为零点
    def set_as_zero(self):
        for i, address in enumerate(self.joint_address_list):
            state = self.RUIWOTools.enter_motor_state(address)
            if isinstance(state, list):
                self.RUIWOTools.run_ptm_mode(self.joint_address_list[i],state[1],0,self.target_pos_kp[i], self.target_pos_kd[i],0)
                self.zero_position[i] = self.origin_joint_status[i][1]  # Assuming state[1] is the position
                self.set_joint_state(i,state)
                print(f"[RUIWO motor]:ID: {address} Current position: {self.origin_joint_status[i][1]}")
            else:
                print(f"[RUIWO motor]:ID: {address} Failed to get position: {self.origin_joint_status[i]}")
        self.save_zero_position()
        self.update_status()

        
    # 逐圈增加或减少零点位置
    def change_encoder_zero_round(self, index, direction):
        round = 360.0/self.ratio[index] * 3.1415926 / 180.0
        round *= direction
        self.zero_position[index] += round
        state = self.RUIWOTools.enter_motor_state(self.joint_address_list[index])
        if isinstance(state, list):
            self.RUIWOTools.run_ptm_mode(self.joint_address_list[index],state[1],0,self.target_pos_kp[index], self.target_pos_kd[index],0)
            self.set_joint_state(index,state)
        self.update_status()
        print(f"[RUIWO motor]:ID: {self.joint_address_list[index]} Change encoder zero position: {self.zero_position[index]}")
    
    # 调整零点位置
    def adjust_zero_position(self, index, offset):
        self.zero_position[index] = offset
        state = self.RUIWOTools.enter_motor_state(self.joint_address_list[index])
        if isinstance(state, list):
            self.RUIWOTools.run_ptm_mode(self.joint_address_list[index],state[1],0,self.target_pos_kp[index], self.target_pos_kd[index],0)
            self.set_joint_state(index,state)
        self.update_status()
        print(f"[RUIWO motor]:ID: {self.joint_address_list[index]} Adjust encoder zero position: {self.zero_position[index]}")
    
    def get_motor_zero_points(self):
        return self.zero_position

    # 设置指定关节的kp_pos和kd_pos参数
    def set_joint_gains(self, joint_indices, kp_pos=None, kd_pos=None):
        """
        设置指定关节的kp_pos和kd_pos参数
        
        Args:
            joint_indices: 关节索引列表 (0-based)
            kp_pos: kp_pos值列表，如果为None则不修改
            kd_pos: kd_pos值列表，如果为None则不修改
        """
        if not isinstance(joint_indices, list):
            joint_indices = [joint_indices]
            
        if kp_pos is not None and not isinstance(kp_pos, list):
            kp_pos = [kp_pos]
            
        if kd_pos is not None and not isinstance(kd_pos, list):
            kd_pos = [kd_pos]
            
        for i, joint_idx in enumerate(joint_indices):
            if joint_idx < 0 or joint_idx >= len(self.target_pos_kp):
                print(f"[RUIWO motor]: Warning: joint index {joint_idx} out of range")
                continue
                
            if kp_pos is not None and i < len(kp_pos):
                self.target_pos_kp[joint_idx] = kp_pos[i]
                print(f"[RUIWO motor]: Set joint {joint_idx} kp_pos to {kp_pos[i]}")
                
            if kd_pos is not None and i < len(kd_pos):
                self.target_pos_kd[joint_idx] = kd_pos[i]
                print(f"[RUIWO motor]: Set joint {joint_idx} kd_pos to {kd_pos[i]}")

    # 获取指定关节的kp_pos和kd_pos参数
    def get_joint_gains(self, joint_indices=None):
        """
        获取指定关节的kp_pos和kd_pos参数
        
        Args:
            joint_indices: 关节索引列表，如果为None则返回所有关节
            
        Returns:
            dict: {'kp_pos': [values], 'kd_pos': [values]}
        """
        if joint_indices is None:
            joint_indices = list(range(len(self.target_pos_kp)))
        elif not isinstance(joint_indices, list):
            joint_indices = [joint_indices]
            
        kp_values = []
        kd_values = []
        
        for joint_idx in joint_indices:
            if joint_idx < 0 or joint_idx >= len(self.target_pos_kp):
                print(f"[RUIWO motor]: Warning: joint index {joint_idx} out of range")
                continue
            kp_values.append(self.target_pos_kp[joint_idx])
            kd_values.append(self.target_pos_kd[joint_idx])
            
        return {'kp_pos': kp_values, 'kd_pos': kd_values}

    # 保存当前的零点位置到文件中
    def save_zero_position(self):
        config_path = self.get_zero_path()
        config = {"arms_zero_position": self.zero_position}
        backup_path = config_path + '.bak'
        if os.path.exists(config_path):
            shutil.copy(config_path, backup_path) # 备份配置文件
        print(f"[RUIWO motor]: Backup config file to {backup_path}")
        with open(config_path, 'w') as file:
            yaml.dump(config, file, default_flow_style=False, allow_unicode=True)

if __name__ == '__main__':
    joint_control = RuiWoActuator()
    time.sleep(1)
    print("按 'f' 设置当前位置为零位，按 'w' 增加4零位, 按's' 减少4零位, \n'c' 保存零位，按 'e' 使能，按 'd' 禁用，按 't' 发送测试指令， 按 'q' 退出。")

    try:
        while True:
            # 检查是否有输入
            key = getKey()
            if key:
                print(key)
                if key == 'f':  # 如果按下 'f' 键
                    joint_control.set_as_zero()
                elif key == 'c':  # 如果按下 'c' 键
                    joint_control.save_zero_position()
                elif key == 'w': 
                    joint_control.change_encoder_zero_round(4, 1)
                elif key == 's': 
                    joint_control.change_encoder_zero_round(4, -1)
                elif key == 'e':
                    ret = joint_control.enable()
                    if ret != 0:
                        print(f"\033[31m[RUIWO motor]: 使能失败，错误码: {ret}，程序退出\033[0m")
                        exit(1)  # 使能失败
                elif key == 'd': 
                    joint_control.disable()
                elif key == 't':
                    test_ids = [2,3,4,5]
                    target_positions = [0.0, 0.0, 0.0, 0.0]
                    torque = [0.0, 0.0, 0.0, 0.0]
                    velocity = [0.0, 0.0, 0.0, 0.0]
                    joint_control.set_positions(test_ids, target_positions, torque, velocity)
                elif key == 'q':  # 如果按下 'q' 键
                    break

            # 这里可以继续进行其他操作
            time.sleep(0.01)  # 控制循环速度
    except KeyboardInterrupt:
        pass  # 允许通过 Ctrl+C 退出

    joint_control.close()
