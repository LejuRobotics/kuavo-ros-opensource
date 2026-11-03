import sys
import os
import time
import signal
import threading
import math
import yaml
import pwd
import numpy as np
from SimpleSDK import RUIWOTools
current_path =os.path.dirname(os.path.abspath(__file__))
sys.path.append('/usr/lib/python3/dist-packages')
# 控制周期
dt=0.001
# 插值规划的速度
max_speed = 2
class RuiWoActuator():
    def __init__(self):
        path = self.get_home_path()
        if not path:
            print("Failed to get home path.")
            exit(1)
        config_file = 'config.yaml'
        config_path = os.path.join(path,config_file)
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        self.get_prarameter_name()
        self.get_config(config)
        self.running = True
        self.RUIWOTools = RUIWOTools()
        print("[RUIWO motor]:Control mode:",self.control_mode)
        print("[RUIWO motor]:Negtive joint ID:",self.negtive_joint_address_list)
        print("---------------INTIALIZED START---------------")
        open_canbus = self.RUIWOTools.open_canbus()
        if not open_canbus:
            print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")
            exit(1)
        print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")
        self.sendposlock = threading.Lock()
        self.recvposlock = threading.Lock()
        self.sendvellock = threading.Lock()
        self.recvvellock = threading.Lock()
        self.sendtorlock = threading.Lock()
        self.recvtorlock = threading.Lock()
        self.statelock = threading.Lock()
        self.updatelock = threading.Lock()
        self.target_update = False
        self.enable()
        time.sleep(0.1)
        self.go_to_zero()
        zero_state = self.get_joint_state()
        print("[RUIWO motor]:Moved to zero succeed")
        print("[RUIWO motor]:Joint zero state:")
        for i in range(len(self.joint_address_list)):
            print(zero_state[i])
        print("---------------INTIALIZED DONE---------------")
        self.control_thread = threading.Thread(target=self.control_thread)
        self.control_thread.start()

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
        self.unnecessary_go_zero_list = [0x04, 0x05, 0x06, 0x0A, 0x0B, 0x0C]
        self.control_mode = "ptm"

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
            current_possitions = [0]*len(self.joint_address_list)
            current_torque = [0]*len(self.joint_address_list)
            current_velocity = [0]*len(self.joint_address_list)
            while self.running:
                time.sleep(dt)
                # 读取当前位置
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
                if self.target_update == False:
                    continue
                with self.sendposlock:
                    target_positions = self.target_positions
                    target_torque = self.target_torque
                    target_velocity = self.target_velocity
                    self.target_update = False
                self.send_positions(index, target_positions, target_torque, target_velocity)
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
                    current_positions[i] = 0
        self.interpolate_move(current_positions, target_positions,max_speed,dt)
        
    def interpolate_move(self,start_positions, target_positions, speed, dt):
        interpolation_list = self.interpolate_positions_with_speed(start_positions, target_positions, speed, dt)
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
        self.disable()
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
        self.head_low_torque = 0
        self.head_high_torque = 0
        for address in range(len(self.joint_address_list)):
            self.RUIWOTools.enter_reset_state(self.joint_address_list[address])
            # for i in range(5):
            #     self.RUIWOTools.run_ptm_mode(self.joint_address_list[address],0,0,0,0,0)
            #     time.sleep(0.01)
            time.sleep(0.02)
            state = self.RUIWOTools.enter_motor_state(self.joint_address_list[address])
            # for i in range(5):
            #     self.RUIWOTools.run_ptm_mode(self.joint_address_list[address],0,0,0,0,0)
            #     time.sleep(0.01)
            if isinstance(state, list):
                self.set_joint_state(address,state)
                self.joint_online_list[address] = True
                print("[RUIWO motor]:ID:",self.joint_address_list[address], "Enable:  [Succeed]")
            else:
                print("[RUIWO motor]:ID:",self.joint_address_list[address], "Enable: ","[",state,"]")
        
    def disable(self):
        joint_online_list_reload = []
        joint_address_list_reload = []
        Joint_parameter_list_reload = []
        for address in range(len(self.joint_address_list)):
            state = self.RUIWOTools.enter_reset_state(self.joint_address_list[address])
            if isinstance(state, list):
                self.joint_online_list[address] = False
                joint_online_list_reload.append(self.joint_online_list[address])
                joint_address_list_reload.append(self.joint_address_list[address])
                Joint_parameter_list_reload.append(self.Joint_parameter_list[address])
                print("[RUIWO motor]:ID:",self.joint_address_list[address], "Disable: [Succeed]")
                for i in range(5):
                    self.RUIWOTools.run_ptm_mode(self.joint_address_list[address],0,0,0,0,0)
                    time.sleep(0.01)
            else:
                print("[RUIWO motor]:ID:",self.joint_address_list[address], "Disable:","[",state,"]")
        del self.joint_online_list[len(joint_online_list_reload):]
        del self.joint_address_list[len(joint_address_list_reload):]
        del self.Joint_parameter_list[len(Joint_parameter_list_reload):]
        self.joint_online_list[:] = joint_online_list_reload
        self.joint_address_list[:] = joint_address_list_reload
        self.Joint_parameter_list[:] = Joint_parameter_list_reload

    def set_zero_position(self):
        for address in range(len(self.joint_address_list)):
            state = self.RUIWOTools.set_zero_positon(self.joint_address_list[address])
            if isinstance(state, list):
                self.set_joint_state(address,state)
            print("[RUIWO motor]:Set all motor zero position",state)
    
    def measure_head_torque(self,pos):
        torque_coefficient = 1
        sin_coefficient = -0.45
        torque = (torque_coefficient / math.sin(sin_coefficient)) * math.sin(pos)
        return torque

    def send_positions(self, index, pos, torque, velocity):
        target_torque = torque
        for i in (index):
            if self.joint_address_list[i] == self.Head_joint_address[1]:
                target_torque[i] = self.head_high_torque
            if self.control_mode == "ptm":
                state = self.RUIWOTools.run_ptm_mode(self.joint_address_list[i], pos[i], velocity[i], self.target_pos_kp[i], self.target_pos_kd[i], target_torque[i])
            elif self.control_mode == "servo":
                state = self.RUIWOTools.run_servo_mode(self.joint_address_list[i], pos[i], velocity[i], self.target_pos_kp[i], self.target_pos_kd[i], self.target_vel_kp[i], self.target_vel_kd[i], self.target_vel_ki[i])
            if isinstance(state, list):
                self.set_joint_state(i,state)
                if self.joint_address_list[i] == self.Head_joint_address[1]:
                    self.head_high_torque = self.measure_head_torque(state[1])

            
    def set_positions(self,index, positions, torque, velocity):
        if not self.running:
            return None 
        with self.sendposlock:
            new_positions = self.target_positions
            new_torque = self.target_torque
            new_velocity = self.target_velocity
        for i, address in enumerate(self.joint_address_list):
            for j, id in enumerate(index):
                if address - 1 == id:
                    if (address) in self.negtive_joint_address_list:
                        new_positions[i] = -positions[j]
                        new_torque[i] = max(-1, -torque[j])
                        new_velocity[i] = 0 * -velocity[j]
                    else:
                        new_positions[i] = positions[j]
                        new_torque[i] = min(1, torque[j])
                        new_velocity[i] = 0 * velocity[j]
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
        with self.statelock:
            self.joint_status[idex] = new_state

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
if __name__ == '__main__':
    joint_control = RuiWoActuator()
    x = 0
    for i in range(500):
        joint_control.set_positions([0,1,2,3,4,5,
                                     6,7,8,9,10,11,
                                     12,13], 
                                    [0.001*i,0.001*i,0.001*i,0.001*i,0.001*i,0.002*i,
                                    0.001*i,0.001*i,0.001*i,0.001*i,0.001*i,0.001*i,
                                    0.001*i,0.001*i], 
                                    [0,0,0,0,0,0,
                                     0,0,0,0,0,0,
                                     0,0], 
                                    [0,0,0,0,0,0,
                                     0,0,0,0,0,0,
                                     0,0])
        time.sleep(0.004)
    state = joint_control.get_joint_state()
    joint_control.close()
    print("[RUIWO motor]:End state:\n",state)

