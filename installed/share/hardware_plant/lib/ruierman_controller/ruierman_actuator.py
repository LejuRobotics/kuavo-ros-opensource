import sys
import os
import time
import signal
import threading
current_path =os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_path)
sys.path.append('/usr/lib/python3/dist-packages')
# print(sys.path)
import numpy as np

from SimpleSDK import WHJ30Tools

# 睿尔曼关节电机地址
Left_joint_address = 0x01
Right_joint_address = 0x02
joint_address_list = [Left_joint_address, Right_joint_address]

# 电机控制模式地址
mode_position_address = 0x03
# 控制周期
dt=0.004
max_speed = 60 #插值规划的速度
class RuiErManActuator():
    def __init__(self):
        self.running = True
        self.whj30 = WHJ30Tools()
        self.whj30.open_canbus()
        self.sendlock = threading.Lock()
        self.recvlock = threading.Lock()
        self.target_update = False
        self.target_positions = [0]*len(joint_address_list)
        self.old_target_positions = [0]*len(joint_address_list)
        self.current_positions = [0]*len(joint_address_list)
        for address in joint_address_list:
            # 跳过IAP更新操作，传入关节ID 0x01 -- 左手
            self.whj30.skip_iap_update(address)
            # self.whj30.set_joint_min_position(address, -10)
            # self.whj30.set_joint_max_position(address, 50)

        # 等待3s
        # time.sleep(3)
        if not self.enable():
            raise RuntimeError("RuiErManActuator Initialization failed!")
        self.go_to_zero()

        self.control_thread = threading.Thread(target=self.control_thread)
        self.control_thread.start()
        # 设置范围
        # self.whj30.set_joint_min_position(Left_joint_address, -10)
        # self.whj30.set_joint_max_position(Left_joint_address, 50)

        # self.whj30.set_joint_min_position(Right_joint_address, -10)
        # self.whj30.set_joint_max_position(Right_joint_address, 50)
        time.sleep(1)

    def control_thread(self):
        print("threadstart")
        target_positions = self.target_positions
        # self.old_target_positions = self.target_positions
        # old_target_positions = self.target_positions
        try:
            current_possitions = [0]*len(joint_address_list)
            while self.running:
                time.sleep(dt)
                # 读取当前位置
                current_possitions = [motor[5] for motor in self.get_joint_state() if type(motor) == list]
                with self.recvlock:
                    self.current_positions = current_possitions
                # print("Current Positions:", self.current_positions)

                if self.target_update == False:
                    continue
                with self.sendlock:
                    target_positions = self.target_positions
                    self.target_update = False
                # self.interpolate_move(self.old_target_positions, target_positions, speed=max_speed, dt=0.01)
                self.send_positions(range(len(joint_address_list)), target_positions)

        except Exception as e:
            print(e)
        print("threadend")
            
    def join(self):
        self.control_thread.join()
        
    def interpolate_positions_with_speed(self,a, b, speed, dt=0.01):
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
        # interpolation_list = [interpolation[:, i] for i in range(interpolation.shape[1])]
        interpolation_list = [[interpolation[i, j] for j in range(interpolation.shape[1])] for i in range(interpolation.shape[0])]
        # print(len(interpolation_list))
        # interpolation_list = [np.pad(interpolation[:, j], (0, max_length - interpolation.shape[0]), constant_values=0.0) for j in range(interpolation.shape[1])]


        return interpolation_list
            
    def go_to_zero(self):
        print("start moving to zero")
        state = self.get_joint_state()
        current_positions = [motor[5] for motor in state]
        target_positions = [0]*len(joint_address_list)
        self.interpolate_move(current_positions, target_positions,speed=max_speed)
        print("moved to zero")
        
    def interpolate_move(self,start_positions, target_positions, speed=50, dt=0.01):
        interpolation_list = self.interpolate_positions_with_speed(start_positions, target_positions, speed, dt)
        for target_position in interpolation_list:
            # print("interpolation_list",target_position)        
            self.send_positions(range(len(joint_address_list)), target_position)
            self.old_target_positions = target_position
            time.sleep(dt)
            with self.recvlock:
                self.current_positions = [motor[5] for motor in self.get_joint_state()]
        
    def enable(self):
        self.disable()
        for i, address in enumerate(joint_address_list):
            self.whj30.enable_joint(address)
            self.whj30.set_joint_operate_mode(address, mode_position_address)  # 设置工作模式为位置模式
            self.whj30.clear_joint_error(address)
            state = self.whj30.get_joint_state(address)
            print(f"ruierman actuator enable joint {i}, state: {state}")
            if type(state) == bool:
                print(f"ruierman actuator enable joint {i} failed")
                return False
        return True
        
    def disable(self):
        for address in joint_address_list:
            print(f"ruierman actuator disable joint {address}")
            # self.whj30.disable_joint(address)
            
    def set_zero_position(self):
        for address in joint_address_list:
            self.whj30.set_joint_zero_position(address)
            self.whj30.save_joint_param(address)
            # self.whj30.clear_joint_error(address)
            
    def send_positions(self, ids, position):
        for i,index in enumerate(ids):
            # print(f"ruierman actuator set joint {joint_address_list[index]} position {position[i]}")
            self.whj30.set_joint_position(joint_address_list[index], position[i])
            self.whj30.clear_joint_error(joint_address_list[i])
            
    def set_positions(self,ids, positions):
        if not self.running:
            return None 
        new_positions = self.target_positions
        for i,index in enumerate(ids):
            new_positions[index] = positions[i]
        with self.sendlock:
            self.target_positions = new_positions
            self.target_update = True
                        
    def get_positions(self):
        if not self.running:
            return None 
        position_list = []
        with self.recvlock:
            position_list = self.current_positions
        return position_list

    def get_joint_state(self):
        state_list = []
        for address in joint_address_list:
            # （错误代码，系统电压，系统温度，使能状态，抱闸状态，当前位置，当前电流），失败返回False
            state = self.whj30.get_joint_state(address)
            # state = [1,2,5,8,9,10,2]
            # print(f"ruierman actuator get joint {address} state {state}")
            state_list.append(state)

        return state_list
    
    def close(self):
        self.running = False
        self.disable()
        print("ruierman actuator disabled")
        # self.whj30.close_canbus()



if __name__ == '__main__':
    joint_control = RuiErManActuator()
    time.sleep(1)
    # joint_control.set_positions([0,1],[20,-20])
    # joint_control.send_positions([0,1],[15,32])
    # joint_control.get_joint_state()
    # joint_control.go_to_zero()
    # joint_control.set_zero_position()
    # joint_control.send_positions([0],[5])
    # joint_control.close()
    try:
        i = 0
        while True:
            i+=1
            # joint_control.set_positions([0,1],[0.2*i,-0.2*i])
            time.sleep(0.01)
            # joint_control.get_joint_state()
            # time.sleep(1)
    except Exception as e :
        print(e)
    joint_control.close()
