import roslibpy
import time
from typing import Tuple
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK


class LaunchRobotStatus:
    # 与硬件节点保持一致
    UNLAUNCH = 'unlaunch'
    INITING = 'initing'
    UNKNOWN = 'unknown'
    READY_STANCE = 'ready_stance'
    CALIBRATE = 'calibrate'
    LAUNCHED = 'launched'

class LaunchRobotTool:
    def __init__(self):
        self._websocket = WebSocketKuavoSDK()
        self._stand_timeout = 10.0
        self._srv_robot_start = roslibpy.Service(self._websocket.client, '/websocket_sdk_srv/start_robot', 'std_srvs/Trigger')
        self._srv_robot_stand = roslibpy.Service(self._websocket.client, '/websocket_sdk_srv/stand_robot', 'std_srvs/Trigger')
        self._srv_get_robot_launch_status = roslibpy.Service(self._websocket.client, '/websocket_sdk_srv/get_robot_launch_status', 'std_srvs/Trigger')
        self._srv_robot_stop = roslibpy.Service(self._websocket.client, '/websocket_sdk_srv/stop_robot', 'std_srvs/Trigger')
        
    def is_unlaunch(self, status:str)->bool:
        return status == LaunchRobotStatus.UNLAUNCH or status == LaunchRobotStatus.UNKNOWN
    
    def is_launched(self, status:str)->bool:
        return status == LaunchRobotStatus.LAUNCHED

    def is_launching(self, status:str)->bool:
        return self.is_calibrate(status) or self.is_ready_stance(status)

    def is_calibrate(self, status:str)->bool:
        return status == LaunchRobotStatus.CALIBRATE
    
    def is_ready_stance(self, status:str)->bool:
        return status == LaunchRobotStatus.READY_STANCE
    
    def robot_start(self) -> bool:
        # launch robot
        if not self.pub_start_command():
            return False
        try:
            timeout = 120 # 120s 等待进入校准或准备姿态
            start_time = time.time()
            last_print = time.time()
            print("\033[32m>_ 机器人启动中\033[0m", end="", flush=True)
            while True:
                if time.time() - start_time > timeout:
                    SDKLogger.error("Robot launch timed out")
                    return False
                success, status = self.get_robot_launch_status()
                # 如果机器人启动成功，已经进入校准或准备状态或者launched状态，则启动成功
                if success and (self.is_launched(status) or self.is_launching(status)):
                    print("\n")
                    return True
                
                current_time = time.time()
                if current_time - last_print >= 0.8:
                    print("\033[32m.\033[0m", end="", flush=True)
                    last_print = current_time
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\n")
            SDKLogger.info("Received keyboard interrupt, stopping robot launch")
            return False
    
    def robot_stop(self)->bool:
        """
        停止机器人
        """
        try:
            request = roslibpy.ServiceRequest({})
            response = self._srv_robot_stop.call(request)
            return response.get('success', False)
        except Exception as e:
            SDKLogger.error(f"[Error] calling stop robot service: {e}")
            return False

    def robot_stand(self)->bool:
        try:
            while True:
                # 获取当前启动状态, 如果已启动则返回
                success, status = self.get_robot_launch_status()
                if not success:
                    SDKLogger.error(f"[Error] Failed to get robot launch status: {status}")
                    return False
                if self.is_launched(status):
                    return True
                
                # 从当前状态确认下一个状态
                next_state = None
                if self.is_calibrate(status):
                    print("\033[32m>_ [校准状态]: 机器人当前处于校准状态，确认无误，请输入 y 键进入准备姿态\033[0m")
                    next_state = LaunchRobotStatus.READY_STANCE
                elif self.is_ready_stance(status):
                    print("\033[32m>_ [准备姿态]: 请等待机器人缩腿到下蹲姿势，并将其下降到距离地面 2cm 处，确认无误后，请输入 y 键\033[0m")
                    next_state = LaunchRobotStatus.LAUNCHED

                if self.is_calibrate(status) or self.is_ready_stance(status):
                    while True:
                        try:
                            choice = input().lower()
                            if choice == 'y':
                                break
                            elif choice == 'n':
                                return False
                            else:
                                SDKLogger.error("输入错误，请输入 y 键")
                        except KeyboardInterrupt:
                            return False
                    
                    # 发布站立命令
                    if not self.pub_stand_command():
                        return False

                    # 等待进入下一个状态
                    start_time = time.time()
                    while True:
                        if time.time() - start_time > self._stand_timeout: 
                            break
                        success, current_status = self.get_robot_launch_status()
                        if not success:
                            SDKLogger.error(f"[Error] Wait next state Failed to get robot launch status: {current_status}")
                            return False
                        if self.is_launched(current_status):
                            return True
                        elif current_status == next_state:
                            # 成功进入下一个状态
                            break
                        time.sleep(0.2)
                time.sleep(0.2)
        except KeyboardInterrupt:
            SDKLogger.info("Received keyboard interrupt, stopping robot stand")
            return False

    def pub_start_command(self) -> bool:
        try:
            request = roslibpy.ServiceRequest({})
            response = self._srv_robot_start.call(request)
            result = response.get('success', False)
            return result
        except Exception as e:
            SDKLogger.error(f"[Error] calling start service: {e}")
            return False

    def pub_stand_command(self) -> bool:
        try:
            request = {}
            response = self._srv_robot_stand.call(request)
            if not response.get('success', False):
                SDKLogger.error(f"Failed to make robot stand: {response.get('message', '')}")
            return response.get('success', False)
        except Exception as e:
            SDKLogger.error(f"[Error] calling stand service: {e}")
            return False
        

    def get_robot_launch_status(self)->Tuple[bool, str]:
        try:
            request = roslibpy.ServiceRequest({})
            response = self._srv_get_robot_launch_status.call(request)
            success = response.get('success', False)
            status = response.get('message', 'unknown')
            return success, status
        except Exception as e:
            SDKLogger.error(f"[Error] calling get robot launch status service: {e}")
            return False, "unknown"