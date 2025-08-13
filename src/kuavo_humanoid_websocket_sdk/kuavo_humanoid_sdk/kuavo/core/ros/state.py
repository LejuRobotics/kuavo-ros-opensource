import roslibpy
import time
import copy
from collections import deque
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK
from kuavo_humanoid_sdk.kuavo.core.ros.param import make_robot_param, EndEffectorType
from kuavo_humanoid_sdk.interfaces.data_types import (KuavoImuData, KuavoJointData, KuavoOdometry, KuavoManipulationMpcFrame, 
                                                      KuavoArmCtrlMode, EndEffectorState, KuavoDexHandTouchState,
                                                      KuavoManipulationMpcCtrlMode, KuavoManipulationMpcControlFlow)
class GaitManager:
    def __init__(self, max_size: int = 20):
        self._max_size = max_size
        self._gait_time_names = deque(maxlen=max_size)

    @property
    def is_empty(self) -> bool:
        """Check if there are any gait time names stored.
        
        Returns:
            bool: True if no gait time names are stored, False otherwise
        """
        return len(self._gait_time_names) == 0
    def add(self, start_time: float, name: str) -> None:
        self._gait_time_names.append((start_time, name))

    def get_gait(self, current_time: float) -> str:
        if not self._gait_time_names:
            return "No_Gait"

        for start_time, name in reversed(self._gait_time_names):
            if current_time >= start_time:
                return name

        return self._gait_time_names[0].name

    def get_gait_name(self, current_time: float) -> str:
        return self.get_gait(current_time)[1]

    def get_last_gait_name(self) -> str:
        if not self._gait_time_names:
            return "No_Gait"
        return self._gait_time_names[-1][1]

class KuavoRobotStateCoreWebsocket:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, '_initialized'):
            try:
                self.websocket = WebSocketKuavoSDK()
                if not self.websocket.client.is_connected:
                    SDKLogger.error("Failed to connect to WebSocket server")
                    raise ConnectionError("Failed to connect to WebSocket server")
                
                # Initialize subscribers
                self._sub_sensors_data = roslibpy.Topic(self.websocket.client, '/sensors_data_raw', 'kuavo_msgs/sensorsData')
                self._sub_odom = roslibpy.Topic(self.websocket.client, '/odom', 'nav_msgs/Odometry')
                self._sub_terrain_height = roslibpy.Topic(self.websocket.client, '/humanoid/mpc/terrainHeight', 'std_msgs/Float64')
                self._sub_gait_time_name = roslibpy.Topic(self.websocket.client, '/humanoid_mpc_gait_time_name', 'kuavo_msgs/gaitTimeName')
                self._sub_mpc_observation = roslibpy.Topic(self.websocket.client, '/humanoid_mpc_observation', 'ocs2_msgs/mpc_observation')
                
                # service calls are time-consuming after subscription, place them before subscription
                kuavo_info = make_robot_param()

                # Subscribe to topics
                self._sub_sensors_data.subscribe(self._sensors_data_raw_callback)
                self._sub_odom.subscribe(self._odom_callback)
                self._sub_terrain_height.subscribe(self._terrain_height_callback)
                self._sub_gait_time_name.subscribe(self._humanoid_mpc_gait_changed_callback)
                self._sub_mpc_observation.subscribe(self._humanoid_mpc_observation_callback)
                
                if kuavo_info is None:
                    SDKLogger.error("Failed to get robot parameters")
                    raise RuntimeError("Failed to get robot parameters")
                
                self._ee_type = kuavo_info['end_effector_type']
                if self._ee_type == EndEffectorType.LEJUCLAW:
                    self._sub_lejuclaw_state = roslibpy.Topic(self.websocket.client, '/leju_claw_state', 'kuavo_msgs/lejuClawState')
                    self._sub_lejuclaw_state.subscribe(self._lejuclaw_state_callback)
                elif self._ee_type == EndEffectorType.QIANGNAO:
                    self._sub_dexhand_state = roslibpy.Topic(self.websocket.client, '/dexhand/state', 'sensor_msgs/JointState')
                    self._sub_dexhand_state.subscribe(self._dexterous_hand_state_callback)
                elif self._ee_type == EndEffectorType.QIANGNAO_TOUCH:
                    self._sub_dexhand_state = roslibpy.Topic(self.websocket.client, '/dexhand/state', 'sensor_msgs/JointState')
                    self._sub_dexhand_touch_state = roslibpy.Topic(self.websocket.client, '/dexhand/touch_state', 'kuavo_msgs/dexhandTouchState')
                    self._sub_dexhand_state.subscribe(self._dexterous_hand_state_callback)
                    self._sub_dexhand_touch_state.subscribe(self._dexhand_touch_state_callback)
                    # Initialize touch state for both hands
                    self._dexhand_touch_state = (
                        KuavoDexHandTouchState(
                            data=(KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                                  KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                                  KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                                  KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                                  KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                        ),
                        KuavoDexHandTouchState(
                            data=(KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                                  KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                                  KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                                  KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                                  KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                        )
                    )

                """ data """
                self._terrain_height = 0.0
                self._imu_data = KuavoImuData(
                    gyro = (0.0, 0.0, 0.0),
                    acc = (0.0, 0.0, 0.0),
                    free_acc = (0.0, 0.0, 0.0),
                    quat = (0.0, 0.0, 0.0, 0.0)
                )
                self._joint_data = KuavoJointData(
                    position = [0.0] * 28,
                    velocity = [0.0] * 28,
                    torque = [0.0] * 28,
                    acceleration = [0.0] * 28
                )
                self._odom_data = KuavoOdometry(
                    position = (0.0, 0.0, 0.0),
                    orientation = (0.0, 0.0, 0.0, 0.0),
                    linear = (0.0, 0.0, 0.0),
                    angular = (0.0, 0.0, 0.0)
                )
                self._eef_state = (EndEffectorState(
                    position = [0.0] * 12 if kuavo_info['end_effector_type'] is not None and kuavo_info['end_effector_type'].startswith('qiangnao') else [0.0] * 2,
                    velocity = [0.0] * 12 if kuavo_info['end_effector_type'] is not None and  kuavo_info['end_effector_type'].startswith('qiangnao') else [0.0] * 2,
                    effort = [0.0] * 12 if kuavo_info['end_effector_type'] is not None and  kuavo_info['end_effector_type'].startswith('qiangnao') else [0.0] * 2,
                    state=EndEffectorState.GraspingState.UNKNOWN
                ), EndEffectorState(
                    position = [0.0] * 12 if kuavo_info['end_effector_type'] is not None and  kuavo_info['end_effector_type'].startswith('qiangnao') else [0.0] * 2,
                    velocity = [0.0] * 12 if kuavo_info['end_effector_type'] is not None and  kuavo_info['end_effector_type'].startswith('qiangnao') else [0.0] * 2,
                    effort = [0.0] * 12 if kuavo_info['end_effector_type'] is not None and  kuavo_info['end_effector_type'].startswith('qiangnao') else [0.0] * 2,
                    state=EndEffectorState.GraspingState.UNKNOWN
                ))
                                
                # gait manager
                self._gait_manager = GaitManager()
                self._prev_gait_name = self.gait_name()

                # Wait for first MPC observation data
                self._mpc_observation_data = None
                start_time = time.time()
                while self._mpc_observation_data is None:
                    if time.time() - start_time > 1.0:  # 1.0s timeout
                        start_time = time.time()
                        SDKLogger.warn("Timeout waiting for MPC observation data")
                        # break
                    SDKLogger.debug("Waiting for first MPC observation data...")
                    time.sleep(0.1)

                if self._mpc_observation_data is not None:
                    if self._gait_manager.is_empty:
                        self._prev_gait_name = self.gait_name()
                        SDKLogger.debug(f"[State] Adding initial gait state: {self._prev_gait_name} at time {self._mpc_observation_data['time']}")
                        self._gait_manager.add(self._mpc_observation_data['time'], self._prev_gait_name)

                # 获取当前手臂控制模式
                self._arm_ctrl_mode = self._srv_get_arm_ctrl_mode()
                
                # 获取manipulation mpc 相关参数
                self._manipulation_mpc_frame = self._srv_get_manipulation_mpc_frame()
                self._manipulation_mpc_ctrl_mode = self._srv_get_manipulation_mpc_ctrl_mode()
                self._manipulation_mpc_control_flow = self._srv_get_manipulation_mpc_control_flow()
                
                self._initialized = True
            except Exception as e:
                SDKLogger.error(f"Failed to initialize KuavoRobotStateCoreWebsocket: {e}")
                raise

    @property
    def com_height(self)->float:
        return self._odom_data.position[2] - self._terrain_height
    
    @property
    def imu_data(self):
        return self._imu_data
    
    @property
    def joint_data(self):
        return self._joint_data
    
    @property
    def odom_data(self):
        return self._odom_data

    @property
    def arm_control_mode(self):
        mode = self._srv_get_arm_ctrl_mode()
        if mode is not None:
            self._arm_ctrl_mode = mode
        return self._arm_ctrl_mode
    
    @property
    def manipulation_mpc_ctrl_mode(self):
        mode = self._srv_get_manipulation_mpc_ctrl_mode()
        if mode is not None:
            self._manipulation_mpc_ctrl_mode = mode
        return self._manipulation_mpc_ctrl_mode
    
    @property
    def manipulation_mpc_frame(self):
        frame = self._srv_get_manipulation_mpc_frame()
        if frame is not None:
            self._manipulation_mpc_frame = frame
        return self._manipulation_mpc_frame
    
    @property
    def manipulation_mpc_control_flow(self):
        flow = self._srv_get_manipulation_mpc_control_flow()
        if flow is not None:
            self._manipulation_mpc_control_flow = flow
        return self._manipulation_mpc_control_flow
    
    @property
    def eef_state(self):
        return self._eef_state
    
    @property
    def dexhand_touch_state(self):
        if self._ee_type != EndEffectorType.QIANGNAO_TOUCH:
            raise Exception("This robot does not support touch state")
        return self._dexhand_touch_state

    @property
    def current_observation_time(self) -> float:
        if self._mpc_observation_data is None:
            return None
        return self._mpc_observation_data['time']
    
    @property
    def current_gait_mode(self) -> int:
        if self._mpc_observation_data is None:
            return None
        return self._mpc_observation_data['mode']

    def gait_name(self)->str:
        return self._srv_get_current_gait_name()
    
    def is_gait(self, gait_name: str) -> bool:
        return self._gait_manager.get_gait(self._mpc_observation_data['time']) == gait_name

    def register_gait_changed_callback(self, callback):
        if not hasattr(self, '_gait_changed_callbacks'):
            self._gait_changed_callbacks = []
        
        from inspect import signature
        sig = signature(callback)
        if len(sig.parameters) != 2:
            raise ValueError("Callback must accept 2 parameters: current_time (float), gait_name (str)")
        if callback not in self._gait_changed_callbacks:
            self._gait_changed_callbacks.append(callback)

    def _terrain_height_callback(self, msg)->None:
        self._terrain_height = msg['data']

    def _sensors_data_raw_callback(self, msg)->None:
        # update imu data
        self._imu_data = KuavoImuData(
            gyro = (msg['imu_data']['gyro']['x'], msg['imu_data']['gyro']['y'], msg['imu_data']['gyro']['z']),
            acc = (msg['imu_data']['acc']['x'], msg['imu_data']['acc']['y'], msg['imu_data']['acc']['z']),
            free_acc = (msg['imu_data']['free_acc']['x'], msg['imu_data']['free_acc']['y'], msg['imu_data']['free_acc']['z']),
            quat = (msg['imu_data']['quat']['x'], msg['imu_data']['quat']['y'], msg['imu_data']['quat']['z'], msg['imu_data']['quat']['w'])
        )
        # update joint data
        self._joint_data = KuavoJointData(
            position = copy.deepcopy(msg['joint_data']['joint_q']),
            velocity = copy.deepcopy(msg['joint_data']['joint_v']),
            torque = copy.deepcopy(msg['joint_data']['joint_vd']),
            acceleration = copy.deepcopy(msg['joint_data'].get('joint_current', msg['joint_data']['joint_torque']))
        )

    def _odom_callback(self, msg)->None:
        # update odom data
        self._odom_data = KuavoOdometry(
            position = (msg['pose']['pose']['position']['x'], msg['pose']['pose']['position']['y'], msg['pose']['pose']['position']['z']),
            orientation = (msg['pose']['pose']['orientation']['x'], msg['pose']['pose']['orientation']['y'], msg['pose']['pose']['orientation']['z'], msg['pose']['pose']['orientation']['w']),
            linear = (msg['twist']['twist']['linear']['x'], msg['twist']['twist']['linear']['y'], msg['twist']['twist']['linear']['z']),
            angular = (msg['twist']['twist']['angular']['x'], msg['twist']['twist']['angular']['y'], msg['twist']['twist']['angular']['z'])
        )

    def _lejuclaw_state_callback(self, msg)->None:
        self._eef_state = (EndEffectorState(
            position = [msg['data']['position'][0]],
            velocity = [msg['data']['velocity'][0]],
            effort = [msg['data']['effort'][0]],
            state=EndEffectorState.GraspingState(msg['state'][0])
        ), EndEffectorState(
            position = [msg['data']['position'][1]],
            velocity = [msg['data']['velocity'][1]],
            effort = [msg['data']['effort'][1]],
            state=EndEffectorState.GraspingState(msg['state'][1])
        ))

    def _dexterous_hand_state_callback(self, msg)->None:
        self._eef_state = (EndEffectorState(
            position = list(msg['position'][:len(msg['position'])//2]),
            velocity = list(msg['velocity'][:len(msg['velocity'])//2]),
            effort = list(msg['effort'][:len(msg['effort'])//2]),
            state=EndEffectorState.GraspingState.UNKNOWN
        ), EndEffectorState(
            position = list(msg['position'][len(msg['position'])//2:]),
            velocity = list(msg['velocity'][len(msg['velocity'])//2:]),
            effort = list(msg['effort'][len(msg['effort'])//2:]),
            state=EndEffectorState.GraspingState.UNKNOWN
        ))

    def _dexhand_touch_state_callback(self, msg)->None:
        self._dexhand_touch_state = (
            KuavoDexHandTouchState(
                data=tuple(KuavoDexHandTouchState.KuavoTouchState(
                    sensor['normal_force1'], sensor['normal_force2'], sensor['normal_force3'],
                    sensor['tangential_force1'], sensor['tangential_force2'], sensor['tangential_force3'],
                    sensor['tangential_direction1'], sensor['tangential_direction2'], sensor['tangential_direction3'],
                    sensor['self_proximity1'], sensor['self_proximity2'], sensor['mutual_proximity'],
                    sensor['status']
                ) for sensor in msg['left_hand'])
            ),
            KuavoDexHandTouchState(
                data=tuple(KuavoDexHandTouchState.KuavoTouchState(
                    sensor['normal_force1'], sensor['normal_force2'], sensor['normal_force3'],
                    sensor['tangential_force1'], sensor['tangential_force2'], sensor['tangential_force3'],
                    sensor['tangential_direction1'], sensor['tangential_direction2'], sensor['tangential_direction3'],
                    sensor['self_proximity1'], sensor['self_proximity2'], sensor['mutual_proximity'],
                    sensor['status']
                ) for sensor in msg['right_hand'])
            )
        )

    def _humanoid_mpc_gait_changed_callback(self, msg):
        SDKLogger.debug(f"[State] Received gait change message: {msg['gait_name']} at time {msg['start_time']}")
        self._gait_manager.add(msg['start_time'], msg['gait_name'])
    
    def _humanoid_mpc_observation_callback(self, msg) -> None:
        try:
            # SDKLogger.debug(f"[State] Received MPC observation message: {msg}")
            self._mpc_observation_data = msg
            if hasattr(self, '_initialized'): 
                curr_time = self._mpc_observation_data['time']
                current_gait = self._gait_manager.get_gait(curr_time)
                if self._prev_gait_name != current_gait:
                    SDKLogger.debug(f"[State] Gait changed to: {current_gait} at time {curr_time}")
                    self._prev_gait_name = current_gait
                    if hasattr(self, '_gait_changed_callbacks') and self._gait_changed_callbacks is not None:
                        for callback in self._gait_changed_callbacks:
                            callback(curr_time, current_gait)
        except Exception as e:
            SDKLogger.error(f"Error processing MPC observation: {e}")

    def _srv_get_arm_ctrl_mode(self):
        try:
            service = roslibpy.Service(self.websocket.client, '/humanoid_get_arm_ctrl_mode', 'kuavo_msgs/changeArmCtrlMode')
            request = {}
            response = service.call(request)
            return KuavoArmCtrlMode(response.get('mode', 0))
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
        return None
    
    def _srv_get_current_gait_name(self)->str:
        try:
            service = roslibpy.Service(self.websocket.client, '/humanoid_get_current_gait_name', 'kuavo_msgs/getCurrentGaitName')
            request = {}
            response = service.call(request)
            if response.get('success', False):
                return response['gait_name']
            return None
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
        return None

    def _srv_get_manipulation_mpc_ctrl_mode(self, ):
        try:
            service_name = '/mobile_manipulator_get_mpc_control_mode'
            service = roslibpy.Service(self.websocket.client, service_name, 'kuavo_msgs/changeTorsoCtrlMode')
            request = {}
            response = service.call(request)
            if not response.get('result', False):
                SDKLogger.error(f"Failed to get manipulation mpc control mode: {response.get('message', '')}")
                return KuavoManipulationMpcCtrlMode.ERROR
            return KuavoManipulationMpcCtrlMode(response.get('mode', 0))
        except Exception as e:
            SDKLogger.error(f"Failed to get manipulation mpc control mode: {e}")
        return KuavoManipulationMpcCtrlMode.ERROR

    def _srv_get_manipulation_mpc_frame(self, ):
        try:
            service_name = '/get_mm_ctrl_frame'
            service = roslibpy.Service(self.websocket.client, service_name, 'kuavo_msgs/setMmCtrlFrame')
            request = {}
            response = service.call(request)
            if not response.get('result', False):
                SDKLogger.error(f"Failed to get manipulation mpc frame: {response.get('message', '')}")
                return KuavoManipulationMpcFrame.ERROR
            return KuavoManipulationMpcFrame(response.get('currentFrame', 0))
        except Exception as e:
            SDKLogger.error(f"Failed to get manipulation mpc frame: {e}")
        return KuavoManipulationMpcFrame.ERROR

    def _srv_get_manipulation_mpc_control_flow(self, ):
        try:
            service_name = '/get_mm_wbc_arm_trajectory_control'
            service = roslibpy.Service(self.websocket.client, service_name, 'kuavo_msgs/changeArmCtrlMode')
            request = {}
            response = service.call(request)
            if not response.get('result', False):
                SDKLogger.error(f"Failed to get manipulation mpc wbc arm trajectory control mode: {response.get('message', '')}")
                return KuavoManipulationMpcControlFlow.Error
            return KuavoManipulationMpcControlFlow(response.get('mode', 0))
        except Exception as e:
            SDKLogger.error(f"Failed to get manipulation mpc wbc arm trajectory control mode: {e}")
        return KuavoManipulationMpcControlFlow.Error

# if __name__ == "__main__":
#     state = KuavoRobotStateCore()
#     print(state.manipulation_mpc_frame)
#     print(state.manipulation_mpc_control_flow)
#     print(state.manipulation_mpc_ctrl_mode)
#     print(state.arm_control_mode)
