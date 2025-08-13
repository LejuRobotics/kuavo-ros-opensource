import time
from typing import Tuple
import copy
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolRequest
from sensor_msgs.msg import JointState
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import sensorsData, lejuClawState, gaitTimeName, dexhandTouchState
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, getCurrentGaitName, getCurrentGaitNameRequest
from kuavo_humanoid_sdk.msg.ocs2_msgs.msg import mpc_observation
from kuavo_humanoid_sdk.msg.kuavo_msgs.srv import (changeArmCtrlMode, changeArmCtrlModeRequest,setMmCtrlFrame, setMmCtrlFrameRequest, changeTorsoCtrlMode, changeTorsoCtrlModeRequest)
from collections import deque
from typing import Tuple
import copy
from kuavo_humanoid_sdk.common.logger import SDKLogger
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


class KuavoRobotStateCore:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, '_initialized'):
            rospy.Subscriber("/sensors_data_raw", sensorsData, self._sensors_data_raw_callback)
            rospy.Subscriber("/odom", Odometry, self._odom_callback)
            rospy.Subscriber("/humanoid/mpc/terrainHeight", Float64, self._terrain_height_callback)
            rospy.Subscriber("/humanoid_mpc_gait_time_name", gaitTimeName, self._humanoid_mpc_gait_changed_callback)
            rospy.Subscriber("/humanoid_mpc_observation", mpc_observation, self._humanoid_mpc_observation_callback)
            
            kuavo_info = make_robot_param()
            self._ee_type = kuavo_info['end_effector_type']
            if self._ee_type == EndEffectorType.LEJUCLAW:
                rospy.Subscriber("/leju_claw_state", lejuClawState, self._lejuclaw_state_callback)
            elif self._ee_type == EndEffectorType.QIANGNAO:
                rospy.Subscriber("/dexhand/state", JointState, self._dexterous_hand_state_callback)
            elif self._ee_type == EndEffectorType.QIANGNAO_TOUCH:
                rospy.Subscriber("/dexhand/state", JointState, self._dexterous_hand_state_callback)
                rospy.Subscriber("/dexhand/touch_state", dexhandTouchState, self._dexhand_touch_state_callback)
                # Initialize touch state for both hands with empty KuavoDexHandTouchState instances
                self._dexhand_touch_state = (
                    KuavoDexHandTouchState(
                        data=(KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                              KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                              KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                              KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                              KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                    ),  # Left hand touch state
                    KuavoDexHandTouchState(
                        data=(KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                              KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                              KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                              KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                              KuavoDexHandTouchState.KuavoTouchState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                    )   # Right hand touch state
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
                velocity = [0.0] * 12 if kuavo_info['end_effector_type'] is not None and kuavo_info['end_effector_type'].startswith('qiangnao') else [0.0] * 2,
                effort = [0.0] * 12 if kuavo_info['end_effector_type'] is not None and kuavo_info['end_effector_type'].startswith('qiangnao') else [0.0] * 2,
                state=EndEffectorState.GraspingState.UNKNOWN
            ), EndEffectorState(
                position = [0.0] * 12 if kuavo_info['end_effector_type'] is not None and kuavo_info['end_effector_type'].startswith('qiangnao') else [0.0] * 2,
                velocity = [0.0] * 12 if kuavo_info['end_effector_type'] is not None and kuavo_info['end_effector_type'].startswith('qiangnao') else [0.0] * 2,
                effort = [0.0] * 12 if kuavo_info['end_effector_type'] is not None and kuavo_info['end_effector_type'].startswith('qiangnao') else [0.0] * 2,
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
                    SDKLogger.warn("Timeout waiting for MPC observation data")
                    break
                SDKLogger.debug("Waiting for first MPC observation data...")
                time.sleep(0.1)
            # 如果 gait_manager 为空，则把当前的状态添加到其中
            if self._mpc_observation_data is not None:
                if self._gait_manager.is_empty:
                    self._prev_gait_name = self.gait_name()
                    SDKLogger.debug(f"[State] Adding initial gait state: {self._prev_gait_name} at time {self._mpc_observation_data.time}")
                    self._gait_manager.add(self._mpc_observation_data.time, self._prev_gait_name)

            # 获取当前手臂控制模式
            self._arm_ctrl_mode = self._srv_get_arm_ctrl_mode()
            self._initialized = True

            # 获取manipulation mpc 相关参数
            self._manipulation_mpc_frame = self._srv_get_manipulation_mpc_frame()
            self._manipulation_mpc_ctrl_mode = self._srv_get_manipulation_mpc_ctrl_mode()
            self._manipulation_mpc_control_flow = self._srv_get_manipulation_mpc_control_flow()

    @property
    def com_height(self)->float:
        # odom.position.z - terrain_height = com_height
        return self._odom_data.position[2] - self._terrain_height
    
    @property
    def imu_data(self)->KuavoImuData:
        return self._imu_data
    
    @property
    def joint_data(self)->KuavoJointData:
        return self._joint_data
    
    @property
    def odom_data(self)->KuavoOdometry:
        return self._odom_data

    @property
    def arm_control_mode(self) -> KuavoArmCtrlMode:
        mode = self._srv_get_arm_ctrl_mode()
        if mode is not None:
            self._arm_ctrl_mode = mode
        return self._arm_ctrl_mode
    
    @property
    def manipulation_mpc_ctrl_mode(self)->KuavoManipulationMpcCtrlMode:
        mode = self._srv_get_manipulation_mpc_ctrl_mode()
        if mode is not None:
            self._manipulation_mpc_ctrl_mode = mode
        return self._manipulation_mpc_ctrl_mode
    
    @property
    def manipulation_mpc_frame(self)->KuavoManipulationMpcFrame:
        frame = self._srv_get_manipulation_mpc_frame()
        if frame is not None:
            self._manipulation_mpc_frame = frame
        return self._manipulation_mpc_frame
    
    @property
    def manipulation_mpc_control_flow(self)->KuavoManipulationMpcControlFlow:
        flow = self._srv_get_manipulation_mpc_control_flow()
        if flow is not None:
            self._manipulation_mpc_control_flow = flow
        return self._manipulation_mpc_control_flow
    
    @property
    def pitch_limit_enabled(self)->bool:
        success, status = self._srv_get_pitch_limit_status()
        if success:
            return not ('disabled' in status)
        else:
            return False
    
    @property
    def eef_state(self)->Tuple[EndEffectorState, EndEffectorState]:
        return self._eef_state
    
    @property
    def dexhand_touch_state(self)-> Tuple[KuavoDexHandTouchState, KuavoDexHandTouchState]:
        if self._ee_type != EndEffectorType.QIANGNAO_TOUCH:
            raise Exception("This robot does not support touch state")
        return self._dexhand_touch_state

    @property
    def current_observation_time(self) -> float:
        """Get the current observation time from MPC.
        
        Returns:
            float: Current observation time in seconds, or None if no observation data available
        """
        if self._mpc_observation_data is None:
            return None
        return self._mpc_observation_data.time
    
    @property
    def current_gait_mode(self) -> int:
        """
        Get the current gait mode from MPC observation.
        
        Notes: FF=0, FH=1, FT=2, FS=3, HF=4, HH=5, HT=6, HS=7, TF=8, TH=9, TT=10, TS=11, SF=12, SH=13, ST=14, SS=15
        
        Returns:
            int: Current gait mode, or None if no observation data available
        """
        if self._mpc_observation_data is None:
            return None
        return self._mpc_observation_data.mode

    def gait_name(self)->str:
        return self._srv_get_current_gait_name()
    
    def is_gait(self, gait_name: str) -> bool:
        """Check if current gait matches the given gait name.
        
        Args:
            gait_name (str): Name of gait to check
            
        Returns:
            bool: True if current gait matches given name, False otherwise
        """
        return self._gait_manager.get_gait(self._mpc_observation_data.time) == gait_name

    def register_gait_changed_callback(self, callback):
        """Register a callback function that will be called when the gait changes.
        
        Args:
            callback: A function that takes current time (float) and gait name (str) as arguments
        Raises:
            ValueError: If callback does not accept 2 parameters: current_time (float), gait_name (str)
        """
        if not hasattr(self, '_gait_changed_callbacks'):
            self._gait_changed_callbacks = []
        
        # Verify callback accepts current_time (float) and gait_name (str) parameters
        from inspect import signature
        sig = signature(callback)
        if len(sig.parameters) != 2:
            raise ValueError("Callback must accept 2 parameters: current_time (float), gait_name (str)")
        if callback not in self._gait_changed_callbacks:
            self._gait_changed_callbacks.append(callback)
        
    """ ------------------------------- callback ------------------------------- """
    def _terrain_height_callback(self, msg)->None:
        self._terrain_height = msg.data

    def _sensors_data_raw_callback(self, msg)->None:
        # update imu data
        self._imu_data = KuavoImuData(
            gyro = (msg.imu_data.gyro.x, msg.imu_data.gyro.y, msg.imu_data.gyro.z),
            acc = (msg.imu_data.acc.x, msg.imu_data.acc.y, msg.imu_data.acc.z),
            free_acc = (msg.imu_data.free_acc.x, msg.imu_data.free_acc.y, msg.imu_data.free_acc.z),
            quat = (msg.imu_data.quat.x, msg.imu_data.quat.y, msg.imu_data.quat.z, msg.imu_data.quat.w)
        )
        # update joint data
        self._joint_data = KuavoJointData(
            position = copy.deepcopy(msg.joint_data.joint_q),
            velocity = copy.deepcopy(msg.joint_data.joint_v),
            torque = copy.deepcopy(msg.joint_data.joint_vd),
            acceleration = copy.deepcopy(msg.joint_data.joint_current if hasattr(msg.joint_data, 'joint_current') else msg.joint_data.joint_torque)
        )

    def _odom_callback(self, msg)->None:
        # update odom data
        self._odom_data = KuavoOdometry(
            position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
            orientation = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            linear = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z),
            angular = (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z)
        )

    def _lejuclaw_state_callback(self, msg)->None:
        self._eef_state = (EndEffectorState(
            # left claw
            position = [msg.data.position[0]],
            velocity = [msg.data.velocity[0]],
            effort = [msg.data.effort[0]],
            state=EndEffectorState.GraspingState(msg.state[0])
        ), EndEffectorState(
            # right claw
            position = [msg.data.position[1]],
            velocity = [msg.data.velocity[1]],
            effort = [msg.data.effort[1]],
            state=EndEffectorState.GraspingState(msg.state[1])
        ))

    def _dexterous_hand_state_callback(self, msg)->None:
        self._eef_state = (EndEffectorState(
            # left hand
            position = list(msg.position[:len(msg.position)//2]),
            velocity = list(msg.velocity[:len(msg.velocity)//2]),
            effort = list(msg.effort[:len(msg.effort)//2]),
            state=EndEffectorState.GraspingState.UNKNOWN
        ), EndEffectorState(
            # right hand
            position = list(msg.position[len(msg.position)//2:]),
            velocity = list(msg.velocity[len(msg.velocity)//2:]),
            effort = list(msg.effort[len(msg.effort)//2:]),
            state=EndEffectorState.GraspingState.UNKNOWN
        ))

    def _dexhand_touch_state_callback(self, msg)->None:
        # Update touch state for both hands
        self._dexhand_touch_state = (
            KuavoDexHandTouchState(
                data=tuple(KuavoDexHandTouchState.KuavoTouchState(
                    sensor.normal_force1, sensor.normal_force2, sensor.normal_force3,
                    sensor.tangential_force1, sensor.tangential_force2, sensor.tangential_force3,
                    sensor.tangential_direction1, sensor.tangential_direction2, sensor.tangential_direction3,
                    sensor.self_proximity1, sensor.self_proximity2, sensor.mutual_proximity,
                    sensor.status
                ) for sensor in msg.left_hand)
            ),  # Left hand touch state
            KuavoDexHandTouchState(
                data=tuple(KuavoDexHandTouchState.KuavoTouchState(
                    sensor.normal_force1, sensor.normal_force2, sensor.normal_force3,
                    sensor.tangential_force1, sensor.tangential_force2, sensor.tangential_force3,
                    sensor.tangential_direction1, sensor.tangential_direction2, sensor.tangential_direction3,
                    sensor.self_proximity1, sensor.self_proximity2, sensor.mutual_proximity,
                    sensor.status
                ) for sensor in msg.right_hand)
            )   # Right hand touch state
        )

    def _humanoid_mpc_gait_changed_callback(self, msg):
        """
        Callback function for gait change messages.
        Updates the current gait name when a gait change occurs.
        """
        SDKLogger.debug(f"[State] Received gait change message: {msg.gait_name} at time {msg.start_time}")
        self._gait_manager.add(msg.start_time, msg.gait_name)
    
    def _humanoid_mpc_observation_callback(self, msg) -> None:
        """
        Callback function for MPC observation messages.
        Updates the current MPC state and input data.
        """
        try:
            self._mpc_observation_data = msg
            # Only update gait info after initialization
            if hasattr(self, '_initialized'): 
                curr_time = self._mpc_observation_data.time
                current_gait = self._gait_manager.get_gait(curr_time)
                if self._prev_gait_name != current_gait:
                    SDKLogger.debug(f"[State] Gait changed to: {current_gait} at time {curr_time}")
                    # Update previous gait name and notify callback if registered
                    self._prev_gait_name = current_gait
                    if hasattr(self, '_gait_changed_callbacks') and self._gait_changed_callbacks is not None:
                        for callback in self._gait_changed_callbacks:
                            callback(curr_time, current_gait)
        except Exception as e:
            SDKLogger.error(f"Error processing MPC observation: {e}")

    def _srv_get_arm_ctrl_mode(self)-> KuavoArmCtrlMode:
        try:
            rospy.wait_for_service('/humanoid_get_arm_ctrl_mode', timeout=1.0)
            get_arm_ctrl_mode_srv = rospy.ServiceProxy('/humanoid_get_arm_ctrl_mode', changeArmCtrlMode)
            req = changeArmCtrlModeRequest()
            resp = get_arm_ctrl_mode_srv(req)
            return KuavoArmCtrlMode(resp.mode)
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call failed: {e}")
        except Exception as e:
            SDKLogger.error(f"[Error] get arm ctrl mode: {e}")
        return None
    
    def _srv_get_current_gait_name(self)->str:
        try:
            rospy.wait_for_service('/humanoid_get_current_gait_name', timeout=1.0)
            srv = rospy.ServiceProxy('/humanoid_get_current_gait_name', getCurrentGaitName)
            request = getCurrentGaitNameRequest()
            response = srv(request)
            if response.success:
                return response.gait_name
            else:
                return None
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
        return None

    def _srv_get_manipulation_mpc_ctrl_mode(self, )->KuavoManipulationMpcCtrlMode:
        try:
            service_name = '/mobile_manipulator_get_mpc_control_mode'
            rospy.wait_for_service(service_name, timeout=2.0)
            get_mode_srv = rospy.ServiceProxy(service_name, changeTorsoCtrlMode)
            
            req = changeTorsoCtrlModeRequest()
            
            resp = get_mode_srv(req)
            if not resp.result:
                SDKLogger.error(f"Failed to get manipulation mpc control mode: {resp.message}")
                return KuavoManipulationMpcCtrlMode.ERROR
            return KuavoManipulationMpcCtrlMode(resp.mode)
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e: # For timeout from wait_for_service
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to get manipulation mpc control mode: {e}")
        return KuavoManipulationMpcCtrlMode.ERROR

    def _srv_get_manipulation_mpc_frame(self, )->KuavoManipulationMpcFrame:
        try:
            service_name = '/get_mm_ctrl_frame'
            rospy.wait_for_service(service_name, timeout=2.0)
            get_frame_srv = rospy.ServiceProxy(service_name, setMmCtrlFrame)
            
            req = setMmCtrlFrameRequest()
            
            resp = get_frame_srv(req)
            if not resp.result:
                SDKLogger.error(f"Failed to get manipulation mpc frame: {resp.message}")
                return KuavoManipulationMpcFrame.ERROR
            return KuavoManipulationMpcFrame(resp.currentFrame)
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e: # For timeout from wait_for_service
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to get manipulation mpc frame: {e}")
        return KuavoManipulationMpcFrame.ERROR

    def _srv_get_manipulation_mpc_control_flow(self, )->KuavoManipulationMpcControlFlow:
        try:
            service_name = '/get_mm_wbc_arm_trajectory_control'
            rospy.wait_for_service(service_name, timeout=2.0)
            get_mode_srv = rospy.ServiceProxy(service_name, changeArmCtrlMode)
            
            req = changeArmCtrlModeRequest()
            
            resp = get_mode_srv(req)
            if not resp.result:
                SDKLogger.error(f"Failed to get manipulation mpc wbc arm trajectory control mode: {resp.message}")
                return KuavoManipulationMpcControlFlow.Error
            return KuavoManipulationMpcControlFlow(resp.mode)
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e: # For timeout from wait_for_service
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to get manipulation mpc wbc arm trajectory control mode: {e}")
        return KuavoManipulationMpcControlFlow.Error

    def _srv_get_manipulation_mpc_ctrl_mode(self, )->KuavoManipulationMpcCtrlMode:
        try:
            service_name = '/mobile_manipulator_get_mpc_control_mode'
            rospy.wait_for_service(service_name, timeout=2.0)
            get_mode_srv = rospy.ServiceProxy(service_name, changeTorsoCtrlMode)
            
            req = changeTorsoCtrlModeRequest()
            
            resp = get_mode_srv(req)
            if not resp.result:
                SDKLogger.error(f"Failed to get manipulation mpc control mode: {resp.message}")
                return KuavoManipulationMpcCtrlMode.ERROR
            return KuavoManipulationMpcCtrlMode(resp.mode)
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e: # For timeout from wait_for_service
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to get manipulation mpc control mode: {e}")
        return KuavoManipulationMpcCtrlMode.ERROR

    def _srv_get_manipulation_mpc_frame(self, )->KuavoManipulationMpcFrame:
        try:
            service_name = '/get_mm_ctrl_frame'
            rospy.wait_for_service(service_name, timeout=2.0)
            get_frame_srv = rospy.ServiceProxy(service_name, setMmCtrlFrame)
            
            req = setMmCtrlFrameRequest()
            
            resp = get_frame_srv(req)
            if not resp.result:
                SDKLogger.error(f"Failed to get manipulation mpc frame: {resp.message}")
                return KuavoManipulationMpcFrame.ERROR
            return KuavoManipulationMpcFrame(resp.currentFrame)
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e: # For timeout from wait_for_service
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to get manipulation mpc frame: {e}")
        return KuavoManipulationMpcFrame.ERROR

    def _srv_get_manipulation_mpc_control_flow(self, )->KuavoManipulationMpcControlFlow:
        try:
            service_name = '/get_mm_wbc_arm_trajectory_control'
            rospy.wait_for_service(service_name, timeout=2.0)
            get_mode_srv = rospy.ServiceProxy(service_name, changeArmCtrlMode)
            
            req = changeArmCtrlModeRequest()
            
            resp = get_mode_srv(req)
            if not resp.result:
                SDKLogger.error(f"Failed to get manipulation mpc wbc arm trajectory control mode: {resp.message}")
                return KuavoManipulationMpcControlFlow.Error
            return KuavoManipulationMpcControlFlow(resp.mode)
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e: # For timeout from wait_for_service
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to get manipulation mpc wbc arm trajectory control mode: {e}")
        return KuavoManipulationMpcControlFlow.Error

    def _srv_get_manipulation_mpc_ctrl_mode(self, )->KuavoManipulationMpcCtrlMode:
        try:
            service_name = '/mobile_manipulator_get_mpc_control_mode'
            rospy.wait_for_service(service_name, timeout=2.0)
            get_mode_srv = rospy.ServiceProxy(service_name, changeTorsoCtrlMode)
            
            req = changeTorsoCtrlModeRequest()
            
            resp = get_mode_srv(req)
            if not resp.result:
                SDKLogger.error(f"Failed to get manipulation mpc control mode: {resp.message}")
                return KuavoManipulationMpcCtrlMode.ERROR
            return KuavoManipulationMpcCtrlMode(resp.mode)
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e: # For timeout from wait_for_service
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to get manipulation mpc control mode: {e}")
        return KuavoManipulationMpcCtrlMode.ERROR

    def _srv_get_manipulation_mpc_frame(self, )->KuavoManipulationMpcFrame:
        try:
            service_name = '/get_mm_ctrl_frame'
            rospy.wait_for_service(service_name, timeout=2.0)
            get_frame_srv = rospy.ServiceProxy(service_name, setMmCtrlFrame)
            
            req = setMmCtrlFrameRequest()
            
            resp = get_frame_srv(req)
            if not resp.result:
                SDKLogger.error(f"Failed to get manipulation mpc frame: {resp.message}")
                return KuavoManipulationMpcFrame.ERROR
            return KuavoManipulationMpcFrame(resp.currentFrame)
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e: # For timeout from wait_for_service
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to get manipulation mpc frame: {e}")
        return KuavoManipulationMpcFrame.ERROR

    def _srv_get_manipulation_mpc_control_flow(self, )->KuavoManipulationMpcControlFlow:
        try:
            service_name = '/get_mm_wbc_arm_trajectory_control'
            rospy.wait_for_service(service_name, timeout=2.0)
            get_mode_srv = rospy.ServiceProxy(service_name, changeArmCtrlMode)
            
            req = changeArmCtrlModeRequest()
            
            resp = get_mode_srv(req)
            if not resp.result:
                SDKLogger.error(f"Failed to get manipulation mpc wbc arm trajectory control mode: {resp.message}")
                return KuavoManipulationMpcControlFlow.Error
            return KuavoManipulationMpcControlFlow(resp.mode)
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e: # For timeout from wait_for_service
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to get manipulation mpc wbc arm trajectory control mode: {e}")
        return KuavoManipulationMpcControlFlow.Error

    def _srv_get_pitch_limit_status(self, )->Tuple[bool, str]:
        try:
            service_name = '/humanoid/mpc/pitch_limit_status'
            rospy.wait_for_service(service_name, timeout=2.0)
            get_pitch_limit_status_srv = rospy.ServiceProxy(service_name, SetBool)
            
            req = SetBoolRequest()
            
            resp = get_pitch_limit_status_srv(req)
            if not resp.success:
                SDKLogger.error(f"Failed to get pitch limit status: {resp.message}")
                return False, 'unknown'
            return resp.success, resp.message
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e: # For timeout from wait_for_service
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to get pitch limit status: {e}")
        return False, 'unknown'
    
if __name__ == "__main__":
    state = KuavoRobotStateCore()
    print(state.manipulation_mpc_frame)
    print(state.manipulation_mpc_control_flow)
    print(state.manipulation_mpc_ctrl_mode)
    print(state.arm_control_mode)