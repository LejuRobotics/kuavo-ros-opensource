import os
import roslibpy
import numpy as np
from typing import Tuple
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK
from kuavo_humanoid_sdk.interfaces.data_types import (KuavoArmCtrlMode, KuavoIKParams, KuavoPose, 
                                                      KuavoManipulationMpcControlFlow, KuavoManipulationMpcCtrlMode
                                                      ,KuavoManipulationMpcFrame)
from kuavo_humanoid_sdk.kuavo.core.ros.sat_utils import RotatingRectangle
from kuavo_humanoid_sdk.kuavo.core.ros.param import EndEffectorType


class ControlEndEffectorWebsocket:
    def __init__(self, eef_type: str = EndEffectorType.QIANGNAO):
        self._eef_type = eef_type
        self._pubs = []
        websocket = WebSocketKuavoSDK()
        if self._eef_type == EndEffectorType.QIANGNAO:
            self._pub_ctrl_robot_hand = roslibpy.Topic(websocket.client, '/control_robot_hand_position', 'kuavo_msgs/robotHandPosition')                
            # publisher, name, require
            self._pubs.append((self._pub_ctrl_robot_hand, False))
        elif self._eef_type == EndEffectorType.QIANGNAO_TOUCH:
            self._pub_ctrl_robot_hand = roslibpy.Topic(websocket.client, '/control_robot_hand_position', 'kuavo_msgs/robotHandPosition')                
            self._pub_dexhand_command = roslibpy.Topic(websocket.client, '/dexhand/command', 'kuavo_msgs/dexhandCommand')
            self._pub_dexhand_right_command = roslibpy.Topic(websocket.client, '/dexhand/right/command', 'kuavo_msgs/dexhandCommand')
            self._pub_dexhand_left_command = roslibpy.Topic(websocket.client, '/dexhand/left/command', 'kuavo_msgs/dexhandCommand')
            # publisher, name, require
            self._pubs.append((self._pub_dexhand_command, False))
            self._pubs.append((self._pub_dexhand_right_command, False))
            self._pubs.append((self._pub_dexhand_left_command, False))

    def connect(self, timeout:float=1.0)-> bool:
        return True

    def pub_control_robot_dexhand(self, left_position:list, right_position:list)->bool:
        if not self._eef_type.startswith(EndEffectorType.QIANGNAO): # qiangnao, qiangnao_touch
            SDKLogger.warning(f"{self._eef_type} not support control dexhand")
            return False
        try:
            hand_pose_msg = {
                "left_hand_position": left_position,
                "right_hand_position": right_position
            }
            self._pub_ctrl_robot_hand.publish(roslibpy.Message(hand_pose_msg))
            SDKLogger.debug(f"publish robot dexhand: {left_position}, {right_position}")
            return True
        except Exception as e:
            SDKLogger.error(f"publish robot dexhand: {e}")
            return False
    
    def pub_dexhand_command(self, data:list, ctrl_mode, hand_side)->bool:
        """
            ctrl_mode: 0 --> POSITION, 1 --> VELOCITY
            hand_side: 0 --> left, 1 --> right, 2-->dual
        """
        if not self._eef_type == EndEffectorType.QIANGNAO_TOUCH:
            SDKLogger.warning(f"{self._eef_type} not support pub_left_dexhand_command")
            return False
        try:
            if hand_side != 2 and len(data) != 6:
                SDKLogger.warning("Data length should be 6")
                return False
            if hand_side == 2 and len(data) != 12:
                SDKLogger.warning("Data length should be 12")
                return False
            if ctrl_mode not in [dexhandCommand.POSITION_CONTROL, dexhandCommand.VELOCITY_CONTROL]:
                SDKLogger.error(f"Invalid mode for pub_left_dexhand_command: {ctrl_mode}")
                return False
            
            msg = {
                "data": [int(d) for d in data],  # Convert data to integers
                "control_mode": ctrl_mode
            }
            if hand_side == 0:
                self._pub_dexhand_left_command.publish(roslibpy.Message(msg))
            elif hand_side == 1:
                self._pub_dexhand_right_command.publish(roslibpy.Message(msg))
            else:
                self._pub_dexhand_command.publish(roslibpy.Message(msg))
            return True
        except Exception as e:
            SDKLogger.error(f"Failed to publish left dexhand command: {e}")
            return False
        
    def srv_execute_gesture(self, gestures:list)->bool:
        if not self._eef_type.startswith(EndEffectorType.QIANGNAO): # qiangnao, qiangnao_touch
            SDKLogger.warning(f"{self._eef_type} not support control dexhand")
            return False
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, 'gesture/execute', 'kuavo_msgs/gestureExecute')
            request = {
                "gestures": [
                    {
                        "gesture_name": gs["gesture_name"],
                        "hand_side": gs["hand_side"]
                    } for gs in gestures
                ]
            }
            response = service.call(request)
            if not response.get('success', False):
                SDKLogger.error(f"Failed to execute gesture '{gestures}': {response.get('message', '')}")
            return response.get('success', False)
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
            return False

    def srv_get_gesture_names(self)->list:
        if not self._eef_type.startswith(EndEffectorType.QIANGNAO): # qiangnao, qiangnao_touch
            SDKLogger.warning(f"{self._eef_type} not support control dexhand")
            return []
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, 'gesture/list', 'kuavo_msgs/gestureList')
            request = {}
            response = service.call(request)
            gestures = []
            for gesture_info in response.get('gesture_infos', []):
                gestures.append(gesture_info['gesture_name'])
                for alias in gesture_info.get('alias', []):
                    gestures.append(alias)
            return list(set(gestures))
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
            return []

    def srv_control_leju_claw(self, postions:list, velocities:list, torques:list) ->bool:
        if self._eef_type != 'lejuclaw':
            SDKLogger.warning(f"{self._eef_type} not support control lejuclaw.")
            return False
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, 'control_robot_leju_claw', 'kuavo_msgs/controlLejuClaw')
            request = {
                "data": {
                    "position": postions,
                    "velocity": velocities,
                    "effort": torques
                }
            }
            response = service.call(request)
            if not response.get('result', False):
                SDKLogger.error(f"Failed to control leju claw: {response.get('message', '')}")
            return response.get('result', False)
        except Exception as e:
            SDKLogger.error(f"Service `control_robot_leju_claw` call failed: {e}")
            return False

class ControlRobotArmWebsocket:
    def __init__(self):
        websocket = WebSocketKuavoSDK()
        self._pub_ctrl_arm_traj_arm_collision =  roslibpy.Topic(websocket.client,'/arm_collision/kuavo_arm_traj', 'sensor_msgs/JointState')
        self._pub_ctrl_arm_target_poses_arm_collision = roslibpy.Topic(websocket.client, '/arm_collision/kuavo_arm_target_poses', 'kuavo_msgs/armTargetPoses')
        self._pub_ctrl_hand_pose_cmd_arm_collision = roslibpy.Topic(websocket.client, '/arm_collision/mm/two_arm_hand_pose_cmd', 'kuavo_msgs/twoArmHandPoseCmd')
        self._sub_arm_collision_info = roslibpy.Topic(websocket.client, '/arm_collision/info', 'kuavo_msgs/armCollisionCheckInfo')
        self._sub_arm_collision_info.subscribe(self.callback_arm_collision_info)
        self._is_collision = False
        self.arm_collision_enable = False
        #正常轨迹发布
        self._pub_ctrl_arm_traj = roslibpy.Topic(websocket.client, '/kuavo_arm_traj', 'sensor_msgs/JointState')
        self._pub_ctrl_arm_target_poses = roslibpy.Topic(websocket.client, '/kuavo_arm_target_poses', 'kuavo_msgs/armTargetPoses')
        self._pub_ctrl_hand_pose_cmd = roslibpy.Topic(websocket.client, '/mm/two_arm_hand_pose_cmd', 'kuavo_msgs/twoArmHandPoseCmd')
        self._pub_ctrl_arm_traj.advertise()
        self._pub_ctrl_arm_target_poses.advertise()
        self._pub_ctrl_hand_pose_cmd.advertise()
        self._pub_ctrl_hand_pose_cmd_arm_collision.advertise()

    def connect(self, timeout:float=1.0)-> bool:
        return True

    def pub_control_robot_arm_traj(self, joint_q: list)->bool:
        try:
            msg = {
                "name": ["arm_joint_" + str(i) for i in range(0, 14)],
                "position": [float(180.0 / np.pi * q) for q in joint_q]  # convert to degree
            }
            if self.arm_collision_enable:
                self._pub_ctrl_arm_traj_arm_collision.publish(roslibpy.Message(msg))
            else:
                self._pub_ctrl_arm_traj.publish(roslibpy.Message(msg))
            return True
        except Exception as e:
            SDKLogger.error(f"publish robot arm traj: {e}")
        return False

    def is_arm_collision_mode(self)->bool:
        return self.arm_collision_enable

    def set_arm_collision_mode(self, enable: bool):
        """
            Set arm collision mode
        """
        self.arm_collision_enable = enable
        websocket = WebSocketKuavoSDK()
        srv_set_arm_collision_mode_srv = roslibpy.Service(websocket.client, '/arm_collision/set_arm_moving_enable', 'std_srvs/SetBool')
        req = {
            "data": enable
        }
        resp = srv_set_arm_collision_mode_srv.call(req)
        if not resp.get('success', False):
            SDKLogger.error(f"Failed to set arm collision mode: {resp.get('message', '')}")

    def is_arm_collision(self)->bool:
        return self._is_collision

    def callback_arm_collision_info(self, msg):
        self._is_collision = True
        SDKLogger.info(f"Arm collision detected")

    def release_arm_collision_mode(self):
        self._is_collision = False
    
    def wait_arm_collision_complete(self):
        if self._is_collision:
            websocket = WebSocketKuavoSDK()
            srv_wait_arm_collision_complete_srv = roslibpy.Service(websocket.client, '/arm_collision/wait_complete', 'std_srvs/SetBool')
            req = {
                "data": True
            }
            resp = srv_wait_arm_collision_complete_srv.call(req)
            if not resp.get('success', False):
                SDKLogger.error(f"Failed to wait arm collision complete: {resp.get('message', '')}")

    def pub_arm_target_poses(self, times:list, joint_q:list)->bool:
        try:
            msg_values = []
            for i in range(len(joint_q)):
                degs = [float(q) for q in joint_q[i]]
                msg_values.extend(degs)
            msg = {
                "times": [float(q) for q in times],
                "values": msg_values
            }
            if self.arm_collision_enable:
                self._pub_ctrl_arm_target_poses_arm_collision.publish(roslibpy.Message(msg))
            else:
                self._pub_ctrl_arm_target_poses.publish(roslibpy.Message(msg))
            return True
        except Exception as e:
            SDKLogger.error(f"publish arm target poses: {e}")
            
        return False

    def pub_end_effector_pose_cmd(self, left_pose: KuavoPose, right_pose: KuavoPose, frame: KuavoManipulationMpcFrame)->bool:
        try:
            msg = {
                "hand_poses": {
                    "left_pose": {
                        "pos_xyz": left_pose.position,
                        "quat_xyzw": left_pose.orientation
                    },
                    "right_pose": {
                        "pos_xyz": right_pose.position,
                        "quat_xyzw": right_pose.orientation
                    }
                },
                "frame": frame.value
            }
            if frame.value not in [0, 1, 2, 3, 4]:
                SDKLogger.error(f"Invalid frame: {frame}")
                return False
            if self.arm_collision_enable:
                self._pub_ctrl_hand_pose_cmd_arm_collision.publish(roslibpy.Message(msg))
            else:
                self._pub_ctrl_hand_pose_cmd.publish(roslibpy.Message(msg))
            return True
        except Exception as e:
            SDKLogger.error(f"publish arm target poses: {e}")
        return False

    def srv_change_manipulation_mpc_frame(self, frame: KuavoManipulationMpcFrame)->bool:
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, '/set_mm_ctrl_frame', 'kuavo_msgs/setMmCtrlFrame')
            request = {
                "frame": frame.value
            }
            response = service.call(request)
            if not response.get('result', False):
                SDKLogger.error(f"Failed to change manipulation mpc frame to {frame}: {response.get('message', '')}")
            return response.get('result', False)
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
        return False
    
    def srv_change_manipulation_mpc_ctrl_mode(self, ctrl_mode: KuavoManipulationMpcCtrlMode)->bool:
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, '/mobile_manipulator_mpc_control', 'kuavo_msgs/changeTorsoCtrlMode')
            request = {
                "control_mode": ctrl_mode.value
            }
            response = service.call(request)
            if not response.get('result', False):
                SDKLogger.error(f"Failed to change manipulation mpc control mode to {ctrl_mode}: {response.get('message', '')}")
            return response.get('result', False)
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
        return False

    def srv_change_manipulation_mpc_control_flow(self, ctrl_flow: KuavoManipulationMpcControlFlow)-> bool:
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, '/enable_mm_wbc_arm_trajectory_control', 'kuavo_msgs/changeArmCtrlMode')
            request = {
                "control_mode": ctrl_flow.value
            }
            response = service.call(request)
            if not response.get('result', False):
                SDKLogger.error(f"Failed to change manipulation mpc wbc arm trajectory control to {ctrl_flow}: {response.get('message', '')}")
            return response.get('result', False)
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
        return False

    def srv_get_manipulation_mpc_ctrl_mode(self, )->KuavoManipulationMpcCtrlMode:
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, '/mobile_manipulator_get_mpc_control_mode', 'kuavo_msgs/changeTorsoCtrlMode')
            request = {}
            response = service.call(request)
            if not response.get('result', False):
                SDKLogger.error(f"Failed to get manipulation mpc control mode: {response.get('message', '')}")
                return KuavoManipulationMpcCtrlMode.ERROR
            return KuavoManipulationMpcCtrlMode(response.get('mode', 0))
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
        return KuavoManipulationMpcCtrlMode.ERROR

    def srv_get_manipulation_mpc_frame(self, )->KuavoManipulationMpcFrame:
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, '/get_mm_ctrl_frame', 'kuavo_msgs/setMmCtrlFrame')
            request = {}
            response = service.call(request)
            if not response.get('result', False):
                SDKLogger.error(f"Failed to get manipulation mpc frame: {response.get('message', '')}")
                return KuavoManipulationMpcFrame.ERROR
            return KuavoManipulationMpcFrame(response.get('currentFrame', 0))
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
        return KuavoManipulationMpcFrame.ERROR

    def srv_get_manipulation_mpc_control_flow(self, )->KuavoManipulationMpcControlFlow:
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, '/get_mm_wbc_arm_trajectory_control', 'kuavo_msgs/changeArmCtrlMode')
            request = {}
            response = service.call(request)
            if not response.get('result', False):
                SDKLogger.error(f"Failed to get manipulation mpc wbc arm trajectory control mode: {response.get('message', '')}")
                return KuavoManipulationMpcControlFlow.Error
            return KuavoManipulationMpcControlFlow(response.get('mode', 0))
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
        return KuavoManipulationMpcControlFlow.Error

    def srv_change_arm_ctrl_mode(self, mode: KuavoArmCtrlMode)->bool:
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, '/arm_traj_change_mode', 'kuavo_msgs/changeArmCtrlMode')
            request = {
                "control_mode": mode.value
            }
            response = service.call(request)
            return response.get('result', False)
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
        return False
    
    def srv_get_arm_ctrl_mode(self)-> KuavoArmCtrlMode:
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, '/humanoid_get_arm_ctrl_mode', 'kuavo_msgs/changeArmCtrlMode')
            request = {}
            response = service.call(request)
            return KuavoArmCtrlMode(response.get('control_mode', 0))
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
        return None

""" Control Robot Waist """

class ControlRobotWaistWebsocket:
    def __init__(self):
        # 创建 WebSocket 客户端
        websocket = WebSocketKuavoSDK()

        # Publisher：发布到 /robot_waist_motion_data，消息类型 kuavo_msgs/robotWaistControl
        self._pub_ctrl_robot_waist = roslibpy.Topic(
            websocket.client,
            '/robot_waist_motion_data',
            'kuavo_msgs/robotWaistControl'
        )
        self._pub_ctrl_robot_waist.advertise()

    def connect(self, timeout: float = 1.0) -> bool:
        return True

    def pub_control_robot_waist(self, waistPos: list) -> bool:
        """
        发布腰部位置控制命令 (WebSocket)
        参数:
            waistPos: 长度为 1 的数组，例如 [30.0]
        """
        # 检查输入维度
        if len(waistPos) != 1:
            SDKLogger.error("Waist data must be 1-dimensional")
            return False

        try:
            # 获取当前时间戳（秒和纳秒）
            import time
            now = time.time()
            secs = int(now)
            nsecs = int((now - secs) * 1e9)
            
            # roslibpy 方式构建消息
            msg = {
                'header': {
                    'stamp': {
                        'secs': secs,
                        'nsecs': nsecs
                    }
                },
                'data': {
                    'data': [float(waistPos[0])]
                }
            }

            # 发布
            self._pub_ctrl_robot_waist.publish(roslibpy.Message(msg))
            return True

        except Exception as e:
            SDKLogger.error(f"Publish waist pos failed: {e}")
            return False


""" Control Robot Head """
class ControlRobotHeadWebsocket:
    def __init__(self):
        websocket = WebSocketKuavoSDK()
        self._pub_ctrl_robot_head = roslibpy.Topic(websocket.client, '/robot_head_motion_data', 'kuavo_msgs/robotHeadMotionData')
        self._pub_ctrl_robot_head.advertise()

    def connect(self, timeout:float=1.0)->bool:
        return True

    def pub_control_robot_head(self, yaw:float, pitch:float)->bool:
        try:
            msg = {
                "joint_data": [float(yaw), float(pitch)]
            }
            self._pub_ctrl_robot_head.publish(roslibpy.Message(msg))
            return True
        except Exception as e:
            SDKLogger.error(f"[Error] publish robot head: {e}")
            return False

    def srv_enable_head_tracking(self, target_id: int)->bool:
        """Enable the head tracking for a specific tag ID.
        
        Args:
            target_id: The ID of the tag to track
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            websocket = WebSocketKuavoSDK()
            # First set the target tag ID
            set_tag_service = roslibpy.Service(websocket.client, '/set_target_tag_id', 'kuavo_msgs/setTagId')
            set_tag_request = {
                "tag_id": target_id
            }
            set_tag_response = set_tag_service.call(set_tag_request)
            if not set_tag_response.get('success', False):
                SDKLogger.error(f"Failed to set target tag ID: {set_tag_response.get('message', '')}")
                return False
                
            SDKLogger.info(f"Successfully set target tag ID to {target_id}: {set_tag_response.get('message', '')}")
            
            # Then start continuous tracking
            track_service = roslibpy.Service(websocket.client, '/continuous_track', 'std_srvs/SetBool')
            track_request = {
                "data": True
            }
            track_response = track_service.call(track_request)
            if not track_response.get('success', False):
                SDKLogger.error(f"Failed to start continuous tracking: {track_response.get('message', '')}")
                return False
                
            SDKLogger.info(f"Successfully started continuous tracking: {track_response.get('message', '')}")
            return True
            
        except Exception as e:
            SDKLogger.error(f"Failed to enable head tracking: {e}")
            return False
        
    def srv_disable_head_tracking(self)->bool:
        """Disable the head tracking.
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, '/continuous_track', 'std_srvs/SetBool')
            request = {
                "data": False
            }
            response = service.call(request)
            if not response.get('success', False):
                SDKLogger.error(f"Failed to stop continuous tracking: {response.get('message', '')}")
                return False
                
            SDKLogger.info(f"Successfully stopped continuous tracking: {response.get('message', '')}")
            return True
            
        except Exception as e:
            SDKLogger.error(f"Failed to disable head tracking: {e}")
            return False

""" Control Robot Motion """

# JoyButton constants
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_BACK = 6
BUTTON_START = 7

# JoyAxis constants
AXIS_LEFT_STICK_Y = 0
AXIS_LEFT_STICK_X = 1
AXIS_LEFT_LT = 2  # 1 -> (-1)
AXIS_RIGHT_STICK_YAW = 3
AXIS_RIGHT_STICK_Z = 4
AXIS_RIGHT_RT = 5  # 1 -> (-1)
AXIS_LEFT_RIGHT_TRIGGER = 6
AXIS_FORWARD_BACK_TRIGGER = 7


class ControlRobotMotionWebsocket:
    def __init__(self):
        websocket = WebSocketKuavoSDK()
        self._pub_cmd_vel = roslibpy.Topic(websocket.client, '/cmd_vel', 'geometry_msgs/Twist')
        self._pub_cmd_pose = roslibpy.Topic(websocket.client, '/cmd_pose', 'geometry_msgs/Twist')
        self._pub_cmd_pose_world = roslibpy.Topic(websocket.client, '/cmd_pose_world', 'geometry_msgs/Twist')
        self._pub_joy = roslibpy.Topic(websocket.client, '/joy', 'sensor_msgs/Joy')
        self._pub_switch_gait = roslibpy.Topic(websocket.client, '/humanoid_switch_gait_by_name', 'kuavo_msgs/switchGaitByName')
        self._pub_step_ctrl = roslibpy.Topic(websocket.client, '/humanoid_mpc_foot_pose_target_trajectories', 'kuavo_msgs/footPoseTargetTrajectories')
        self._pub_mpc_target_pose = roslibpy.Topic(websocket.client, '/humanoid_mpc_target_pose', 'kuavo_msgs/mpc_target_trajectories')
        self._pub_cmd_vel.advertise()
        self._pub_cmd_pose.advertise()
        self._pub_cmd_pose_world.advertise()
        self._pub_joy.advertise()
        self._pub_switch_gait.advertise()
        self._pub_step_ctrl.advertise()
        self._pub_mpc_target_pose.advertise()

    def connect(self, timeout:float=2.0)-> bool:
        return True

    def pub_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float)->bool:
        try:
            msg = {
                "linear": {"x": float(linear_x), "y": float(linear_y), "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": float(angular_z)}
            }
            self._pub_cmd_vel.publish(roslibpy.Message(msg))
            return True
        except Exception as e:
            SDKLogger.error(f"[Error] publish cmd vel: {e}")
            return False

    def pub_cmd_pose(self, msg)->bool:
        try:
            self._pub_cmd_pose.publish(roslibpy.Message(msg))
            return True
        except Exception as e:
            SDKLogger.error(f"[Error] publish cmd pose: {e}")
            return False

    def pub_cmd_pose_world(self, msg)->bool:
        try:
            self._pub_cmd_pose_world.publish(roslibpy.Message(msg))
            return True
        except Exception as e:
            SDKLogger.error(f"[Error] publish cmd pose world: {e}")
            return False

    def _pub_joy_command(self, button_index: int, command_name: str) -> bool:
        try:
            msg = {
                "axes": [0.0] * 8,
                "buttons": [0] * 16
            }
            msg["buttons"][button_index] = 1
            self._pub_joy.publish(roslibpy.Message(msg))
            SDKLogger.debug(f"Published {command_name} command")
            return True
        except Exception as e:
            SDKLogger.error(f"[Error] publish {command_name}: {e}")
            return False

    def _pub_switch_gait_by_name(self, gait_name: str) -> bool:
        try:
            msg = {
                "gait_name": gait_name
            }
            self._pub_switch_gait.publish(roslibpy.Message(msg))
            return True
        except Exception as e:
            SDKLogger.error(f"[Error] publish switch gait {gait_name}: {e}")
            return False

    def pub_walk_command(self) -> bool:
        return self._pub_switch_gait_by_name("walk")

    def pub_stance_command(self) -> bool:
        try:
            self.pub_cmd_vel(linear_x=0.0, linear_y=0.0, angular_z=0.0)
            return self._pub_switch_gait_by_name("stance")
        except Exception as e:
            SDKLogger.error(f"[Error] publish stance: {e}")
            return False

    def pub_trot_command(self) -> bool:
        return self._pub_switch_gait_by_name("walk")
    
    def pub_mpc_target_pose(self, target_pose: list, initial_pose: list = None, time_horizon: float = 2.0)->bool:
        """
        发布6DOF躯干姿态目标轨迹到MPC
        
        参数:
            target_pose: 6DOF目标姿态 [x, y, z, yaw, pitch, roll]
            initial_pose: 6DOF初始姿态 [x, y, z, yaw, pitch, roll]，如果为None则从当前observation获取
            time_horizon: 目标时间（相对于当前MPC时间），单位秒
        返回:
            bool: 发布成功返回True，否则返回False
        
        注意:
            - 如果initial_pose为None，必须能够从MPC observation中获取当前状态，否则返回False
            - 必须能够获取MPC observation中的时间，否则返回False
            - 时间使用MPC observation中的时间，而不是系统时间
        """
        try:
            if len(target_pose) != 6:
                SDKLogger.error(f"[Error] target_pose must have 6 elements, got {len(target_pose)}")
                return False
            
            # 获取MPC observation数据（用于获取当前时间和状态）
            from kuavo_humanoid_sdk.kuavo.core.ros.state import KuavoRobotStateCoreWebsocket
            state_core = KuavoRobotStateCoreWebsocket()
            current_time = None
            current_state = None
            
            # 检查是否能够获取MPC observation数据
            if not hasattr(state_core, '_mpc_observation_data') or state_core._mpc_observation_data is None:
                SDKLogger.error("[Error] Cannot get MPC observation data. Please ensure the MPC controller is running and publishing observation data.")
                return False
            
            obs = state_core._mpc_observation_data
            
            # 获取MPC时间（websocket版本中obs是字典）
            if 'time' not in obs:
                SDKLogger.error("[Error] MPC observation data does not have 'time' field.")
                return False
            current_time = obs['time']
            
            # 如果需要从observation获取初始状态
            if initial_pose is None:
                # 检查observation中是否有state数据
                if 'state' not in obs or 'value' not in obs['state']:
                    SDKLogger.error("[Error] MPC observation data does not have 'state.value' field.")
                    return False
                
                # MPC状态向量索引说明：
                # 0-5: 质心动量 (vcom_x, vcom_y, vcom_z, L_x/m, L_y/m, L_z/m)
                # 6-11: 躯干姿态 (p_base_x, p_base_y, p_base_z, theta_base_z/yaw, theta_base_y/pitch, theta_base_x/roll)
                if len(obs['state']['value']) < 12:
                    SDKLogger.error(f"[Error] MPC observation state value length ({len(obs['state']['value'])}) is less than 12. Cannot extract current pose.")
                    return False
                
                # 从observation的state中提取索引6-11的元素作为当前姿态 [x, y, z, yaw, pitch, roll]
                current_state = [
                    obs['state']['value'][6],   # p_base_x
                    obs['state']['value'][7],   # p_base_y
                    obs['state']['value'][8],   # p_base_z
                    obs['state']['value'][9],   # theta_base_z (yaw)
                    obs['state']['value'][10],  # theta_base_y (pitch)
                    obs['state']['value'][11]   # theta_base_x (roll)
                ]
                initial_pose = current_state
            elif len(initial_pose) != 6:
                SDKLogger.error(f"[Error] initial_pose must have 6 elements, got {len(initial_pose)}")
                return False
            
            # 验证时间是否有效
            if current_time is None or current_time <= 0:
                SDKLogger.error(f"[Error] Invalid MPC time: {current_time}. Cannot publish trajectory.")
                return False
            
            # 创建mpc_target_trajectories消息（websocket版本使用字典格式）
            msg = {
                "timeTrajectory": [current_time, current_time + time_horizon],
                "stateTrajectory": [
                    {"value": [float(x) for x in initial_pose]},
                    {"value": [float(x) for x in target_pose]}
                ],
                "inputTrajectory": [
                    {"value": []},
                    {"value": []}
                ]
            }
            
            self._pub_mpc_target_pose.publish(roslibpy.Message(msg))
            return True
        except Exception as e:
            SDKLogger.error(f"[Error] publish mpc target pose: {e}")
            return False
    
    def pub_step_ctrl(self, msg)->bool:
        try:
            websocket_msg = {
                "timeTrajectory": msg["timeTrajectory"],
                "footIndexTrajectory": msg["footIndexTrajectory"],
                "footPoseTrajectory": [
                    {
                        "footPose": list(fp["footPose"]),
                        "torsoPose": list(fp["torsoPose"])
                    } for fp in msg["footPoseTrajectory"]
                ]
            }
            SDKLogger.debug(f"websocket_msg {websocket_msg}")
            self._pub_step_ctrl.publish(roslibpy.Message(websocket_msg))
            SDKLogger.debug(f"after publish")
            return True
        except Exception as e:
            SDKLogger.error(f"[Error] publish step ctrl: {e}")
            return False


class KuavoRobotArmIKFKWebsocket:
    def __init__(self):
        pass

    def arm_ik(self, 
               left_pose: KuavoPose, 
               right_pose: KuavoPose, 
               left_elbow_pos_xyz: list = [0.0, 0.0, 0.0],
               right_elbow_pos_xyz: list = [0.0, 0.0, 0.0],
               arm_q0: list = None,
               params: KuavoIKParams=None) -> list:
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, '/ik/two_arm_hand_pose_cmd_srv', 'kuavo_msgs/twoArmHandPoseCmdSrv')
            
            request = {
                "twoArmHandPoseCmdRequest": {
                    "hand_poses": {
                        "header": {
                            "seq": 0,
                            "stamp": {
                                "secs": 0,
                                "nsecs": 0
                            },
                            "frame_id": ""
                        },
                        "left_pose": {
                            "pos_xyz": left_pose.position,
                            "quat_xyzw": left_pose.orientation,
                            "elbow_pos_xyz": left_elbow_pos_xyz,
                        },
                        "right_pose": {
                            "pos_xyz": right_pose.position,
                            "quat_xyzw": right_pose.orientation,
                            "elbow_pos_xyz": right_elbow_pos_xyz,
                        }
                    },
                    "use_custom_ik_param": params is not None,
                    "joint_angles_as_q0": arm_q0 is not None,
                    "ik_param": {
                        "major_optimality_tol": params.major_optimality_tol if params else 0.0,
                        "major_feasibility_tol": params.major_feasibility_tol if params else 0.0,
                        "minor_feasibility_tol": params.minor_feasibility_tol if params else 0.0,
                        "major_iterations_limit": params.major_iterations_limit if params else 0,
                        "oritation_constraint_tol": params.oritation_constraint_tol if params else 0.0,
                        "pos_constraint_tol": params.pos_constraint_tol if params else 0.0,
                        "pos_cost_weight": params.pos_cost_weight if params else 0.0
                    }
                }
            }
            
            response = service.call(request)
            if response.get('success', False):
                return response['hand_poses']['left_pose']['joint_angles'] + response['hand_poses']['right_pose']['joint_angles']
            return None
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
            return None

    def arm_fk(self, q: list) -> Tuple[KuavoPose, KuavoPose]:
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, '/ik/fk_srv', 'kuavo_msgs/fkSrv')
            request = {"q": q}
            response = service.call(request)
            
            if response.get('success', False):
                left_pose = KuavoPose(
                    position=response['hand_poses']['left_pose']['pos_xyz'],
                    orientation=response['hand_poses']['left_pose']['quat_xyzw']
                )
                right_pose = KuavoPose(
                    position=response['hand_poses']['right_pose']['pos_xyz'],
                    orientation=response['hand_poses']['right_pose']['quat_xyzw']
                )
                return left_pose, right_pose
            return None
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
            return None

"""
    Kuavo Robot Control 
"""
class KuavoRobotControlWebsocket:
    _instance = None
    
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        if not hasattr(self, '_initialized'):
            try:
                self._initialized = True
                self.kuavo_eef_control = None
                self.kuavo_head_control = ControlRobotHeadWebsocket()
                self.kuavo_arm_control = ControlRobotArmWebsocket()
                self.kuavo_motion_control = ControlRobotMotionWebsocket()
                self.kuavo_arm_ik_fk = KuavoRobotArmIKFKWebsocket()
                self.kuavo_waist_control = ControlRobotWaistWebsocket()
            except Exception as e:
                SDKLogger.error(f"Failed to initialize KuavoRobotControlWebsocket: {e}")
                raise

    def initialize(self, eef_type:str=None, debug:bool=False, timeout:float=1.0)-> Tuple[bool, str]:
        try:
            # init eef control
            if eef_type is None:
                self.kuavo_eef_control = None
            else:
                self.kuavo_eef_control = ControlEndEffectorWebsocket(eef_type=eef_type)
            
            connect_success = True
            err_msg = ''
            if not self.kuavo_arm_control.connect(timeout):
                connect_success = False
                err_msg = "Failed to connect to arm control topics, \n"
            if not self.kuavo_head_control.connect(timeout):
                connect_success = False
                err_msg += "Failed to connect to head control topics, \n"
            if not self.kuavo_motion_control.connect(timeout):
                err_msg += "Failed to connect to motion control topics, \n"
                connect_success = False

            if self.kuavo_eef_control is not None and not self.kuavo_eef_control.connect(timeout):
                connect_success = False
                err_msg += "Failed to connect to end effector control topics."

            if connect_success:
                err_msg = 'success'
            return connect_success, err_msg
        except Exception as e:
            SDKLogger.error(f"Failed to initialize KuavoRobotControlWebsocket: {e}")
            return False, str(e)
    
    """ End Effector Control"""
    def control_robot_dexhand(self, left_position:list, right_position:list)->bool:
        """
            Control robot dexhand
            Args:
                left_position: list of 6 floats between 0 and 100
                right_position: list of 6 floats between 0 and 100
        """
        if self.kuavo_eef_control is None:
            SDKLogger.error("End effector control is not initialized.")
            return False
        
        if len(left_position) != 6 or len(right_position) != 6:
            raise ValueError("Position lists must have a length of 6.")
        
        for position in left_position + right_position:
            if position < 0.0 or position > 100.0:
                raise ValueError("All position values must be in the range [0.0, 100.0].")    
        
        SDKLogger.debug(f"Control robot dexhand: {left_position}, {right_position}")
        return self.kuavo_eef_control.pub_control_robot_dexhand(left_position, right_position)

    def robot_dexhand_command(self, data, ctrl_mode, hand_side):
        """
            Publish dexhand command
            Args:
                - data: list of 6 floats between 0 and 100
                - ctrl_mode: int between 0(position), 1(velocity)
                - hand_side: int between 0(left), 1(right), 2(dual)
        """
        if self.kuavo_eef_control is None:
            SDKLogger.error("End effector control is not initialized.")
            return False
        return self.kuavo_eef_control.pub_dexhand_command(data, ctrl_mode, hand_side)

    def execute_gesture(self, gestures:list)->bool:
        """
            Execute gestures
            Arguments:
                - gestures: list of dicts with keys 'gesture_name' and 'hand_side'
                 e.g. [{'gesture_name': 'fist', 'hand_side': 0},]
        """
        if self.kuavo_eef_control is None:
            SDKLogger.warn("End effectors control is not initialized.")
            return False
        return self.kuavo_eef_control.srv_execute_gesture(gestures)

    def get_gesture_names(self)->list:
        """
            Get the names of all gestures.
        """
        if self.kuavo_eef_control is None:
            SDKLogger.warn("End effectors control is not initialized.")
            return []
        return self.kuavo_eef_control.srv_get_gesture_names()

    def control_leju_claw(self, postions:list, velocities:list=[90, 90], torques:list=[1.0, 1.0]) ->bool:
        """
            Control leju claw
            Arguments:
                - postions: list of positions for left and right claw
                - velocities: list of velocities for left and right claw
                - torques: list of torques for left and right claw
        """
        if self.kuavo_eef_control is None:
            SDKLogger.warn("End effectors control is not initialized.")
            return False
        SDKLogger.debug(f"Control leju claw: {postions}, {velocities}, {torques}")
        if len(postions) != 2 or len(velocities) != 2 or len(torques) != 2:
                raise ValueError("Position, velocity, and torque lists must have a length of 2.")
        return self.kuavo_eef_control.srv_control_leju_claw(postions, velocities, torques)
    
    def control_robot_waist(self, yaw: float) -> bool:
        """
            Control robot waist
            Arguments:
                - yaw:   waist yaw angle, radian
        """
        SDKLogger.debug(f"Control robot waist: {yaw}")
        return self.kuavo_waist_control.pub_control_robot_waist(yaw)

    def control_robot_head(self, yaw:float, pitch:float)->bool:
        """
            Control robot head
            Arguments:
                - yaw: yaw angle, radian
                - pitch: pitch angle, radian
        """
        SDKLogger.debug(f"Control robot head: {yaw}, {pitch}")
        return self.kuavo_head_control.pub_control_robot_head(yaw, pitch)
    
    def control_robot_arm_joint_positions(self, joint_data:list)->bool:
        """
            Control robot arm joint positions
            Arguments:
                - joint_data: list of joint data (degrees)
        """
        return self.kuavo_arm_control.pub_control_robot_arm_traj(joint_data)

    def is_arm_collision(self)->bool:
        return self.kuavo_arm_control.is_arm_collision()
    
    def is_arm_collision_mode(self)->bool:
        """
            Check if arm collision mode is enabled
            Returns:
                bool: True if collision mode is enabled, False otherwise
        """
        return self.kuavo_arm_control.is_arm_collision_mode()



    def release_arm_collision_mode(self):
        return self.kuavo_arm_control.release_arm_collision_mode()
    
    def wait_arm_collision_complete(self):
        return self.kuavo_arm_control.wait_arm_collision_complete()

    def set_arm_collision_mode(self, enable: bool):
        """
            Set arm collision mode
        """
        return self.kuavo_arm_control.set_arm_collision_mode(enable)

    
    def control_robot_arm_joint_trajectory(self, times:list, joint_q:list)->bool:
        """
            Control robot arm joint trajectory
            Arguments:
                - times: list of times (seconds)
                - joint_q: list of joint data (degrees)
        """
        if len(times) != len(joint_q):
            raise ValueError("Times and joint_q must have the same length.")
        elif len(times) == 0:
            raise ValueError("Times and joint_q must not be empty.")
        
        return self.kuavo_arm_control.pub_arm_target_poses(times=times, joint_q=joint_q)
    
    def control_robot_end_effector_pose(self, left_pose: KuavoPose, right_pose: KuavoPose, frame: KuavoManipulationMpcFrame)->bool:
        """
            Control robot end effector pose
            Arguments:
                - left_pose: left end effector pose
                - right_pose: right end effector pose
                - frame: frame of the end effector pose, 0: keep current frame, 1: world frame, 2: local frame, 3: VR frame, 4: manipulation world frame
        """
        return self.kuavo_arm_control.pub_end_effector_pose_cmd(left_pose, right_pose, frame)
    
    def change_manipulation_mpc_frame(self, frame: KuavoManipulationMpcFrame)->bool:
        """
            Change manipulation mpc frame
            Arguments:
                - frame: frame of the manipulation mpc
        """
        return self.kuavo_arm_control.srv_change_manipulation_mpc_frame(frame)
    
    def change_manipulation_mpc_ctrl_mode(self, ctrl_mode: KuavoManipulationMpcCtrlMode)->bool:
        """
            Change manipulation mpc control mode
            Arguments:
                - control_mode: control mode of the manipulation mpc
        """
        return self.kuavo_arm_control.srv_change_manipulation_mpc_ctrl_mode(ctrl_mode)
    
    def change_manipulation_mpc_control_flow(self, ctrl_flow: KuavoManipulationMpcControlFlow)->bool:
        """
            Change manipulation mpc wbc arm traj control mode, control signal will be sent to wbc directly
            Arguments:
                - control_mode: control mode of the manipulation mpc wbc arm traj
        """
        return self.kuavo_arm_control.srv_change_manipulation_mpc_control_flow(ctrl_flow)
    
    def get_manipulation_mpc_ctrl_mode(self)->KuavoManipulationMpcCtrlMode:
        """
            Get manipulation mpc control mode
        """
        return self.kuavo_arm_control.srv_get_manipulation_mpc_ctrl_mode()
    
    def get_manipulation_mpc_frame(self)-> KuavoManipulationMpcFrame:
        """
            Get manipulation mpc frame
        """
        return self.kuavo_arm_control.srv_get_manipulation_mpc_frame()
    
    def get_manipulation_mpc_control_flow(self)->KuavoManipulationMpcControlFlow:
        """
            Get manipulation mpc wbc arm traj control mode
        """
        return self.kuavo_arm_control.srv_get_manipulation_mpc_control_flow()
    
    def change_robot_arm_ctrl_mode(self, mode:KuavoArmCtrlMode)->bool:
        """
            Change robot arm control mode
            Arguments:
                - mode: arm control mode
        """
        SDKLogger.debug(f"[WebSocket] Change robot arm control mode: {mode}")
        return self.kuavo_arm_control.srv_change_arm_ctrl_mode(mode)
    
    def get_robot_arm_ctrl_mode(self)->int:
        """
            Get robot arm control mode
        """
        return self.kuavo_arm_control.srv_get_arm_ctrl_mode()
    
    def arm_ik(self, left_pose: KuavoPose, right_pose: KuavoPose, 
               left_elbow_pos_xyz: list = [0.0, 0.0, 0.0],  
               right_elbow_pos_xyz: list = [0.0, 0.0, 0.0],
               arm_q0: list = None, params: KuavoIKParams=None) -> list:
        return self.kuavo_arm_ik_fk.arm_ik(left_pose, right_pose, left_elbow_pos_xyz, right_elbow_pos_xyz, arm_q0, params)

    def arm_fk(self, q: list) -> Tuple[KuavoPose, KuavoPose]:
        return self.kuavo_arm_ik_fk.arm_fk(q)
    
    """ Motion """
    def robot_stance(self)->bool:
        return self.kuavo_motion_control.pub_stance_command()

    def robot_trot(self)->bool:
        return self.kuavo_motion_control.pub_trot_command()
    
    def robot_walk(self, linear_x:float, linear_y:float, angular_z:float)->bool:
        return self.kuavo_motion_control.pub_cmd_vel(linear_x, linear_y, angular_z)
    
    def control_torso_height(self, height:float, pitch:float=0.0)->bool:
        """
        控制躯干高度和俯仰角（使用MPC目标轨迹接口）
        参数:
            height: 相对于当前高度的变化量（米），负值表示下蹲，正值表示上升
            pitch: 相对于当前俯仰角的变化量（弧度），默认0.0
        返回:
            bool: 控制成功返回True，否则返回False
        """
        # 获取当前状态
        from kuavo_humanoid_sdk.kuavo.core.ros.state import KuavoRobotStateCoreWebsocket
        state_core = KuavoRobotStateCoreWebsocket()
        if not hasattr(state_core, '_mpc_observation_data') or state_core._mpc_observation_data is None:
            SDKLogger.error("[Error] Cannot get MPC observation data for control_torso_height")
            return False
        
        obs = state_core._mpc_observation_data
        if 'state' not in obs or 'value' not in obs['state'] or len(obs['state']['value']) < 12:
            SDKLogger.error("[Error] Cannot get current state from observation for control_torso_height")
            return False
        
        # 从observation获取当前姿态 [x, y, z, yaw, pitch, roll]
        current_pose = [
            obs['state']['value'][6],   # p_base_x
            obs['state']['value'][7],   # p_base_y
            obs['state']['value'][8],   # p_base_z
            obs['state']['value'][9],   # theta_base_z (yaw)
            obs['state']['value'][10],  # theta_base_y (pitch)
            obs['state']['value'][11]   # theta_base_x (roll)
        ]
        
        # 计算目标姿态：当前姿态 + 变化量
        target_pose = [
            current_pose[0],           # x: 保持不变
            current_pose[1],           # y: 保持不变
            height,                    # z: 目标高度
            current_pose[3],           # yaw: 保持不变
            pitch,                     # pitch: 目标俯仰角
            current_pose[5]            # roll: 保持不变
        ]
        
        # 使用当前姿态作为初始姿态，目标姿态 = 当前姿态 + 变化量
        return self.kuavo_motion_control.pub_mpc_target_pose(target_pose, initial_pose=current_pose, time_horizon=3.0)

    def control_command_pose_world(self, target_pose_x:float, target_pose_y:float, target_pose_z:float, target_pose_yaw:float)->bool:
        """
            odom下的机器人cmd_pose_world
        """
        msg = {
            "linear": {"x": float(target_pose_x), "y": float(target_pose_y), "z": float(target_pose_z)},
            "angular": {"x": 0.0, "y": 0.0, "z": float(target_pose_yaw)}
        }
        return self.kuavo_motion_control.pub_cmd_pose_world(roslibpy.Message(msg))

    def control_command_pose(self, target_pose_x:float, target_pose_y:float, target_pose_z:float, target_pose_yaw:float)->bool:
        """
            base_link下的机器人cmd_pose
        """
        msg = {
            "linear": {"x": float(target_pose_x), "y": float(target_pose_y), "z": float(target_pose_z)},
            "angular": {"x": 0.0, "y": 0.0, "z": float(target_pose_yaw)}
        }
        return self.kuavo_motion_control.pub_cmd_pose(roslibpy.Message(msg))

    def step_control(self, body_poses:list, dt:float, is_left_first_default:bool=True, collision_check:bool=True)->bool:
        """
            Step control
            Arguments:
                - body_poses: list of body poses (x, y, z, yaw), meters and degrees
                - dt: time step (seconds)
                - is_left_first_default: whether to start with left foot
                - collision_check: whether to check for collisions
        """
        if len(body_poses) == 0:
            raise ValueError("Body poses must not be empty.")
        if dt <= 0.0:
            raise ValueError("Time step must be greater than 0.0.")
        for bp in body_poses:
            if len(bp) != 4:
                raise ValueError("Body pose must have 4 elements: [x, y, z, yaw]")
        
        msg = get_multiple_steps_msg(body_poses, dt, is_left_first_default, collision_check)
        return self.kuavo_motion_control.pub_step_ctrl(msg)


def euler_to_rotation_matrix(yaw, pitch, roll):
    # 计算各轴的旋转矩阵
    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0],
                      [0, 0, 1]])

    R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])

    # 按照 Yaw-Pitch-Roll 的顺序组合旋转矩阵
    R = np.dot(R_roll, np.dot(R_pitch, R_yaw))
    return R

def get_foot_pose_traj_msg(time_traj:list, foot_idx_traj:list, foot_traj:list, torso_traj:list):
    num = len(time_traj)

    msg = {
        "timeTrajectory": time_traj,
        "footIndexTrajectory": foot_idx_traj,
        "footPoseTrajectory": []
    }
    
    for i in range(num):
        foot_pose = {
            "footPose": foot_traj[i],
            "torsoPose": torso_traj[i]
        }
        msg["footPoseTrajectory"].append(foot_pose)


    return msg

def generate_steps(torso_pos, torso_yaw, foot_bias):
    l_foot_bias = np.array([0, foot_bias, -torso_pos[2]])
    r_foot_bias = np.array([0, -foot_bias, -torso_pos[2]])
    R_z = np.array([
        [np.cos(torso_yaw), -np.sin(torso_yaw), 0],
        [np.sin(torso_yaw), np.cos(torso_yaw), 0],
        [0, 0, 1]
    ])
    l_foot = torso_pos + R_z.dot(l_foot_bias)
    r_foot = torso_pos + R_z.dot(r_foot_bias)
    return l_foot, r_foot   

def get_multiple_steps_msg(body_poses:list, dt:float, is_left_first:bool=True, collision_check:bool=True):
    num_steps = 2*len(body_poses)
    time_traj = []
    foot_idx_traj = []
    foot_traj = []
    torso_traj = []
    l_foot_rect_last = RotatingRectangle(center=(0, 0.1), width=0.24, height=0.1, angle=0)
    r_foot_rect_last = RotatingRectangle(center=(0,-0.1), width=0.24, height=0.1, angle=0)
    torso_pose_last = np.array([0, 0, 0, 0])
    for i in range(num_steps):
        time_traj.append(dt * (i+1))
        body_pose = body_poses[i//2]
        torso_pos = np.asarray(body_pose[:3])
        torso_yaw = np.radians(body_pose[3])
        l_foot, r_foot = generate_steps(torso_pos, torso_yaw, 0.1)
        l_foot = [*l_foot[:3], torso_yaw]
        r_foot = [*r_foot[:3], torso_yaw]

        if(i%2 == 0):        
            torso_pose = np.array([*body_pose[:3], torso_yaw])
            R_wl = euler_to_rotation_matrix(torso_pose_last[3], 0, 0)
            delta_pos = R_wl.T @ (torso_pose[:3] - torso_pose_last[:3])
            # print("delta_pos:", delta_pos)
            if(torso_yaw > 0.0 or delta_pos[1] > 0.0):
                is_left_first = True
            else:
                is_left_first = False

        if(collision_check and i%2 == 0):
            l_foot_rect_next = RotatingRectangle(center=(l_foot[0],l_foot[1]), width=0.24, height=0.1, angle=torso_yaw)
            r_foot_rect_next = RotatingRectangle(center=(r_foot[0],r_foot[1]), width=0.24, height=0.1, angle=torso_yaw)
            l_collision = l_foot_rect_next.is_collision(r_foot_rect_last)
            r_collision = r_foot_rect_next.is_collision(l_foot_rect_last)
            if l_collision and r_collision:
                SDKLogger.error("[Control] Detect collision, Please adjust your body_poses input!!!")
                break
            elif l_collision:
                SDKLogger.warn("[Control] Left foot is in collision, switch to right foot")
                is_left_first = False
            elif r_collision:
                SDKLogger.warn("[Control] Right foot is in collision, switch to left foot")
                is_left_first = True
            l_foot_rect_last = l_foot_rect_next
            r_foot_rect_last = r_foot_rect_next
        if(i%2 == 0):
            torso_traj.append((torso_pose_last + torso_pose)/2.0)
            if is_left_first:
                foot_idx_traj.append(0)
                foot_traj.append(l_foot)
            else:
                foot_idx_traj.append(1)
                foot_traj.append(r_foot)
        else:
            torso_traj.append(torso_pose)
            if is_left_first:
                foot_idx_traj.append(1)
                foot_traj.append(r_foot)
            else:
                foot_idx_traj.append(0)
                foot_traj.append(l_foot)
        torso_pose_last = torso_traj[-1]
    # print("time_traj:", time_traj)
    # print("foot_idx_traj:", foot_idx_traj)
    # print("foot_traj:", foot_traj)
    # print("torso_traj:", torso_traj)
    return get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj)
""" ------------------------------------------------------------------------------"""


# if __name__ == "__main__":
#     control = KuavoRobotControl()
#     control.change_manipulation_mpc_frame(KuavoManipulationMpcFrame.KeepCurrentFrame)
#     control.change_manipulation_mpc_ctrl_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
#     control.change_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.DirectToWbc)
#     print(control.get_manipulation_mpc_ctrl_mode())
#     print(control.get_manipulation_mpc_frame())
#     print(control.get_manipulation_mpc_control_flow())
#     control.change_manipulation_mpc_frame(KuavoManipulationMpcFrame.WorldFrame)
#     control.change_robot_arm_ctrl_mode(KuavoArmCtrlMode.ExternalControl)
#     control.control_robot_end_effector_pose(KuavoPose(position=[0.3, 0.4, 0.9], orientation=[0.0, 0.0, 0.0, 1.0]), KuavoPose(position=[0.3, -0.5, 1.0], orientation=[0.0, 0.0, 0.0, 1.0]), KuavoManipulationMpcFrame.WorldFrame)
