import os
import numpy as np
from typing import Tuple
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.interfaces.data_types import (KuavoArmCtrlMode, KuavoIKParams, KuavoPose, 
                                                      KuavoManipulationMpcControlFlow, KuavoManipulationMpcCtrlMode
                                                      ,KuavoManipulationMpcFrame, KuavoMotorParam)
from kuavo_humanoid_sdk.kuavo.core.ros.sat_utils import RotatingRectangle
from kuavo_humanoid_sdk.kuavo.core.ros.param import EndEffectorType

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Joy
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import (gestureTask,robotHandPosition, robotHeadMotionData, armTargetPoses, switchGaitByName,
                                footPose, footPoseTargetTrajectories, dexhandCommand, motorParam)
from kuavo_humanoid_sdk.msg.kuavo_msgs.srv import (gestureExecute, gestureExecuteRequest,gestureList, gestureListRequest,
                        controlLejuClaw, controlLejuClawRequest, changeArmCtrlMode, changeArmCtrlModeRequest,
                        changeTorsoCtrlMode, changeTorsoCtrlModeRequest, setMmCtrlFrame, setMmCtrlFrameRequest,
                        setTagId, setTagIdRequest, getMotorParam, getMotorParamRequest,
                        changeMotorParam, changeMotorParamRequest)
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam, armHandPose, armCollisionCheckInfo
from kuavo_humanoid_sdk.msg.kuavo_msgs.srv import twoArmHandPoseCmdSrv, fkSrv
from std_srvs.srv import SetBool, SetBoolRequest
from std_msgs.msg import Float64MultiArray




class ControlEndEffector:
    def __init__(self, eef_type: str = EndEffectorType.QIANGNAO):
        self._eef_type = eef_type
        self._pubs = []
        if self._eef_type == EndEffectorType.QIANGNAO:
            self._pub_ctrl_robot_hand = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)                
            # publisher, name, require
            self._pubs.append((self._pub_ctrl_robot_hand, False))
        elif self._eef_type == EndEffectorType.QIANGNAO_TOUCH:
            self._pub_ctrl_robot_hand = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)                
            self._pub_dexhand_command = rospy.Publisher('/dexhand/command', dexhandCommand, queue_size=10)
            self._pub_dexhand_right_command = rospy.Publisher('/dexhand/right/command', dexhandCommand, queue_size=10)
            self._pub_dexhand_left_command = rospy.Publisher('/dexhand/left/command', dexhandCommand, queue_size=10)
            # publisher, name, require
            self._pubs.append((self._pub_dexhand_command, False))
            self._pubs.append((self._pub_dexhand_right_command, False))
            self._pubs.append((self._pub_dexhand_left_command, False))
            
    def connect(self, timeout:float=1.0)-> bool:
        start_time = rospy.Time.now()
        
        success = True
        for pub, require in self._pubs:
            while pub.get_num_connections() == 0:
                if (rospy.Time.now() - start_time).to_sec() > timeout:
                    if require:
                        SDKLogger.error(f"Timeout waiting for {pub.name} connection")
                        success = False
                    break
                rospy.sleep(0.1)

        return success

    """ Control Kuavo Robot Dexhand """
    def pub_control_robot_dexhand(self, left_position:list, right_position:list)->bool:
        if not self._eef_type.startswith(EndEffectorType.QIANGNAO): # qiangnao, qiangnao_touch
            SDKLogger.warning(f"{self._eef_type} not support control dexhand")
            return False
        try :
            hand_pose_msg = robotHandPosition()
            hand_pose_msg.left_hand_position = bytes(left_position)
            hand_pose_msg.right_hand_position = bytes(right_position)
            self._pub_ctrl_robot_hand.publish(hand_pose_msg)
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
            msg = dexhandCommand()
            msg.data = [int(d) for d in data]  # Convert data to integers
            msg.control_mode = ctrl_mode
            if hand_side == 0:
                self._pub_dexhand_left_command.publish(msg)
            elif hand_side == 1:
                self._pub_dexhand_right_command.publish(msg)
            else:
                self._pub_dexhand_command.publish(msg)
            return True        
        except Exception as e:
            SDKLogger.error(f"Failed to publish left dexhand command: {e}")
            return False
        
    def srv_execute_gesture(self, gestures:list)->bool:
        if not self._eef_type.startswith(EndEffectorType.QIANGNAO): # qiangnao, qiangnao_touch
            SDKLogger.warning(f"{self._eef_type} not support control dexhand")
            return False
        try:
            service_name = 'gesture/execute'
            rospy.wait_for_service(service=service_name, timeout=2.0)
            # create service proxy
            gesture_service = rospy.ServiceProxy(service_name, gestureExecute)
            
            # request
            request = gestureExecuteRequest()
            for gs in gestures:
                gesture = gestureTask(gesture_name=gs["gesture_name"], hand_side=gs["hand_side"])
                request.gestures.append(gesture)
            
            # call service
            response = gesture_service(request)
            if not response.success:
                SDKLogger.error(f"Failed to execute gesture '{gestures}': {response.message}")
            return response.success
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Service call failed: {e}")
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
        return False

    def srv_get_gesture_names(self)->list:
        if not self._eef_type.startswith(EndEffectorType.QIANGNAO): # qiangnao, qiangnao_touch
            SDKLogger.warning(f"{self._eef_type} not support control dexhand")
            return []
        try:
            service_name = 'gesture/list'
            rospy.wait_for_service(service=service_name, timeout=2.0)
            gesture_service = rospy.ServiceProxy(service_name, gestureList)
            request = gestureListRequest()
            response = gesture_service(request)
            gestures = []
            for gesture_info in response.gesture_infos:
                gestures.append(gesture_info.gesture_name)
                for alias in gesture_info.alias:
                    gestures.append(alias)
            return list(set(gestures))
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call failed: {e}")
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
        return []

    def srv_control_leju_claw(self, postions:list, velocities:list, torques:list) ->bool:
        if self._eef_type != 'lejuclaw':
            SDKLogger.warning(f"{self._eef_type} not support control lejuclaw.")
            return False
        try:
            # ros service
            service_name = 'control_robot_leju_claw'
            rospy.wait_for_service(service_name, timeout=2.0)
            control_lejucalw_srv = rospy.ServiceProxy(service_name, controlLejuClaw)
            
            # request
            request = controlLejuClawRequest()
            request.data.position = postions
            request.data.velocity = velocities
            request.data.effort = torques
            
            response = control_lejucalw_srv(request)
            if not response.success:
                SDKLogger.error(f"Failed to control leju claw: {response.message}")
            return response.success
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service `control_robot_leju_claw` call failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Service `control_robot_leju_claw` call failed: {e}")
        except Exception as e:
            SDKLogger.error(f"Service `control_robot_leju_claw` call failed: {e}")
        return False

class ControlRobotArm:
    def __init__(self):

        # 带有碰撞检查的轨迹发布
        self._pub_ctrl_arm_traj_arm_collision = rospy.Publisher('/arm_collision/kuavo_arm_traj', JointState, queue_size=10)
        self._pub_ctrl_arm_target_poses_arm_collision = rospy.Publisher('/arm_collision/kuavo_arm_target_poses', armTargetPoses, queue_size=10)

        # 判断当前是否发生碰撞
        self._sub_arm_collision_info = rospy.Subscriber('/arm_collision/info', armCollisionCheckInfo, self.callback_arm_collision_info, queue_size=10)
        self._is_collision = False
        self.arm_collision_enable = False

        # 正常轨迹发布
        self._pub_ctrl_arm_traj = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
        self._pub_ctrl_arm_target_poses = rospy.Publisher('/kuavo_arm_target_poses', armTargetPoses, queue_size=10)
        self._pub_ctrl_hand_pose_cmd = rospy.Publisher('/mm/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
        self._pub_hand_wrench = rospy.Publisher('/hand_wrench_cmd', Float64MultiArray, queue_size=10)

    def is_arm_collision(self)->bool:
        return self._is_collision
    
    def is_arm_collision_mode(self)->bool:
        return self.arm_collision_enable
    
    def callback_arm_collision_info(self, msg: armCollisionCheckInfo):
        self._is_collision = True
        SDKLogger.info(f"Arm collision detected")

    def set_arm_collision_mode(self, enable: bool):
        """
            Set arm collision mode
        """
        self.arm_collision_enable = enable
        srv_set_arm_collision_mode_srv = rospy.ServiceProxy('/arm_collision/set_arm_moving_enable', SetBool)
        req = SetBoolRequest()
        req.data = enable
        resp = srv_set_arm_collision_mode_srv(req)
        if not resp.success:
            SDKLogger.error(f"Failed to wait arm collision complete: {resp.message}")
    
        

    def release_arm_collision_mode(self):
        self._is_collision = False
    
    def wait_arm_collision_complete(self):
        if self._is_collision:
            srv_wait_arm_collision_complete_srv = rospy.ServiceProxy('/arm_collision/wait_complete', SetBool)
            req = SetBoolRequest()
            req.data = True
            resp = srv_wait_arm_collision_complete_srv(req)
            if not resp.success:
                SDKLogger.error(f"Failed to wait arm collision complete: {resp.message}")

    def connect(self, timeout:float=1.0)-> bool:
        start_time = rospy.Time.now()
        publishers = [
            (self._pub_ctrl_arm_traj, "arm trajectory publisher", False),
            (self._pub_ctrl_arm_target_poses, "arm target poses publisher", False)
        ]
        
        success = True
        for pub, name, required in publishers:
            while pub.get_num_connections() == 0:
                if (rospy.Time.now() - start_time).to_sec() > timeout:
                    SDKLogger.error(f"Timeout waiting for {name} connection, '{pub.name}'")
                    if required:
                        success = False
                    break
                rospy.sleep(0.1)
        return success

    def pub_control_robot_arm_traj(self, joint_q: list)->bool:
        
        try:
            msg = JointState()
            msg.name = ["arm_joint_" + str(i) for i in range(0, 14)]
            msg.header.stamp = rospy.Time.now()
            msg.position = 180.0 / np.pi * np.array(joint_q) # convert to degree
            if self.arm_collision_enable:
                self._pub_ctrl_arm_traj_arm_collision.publish(msg)
            else:
                self._pub_ctrl_arm_traj.publish(msg)
            return True
        except Exception as e:
            SDKLogger.error(f"publish robot arm traj: {e}")
        return False
        
    def pub_arm_target_poses(self, times:list, joint_q:list):
        try:
            msg = armTargetPoses()
            msg.times = times
            for i in range(len(joint_q)):
                degs = [q for q in joint_q[i]]
                msg.values.extend(degs)
            if self.arm_collision_enable:
                self._pub_ctrl_arm_target_poses_arm_collision.publish(msg)
            else:
                self._pub_ctrl_arm_target_poses.publish(msg)
            return True
        except Exception as e:
            SDKLogger.error(f"publish arm target poses: {e}")
        return False
    def pub_end_effector_pose_cmd(self, left_pose: KuavoPose, right_pose: KuavoPose, frame: KuavoManipulationMpcFrame)->bool:
        try:
            msg = twoArmHandPoseCmd()
            left_pose_msg = armHandPose()
            left_pose_msg.pos_xyz = left_pose.position
            left_pose_msg.quat_xyzw = left_pose.orientation
            right_pose_msg = armHandPose()
            right_pose_msg.pos_xyz = right_pose.position
            right_pose_msg.quat_xyzw = right_pose.orientation
            msg.hand_poses.left_pose = left_pose_msg
            msg.hand_poses.right_pose = right_pose_msg
            if frame.value not in [0, 1, 2, 3, 4]:
                SDKLogger.error(f"Invalid frame: {frame}")
                return False
            msg.frame = frame.value
            self._pub_ctrl_hand_pose_cmd.publish(msg)
            return True
        except Exception as e:
            SDKLogger.error(f"publish arm target poses: {e}")
        return False

    def srv_change_manipulation_mpc_frame(self, frame: KuavoManipulationMpcFrame)->bool:
        try:
            service_name = '/set_mm_ctrl_frame'
            rospy.wait_for_service(service_name, timeout=2.0)
            set_frame_srv = rospy.ServiceProxy(service_name, setMmCtrlFrame)
            
            req = setMmCtrlFrameRequest()
            req.frame = frame.value
            
            resp = set_frame_srv(req)
            if not resp.result:
                SDKLogger.error(f"Failed to change manipulation mpc frame to {frame}: {resp.message}")
            return resp.result
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e: # For timeout from wait_for_service
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to change manipulation mpc frame: {e}")
        return False
    
    def srv_change_manipulation_mpc_ctrl_mode(self, ctrl_mode: KuavoManipulationMpcCtrlMode)->bool:
        try:
            service_name = '/mobile_manipulator_mpc_control'
            rospy.wait_for_service(service_name, timeout=2.0)
            set_mode_srv = rospy.ServiceProxy(service_name, changeTorsoCtrlMode)
            
            req = changeTorsoCtrlModeRequest()
            req.control_mode = ctrl_mode.value
            
            resp = set_mode_srv(req)
            if not resp.result:
                SDKLogger.error(f"Failed to change manipulation mpc control mode to {ctrl_mode}: {resp.message}")
            return resp.result
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e: # For timeout from wait_for_service
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to change manipulation mpc control mode: {e}")
        return False

    def srv_change_manipulation_mpc_control_flow(self, ctrl_flow: KuavoManipulationMpcControlFlow)-> bool:
        try:
            service_name = '/enable_mm_wbc_arm_trajectory_control'
            rospy.wait_for_service(service_name, timeout=2.0)
            set_mode_srv = rospy.ServiceProxy(service_name, changeArmCtrlMode)

            req = changeArmCtrlModeRequest()
            req.control_mode = ctrl_flow.value

            resp = set_mode_srv(req)
            if not resp.result:
                SDKLogger.error(f"Failed to change manipulation mpc wbc arm trajectory control to {ctrl_flow}: {resp.message}")
            return resp.result
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e:  # For timeout from wait_for_service
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to change manipulation mpc control flow: {e}")
        return False

    def srv_get_manipulation_mpc_ctrl_mode(self, )->KuavoManipulationMpcCtrlMode:
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

    def srv_get_manipulation_mpc_frame(self, )->KuavoManipulationMpcFrame:
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

    def srv_get_manipulation_mpc_control_flow(self, )->KuavoManipulationMpcControlFlow:
        try:
            service_name = '/get_mm_wbc_arm_trajectory_control'
            rospy.wait_for_service(service_name, timeout=2.0)
            get_mode_srv = rospy.ServiceProxy(service_name, changeArmCtrlMode)
            
            req = changeArmCtrlModeRequest()
            
            resp = get_mode_srv(req)
            if not resp.result:
                SDKLogger.error(f"Failed to get manipulation mpc wbc arm trajectory control mode: {resp.message}")
                return KuavoManipulationMpcControlFlow.ERROR
            return KuavoManipulationMpcControlFlow(resp.mode)
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e: # For timeout from wait_for_service
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to get manipulation mpc wbc arm trajectory control mode: {e}")
        return KuavoManipulationMpcControlFlow.ERROR


    def srv_change_arm_ctrl_mode(self, mode: KuavoArmCtrlMode)->bool:
        try:
            rospy.wait_for_service('/change_arm_ctrl_mode', timeout=2.0)
            change_arm_ctrl_mode_srv = rospy.ServiceProxy('/change_arm_ctrl_mode', changeArmCtrlMode)
            req = changeArmCtrlModeRequest()
            req.control_mode = mode.value
            resp = change_arm_ctrl_mode_srv(req)
            return resp.result
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call failed: {e}")
        except Exception as e:
            SDKLogger.error(f"[Error] change arm ctrl mode: {e}")
        return False
    
    def srv_get_arm_ctrl_mode(self)-> KuavoArmCtrlMode:
        try:
            rospy.wait_for_service('/humanoid_get_arm_ctrl_mode')
            get_arm_ctrl_mode_srv = rospy.ServiceProxy('/humanoid_get_arm_ctrl_mode', changeArmCtrlMode)
            req = changeArmCtrlModeRequest()
            resp = get_arm_ctrl_mode_srv(req)
            return KuavoArmCtrlMode(resp.control_mode)
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call failed: {e}")
        except Exception as e:
            SDKLogger.error(f"[Error] get arm ctrl mode: {e}")
        return None
    
    def pub_hand_wrench_cmd(self, left_wrench, right_wrench):
        """
        发布末端力控制命令
        参数:
            left_wrench: 左手6维力控指令 [Fx, Fy, Fz, Tx, Ty, Tz]
            right_wrench: 右手6维力控指令 [Fx, Fy, Fz, Tx, Ty, Tz]
            Fx: 沿X轴的线性力
            Fy: 沿Y轴的线性力
            Fz: 沿Z轴的线性力
            Tx: 绕X轴的力矩
            Ty: 绕Y轴的力矩
            Tz: 绕Z轴的力矩
        """
        if len(left_wrench) != 6 or len(right_wrench) != 6:
            SDKLogger.error("Wrench data must be 6-dimensional")
            return False
            
        try:
            msg = Float64MultiArray()
            msg.data = list(left_wrench) + list(right_wrench)
            self._pub_hand_wrench.publish(msg)
            return True
        except Exception as e:
            SDKLogger.error(f"Publish hand wrench failed: {e}")
            return False
        
""" Control Robot Head """
class ControlRobotHead:
    def __init__(self):
        self._pub_ctrl_robot_head = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=10)
    
    def connect(self, timeout:float=1.0)->bool:
        start_time = rospy.Time.now()
        publishers = [
            (self._pub_ctrl_robot_head, "robot head publisher", False) # not need check!
        ]
        for pub, name, required in publishers:
            while pub.get_num_connections() == 0:
                if (rospy.Time.now() - start_time).to_sec() > timeout:
                    SDKLogger.error(f"Timeout waiting for {name} connection, '{pub.name}'")
                    if required:
                        return False
                    break
                rospy.sleep(0.1)
        return True
    def pub_control_robot_head(self, yaw:float, pitch:float)->bool:
        try :
            msg = robotHeadMotionData()
            msg.joint_data = [yaw, pitch]
            self._pub_ctrl_robot_head.publish(msg)
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
            # 首先设置追踪目标ID
            service_name = '/set_target_tag_id'
            rospy.wait_for_service(service_name, timeout=2.0)
            set_tag_id_srv = rospy.ServiceProxy(service_name, setTagId)
            
            req = setTagIdRequest()
            req.tag_id = target_id
            
            resp = set_tag_id_srv(req)
            if not resp.success:
                SDKLogger.error(f"Failed to set target tag ID: {resp.message}")
                return False
                
            SDKLogger.info(f"Successfully set target tag ID to {target_id}: {resp.message}")
            
            # 然后启动连续追踪
            service_name = '/continuous_track'
            rospy.wait_for_service(service_name, timeout=2.0)
            continuous_track_srv = rospy.ServiceProxy(service_name, SetBool)
            
            req = SetBoolRequest()
            req.data = True
            
            resp = continuous_track_srv(req)
            if not resp.success:
                SDKLogger.error(f"Failed to start continuous tracking: {resp.message}")
                return False
                
            SDKLogger.info(f"Successfully started continuous tracking: {resp.message}")
            return True
            
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Failed to connect to service: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to enable head tracking: {e}")
            
        return False
        
    def srv_disable_head_tracking(self)->bool:
        """Disable the head tracking.
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            service_name = '/continuous_track'
            rospy.wait_for_service(service_name, timeout=2.0)
            continuous_track_srv = rospy.ServiceProxy(service_name, SetBool)
            
            req = SetBoolRequest()
            req.data = False
            
            resp = continuous_track_srv(req)
            if not resp.success:
                SDKLogger.error(f"Failed to stop continuous tracking: {resp.message}")
                return False
                
            SDKLogger.info(f"Successfully stopped continuous tracking: {resp.message}")
            return True
            
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Failed to connect to service: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to disable head tracking: {e}")
            
        return False

    def srv_enable_head_tracking(self, target_id: int)->bool:
        """Enable the head tracking for a specific tag ID.
        
        Args:
            target_id: The ID of the tag to track
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # 首先设置追踪目标ID
            service_name = '/set_target_tag_id'
            rospy.wait_for_service(service_name, timeout=2.0)
            set_tag_id_srv = rospy.ServiceProxy(service_name, setTagId)
            
            req = setTagIdRequest()
            req.tag_id = target_id
            
            resp = set_tag_id_srv(req)
            if not resp.success:
                SDKLogger.error(f"Failed to set target tag ID: {resp.message}")
                return False
                
            SDKLogger.info(f"Successfully set target tag ID to {target_id}: {resp.message}")
            
            # 然后启动连续追踪
            service_name = '/continuous_track'
            rospy.wait_for_service(service_name, timeout=2.0)
            continuous_track_srv = rospy.ServiceProxy(service_name, SetBool)
            
            req = SetBoolRequest()
            req.data = True
            
            resp = continuous_track_srv(req)
            if not resp.success:
                SDKLogger.error(f"Failed to start continuous tracking: {resp.message}")
                return False
                
            SDKLogger.info(f"Successfully started continuous tracking: {resp.message}")
            return True
            
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Failed to connect to service: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to enable head tracking: {e}")
            
        return False
        
    def srv_disable_head_tracking(self)->bool:
        """Disable the head tracking.
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            service_name = '/continuous_track'
            rospy.wait_for_service(service_name, timeout=2.0)
            continuous_track_srv = rospy.ServiceProxy(service_name, SetBool)
            
            req = SetBoolRequest()
            req.data = False
            
            resp = continuous_track_srv(req)
            if not resp.success:
                SDKLogger.error(f"Failed to stop continuous tracking: {resp.message}")
                return False
                
            SDKLogger.info(f"Successfully stopped continuous tracking: {resp.message}")
            return True
            
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Failed to connect to service: {e}")
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


class ControlRobotMotion:
    def __init__(self):
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._pub_cmd_pose = rospy.Publisher('/cmd_pose', Twist, queue_size=10)
        self._pub_cmd_pose_world = rospy.Publisher('/cmd_pose_world', Twist, queue_size=10)
        self._pub_joy = rospy.Publisher('/joy', Joy, queue_size=10)
        self._pub_switch_gait = rospy.Publisher('/humanoid_switch_gait_by_name', switchGaitByName, queue_size=10)
        self._pub_step_ctrl = rospy.Publisher('/humanoid_mpc_foot_pose_target_trajectories', footPoseTargetTrajectories, queue_size=10)

    def connect(self, timeout:float=3.0)-> bool:
        start_time = rospy.Time.now()
        publishers = [
            # (self._pub_joy, "joy publisher"),
            # pub name required
            (self._pub_cmd_vel, "cmd_vel publisher", False),
            (self._pub_cmd_pose, "cmd_pose publisher", False),
            (self._pub_step_ctrl, "step_ctrl publisher", False),
            (self._pub_switch_gait, "switch_gait publisher", False),
            (self._pub_cmd_pose_world, "cmd_pose_world publisher", False),
        ]
        
        success = True
        for pub, name, required in publishers:
            while pub.get_num_connections() == 0:
                if (rospy.Time.now() - start_time).to_sec() > timeout:
                    SDKLogger.error(f"Timeout waiting for {name} connection, '{pub.name}'")
                    if required:
                        success = False
                    break
                rospy.sleep(0.1)
        return success

    def pub_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float)->bool:
        try:
            twist = Twist()
            twist.linear.x = linear_x
            twist.linear.y = linear_y
            twist.angular.z = angular_z
            self._pub_cmd_vel.publish(twist)
            return True
        except Exception as e:
            SDKLogger.error(f"[Error] publish cmd vel: {e}")
            return False

    def pub_cmd_pose(self, twist)->bool:
        try:
            self._pub_cmd_pose.publish(twist)
            return True
        except Exception as e:
            SDKLogger.error(f"[Error] publish cmd pose: {e}")
            return False    

    def pub_cmd_pose_world(self, twist:Twist)->bool:
        try:
            self._pub_cmd_pose_world.publish(twist)
            return True
        except Exception as e:
            SDKLogger.error(f"[Error] publish cmd pose world: {e}")
            return False

    def _create_joy_msg(self)->Joy:
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 8  # Initialize 8 axes
        joy_msg.buttons = [0] * 16  # Initialize 16 buttons
        return joy_msg

    def _pub_joy_command(self, button_index: int, command_name: str) -> bool:
        try:
            joy_msg = self._create_joy_msg()
            joy_msg.buttons[button_index] = 1
            self._pub_joy.publish(joy_msg)
            SDKLogger.debug(f"Published {command_name} command")
            return True
        except Exception as e:
            SDKLogger.error(f"[Error] publish {command_name}: {e}")
            return False

    def _pub_switch_gait_by_name(self, gait_name: str) -> bool:
        try:
            msg = switchGaitByName()
            msg.header.stamp = rospy.Time.now()
            msg.gait_name = gait_name
            self._pub_switch_gait.publish(msg)
            # print(f"Published {gait_name} gait")
            return True
        except Exception as e:
            SDKLogger.error(f"[Error] publish switch gait {gait_name}: {e}")
            return False

    def pub_walk_command(self) -> bool:
        # return self._pub_joy_command(BUTTON_Y, "walk")
        return self._pub_switch_gait_by_name("walk")

    def pub_stance_command(self) -> bool:
        try:
            self.pub_cmd_vel(linear_x=0.0, linear_y=0.0, angular_z=0.0)
            # return self._pub_joy_command(BUTTON_A, "stance") 
            return self._pub_switch_gait_by_name("stance")
        except Exception as e:
            SDKLogger.error(f"[Error] publish stance: {e}")
            return False

    def pub_trot_command(self) -> bool:
        # return self._pub_joy_command(BUTTON_B, "trot")
        return self._pub_switch_gait_by_name("walk")
    
    def pub_step_ctrl(self, msg)->bool:
        try:
            self._pub_step_ctrl.publish(msg)
            return True
        except Exception as e:
            SDKLogger.error(f"[Error] publish step ctrl: {e}")
            return False

class KuavoRobotArmIKFK:
    def __init__(self):
        pass
    def arm_ik(self, 
               left_pose: KuavoPose, 
               right_pose: KuavoPose, 
               left_elbow_pos_xyz: list = [0.0, 0.0, 0.0],
               right_elbow_pos_xyz: list = [0.0, 0.0, 0.0],
               arm_q0: list = None,
               params: KuavoIKParams=None) -> list:
        eef_pose_msg = twoArmHandPoseCmd()
        if arm_q0 is None:
            eef_pose_msg.joint_angles_as_q0 = False
        else:
            eef_pose_msg.joint_angles_as_q0 = True
            eef_pose_msg.joint_angles = arm_q0
        
        if params is None:
            eef_pose_msg.use_custom_ik_param = False
        else:
            eef_pose_msg.use_custom_ik_param = True
            eef_pose_msg.ik_param.major_optimality_tol = params.major_optimality_tol
            eef_pose_msg.ik_param.major_feasibility_tol = params.major_feasibility_tol
            eef_pose_msg.ik_param.minor_feasibility_tol = params.minor_feasibility_tol
            eef_pose_msg.ik_param.major_iterations_limit = params.major_iterations_limit
            eef_pose_msg.ik_param.oritation_constraint_tol= params.oritation_constraint_tol
            eef_pose_msg.ik_param.pos_constraint_tol = params.pos_constraint_tol 
            eef_pose_msg.ik_param.pos_cost_weight = params.pos_cost_weight

        # left hand
        eef_pose_msg.hand_poses.left_pose.pos_xyz =  left_pose.position
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = left_pose.orientation
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = left_elbow_pos_xyz

        # right hand
        eef_pose_msg.hand_poses.right_pose.pos_xyz =  right_pose.position
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = right_pose.orientation
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = right_elbow_pos_xyz

        return self._srv_arm_ik(eef_pose_msg)   

    def arm_fk(self, q: list) -> Tuple[KuavoPose, KuavoPose]:
        return self._srv_arm_fk(q)
    
    def _srv_arm_ik(self, eef_pose_msg)->list:
        try:
            rospy.wait_for_service('/ik/two_arm_hand_pose_cmd_srv',timeout=1.0)
            ik_srv = rospy.ServiceProxy('/ik/two_arm_hand_pose_cmd_srv', twoArmHandPoseCmdSrv)
            res = ik_srv(eef_pose_msg)
            # print(eef_pose_msg)
            if res.success:
                return res.hand_poses.left_pose.joint_angles + res.hand_poses.right_pose.joint_angles
            else:
                return None
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None
        except Exception as e:
            print(f"Failed to call ik/fk_srv: {e}")
            return None
    def _srv_arm_fk(self, q: list) -> Tuple[KuavoPose, KuavoPose]:
        try:
            rospy.wait_for_service('/ik/fk_srv',timeout=1.0)
            ik_srv = rospy.ServiceProxy('/ik/fk_srv', fkSrv)
            res = ik_srv(q)
            if res.success:
                return KuavoPose(position=res.hand_poses.left_pose.pos_xyz, orientation=res.hand_poses.left_pose.quat_xyzw), \
                    KuavoPose(position=res.hand_poses.right_pose.pos_xyz, orientation=res.hand_poses.right_pose.quat_xyzw),
            else:
                return None
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None
        except Exception as e:
            print(f"Failed to call ik/fk_srv: {e}")
            return None



"""
    Kuavo Robot Control 
"""
class KuavoRobotControl:
    _instance = None
    
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            if not rospy.core.is_initialized():
                if not rospy.get_node_uri():
                    rospy.init_node(f'kuavo_sdk_node_{os.getpid()}', anonymous=True, disable_signals=True)
        return cls._instance
    
    def __init__(self):
        if not hasattr(self, '_initialized'):
            self._initialized = True
            self.kuavo_eef_control = None
            self.kuavo_head_control = ControlRobotHead()
            self.kuavo_arm_control = ControlRobotArm()
            self.kuavo_motion_control = ControlRobotMotion()
            self.kuavo_arm_ik_fk = KuavoRobotArmIKFK()
            # SDKLogger.debug("KuavoRobotControl initialized.")

    def initialize(self, eef_type:str=None, debug:bool=False, timeout:float=1.0)-> Tuple[bool, str]:
        # init eef control
        if eef_type is None:
            self.kuavo_eef_control = None
        else:
            self.kuavo_eef_control = ControlEndEffector(eef_type=eef_type)
        
        connect_success = True
        err_msg = ''
        if not self.kuavo_arm_control.connect(timeout):
            connect_success  = False
            err_msg = "Failed to connect to arm control topics, \n"
        if not self.kuavo_head_control.connect(timeout):
            connect_success  = False
            err_msg += "Failed to connect to head control topics, \n"
        if not self.kuavo_motion_control.connect(timeout):
            err_msg += "Failed to connect to motion control topics, \n"
            connect_success  = False

        if self.kuavo_eef_control is not None and not self.kuavo_eef_control.connect(timeout):
            connect_success  = False
            err_msg += "Failed to connect to end effector control topics."

        if connect_success:
            err_msg = 'success'
        return connect_success, err_msg
    
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
    """--------------------------------------------------------------------------------------------"""
    def control_robot_head(self, yaw:float, pitch:float)->bool:
        """
            Control robot head
            Arguments:
                - yaw: yaw angle, radian
                - pitch: pitch angle, radian
        """
        SDKLogger.debug(f"Control robot head: {yaw}, {pitch}")
        return self.kuavo_head_control.pub_control_robot_head(yaw, pitch)
    
    def enable_head_tracking(self, target_id: int)->bool:
        """Enable the head tracking for a specific tag ID.
        
        Args:
            target_id: The ID of the tag to track
            
        Returns:
            bool: True if successful, False otherwise
        """
        SDKLogger.debug(f"Enable head tracking: {target_id}")
        return self.kuavo_head_control.srv_enable_head_tracking(target_id)
    
    def disable_head_tracking(self)->bool:
        """Disable the head tracking.
        
        Returns:
            bool: True if successful, False otherwise
        """
        SDKLogger.debug(f"Disable head tracking")
        return self.kuavo_head_control.srv_disable_head_tracking()
    
    def control_robot_arm_joint_positions(self, joint_data:list)->bool:
        """
            Control robot arm joint positions
            Arguments:
                - joint_data: list of joint data (degrees)
        """
        # SDKLogger.debug(f"[ROS] Control robot arm trajectory: {joint_data}")
        return self.kuavo_arm_control.pub_control_robot_arm_traj(joint_data)
    
    def is_arm_collision(self)->bool:
        """
            Check if arm collision is happening
            Returns:
                bool: True if collision is happening, False otherwise
        """
        return self.kuavo_arm_control.is_arm_collision()
    
    def is_arm_collision_mode(self)->bool:
        """
            Check if arm collision mode is enabled
            Returns:
                bool: True if collision mode is enabled, False otherwise
        """
        return self.kuavo_arm_control.is_arm_collision_mode()

    def release_arm_collision_mode(self):
        """
            Release arm collision mode
        """
        return self.kuavo_arm_control.release_arm_collision_mode()
    
    def wait_arm_collision_complete(self):
        """
            Wait for arm collision to complete
        """
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
        SDKLogger.debug(f"[ROS] Change robot arm control mode: {mode}")
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
        com_msg = Twist()
        com_msg.linear.z = height
        com_msg.angular.y = pitch
        return self.kuavo_motion_control.pub_cmd_pose(com_msg)

    def control_command_pose_world(self, target_pose_x:float, target_pose_y:float, target_pose_z:float, target_pose_yaw:float)->bool:
        """
            odom下的机器人cmd_pose_world
        """
        com_msg = Twist()
        com_msg.linear.x = target_pose_x
        com_msg.linear.y = target_pose_y
        com_msg.linear.z = target_pose_z
        com_msg.angular.z = target_pose_yaw
        return self.kuavo_motion_control.pub_cmd_pose_world(com_msg)

    def control_command_pose(self, target_pose_x:float, target_pose_y:float, target_pose_z:float, target_pose_yaw:float)->bool:
        """
            base_link下的机器人cmd_pose
        """
        com_msg = Twist()
        com_msg.linear.x = target_pose_x
        com_msg.linear.y = target_pose_y
        com_msg.linear.z = target_pose_z
        com_msg.angular.z = target_pose_yaw
        return self.kuavo_motion_control.pub_cmd_pose(com_msg)

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
    
    def change_motor_param(self, motor_param:list)->Tuple[bool, str]:
        """
            Change motor param
        """
        try:
            service_name = '/hardware/change_motor_param'
            rospy.wait_for_service(service=service_name, timeout=2.0)
            change_motor_param_service = rospy.ServiceProxy(service_name, changeMotorParam)
            
            # request
            request = changeMotorParamRequest()
            for param in motor_param:
                request.data.append(motorParam(Kp=param.Kp, Kd=param.Kd, id=param.id))
            
            # call service
            response = change_motor_param_service(request)
            if not response.success:
                SDKLogger.error(f"Failed to change motor param: {response.message}")
                return False, response.message
            return True, 'success'
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Service call failed: {e}")
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
        return False, 'failed'
    
    def get_motor_param(self)->Tuple[bool, list, str]:
        try:
            service_name = '/hardware/get_motor_param'
            rospy.wait_for_service(service=service_name, timeout=2.0)
            motor_param_service = rospy.ServiceProxy(service_name, getMotorParam)
            
            # request
            request = getMotorParamRequest()
            response = motor_param_service(request)
            if not response.success:
                SDKLogger.error(f"Failed to get motor param: {response.message}")
                return False, None, response.message
            params = []
            for param in response.data:
                params.append(KuavoMotorParam(Kp=param.Kp, Kd=param.Kd, id=param.id))
            return True, params, 'success'
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Service call failed: {e}")
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
        return False, None, 'failed'

    def control_hand_wrench(self, left_wrench: list, right_wrench: list) -> bool:
        return self.kuavo_arm_control.pub_hand_wrench_cmd(left_wrench, right_wrench)
    
    def enable_base_pitch_limit(self, enable: bool) -> Tuple[bool, str]:
        res_msg = 'failed'
        try:
            service_name = '/humanoid/mpc/enable_base_pitch_limit'
            rospy.wait_for_service(service=service_name, timeout=2.0)
            pitch_limit_service = rospy.ServiceProxy(service_name, SetBool)
            
            # request
            request = SetBoolRequest()
            request.data = enable
            response = pitch_limit_service(request)
            if not response.success:
                SDKLogger.error(f"Failed to enable pitch limit: {response.message}")
                return False, response.message
            return True, 'success'
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call failed: {e}")
            res_msg = str(e)
        except rospy.ROSException as e:
            SDKLogger.error(f"Service call failed: {e}")
            res_msg = str(e)
        except Exception as e:
            SDKLogger.error(f"Service call failed: {e}")
            res_msg = str(e)
        return False, res_msg

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

    msg = footPoseTargetTrajectories()
    msg.timeTrajectory = time_traj
    msg.footIndexTrajectory = foot_idx_traj
    msg.footPoseTrajectory = []

    for i in range(num):
        foot_pose_msg = footPose()
        foot_pose_msg.footPose = foot_traj[i]
        foot_pose_msg.torsoPose = torso_traj[i]
        msg.footPoseTrajectory.append(foot_pose_msg)

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