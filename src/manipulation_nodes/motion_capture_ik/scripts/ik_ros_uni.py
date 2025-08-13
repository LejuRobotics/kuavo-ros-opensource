#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import signal
import rospy
import argparse
import argparse
from std_msgs.msg import Float32, Float32MultiArray, Int32
from sensor_msgs.msg import JointState
from handcontrollerdemorosnode.msg import armPoseWithTimeStamp
from kuavo_msgs.msg import robotHandPosition
from kuavo_msgs.srv import controlLejuClaw, controlLejuClawRequest
from kuavo_msgs.msg import lejuClawCommand
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3
import time
import math
import sys
import struct
import threading
import ctypes
from tools.drake_trans import *

from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeKuavo
from kuavo_msgs.msg import sensorsData
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolRequest, SetBoolResponse

import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    JacobianWrtVariable,
    MathematicalProgram,
    MeshcatVisualizer,
    PiecewisePolynomial,
    PiecewisePose,
    RigidTransform,
    RotationMatrix,
    Solve,
    StartMeshcat,
    Quaternion,
    RollPitchYaw,
    Parser,
)

import threading

# import matplotlib.pyplot as plt

# from scipy.spatial.transform import Rotation as R

from ik.diff_ik import DiffIK, interpolate_pose
from ik.torso_ik import ArmIk

import rospy
from noitom_hi5_hand_udp_python.msg import handRotationEular
from noitom_hi5_hand_udp_python.msg import PoseInfo, PoseInfoList
from kuavo_msgs.msg import JoySticks
from tools.quest3_utils import Quest3ArmInfoTransformer
from kuavo_msgs.msg import ikSolveError, handPose, robotArmQVVD, armHandPose, twoArmHandPose, twoArmHandPoseCmd

from tools.utils import get_package_path, ArmIdx, IkTypeIdx, rotation_matrix_diff_in_angle_axis, limit_value
from tools.drake_trans import rpy_to_matrix
from tools.kalman_filter import KalmanFilter3D
import os


# 定义调度策略常量
SCHED_OTHER = 0
SCHED_FIFO = 1
SCHED_RR = 2
num_arm_joints_var = 14

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

# 定义sched_param结构体
class sched_param(ctypes.Structure):
    _fields_ = [('sched_priority', ctypes.c_int)]

# 加载pthread库
pthread = ctypes.CDLL('libpthread.so.0')

# 定义pthread_setschedparam函数
pthread_setschedparam = pthread.pthread_setschedparam
pthread_setschedparam.argtypes = [ctypes.c_ulong, ctypes.c_int, ctypes.POINTER(sched_param)]
pthread_setschedparam.restype = ctypes.c_int

def set_thread_priority(thread, policy, priority):
    try:
        param = sched_param()
        param.sched_priority = priority
        thread_id = thread.ident
        ret = pthread_setschedparam(thread_id, policy, ctypes.byref(param))
        if ret != 0:
            rospy.logerr(f"Failed to set thread priority! try to run as root.")
    except Exception as e:
        rospy.logerr(f"Failed to set thread priority: {e}")
    
QIANGNAO = "qiangnao"
JODELL = "jodell"
LEJUCLAW = "lejuclaw"
QIANGNAO_TOUCH = "qiangnao_touch"
control_finger_type = 0
control_torso = 0

class IkRos:
    def __init__(self, ik, ctrl_arm_idx=ArmIdx.LEFT, q_limit=None, publish_err=True, use_original_pose=False, end_effector_type="", send_srv=True, predict_gesture=False, hand_reference_mode="thumb_index"):
        self.__start_time = None
        self.__timestamp = None
        self.__ctrl_arm_idx = ctrl_arm_idx
        self.__q_lb = np.array(q_limit[0], dtype=float) if q_limit is not None else None
        self.__q_ub = np.array(q_limit[1], dtype=float) if q_limit is not None else None
        self.__publish_err = publish_err
        self.__use_original_pose = use_original_pose
        self.arm_ik = ik
        self.controller_dt = 0.01  # 10ms
        self.__target_pose = (None, None)  # tuple(pos, quat), quat(x, y, z, w)
        self.__target_pose_right = (None, None)  # tuple(pos, quat), quat(x, y, z, w)
        self.__left_elbow_pos = None # agument ik problem
        self.__right_elbow_pos = None

        self.__recieved_new_target_pose = False
        self.__target_pose = (None, None)
        self.__current_pose = (None, None)
        self.__current_pose_right = (None, None)
        self.__joint_states = None  # left and right arm joint states
        self.__last_mc_time = None
        self.__time_cost_mc_glove = 0.0
        self.joySticks_data = None
        self.hand_finger_data = None
        self.__as_mc_ik = True  # 默认作为遥操作的IK，精度较低，速度要求较高
        self.__send_srv = send_srv
        self.__freeze_finger = False
        self.__button_y_last = False
        self.__arm_dof = num_arm_joints_var
        self.__single_arm_dof = self.__arm_dof//2
        self.trigger_reset_mode = False

        # 检查是否是半身模式
        self.only_half_up_body = False
        if rospy.has_param('/only_half_up_body'):
            self.only_half_up_body = rospy.get_param('/only_half_up_body')

        self.use_arm_collision = rospy.get_param('~use_arm_collision', False)
        # 添加服务
        self.arm_mode_service = rospy.Service('/quest3/set_arm_mode_changing', Trigger, self.set_arm_mode_changing_callback)

        # self.hand_pub_timer = rospy.Timer(rospy.Duration(0.001), self.hand_finger_data_process)


        model_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
        self.quest3_arm_info_transformer = Quest3ArmInfoTransformer(model_path, predict_gesture=predict_gesture, hand_reference_mode=hand_reference_mode)
        self.quest3_arm_info_transformer.control_torso = control_torso
        initial_state = np.array([0, 0, 0, 0, 0, 0])  # 初始状态 [x, y, z, vx, vy, vz]
        initial_covariance = np.eye(6)  # 初始协方差矩阵
        process_noise = np.eye(6) * 0.001  # 过程噪声协方差矩阵
        measurement_noise = np.eye(3) * 1.1  # 测量噪声协方差矩阵

        initial_state[0:3] = self.arm_ik.left_hand_pose(self.arm_ik.q0())[0]
        self.kf_left = KalmanFilter3D(initial_state, initial_covariance, process_noise, measurement_noise,dt = 1)
        initial_state[0:3] = self.arm_ik.right_hand_pose(self.arm_ik.q0())[0]
        self.kf_right = KalmanFilter3D(initial_state, initial_covariance, process_noise, measurement_noise,dt = 1)
        self.external_q0 = None
        # TO-DO(matthew): subscribe to joint states
        # update joint states
        self.joint_sub = rospy.Subscriber(
            "/robot_arm_q_v_tau", robotArmQVVD, self.kuavo_joint_states_callback, queue_size=10
        )
        self.quest_bone_poses_sub = rospy.Subscriber(
            "/leju_quest_bone_poses", PoseInfoList, self.quest_bone_poses_callback, queue_size=3
        )

        self.joySticks_sub = rospy.Subscriber(
            "/quest_joystick_data",
            JoySticks,
            self.joySticks_data_callback,
            queue_size=3
        )

        self.ik_cmd_sub = rospy.Subscriber(
            "/ik/two_arm_hand_pose_cmd", twoArmHandPoseCmd, self.two_arm_hand_pose_target_callback, queue_size=10
        )

        self.sensor_data_raw_sub = rospy.Subscriber(
            "/sensors_data_raw", sensorsData, self.sensor_data_raw_callback, queue_size=1
        )
        self.arm_mode_changing = False
        # 检测到碰撞后，由外部控制手臂
        self.collision_check_control = False
        self.sensor_data_raw = None
        self.maxSpeed = rospy.get_param("/arm_move_spd_half_up_body", 0.21)
        self.threshold_arm_diff_half_up_body = rospy.get_param("/threshold_arm_diff_half_up_body", 0.2)
        self._interp_time_last = rospy.Time.now().to_sec()


        if self.use_arm_collision:
            self.pub = rospy.Publisher("/arm_collision/kuavo_arm_traj", JointState, queue_size=10)
        else:
            self.pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
        self.pub_origin_joint = rospy.Publisher("/kuavo_arm_traj_origin", Float32MultiArray, queue_size=10)
        self.pub_filtered_joint = rospy.Publisher("/kuavo_arm_traj_filtered", Float32MultiArray, queue_size=10)
        self.pub_real_arm_hand_pose = rospy.Publisher("/drake_ik/real_arm_hand_pose", twoArmHandPose, queue_size=10)
        self.pub_time_cost = rospy.Publisher(
            "/drake_ik/time_cost/ik", Float32, queue_size=10
        )
        self.pub_ik_solve_error = rospy.Publisher(
            "/drake_ik/ik_solve_error", ikSolveError, queue_size=10
        )
        self.control_robot_hand_position_pub = rospy.Publisher(
            "control_robot_hand_position", robotHandPosition, queue_size=10
        )
        self.pub_ik_solved_eef_pose = rospy.Publisher(
            "/drake_ik/eef_pose", twoArmHandPose, queue_size=10
        )
        self.pub_ik_input_pos = rospy.Publisher(
            "/drake_ik/input_pos", Float32MultiArray, queue_size=10
        )
        self.pub_q0_tmp = rospy.Publisher(
            "/drake_ik/q0_tmp", Float32MultiArray, queue_size=10
        )
        self.leju_claw_command_pub = rospy.Publisher(
            "leju_claw_command", lejuClawCommand, queue_size=10
        )
        
        # 添加可视化marker发布器
        self.ik_visualization_pub = rospy.Publisher(
            "/ik_visualization_markers", MarkerArray, queue_size=10
        )
        
        try:
            end_effector_mapping = {
                QIANGNAO: QIANGNAO,
                JODELL: JODELL,
                LEJUCLAW: LEJUCLAW,
                QIANGNAO_TOUCH:QIANGNAO_TOUCH
            }
            if end_effector_type in end_effector_mapping:
                self.end_effector_type = end_effector_mapping[end_effector_type]
            else:
                self.end_effector_type = QIANGNAO
        except Exception as e:
            print(f"get end effector type error: {e}, use default qiangnao")
            self.end_effector_type = QIANGNAO
        print(f"\033[93m--------------------------------------------------\033[0m")        
        print(f"\033[93m- End effector type: {self.end_effector_type} \033[0m")
        print(f"\033[93m--------------------------------------------------\033[0m")        

        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)
        self.stop_event = threading.Event()
        self.ik_thread = threading.Thread(target=self.ik_controller_thread)
        self.ik_thread.start()
        set_thread_priority(self.ik_thread, int(SCHED_FIFO), 50)

        # 保存初始关节角度
        self.initial_q_first = None
        
        # 订阅手臂模式topic
        self.arm_mode_sub = rospy.Subscriber('/quest3/triger_arm_mode', Int32, self.arm_mode_callback)

        self.run()
        self.ik_thread.join()
   
    def run(self):
        while rospy.is_shutdown() is False:
            rospy.spin()

    def shutdown(self, signal, frame):
        rospy.loginfo("Shutting down...")
        self.stop_event.set()
        rospy.signal_shutdown("Shutdown")
        rospy.loginfo("Shutdown complete.")

    @staticmethod
    def drake_pose_to_tuple(drake_pose):
        quat = drake_pose.rotation().ToQuaternion()
        quat_vec = np.array([quat.x(), quat.y(), quat.z(), quat.w()])
        return (drake_pose.translation(), quat_vec)

    def get_two_arm_pose(self, q_arm):
        # q0 = self.arm_ik.q0()
        # q0[7:] = q_arm
        left_hand_pose = self.arm_ik.left_hand_pose(q_arm)
        right_hand_pose = self.arm_ik.right_hand_pose(q_arm)
        # print(f"left_hand_pose: {left_hand_pose}")
        quat_left = rpy_to_quaternion(left_hand_pose[1][0], left_hand_pose[1][1], left_hand_pose[1][2])
        quat_right = rpy_to_quaternion(right_hand_pose[1][0], right_hand_pose[1][1], right_hand_pose[1][2])
        return (left_hand_pose[0], quat_left), (right_hand_pose[0], quat_right)

    def current_pose(self):
        return self.__current_pose

    def limit_angle(self, q):
        if self.__q_lb is not None and self.__q_ub is not None:
            q_limited = np.zeros(self.__arm_dof)
            for i in range(self.__arm_dof):
                q_limited[i] = max(self.__q_lb[i], min(q[i], self.__q_ub[i]))
            return q_limited
        else:
            return q

    def limit_angle_by_velocity(self, q_last, q_now, vel_limit=50.0):
        """
        limit the angle change by velocity, default 50 deg/s
        """
        size = len(q_now)
        q_limited = q_now.copy()
        agl_limit = self.controller_dt * vel_limit * np.pi / 180.0  # deg/s to rad/s
        for i in range(size):
            q_limited[i] = max(q_last[i] - agl_limit, min(q_now[i], q_last[i] + agl_limit))
        return q_limited

    @staticmethod
    def change_arm_ctrl_mode(mode: int):
        service_name = "/change_arm_ctrl_mode"
        try:
            rospy.wait_for_service(service_name)
            changeHandTrackingMode_srv = rospy.ServiceProxy(
                service_name, changeArmCtrlMode
            )
            changeHandTrackingMode_srv(mode)
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")

    @staticmethod
    def change_arm_ctrl_mode4kuavo(mode: bool):
        service_name = "/change_arm_ctrl_mode"
        try:
            rospy.wait_for_service(service_name)
            changeHandTrackingMode_srv = rospy.ServiceProxy(
                service_name, changeArmCtrlModeKuavo
            )
            changeHandTrackingMode_srv(mode)
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")

    @staticmethod
    def control_lejuclaw(pos:list):
        # print(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> control_lejuclaw: {pos}")
        service_name = "/control_robot_leju_claw"
        try:
            rospy.wait_for_service("/control_robot_leju_claw", timeout=1)
            control_lejucalw_srv = rospy.ServiceProxy(
                service_name, controlLejuClaw
            )
            req = controlLejuClawRequest()
            req.data.name = ['left_claw', 'right_claw']
            req.data.position = pos
            req.data.velocity = [90, 90]
            # print(f">>>>>>>>>>>>>>>> control_lejucalw_srv: {req}")
            control_lejucalw_srv(req)
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")
        except Exception as e:
            rospy.logerr(f"Error: {e}")   

    def pub_leju_claw_command(self, pos:list):
        msg = lejuClawCommand()
        msg.header.stamp = rospy.Time.now()
        msg.data.name = ['left_claw', 'right_claw']
        msg.data.position = pos
        msg.data.velocity = [90, 90]
        self.leju_claw_command_pub.publish(msg)

    def pub_solved_arm_eef_pose(self, q_robot, current_pose, current_pose_right):
        msg = twoArmHandPose()
        msg.header.frame_id = "torso"
        msg.header.stamp = rospy.Time.now()
        msg.left_pose.pos_xyz = current_pose[0]
        msg.left_pose.quat_xyzw = current_pose[1]
        msg.left_pose.joint_angles = q_robot[-self.__arm_dof:-self.__single_arm_dof]
        msg.right_pose.pos_xyz = current_pose_right[0]
        msg.right_pose.quat_xyzw = current_pose_right[1]
        msg.right_pose.joint_angles = q_robot[-self.__single_arm_dof:]
        self.pub_ik_solved_eef_pose.publish(msg)

    def ik_controller_thread(self):
        rate = rospy.Rate(1 / self.controller_dt)
        traj_X_G = None
        traj_X_G_right = None
        # print("[ik]:waiting for first joint states")
        # while self.__joint_states is None:
        #     rate.sleep()
        # print("[ik]:first joint states recieved.")
        print("[ik]: Waiting for first eef target pose")
        while not self.__recieved_new_target_pose:
            if self.stop_event.is_set():
                print("[ik]: Stop event is set, exit.")
                return
            rate.sleep()
        print("[ik]: First eef target pose recieved.")
        if self.__as_mc_ik:
            print("[ik]: Waiting for OK-guesture(hold on for 1-2 seconds) to start teleoperation...")
            while not self.quest3_arm_info_transformer.is_runing:
                self.hand_finger_data_process(0)
                if self.stop_event.is_set():
                    print("[ik]: Stop event is set, exit.")
                    return
                if(self.quest3_arm_info_transformer.check_if_vr_error()):
                    sys.stdout.write("\r\033[91mDetected VR ERROR!!! Please restart VR app in quest3 or check the battery level of the joystick!!!\033[0m")
                rate.sleep()
            print("[ik]: OK-guesture recieved!!!")

        if self.__send_srv:
            print("[ik]: Send start service signal to robot, wait for response.")
            self.change_arm_ctrl_mode(2)
            # self.change_arm_ctrl_mode4kuavo(True)
            print("\033[92m[ik]: Recied start signal response, Start teleoperation.\033[0m")

        self.arm_mode_changing = True
        if self.__as_mc_ik:
            print("[ik]: If you want to stop teleoperation, please make a Shot-guesture(hold on for 1-2 seconds).")
        q_last = self.arm_ik.q0() if self.external_q0 is None else self.external_q0
        pre_q_first = q_last.copy()
        # q_last[-14:] = self.__joint_states  # two arm joint states
        # q_last[7:14] = [0.1084,  0.0478 , 0.1954 ,-0.0801 , 0.1966 ,-0.5861 , 0.0755]
        q_now = q_last
        t_ctrl = 0.0
        print(f"IK Type: {self.arm_ik.type()}")
        run_count, fail_count = 0, 0
        sum_time_cost = 0.0
        arm_q_filtered = [0.0] * self.__arm_dof
        is_runing = False
        while not rospy.is_shutdown():
            self.hand_finger_data_process(0)
            # print(f"q_now: {q_now}")
            is_runing_last = is_runing
            is_runing = self.quest3_arm_info_transformer.is_runing if self.__as_mc_ik else True
            self.__current_pose, self.__current_pose_right = self.get_two_arm_pose(q_last)
            self.pub_solved_arm_eef_pose(q_last, self.__current_pose, self.__current_pose_right)
            if self.trigger_reset_mode:
                self.__target_pose = (None, None)
                self.__current_pose = (None, None)
                self.__target_pose_right = (None, None)
                self.__current_pose_right = (None, None)
                q_last = pre_q_first.copy()
                self.trigger_reset_mode = False

            if(self.__as_mc_ik and self.quest3_arm_info_transformer.check_if_vr_error()):
                rate.sleep()
                print("\033[91mDetected VR ERROR!!! Please restart VR app in quest3 or check the battery level of the joystick!!!\.\033[0m")
                continue
            elif(self.__as_mc_ik and (not is_runing)):
                rate.sleep()
                sys.stdout.write("\rStatus0: {}, is target far?: {}".format("RUNING" if is_runing else "STOPED", self.judge_target_is_far()))
                continue
            
            if(not is_runing_last and is_runing):
                self.arm_mode_changing = True
            if self.__target_pose[0] is None or self.__target_pose_right[0] is None or \
                self.__current_pose[0] is None or self.__current_pose_right[0] is None:
                rate.sleep()
                continue
            if self.arm_ik.type().name() == IkTypeIdx.TorsoIK.name():
                l_hand_pose, l_hand_RPY = None, None
                r_hand_pose, r_hand_RPY = None, None
                l_elbow_pos, r_elbow_pos = None, None
                left_shoulder_rpy_in_robot, right_shoulder_rpy_in_robot = None, None
                if self.__target_pose[0] is not None and (self.__ctrl_arm_idx == ArmIdx.BOTH
                                                          or self.__ctrl_arm_idx == ArmIdx.LEFT):
                    l_hand_pose_recorded = []
                    l_hand_pose, l_hand_quat = self.__target_pose
                    l_hand_pose_recorded.append(l_hand_pose)
                    l_hand_pose = self.kf_left.filter(l_hand_pose)
                    l_hand_pose_recorded.append(l_hand_pose)
                    self.pub_ik_input_pos.publish(Float32MultiArray(data=np.asarray(l_hand_pose_recorded).flatten()))
                    l_hand_RPY = quaternion_to_RPY(l_hand_quat)
                    l_elbow_pos = self.__left_elbow_pos
                    if l_elbow_pos is not None:
                        # print(f"l_elbow_pos: {l_elbow_pos}")
                        l_elbow_pos[0] = 0.0 if l_elbow_pos[0] < 0.0 else l_elbow_pos[0]
                    left_shoulder_rpy_in_robot = self.quest3_arm_info_transformer.left_shoulder_rpy_in_robot
                if self.__target_pose_right[0] is not None and (self.__ctrl_arm_idx == ArmIdx.BOTH
                                                                or self.__ctrl_arm_idx == ArmIdx.RIGHT):
                    r_hand_pose, r_hand_quat = self.__target_pose_right
                    r_hand_pose = self.kf_right.filter(r_hand_pose)
                    r_hand_RPY = quaternion_to_RPY(r_hand_quat)
                    r_elbow_pos = self.__right_elbow_pos
                    if r_elbow_pos is not None:
                        r_elbow_pos[0] = 0.0 if r_elbow_pos[0] < 0.0 else r_elbow_pos[0]
                    right_shoulder_rpy_in_robot = self.quest3_arm_info_transformer.right_shoulder_rpy_in_robot
                time_0 = time.time()
                # 通过限制初值，避免迭代到不可解的区域
                q0_tmp = q_last.copy()
                threashold = -3.0
                q0_tmp[-self.__arm_dof] += 0.5 if q0_tmp[-self.__arm_dof] < threashold else 0.0
                q0_tmp[-self.__single_arm_dof] += 0.5 if q0_tmp[-self.__single_arm_dof] < threashold else 0.0
                if self.__arm_dof == 8: # 针对roban的调整
                    if q0_tmp[-self.__arm_dof + 2] > 0.5:
                        q0_tmp[-self.__arm_dof + 2] = 0.5
                    if q0_tmp[-self.__arm_dof + 2] < -0.5:
                        q0_tmp[-self.__arm_dof + 2] = -0.5
                    if q0_tmp[-self.__single_arm_dof + 2] > 0.5:
                        q0_tmp[-self.__single_arm_dof + 2] = 0.5
                    if q0_tmp[-self.__single_arm_dof + 2] < -0.5:
                        q0_tmp[-self.__single_arm_dof + 2] = -0.5
                    if q0_tmp[-self.__single_arm_dof] > 0.0:
                        q0_tmp[-self.__single_arm_dof] = 0.0
                    if q0_tmp[0] > 0.0:
                        q0_tmp[0] = 0.0

                
                # q0_tmp[-self.__single_arm_dof + 3] = -0.5
                # q0_tmp[-self.__arm_dof+3] = -0.5
                
                # # 计算肘部角度并设置到q0_tmp中
                # # 获取肩膀位置
                # left_shoulder_pos = self.get_shoulder_position(q0_tmp, "left")
                # right_shoulder_pos = self.get_shoulder_position(q0_tmp, "right")
                
                # # 计算左臂肘部角度
                # if left_shoulder_pos is not None and l_elbow_pos is not None and l_hand_pose is not None:
                #     left_elbow_angle = self.calculate_elbow_angle(left_shoulder_pos, l_elbow_pos, l_hand_pose)
                #     if left_elbow_angle is not None:
                #         # 左臂肘部关节通常是第4个关节（索引3）
                #         elbow_joint_idx = 3
                #         q0_tmp[-self.__arm_dof + elbow_joint_idx] = -left_elbow_angle
                #         print(f"左臂肘部角度: {left_elbow_angle * 180.0 / np.pi:.2f}°")
                
                # # 计算右臂肘部角度
                # if right_shoulder_pos is not None and r_elbow_pos is not None and r_hand_pose is not None:
                #     right_elbow_angle = self.calculate_elbow_angle(right_shoulder_pos, r_elbow_pos, r_hand_pose)
                #     if right_elbow_angle is not None:
                #         # 右臂肘部关节通常是第4个关节（索引3）
                #         elbow_joint_idx = 3
                #         q0_tmp[-self.__single_arm_dof + elbow_joint_idx] = -right_elbow_angle
                #         print(f"右臂肘部角度: {right_elbow_angle * 180.0 / np.pi:.2f}°")
                
                # # 发布q0_tmp
                # q0_tmp_msg = Float32MultiArray()
                # q0_tmp_msg.data = q0_tmp * 180.0 / np.pi  # 转换为角度
                # self.pub_q0_tmp.publish(q0_tmp_msg)
                
                
                q_now = arm_ik.computeIK(
                    q0_tmp, l_hand_pose, r_hand_pose, l_hand_RPY, r_hand_RPY, l_elbow_pos, r_elbow_pos, left_shoulder_rpy_in_robot, right_shoulder_rpy_in_robot
                )
                time_cost = time.time() - time_0
                if time_cost >= 10.0:
                    print(f"\033[91m The time-cost of ik is {1e3*time_cost:.2f} ms !!!\033[0m")
                self.pub_time_cost.publish(Float32(1e3 * time_cost))
                if q_now is not None:
                    msg = Float32MultiArray()
                    msg.data = q_now[-self.__arm_dof:] * 180.0 / np.pi
                    self.pub_origin_joint.publish(msg)
                    arm_q_filtered = self.limit_angle(q_now[-self.__arm_dof:])
                    arm_q_filtered = self.limit_angle_by_velocity(q_last[-self.__arm_dof:], arm_q_filtered, vel_limit=720.0)
                    msg.data = arm_q_filtered * 180.0 / np.pi
                    self.pub_filtered_joint.publish(msg)
                    self.publish_joint_states(q_now=arm_q_filtered, q_last=q_last)
                    # self.quest3_arm_info_transformer.pub_whole_body_joint_state_msg(arm_q_filtered, map_finger=self.__as_mc_ik)
                    q_last[:self.__single_arm_dof] = q_now[:self.__single_arm_dof]
                    q_last[-self.__arm_dof:] = arm_q_filtered
                else:
                    fail_count += 1
                    # print(f"""\nq_last:{q_last}\n l_hand_pose:{l_hand_pose}\n 
                    #       r_hand_pose:{r_hand_pose}\n 
                    #       l_hand_RPY:{l_hand_RPY}\n 
                    #       r_hand_RPY:{r_hand_RPY}\n 
                    #       l_elbow_pos:{l_elbow_pos}\n 
                    #       r_elbow_pos:{r_elbow_pos}\n 
                    #       left_shoulder_rpy_in_robot:{left_shoulder_rpy_in_robot}\n 
                    #       right_shoulder_rpy_in_robot:{right_shoulder_rpy_in_robot}\n""")

                run_count += 1
                success_rate = 100 * (1.0 - fail_count / float(run_count))
                sum_time_cost += time_cost
                if run_count % 10 == 0:
                    sys.stdout.write(
                        "\rStatus1: {}, IK success rate: {:.1f}%, avg time-cost: {:.1f} ms, is target far?: {}".format(
                            "RUNING" if is_runing else "STOPED", success_rate, 1e3 * sum_time_cost/run_count, self.judge_target_is_far()
                        )
                    )

            if self.__publish_err and q_now is not None:
                msg_pose_err = ikSolveError()
                msg_pose_err.ik_type = self.arm_ik.type().name()
                pose_left = arm_ik.left_hand_pose(q_now)
                pose_right = arm_ik.right_hand_pose(q_now)
                if self.__target_pose[0] is not None:
                    pose_left_des = (self.__target_pose[0], quaternion_to_RPY(self.__target_pose[1]))
                    msg_pose_err.left_pose_error = self.generate_ik_solve_error_msg(pose_res=pose_left, pose_des=pose_left_des)
                if self.__target_pose_right[0] is not None:
                    pose_right_des = (self.__target_pose_right[0], quaternion_to_RPY(self.__target_pose_right[1]))
                    msg_pose_err.right_pose_error = self.generate_ik_solve_error_msg(pose_res=pose_right, pose_des=pose_right_des)
                self.pub_ik_solve_error.publish(msg_pose_err)
            rate.sleep()

    def publish_joint_states(self, q_now, q_last):
        arm_agl_limited = self.limit_angle(q_now[-self.__arm_dof:])
        msg = JointState()
        msg.name = ["arm_joint_" + str(i) for i in range(1, self.__arm_dof+1)]
        msg.header.stamp = rospy.Time.now()
        
        if self.only_half_up_body and self.sensor_data_raw is None:
            print(f"[ik_ros_uni]: sensor_data_raw is None")
            return

        if self.only_half_up_body and self.arm_mode_changing:
            # 获取当前关节角度
            arm_current_state = np.array(self.sensor_data_raw.joint_data.joint_q[12:26]).copy()
            
            # 计算状态差
            delta_state = np.array(arm_agl_limited) - np.array(arm_current_state)
            total_distance = np.linalg.norm(delta_state)
            
            # 如果距离太小，直接使用目标状态
            if total_distance < self.threshold_arm_diff_half_up_body:
                arm_agl_interpolated = arm_agl_limited
                self.arm_mode_changing = False
            else:
                
                max_move = self.maxSpeed
            
                scale = np.clip(max_move / total_distance, 0, 1)
                arm_agl_interpolated = arm_current_state + delta_state * scale
            
            msg.position = 180.0 / np.pi * np.array(arm_agl_interpolated)
        else:
            # 非插值模式下直接使用目标状态
            msg.position = 180.0 / np.pi * np.array(arm_agl_limited)
        
        self.pub.publish(msg)

    def kuavo_joint_states_callback(self, joint_states_msg):
        # 手臂状态正解
        self.__joint_states = np.array(joint_states_msg.q)
        q_drake = np.zeros(7+self.__arm_dof)
        q_drake[0] = 1.0
        q_drake[7:] = self.__joint_states
        left_hand_pose = self.arm_ik.left_hand_pose(q_drake)
        right_hand_pose = self.arm_ik.right_hand_pose(q_drake)
        arm_hand_pose_msg = twoArmHandPose()
        arm_hand_pose_msg.header.frame_id = "torso"
        arm_hand_pose_msg.header.stamp = rospy.Time.now()
        arm_hand_pose_msg.left_pose.pos_xyz = left_hand_pose[0]
        r, p, y = left_hand_pose[1]
        arm_hand_pose_msg.left_pose.quat_xyzw = rpy_to_quaternion(r, p, y)
        arm_hand_pose_msg.left_pose.joint_angles = self.__joint_states[:self.__single_arm_dof]
        arm_hand_pose_msg.right_pose.pos_xyz = right_hand_pose[0]
        r, p, y = right_hand_pose[1]
        arm_hand_pose_msg.right_pose.quat_xyzw = rpy_to_quaternion(r, p, y)
        arm_hand_pose_msg.right_pose.joint_angles = self.__joint_states[-self.__single_arm_dof:]
        self.pub_real_arm_hand_pose.publish(arm_hand_pose_msg)
        # print(f"received joint_states: {self.__joint_states}")

    def quest_bone_poses_callback(self, quest_bone_poses_msg):
        self.quest3_arm_info_transformer.read_msg(quest_bone_poses_msg)
        pose, self.__left_elbow_pos = self.quest3_arm_info_transformer.get_hand_pose("Left")
        self.__target_pose = pose
        pose, self.__right_elbow_pos = self.quest3_arm_info_transformer.get_hand_pose("Right")
        self.__target_pose_right = pose
        if self.__recieved_new_target_pose == False:
            self.__recieved_new_target_pose = True
        left_finger_joints = self.quest3_arm_info_transformer.get_finger_joints("Left")
        right_finger_joints = self.quest3_arm_info_transformer.get_finger_joints("Right")
        self.hand_finger_data = [left_finger_joints, right_finger_joints]
        # self.pub_robot_end_hand(left_finger_joints, right_finger_joints)

    def two_arm_hand_pose_target_callback(self, msg_ori):
        msg = msg_ori.hand_poses
        if msg_ori.use_custom_ik_param:
            self.external_q0 = list(msg.left_pose.joint_angles) + list(msg.right_pose.joint_angles)
        self.__target_pose = (msg.left_pose.pos_xyz, msg.left_pose.quat_xyzw)
        self.__target_pose_right = (msg.right_pose.pos_xyz, msg.right_pose.quat_xyzw)
        if(abs(msg.left_pose.elbow_pos_xyz[0]) <= 1e-5 
            and abs(msg.left_pose.elbow_pos_xyz[1]) <= 1e-5 
            and abs(msg.left_pose.elbow_pos_xyz[2]) <= 1e-5):  # 都为0，则不控制elbow
            self.__left_elbow_pos = None      
        else:
            self.__left_elbow_pos = np.array(msg.left_pose.elbow_pos_xyz)
        if(abs(msg.right_pose.elbow_pos_xyz[0]) <= 1e-5 
            and abs(msg.right_pose.elbow_pos_xyz[1]) <= 1e-5 
            and abs(msg.right_pose.elbow_pos_xyz[2]) <= 1e-5):
            self.__right_elbow_pos = None
        else:
            self.__right_elbow_pos = np.array(msg.right_pose.elbow_pos_xyz)
        
        if self.__recieved_new_target_pose == False:
            self.__recieved_new_target_pose = True
        if self.__as_mc_ik:
            self.__as_mc_ik = False
            self.arm_ik.set_as_mc_ik(self.__as_mc_ik)

    def joySticks_data_callback(self, msg):
        self.quest3_arm_info_transformer.read_joySticks_msg(msg)
        self.joySticks_data = msg

    @staticmethod
    def vector3_to_milliseconds(x, y, z):
        # Reconstruct the bytes from the Vector3 components
        # x, y, z are assumed to be float but should contain integer values for this purpose
        byte0 = int(x) & 0xFF
        byte1 = int(y) >> 8 & 0xFF
        byte2 = int(y) & 0xFF
        byte3 = int(z) & 0xFF

        # Reverse the order of the bytes
        reversed_bytes = [byte3, byte2, byte1, byte0]

        # Pack the reversed bytes back into a single integer
        # '<' denotes little-endian byte order, 'I' denotes an unsigned int
        milliseconds_int = struct.unpack("<I", bytes(reversed_bytes))[0]
        return milliseconds_int
    
    def get_shoulder_position(self, q, side="left"):
        """
        获取肩膀位置
        Args:
            q: 关节角度
            side: "left" 或 "right"
        Returns:
            np.array: 肩膀位置 [x, y, z]
        """
        try:
            self.arm_ik._ArmIk__plant.SetPositions(self.arm_ik._ArmIk__plant_context, q)
            if side.lower() == "left":
                shoulder_frame = self.arm_ik._ArmIk__plant.GetFrameByName(self.arm_ik.shoulder_frame_names[0])
            else:
                shoulder_frame = self.arm_ik._ArmIk__plant.GetFrameByName(self.arm_ik.shoulder_frame_names[1])
            
            shoulder_pose = shoulder_frame.CalcPoseInWorld(self.arm_ik._ArmIk__plant_context)
            return shoulder_pose.translation()
        except Exception as e:
            print(f"获取肩膀位置失败: {e}")
            return None

    def calculate_elbow_angle(self, shoulder_pos, elbow_pos, hand_pos):
        """
        计算肘部关节的夹角
        Args:
            shoulder_pos: 肩膀位置 [x, y, z]
            elbow_pos: 肘部位置 [x, y, z]
            hand_pos: 手部位置 [x, y, z]
        Returns:
            float: 肘部关节角度（弧度）
        """
        if shoulder_pos is None or elbow_pos is None or hand_pos is None:
            return None
            
        # 计算向量
        vec_shoulder_to_elbow = np.array(elbow_pos) - np.array(shoulder_pos)
        vec_elbow_to_hand = np.array(hand_pos) - np.array(elbow_pos)
        
        # 计算向量长度
        len_shoulder_elbow = np.linalg.norm(vec_shoulder_to_elbow)
        len_elbow_hand = np.linalg.norm(vec_elbow_to_hand)
        
        # 避免除零错误
        if len_shoulder_elbow < 1e-6 or len_elbow_hand < 1e-6:
            return None
            
        # 计算夹角（弧度）
        cos_angle = np.dot(vec_shoulder_to_elbow, vec_elbow_to_hand) / (len_shoulder_elbow * len_elbow_hand)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)  # 限制在[-1, 1]范围内
        angle = np.arccos(cos_angle)
        
        return angle

    def generate_ik_solve_error_msg(self, pose_res, pose_des):
        rad2deg = 180.0/np.pi      
        hand_pose_err = handPose()
        pos_res, rpy_res = pose_res
        pos_des, rpy_des = pose_des
        hand_pose_err.x = pos_res[0] - pos_des[0]
        hand_pose_err.y = pos_res[1] - pos_des[1]
        hand_pose_err.z = pos_res[2] - pos_des[2]
        rpy_err = compute_rpy_error(rpy_res, rpy_des)
        hand_pose_err.roll = rad2deg * rpy_err[0]
        hand_pose_err.pitch = rad2deg * rpy_err[1]
        hand_pose_err.yaw = rad2deg * rpy_err[2]
        return hand_pose_err


    def control_to_pos(self, diff_ik: DiffIK, q_now, traj, ctrl_arm_idx=ArmIdx.LEFT, dt=0.01, pub_joint=None):
        """
        ctrl_arm_idx: Note: ONLY support ArmIdx.LEFT or ArmIdx.RIGHT
        """
        if ctrl_arm_idx.name() == ArmIdx.BOTH.name():
            print(f"\033[91mControl both arms is not supported.\033[0m")
            return None
        t_duration = traj.get_segment_times()[1] - traj.get_segment_times()[0]
        t_sim = traj.get_segment_times()[0]
        q0 = q_now
        last_q = q_now
        # diff_ik.start_recording()
        traj_V_G = traj.MakeDerivative() if traj is not None else None

        t_play = 0.0
        last_norm = 100.0
        last_v = np.zeros(self.__single_arm_dof)
        while t_sim < t_duration:
            pose = arm_ik.left_hand_pose(last_q) if ctrl_arm_idx.name() == ArmIdx.LEFT.name() else arm_ik.right_hand_pose(last_q)
            pos_target, quat_target = self.__target_pose if ctrl_arm_idx.name() == ArmIdx.LEFT.name() else self.__target_pose_right

            is_close, norm = self.check_if_close(pose, (pos_target, quaternion_to_RPY(quat_target)))
            if is_close:
                break

            t_sim += dt
            time_0 = time.time()
            V_G_vec = np.array(traj_V_G.value(t_sim))[:, 0] if traj is not None else None

            # 仅控制单臂
            v_max = 1.0
            vd_max = 5.0
            v0 = np.zeros(self.__single_arm_dof)
            if ctrl_arm_idx.name() == ArmIdx.LEFT.name():
                v0 = diff_ik.solve_left_hand(last_q, last_v, V_G_vec, dt, v_max, vd_max)
                q0[self.__single_arm_dof:self.__arm_dof] += dt*v0
            if ctrl_arm_idx.name() == ArmIdx.RIGHT.name():
                v0 = diff_ik.solve_right_hand(last_q, last_v, V_G_vec, dt, v_max, vd_max)
                q0[-self.__single_arm_dof:] += dt*v0
            time_cost = time.time() - time_0
            # animate trajectory
            diff_ik.visualize_animation([last_q, q0], t_play, dt)
            last_q = q0
            last_v = v0
            t_play = t_play + dt
            # if self.pub is not None:
            #     self.publish_joint_states(q_now=q_now, q_last=None)
            if last_norm < norm:
                break
            last_norm = norm
        return last_q

    def check_if_close(self, pose_res, pose_des, threshold_pos=0.002, threshold_theta=3.0*np.pi/180.0):
        """
        check if the pose_res is close to pose_des
        位置误差: 默认0.002m
        姿态误差: 默认3.0deg
        Returns:
            bool: True if the pose_res is close to pose_des, False otherwise
            float: the norm of the position error
        """
        norm = np.linalg.norm(pose_res[0] - pose_des[0])
        # print(f"pose_res: {pose_res}")
        # print(f"pose_res: {pose_des}")
        mat_res, mat_des = rpy_to_matrix(pose_res[1]), rpy_to_matrix(pose_des[1])

        delta_theta, _ = rotation_matrix_diff_in_angle_axis(mat_res, mat_des)
        if norm > threshold_pos:
            return False, norm
        if delta_theta > threshold_theta:
            return False, norm
        return True, norm

    def judge_target_is_far(self, threshold=0.4):
        """
        If target is far, return True, else return False.
        """
        if self.__target_pose[0] is None or self.__target_pose_right[0] is None:
            return False
        if self.__current_pose[0] is None or self.__current_pose_right[0] is None:
            return False
        pos_left, _ = self.__current_pose
        pos_right, _ = self.__current_pose_right
        pos_target_left, _ = self.__target_pose
        pos_target_right, _ = self.__target_pose_right
        dist_left = np.linalg.norm(pos_left - pos_target_left)
        dist_right = np.linalg.norm(pos_right - pos_target_right)
        if dist_left > threshold and self.__ctrl_arm_idx.name() != ArmIdx.RIGHT.name():
            # print(f"\033[91mLeft arm is far from target, distance: {dist_left:.3f} m.\033[0m")
            return True
        if dist_right > threshold and self.__ctrl_arm_idx.name() != ArmIdx.LEFT.name():
            # print(f"\033[91mRight arm is far from target, distance: {dist_right:.3f} m.\033[0m")
            return True
        return False

    @staticmethod
    def isJoyPushed(joySticks_data):
        pushed = False
        pushed |= (joySticks_data.left_trigger > 0.0)
        pushed |= (joySticks_data.right_trigger > 0.0)
        pushed |= (joySticks_data.left_grip > 0.0)
        pushed |= (joySticks_data.right_grip > 0.0)
        pushed |= joySticks_data.left_first_button_touched
        pushed |= joySticks_data.right_first_button_touched
        return pushed

    def hand_finger_data_process(self, event):
        # if(self.isJoyPushed(self.joySticks_data)):
        if(not self.quest3_arm_info_transformer.is_hand_tracking):
            # print(f"\033[91mJoystick is pushed, stop control.\033[0m")
            self.pub_robot_end_hand(joyStick_data=self.joySticks_data)            
        else:
            # print(f"\033[91mJoystick is not pushed, continue control.\033[0m")
            self.pub_robot_end_hand(hand_finger_data=self.hand_finger_data)


    def pub_robot_end_hand(self, joyStick_data=None, hand_finger_data = None):
        global control_finger_type
        left_hand_position = [0 for i in range(6)]
        right_hand_position = [0 for i in range(6)]
        robot_hand_position = robotHandPosition()
        robot_hand_position.header.stamp = rospy.Time.now()
        if self.end_effector_type == QIANGNAO or self.end_effector_type == QIANGNAO_TOUCH:
            if joyStick_data is not None:
                if joyStick_data.left_second_button_pressed and self.__button_y_last is False:
                    print(f"\033[91mButton Y is pressed.\033[0m")
                    self.__freeze_finger = not self.__freeze_finger
                self.__button_y_last = joyStick_data.left_second_button_pressed
                if self.__freeze_finger is True:
                    # print(f"\033[91mFinger is frozen.\033[0m")
                    return
                for i in range(6):
                    idx = 6 if (control_finger_type == 0) else 2
                    if i <= idx:
                        left_hand_position[i] = int(100.0 * joyStick_data.left_trigger)
                        right_hand_position[i] = int(100.0 * joyStick_data.right_trigger)
                    else:
                        left_hand_position[i] = int(100.0 * joyStick_data.left_grip)
                        right_hand_position[i] = int(100.0 * joyStick_data.right_grip)
                    left_hand_position[i] = limit_value(left_hand_position[i], 0, 100)
                    right_hand_position[i] = limit_value(right_hand_position[i], 0, 100)
                left_hand_position[1] = 100 if joyStick_data.left_first_button_touched else 0
                right_hand_position[1] = 100 if joyStick_data.right_first_button_touched else 0
                # print(f"left_hand_position[1]: {left_hand_position[1]}, right_hand_position[1]: {right_hand_position[1]}\n")
            elif hand_finger_data is not None:
                left_qpos = hand_finger_data[0]
                right_qpos = hand_finger_data[1]
                for i in range(6):
                    left_hand_position[i] = int(100.0 * left_qpos[i]/1.70)
                    right_hand_position[i] = int(100.0 * right_qpos[i]/1.70)
                    left_hand_position[i] = limit_value(left_hand_position[i], 0, 100)
                    right_hand_position[i] = limit_value(right_hand_position[i], 0, 100)
            
            robot_hand_position.left_hand_position = left_hand_position
            robot_hand_position.right_hand_position = right_hand_position
            self.control_robot_hand_position_pub.publish(robot_hand_position)
        elif self.end_effector_type == JODELL:
            if hand_finger_data is not None:
                left_qpos = hand_finger_data[0]
                right_qpos = hand_finger_data[1]
                left_hand_position[0] = limit_value(int(255.0 * left_qpos[2] / 1.70), 0, 255)
                right_hand_position[0] = limit_value(int(255.0 * right_qpos[2] / 1.70), 0, 255)
            else:
                return
            robot_hand_position.left_hand_position = left_hand_position
            robot_hand_position.right_hand_position = right_hand_position
            self.control_robot_hand_position_pub.publish(robot_hand_position)
        elif self.end_effector_type == LEJUCLAW:
            if joyStick_data is not None:
                if joyStick_data.left_second_button_pressed and self.__button_y_last is False:
                    print(f"\033[91mButton Y is pressed.\033[0m")
                    self.__freeze_finger = not self.__freeze_finger
                self.__button_y_last = joyStick_data.left_second_button_pressed
                if self.__freeze_finger is True:
                    # print(f"\033[91mFinger is frozen.\033[0m")
                    return
                pos = [0.0] * 2
                pos[0] = int(100.0 * joyStick_data.left_trigger)
                pos[1] = int(100.0 * joyStick_data.right_trigger)
                pos[0] = limit_value(pos[0], 0, 100)
                pos[1] = limit_value(pos[1], 0, 100)
                self.pub_leju_claw_command(pos)
            elif hand_finger_data is not None:
                left_qpos = hand_finger_data[0]
                right_qpos = hand_finger_data[1]
                left_claw_pos = limit_value(int(100.0 * left_qpos[2] / 1.70), 0, 100)
                right_claw_pos = limit_value(int(100.0 * right_qpos[2] / 1.70), 0, 100)
                self.pub_leju_claw_command([left_claw_pos, right_claw_pos])
                # print(f"left_claw_pos: {left_claw_pos}, right_claw_pos: {right_claw_pos}")
            else:
                return

    # 添加手臂模式回调函数
    def arm_mode_callback(self, msg):
        new_mode = msg.data
        if new_mode != 2:  # 当模式不是2时
            # 重置所有姿态
            print(f"\033[91m[IK]Reset arm mode.\033[0m")
            self.trigger_reset_mode = True
            self.arm_mode_changing = False
            self.collision_check_control = False
        elif new_mode == 2:
            print(f"\033[91m[IK]Arm mode changing.\033[0m")
            self.arm_mode_changing = True
            
    def sensor_data_raw_callback(self, msg):
        self.sensor_data_raw = msg

    def set_arm_mode_changing_callback(self, req):
        """服务回调函数，设置arm_mode_changing为True"""

        self.arm_mode_changing = True
        
        if self.only_half_up_body:
            # 发送当前手臂的关节状态到kuavo_arm_traj来清空mpc节点话题接收队列
            # 防止半身手臂切换时刻mpc执行旧的kuavo_arm_tarj
            if self.sensor_data_raw is None:
                print(f"[ik_ros_uni]: sensor_data_raw is None")
            else:
                rate = rospy.Rate(1 / self.controller_dt)
                arm_current_state = np.array(self.sensor_data_raw.joint_data.joint_q[12:26]).copy()
                msg = JointState()
                msg.name = ["arm_joint_" + str(i) for i in range(1, 15)]
                msg.header.stamp = rospy.Time.now()
                msg.position = 180.0 / np.pi * np.array(arm_current_state)
                for i in range(20):
                    self.pub.publish(msg)
                    rate.sleep()

        response = TriggerResponse()
        response.success = True
        response.message = "Arm mode changing set to True"
        return response
    
    def collision_control_complete(self, req):
        """服务回调函数，设置collision_check_control状态"""
        self.collision_check_control = req.data
        if not req.data:
            self.arm_mode_changing = True

        response = SetBoolResponse()
        response.success = True
        response.message = "Collision check control set to " + str(self.collision_check_control)
        return response

if __name__ == "__main__":
    rospy.init_node("diff_ik_node", anonymous=True)

    np.set_printoptions(linewidth=240)
    np.set_printoptions(threshold=2000)
    np.set_printoptions(precision=4)
    np.set_printoptions(suppress=True)

    meshcat = None
    version = 4
    ik_type_idx = IkTypeIdx.TorsoIK
    ctrl_arm_idx = ArmIdx.LEFT  # 默认只控制左臂
    eef_z_bias = -0.0  # 末端坐标系的z轴偏移量
    parser = argparse.ArgumentParser()
    parser.add_argument("--ctrl_arm_idx", type=int, default=0, help="Control left or right arm, 0 for left, 1 for right.2 for both.")
    parser.add_argument("--ik_type_idx", type=int, default=0, help="Ik type, 0 for TorsoIK, 1 for DiffIK.")
    parser.add_argument("--ee_type", "--end_effector_type", dest="end_effector_type", type=str, default="", help="End effector type, jodell , qiangnao or lejuclaw.")
    parser.add_argument("--send_srv", type=int, default=1, help="Send arm control service, True or False.")
    parser.add_argument("--control_finger_type", type=int, default=0, help="0: control all fingers by upper-gripper. 1: control thumb and index fingers by upper-gripper, control other fingers by lower-gripper.")
    parser.add_argument("--control_torso", type=int, default=0, help="0: do NOT control, 1: control torso.")
    parser.add_argument("--predict_gesture", type=str2bool, default=False, help="Use Neural Network to predict hand gesture, True or False.")
    parser.add_argument("--eef_z_bias", type=float, default=-0.0, help="End effector z-axis bias distance.")
    parser.add_argument("--hand_reference_mode", type=str, default="thumb_index", help="Hand reference mode: fingertips, middle_finger, or thumb_index.")
    args, unknown = parser.parse_known_args()
    # ee_type
    end_effector_type=""
    try:
        if rospy.has_param("/end_effector_type"):
            end_effector_type = rospy.get_param("/end_effector_type")
            print(f"\033[92mend_effector_type from rosparm: {end_effector_type}\033[0m")
        else:
            print(f"\033[92mend_effector_type from args: {end_effector_type}\033[0m")
            end_effector_type = args.end_effector_type
    except Exception as e:
        print(e)
    ctrl_arm_idx = ArmIdx(args.ctrl_arm_idx)
    ik_type_idx = IkTypeIdx(args.ik_type_idx)
    send_srv = args.send_srv
    control_finger_type = args.control_finger_type
    control_torso = args.control_torso
    predict_gesture = args.predict_gesture
    hand_reference_mode = args.hand_reference_mode

    print(f"\033[92mControl {ctrl_arm_idx.name()} arms.\033[0m")
    print(f"\033[92mIk type: {ik_type_idx.name()}\033[0m")
    print(f"\033[92mControl_torso: {control_torso}\033[0m")
    
    current_pkg_path = get_package_path("motion_capture_ik")
    kuavo_assests_path = get_package_path("kuavo_assets")
    robot_version = os.environ.get('ROBOT_VERSION', '40')

    model_file = kuavo_assests_path + f"/models/biped_s{robot_version}/urdf/drake/biped_v3_arm.urdf"
    model_config_file = kuavo_assests_path + f"/config/kuavo_v{robot_version}/kuavo.json"
    # model_file = current_pkg_path + "/models/biped_gen4.0/urdf/biped_v3_arm.urdf"
    
    assert os.path.exists(model_file), f"Model file {model_file} does not exist."
    assert os.path.exists(model_config_file), f"Model config file {model_config_file} does not exist."
    
    # end_frames_name = ["torso", "l_hand_roll", "r_hand_roll", "l_forearm_pitch", "r_forearm_pitch"]
    import json
    with open(model_config_file, 'r') as f:
        model_config = json.load(f)
    end_frames_name = model_config["end_frames_name_ik"]
    shoulder_frame_names = model_config["shoulder_frame_names"]
    upper_arm_length = model_config["upper_arm_length"]
    lower_arm_length = model_config["lower_arm_length"]
    num_arm_joints_var = model_config["NUM_ARM_JOINT"]
    eef_z_bias = model_config.get("eef_z_offset", 0.0)
    base_chest_offset_x = model_config.get("base_chest_offset_x", 0.0)
    print(f"num_arm_joints_var: {num_arm_joints_var}")
    print(f"upper_arm_length: {upper_arm_length}, lower_arm_length: {lower_arm_length}")

    rospy.set_param("/quest3/shoulder_width", model_config.get("shoulder_width", 0.15))
    rospy.set_param("/quest3/base_height_offset", model_config.get("base_height_offset", 0.23))
    rospy.set_param("/quest3/base_chest_offset_x", base_chest_offset_x)
    rospy.set_param("/quest3/upper_arm_length", upper_arm_length)
    rospy.set_param("/quest3/lower_arm_length", lower_arm_length)
    
    print(f"shoulder_width: {model_config.get('shoulder_width', 0.15)}")
    print(f"Model file: {model_file}")
    print(f"Model config file: {model_config_file}")
    print(f"shoulder_frame_names: {shoulder_frame_names}")
    print(f"End effector z-axis bias distance: {eef_z_bias} m.")
    print(f"End frames names: {end_frames_name}")
    print(f"Send srv?: {send_srv}")
    print(f"Control finger type: {control_finger_type}")
    print(f"Predict gesture?: {predict_gesture}")
    arm_ik = None


    arm_min = np.array([-3.14, -0.70, -1.57, -1.57, -1.57, -1.57, -1.57, -3.14, -2.09, -1.57, -1.57, -1.57, -1.57, -1.57], dtype=float)
    arm_max = np.array([0.520, 2.09, 1.570, 0.000, 1.570, 1.570, 1.570, 0.7, 1.000, 1.570, 0.000, 1.570, 1.570, 1.570], dtype=float)
    q_limit = None
    if ik_type_idx == IkTypeIdx.DiffIK:        
        arm_ik = DiffIK(
            model_file, 
            end_frames_name,
            arm_idx=ctrl_arm_idx, 
            q_limit=q_limit, 
            meshcat=meshcat,
            eef_z_bias=eef_z_bias,
            shoulder_frame_names=shoulder_frame_names
            )
    solver_tol_default = 9.0e-3
    iterations_limit_default = 100
    if robot_version == "13":
        solver_tol_default = 9.0e-6
        iterations_limit_default = 2000
    if ik_type_idx == IkTypeIdx.TorsoIK:
        arm_ik = ArmIk(
            model_file,
            end_frames_name,
            meshcat,
            constraint_tol=9e-3,
            solver_tol=solver_tol_default,
            iterations_limit=iterations_limit_default,
            eef_z_bias=eef_z_bias,
            ctrl_arm_idx=ctrl_arm_idx,
            as_mc_ik=True,
            shoulder_frame_names=shoulder_frame_names

        )
        arm_ik.init_state(0.0, 0.0)
        
    print("\n" + "*"*10 + "IK ARM INFO" + "*"*10)
    arm_length_left, arm_length_right = arm_ik.get_arm_length()
    p_bS = arm_ik.get_two_frame_dis_vec(shoulder_frame_names[0], end_frames_name[0])
    upper_arm_length = arm_ik.get_two_frame_dis(shoulder_frame_names[0], end_frames_name[3])
    lower_arm_length = arm_ik.get_two_frame_dis(end_frames_name[3], end_frames_name[1])
    shoulder_width_vec = arm_ik.get_two_frame_dis_vec(shoulder_frame_names[0], shoulder_frame_names[1])
    # shoulder_width = shoulder_width_vec[1]/2
    print(f"upper_arm_length: {upper_arm_length:.3f} cm, lower_arm_length: {lower_arm_length:.3f} cm")
    print(f"shoulder_width: {shoulder_width_vec[1]/2} m")
    print(f"bias_chest_to_base_link: {p_bS} m")
    # rospy.set_param("/quest3/base_shoulder_x_bias", float(p_bS[0]))
    # rospy.set_param("/quest3/base_shoulder_y_bias", float(p_bS[1]))
    # rospy.set_param("/quest3/base_shoulder_z_bias", float(p_bS[2]))
    # rospy.set_param("/quest3/upper_arm_length", float(upper_arm_length))
    # rospy.set_param("/quest3/lower_arm_length", float(lower_arm_length))
    # rospy.set_param("/quest3/shoulder_width", float(shoulder_width))
    # print(f"\033[92mLeft Arm Length: {arm_length_left:.3f} m, Right Arm Length:{arm_length_right:.3f} m.\033[0m")
    print("*"*10 + "IK ARM INFO END" + "*"*10 + "\n")


    ik_ros = IkRos(arm_ik, ctrl_arm_idx=ctrl_arm_idx, q_limit=q_limit, end_effector_type=end_effector_type, send_srv=send_srv, predict_gesture=predict_gesture, hand_reference_mode=hand_reference_mode)
