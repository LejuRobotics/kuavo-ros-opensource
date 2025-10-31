#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import termios
import tty
import select
import time
import math
import rospy
import numpy as np
from enum import Enum

from kuavo_msgs.srv import fkSrv
from kuavo_msgs.msg import sensorsData

from kuavo_msgs.msg import armTargetPoses
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse
from kuavo_msgs.srv import twoArmHandPoseCmdSrv
from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam

# 获取机器人版本
def get_parameter(param_name):
    try:
        # 获取参数值
        param_value = rospy.get_param(param_name)
        rospy.loginfo(f"参数 {param_name} 的值为: {param_value}")
        return param_value
    except rospy.ROSException:
        rospy.logerr(f"参数 {param_name} 不存在！程序退出。")
        rospy.signal_shutdown("参数获取失败") 
        return None

# FK正解服务
def fk_srv_client(joint_angles):
  # 确保要调用的服务可用
  rospy.wait_for_service('/ik/fk_srv')
  try:
      # 初始化服务代理
      fk_srv = rospy.ServiceProxy('/ik/fk_srv', fkSrv)
      # 调取服务并获得响应
      fk_result = fk_srv(joint_angles)
      # 打印是否求解成功
      print("FK result:", fk_result.success,"\r")
      # 返回正解结果（手臂位置姿态等信息）
      return fk_result.hand_poses
  except rospy.ServiceException as e:
      print("Service call failed: %s"%e)

# 通过四元数计算角度（弧度制）
class Euler:
    def __init__(self):
        self.roll = 0    
        self.pitch = 0    
        self.yaw = 0     
def quaternion_to_euler(x, y, z, w):
    """
    将四元数转换为欧拉角。
    :param x, y, z, w: 四元数的分量。
    :return: 弧度制的roll, pitch, yaw角。
    """
    e = Euler()
    # 计算roll, pitch, yaw
    sinr_cosp = 2 * (w * z + x * y)
    cosr_cosp = 1 - 2 * (y**2 + z**2)
    e.roll = math.atan2(sinr_cosp, cosr_cosp)
    #print(f"roll: {math.degrees(roll)}")
    sinp = 2 * (w * y - z * x)
    e.pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)
    #print(f"pitch: {math.degrees(pitch)}")
    siny_cosp = 2 * (w * x + y * z)
    cosy_cosp = 1 - 2 * (x**2 + y**2)
    e.yaw = math.atan2(siny_cosp, cosy_cosp)
    #print(f"yaw: {math.degrees(yaw)}")
    return e
# 通过角度（弧度制）计算四元数
class Quaternion:
    def __init__(self):
        self.w = 0    
        self.x = 0    
        self.y = 0     
        self.z = 0
def euler_to_quaternion(yaw, pitch, roll): # yaw (Z), pitch (Y), roll (X)

    # Abbreviations for the various angular functions
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    
    norm = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
    if norm > 0:
        q.w /= norm
        q.x /= norm
        q.y /= norm
        q.z /= norm

    return q

# 欧拉角(Z-Y-X顺序) → 旋转矩阵 → 四元数
def euler_to_rotation_matrix(yaw_adaptive=0, pitch_adaptive=0, roll_adaptive=0,
                            yaw_manual=0, pitch_manual=0, roll_manual=0):
    """
    欧拉角(Z-Y-X顺序) → 旋转矩阵
    参数:
        yaw (float):   绕Z轴旋转角度（弧度）
        pitch (float): 绕Y轴旋转角度（弧度）
        roll (float):  绕X轴旋转角度（弧度）
    返回:
        np.ndarray: 3x3旋转矩阵
    """
    # 计算三角函数值
    cy, sy = np.cos(yaw_adaptive), np.sin(yaw_adaptive)
    cp, sp = np.cos(pitch_adaptive), np.sin(pitch_adaptive)
    
    R = np.array([
        [cy * cp,   -sy,        cy * sp],
        [sy * cp,    cy,        sy * sp],
        [-sp,        0,         cp     ]
    ])

    # 存在自定义参数 需要二次旋转
    if yaw_manual or pitch_manual or roll_manual:

        # 初始化为单位矩阵
        R_manual = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        if abs(yaw_manual) > 0.01:
            print("yaw_manual=",yaw_manual)
            c, s = np.cos(yaw_manual), np.sin(yaw_manual)
            R_manual = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ R_manual

        if abs(pitch_manual) > 0.01:
            print("pitch_manual=",pitch_manual)
            c, s = np.cos(pitch_manual), np.sin(pitch_manual)
            R_manual = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) @ R_manual

        if abs(roll_manual) > 0.01:
            print("roll_manual=",roll_manual)
            c, s = np.cos(roll_manual), np.sin(roll_manual)
            R_manual = np.array([[1, 0, 0], [0, c, -s], [0, s, c]]) @ R_manual

        return R @ R_manual
    # 不存在自定义参数,直接输出旋转矩阵
    else :
        return R

def rotation_matrix_to_quaternion(R):
    """
    旋转矩阵 → 四元数
    参数:
        R (np.ndarray): 3x3旋转矩阵
    返回:
        np.ndarray: 四元数 [x, y, z, w]
    """
    # 计算四元数分量
    trace = np.trace(R)

    q = Quaternion()

    if trace > 0:
        q.w = math.sqrt(trace + 1.0) / 2
        q.x = (R[2, 1] - R[1, 2]) / (4 * q.w)
        q.y = (R[0, 2] - R[2, 0]) / (4 * q.w)
        q.z = (R[1, 0] - R[0, 1]) / (4 * q.w)
    else:
        # 处理w接近零的情况
        i = np.argmax([R[0, 0], R[1, 1], R[2, 2]])
        j = (i + 1) % 3
        k = (j + 1) % 3
        t = np.zeros(4)
        t[i] = math.sqrt(R[i, i] - R[j, j] - R[k, k] + 1) / 2
        t[j] = (R[i, j] + R[j, i]) / (4 * t[i])
        t[k] = (R[i, k] + R[k, i]) / (4 * t[i])
        t[3] = (R[k, j] - R[j, k]) / (4 * t[i])

        q.x, q.y, q.z, q.w = t  # 重排序为[x, y, z, w]

    # 归一化（防止数值误差）
    norm = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
    if norm > 0:
        q.w /= norm
        q.x /= norm
        q.y /= norm
        q.z /= norm
    return q

def euler_to_quaternion_via_matrix(yaw_adaptive=0, pitch_adaptive=0, roll_adaptive=0,
                                    yaw_manual=0, pitch_manual=0, roll_manual=0):
    """
    欧拉角 → 旋转矩阵 → 四元数
    参数:
        yaw (float):   绕Z轴旋转角度(弧度)
        pitch (float): 绕Y轴旋转角度(弧度)
        roll (float):  绕X轴旋转角度(弧度)
    返回:
        np.ndarray: 四元数 [x, y, z, w]
    """
    R = euler_to_rotation_matrix(yaw_adaptive, pitch_adaptive, roll_adaptive,
                                yaw_manual, pitch_manual, roll_manual)
    return rotation_matrix_to_quaternion(R)


# 通过末端坐标自适应计算末端合理的角度 pitch
def eff_orientation_pitch(Proximal_Arm, Distal_Arm, pos_x, pos_y):
    # 计算 D 并检查可达性
    D = (pos_x**2 + pos_y**2 - Proximal_Arm**2 - Distal_Arm**2) / (2 * Proximal_Arm * Distal_Arm)
    D = np.clip(D, -1.0, 1.0)  # 避免数值误差
    
    # 强制 theta2 > 0（肘部上方）
    theta2 = np.arccos(D)
    
    # 计算 theta1
    numerator = Distal_Arm * np.sin(theta2)
    denominator = Proximal_Arm + Distal_Arm * np.cos(theta2)
    theta1 = np.arctan2(pos_y, pos_x) - np.arctan2(numerator, denominator)
    
    # 末端朝向（逆时针为正）
    phi = theta1 + theta2
    
    return -1*(phi+1.57)  # 返回弧度值

# 设置手臂运动模式
def set_arm_control_mode(mode):
    # 创建服务代理，用于与服务通信
    arm_traj_change_mode_client = rospy.ServiceProxy("/arm_traj_change_mode", changeArmCtrlMode)

    # 创建请求对象
    request = changeArmCtrlModeRequest()
    request.control_mode = mode  # 设置请求的控制模式

    # 发送请求并接收响应
    response = arm_traj_change_mode_client(request)

    if response.result:
        # 如果响应结果为真，表示成功更改控制模式
        rospy.loginfo(f"Successfully changed arm control mode to {mode}: {response.message}")
    else:
        # 如果响应结果为假，表示更改控制模式失败
        rospy.logwarn(f"Failed to change arm control mode to {mode}: {response.message}")

# 发布手臂目标姿态
def publish_arm_target_poses(times, values):
    # 创建Publisher对象
    pub = rospy.Publisher('kuavo_arm_target_poses', armTargetPoses, queue_size=10)
    rospy.sleep(0.5)  # 确保Publisher注册
    # 创建消息对象并设置传入的times和values
    msg = armTargetPoses()
    msg.times = times
    msg.values = values

    #rospy.loginfo("正在将手臂目标姿态发布到话题 'kuavo_arm_target_poses'")

    # 等待订阅者连接
    rate = rospy.Rate(10)  # 10Hz
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("等待订阅者连接...")
        rate.sleep()

    # 发布消息
    pub.publish(msg)
    #rospy.loginfo("消息已发布。")
    rospy.loginfo("move over")

# ik相关
class IkArmService:
    def __init__(self):
        # 自定义ik参数
        use_custom_ik_param = True
        # 使用默认的关节角度作为ik的初始预测
        joint_angles_as_q0 = False 
        # 创建ikSolverParam对象
        ik_solve_param = ikSolveParam()
        # 设置ikSolveParam对应参数
        ik_solve_param.major_optimality_tol = 1e-3
        ik_solve_param.major_feasibility_tol = 1e-3
        ik_solve_param.minor_feasibility_tol = 1e-3
        ik_solve_param.major_iterations_limit = 100
        ik_solve_param.oritation_constraint_tol= 1e-3
        ik_solve_param.pos_constraint_tol = 1e-3 
        ik_solve_param.pos_cost_weight = 0.0 

        # 创建请求对象
        self.eef_pose_msg = twoArmHandPoseCmd()
        # 设置请求参数
        self.eef_pose_msg.ik_param = ik_solve_param
        self.eef_pose_msg.use_custom_ik_param = use_custom_ik_param
        #self.eef_pose_msg.joint_angles_as_q0 = joint_angles_as_q0
        # joint_angles_as_q0 为 False 时，这两个参数不会被使用（单位：弧度）
        #self.eef_pose_msg.hand_poses.left_pose.joint_angles = np.zeros(7)
        #self.eef_pose_msg.hand_poses.right_pose.joint_angles = np.zeros(7)

        # 初始化末端姿态数组
        self.eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)
        self.eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)

    def ik_one_hand(self, current_joint_values = [0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0, 
                                0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0], hand_flag = 2 ,
                    l_hand_pose=np.array([0.45,0.25,0.11988012]), r_hand_pose=np.array([0.45,-0.25,0.11988012]), 
                    l_hand_quat=[0.0,-0.70682518,0.0,0.70738827], r_hand_quat=[0.0,-0.70682518,0.0,0.70738827]):
        
        self.eef_pose_msg.joint_angles_as_q0 = True
        self.eef_pose_msg.hand_poses.left_pose.joint_angles = current_joint_values[:7]
        self.eef_pose_msg.hand_poses.right_pose.joint_angles = current_joint_values[-7:]
        
        self.eef_pose_msg.hand_poses.left_pose.pos_xyz = l_hand_pose
        self.eef_pose_msg.hand_poses.left_pose.quat_xyzw = l_hand_quat
        self.eef_pose_msg.hand_poses.right_pose.pos_xyz = r_hand_pose
        self.eef_pose_msg.hand_poses.right_pose.quat_xyzw = r_hand_quat

        res = self.call_ik_srv()
        
        if(res.success):
            print("ik success")
            l_pos = res.hand_poses.left_pose.pos_xyz
            l_pos_error = np.linalg.norm(l_pos - self.eef_pose_msg.hand_poses.left_pose.pos_xyz)
            r_pos = res.hand_poses.right_pose.pos_xyz
            r_pos_error = np.linalg.norm(r_pos - self.eef_pose_msg.hand_poses.right_pose.pos_xyz)

            if  hand_flag == 1 :# 左手
                joint_end_angles = np.concatenate([res.hand_poses.left_pose.joint_angles, current_joint_values[-7:]])
            else :
                joint_end_angles = np.concatenate([current_joint_values[:7], res.hand_poses.right_pose.joint_angles])
            
            #degrees_list = [math.degrees(rad) for rad in joint_end_angles]
            # 调用函数并传入times和values
            #self.publish_arm_target_poses([5], degrees_list)
            #time.sleep(5)
            #return degrees_list
            return joint_end_angles
        else :
            print("ik fail")
            return False
        print(" ")

    # IK 逆解服务
    def call_ik_srv(self):
        # 确保要调用的服务可用
        rospy.wait_for_service('/ik/two_arm_hand_pose_cmd_srv')
        try:
            # 初始化服务代理
            ik_srv = rospy.ServiceProxy('/ik/two_arm_hand_pose_cmd_srv', twoArmHandPoseCmdSrv)
            # 调取服务并获得响应
            res = ik_srv(self.eef_pose_msg)
            # 返回逆解结果
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False, []

class ArmType(Enum):
    Right = 0,
    Left = 1
class KeyBoardArmController:
    def __init__(self, x_gap = 0.03, y_gap = 0.03, z_gap = 0.03, 
                 roll_gap = 0.03, pitch_gap = 0.03, yaw_gap = 0.03, 
                 time_gap = 0.5, robot_version = 45 , which_hand=ArmType.Right):
        rospy.init_node("arm_control_keyboard_node", anonymous=True)

        self.old_settings = termios.tcgetattr(sys.stdin)
        self.input_buffer = []  # 存储按键缓冲区

        """ Variables """
        self._control_xyz_keys = {                      # 控制位置的按键映射
            'w': {'axis':'x', 'index':0, 'gap': x_gap},
            's': {'axis':'x', 'index':0, 'gap': -x_gap},
            'd': {'axis':'y', 'index':1, 'gap': y_gap},
            'a': {'axis':'y', 'index':1, 'gap': -y_gap},
            'q': {'axis':'z', 'index':2, 'gap': z_gap},
            'e': {'axis':'z', 'index':2, 'gap': -z_gap}
        }
        self._control_rpy_keys = {                      # 控制姿态的按键映射
            'u': {'axis':'roll', 'index':2, 'gap': roll_gap},
            'o': {'axis':'roll', 'index':2, 'gap': -roll_gap},
            'i': {'axis':'picth', 'index':1, 'gap': pitch_gap},
            'k': {'axis':'picth', 'index':1, 'gap': -pitch_gap},
            'j': {'axis':'yaw', 'index':0, 'gap': yaw_gap},
            'l': {'axis':'yaw', 'index':0, 'gap': -yaw_gap}
        }
        self.eef_target_xyz = np.array(np.zeros(3), dtype=object)    # 目标末端位置xyz
        self.eef_target_ypr = np.array(np.zeros(3), dtype=object)    # 目标末端姿态ypr
        self.eef_angle_manual = np.array(np.zeros(3), dtype=object)  # 手动设置的目标末端姿态ypr
        self.current_joint_values = [0] * 14           # 当前关节数值
        self.zero_joint_values = [0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0, 
                                    0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0]
        
        self.which_hand = which_hand                   # 左/右手
        self.control_rpy_flag = False
        self._flag_pose_inited = False

        self.ik_service=IkArmService()

        #不同型号机器人的初始位置 (机器人坐标系) 和 手臂长度(单位米)
        def start_with_version(version_number:int, series:int):
            """判断版本号是否属于某系列"""
            # PPPPMMMMN
            MMMMN_MASK = 100000
            return (version_number % MMMMN_MASK) == series
        if start_with_version(robot_version, 45) or start_with_version(robot_version, 49):
            self.robot_zero_x = -0.0173
            self.robot_zero_y = -0.2927
            self.robot_zero_z = -0.2837
            self.robot_upper_arm = 0.30
            self.robot_lower_arm = 0.40

        elif start_with_version(robot_version, 42):
            self.robot_zero_x = -0.0175
            self.robot_zero_y = -0.25886
            self.robot_zero_z = -0.20115
            self.robot_upper_arm = 0.18
            self.robot_lower_arm = 0.32

        self.exec_time = 0
        self.time_gap = time_gap    # 每段执行时间

        self.count=0

        self.joint_state_subscriber = rospy.Subscriber('/sensors_data_raw', sensorsData, self.update_joint_state_callback)
    
    # 获取传感器数据，回调函数，更新当前角度
    # 通过一次fk求解，将当前角度转换为当前坐标，以此进行初始化，用于展示当前坐标和后续修改目标信息
    def update_joint_state_callback(self, data):
        arm_joint_data = data.joint_data.joint_q[12:26]
        self.current_joint_values = arm_joint_data
        # 初始化
        if not self._flag_pose_inited:
            # 调用 FK 正解服务
            fk_hand_poses = fk_srv_client(arm_joint_data)
            if fk_hand_poses is not None:
                if self.which_hand == ArmType.Left:
                    print("left hand poses ready","\r")
                    self.eef_target_xyz = np.array(fk_hand_poses.left_pose.pos_xyz)
                    x, y, z, w = fk_hand_poses.left_pose.quat_xyzw
                    self.eef_target_ypr = np.array([euler.yaw, euler.pitch, euler.roll])
                else :
                    print("right hand poses ready","\r")
                    self.eef_target_xyz = np.array(fk_hand_poses.right_pose.pos_xyz)
                    x, y, z, w = fk_hand_poses.right_pose.quat_xyzw
                    euler =quaternion_to_euler(x, y, z, w)
                    self.eef_target_ypr = np.array([euler.yaw, euler.pitch, euler.roll])

                pos_str = [f"{x:.3f}m" for x in self.eef_target_xyz]
                rot_str = [f"{np.degrees(x):.1f}°" for x in self.eef_target_ypr]
                print(f"Current: pos={pos_str}, rot={rot_str}", end='\r\n')    
                
                self._flag_pose_inited = True
            else:
                print("No hand poses returned")

    # 按键检测 
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

        # 更新末端位置
        if  key in self._control_xyz_keys:
            try:
                ctrl = self._control_xyz_keys[key]
                index = ctrl['index']
                self.eef_target_xyz[index] += ctrl['gap']
                # 自适应末端姿态
                # yaw
                if self.which_hand == ArmType.Left:
                    self.eef_target_ypr[0]=math.atan((self.robot_zero_y - self.eef_target_xyz[1])/(self.eef_target_xyz[0] - self.robot_zero_x))
                else :
                    self.eef_target_ypr[0]=math.atan((self.eef_target_xyz[1] - self.robot_zero_y)/(self.eef_target_xyz[0] - self.robot_zero_x))
                # pitch
                self.eef_target_ypr[1]=eff_orientation_pitch(self.robot_upper_arm, self.robot_lower_arm, 
                                                            math.sqrt((self.eef_target_xyz[1] - self.robot_zero_y)**2+self.eef_target_xyz[0]**2), 
                                                            self.eef_target_xyz[2]-0.4240)
                # roll 
                self.eef_target_ypr[2]= 0.0
                pos_str = [f"{x:.3f}m" for x in self.eef_target_xyz]
                rot_str = [f"{np.degrees(x):.1f}°" for x in self.eef_target_ypr]
                rot_manual_str = [f"{np.degrees(x):.1f}°" for x in self.eef_angle_manual]
                print(f"按下{key}键, {ctrl['axis']} 值变化 {ctrl['gap']}")

            except Exception as e:
                print(e)
        # 手动调整末端姿态
        elif key in self._control_rpy_keys:
            try:
                ctrl = self._control_rpy_keys[key]
                index = ctrl['index']
                self.eef_angle_manual[index] += ctrl['gap']

                pos_str = [f"{x:.3f}m" for x in self.eef_target_xyz]
                rot_str = [f"{np.degrees(x):.1f}°" for x in self.eef_target_ypr]
                rot_manual_str = [f"{np.degrees(x):.1f}°" for x in self.eef_angle_manual]
                print(f"按下{key}键, {ctrl['axis']} 值变化 {ctrl['gap']}")
            except Exception as e:
                print(e)
        elif key == 'g':
            try:
                pos_str = [f"{x:.3f}m" for x in self.eef_target_xyz]
                rot_str = [f"{np.degrees(x):.1f}°" for x in self.eef_target_ypr]
                rot_manual_str = [f"{np.degrees(x):.1f}°" for x in self.eef_angle_manual]

                if self.control_rpy_flag==False:
                    self.control_rpy_flag=True
                    print("手动控制末端姿态模式")
                else :
                    self.control_rpy_flag=False
                    print("自动控制末端姿态模式")
            except Exception as e:
                print(e)
        elif key != '\x03':
            key=""
            return key
        else :
            return key
        
        # 打印当前状态
        if self.control_rpy_flag==False:
            print(f"Target: pos={pos_str}, auto_rot={rot_str}, manual_rot(no use)={rot_manual_str}", end='\r\n')
        else :
            print(f"Target: pos={pos_str}, auto_rot={rot_str}, manual_rot={rot_manual_str}", end='\r\n')
        return key
    
    # 进行ik逆求解 进行线性插值 将任务发给关节
    def update_response_2(self):
        # ik逆解
        
        #quat=euler_to_quaternion(self.eef_target_ypr[0],self.eef_target_ypr[1],self.eef_target_ypr[2])
        if self.control_rpy_flag==True:
            # 求解带手动参数的ik结果
            quat=euler_to_quaternion_via_matrix(self.eef_target_ypr[0],self.eef_target_ypr[1],self.eef_target_ypr[2],
                                                self.eef_angle_manual[0],self.eef_angle_manual[1],self.eef_angle_manual[2])
            joint_end_angles=self.ik_service.ik_one_hand(current_joint_values = self.current_joint_values, hand_flag = 2 ,
                                        r_hand_pose=self.eef_target_xyz, r_hand_quat=[quat.x,quat.y,quat.z,quat.w])
        else :
            quat=euler_to_quaternion_via_matrix(self.eef_target_ypr[0],self.eef_target_ypr[1],self.eef_target_ypr[2])
            joint_end_angles=self.ik_service.ik_one_hand(current_joint_values = self.current_joint_values, hand_flag = 2 ,
                                        r_hand_pose=self.eef_target_xyz, r_hand_quat=[quat.x,quat.y,quat.z,quat.w])
        if isinstance(joint_end_angles, np.ndarray):  # 判断是否是 NumPy 数组 
        #if True :
            degrees_list = [math.degrees(rad) for rad in joint_end_angles]
            # 线性插值
            self.exec_time = self.exec_time + self.time_gap
            # 发送任务
            publish_arm_target_poses([5], degrees_list)
        # print("update_joy over")


    def update_response(self):
        self.count+=1
        #print("check:",self.count)
        print("test", end='\r')
        print(f"check:",self.count, end='\r')

    def run(self): 
        print("waiting for ik server...")
        # 等待初始化结束
        while not self._flag_pose_inited and not rospy.is_shutdown():
            time.sleep(0.2)
        
        try:
            print("Use keys to control:")
            print("WS: position - X")
            print("AD: position - Y")
            print("QE: position - Z")
            print("UO: rotation - X - ROLL")
            print("IK: rotation - Y - PITCH")
            print("JL: rotation - Z - YAW")
            print("Press Ctrl-C to exit")

            set_arm_control_mode(2)
            
            while not rospy.is_shutdown():
                key = self.getKey()
                if (key == '\x03'):  # Ctrl-C
                    break
                if key:
                    self.update_response_2()
                

            set_arm_control_mode(1)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


if __name__ == "__main__":
    try:
        # 获取机器人版本
        my_robot_version = get_parameter('robot_version')
        def start_with_version(version_number:int, series:int):
            """判断版本号是否属于某系列"""
            # PPPPMMMMN
            MMMMN_MASK = 100000
            return (version_number % MMMMN_MASK) == series
        if start_with_version(my_robot_version, 42) or start_with_version(my_robot_version, 45) or start_with_version(my_robot_version, 49):
            print("机器人版本为:",my_robot_version)
        else :
            print("机器人版本号错误, 仅支持42 45 49系列")
            pass

        # Right Arm
        keyboard_arm_controller = KeyBoardArmController(x_gap = 0.05, y_gap = 0.05, z_gap = 0.05,
                                                        roll_gap = 0.157, pitch_gap = 0.157, yaw_gap = 0.157, 
                                                        time_gap = 0.5,
                                                        robot_version = my_robot_version,
                                                        which_hand=ArmType.Right)
        keyboard_arm_controller.run()
    except rospy.ROSInterruptException:
        pass