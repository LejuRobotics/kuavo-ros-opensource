#!/usr/bin/env python3

import rospy
from vision_msgs.msg import Detection2DArray
from collections import defaultdict

import math
import numpy as np  # 引入numpy库用于数值计算

from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse
from kuavo_msgs.srv import twoArmHandPoseCmdSrv
from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam

from kuavo_msgs.msg import armTargetPoses
from kuavo_msgs.msg import robotHandPosition
from kuavo_msgs.msg import robotHeadMotionData
from kuavo_msgs.msg import robotWaistControl

######################## ik求解部分 ############################################

# 创建ikSolverParam对象
ik_solve_param = ikSolveParam()
# 设置ikSolveParam对应参数
ik_solve_param.major_optimality_tol = 1e-3
ik_solve_param.major_feasibility_tol = 1e-3
ik_solve_param.minor_feasibility_tol = 1e-3
ik_solve_param.major_iterations_limit = 500
ik_solve_param.oritation_constraint_tol= 1e-3
ik_solve_param.pos_constraint_tol = 1e-3 
ik_solve_param.pos_cost_weight = 1.0 # 最大化位置权重, 确保目标点结果准确

# 获取机器人版本
def get_version_parameter():
    param_name = 'robot_version'
    try:
        # 获取参数值
        param_value = rospy.get_param(param_name)
        rospy.loginfo(f"参数 {param_name} 的值为: {param_value}")
        # 适配1000xx版本号
        valid_series = [42, 45, 49, 52, 53, 54]
        MMMMN_MASK = 100000
        series = param_value % MMMMN_MASK
        if series not in valid_series:
            rospy.logwarn(f"无效的机器人版本号: {param_value}，仅支持 {valid_series} 系列！")
            return None
        else:
            rospy.loginfo(f"✅ 机器人版本号有效: {param_value}")
            return param_value
    except rospy.ROSException:
        rospy.logerr(f"参数 {param_name} 不存在！") 
        return None

# IK 逆解服务
def call_ik_srv(eef_pose_msg):
    # 确保要调用的服务可用
    rospy.wait_for_service('/ik/two_arm_hand_pose_cmd_srv')
    try:
        # 初始化服务代理
        ik_srv = rospy.ServiceProxy('/ik/two_arm_hand_pose_cmd_srv', twoArmHandPoseCmdSrv)
        # 调取服务并获得响应
        res = ik_srv(eef_pose_msg)
        # 返回逆解结果
        return res
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s"%e)
        return False, []

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

######################### 几何计算部分 计算抓取位姿四元数 ###########################################

# 通过角度（弧度制）计算四元数
class Quaternion:
    def __init__(self):
        self.w = 0    
        self.x = 0    
        self.y = 0     
        self.z = 0

# yaw (Z), pitch (Y), roll (X)
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
            # print("yaw_manual=",yaw_manual)
            c, s = np.cos(yaw_manual), np.sin(yaw_manual)
            R_manual = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ R_manual

        if abs(pitch_manual) > 0.01:
            # print("pitch_manual=",pitch_manual)
            c, s = np.cos(pitch_manual), np.sin(pitch_manual)
            R_manual = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) @ R_manual

        if abs(roll_manual) > 0.01:
            # print("roll_manual=",roll_manual)
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

######################### 几何计算部分 计算转腰抓取点及转腰角度 ###########################################

def find_mid_point_of_two_intersections(a, b, r, R):
    """
    预设定的区间 计算线段+半圆弧双交点，输出【圆弧中点】坐标
    :param a: A的x坐标 (a>0)
    :param b: A的y坐标为-b (b>0)
    :param r: A的半圆半径 (r>0且r<=b)
    :param R: 原点圆半径 (R>=a)
    :return: (是否存在:bool, 圆弧中点坐标tuple/None)
    """
    # ========= 参数合法性 =========
    if not (a > 0 and b > 0 and r > 0 and r <= b and R >= a):
        return (False, None)

    OA = math.hypot(a, b)
    L = math.hypot(a, b - r)
    M = OA
    N = OA + r

    # ========= 情况 1：完全不可达 =========
    if R < L:
        return (False, None)
    # ========= 情况 2：L ≤ R < M =========
    elif L <= R < M:
        # 交点1：垂直线x=a 与 原点圆的交点
        y1 = -math.sqrt(R**2 - a**2)
        p1 = (a, y1)
        # 交点2：半圆轨迹 与 原点圆的交点（迭代求解）
        # --- 判断是否可能与半圆所在整圆相交 ---
        OC = math.hypot(a, b)
        if R < abs(OC - r) or R > OC + r:
            # 原点圆与半圆所在整圆不相交
            return (False, None)
        # --- 安全的半圆 y 求解 ---
        def solve_y(x):
            val = r**2 - (x - a)**2
            if val < 0:
                return None
            return -b - math.sqrt(val)
        # --- 数值搜索第二交点 ---
        found = False
        x2 = a
        step = r / 500.0  # 自适应步长
        while x2 <= a + r:
            y2 = solve_y(x2)
            if y2 is not None:
                if abs(x2**2 + y2**2 - R**2) < 1e-5:
                    p2 = (x2, y2)
                    found = True
                    break
            x2 += step
        if not found:
            return (False, None)
        
        # 1. 计算两个交点的极角（弧度制）
        theta1 = math.atan2(p1[1], p1[0])
        theta2 = math.atan2(p2[1], p2[0])
        # 2. 极角取平均，计算圆弧中点的极坐标→直角坐标
        theta_mid = (theta1 + theta2) / 2
        mid_x = R * math.cos(theta_mid)
        mid_y = R * math.sin(theta_mid)
        
        return (True, (round(mid_x, 4), round(mid_y, 4)))
    # ========= 情况 3：M ≤ R < N =========
    elif M <= R < N:
        # 交点1：垂直线x=a 与 原点圆的交点
        y1 = -math.sqrt(R**2 - a**2)
        p1 = (a, y1)
        # 交点2：射线AO延长线 与 半圆的交点
        factor = r / OA
        x2 = a + a * factor
        y2 = -b - b * factor
        p2 = (x2, y2)
        
        theta1 = math.atan2(p1[1], p1[0])
        theta2 = math.atan2(p2[1], p2[0])
        theta_mid = (theta1 + theta2) / 2
        mid_x = R * math.cos(theta_mid)
        mid_y = R * math.sin(theta_mid)
        
        return (True, (round(mid_x, 4), round(mid_y, 4)))
    # ========= 情况 4：完全不可达 =========
    else:
        return (False, None)

def cal_target_with_rotation(a, b, r, m, n):
    """
    输入m,n转R → 调用原函数 → 计算旋转角度（逆时针为正）
    兼容n>0/n<0, 实现x轴对称+角度反向
    :param a: 定点A的x坐标
    :param b: 定点A的y坐标(-b)
    :param r: A的半圆半径
    :param m: 动点初始x坐标 (m>0)
    :param n: 动点初始y坐标(任意正负)
    :return: (是否存在:bool, 目标点坐标tuple/None, 旋转角度:float)
    """
    # 1. 计算原点圆半径R（与n符号无关）
    R = math.hypot(m, n)
    # 2. 调用自定义的函数获取【基准目标点】(n<0时的结果)
    exist, base_point = find_mid_point_of_two_intersections(a, b, r, R)
    # 3. 初始化目标点、旋转角度
    target_point = None
    rotate_degree = 0.0
    # 4. 根据n的符号，实现x轴对称+角度反向
    if exist and base_point is not None:
        base_x, base_y = base_point
        # ========== 目标点x轴对称 ==========
        if n > 0:
            # n>0 → 目标点y坐标取反，实现关于x轴对称
            target_point = (base_x, -base_y)
        else:
            # n≤0 → 沿用原结果
            target_point = (base_x, base_y)
        
        # ========== 旋转角度计算+反向适配 ==========
        # 初始点(m,n)的极角 → 转角度
        init_rad = math.atan2(n, m)
        init_deg = math.degrees(init_rad)
        # 目标点的极角 → 转角度
        tar_x, tar_y = target_point
        tar_rad = math.atan2(tar_y, tar_x)
        tar_deg = math.degrees(tar_rad)
        
        # 计算旋转角度 + 归一化到[-180, 180]区间，保留4位小数
        rotate_deg = init_deg - tar_deg
        if rotate_deg > 180:
            rotate_deg -= 360
        elif rotate_deg < -180:
            rotate_deg += 360
        rotate_degree = round(rotate_deg, 4)
    
    # 返回格式：(bool, 坐标元组/None, 角度值)
    return (exist, target_point, rotate_degree)

######################### 识别YOLOV8标签部分 ###########################################

class ObjectPositionTracker:
    def __init__(self):
        # 初始化ROS节点
        if not rospy.get_node_uri():
            rospy.init_node('object_position_tracker', anonymous=True)
        
        # 初始化存储结构
        self.object_positions = defaultdict(list)  # 按标签ID存储位置
        self.latest_positions = {}                 # 每个ID的最新位置
        
        # 订阅 YOLOv8 检测结果
        self.sub = rospy.Subscriber(
            '/robot_yolov8_info', 
            Detection2DArray, 
            self.detection_callback
        )
        
        rospy.loginfo("ObjectPositionTracker initialized. Waiting for detections...")
    
    def detection_callback(self, msg):
        """处理检测结果回调"""
        # 清空前一次的结果
        self.object_positions.clear()
        self.latest_positions.clear()
        
        # 处理每个检测结果
        for detection in msg.detections:
            # 确保有检测结果
            if not detection.results:
                continue
                
            # 获取第一个结果（通常只有一个）
            result = detection.results[0]
            obj_id = result.id
            
            # 获取位置信息
            position = result.pose.pose.position
            x, y, z = position.x, position.y, position.z
            
            # 存储位置
            self.object_positions[obj_id].append((x, y, z))
            self.latest_positions[obj_id] = (x, y, z)
        
        # 打印调试信息（可选）
        #self.print_positions()
    
    def get_positions_by_id(self, obj_id):
        """获取特定ID的所有位置"""
        return self.object_positions.get(obj_id, [])
    
    def get_latest_position_by_id(self, obj_id):
        """获取特定ID的最新位置"""
        return self.latest_positions.get(obj_id, None)
    
    def get_all_positions(self):
        """获取所有检测到的位置"""
        return dict(self.object_positions)
    
    def get_all_latest_positions(self):
        """获取所有检测的最新位置"""
        return dict(self.latest_positions)
    
    def print_positions(self):
        """打印位置信息（调试用）"""
        if not self.object_positions:
            rospy.loginfo("No objects detected")
            return
            
        rospy.loginfo("=== Detected Objects ===")
        for obj_id, positions in self.object_positions.items():
            rospy.loginfo(f"ID {obj_id}:")
            for i, (x, y, z) in enumerate(positions):
                rospy.loginfo(f"  Object {i+1}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        rospy.loginfo("========================")


######################### 运动控制api部分 ###########################################

class KuavoMotionController:
    def __init__(self):
        # 初始化ROS节点
        if not rospy.get_node_uri():
            rospy.init_node('kuavo_motion_controller', anonymous=True)
        
        self.rate = rospy.Rate(10)  # 10 Hz

        # 手部控制api
        self.hand_control_pub = rospy.Publisher('control_robot_hand_position', robotHandPosition, queue_size=10)
        self.hand_control_msg = robotHandPosition()
        # 等待订阅者连接
        # while hand_control_pub.get_num_connections() == 0 and not rospy.is_shutdown():
        #     rospy.loginfo("等待 control_robot_hand_position 订阅者连接...")
        #     rate.sleep()
        
        # 手臂控制api
        self.arm_control_pub = rospy.Publisher('kuavo_arm_target_poses', armTargetPoses, queue_size=10)
        self.arm_control_msg = armTargetPoses()
        # 等待订阅者连接
        while self.arm_control_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("等待 kuavo_arm_target_poses 订阅者连接...")
            self.rate.sleep()

        # 腰部控制api
        self.waist_control_pub = rospy.Publisher('/robot_waist_motion_data', robotWaistControl, queue_size=10)
        self.waist_control_msg = robotWaistControl()
        # 等待订阅者连接
        while self.waist_control_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("等待 /robot_waist_motion_data 订阅者连接中...")
            self.rate.sleep()

        # 头部控制api
        self.head_control_pub = rospy.Publisher('robot_head_motion_data', robotHeadMotionData, queue_size=10)
        self.head_control_msg = robotHeadMotionData()
        # 等待订阅者连接
        while self.head_control_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("等待 robot_head_motion_data 订阅者连接...")
            self.rate.sleep()

        rospy.loginfo("KuavoMotionController initialized.")

    # 发布手部目标姿态函数
    def publish_hand_target_pos(self, left_hand_param, right_hand_param):
        self.hand_control_msg.left_hand_position = left_hand_param   # 赋值左手参数
        self.hand_control_msg.right_hand_position = right_hand_param # 赋值右手参数
        self.hand_control_pub.publish(self.hand_control_msg)              # 发布消息
        rospy.loginfo("手部目标位置指令发布完成")

    # 发布手臂目标姿态函数
    def publish_arm_target_poses(self, times, values):
        self.arm_control_msg.times = times
        self.arm_control_msg.values = values
        self.arm_control_pub.publish(self.arm_control_msg)
        # rospy.loginfo("move msg publish over")

    # 发布腰部目标角度函数
    def publish_waist_target_angle(self, target_angle_deg):
        self.waist_control_msg.header.stamp = rospy.Time.now()
        self.waist_control_msg.data.data = [target_angle_deg]
        self.waist_control_pub.publish(self.waist_control_msg)
        # rospy.loginfo(f"腰部角度指令发布完成 | 目标角度：{target_angle_deg}° ")

    # 发布头部目标姿态函数
    def publish_head_target_pos(self, yaw, pitch):
        self.head_control_msg.joint_data = [yaw, pitch]
        self.head_control_pub.publish(self.head_control_msg)
        rospy.loginfo("头部目标位置指令发布完成")

    # 平滑发布发布腰部目标角度函数
    def publish_waist_target_angle_smooth(self, current_angle_deg, target_angle_deg, step, rate):
        """
        平滑发布腰部目标角度，逐步从当前角度过渡到目标角度
        :param current_angle_deg: 当前腰部角度（单位：度）
        :param target_angle_deg: 目标腰部角度（单位：度）
        :param step: 每次步进的角度（单位：度，需为正数）
        :param rate: ROS Rate对象,控制发布频率(如rospy.Rate(10)表示10Hz)
        """
        # 输入参数校验，避免无效步长
        if step <= 0:
            rospy.logerr("步长(step)必须为正数！")
            return
        
        # 如果当前角度已等于目标角度，直接返回
        if abs(current_angle_deg - target_angle_deg) < 1e-6:
            rospy.loginfo(f"当前角度{current_angle_deg}°已等于目标角度{target_angle_deg}°，无需发布")
            return
        
        # 计算角度差值，确定步进方向（正向/负向）
        angle_diff = target_angle_deg - current_angle_deg
        step_direction = 1 if angle_diff > 0 else -1  # 1为增加角度，-1为减少角度
        current_step_angle = current_angle_deg  # 初始化当前步进角度
        
        # 循环逐步发布角度，直到接近目标角度
        while not rospy.is_shutdown():
            # 计算下一步的角度
            next_angle = current_step_angle + step * step_direction
            
            # 判断下一步角度是否超出目标角度，若超出则直接设为目标角度
            if (step_direction == 1 and next_angle >= target_angle_deg) or \
               (step_direction == -1 and next_angle <= target_angle_deg):
                next_angle = target_angle_deg
            
            # 发布当前步进角度
            self.publish_waist_target_angle(next_angle)
            
            # 更新当前步进角度
            current_step_angle = next_angle
            
            # 若已到达目标角度，退出循环
            if abs(current_step_angle - target_angle_deg) < 1e-6:
                rospy.loginfo(f"已平滑到达腰部角度：{target_angle_deg}°")
                break
            
            # 按照指定频率休眠，控制发布速度
            rate.sleep()