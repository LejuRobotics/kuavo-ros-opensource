#!/usr/bin/env python3

import rospy
#from kuavo_msgs.msg import AprilTagDetectionArray
from vision_msgs.msg import Detection2DArray
from collections import defaultdict

import math
import numpy as np  # 引入numpy库用于数值计算

import time
import argparse
from kuavo_msgs.msg import armTargetPoses
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse
from kuavo_msgs.srv import twoArmHandPoseCmdSrv
from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam

from kuavo_msgs.msg import robotHandPosition
from kuavo_msgs.msg import robotHeadMotionData


####################################################################

# 自定义ik参数
use_custom_ik_param = True
# 使用默认的关节角度作为ik的初始预测
joint_angles_as_q0 = True 
# 创建ikSolverParam对象
ik_solve_param = ikSolveParam()
# 设置ikSolveParam对应参数
ik_solve_param.major_optimality_tol = 1e-3
ik_solve_param.major_feasibility_tol = 1e-3
ik_solve_param.minor_feasibility_tol = 1e-3
ik_solve_param.major_iterations_limit = 500
ik_solve_param.oritation_constraint_tol= 1e-3
ik_solve_param.pos_constraint_tol = 1e-3 
ik_solve_param.pos_cost_weight = 0.0 

# 手部开合控制
close_hand = [80, 100, 80, 75, 75, 75]    # catch pose
open_hand = [0, 100, 0, 0, 0, 0]          # open pose
zero_hand = [0, 0, 0, 0, 0, 0]          # zero pose

# 头部抬头低头控制
def set_head_target(yaw, pitch):
    """
    设置头部目标位置，并发布消息
    :param yaw: 头部的偏航角，范围为[-30, 30]度
    :param pitch: 头部的俯仰角，范围为[-25, 25]度
    """
    
    # 创建一个发布者，发布到'/robot_head_motion_data'话题
    # 使用robotHeadMotionData消息类型，队列大小为10
    pub_head_pose = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=10)
    rospy.sleep(0.5)  # 确保Publisher注册
    # 创建一个robotHeadMotionData消息对象
    head_target_msg = robotHeadMotionData()
    
    # 设置关节数据，包含偏航和俯仰角
    # 确保yaw在[-30, 30]范围内，pitch在[-25, 25]范围内
    head_target_msg.joint_data = [yaw, pitch]
    
    # 发布消息到指定话题
    pub_head_pose.publish(head_target_msg)
    
    # 打印日志信息，显示已发布的头部目标位置
    rospy.loginfo(f"Published head target: yaw={yaw}, pitch={pitch}")

######################## ik求解部分 ############################################

# 获取机器人版本
def get_version_parameter():
    param_name = 'robot_version'
    try:
        # 获取参数值
        param_value = rospy.get_param(param_name)
        rospy.loginfo(f"参数 {param_name} 的值为: {param_value}")
        # 适配1000xx版本号
        valid_series = [42, 45, 49, 52]
        MMMMN_MASK = 100000
        series = param_value % MMMMN_MASK
        if series not in valid_series:
            rospy.logerr(f"无效的机器人版本号: {param_value}，仅支持 {valid_series} 系列！程序退出。")
            rospy.signal_shutdown("参数无效")
        else:
            rospy.loginfo(f"✅ 机器人版本号有效: {param_value}")
        return param_value
    except rospy.ROSException:
        rospy.logerr(f"参数 {param_name} 不存在！程序退出。")
        rospy.signal_shutdown("参数获取失败") 
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
        print("Service call failed: %s"%e)
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

######################### 识别YOLOV8标签部分 ###########################################

class ObjectPositionTracker:
    def __init__(self):
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

########################### 主函数 #########################################

def main():

    # 创建ObjectPositionTracker实例 并初始化ROS节点
    processor = ObjectPositionTracker()

    # 解析命令行参数  
    parser = argparse.ArgumentParser(description="是否启用偏移量")
    parser.add_argument("--offset_start", type=str, choices=["False", "True"], required="True", help="选择 offset_start = True or Flase")
    args = parser.parse_args()

    # offset_start="True"表示启用偏移量 否则不启用偏移量
    if args.offset_start == "True":
        offset_z=0.033
        temp_x_l=-0.016
        temp_y_l=0.038
        temp_x_r=-0.016 
        temp_y_r=0.038
    else :
        offset_z = temp_x_l = temp_y_l = temp_x_r = temp_y_r = 0.0

    # 角度偏移量（修正绕z轴的偏移角度）
    offset_angle=1.00

    # 低头
    set_head_target(0, 20)
    print("head down")
    time.sleep(1.5)

##########################################   寻找YOLOv8结果   #########################################

    # 设置要监控的物体ID
    # target_ids = [39, 41]  # 瓶子和杯子
    target_ids = [39]  # 只检测瓶子
    # 主循环
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        cup_flag = False
        bottle_flag = False
        # 获取特定ID的最新位置
        for obj_id in target_ids:
            position = processor.get_latest_position_by_id(obj_id)
            if position:
                # if obj_id == 41: # 杯子
                #     cup_flag = True
                #     cup_x, cup_y, cup_z = position
                #     rospy.loginfo(f"Position of ID {obj_id}: "
                #                 f"x={cup_x:.3f}, y={cup_y:.3f}, z={cup_z:.3f}")
                if obj_id == 39: # 瓶子
                    bottle_flag = True
                    bottle_x, bottle_y, bottle_z = position
                    rospy.loginfo(f"Position of ID {obj_id}: "
                                f"x={bottle_x:.3f}, y={bottle_y:.3f}, z={bottle_z:.3f}")
        # 两个标签识别到其一,则进行下一步(目前只支持识别瓶子)
        if cup_flag == True or bottle_flag == True:
            break
        rate.sleep()

##########################################   参数准备   #########################################

    # 判断左手还是右手 后续都会根据这个参数进行判断
    # position_flag > 0 为左手，否则为右手
    # 若要固定用哪只手抓取, 在这里固定position_flag的值即可
    if False:
        position_flag=-1
    else :
        position_flag=bottle_y
        
    print(f"tag position: {position_flag}")

    # 获取机器人版本
    robot_version = get_version_parameter()
    #不同型号机器人的初始位置 (机器人坐标系)
    if robot_version == 52:
        robot_zero_x = -0.012
        robot_zero_y = -0.255 + 0.03
        robot_zero_z = -0.315

    else :
        print("机器人版本号错误, 仅支持 52 系列")
        return


    # 设置手臂运动模式为外部控制
    set_arm_control_mode(2)

    # 手部控制api
    # 初始化话题发布者
    hand_control_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)
    # 创建消息对象
    hand_control_msg = robotHandPosition()
    
    # 手臂控制api
    arm_control_pub = rospy.Publisher('kuavo_arm_target_poses', armTargetPoses, queue_size=10)
    arm_control_msg = armTargetPoses()
    # 等待订阅者连接,检查机器人是否启动
    rate = rospy.Rate(10)  # 10Hz
    while arm_control_pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("等待 kuavo_arm_target_poses 订阅者连接...")
        rate.sleep()
    # 发布手臂目标姿态
    def publish_arm_target_poses(times, values):
        arm_control_msg.times = times
        arm_control_msg.values = values
        arm_control_pub.publish(arm_control_msg)
        rospy.loginfo("move msg publish over")

########################################## 运动控制 ik求解 #########################################
    # 创建请求对象
    eef_pose_msg = twoArmHandPoseCmd()
    # 设置请求参数
    eef_pose_msg.ik_param = ik_solve_param
    eef_pose_msg.use_custom_ik_param = use_custom_ik_param
    eef_pose_msg.joint_angles_as_q0 = joint_angles_as_q0
    # joint_angles_as_q0 为 False 时，这两个参数不会被使用（单位：弧度）
    eef_pose_msg.hand_poses.left_pose.joint_angles = np.array([0.0, 0.0, 0.0, -1.57079633, 0.0, 0.0, 0.0])
    eef_pose_msg.hand_poses.right_pose.joint_angles = np.array([0.0, 0.0, 0.0, -1.57079633, 0.0, 0.0, 0.0])

    # 抓取位置修正
    if  position_flag > 0 :
        # 设置左手末端执行器的位置
        set_x=bottle_x+temp_x_l
        set_y=bottle_y+temp_y_l
        set_z=bottle_z+offset_z
    else :
        # 设置右手末端执行器的位置
        set_x=bottle_x+temp_x_r
        set_y=bottle_y-temp_y_r
        set_z=bottle_z+offset_z

    # 根据左右手 计算ik参数
    if  position_flag > 0 :
        # 设置左手末端执行器的位置和姿态
        # 使用set_xyz
        eef_pose_msg.hand_poses.left_pose.pos_xyz = np.array([set_x,set_y,set_z])
        #计算末端相对角度
        relative_angle= math.atan((robot_zero_y-set_y)/(set_x-robot_zero_x))
        print(f"relative_angle: {relative_angle}")
        #计算四元数
        quat=euler_to_quaternion_via_matrix(relative_angle*offset_angle, -1.57 , 0)
        #quat=euler_to_quaternion_via_matrix(relative_angle*offset_angle, -1.57 , 0, 1.57, 0, 0 ) # 末端姿态额外调整
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = [quat.x,quat.y,quat.z,quat.w]  # 带yaw角
        #eef_pose_msg.hand_poses.left_pose.quat_xyzw = [-0.5, -0.5, 0.5, 0.5]  # 水平状态
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)
        
        # 右手为机器人初始位置
        eef_pose_msg.hand_poses.right_pose.pos_xyz = np.array([ robot_zero_x, robot_zero_y, robot_zero_z + 0.05])
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = [0.0,0.0,0.0,1.0]  # 竖直状态
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)
    else :
        # 左手为机器人初始位置 
        eef_pose_msg.hand_poses.left_pose.pos_xyz = np.array([ robot_zero_x, -1*robot_zero_y, robot_zero_z + 0.05])
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = [0.0,0.0,0.0,1.0]   # 竖直状态
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)

        # 设置右手末端执行器的位置和姿态
        # 使用set_xyz
        eef_pose_msg.hand_poses.right_pose.pos_xyz = np.array([set_x,set_y,set_z])
        # 计算末端相对角度
        relative_angle=math.atan((set_y-robot_zero_y)/(set_x-robot_zero_x)) # 基础偏移量
        print(f"relative_angle: {relative_angle}")
        # 计算四元数
        quat=euler_to_quaternion_via_matrix(relative_angle*offset_angle, -1.57 , 0)
        #quat=euler_to_quaternion_via_matrix(relative_angle*offset_angle, -1.57 , 0, 1.57, 0, 0 ) # 末端姿态额外调整
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = [quat.x,quat.y,quat.z,quat.w]  # 带yaw角
        #eef_pose_msg.hand_poses.right_pose.quat_xyzw =[0.5, -0.5, -0.5, 0.5]  # 水平状态
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)
    
    print("抓取点x y z")
    print(set_x," , ",set_y," , ",set_z)

    # 调用 IK 逆解服务
    res = call_ik_srv(eef_pose_msg)

    # 逆解成功
    if(res.success):
########################################## 展示ik结果 ####################################################
        
        l_pos = res.hand_poses.left_pose.pos_xyz
        l_pos_error = np.linalg.norm(l_pos - eef_pose_msg.hand_poses.left_pose.pos_xyz)
        r_pos = res.hand_poses.right_pose.pos_xyz
        r_pos_error = np.linalg.norm(r_pos - eef_pose_msg.hand_poses.right_pose.pos_xyz)
        
        # 打印部分逆解结果
        print(f"time_cost: {res.time_cost:.2f} ms. left_pos_error: {1e3*l_pos_error:.2f} mm, right_pos_error: {1e3*r_pos_error:.2f} mm")
        print(f"left_joint_angles: {res.hand_poses.left_pose.joint_angles}")
        print(f"right_joint_angles: {res.hand_poses.right_pose.joint_angles}")
        print(f"res.q_arm: {res.q_arm}")
        
########################################## 运动控制 准备姿态 #########################################
        
        # 双手归零
        hand_control_msg.left_hand_position =zero_hand  # 左手位置   
        hand_control_msg.right_hand_position = zero_hand  # 右手位置
        hand_control_pub.publish(hand_control_msg)  # 发布消息

        # 初始位置
        print("move to position 0")
        publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
        20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
        print("回到初始位置")

        time.sleep(0.5)
        
        # 到达等待位置
        if  position_flag > 0 :
            # 直接提肘 
            print("move to position 3")
            publish_arm_target_poses([2.5], [40, 20, 0, -120, 0, 0, -20,
            20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(2) 
            print("move over")
            # 左手松开
            hand_control_msg.left_hand_position =open_hand  # 左手位置   
            hand_control_pub.publish(hand_control_msg)  # 发布消息
        else :
            # 直接提肘 
            print("move to position 3")
            publish_arm_target_poses([2.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            40, -20, 0, -120, 0, 0, -20])
            time.sleep(2) 
            print("move over")
            # 右手松开
            hand_control_msg.right_hand_position = open_hand  # 右手位置
            hand_control_pub.publish(hand_control_msg)  # 发布消息
        time.sleep(1.5)
########################################## 运动控制 执行ik结果 #########################################
        
        # 0.35 0.52
        if  position_flag > 0 :
            joint_end_angles = np.concatenate([res.hand_poses.left_pose.joint_angles, [0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0]])
        else :
            joint_end_angles = np.concatenate([ [0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0], res.hand_poses.right_pose.joint_angles])
        #joint_end_angles = res.hand_poses.left_pose.joint_angles + res.hand_poses.right_pose.joint_angles
        degrees_list = [math.degrees(rad) for rad in joint_end_angles]
        # 调用函数并传入times和values
        publish_arm_target_poses([2.5], degrees_list)
        print("完成逆解并根据逆解结果到达定位置")
        time.sleep(5)

        print("ik结束")

########################################## 运动控制 递水流程 #########################################        
        if  position_flag > 0 :
            # 手部握紧
            hand_control_msg.left_hand_position =close_hand  # 左手位置   
            hand_control_pub.publish(hand_control_msg)  # 发布消息
            time.sleep(1)   
            publish_arm_target_poses([2], [-60.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(3)
            # 手部松开
            hand_control_msg.left_hand_position =open_hand  # 左手位置   
            hand_control_pub.publish(hand_control_msg)  # 发布消息 
            time.sleep(1) 
        else :
            hand_control_msg.right_hand_position = close_hand  # 右手位置
            hand_control_pub.publish(hand_control_msg)  # 发布消息
            time.sleep(1)   
            publish_arm_target_poses([2], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                -60.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(3)
            # 手部松开 
            hand_control_msg.right_hand_position = open_hand  # 右手位置
            hand_control_pub.publish(hand_control_msg)  # 发布消息 
            time.sleep(1) 
        print("递水完成")

        # 松手后多等一秒
        time.sleep(2) 
    ########################################## 运动控制 后续处理 #########################################
        # 手臂复位
        if  position_flag > 0 :
            publish_arm_target_poses([2.5], [6.0, 50.0, 0.0, -90.0, 0.0, 0.0, 0.0,
            20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(2.5) 
            publish_arm_target_poses([2.5], [6.0, 50.0, 0.0, -20.0, 0.0, 0.0, 0.0,
            20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(2.5) 
        else :
            publish_arm_target_poses([2.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            6.0, -50.0, 0.0, -90.0, 0.0, 0.0, 0.0])
            time.sleep(2.5) 
            publish_arm_target_poses([2.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            6.0, -50.0, 0.0, -20.0, 0.0, 0.0, 0.0])
            time.sleep(2.5) 
        # 双手归零
        hand_control_msg.left_hand_position =zero_hand  # 左手位置   
        hand_control_msg.right_hand_position = zero_hand  # 右手位置
        hand_control_pub.publish(hand_control_msg)  # 发布消息
    # ik失败
    else :
        print("ik失败,程序退出")
########################################## 流程结束 后续处理 #########################################
    # 回到初始位置
    publish_arm_target_poses([1.5], [6.0, 0.0, 0.0, -20.0, 0.0, 0.0, 0.0, 
                                6.0, 0.0, 0.0, -20.0, 0.0, 0.0, 0.0])

    # 恢复抬头
    set_head_target(0, 0)
    print("head reset")

    time.sleep(1.5)    

    # 设置手臂控制模式 恢复为 行走时自动摆手
    set_arm_control_mode(1)

if __name__ == '__main__':
    main()
    