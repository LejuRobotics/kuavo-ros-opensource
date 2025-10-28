#!/usr/bin/env python3

import rospy
from kuavo_msgs.msg import AprilTagDetectionArray
import math
import numpy as np  # 引入numpy库用于数值计算

import time
import argparse
from kuavo_msgs.msg import armTargetPoses
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse
from motion_capture_ik.srv import twoArmHandPoseCmdSrv
from motion_capture_ik.msg import twoArmHandPoseCmd, ikSolveParam

from kuavo_msgs.srv import controlLejuClaw, controlLejuClawRequest, controlLejuClawResponse
from kuavo_msgs.msg import robotHeadMotionData


####################################################################

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

# 夹爪参数
claw_open=[40,40]
claw_close=[75,75]

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

# 发布手臂目标姿态
def publish_arm_target_poses(times, values):
    # 创建Publisher对象
    pub = rospy.Publisher('kuavo_arm_target_poses', armTargetPoses, queue_size=10)
    rospy.sleep(0.5)  # 确保Publisher注册
    # 创建消息对象并设置传入的times和values
    msg = armTargetPoses()
    msg.times = times
    msg.values = values

    rospy.loginfo("正在将手臂目标姿态发布到话题 'kuavo_arm_target_poses'")

    # 等待订阅者连接
    rate = rospy.Rate(10)  # 10Hz
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("等待订阅者连接...")
        rate.sleep()

    # 发布消息
    pub.publish(msg)
    rospy.loginfo("消息已发布。")

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

######################### 识别apriltag标签部分 ###########################################

class AprilTagProcessor:
    def __init__(self, is_init=False):
        """
        初始化AprilTagProcessor类。
        :param is_init: 是否初始化ROS节点,默认为False。
        """
        if is_init:
            rospy.init_node('tag_detections_listener', anonymous=True)

    def quaternion_to_euler(self, w, x, y, z):
        """
        将四元数转换为欧拉角（yaw）。
        :param w, x, y, z: 四元数的分量。
        :return: 以度为单位的yaw角。
        """
        # 计算roll, pitch, yaw
        sinr_cosp = 2 * (w * z + x * y)
        cosr_cosp = 1 - 2 * (y**2 + z**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        #print(f"roll: {math.degrees(roll)}")
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)
        #print(f"pitch: {math.degrees(pitch)}")
        siny_cosp = 2 * (w * x + y * z)
        cosy_cosp = 1 - 2 * (x**2 + y**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        #print(f"yaw: {math.degrees(yaw)}")
        return math.degrees(yaw)

    def get_apriltag_data(self):
        """
        从指定的ROS话题中获取AprilTag检测数据。
        :return: 包含每个AprilTag信息的列表。
        """
        try:
            # 等待从话题"/robot_tag_info"接收到AprilTagDetectionArray消息
            msg = rospy.wait_for_message("/robot_tag_info", AprilTagDetectionArray, timeout=5)
        except rospy.ROSException as e:
            rospy.logerr(f"未能获取到 AprilTag 数据: {e}")
            return None

        data_list = []
        for detection in msg.detections:
            id = detection.id[0]  # 获取AprilTag的ID
            quaternion = detection.pose.pose.pose.orientation  # 获取姿态的四元数
            pos = detection.pose.pose.pose.position  # 获取位置

            # 将四元数转换为yaw角
            yaw_angle = self.quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            
            # 构建AprilTag数据字典
            tag_data = {
                "id": id,
                "off_horizontal": round(pos.x, 3),
                "off_camera": round(pos.y, 3),
                "off_vertical": round(pos.z, 3),
                "yaw_angle": yaw_angle
            }
            data_list.append(tag_data)

        return data_list

    def get_apriltag_by_id(self, tag_id):
        """
        根据ID获取特定的AprilTag数据。
        :param tag_id: 要查找的AprilTag的ID。
        :return: 匹配的AprilTag数据字典。
        """
        all_tags = self.get_apriltag_data()
        if all_tags is None:
            rospy.logerr("未能获取到 AprilTag 数据")
            return None

        for tag in all_tags:
            if tag["id"] == tag_id:
                return tag

        return None

    def get_averaged_apriltag_data(self, tag_id, num_samples=10):
        """
        获取指定ID的AprilTag的平均位置和姿态数据。
        :param tag_id: 要查找的AprilTag的ID。
        :param num_samples: 用于计算平均值的样本数量，默认为10。
        :return: 包含平均位置和姿态的字典。
        """
        data_list = []

        while len(data_list) < num_samples:
            if rospy.is_shutdown():
                return None
            tag_data = self.get_apriltag_by_id(tag_id)
            if tag_data:
                data_list.append(tag_data)
            else :
                rospy.loginfo(f"未检测到AprilTag ID {tag_id}，等待中...")
                time.sleep(0.1) 

        # 使用numpy计算平均值
        avg_off_horizontal = np.mean([tag["off_horizontal"] for tag in data_list])
        avg_off_camera = np.mean([tag["off_camera"] for tag in data_list])
        avg_off_vertical = np.mean([tag["off_vertical"] for tag in data_list])
        avg_yaw_angle = np.mean([tag["yaw_angle"] for tag in data_list])
        
        #print(f"avg_yaw_angle: {avg_yaw_angle}")
        result = {
            "id": tag_id,
            "avg_off_horizontal": round(avg_off_horizontal, 3),
            "avg_off_camera": round(avg_off_camera, 3),
            "avg_off_vertical": round(avg_off_vertical, 3),
            "avg_yaw_angle": round(avg_yaw_angle, 3)
        }
        rospy.loginfo(f"AprilTag ID: {result['id']}, 位置: x={result['avg_off_horizontal']}, y={result['avg_off_camera']}, z={result['avg_off_vertical']}, 倾斜角: {result['avg_yaw_angle']}")
        return result

########################### 主函数 #########################################

def main():

    # 创建AprilTagProcessor实例 并初始化ROS节点
    processor = AprilTagProcessor(is_init=True)

    # 解析命令行参数  
    parser = argparse.ArgumentParser(description="是否启用偏移量")
    parser.add_argument("--offset_start", type=str, choices=["False", "True"], required="True", help="选择 offset_start = True or Flase")
    args = parser.parse_args()

    # offset_start="True"表示启用偏移量 否则不启用偏移量
    if args.offset_start == "True":
        # 偏向侧后边一点
        offset_z=-0.08  # 抓取点位于标签正下方
        temp_x_l=-0.035
        temp_y_l=0.0
        temp_x_r=-0.01
        temp_y_r=0.0
    else :
        offset_z=0.00
        temp_x_l=0.00
        temp_y_l=0.00
        temp_x_r=0.00
        temp_y_r=0.00

    # 角度偏移量（修正绕z轴的偏移角度）
    offset_angle=1.00

    # 低头
    set_head_target(0, 20)
    print("head down")
    time.sleep(2)

##########################################   寻找AprilTag码   #########################################

    # 获取指定ID的AprilTag的平均数据
    tag_data = processor.get_averaged_apriltag_data(tag_id=0)

    # 检测二维码期间 按ctrl+c可以退出程序
    if rospy.is_shutdown():
        return None

##########################################   参数准备   #########################################

    # 判断左手还是右手 后续都会根据这个参数进行判断
    # position_flag > 0 为左手，否则为右手
    # 若要固定用哪只手抓取, 在这里固定position_flag的值即可
    if False:
        position_flag=-1
    else :
        position_flag=tag_data['avg_off_camera']*100
        
    print(f"tag position: {position_flag}")

    # 获取机器人版本
    robot_version = get_parameter('robot_version')
    #不同型号机器人的初始位置 (机器人坐标系)
    def start_with_version(version_number:int, series:int):
        """判断版本号是否属于某系列"""
        # PPPPMMMMN
        MMMMN_MASK = 100000
        return (version_number % MMMMN_MASK) == series
    if start_with_version(robot_version, 45) or start_with_version(robot_version, 49):
        robot_zero_x = -0.0173
        robot_zero_y = -0.2927
        robot_zero_z = -0.2837  
        
    elif start_with_version(robot_version, 42):
        robot_zero_x = -0.0175
        robot_zero_y = -0.25886
        robot_zero_z = -0.20115

    else :
        print("机器人版本号错误, 仅支持 42 45 49 系列")


    # 设置手臂运动模式为外部控制
    set_arm_control_mode(2)

    # 手部控制api
    # 创建请求对象
    claw_control_msg = controlLejuClawRequest()
    claw_control_msg.data.name = ['left_claw', 'right_claw']
    claw_control_msg.data.position = claw_close
    claw_control_msg.data.velocity = [50, 50]
    claw_control_msg.data.effort = [1.0, 1.0]
    
    #确保服务启动
    rospy.wait_for_service('/control_robot_leju_claw')
    #调用服务并获取响应
    control_leju_claw = rospy.ServiceProxy('/control_robot_leju_claw', controlLejuClaw)

########################################## 运动控制 ik求解 #########################################
    # 创建请求对象
    eef_pose_msg = twoArmHandPoseCmd()
    # 设置请求参数
    eef_pose_msg.ik_param = ik_solve_param
    eef_pose_msg.use_custom_ik_param = use_custom_ik_param
    eef_pose_msg.joint_angles_as_q0 = joint_angles_as_q0
    # joint_angles_as_q0 为 False 时，这两个参数不会被使用（单位：弧度）
    eef_pose_msg.hand_poses.left_pose.joint_angles = np.zeros(7)
    eef_pose_msg.hand_poses.right_pose.joint_angles = np.zeros(7)

    # 抓取位置修正
    if  position_flag > 0 :
        # 设置左手末端执行器的位置
        set_x=tag_data['avg_off_horizontal']+temp_x_l
        set_y=tag_data['avg_off_camera']+temp_y_l
        set_z=tag_data['avg_off_vertical']+offset_z
    else :
        # 设置右手末端执行器的位置
        set_x=tag_data['avg_off_horizontal']+temp_x_r
        set_y=tag_data['avg_off_camera']-temp_y_r
        set_z=tag_data['avg_off_vertical']+offset_z

    # 根据左右手 计算ik参数
    if  position_flag > 0 :
        # 设置左手末端执行器的位置和姿态
        # 使用set_xyz
        eef_pose_msg.hand_poses.left_pose.pos_xyz = np.array([set_x,set_y,set_z])
        #计算末端相对角度
        relative_angle= math.atan((robot_zero_y-set_y)/(set_x-robot_zero_x))
        print(f"relative_angle: {relative_angle}")
        #计算四元数
        #quat=euler_to_quaternion_via_matrix(relative_angle*offset_angle, -1.57 , 0)
        quat=euler_to_quaternion_via_matrix(relative_angle*offset_angle, -1.57 , 0, 1.57, 0, 0 )
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = [quat.x,quat.y,quat.z,quat.w]  # 带yaw角
        #eef_pose_msg.hand_poses.left_pose.quat_xyzw = [-0.4996018366446333, -0.49999984146591725, 0.49999984146591725, 0.5003981633553666]  # 水平状态
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)
        
        # 右手为机器人初始位置
        eef_pose_msg.hand_poses.right_pose.pos_xyz = np.array([ robot_zero_x, robot_zero_y, robot_zero_z])
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = [0.0,0.0,0.0,1.0]  # 竖直状态
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)
    else :
        # 左手为机器人初始位置 
        eef_pose_msg.hand_poses.left_pose.pos_xyz = np.array([ robot_zero_x, -1*robot_zero_y, robot_zero_z])
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = [0.0,0.0,0.0,1.0]   # 竖直状态
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)

        # 设置右手末端执行器的位置和姿态
        # 使用set_xyz
        eef_pose_msg.hand_poses.right_pose.pos_xyz = np.array([set_x,set_y,set_z])
        # 计算末端相对角度
        relative_angle=math.atan((set_y-robot_zero_y)/(set_x-robot_zero_x))
        print(f"relative_angle: {relative_angle}")
        # 计算四元数
        #quat=euler_to_quaternion_via_matrix(relative_angle*offset_angle, -1.57 , 0)
        quat=euler_to_quaternion_via_matrix(relative_angle*offset_angle, -1.57 , 0, 1.57, 0, 0 )
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = [quat.x,quat.y,quat.z,quat.w]  # 带yaw角
        #eef_pose_msg.hand_poses.right_pose.quat_xyzw =[0.4996018366446333, -0.49999984146591725, -0.49999984146591725, 0.5003981633553666]  # 水平状态
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
        
        # 夹爪张开
        claw_control_msg.data.position = claw_open
        control_leju_claw(claw_control_msg)

        # 初始位置
        print("move to position 0")
        publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
        20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
        print("回到初始位置")

        time.sleep(0.5)
        
        # 到达等待位置
        if  position_flag > 0 :
            # 打开臂展
            print("move to position 1")
            publish_arm_target_poses([1.5], [20.0, 60.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(1.5)
            # 弯曲肘部
            print("move to position 2")
            publish_arm_target_poses([1.5], [0.0, 60.0, 0.0, -90.0, 0.0, 0.0, 0.0,
            20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(1.5) 
            # 回收臂展 
            #print("move to position 3")
            #publish_arm_target_poses([1.5], [-2.0, 5.0, 0.0, -82.0, -35.0, -7.0, 0.0,
            #20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            #time.sleep(2) 
            print("move over")
        else :
            # 打开臂展
            print("move to position 1")
            publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            20.0, -60.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            time.sleep(1.5)
            # 弯曲肘部
            print("move to position 2")
            publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            40.0, 0.0, 0.0, -130.0, 90.0, 0.0, 0.0])
            #0.0, -60.0, 0.0, -90.0, 0.0, 0.0, 0.0])
            time.sleep(1.5) 
            #time.sleep(3) 
            # 回收臂展 
            #print("move to position 3")
            #publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            #8.0, -10.0, 0.0, -90.0, 35.0, 0.0, 0.0])
            #time.sleep(2) 
            print("move over")

########################################## 运动控制 执行ik结果 #########################################

        # 0.35 0.52
        if  position_flag > 0 :
            joint_end_angles = np.concatenate([res.hand_poses.left_pose.joint_angles, [0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0]])
        else :
            joint_end_angles = np.concatenate([ [0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0], res.hand_poses.right_pose.joint_angles])
        #joint_end_angles = res.hand_poses.left_pose.joint_angles + res.hand_poses.right_pose.joint_angles
        degrees_list = [math.degrees(rad) for rad in joint_end_angles]
        # 调用函数并传入times和values
        publish_arm_target_poses([3], degrees_list)
        print("完成逆解并根据逆解结果到达定位置")
        time.sleep(5)

        print("ik结束")

########################################## 运动控制 递水流程 #########################################        
        if  position_flag > 0 :
            # 夹爪闭合
            claw_control_msg.data.position = claw_close
            control_leju_claw(claw_control_msg)  
            time.sleep(1)   
            publish_arm_target_poses([1.5], [-60.0, 0.0, 0.0, -30.0, -20.0, 0.0, 0.0,
                20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(2.5)
            # 夹爪张开
            claw_control_msg.data.position = claw_open
            control_leju_claw(claw_control_msg) 
            time.sleep(1) 
            #publish_arm_target_poses([1.5], degrees_list)
            #time.sleep(2)
            #time.sleep(1)     
        else :
            # 夹爪闭合
            claw_control_msg.data.position = claw_close
            control_leju_claw(claw_control_msg)  
            time.sleep(1)   
            publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                -60.0, 0.0, 0.0, -30.0, 90.0, 0.0, 0.0])
            time.sleep(2.5)
            # 夹爪张开
            claw_control_msg.data.position = claw_open
            control_leju_claw(claw_control_msg)  
            time.sleep(1) 
            #publish_arm_target_poses([1.5], degrees_list)
            #time.sleep(2) 
            #time.sleep(1)
        print("递水完成")

        # 松手后多等一秒
        time.sleep(1) 
    ########################################## 运动控制 后续处理 #########################################
        # 手臂复位 与前面的角度相对应
        if  position_flag > 0 :
            #publish_arm_target_poses([1.5], [0.0, 0.0, 0.0, -90.0, 0.0, 0.0, 0.0,
            #20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            #time.sleep(1.5) 
            publish_arm_target_poses([1.5], [0.0, 60.0, 0.0, -90.0, 0.0, 0.0, 0.0,
            20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(1.5) 
            publish_arm_target_poses([1.5], [20.0, 60.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(1.5) 
        else :
            #publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            #0.0, 0.0, 0.0, -90.0, 0.0, 0.0, 0.0])
            #time.sleep(1.5) 
            publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            0.0, -75.0, 0.0, -90.0, 0.0, 0.0, 0.0])
            time.sleep(1.5) 
            publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            0.0, -75.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            time.sleep(1.5) 
    # ik失败
    else :
        print("ik失败,程序退出")
########################################## 流程结束 后续处理 #########################################
    # 回到初始位置
    publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0, 
                                20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])

    # 恢复抬头
    set_head_target(0, 0)
    print("head reset")

    time.sleep(1.5)    

    # 设置手臂控制模式 恢复为 行走时自动摆手
    set_arm_control_mode(1)

if __name__ == '__main__':
    main()
    