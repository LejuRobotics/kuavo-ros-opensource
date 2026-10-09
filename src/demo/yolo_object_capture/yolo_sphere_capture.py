#!/usr/bin/env python3

import rospy

import math
import numpy as np  # 引入numpy库用于数值计算
import os
import yaml
import argparse
from kuavo_msgs.msg import twoArmHandPoseCmd

from utils.tools import get_version_parameter, \
                        call_ik_srv, ik_solve_param, set_arm_control_mode, \
                        cal_target_with_rotation, \
                        euler_to_quaternion_via_matrix, \
                        ObjectPositionTracker, \
                        KuavoMotionController

####################################################################

# 橙子ID
yolo_object_id = 49

# 自定义ik参数
use_custom_ik_param = True
# 使用默认的关节角度作为ik的初始预测
joint_angles_as_q0 = True 

# 手部开合控制
# close_hand = [30, 100, 30, 30, 30, 30]    # 虚握, 测试用
close_hand = [42, 100, 42, 42, 42, 42]    # catch pose
open_hand = [0, 100, 0, 0, 0, 0]          # open pose
zero_hand = [0, 0, 0, 0, 0, 0]          # zero pose

# 单臂7轴复位角度
ARM_RESET_POSE = [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0]    
ARM_ZERO_POSE = [6.0, 0.0, 0.0, -20.0, 0.0, 0.0, 0.0]    

########################### 主函数 #########################################

def main():

    # 创建ObjectPositionTracker实例 并初始化ROS节点
    processor = ObjectPositionTracker()

########################### 程序参数配置 #########################################

    # 解析命令行参数  
    parser = argparse.ArgumentParser(description="程序参数参数配置")
    parser.add_argument("--use_offset", action="store_true", help="是否启用抓取偏移量(默认 False)")
    parser.add_argument("--no_waist", action="store_true", help="是否关闭自动转腰功能(默认 False)")
    args = parser.parse_args()
    
    # 加载配置文件
    current_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(current_dir, "config", "offset.yaml")
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)
    targets_cfg = cfg.get("targets", {})

    # 参数赋值
    target_id = str(yolo_object_id)
    if args.use_offset and target_id in targets_cfg:
        rospy.loginfo(f"=========== 启用抓取偏移量 | target_id={target_id} =============")
        tcfg = targets_cfg[target_id]
        offset_z = tcfg["offset"]["z"]
        temp_x_l = tcfg["offset"]["left"]["x"]
        temp_y_l = tcfg["offset"]["left"]["y"]
        temp_x_r = tcfg["offset"]["right"]["x"]
        temp_y_r = tcfg["offset"]["right"]["y"]
        offset_angle = tcfg.get("offset_angle", 1.0)
    else:
        rospy.loginfo("=========== 不启用抓取偏移量 =============")
        offset_z = 0.0
        temp_x_l = temp_y_l = temp_x_r = temp_y_r = 0.0
        offset_angle = 1.0

    # 获取机器人版本
    robot_version = get_version_parameter()
    #不同型号机器人的初始位置 (机器人坐标系)
    if robot_version == 52 or 53 or 54:
        robot_zero_x = -0.003 
        robot_zero_y = -0.2527
        robot_zero_z = -0.3144 
    else :
        rospy.logerr("机器人版本号错误, 仅支持 52 53 54 系列")
        return

##########################################   运动控制 api准备   #########################################
    
    # 创建KuavoMotionController实例
    motion_controller = KuavoMotionController()

##########################################   寻找YOLOv8结果   #########################################

    # 腰部归位, 低头
    motion_controller.publish_waist_target_angle(0.0)  # 初始腰部归零
    rospy.sleep(2.0)  # 等待腰部运动到位
    motion_controller.publish_head_target_pos(0, 20)
    rospy.loginfo("head down")
    rospy.sleep(1.5)

    # 设置要监控的物体ID
    target_ids = [yolo_object_id]
    
    # 循环等待检测结果
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        yolo_object_flag = False
        # 获取特定ID的最新位置
        for obj_id in target_ids:
            position = processor.get_latest_position_by_id(obj_id)
            if position:
                if obj_id == yolo_object_id:
                    yolo_object_flag = True
                    yolo_object_x, yolo_object_y, yolo_object_z = position
                    rospy.loginfo(f"Position of ID {obj_id}: "
                                f"x={yolo_object_x:.3f}, y={yolo_object_y:.3f}, z={yolo_object_z:.3f}")
        # 识别到标签则进行下一步
        if yolo_object_flag == True:
            break
        rospy.loginfo_throttle(2.0, f"未检测到YOLO ID {target_ids}，等待中...") # 每 2 秒最多输出一次
        rate.sleep()

    # YOLO检测期间 按ctrl+c可以退出程序
    if rospy.is_shutdown():
        return None

########################################## 运动控制 ik求解 #########################################

    rospy.loginfo("=========== 开始ik求解流程 ===========")
    # 创建请求对象
    eef_pose_msg = twoArmHandPoseCmd()
    # 设置请求参数
    eef_pose_msg.ik_param = ik_solve_param
    eef_pose_msg.use_custom_ik_param = use_custom_ik_param
    eef_pose_msg.joint_angles_as_q0 = joint_angles_as_q0
    # joint_angles_as_q0 为 False 时，这两个参数不会被使用（单位：弧度）
    eef_pose_msg.hand_poses.left_pose.joint_angles = np.array([-0.377, 0.109, -0.310, -1.845, -0.749, -1.309, 0.698])
    eef_pose_msg.hand_poses.right_pose.joint_angles = np.array([-0.377, -0.109, 0.310, -1.845, 0.749, 1.309, 0.698])
    # 定义ik请求函数, 便于多次调用
    def calculate_ik_result(position_flag, set_x, set_y, set_z):
        # 根据左右手 计算ik参数
        is_left_target = position_flag > 0
        sign_coeff = -1 if is_left_target else 1  # 左手目标为-1，右手目标为1
        target_hand = eef_pose_msg.hand_poses.left_pose if is_left_target else eef_pose_msg.hand_poses.right_pose
        non_target_hand = eef_pose_msg.hand_poses.right_pose if is_left_target else eef_pose_msg.hand_poses.left_pose
        # 非目标手配置
        non_target_hand.pos_xyz = np.array([ robot_zero_x, sign_coeff * robot_zero_y, robot_zero_z + 0.05])
        non_target_hand.quat_xyzw = [0.0,0.0,0.0,1.0]
        non_target_hand.elbow_pos_xyz = np.zeros(3)

        # 目标手位置配置
        target_hand.pos_xyz = np.array([set_x,set_y,set_z])
        #计算末端相对角度
        relative_angle = math.atan(sign_coeff * (set_y-robot_zero_y)/(set_x-robot_zero_x))
        rospy.loginfo(f"relative_angle: {relative_angle}")
        #quat=euler_to_quaternion_via_matrix(relative_angle*offset_angle, -1.57 , 0)
        quat=euler_to_quaternion_via_matrix(relative_angle*offset_angle, -1.57 , 0, sign_coeff * 1.57, 1.57/2, sign_coeff * 0 ) # 末端姿态额外调整
        target_hand.quat_xyzw = [quat.x,quat.y,quat.z,quat.w]
        target_hand.elbow_pos_xyz = np.zeros(3)
        
        rospy.loginfo("ik请求目标点x y z:")
        rospy.loginfo(f"set_x: {set_x}, set_y: {set_y}, set_z: {set_z}")
        
        res = call_ik_srv(eef_pose_msg)
        return res

    # 判断左手还是右手 position_flag > 0 为左手，否则为右手
    # 若要固定用哪只手抓取, 在这里固定position_flag的值即可
    if False:
        position_flag=-1
    else :
        position_flag=yolo_object_y
    hand = "左手" if position_flag > 0 else "右手"
    rospy.loginfo(f"使用{hand}抓取")

    # 抓取位置修正
    set_x = yolo_object_x + (temp_x_l if position_flag > 0 else temp_x_r)
    set_y = yolo_object_y + (temp_y_l if position_flag > 0 else temp_y_r)
    set_z = yolo_object_z + offset_z 

    # 转腰抓取点计算
    if args.no_waist==False:
        rospy.logwarn("未设置no_waist,尝试计算拟转腰抓取点")
        exist, target_point, rotate_angle = cal_target_with_rotation(
            a=0.40, b=-1*robot_zero_y, r=-1*robot_zero_y, m=set_x, n=set_y)
        if exist:
            rospy.loginfo(f"拟抓取目标点坐标：{target_point}")
            rospy.loginfo(f"拟抓取旋转角度(°)(目标点->初始点):{rotate_angle}")
            set_x, set_y = target_point
        else:
            rospy.logwarn("拟计算抓取点失败,尝试不转腰抓取")
    else :
        rospy.logwarn("已设置no_waist,尝试不转腰抓取")
        exist = False
    # 两次ik求解
    rospy.loginfo("第一次ik:")
    res_1st=calculate_ik_result(position_flag, set_x, set_y, set_z + 0.15)  # 进行第一次求解
    rospy.loginfo("第二次ik:")
    res_2nd=calculate_ik_result(position_flag, set_x, set_y, set_z)

    # 两次逆解均成功
    if(res_1st.success and res_2nd.success):
        rospy.loginfo("两次ik均成功,开始控制流程")
########################################## 展示ik结果 ####################################################
        
        l_pos = res_1st.hand_poses.left_pose.pos_xyz
        l_pos_error = np.linalg.norm(l_pos - eef_pose_msg.hand_poses.left_pose.pos_xyz)
        r_pos = res_1st.hand_poses.right_pose.pos_xyz
        r_pos_error = np.linalg.norm(r_pos - eef_pose_msg.hand_poses.right_pose.pos_xyz)
        
        # 打印部分逆解结果
        # rospy.loginfo("第一次ik求解结果:")
        # rospy.loginfo(f"time_cost: {res_1st.time_cost:.2f} ms. left_pos_error: {1e3*l_pos_error:.2f} mm, right_pos_error: {1e3*r_pos_error:.2f} mm")
        # rospy.loginfo(f"left_joint_angles: {res_1st.hand_poses.left_pose.joint_angles}")
        # rospy.loginfo(f"right_joint_angles: {res_1st.hand_poses.right_pose.joint_angles}")
        # rospy.loginfo(f"res_1st.q_arm: {res_1st.q_arm}")

        l_pos_2 = res_2nd.hand_poses.left_pose.pos_xyz
        l_pos_error_2 = np.linalg.norm(l_pos_2 - eef_pose_msg.hand_poses.left_pose.pos_xyz)
        r_pos_2 = res_2nd.hand_poses.right_pose.pos_xyz
        r_pos_error_2 = np.linalg.norm(r_pos_2 - eef_pose_msg.hand_poses.right_pose.pos_xyz)
        
        # 打印部分逆解结果
        # rospy.loginfo("第二次ik求解结果:")
        # rospy.loginfo(f"time_cost: {res_2nd.time_cost:.2f} ms. left_pos_error: {1e3*l_pos_error_2:.2f} mm, right_pos_error: {1e3*r_pos_error_2:.2f} mm")
        # rospy.loginfo(f"left_joint_angles: {res_2nd.hand_poses.left_pose.joint_angles}")
        # rospy.loginfo(f"right_joint_angles: {res_2nd.hand_poses.right_pose.joint_angles}")
        # rospy.loginfo(f"res_2nd.q_arm: {res_2nd.q_arm}")

########################################## 运动控制 准备姿态 #########################################
        
        # 设置手臂运动模式为外部控制
        set_arm_control_mode(2)
        rospy.sleep(1.0)

        # 双手归零
        motion_controller.publish_hand_target_pos(zero_hand, zero_hand)

        # 初始位置
        motion_controller.publish_arm_target_poses([1.5], ARM_RESET_POSE + ARM_RESET_POSE)
        rospy.loginfo("回到初始位置")

        rospy.sleep(1.5)
        rospy.loginfo("=========== 开始移动到准备姿态 ===========")
        # 到达等待位置
        if  position_flag > 0 :
            # 预抓取姿势
            motion_controller.publish_arm_target_poses([1.5], [40, 20, 0, -120, 0, 0, -20] + ARM_RESET_POSE)
            rospy.sleep(1.5) 
            # motion_controller.publish_arm_target_poses([2.5], [-21.600, 6.245, -17.762, -105.711, -42.914, -74.999, 40.000] + ARM_RESET_POSE)
            # rospy.sleep(2.5) 
            rospy.loginfo("已移动到准备姿态")
            # 左手松开
            motion_controller.publish_hand_target_pos(open_hand, zero_hand)
        else :
            # 预抓取姿势 
            motion_controller.publish_arm_target_poses([1.5], ARM_RESET_POSE + [40, -20, 0, -120, 0, 0, -20])
            rospy.sleep(1.5) 
            # motion_controller.publish_arm_target_poses([2.5], ARM_RESET_POSE + [-21.600, -6.245, 17.762, -105.711, 42.914, 74.999, 40.000])
            # rospy.sleep(3.5) 
            rospy.loginfo("已移动到准备姿态")
            # 右手松开
            motion_controller.publish_hand_target_pos(zero_hand, open_hand)
        rospy.sleep(0.5)
        
########################################## 运动控制 执行ik结果 #########################################
        rospy.loginfo("=========== 开始执行ik结果 ===========")
        rospy.loginfo("开始第一次ik运动")
        if  position_flag > 0 :
            joint_end_angles = np.concatenate([res_1st.hand_poses.left_pose.joint_angles, [0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0]])
        else :
            joint_end_angles = np.concatenate([ [0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0], res_1st.hand_poses.right_pose.joint_angles])
        degrees_list = [math.degrees(rad) for rad in joint_end_angles]
        # 调用函数并传入times和values
        motion_controller.publish_arm_target_poses([2.5], degrees_list)
        if exist:
            motion_controller.publish_waist_target_angle_smooth(        
                current_angle_deg=0.0, target_angle_deg=rotate_angle,
                step=1.0, rate=rospy.Rate(10)
            )
        rospy.sleep(2)
        rospy.loginfo("完成逆解并根据逆解结果到达定位置")
        
        rospy.loginfo("开始第二次ik运动")
        if  position_flag > 0 :
            joint_end_angles = np.concatenate([res_2nd.hand_poses.left_pose.joint_angles, [0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0]])
        else :
            joint_end_angles = np.concatenate([ [0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0], res_2nd.hand_poses.right_pose.joint_angles])
        degrees_list = [math.degrees(rad) for rad in joint_end_angles]
        # 调用函数并传入times和values
        motion_controller.publish_arm_target_poses([2], degrees_list)
        rospy.sleep(3.5)
        rospy.loginfo("完成逆解并根据逆解结果到达定位置")
        
        rospy.loginfo("ik结束")
        
########################################## 运动控制 抬手展示流程 #########################################    
        rospy.loginfo("=========== 开始抬手展示流程 ===========")    
        if  position_flag > 0 :
            # 手部握紧
            motion_controller.publish_hand_target_pos(close_hand, zero_hand)
            rospy.sleep(1)   
            motion_controller.publish_arm_target_poses([1.5], [-60.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0] + ARM_RESET_POSE)
            rospy.sleep(0.5)
            if exist:
                motion_controller.publish_waist_target_angle_smooth(        
                    current_angle_deg=rotate_angle, target_angle_deg=0.0,
                    step=1.7, rate=rospy.Rate(10)
                )
            rospy.sleep(1)
            # 手部松开
            motion_controller.publish_hand_target_pos(open_hand, zero_hand)
            rospy.sleep(1) 
        else :
            # 手部握紧
            motion_controller.publish_hand_target_pos(zero_hand, close_hand)
            rospy.sleep(1)   
            motion_controller.publish_arm_target_poses([1.5], ARM_RESET_POSE + [-60.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            rospy.sleep(0.5)
            if exist:
                motion_controller.publish_waist_target_angle_smooth(        
                    current_angle_deg=rotate_angle, target_angle_deg=0.0,
                    step=1.7, rate=rospy.Rate(10)
                )
            rospy.sleep(1)
            # 手部松开 
            motion_controller.publish_hand_target_pos(zero_hand, open_hand)
            rospy.sleep(1) 
        rospy.loginfo("递送完成")

        # 松手后多等两秒
        # rospy.sleep(2) 
    ########################################## 运动控制 后续处理 #########################################
        rospy.loginfo("=========== 开始手臂复位流程 ===========")  
        # 手臂复位
        my_reset_time=1.5
        if  position_flag > 0 :
            motion_controller.publish_arm_target_poses([my_reset_time], [6.0, 50.0, 0.0, -90.0, 0.0, 0.0, 0.0] + ARM_RESET_POSE)
            rospy.sleep(my_reset_time) 
            motion_controller.publish_arm_target_poses([my_reset_time], [6.0, 50.0, 0.0, -20.0, 0.0, 0.0, 0.0] + ARM_RESET_POSE)
            rospy.sleep(my_reset_time) 
        else :
            motion_controller.publish_arm_target_poses([my_reset_time], ARM_RESET_POSE + [6.0, -50.0, 0.0, -90.0, 0.0, 0.0, 0.0])
            rospy.sleep(my_reset_time) 
            motion_controller.publish_arm_target_poses([my_reset_time], ARM_RESET_POSE + [6.0, -50.0, 0.0, -20.0, 0.0, 0.0, 0.0])
            rospy.sleep(my_reset_time)  
        # 双手归零
        motion_controller.publish_hand_target_pos(zero_hand, zero_hand)
    # ik失败
    else :
        rospy.logerr("ik失败,程序退出")
########################################## 流程结束 后续处理 #########################################
    # 回到初始位置
    motion_controller.publish_arm_target_poses([1.5], ARM_ZERO_POSE + ARM_ZERO_POSE)

    # 恢复抬头
    motion_controller.publish_head_target_pos(0, 0)
    rospy.loginfo("head reset")

    # 设置手臂控制模式 恢复为 行走时自动摆手
    set_arm_control_mode(1)
    rospy.sleep(0.5)
    rospy.loginfo("=========== 流程结束 ===========")
if __name__ == '__main__':
    main()
    