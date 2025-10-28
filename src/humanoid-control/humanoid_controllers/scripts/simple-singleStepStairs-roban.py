#!/usr/bin/env python3
"""
增强版6D步态控制器
集成自动步态控制功能，保留斜坡功能，替换单步移动功能
"""

import rospy
import numpy as np
import sys
import time
from kuavo_msgs.msg import footPose6DTargetTrajectories, footPose6D
from std_srvs.srv import SetBool, SetBoolRequest

def get_user_input():
    """获取用户输入"""
    print("\n" + "="*60)
    print("增强版6D步态控制器 (集成自动步态)")
    print("="*60)
    print("请选择要执行的功能:")
    print("┌─" + "─"*56 + "─┐")
    print("│ 1. 上楼梯 (stair) - 6D轨迹控制                    │")
    print("│ 2. 退出                                             │")
    print("└─" + "─"*56 + "─┘")
    
    while True:
        choice = input("\n请输入选择 (1-2): ").strip()
        
        if choice == '1':
            return 'stair'
        elif choice == '2':
            return 'quit'
        else:
            print("无效选择，请输入 1-2")

def set_pitch_limit(enable):
    """设置基座俯仰角限制"""
    rospy.wait_for_service('/humanoid/mpc/enable_base_pitch_limit')
    try:
        set_pitch_limit_service = rospy.ServiceProxy('/humanoid/mpc/enable_base_pitch_limit', SetBool)
        req = SetBoolRequest()
        req.data = enable
        resp = set_pitch_limit_service(req)
        if resp.success:
            rospy.loginfo(f"成功{'启用' if enable else '禁用'}pitch限制")
        else:
            rospy.logwarn(f"失败{'启用' if enable else '禁用'}pitch限制")
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")

class EnhancedSlopeController:
    """增强版斜坡控制器，集成自动步态控制"""
    
    def __init__(self):
        # 全局高度管理
        self.swing_height = 0.12
        self.torso_height = 0.0
        
        # 轨迹发布器 (用于斜坡控制)
        self.traj_pub = rospy.Publisher('/humanoid_mpc_foot_pose_6d_target_trajectories', 
                                       footPose6DTargetTrajectories, queue_size=1)
        
        # 等待发布器准备就绪
        rospy.sleep(0.5)
    
    def simple_up_stairs(self, stair_height, stair_length, stair_num, foot_length):  # 输入x方向距离，坡度，除第一步和最后一步外，机器人需要进行的步数，机器人脚掌长度
        
        """完整的楼梯轨迹生成函数

        Args:
            stair_height: 单级楼梯高度
            stair_length: 单级楼梯长度（深度）
            stair_num: 楼梯级数（需要迈步的次数）
            foot_length: 机器人脚掌长度
        """
        
        # 两脚之间的宽度
        foot_width = 0.108536  # 脚宽度
        
        # 脚掌距离斜坡的初始距离
        foot_distance_initial = 0.03
        
        # 脚末端到脚掌中心的 x 方向距离
        foot_offset = 0.0185
        foot_toe = 0.126
        foot_heel = -0.089
        
        # 数据结构初始化
        time_traj = []
        foot_idx_traj = []  # 默认左脚
        foot_poses_6d = []
        torso_poses_6d = []
        
        # 迈步时间间隔
        swing_time = 0.6  # 基础时间间隔
        contact_time = 0.8
        
        # 除第一步外的步进距离
        step_x_increment = stair_length
        step_z_increment = stair_height
        
        # 躯干位置偏置
        torso_offset = -0.02
        
        # 生成每一步的轨迹
        step_x_init = foot_distance_initial + foot_toe + stair_length/2 - foot_offset
        step_z_init = stair_height
        torso_x_init = step_x_init + torso_offset
        torso_z_init = stair_height + torso_offset
        step_swing_time = swing_time
        
        # 存储上一摆动相的足端高度，用于接触相
        last_step_x = step_x_init
        last_step_z = step_z_init
        
        # 存储当前躯干位置
        current_torso_x = 0
        current_torso_z = 0
        
        # 添加最后的并拢步
        total_phases = stair_num * 2 + 1  # 包括所有楼梯步和最后的并拢步
    
        for step in range(total_phases):  # 生成第一步和上斜坡的步态
            print("step_swing_time: ", step_swing_time)
            if step < stair_num * 2:    # 楼梯步态
                if step % 2 == 0:  # 摆动相（实际迈步）
                    step_index = step // 2
                    foot_index = step_index % 2  # 0:左脚, 1:右脚

                    if step_index == 0:
                        # 第一步：准备姿势
                        time_traj.append(step_swing_time)
                        foot_idx_traj.append(foot_index)
                        foot_poses_6d.append([
                            step_x_init, foot_width, step_z_init, 0.0, 0.0, 0.0
                        ])
                        torso_poses_6d.append([
                            current_torso_x, 0.0, current_torso_z, 0.0, 0.0, 0.0
                        ])
                        current_torso_x += torso_x_init
                        current_torso_z += torso_z_init
                    else:
                        # 正常迈步
                        time_traj.append(step_swing_time)
                        foot_idx_traj.append(foot_index)

                        # 更新下一步位置
                        step_x_new = step_x_init + step_x_increment * step_index
                        step_z_new = step_z_init + step_z_increment * step_index

                        # y坐标：左脚在右侧，右脚在左侧
                        y_pos = foot_width if foot_index == 0 else -foot_width

                        # 足部姿态
                        foot_poses_6d.append([
                            step_x_new, y_pos, step_z_new, 0.0, 0.0, 0.0
                        ])

                        # 躯干姿态
                        torso_poses_6d.append([
                            current_torso_x, 0.0, current_torso_z, 0.0, 0.0, 0.0
                        ])
                        
                        current_torso_x = torso_x_init + step_x_increment * (step_index)
                        current_torso_z = torso_z_init + step_z_increment * (step_index)
                        
                        last_step_z = step_z_new
                        last_step_x = step_x_new

                    step_swing_time += contact_time

                else:  # 接触相（双脚着地）
                    step_index = (step - 1) // 2
                    time_traj.append(step_swing_time)
                    foot_idx_traj.append(2)  # 接触相标志

                    # 保持当前位置不变
                    if step_index == 0:
                        foot_poses_6d.append([
                            step_x_init, 0.0, step_z_init, 0.0, 0.0, 0.0
                        ])
                        torso_poses_6d.append([
                            current_torso_x, 0.0, current_torso_z, 0.0, 0.0, 0.0
                        ])
                    else:
                        step_x_current = last_step_x
                        step_z_current = last_step_z

                        # 保持双脚着地状态
                        foot_poses_6d.append([
                            step_x_current, 0.0, step_z_current, 0.0, 0.0, 0.0
                        ])
                        torso_poses_6d.append([
                            current_torso_x, 0.0, current_torso_z, 0.0, 0.0, 0.0
                        ])

                    step_swing_time += swing_time
            else:
                step_index = step // 2
                foot_index = step_index % 2  # 0:左脚, 1:右脚
                # y坐标：左脚在右侧，右脚在左侧
                y_pos = foot_width if foot_index == 0 else -foot_width
                
                # 更新最后一步位置
                step_x_last = step_x_init + step_x_increment * (stair_num-1)
                step_z_last = step_z_init + step_z_increment * (stair_num-1)
                torso_x_last = torso_x_init + step_x_increment * (stair_num-1)
                torso_z_last = torso_z_init + step_z_increment * (stair_num-1)
                
                time_traj.append(step_swing_time)
                foot_idx_traj.append(foot_index)
                foot_poses_6d.append([
                    step_x_last, y_pos, step_z_last, 0.0, 0.0, 0.0
                ])
                torso_poses_6d.append([
                    torso_x_last, 0.0, torso_z_last, 0.0, 0.0, 0.0
                ])
                
        return self.get_foot_pose_6d_traj_msg(time_traj, foot_idx_traj, foot_poses_6d, torso_poses_6d)
    
    def get_foot_pose_6d_traj_msg(self, time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_height_traj=None):
        """
        创建6D足部姿态目标轨迹消息

        Args:
            time_traj: 时间轨迹列表
            foot_idx_traj: 足部索引轨迹列表
            foot_traj_6d: 6D足部姿态轨迹列表，每个元素为[x, y, z, yaw, pitch, roll]
            torso_traj_6d: 6D躯干姿态轨迹列表，每个元素为[x, y, z, yaw, pitch, roll]
            swing_height_traj: 摆动高度轨迹列表（可选）

        Returns:
            footPose6DTargetTrajectories消息
        """
        num = len(time_traj)
        
        # 创建消息实例
        msg = footPose6DTargetTrajectories()
        msg.timeTrajectory = time_traj  # 设置时间轨迹
        msg.footIndexTrajectory = foot_idx_traj  # 设置脚索引
        msg.footPoseTrajectory = []  # 初始化脚姿态轨迹

        for i in range(num):
            # 创建6D脚姿态信息
            foot_pose_msg = footPose6D()
            foot_pose_msg.footPose6D = foot_traj_6d[i]  # 设置6D脚姿态 [x, y, z, yaw, pitch, roll]
            foot_pose_msg.torsoPose6D = torso_traj_6d[i]  # 设置6D躯干姿态 [x, y, z, yaw, pitch, roll]

            # 将脚姿态添加到消息中
            msg.footPoseTrajectory.append(foot_pose_msg)
        
        self.torso_height = 0.0
        # 设置摆动高度轨迹（如果提供）
        if swing_height_traj is not None:
            msg.swingHeightTrajectory = swing_height_traj
        else:
            # 默认摆动高度
            msg.swingHeightTrajectory = [self.swing_height] * num

        return msg
    
    def publish_trajectory(self, msg):
        """发布轨迹消息"""
        if msg is not None:
            self.traj_pub.publish(msg)
            print("轨迹消息发布成功")
            return True
        else:
            print("错误: 轨迹消息为空")
            return False
        
def main():
    # 初始化ROS节点
    rospy.init_node('enhanced_slope_controller', anonymous=True)
    
    print("增强版6D步态控制器启动")
    print("正在连接ROS节点...")
    
    # 禁用俯仰角限制
    set_pitch_limit(False)
    # 创建增强控制器
    controller = EnhancedSlopeController()
    
    # ========== 所有参数统一设置 ==========
    
    # 楼梯参数
    STAIR_HEIGHT = 0.08
    STAIR_LENGTH = 0.25
    STAIR_NUM = 4
    
    # 机器人参数
    FOOT_LENGTH = 0.215     # 机器人脚掌长度
    
    msg_6d_slope = None
    
    while not rospy.is_shutdown():
        try:
            # 获取用户选择
            command = get_user_input()
            
            if command == 'quit':
                print("退出程序...")
                break
            
            elif command == 'stair':
                msg_6d_slope = controller.simple_up_stairs(STAIR_HEIGHT, STAIR_LENGTH, STAIR_NUM, FOOT_LENGTH)
            
            # 发布轨迹 (仅用于斜坡控制)
            controller.publish_trajectory(msg_6d_slope)
                
        except KeyboardInterrupt:
            print("\n用户中断执行")
            break
        except Exception as e:
            print(f"执行出错: {e}")
            import traceback
            traceback.print_exc()
            continue

if __name__ == '__main__':
    main() 
