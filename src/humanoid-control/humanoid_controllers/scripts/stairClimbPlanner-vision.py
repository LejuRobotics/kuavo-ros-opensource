#!/usr/bin/env python3

import rospy
import numpy as np
from kuavo_msgs.msg import footPoseWithVisionArray, footPoseTargetTrajectories, footPose, footPoses, footPose6D, footPose6DTargetTrajectories, footPoses6D
from stairClimbPlanner import StairClimbingPlanner
from std_srvs.srv import SetBool, SetBoolRequest
import signal
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped
import sys
import os
import argparse
from ssh_executor import SSHExecutor
from std_msgs.msg import Float32MultiArray
SCRIPT_PATH = os.path.dirname(os.path.abspath(__file__))
REMOTE_CONFIG_PATH = os.path.join(SCRIPT_PATH, "remote-config.json")
TIME_OUT = 10
class VisionBasedStairClimbingPlanner:
    def __init__(self, debug_mode=False):
        # 初始化ROS节点
        rospy.init_node('vision_based_stair_climbing_planner', anonymous=True)
        
        # 设置debug模式
        self.debug_mode = debug_mode
        
        # 创建规划器实例
        self.planner = StairClimbingPlanner()
        
        # 创建发布器
        self.trajectory_pub = rospy.Publisher(
            '/humanoid_mpc_foot_pose_6d_target_trajectories',
            footPose6DTargetTrajectories,
            queue_size=10
        )
        rospy.sleep(1) # 等待1秒，确保发布器已经初始化
        
        # 初始化TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 等待TF树建立
        rospy.loginfo("Waiting for TF tree to be established...")
        try:
            self.tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(5.0))
            rospy.loginfo("TF tree established successfully")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to establish TF tree: {e}")
            return
        
        # 创建订阅器，接收楼梯yaw角
        self.yaw_sub = rospy.Subscriber(
            "/stair_segments_yaw", 
            Float32MultiArray, 
            self.yaw_callback
        )
        
        # 创建订阅器
        self.subscriber = rospy.Subscriber(
            '/leju/odom/foot_pose_with_vision_array',
            footPoseWithVisionArray,
            self.vision_callback,
            queue_size=1
        )
        
        # 初始化变量
        self.stair_info = None
        self.is_planning = False
        self.planning_done = False
        self.filtered_yaw=0
        self.vision_received = False  # 添加标志位，用于跟踪是否收到视觉数据
        
        # 等待服务
        rospy.wait_for_service('/humanoid/mpc/enable_base_pitch_limit')
        self.set_pitch_limit_service = rospy.ServiceProxy('/humanoid/mpc/enable_base_pitch_limit', SetBool)
        
        # 设置超时定时器
        self.timeout_timer = rospy.Timer(rospy.Duration(TIME_OUT), self.timeout_callback, oneshot=True)
        rospy.loginfo(f"Set timeout timer for {TIME_OUT} seconds")
        
        if self.debug_mode:
            rospy.loginfo("Debug mode enabled - trajectory will not be published")

    def yaw_callback(self, msg):
        """处理接收到的Yaw角消息，去除极值后取平均"""
        if not msg.data:
            rospy.loginfo("Received empty Yaw data")
            return
        
        yaw_data = msg.data
        
        # 检查数据点数量
        if len(yaw_data) < 3:
            rospy.loginfo(f"Not enough data points ({len(yaw_data)}) to filter, using raw data")
            self.filtered_yaw = sum(yaw_data) / len(yaw_data)
        else:
            # 去除最大值和最小值
            sorted_yaw = sorted(yaw_data)
            filtered_data = sorted_yaw[1:-1]  # 去除第一个（最小）和最后一个（最大）
            
            # 计算平均值
            self.filtered_yaw = sum(filtered_data) / len(filtered_data)
            
            # rospy.loginfo(f"Filtered Yaw: raw={yaw_data}, filtered={self.filtered_yaw:.3f}")
        
    def get_base_link_pose(self):
        """获取base_link相对于odom的位姿"""
        try:
            # 使用最新的可用时间
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0))
            return transform.transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF error: {e}")
            return None
            
    def transform_point_to_base_link(self, point_odom):
        """将odom坐标系下的点转换到base_link坐标系"""
        try:
            # 创建PointStamped消息
            point_stamped = PointStamped()
            point_stamped.header.frame_id = "odom"
            point_stamped.header.stamp = rospy.Time(0)  # 使用最新的可用时间
            point_stamped.point.x = point_odom[0]
            point_stamped.point.y = point_odom[1]
            point_stamped.point.z = point_odom[2]
            
            # 转换到base_link坐标系
            point_base_link = self.tf_buffer.transform(point_stamped, "base_link", rospy.Duration(0.1))
            return np.array([point_base_link.point.x, point_base_link.point.y, point_base_link.point.z])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF error: {e}")
            return None
            
    def set_pitch_limit(self, enable):
        """设置基座俯仰角限制"""
        try:
            req = SetBoolRequest()
            req.data = enable
            resp = self.set_pitch_limit_service(req)
            if resp.success:
                rospy.loginfo(f"Successfully {'enabled' if enable else 'disabled'} pitch limit")
            else:
                rospy.logwarn(f"Failed to {'enable' if enable else 'disable'} pitch limit")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            
    def vision_callback(self, msg):
        """处理视觉识别结果"""
        # 设置标志位，表示已收到视觉数据
        self.vision_received = True
        
        if self.is_planning or self.planning_done:
            return
            
        # 检查是否有楼梯信息
        if len(msg.foot_poses) == 0:
            rospy.logwarn("No stair information received")
            return
            
        # 获取楼梯信息
        points = []
        for foot_pose in msg.foot_poses:
            if abs(foot_pose.footPose[2]) > 0.03:  # 只处理高度差大于3cm的台阶
                points.append(np.array(foot_pose.footPose))
                
        if len(points) == 0:
            rospy.logwarn("No valid stair steps found")
            return
            
        self.stair_info = points
        rospy.loginfo(f"Received stair information: {len(self.stair_info)} steps")
        
        # 打印接收到的坐标
        for i, point in enumerate(points):
            rospy.loginfo(f"  台阶 {i+1}: ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})")
        
        # 获取第一个台阶在base_link坐标系下的位置
        first_step_base_link = self.transform_point_to_base_link(points[0])
        second_step_base_link = self.transform_point_to_base_link(points[1]) if len(points) > 1 else None
        if first_step_base_link is None:
            rospy.logerr("Failed to transform first step position to base_link frame")
            return
        
        self.yaw_error=np.arctan2(second_step_base_link[1] - first_step_base_link[1], second_step_base_link[0] - first_step_base_link[0]) if second_step_base_link is not None else np.arctan2(first_step_base_link[1], first_step_base_link[0])
        print(f"Yaw error to first step: {self.yaw_error:.3f} radians")

        # 计算到第一个台阶的距离
        distance_to_first_step = np.linalg.norm(first_step_base_link[:2])  # 只考虑x-y平面距离
        rospy.loginfo(f"Distance to first step: {distance_to_first_step:.3f}m")
        
        # 开始规划
        self.plan_stair_climbing(distance_to_first_step)
        
        # 标记规划完成
        self.planning_done = True
        rospy.loginfo("Planning and publishing completed, waiting for Ctrl+C to exit...")
        
    def plan_stair_climbing(self, distance_to_first_step):
        """基于视觉信息规划上楼梯动作"""
        if self.stair_info is None:
            rospy.logwarn("No stair information available for planning")
            return
            
        self.is_planning = True
        
        try:
            # 禁用俯仰角限制
            self.set_pitch_limit(False)
            
            # 获取楼梯信息
            num_steps = len(self.stair_info)
            rospy.loginfo(f"Planning for {num_steps} steps")
            
            # 初始化轨迹变量
            time_traj = []
            foot_idx_traj = []
            foot_traj = []
            torso_traj = []
            swing_trajectories = []

            print("yaw:",self.filtered_yaw)
            
            # 如果距离太远，先规划走到楼梯前
            safe_distance = 0.385 # 半截楼梯宽度0.15 + 脚尖到base的距离0.15左右 + 距离楼梯前边缘安全距离
            if distance_to_first_step > safe_distance: 
                rospy.loginfo(f"Distance to first step ({distance_to_first_step:.3f}m) is too far, planning approach movement")
                print("distance_to_first_step", distance_to_first_step - safe_distance)
                time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = self.planner.plan_move_to(
                    dx=distance_to_first_step - safe_distance,
                    dy=0,
                    # dyaw=0,  # Convert radians to degrees
                    dyaw=np.degrees(self.filtered_yaw),  
                    time_traj=time_traj,
                    foot_idx_traj=foot_idx_traj,
                    foot_traj=foot_traj,
                    torso_traj=torso_traj,
                    swing_trajectories=swing_trajectories
                )
            
            # 规划上楼梯动作
            time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = self.planner.plan_up_stairs(
                num_steps=num_steps+1,
                time_traj=time_traj,
                foot_idx_traj=foot_idx_traj,
                foot_traj=foot_traj,
                torso_traj=torso_traj,
                swing_trajectories=swing_trajectories
            )
            
            # 规划转身
            # time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = self.planner.plan_move_to(0.2,0,0, time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)
            # time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = self.planner.plan_move_to(0.0,0,180, time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)
            # time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = self.planner.plan_move_to(0.16,0,0, time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)
            # print("\nMove to down stairs plan done.")
            if (time_traj is not None):
                for i,t in enumerate(time_traj):
                    print(f"{i:2}:{t:3.2f} {foot_idx_traj[i]} {foot_traj[i]} {torso_traj[i]}")
            
            # 发布轨迹
            self.publish_trajectory(time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)
            
            # 等待轨迹执行完成
            self.wait_for_trajectory_completion(time_traj)
            
            self.planning_done = True
            rospy.loginfo("Stair climbing planning completed")
            
        except Exception as e:
            rospy.logerr(f"Error during planning: {e}")
        finally:
            self.is_planning = False
            
    def publish_trajectory(self, time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories):
        """发布轨迹到控制器"""
        if self.debug_mode:
            rospy.loginfo("Debug mode: Skipping trajectory publication")
            return
            
        msg = footPose6DTargetTrajectories()
        msg.timeTrajectory = time_traj
        msg.footIndexTrajectory = foot_idx_traj
        msg.footPoseTrajectory = []
        msg.additionalFootPoseTrajectory = []

        for i in range(len(time_traj)):
            foot_pose_msg = footPose6D()
            # 将4D数据转换为6D格式 [x, y, z, yaw] -> [x, y, z, yaw, pitch, roll]
            foot_pose_msg.footPose6D = [*foot_traj[i], 0.0, 0.0]  # 添加pitch和roll为0
            foot_pose_msg.torsoPose6D = [*torso_traj[i], 0.0, 0.0]  # 添加pitch和roll为0
            msg.footPoseTrajectory.append(foot_pose_msg)
            
            if swing_trajectories is not None and i < len(swing_trajectories):
                if swing_trajectories[i] is not None:
                    msg.additionalFootPoseTrajectory.append(swing_trajectories[i])
                else:
                    msg.additionalFootPoseTrajectory.append(footPoses6D())
            else:
                msg.additionalFootPoseTrajectory.append(footPoses6D())

        self.trajectory_pub.publish(msg)
        rospy.loginfo("Trajectory published")

    def wait_for_trajectory_completion(self, time_traj):
        """等待轨迹执行完成并自动退出程序"""
        if self.debug_mode:
            rospy.loginfo("Debug mode: Skipping trajectory completion wait")
            return
            
        # 获取规划总时间并等待
        if time_traj and len(time_traj) > 0:
            total_time = time_traj[-1]
            rospy.loginfo(f"Planning total time: {total_time:.2f} seconds")
            rospy.loginfo(f"Waiting {total_time:.2f} seconds for trajectory execution to complete...")
            
            # 等待轨迹执行完成
            rospy.sleep(total_time)
            rospy.loginfo("Trajectory execution completed, exiting...")
            
            # 重新启用俯仰角限制
            self.set_pitch_limit(True)
            # 退出程序
            rospy.signal_shutdown("Trajectory execution completed")
        else:
            rospy.logwarn("No valid time trajectory found")

    def timeout_callback(self, event):
        """超时回调函数"""
        if not self.vision_received:
            rospy.logwarn("No vision data received within the timeout period")
            rospy.signal_shutdown("No vision data received within the timeout period")

planner = None  # 全局变量，供信号处理函数访问

def handle_sigint(signum, frame):
    global planner
    try:
        if planner is not None:
            planner.set_pitch_limit(True)
            rospy.loginfo("Pitch limit re-enabled (via signal handler)")
    except Exception as e:
        rospy.logwarn(f"Signal handler: set_pitch_limit failed: {e}")
    import sys
    sys.exit(0)

def main():
    global planner
    try:
        # 解析命令行参数
        parser = argparse.ArgumentParser(description='Vision based stair climbing planner')
        parser.add_argument('--debug', action='store_true', help='Enable debug mode (skip trajectory publication)')
        args = parser.parse_args()
        planner = VisionBasedStairClimbingPlanner(debug_mode=args.debug)
        # 注册SIGINT信号处理
        signal.signal(signal.SIGINT, handle_sigint)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
