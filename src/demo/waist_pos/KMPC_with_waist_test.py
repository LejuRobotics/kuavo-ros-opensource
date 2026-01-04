#!/usr/bin/env python3
# 带腰部控制的KMPC测试脚本：
#仿真启动：
#1.在kuavo_ros_application仓库中，在终端运行：roslaunch kuavo_tf2_web_republisher start_websocket_server.launch
#2.终端启动:roslaunch humanoid_controllers load_kuavo_gazebo_manipulate.launch
#3.另起终端：roslaunch ar_control robot_strategies.launch
#4.另起终端：python3 ./src/demo/waist_pos/KMPC_with_waist_test.py
import numpy as np
import time
import rospy
from typing import Tuple
from kuavo_msgs.msg import robotWaistControl
from ocs2_msgs.msg import mpc_target_trajectories

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Frame
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoManipulationMpcFrame, KuavoManipulationMpcCtrlMode, 
    KuavoManipulationMpcControlFlow, KuavoPose
)

class ArmController:
    def __init__(self):
        self.robot_sdk = RobotSDK()
        self.control_initialized = False
        self.initial_poses = None
        self.last_waist_yaw = 0.0  # 记录上一次的腰部角度（度）
        try:
            rospy.init_node('waist_yaw_test', anonymous=True)
        except rospy.exceptions.ROSException:
            pass
        self.waist_pub = rospy.Publisher('/robot_waist_motion_data', robotWaistControl, queue_size=10)
    
    def record_initial_pose(self):
        print("记录初始位姿...")
        self.initial_poses = self.get_current_poses()
        left, right = self.initial_poses
        left_euler = left.get_euler(degrees=True)
        right_euler = right.get_euler(degrees=True)
        print(f"✅ 已记录初始位姿:")
        print(f"左手: 位置({left.pos[0]:.3f}, {left.pos[1]:.3f}, {left.pos[2]:.3f}) "
              f"姿态(roll={left_euler[0]:.1f}°, pitch={left_euler[1]:.1f}°, yaw={left_euler[2]:.1f}°)")
        print(f"右手: 位置({right.pos[0]:.3f}, {right.pos[1]:.3f}, {right.pos[2]:.3f}) "
              f"姿态(roll={right_euler[0]:.1f}°, pitch={right_euler[1]:.1f}°, yaw={right_euler[2]:.1f}°)")
    
    def reset_waist_to_zero(self):
        print("腰部回零...")
        try:
            msg = robotWaistControl()
            msg.header.stamp = rospy.Time.now()
            msg.data.data = [0.0]
            self.waist_pub.publish(msg)
            time.sleep(2)
            self.set_waist_yaw(0.0)  # 更新腰部角度状态为0
            print("✅ 腰部已回零")
        except Exception as e:
            print(f"❌ 腰部回零失败: {e}")
    
    def return_to_initial(self):
        if self.initial_poses is None:
            print("❌ 未记录初始位姿，请先使用选项0记录")
            return False
        
        print("回到初始位姿...")
        try:
            self.reset_waist_to_zero()
            current_left, current_right = self.get_current_poses()
            initial_left, initial_right = self.initial_poses
            
            left_traj = self.interpolate_poses_simple(current_left, initial_left)
            right_traj = self.interpolate_poses_simple(current_right, initial_right)
            return self.execute_motion(left_traj, right_traj)
        except Exception as e:
            print(f"❌ 回到初始位姿失败: {e}")
            return False
    
    def setup_control(self):
        if not self.control_initialized:
            print("设置手臂控制模式...")
            self.robot_sdk.control.set_external_control_arm_mode()
            time.sleep(0.5)
            self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
            time.sleep(0.5)
            self.control_initialized = True
            print("✅ 手臂控制模式已设置")
    
    def cleanup_control(self):
        if self.control_initialized:
            self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.NoControl)
            self.control_initialized = False
    
    def get_current_poses(self) -> Tuple[Pose, Pose]:
        try:
            left_pose = self.robot_sdk.tools.get_link_pose("zarm_l7_end_effector", Frame.ODOM)
            right_pose = self.robot_sdk.tools.get_link_pose("zarm_r7_end_effector", Frame.ODOM)
            if left_pose and right_pose:
                return (Pose(pos=left_pose.position, quat=left_pose.orientation, frame=Frame.ODOM),
                        Pose(pos=right_pose.position, quat=right_pose.orientation, frame=Frame.ODOM))
        except:
            pass
        return (Pose.from_euler(pos=(0.4, 0.3, 1.0), euler=(0, -90, 0), frame=Frame.ODOM, degrees=True),
                Pose.from_euler(pos=(0.4, -0.3, 1.0), euler=(0, -90, 0), frame=Frame.ODOM, degrees=True))
    
    def get_current_waist_yaw(self) -> float:
        """获取当前腰部yaw角度（度）"""
        return self.last_waist_yaw
    
    def set_waist_yaw(self, yaw_deg: float):
        """设置并记录腰部yaw角度"""
        self.last_waist_yaw = yaw_deg
    
    def transform_by_yaw(self, pose: Pose, yaw_deg: float) -> Pose:
        yaw_rad = np.deg2rad(yaw_deg)
        x, y, z = pose.pos
        x_new = x * np.cos(yaw_rad) - y * np.sin(yaw_rad)
        y_new = x * np.sin(yaw_rad) + y * np.cos(yaw_rad)
        euler = pose.get_euler(degrees=True)
        return Pose.from_euler(pos=(x_new, y_new, z), 
                               euler=(euler[0], euler[1], euler[2] + yaw_deg), 
                               frame=Frame.ODOM, degrees=True)
    
    def interpolate_poses_by_waist_yaw(self, initial_pose: Pose, target_yaw_deg: float, current_yaw_deg: float = 0.0, num_points=200):
        """根据腰部yaw角度插值生成位姿轨迹"""
        poses = []
        for i in range(num_points):
            t = 3 * (i/(num_points-1))**2 - 2 * (i/(num_points-1))**3  # S曲线插值
            # 从当前腰部角度插值到目标角度
            interpolated_yaw = current_yaw_deg + (target_yaw_deg - current_yaw_deg) * t
            transformed_pose = self.transform_by_yaw(initial_pose, interpolated_yaw)
            poses.append(transformed_pose)
        return poses
    
    def generate_waist_yaw_trajectory(self, left_initial: Pose, right_initial: Pose, target_yaw_deg: float, current_yaw_deg: float = 0.0, num_points=200):
        """生成基于腰部yaw角度的双臂轨迹"""
        left_traj = self.interpolate_poses_by_waist_yaw(left_initial, target_yaw_deg, current_yaw_deg, num_points)
        right_traj = self.interpolate_poses_by_waist_yaw(right_initial, target_yaw_deg, current_yaw_deg, num_points)
        return left_traj, right_traj
    
    def interpolate_poses_simple(self, start: Pose, end: Pose, num_points=200):
        """简单位姿插值（用于回到初始位姿）"""
        from scipy.spatial.transform import Rotation as R
        from scipy.spatial.transform import Slerp
        poses = []
        rots = R.from_quat([start.quat, end.quat])
        times = [0, 1]
        slerp = Slerp(times, rots)
        for i in range(num_points):
            t = 3 * (i/(num_points-1))**2 - 2 * (i/(num_points-1))**3
            pos = start.pos + (end.pos - start.pos) * t
            quat = slerp(t).as_quat()
            poses.append(Pose(pos=pos.tolist(), quat=quat.tolist(), frame=Frame.ODOM))
        return poses
    
    def execute_motion(self, left_traj, right_traj):
        print(f"执行运动，共{len(left_traj)}个点...")
        time_per_point = 3.0 / len(left_traj)
        try:
            for i, (left_pose, right_pose) in enumerate(zip(left_traj, right_traj)):
                if i % 5 == 0:
                    print(f"进度: {i+1}/{len(left_traj)}")
                
                self.robot_sdk.control.control_robot_end_effector_pose(
                    left_pose=KuavoPose(position=left_pose.pos.tolist(), orientation=left_pose.quat.tolist()),
                    right_pose=KuavoPose(position=right_pose.pos.tolist(), orientation=right_pose.quat.tolist()),
                    frame=KuavoManipulationMpcFrame.WorldFrame
                )
                if i < len(left_traj) - 1:
                    time.sleep(time_per_point)
            print("✅ 运动完成")
            return True
        except Exception as e:
            print(f"❌ 运动失败: {e}")
            return False
    

def show_menu():
    print("带腰的KMPC测试")
    print("="*40)
    print("c - 记录当前位姿为初始位姿")
    print("r - 回到初始位姿")
    print("1 - 自定义变换")
    print("2 - 快速测试(90°)")
    print("3 - 显示当前位姿")
    print("q - 退出")
    print("="*40)

def input_pose_and_yaw():
    print("\n输入目标位姿（回车使用默认值）:")
    left_x = float(input("左手X (0.4): ") or "0.4")
    left_y = float(input("左手Y (0.3): ") or "0.3")
    left_z = float(input("左手Z (1.0): ") or "1.0")
    right_x = float(input("右手X (0.4): ") or "0.4")
    right_y = float(input("右手Y (-0.3): ") or "-0.3")
    right_z = float(input("右手Z (1.0): ") or "1.0")
    yaw = float(input("转腰yaw角(90): ") or "90")
    
    left_pose = Pose.from_euler(pos=(left_x, left_y, left_z), euler=(0, -90, 0), frame=Frame.ODOM, degrees=True)
    right_pose = Pose.from_euler(pos=(right_x, right_y, right_z), euler=(0, -90, 0), frame=Frame.ODOM, degrees=True)
    return left_pose, right_pose, yaw

def main():
    print("欢迎使用腰部yaw角变换测试程序")
    controller = ArmController()
    
    try:
        while True:
            show_menu()
            choice = input("\n选择功能: ").strip()
            
            if choice == 'q':
                break
            elif choice == 'c':
                controller.record_initial_pose()
            elif choice == 'r':
                controller.setup_control()
                controller.return_to_initial()
            elif choice == '1':
                target_left, target_right, yaw_deg = input_pose_and_yaw()
                controller.setup_control()
                
                new_left = controller.transform_by_yaw(target_left, yaw_deg)
                new_right = controller.transform_by_yaw(target_right, yaw_deg)
                print(f"\n变换后位姿:")
                left_euler = new_left.get_euler(degrees=True)
                right_euler = new_right.get_euler(degrees=True)
                print(f"左手位置: ({new_left.pos[0]:.2f}, {new_left.pos[1]:.2f}, {new_left.pos[2]:.2f})")
                print(f"左手姿态: (roll={left_euler[0]:.1f}°, pitch={left_euler[1]:.1f}°, yaw={left_euler[2]:.1f}°)")
                print(f"右手位置: ({new_right.pos[0]:.2f}, {new_right.pos[1]:.2f}, {new_right.pos[2]:.2f})")
                print(f"右手姿态: (roll={right_euler[0]:.1f}°, pitch={right_euler[1]:.1f}°, yaw={right_euler[2]:.1f}°)")
                
                if input("执行变换? (y/n): ").lower() == 'y':
                    current_waist_yaw = controller.get_current_waist_yaw()
                    left_traj, right_traj = controller.generate_waist_yaw_trajectory(target_left, target_right, yaw_deg, current_waist_yaw)
                    controller.execute_motion(left_traj, right_traj)
                    controller.set_waist_yaw(yaw_deg)  # 更新腰部角度状态
            
            elif choice == '2':
                controller.setup_control()
                default_left = Pose.from_euler(pos=(0.4, 0.3, 1.0), euler=(0, -90, 0), frame=Frame.ODOM, degrees=True)
                default_right = Pose.from_euler(pos=(0.4, -0.3, 1.0), euler=(0, -90, 0), frame=Frame.ODOM, degrees=True)
                new_left = controller.transform_by_yaw(default_left, 90)
                new_right = controller.transform_by_yaw(default_right, 90)
                
                current_waist_yaw = controller.get_current_waist_yaw()
                left_traj, right_traj = controller.generate_waist_yaw_trajectory(default_left, default_right, 90, current_waist_yaw)
                controller.execute_motion(left_traj, right_traj)
                controller.set_waist_yaw(90)  # 更新腰部角度状态
            
            elif choice == '3':
                current_left, current_right = controller.get_current_poses()
                left_euler = current_left.get_euler(degrees=True)
                right_euler = current_right.get_euler(degrees=True)
                print(f"\n当前位姿:")
                print(f"左手位置: ({current_left.pos[0]:.3f}, {current_left.pos[1]:.3f}, {current_left.pos[2]:.3f})")
                print(f"左手姿态: (roll={left_euler[0]:.1f}°, pitch={left_euler[1]:.1f}°, yaw={left_euler[2]:.1f}°)")
                print(f"右手位置: ({current_right.pos[0]:.3f}, {current_right.pos[1]:.3f}, {current_right.pos[2]:.3f})")
                print(f"右手姿态: (roll={right_euler[0]:.1f}°, pitch={right_euler[1]:.1f}°, yaw={right_euler[2]:.1f}°)")
            
            else:
                print("❌ 无效选择")
    
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        controller.cleanup_control()
        print("程序结束")

if __name__ == "__main__":
    main()