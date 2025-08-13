#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../scripts')))

import rospy
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories
from robot_tool import KuavoRobotTools
from tf.transformations import euler_from_quaternion
import numpy as np


class FootPosePlanner:
    def __init__(self):
        self.init_com_z = None
        self.init_foot_spacing = 0.2
        self.robot_tools = KuavoRobotTools()

    def get_delta_com_z(self, step_length):
        leg_length = np.sqrt(self.init_com_z ** 2 + (self.init_foot_spacing / 2) ** 2)
        target_com_z = np.sqrt(leg_length ** 2 - (step_length / 2) ** 2)
        delta_com_z = target_com_z - self.init_com_z
        print(f"target_com_z:{target_com_z}")
        print(f"delta_com_z:{delta_com_z}")
        return delta_com_z 

    def set_init_com_z(self):
        if self.init_com_z is None:
            current_torso = self.robot_tools.get_base_to_odom()
            current_com_z = current_torso.position[2]
            self.init_com_z = current_com_z
            print(f"init_com_z: {self.init_com_z}")

    def get_foot_pose(self):
        current_left_foot = self.robot_tools.get_tf_transform(
            target_frame="odom",
            source_frame="leg_l6_link",
            return_type="pose_quaternion"
        )
        current_right_foot = self.robot_tools.get_tf_transform(
            target_frame="odom",
            source_frame="leg_r6_link",
            return_type="pose_quaternion"
        )
        _, _, yaw_l6 = euler_from_quaternion(current_left_foot.orientation)
        _, _, yaw_r6 = euler_from_quaternion(current_right_foot.orientation)

        current_left_foot = np.array([*current_left_foot.position, yaw_l6])
        current_right_foot = np.array([*current_right_foot.position, yaw_r6])

        self.set_init_com_z()

        return current_left_foot, current_right_foot

    def get_foot_pose_traj_msg(self, time_traj, foot_idx_traj, foot_traj, torso_traj):
        msg = footPoseTargetTrajectories()
        msg.timeTrajectory = time_traj
        msg.footIndexTrajectory = [int(idx) for idx in foot_idx_traj]
        msg.footPoseTrajectory = []

        for i in range(len(time_traj)):
            foot_pose_msg = footPose()
            foot_pose_msg.footPose = foot_traj[i]
            foot_pose_msg.torsoPose = torso_traj[i]
            msg.footPoseTrajectory.append(foot_pose_msg)

        return msg

    def get_single_foot_msg(self, side, movement):
        time_traj = [0.4, 0.8]
        foot_idx_traj = [side, 2]
        foot_traj = []
        torso_traj = []
        current_left_foot, current_right_foot = self.get_foot_pose()
        ss_action = [0, 0, 0, 0]

        if side == 0:
            current_support_foot = current_right_foot.copy()
        else:
            current_support_foot = current_left_foot.copy()

        movement_pose = np.asarray(movement)

        print(f"current_swing_foot:{movement_pose}")
        print(f"current_support_foot:{current_support_foot}")

        step_length = np.linalg.norm(movement_pose[:2] - current_support_foot[:2])

        torso_pose = (movement_pose + current_support_foot) / 2

        delta_com_z = self.get_delta_com_z(step_length)

        torso_pose[2] = delta_com_z
        ss_action = torso_pose.copy()

        foot_traj.append(movement_pose)
        torso_traj.append(torso_pose)
        foot_traj.append(movement_pose)
        torso_traj.append(ss_action)

        return self.get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj)

    def get_max_step_msg(self, max_step_length):
        self.set_init_com_z()
        delta_com_z = self.get_delta_com_z(max_step_length)
        foot_traj = [
            [max_step_length, 0.1, 0.0, 0.0], [0.0, 0.1, 0.0, 0.0],
            [0.0, 0.1 + max_step_length, 0.0, 0.0], [0.0, 0.1, 0.0, 0.0],
            [max_step_length, -0.1, 0.0, 0.0], [0.0, -0.1, 0.0, 0.0],
            [0.0, -0.1 - max_step_length, 0.0, 0.0], [0.0, -0.1, 0.0, 0.0]
        ]

        torso_traj = [
            [max_step_length / 2, 0.0, delta_com_z, 0.0], [0.0, 0.0, 0.0, 0.0],
            [0.0, max_step_length / 2, delta_com_z, 0.0], [0.0, 0.0, 0.0, 0.0],
            [max_step_length / 2, 0.0, delta_com_z, 0.0], [0.0, 0.0, 0.0, 0.0],
            [0.0, -max_step_length / 2, delta_com_z, 0.0], [0.0, 0.0, 0.0, 0.0]
        ]
        num = len(foot_traj)
        time_traj = [0.4 * i for i in range(1, 2 * num + 1)]
        foot_idx_traj = [0, 2] * int(num / 2) + [1, 2] * int(num / 2)
        msg = footPoseTargetTrajectories()
        msg.timeTrajectory = time_traj
        msg.footIndexTrajectory = foot_idx_traj

        for i in range(num):
            for _ in range(2):  # 增加了双足支撑相
                foot_pose_msg = footPose()
                foot_pose_msg.footPose = foot_traj[i]
                foot_pose_msg.torsoPose = torso_traj[i]
                msg.footPoseTrajectory.append(foot_pose_msg)
        print(f"msg:\n{msg}")
        return msg

    def test_max_step_length(self, pub, max_step_length):
        msg = self.get_max_step_msg(max_step_length)
        pub.publish(msg)

    def test_workspace(self, pub):
        # ============== ACTIONS =============
        #       ↖_, ↘_, ↓_, ↑_, ←_， →_, 
        #       _↖, _↘, _↓, _↑, _→， _←
        # ====================================
        movements = [
            [0.4, 0.15, 0.0, 0.5], [0.0, 0.15, 0.0, 0.0],
            [-0.2, 0.15, 0.0, 0.0], [0.0, 0.15, 0.0, 0.0],
            [0.0, 0.3, 0.0, 0.0], [0.0, 0.15, 0.0, 0.0],
            [0.4, -0.15, 0.0, -0.5], [0.0, -0.15, 0.0, 0.0],
            [-0.2, -0.15, 0.0, 0.0], [0.0, -0.15, 0.0, 0.0],
            [0.0, -0.3, 0.0, 0.0], [0.0, -0.15, 0.0, 0.0]
        ]

        for i, movement in enumerate(movements):
            side = 0 if i < len(movements) // 2 else 1
            msg = self.get_single_foot_msg(side, movement)
            pub.publish(msg)
            rospy.loginfo(f"Published workspace msg {i+1}/{len(movements)}:\n{msg}")
            rospy.sleep(2)


if __name__ == '__main__':
    rospy.init_node('foot_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/humanoid_mpc_foot_pose_world_target_trajectories', footPoseTargetTrajectories, queue_size=10)
    rospy.sleep(1)

    # 初始化类，设置 com_z 和足间距
    planner = FootPosePlanner()

    try:
        # 示例调用：最大步长测试
        # planner.test_max_step_length(pub, 0.4)

        # 示例调用：动作空间测试
        planner.test_workspace(pub)
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted. Exiting gracefully...")
    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupt received. Shutting down...")
    finally:
        rospy.loginfo("Foot pose publisher node shutdown complete.")


