#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
参考脚本：输入右手末端期望位姿（位置+四元数），调用IK服务求解关节角度，并通过/kuavo_arm_traj话题发布。
"""
import rospy
import numpy as np
import time
import subprocess
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import twoArmHandPoseCmd, lejuClawCommand
from kuavo_msgs.srv import twoArmHandPoseCmdSrv

def deg(rad):
    return rad * 180.0 / np.pi

def main():
    rospy.init_node('ik_to_arm_traj_demo')
    pub_arm = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)

    # 简化流程，直接调用服务切换手臂控制模式为2
    rospy.wait_for_service('/humanoid_change_arm_ctrl_mode', timeout=3.0)
    from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
    ctrl_proxy = rospy.ServiceProxy('/humanoid_change_arm_ctrl_mode', changeArmCtrlMode)
    req = changeArmCtrlModeRequest(control_mode=2)
    ctrl_proxy(req)
    rospy.loginfo('已切换手臂控制模式为2 (ROS服务)')

    rospy.wait_for_service('/ik/two_arm_hand_pose_cmd_srv')
    ik_srv = rospy.ServiceProxy('/ik/two_arm_hand_pose_cmd_srv', twoArmHandPoseCmdSrv)

    # 夹爪控制发布器（参考 leju_claw_pub.py）
    pub_claw = rospy.Publisher('/leju_claw_command', lejuClawCommand, queue_size=10)

    # 等待各发布器有订阅者（最多等待5秒）
    t0 = time.time()
    while pub_arm.get_num_connections() == 0 and (time.time() - t0) < 5.0 and not rospy.is_shutdown():
        time.sleep(0.05)
    t1 = time.time()
    while pub_claw.get_num_connections() == 0 and (time.time() - t1) < 5.0 and not rospy.is_shutdown():
        time.sleep(0.05)

    from tf.transformations import quaternion_from_euler

    def move_arm(pos_xyz, rpy_xyz):
        """调用IK并将结果发布到 /kuavo_arm_traj，pos/rpy 单位: m / rad"""
        request = twoArmHandPoseCmd()
        request.use_custom_ik_param = False
        request.joint_angles_as_q0 = False
        request.hand_poses.right_pose.pos_xyz = list(map(float, pos_xyz))
        quat_xyzw = list(quaternion_from_euler(*rpy_xyz))
        request.hand_poses.right_pose.quat_xyzw = quat_xyzw
        request.hand_poses.right_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
        # 左手保持默认
        request.hand_poses.left_pose.pos_xyz = [0.0, 0.3, -0.5]
        request.hand_poses.left_pose.quat_xyzw = [0.0, 0.0, 0.0, 1.0]
        request.hand_poses.left_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]

        resp = ik_srv(request)
        if not resp.success:
            rospy.logwarn(f"IK求解失败，pos={pos_xyz}, rpy={rpy_xyz}")
            return False

        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = [
            "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch", "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
            "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", "r_hand_roll",
        ]
        js.position = [deg(x) for x in resp.q_arm]
        # 短时间内重复发布，确保控制器接收
        for _ in range(20):  # 约0.4秒 @50Hz
            js.header.stamp = rospy.Time.now()
            pub_arm.publish(js)
            time.sleep(0.02)
        return True

    def send_claw(pos, vel, effort):
        msg = lejuClawCommand()
        msg.data.name = ['left_claw', 'right_claw']
        msg.data.position = pos
        msg.data.velocity = vel
        msg.data.effort = effort
        pub_claw.publish(msg)

    def open_claw():
        # 参考示例：打开 -> 位置 [10,10]
        send_claw([50.0, 50.0], [90, 90], [1.0, 1.0])

    def close_claw():
        # 参考示例：闭合 -> 位置 [90,90]
        send_claw([100, 100], [90, 90], [1.0, 1.0])

    # 抓取圆柱体任务流程序列
    rpy_pick = [-1.57, 0.0, 1.57]
    # 1) 运动到 [0.2, -0.1, -0.2] @ rpy_pick
    move_arm([0.15, -0.1, -0.2], rpy_pick)
    rospy.sleep(3)
    # 2) 打开夹爪
    open_claw()
    rospy.sleep(1.0)
    # 3) 前移至 [0.4, -0.1, -0.2]
    move_arm([0.5, -0.1, -0.2], rpy_pick)
    rospy.sleep(1)
    # 4) 闭合夹爪
    close_claw()
    rospy.sleep(1.0)
    # 5) 抬升至 [0.4, -0.1, 0.0]
    move_arm([0.5, -0.3, 0.1], rpy_pick)
    rospy.sleep(3)
    # 6) 打开夹爪
    open_claw()
    rospy.sleep(1.0)

if __name__ == '__main__':
    main()