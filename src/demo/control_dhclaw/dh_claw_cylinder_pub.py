#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DH 夹爪 + 手臂 IK 抓取圆柱体示例（真机/仿真，EndEffectorType=dhclaw）。

夹爪通过独立 /dh_claw_command topic 下发，速度/力使用 DH 百分比：
  position: 0~100
  velocity: 1~100
  effort:   20~100  （力 %）
"""
import time
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import twoArmHandPoseCmd, dhClawCommand
from kuavo_msgs.srv import twoArmHandPoseCmdSrv

# DH 速度/力百分比
DH_SPEED = 50
DH_FORCE = 50
POS_OPEN = 10
POS_CLOSE = 90


def deg(rad):
    return rad * 180.0 / np.pi


def main():
    rospy.init_node("dh_claw_cylinder_demo")
    pub_arm = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
    pub_claw = rospy.Publisher("/dh_claw_command", dhClawCommand, queue_size=10)

    rospy.wait_for_service("/humanoid_change_arm_ctrl_mode", timeout=3.0)
    from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest

    ctrl_proxy = rospy.ServiceProxy("/humanoid_change_arm_ctrl_mode", changeArmCtrlMode)
    ctrl_proxy(changeArmCtrlModeRequest(control_mode=2))
    rospy.loginfo("已切换手臂控制模式为 2")

    rospy.wait_for_service("/ik/two_arm_hand_pose_cmd_srv")
    ik_srv = rospy.ServiceProxy("/ik/two_arm_hand_pose_cmd_srv", twoArmHandPoseCmdSrv)

    t0 = time.time()
    while pub_arm.get_num_connections() == 0 and (time.time() - t0) < 5.0 and not rospy.is_shutdown():
        time.sleep(0.05)
    t1 = time.time()
    while pub_claw.get_num_connections() == 0 and (time.time() - t1) < 5.0 and not rospy.is_shutdown():
        time.sleep(0.05)

    from tf.transformations import quaternion_from_euler

    def move_arm(pos_xyz, rpy_xyz):
        request = twoArmHandPoseCmd()
        request.use_custom_ik_param = False
        request.joint_angles_as_q0 = False
        request.hand_poses.right_pose.pos_xyz = list(map(float, pos_xyz))
        request.hand_poses.right_pose.quat_xyzw = list(quaternion_from_euler(*rpy_xyz))
        request.hand_poses.right_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
        request.hand_poses.left_pose.pos_xyz = [0.0, 0.3, -0.5]
        request.hand_poses.left_pose.quat_xyzw = [0.0, 0.0, 0.0, 1.0]
        request.hand_poses.left_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]

        resp = ik_srv(request)
        if not resp.success:
            rospy.logwarn("IK 失败 pos=%s rpy=%s", pos_xyz, rpy_xyz)
            return False

        js = JointState()
        js.name = [
            "l_arm_pitch",
            "l_arm_roll",
            "l_arm_yaw",
            "l_forearm_pitch",
            "l_hand_yaw",
            "l_hand_pitch",
            "l_hand_roll",
            "r_arm_pitch",
            "r_arm_roll",
            "r_arm_yaw",
            "r_forearm_pitch",
            "r_hand_yaw",
            "r_hand_pitch",
            "r_hand_roll",
        ]
        js.position = [deg(x) for x in resp.q_arm]
        for _ in range(20):
            js.header.stamp = rospy.Time.now()
            pub_arm.publish(js)
            time.sleep(0.02)
        return True

    def send_claw(pos, vel=None, effort=None):
        if vel is None:
            vel = [DH_SPEED, DH_SPEED]
        if effort is None:
            effort = [DH_FORCE, DH_FORCE]
        msg = dhClawCommand()
        msg.data.name = ["left_claw", "right_claw"]
        msg.data.position = list(map(float, pos))
        msg.data.velocity = list(map(float, vel))
        msg.data.effort = list(map(float, effort))
        pub_claw.publish(msg)
        rospy.loginfo("claw pos=%s speed=%.0f%% force=%.0f%%", pos, vel[0], effort[0])

    def open_claw():
        send_claw([POS_OPEN, POS_OPEN])

    def close_claw():
        send_claw([POS_CLOSE, POS_CLOSE])

    rpy_pick = [-1.57, 0.0, 1.57]
    move_arm([0.15, -0.1, -0.2], rpy_pick)
    rospy.sleep(3.0)
    open_claw()
    rospy.sleep(1.0)
    move_arm([0.5, -0.1, -0.2], rpy_pick)
    rospy.sleep(1.0)
    close_claw()
    rospy.sleep(1.0)
    move_arm([0.5, -0.3, 0.1], rpy_pick)
    rospy.sleep(3.0)
    open_claw()
    rospy.sleep(1.0)


if __name__ == "__main__":
    main()
