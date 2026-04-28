#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试FK服务，使用指定的参考坐标系
"""

import rospy
import sys
from kuavo_msgs.srv import fkSrvWithReferFrame, fkSrvWithReferFrameRequest

def main():
    rospy.init_node('test_fk_with_refer_frame', anonymous=True)
    
    # 参数设置
    # 如果通过命令行传入，则使用命令行关节角：
    #   rosrun ... test_fk_with_refer_frame.py j1 j2 ... j14
    # 否则使用下面代码中逐个手动设置的默认关节角
    if len(sys.argv) > 1:
        joint_angles = [float(x) for x in sys.argv[1:]]
    else:
        # 在这里分别设置 14 个关节角（单位：弧度）
        j1  = 0.0
        j2  = 0.0
        j3  = 0.0
        j4  = -1.57
        j5  = 0.0
        j6  = 0.0
        j7  = 0.0
        j8  = 0.0
        j9  = 0.0
        j10 = 0.0
        j11 = 0.0
        j12 = 0.0
        j13 = 0.0
        j14 = 0.0

        joint_angles = [
            j1, j2, j3, j4, j5, j6, j7,
            j8, j9, j10, j11, j12, j13, j14,
        ]
    base_frame = "zarm_l1_ref_link"
    end_effector_frame = "zarm_l7_end_effector"
    hand_side = 0  # 0=左臂, 1=右臂
    
    # 调用服务
    rospy.wait_for_service("/ik/fk_srv_with_refer_frame", timeout=5.0)
    fk_client = rospy.ServiceProxy("/ik/fk_srv_with_refer_frame", fkSrvWithReferFrame)
    
    req = fkSrvWithReferFrameRequest()
    req.q = joint_angles
    req.base_frame = base_frame
    req.end_effector_frame = end_effector_frame
    req.hand_side = hand_side
    
    res = fk_client(req)
    
    # 打印结果
    if res.success:
        if hand_side == 0:
            pose = res.hand_poses.left_pose
        else:
            pose = res.hand_poses.right_pose
        
        print(f"Success: {res.success}")
        print(f"Position: {pose.pos_xyz}")
        print(f"Quaternion: {pose.quat_xyzw}")
        print(f"Joint angles: {pose.joint_angles}")
    else:
        print(f"Failed: {res.error_message}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
