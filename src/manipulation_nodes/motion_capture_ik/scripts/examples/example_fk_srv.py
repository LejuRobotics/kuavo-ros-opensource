#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import rospy
import rospkg
from kuavo_msgs.srv import fkSrv

# 自动导入 kuavo_common/python 下的模块
rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path('kuavo_common') + '/python')
from robot_version import RobotVersion

def fk_srv_client(joint_angles):
    rospy.wait_for_service('/ik/fk_srv')
    try:
        fk_srv = rospy.ServiceProxy('/ik/fk_srv', fkSrv)
        fk_result = fk_srv(joint_angles)
        print("FK result success:", fk_result.success)
        return fk_result.hand_poses if fk_result.success else None
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return None


if __name__ == "__main__":
    rospy.init_node("example_fk_srv_node", anonymous=True)

    # 从环境变量读取 ROBOT_VERSION，默认 "45"
    robot_version_str = os.getenv("ROBOT_VERSION", "45")

    # 解析版本，获取“第几代”（major）
    try:
        big_number = int(robot_version_str)
        version = RobotVersion.create(big_number)
        generation = version.major()
    except ValueError:
        rospy.logerr("ROBOT_VERSION must be a valid integer string (e.g., '45' or '10'), but got: '%s'" % robot_version_str)
        sys.exit(1)
    except Exception as e:
        rospy.logerr("RobotVersion parsing failed for '%s': %s" % (robot_version_str, str(e)))
        sys.exit(1)


    # 根据代数选择固定关节角（不修改数值！）
    if generation == 1:
        joint_angles = [0.0, -1.38, -1.39, -0.29, -0.43, 0.0, -0.17, 0.0]
        rospy.loginfo("Using 8-joint angles for Gen-1")
    else:
        joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.38, -1.39, -0.29, -0.43, 0.0, -0.17, 0.0]
        rospy.loginfo("Using 14-joint angles for Gen-%d" % generation)
    
    hand_poses = fk_srv_client(joint_angles)
    if hand_poses is not None:
        print("left eef position:", hand_poses.left_pose.pos_xyz)
        print("\nright eef position: ", hand_poses.right_pose.pos_xyz)
    else:
        print("No hand poses returned")