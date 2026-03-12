#!/usr/bin/env python3
"""
场景二：分拣归档任务脚本模板

子任务一 - 料盘下架（40 分）：从料盘架夹取料盘放入物料箱
子任务二 - 分拣零件（30 分）：工作台上两种零件分类放入两侧物料箱

运行方式：
  python3 scene2_sorting.py              # 使用默认种子
  python3 scene2_sorting.py --seed 123   # 指定随机种子

可用接口：
  - /cmd_pose_world (geometry_msgs/Twist)        发送速度指令: linear.x=前进, linear.y=侧移, angular.z=转向
  - /kuavo_arm_traj (sensor_msgs/JointState)      手臂轨迹控制
  - /lidar/points (sensor_msgs/PointCloud2)       雷达点云数据
  - /sensors_data_raw (kuavo_msgs/sensorsData)    传感器原始数据（IMU、关节等）
  - /humanoid_controller/switch_controller        切换控制器
  - GripperController (craic_simulator)           夹爪控制
    - set_gripper_position(left, right)  左右夹爪命令 (0=张开, 255=闭合)
    - open_grippers() / close_grippers()
    - wait_for_position(target_left, target_right, tolerance, timeout)
"""

import argparse
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../craic_simulator/utils'))
from sim_launcher import SimLauncher


def main():
    parser = argparse.ArgumentParser(description="场景二：分拣归档")
    parser.add_argument('--seed', type=int, default=0, help='随机种子（控制机器人初始位姿，默认0）')
    args = parser.parse_args()

    # ---- 启动仿真（自动启动 roscore + 初始化 ROS 节点） ----
    launcher = SimLauncher(scene="scene2", seed=args.seed)
    launcher.start(node_name="scene2_sorting")

    # 以下代码在 ROS 节点初始化完成后执行
    import rospy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState, PointCloud2
    from gripper_controller import GripperController

    rospy.loginfo("=== 场景二：分拣归档任务启动 ===")

    # ---- 发布器 ----
    cmd_vel_pub = rospy.Publisher('/cmd_pose_world', Twist, queue_size=10)
    arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)

    # ---- 夹爪控制器 ----
    gripper = GripperController()

    rospy.sleep(1.0)  # 等待节点初始化

    # ========================================
    # TODO: 在此实现你的分拣归档逻辑
    # ========================================

    # 示例：张开夹爪
    # gripper.open_grippers()
    # rospy.sleep(1.0)

    # 示例：闭合夹爪抓取
    # gripper.close_grippers()
    # gripper.wait_for_position(target_left=0.8, target_right=0.8, timeout=3.0)

    # 示例：发送手臂轨迹
    # arm_msg = JointState()
    # arm_msg.header.stamp = rospy.Time.now()
    # arm_msg.position = [0.0] * 14  # 左臂7 + 右臂7
    # arm_traj_pub.publish(arm_msg)

    # 保持运行，Ctrl+C 退出
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
