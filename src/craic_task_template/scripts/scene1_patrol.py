#!/usr/bin/env python3
"""
场景一：安全巡检任务脚本模板

路线：起点 → 障碍区 → 复杂地形区 → 操作台1 → 斜坡区 → 台阶区 → 操作台2 → 终点
附加任务（可选）：终点 → 台阶区 → 斜坡区 → 复杂地形区 → 障碍区 → 起点

运行方式：
  python3 scene1_patrol.py              # 使用默认种子
  python3 scene1_patrol.py --seed 123   # 指定随机种子

可用接口：
  - /cmd_pose_world (geometry_msgs/Twist)        发送速度指令: linear.x=前进, linear.y=侧移, angular.z=转向
  - /kuavo_arm_traj (sensor_msgs/JointState)      手臂轨迹控制
  - /lidar/points (sensor_msgs/PointCloud2)       雷达点云数据
  - /sensors_data_raw (kuavo_msgs/sensorsData)    传感器原始数据（IMU、关节等）
  - /humanoid_controller/switch_controller        切换控制器（如 amp_controller）
  - GripperController (craic_simulator)           夹爪控制（用于按钮操作）
"""

import argparse
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../craic_simulator/utils'))
from sim_launcher import SimLauncher


def main():
    parser = argparse.ArgumentParser(description="场景一：安全巡检")
    parser.add_argument('--seed', type=int, default=0, help='随机种子（控制机器人初始位姿，默认0）')
    args = parser.parse_args()

    # ---- 启动仿真（自动启动 roscore + 初始化 ROS 节点） ----
    launcher = SimLauncher(scene="scene1", seed=args.seed)
    launcher.start(node_name="scene1_patrol")

    # 以下代码在 ROS 节点初始化完成后执行
    import rospy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState, PointCloud2
    from gripper_controller import GripperController

    rospy.loginfo("=== 场景一：安全巡检任务启动 ===")

    # ---- 发布器 ----
    cmd_vel_pub = rospy.Publisher('/cmd_pose_world', Twist, queue_size=10)
    arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)

    # ---- 夹爪控制器（按钮操作用） ----
    gripper = GripperController()

    # ---- 订阅雷达点云 ----
    # def lidar_callback(msg):
    #     pass  # 处理点云数据，用于避障和导航
    # rospy.Subscriber('/lidar/points', PointCloud2, lidar_callback)

    rospy.sleep(1.0)  # 等待节点初始化

    # ========================================
    # TODO: 在此实现你的安全巡检逻辑
    # ========================================

    # 示例：发送前进速度指令
    # cmd = Twist()
    # cmd.linear.x = 0.3   # 前进速度 m/s
    # cmd.angular.z = 0.0   # 转向角速度 rad/s
    # cmd_vel_pub.publish(cmd)

    # 示例：切换控制器
    # from kuavo_msgs.srv import switchController
    # rospy.wait_for_service('/humanoid_controller/switch_controller')
    # switch = rospy.ServiceProxy('/humanoid_controller/switch_controller', switchController)
    # switch('amp_controller')

    # 保持运行，Ctrl+C 退出
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
