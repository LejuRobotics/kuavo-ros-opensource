#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Roban2机器人手臂测试脚本

此脚本用于测试Roban2机器人的手臂运动控制功能，包括：
- 手臂控制模式切换
- 多段轨迹规划与执行
- 关节位置控制测试

适用机器人型号：Roban2
测试接口：/kuavo_arm_traj, /arm_traj_change_mode
"""

import rospy
import time
import sys
import os
from sensor_msgs.msg import JointState
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest


class ArmTrajectoryController:
    
    def __init__(self, node_name="arm_trajectory_controller"):
        rospy.init_node(node_name)
        self.arm_pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
        self.change_mode_service = None
        self._wait_for_connections()
        rospy.loginfo("手臂轨迹控制器初始化完成")
    
    def _wait_for_connections(self, timeout=10.0):
        start_time = time.time()
        while self.arm_pub.get_num_connections() == 0:
            if time.time() - start_time > timeout:
                rospy.logwarn("等待订阅者连接超时，继续执行...")
                break
            rospy.loginfo("等待订阅者连接到 /kuavo_arm_traj 话题...")
            rospy.sleep(0.1)
    
    def _init_change_mode_service(self):
        if self.change_mode_service is None:
            rospy.wait_for_service('/arm_traj_change_mode')
            self.change_mode_service = rospy.ServiceProxy('/arm_traj_change_mode', changeArmCtrlMode)
    
    def change_arm_ctrl_mode(self, control_mode):
        try:
            self._init_change_mode_service()
            req = changeArmCtrlModeRequest()
            req.control_mode = control_mode
            res = self.change_mode_service(req)
            if res.result:
                rospy.loginfo("手臂控制模式已更改为 %d", control_mode)
            else:
                rospy.logerr("无法将手臂控制模式更改为 %d", control_mode)
        except rospy.ServiceException as e:
            rospy.logerr("服务调用失败: %s", e)
    
    def publish_joint_states(self, joint_positions):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["arm_joint_" + str(i) for i in range(0, 8)]
        msg.position = joint_positions
        self.arm_pub.publish(msg)
    
    
    def execute_trajectory(self, trajectory, interval=0.02):
        for joint_positions in trajectory:
            # 不需要转成弧度，直接发送原始角度
            self.publish_joint_states(joint_positions)
            time.sleep(interval)
    
    def execute_arm_trajectory(self, q0, q1, duration, interval):
        num_steps = int(duration / interval)
        trajectory_to_target = []
        for i in range(num_steps):
            q_tmp = [0.0] * 8
            for j in range(8):
                q_tmp[j] = q0[j] + i / float(num_steps) * (q1[j] - q0[j])
            trajectory_to_target.append(q_tmp)
        
        self.execute_trajectory(trajectory_to_target, interval)
    def shutdown(self):
        rospy.loginfo("手臂轨迹控制器关闭")


def main():
    # 手臂测试脚本
    
    try:
        rospy.loginfo("手臂测试脚本启动")
        target_time_ns = None
        
        arm_controller = ArmTrajectoryController()
        interval = 1/60
        execute_time=1.2
        q0 = [0, 10, 0, 0, 0.0, -10, 0, 0 ]
        q1 = [0, 50, 90, 0, 0.0, -50, -90, 0 ]
        q2 = [-40, 0, 0, -30, -40, 0, 0, -100]
        q3 = [-40, 0, 0, -100, -40.0, 0, 0, -30]
        
        # 更改手臂控制模式
        arm_controller.change_arm_ctrl_mode(2)

        # 执行手臂轨迹
        for loop_num in range(3):
            arm_controller.execute_arm_trajectory(q0, q1, execute_time, interval)
            arm_controller.execute_arm_trajectory(q1, q0, execute_time, interval)
            time.sleep(0.5)
        
        # 第三段：q0 -> q2
        arm_controller.execute_arm_trajectory(q0, q2, execute_time, interval)

        # 循环段：q2 <-> q3
        for i in range(5):
            arm_controller.execute_arm_trajectory(q2, q3, 1.0, interval)
            arm_controller.execute_arm_trajectory(q3, q2, 1.0, interval)

        time.sleep(0.5)
        # 最后一段：q2 -> q0
        arm_controller.execute_arm_trajectory(q2, q0, execute_time, interval)

        # 恢复控制模式
        arm_controller.change_arm_ctrl_mode(1)
        rospy.loginfo("手臂测试完成")
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断")
    except KeyboardInterrupt:
        rospy.loginfo("用户中断")
    except Exception as e:
        rospy.logerr("发生错误: %s", e)
    finally:
        if 'arm_controller' in locals():
            arm_controller.shutdown()


if __name__ == '__main__':
    main()