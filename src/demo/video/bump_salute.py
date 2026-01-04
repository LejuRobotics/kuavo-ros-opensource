#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist 
from kuavo_msgs.srv import ExecuteArmAction, changeArmCtrlMode, gestureExecute
from kuavo_msgs.msg import robotHandPosition, RobotActionState, gestureTask,robotWaistControl  # 导入自定义消息类型
# from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState
# from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot



class RobotForwardWalk:
    def __init__(self):
        # 初始化节点
        rospy.init_node('forward_walk_node', anonymous=True)

        # self.robot = KuavoRobot()
        # self.robot = KuavoRobot()
        # self.robot_state = KuavoRobotState()

        # 创建发布者，发布到/cmd_vel话题
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.waist_pub = rospy.Publisher('/robot_waist_motion_data', robotWaistControl, queue_size=10)

        self.hand_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)


        # 抓取动作服务
        self.execute_service = rospy.ServiceProxy('/execute_arm_action', ExecuteArmAction)

        # 订阅抓取状态
        self.grab_status_sub = rospy.Subscriber('/robot_action_state', RobotActionState, self.grab_status_callback)

        self.gesture_service = rospy.ServiceProxy('/gesture/execute', gestureExecute)


        # 设置发布频率
        self.rate = rospy.Rate(10)  # 10Hz

        self.waist_msg = robotWaistControl()
        self.waist_msg.header.stamp = rospy.Time.now()
        self.waist_msg.data.data = [0]

        self.left_pos = [0] * 6
        self.right_pos = [0] * 6

    def control_robot_hand_position(self, left_pos, right_pos):
        """
        Control both hands' positions
        Args:
            left_pos: List of 6 values (0-100) for left hand finger positions
            right_pos: List of 6 values (0-100) for right hand finger positions
        """
        msg = robotHandPosition()
        msg.left_hand_position = left_pos
        msg.right_hand_position = right_pos
        self.hand_pub.publish(msg)

    def walk_forward(self, linear_speed=0.0, angular_speed=0.0, duration=0.0, waist_angle=0):
        """
        控制机器人向前行走

        Args:
            linear_speed: 线速度 (m/s)，限制在0-0.3范围内
            angular_speed: 角速度 (rad/s)，限制在-0.15到0.15范围内
            duration: 持续时间 (seconds)
        """
        # 限制线速度在0-0.3范围内
        linear_speed = max(-0.3, min(linear_speed, 0.3))
        
        # 限制角速度在-0.15到0.15范围内
        angular_speed = max(-0.4, min(angular_speed, 0.4))

        # 创建Twist消息
        cmd_vel_msg = Twist()

        # 设置线速度，x方向为正表示向前
        cmd_vel_msg.linear.x = linear_speed
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = 0.0

        # 设置角速度
        cmd_vel_msg.angular.x = 0.0
        cmd_vel_msg.angular.y = 0.0
        cmd_vel_msg.angular.z = angular_speed

        # 计算持续时间
        start_time = rospy.Time.now().to_sec()
        current_time = rospy.Time.now().to_sec()

        # rospy.loginfo(f"开始向前行走，速度: {linear_speed} m/s")

        # 持续发布速度指令
        while (current_time - start_time) < duration and not rospy.is_shutdown():
            # self.cmd_vel_pub.publish(cmd_vel_msg)
            self.turn_waist(waist_angle)
            self.rate.sleep()
            current_time = rospy.Time.now().to_sec()

        # # 停止机器人
        # self.stop_robot()
        # rospy.loginfo("行走结束")

    # def stop_robot(self):
    #     """停止机器人运动"""
    #     # 发送多次全零指令确保机器人完全停止
    #     cmd_vel_msg = Twist()
    #     for i in range(10):  # 发送10次全零指令
    #         # self.cmd_vel_pub.publish(cmd_vel_msg)
    #         self.rate.sleep()

    #     # self.robot.trot()
    #     # rospy.sleep(0.1)
    #     # self.robot.stance()
    #     # rospy.sleep(0.5)

    #     rospy.loginfo("机器人已完全停止")

    # 转腰
    def turn_waist(self, waist_angle=0):
        self.waist_msg.data.data = [waist_angle]
        self.waist_pub.publish(self.waist_msg)

    def grab_status_callback(self, msg):
        """处理抓取状态回调"""
        self.is_grad_finish = msg.state

    def call_change_arm_ctrl_mode_service(self, arm_ctrl_mode):
        result = True
        service_name = "humanoid_change_arm_ctrl_mode"
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            change_arm_ctrl_mode = rospy.ServiceProxy(
                "humanoid_change_arm_ctrl_mode", changeArmCtrlMode
            )
            change_arm_ctrl_mode(control_mode=arm_ctrl_mode)
            rospy.loginfo("Service call successful")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)
            result = False
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")
            result = False
        finally:
            return result

    def arm_action(self, arm_action_name=""):
        """
        控制机械臂执行抓取箱子的动作，并处理抓取结果。

        本函数首先将抓取请求标志设置为True，然后调用名为'roban2_arm_action'的服务。
        如果服务调用成功，将完成抓取动作，并更新相关状态。
        """
        # 发送一个抓取请求
        service_name = 'execute_arm_action'

        # 等待服务变为可用状态，超时设置为5秒
        rospy.wait_for_service(service_name, timeout=5)

        try:
            # 调用抓取箱子的服务
            response = self.execute_service(arm_action_name)
        except rospy.ServiceException as e:
            # 服务调用失败时，记录错误信息
            rospy.logerr(f"${arm_action_name} :Service call failed: {e}")
            return

        # 记录抓取结果
        rospy.loginfo(f"结果: {response.message}")

        # rospy.sleep(5)

        # 等待抓取完成
        # 耗时计算
        # start_time = rospy.Time.now().to_sec()
        # while self.is_grad_finish != 2 and not rospy.is_shutdown():
        #     rospy.loginfo(f"等待抓取完成...{self.is_grad_finish}")
        #     rospy.sleep(0.5)

        # coast_time = rospy.Time.now().to_sec() - start_time
        # rospy.loginfo(f"抓取完成，耗时: {coast_time} s")


if __name__ == '__main__':

    try:

        # 创建机器人行走控制器实例
        robot_walker = RobotForwardWalk()

        # 等待连接建立
        rospy.sleep(1.0)
        robot_walker.turn_waist(0)

        robot_walker.control_robot_hand_position([0,0,0,0,0,0], [0,0,0,0,0,0])
        rospy.sleep(1.0)

        robot_walker.control_robot_hand_position([0,0,100,100,100,100], [0,0,100,100,100,100])
        rospy.sleep(0.3)
        robot_walker.control_robot_hand_position([70,70,100,100,100,100], [70,70,100,100,100,100])
        robot_walker.call_change_arm_ctrl_mode_service(1)

        rospy.sleep(1.0)


        # # 击拳
        # input("1.开始击拳:请在终端按回车键继续...")
        # robot_walker.turn_waist(30)
        # robot_walker.arm_action('bump')
        # rospy.sleep(6)
        # robot_walker.turn_waist(0)
 
        # 抱拳
        input("2.抱拳：请在终端按回车键继续...")
        robot_walker.turn_waist(30)
        robot_walker.arm_action('抱拳')
        # rospy.sleep(2.5)
        for i in range(50):
            robot_walker.control_robot_hand_position([0,0,0,0,0,0], [int(70 / 50 * (i+1)),int(70 / 50 * (i+1)),int(100 / 50 * (i+1)),int(100 / 50 * (i+1)),int(100 / 50 * (i+1)),int(100 / 50 * (i+1))])
            rospy.sleep(0.05)
        # robot_walker.control_robot_hand_position([0,0,0,0,0,0], [70,70,100,100,100,100])
        rospy.sleep(3.5)
        robot_walker.turn_waist(0)

        # 回正握拳
        # robot_walker.control_robot_hand_position([0,0,100,100,100,100], [70,70,100,100,100,100])
        # rospy.sleep(0.3)
        # robot_walker.control_robot_hand_position([70,70,100,100,100,100], [70,70,100,100,100,100])
        for i in range(50):
            robot_walker.control_robot_hand_position([int(70 / 50 * (i+1)),int(70 / 50 * (i+1)),int(100 / 50 * (i+1)),int(100 / 50 * (i+1)),int(100 / 50 * (i+1)),int(100 / 50 * (i+1))], [70,70,100,100,100,100])
            rospy.sleep(0.05)



    except rospy.ROSInterruptException:
        pass
