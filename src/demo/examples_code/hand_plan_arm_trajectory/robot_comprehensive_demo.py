#!/usr/bin/env python3

import rospy
import json
import math
import numpy as np
from humanoid_plan_arm_trajectory.srv import planArmTrajectoryBezierCurve, planArmTrajectoryBezierCurveRequest
from humanoid_plan_arm_trajectory.msg import bezierCurveCubicPoint, jointBezierTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
from kuavo_msgs.msg import sensorsData
from kuavo_msgs.msg import robotHandPosition, robotHeadMotionData

class ComprehensiveRobotControl:
    def __init__(self):
        # 初始化节点
        rospy.init_node('comprehensive_robot_control')
        
        # 初始化状态变量
        self.joint_state = JointState()
        self.hand_state = robotHandPosition()
        self.head_state = robotHeadMotionData()
        self.current_arm_joint_state = []
        
        # 发布者
        self.arm_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=1, tcp_nodelay=True)
        self.hand_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=1, tcp_nodelay=True)
        self.head_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=1, tcp_nodelay=True)
        
        # 订阅者
        rospy.Subscriber('/bezier/arm_traj', JointTrajectory, self.traj_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/sensors_data_raw', sensorsData, self.sensors_data_callback, queue_size=1, tcp_nodelay=True)

    def load_tact_file(self, file_path):
        """加载并解析.tact文件"""
        with open(file_path, 'r') as f:
            data = json.load(f)
        return data

    def process_frame(self, frame):
        """处理单个关键帧数据"""
        servos = frame['servos']
        
        # 处理手指数据 (14-26)
        left_hand = [max(0, int(pos)) for pos in servos[14:20]]
        right_hand = [max(0, int(pos)) for pos in servos[20:26]]
        self.hand_state.left_hand_position = left_hand
        self.hand_state.right_hand_position = right_hand
        
        # 处理头部数据 (26-28)
        self.head_state.joint_data = servos[26:28]

    def change_control_mode(self, mode):
        """切换控制模式"""
        try:
            rospy.wait_for_service('humanoid_change_arm_ctrl_mode')
            change_mode = rospy.ServiceProxy('humanoid_change_arm_ctrl_mode', changeArmCtrlMode)
            req = changeArmCtrlModeRequest()
            req.control_mode = mode
            change_mode(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def sensors_data_callback(self, msg):
        """处理传感器数据回调"""
        self.current_arm_joint_state = msg.joint_data.joint_q[12:26]
        self.current_arm_joint_state = [round(pos, 2) for pos in self.current_arm_joint_state]
        self.current_arm_joint_state.extend([0] * 14)

    def traj_callback(self, msg):
        """处理轨迹回调"""
        if len(msg.points) == 0:
            return
        point = msg.points[0]
        self.joint_state.name = [
            "l_arm_pitch", "l_arm_roll", "l_arm_yaw",
            "l_forearm_pitch", "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
            "r_arm_pitch", "r_arm_roll", "r_arm_yaw", 
            "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", "r_hand_roll"
        ]
        # 将关节位置和速度从弧度转换为角度
        self.joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
        self.joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
        self.joint_state.effort = [0] * 14


    def plan_arm_trajectory(self, action_data):
        """规划手臂轨迹"""
        service_name = '/bezier/plan_arm_trajectory'
        try:
            rospy.wait_for_service(service_name)
            plan_service = rospy.ServiceProxy(service_name, planArmTrajectoryBezierCurve)
            req = self.create_bezier_request(action_data)
            res = plan_service(req)
            return res.success
        except rospy.ServiceException as e:
            rospy.logerr(f"轨迹规划服务调用失败: {e}")
            return False

    def create_bezier_request(self, action_data):
        """创建贝塞尔曲线请求"""
        req = planArmTrajectoryBezierCurveRequest()
        # 为每个关节创建贝塞尔曲线轨迹
        for key, value in action_data.items():
            msg = jointBezierTrajectory()
            for frame in value:
                point = bezierCurveCubicPoint()
                point.end_point = frame[0]
                point.left_control_point = frame[1]
                point.right_control_point = frame[2]
                msg.bezier_curve_points.append(point)
            req.multi_joint_bezier_trajectory.append(msg)
        
        req.start_frame_time = 0
        req.end_frame_time = 22  # 可根据实际需求调整
        req.joint_names = self.joint_state.name
        return req

    def execute_action(self, tact_file):
        """执行动作序列"""
        # 加载动作文件
        data = self.load_tact_file(tact_file)
        frames = data['frames']
        
        # 切换到轨迹控制模式
        self.change_control_mode(2)
        
        # 生成贝塞尔曲线轨迹数据（仅用于手臂）
        action_data = self.process_frames_to_bezier(frames)
        
        # 规划并执行轨迹
        if self.plan_arm_trajectory(action_data):
            rate = rospy.Rate(100)  # 100Hz控制频率
            frame_count = 0  # 帧计数器
            
            while not rospy.is_shutdown():
                # 发布手臂轨迹
                self.arm_pub.publish(self.joint_state)
                
                # 检查当前帧是否有对应的关键帧数据
                current_time = frame_count / 100.0  # 将帧数转换为秒
                for frame in frames:
                    if abs(frame['keyframe']/100.0 - current_time) < 0.01:  # 允许0.01秒的误差
                        # 处理当前帧的手指和头部数据
                        self.process_frame(frame)
                        # 发布手指和头部命令
                        self.hand_pub.publish(self.hand_state)
                        self.head_pub.publish(self.head_state)
                        break
                
                frame_count += 1
                rate.sleep()
                
                # 检查是否完成所有帧的执行
                if frame_count > (frames[-1]['keyframe'] * 2):  # 给予足够的时间完成动作
                    break
        
        # 恢复自然控制模式
        self.change_control_mode(1)
        rospy.sleep(5)

    def process_frames_to_bezier(self, frames):
        """处理帧数据为贝塞尔曲线格式"""
        action_data = {}
        for frame in frames:
            servos = frame['servos']
            keyframe = frame['keyframe']
            attribute = frame['attribute']
            
            for index, value in enumerate(servos[:14]):  # 只处理前14个关节
                key = index + 1
                if key not in action_data:
                    action_data[key] = []
                
                if value is not None:
                    CP = attribute[str(key)]['CP']
                    left_CP, right_CP = CP
                    
                    action_data[key].append([
                        [round(keyframe/100, 1), math.radians(value)],
                        [round((keyframe+left_CP[0])/100, 1), math.radians(value+left_CP[1])],
                        [round((keyframe+right_CP[0])/100, 1), math.radians(value+right_CP[1])]
                    ])
        
        return action_data

    def run(self):
        """主运行函数"""
        # 设置动作文件路径
        tact_file = rospy.get_param('~tact_file', './action_files/挥拳握手抬头.tact')
        
        try:
            self.execute_action(tact_file)
        except Exception as e:
            rospy.logerr(f"执行动作失败: {e}")
        finally:
            # 确保恢复自然控制模式
            self.change_control_mode(1)

if __name__ == '__main__':
    try:
        controller = ComprehensiveRobotControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass