#!/usr/bin/env python3

import rospy
import json
import math
import time
import threading
import numpy as np
from humanoid_plan_arm_trajectory.srv import planArmTrajectoryBezierCurve, planArmTrajectoryBezierCurveRequest
from humanoid_plan_arm_trajectory.msg import bezierCurveCubicPoint, jointBezierTrajectory
from kuavo_msgs.msg import robotHandPosition, robotHeadMotionData
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
from ocs2_msgs.msg import mpc_observation
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from humanoid_plan_arm_trajectory.msg import RobotActionState
from humanoid_plan_arm_trajectory.srv import ExecuteArmAction, ExecuteArmActionResponse  # Import new service type

class ArmTrajectoryBezierDemo:
    INIT_ARM_POS = [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # START_FRAME_TIME = 0
    END_FRAME_TIME = 10000

    def __init__(self):
        self.START_FRAME_TIME = 0
        self.x_shift = self.START_FRAME_TIME
        self.joint_state = JointState()
        self.hand_state = robotHandPosition()
        self.head_state = robotHeadMotionData()
        self.current_arm_joint_state = []
        self.running_action = False
        self.arm_flag = False
        # rospy.spin()


        # Initialize ROS node
        rospy.init_node('autostart_arm_trajectory_bezier_demo')
        
        # Subscribers and Publishers
        rospy.loginfo("***************************arm_trajectory_bezier_process_start*****************************************")
        self.traj_sub = rospy.Subscriber('/bezier/arm_traj', JointTrajectory, self.traj_callback, queue_size=1, tcp_nodelay=True)
        self.kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=1, tcp_nodelay=True)
        self.control_hand_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=1, tcp_nodelay=True)
        self.control_head_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=1, tcp_nodelay=True)

        self.mpc_obs_sub = rospy.Subscriber('/humanoid_mpc_observation', mpc_observation, self.mpc_obs_callback)
         # 添加发布者
        self.robot_action_state_pub = rospy.Publisher('/robot_action_state', RobotActionState, queue_size=1)

        
        # Add service to execute arm actions
        self.execute_service = rospy.Service('/execute_arm_action', ExecuteArmAction, self.handle_execute_action)
        
        # Store the file path base directory for actions
        # self.action_files_path = "/home/lab/kuavo-ros-control/src/humanoid-control/humanoid_plan_arm_trajectory/script/action_files"
        self.action_files_path = "/home/lab/.config/lejuconfig/action_files"
        rospy.loginfo("arm_trajectory_bezier_process is ready.")
        rospy.loginfo("***************************arm_trajectory_bezier_process_end*****************************************")

        # self.run()
        rospy.spin()



    def mpc_obs_callback(self, msg):
        self.current_arm_joint_state = msg.state.value[24:]
        self.current_arm_joint_state = [round(pos, 2) for pos in self.current_arm_joint_state]
        self.current_arm_joint_state.extend([0] * 14)

    def traj_callback(self, msg):
        if len(msg.points) == 0:
            return
        point = msg.points[0]
        # print("1111111111",point)
        self.joint_state.name = [
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
        self.joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
        # print("2222222",self.joint_state.position)
        self.joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
        self.joint_state.effort = [0] * 14

        # 修复 hand 和 head 的数据类型
        self.hand_state.left_hand_position = [max(0, int(math.degrees(pos))) for pos in point.positions[14:20]]  # 无符号整数
        self.hand_state.right_hand_position = [max(0, int(math.degrees(pos))) for pos in point.positions[20:26]]  # 无符号整数
        self.head_state.joint_data = [math.degrees(pos) for pos in point.positions[26:]]


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

    def load_json_file(self, file_path):
        try:
            with open(file_path, "r") as f:
                return json.load(f)
        except IOError as e:
            rospy.logerr(f"Error reading file {file_path}: {e}")
            return None

    def add_init_frame(self, frames):
        action_data = {}
        for frame in frames:
            servos, keyframe, attribute = frame["servos"], frame["keyframe"], frame["attribute"]
            for index, value in enumerate(servos):
                key = index + 1
                if key == 29:
                    break
                if key not in action_data:
                    action_data[key] = []
                    if keyframe != 0 and len(action_data[key]) == 0:
                        if key <= len(self.INIT_ARM_POS):
                            action_data[key].append([
                                [0, math.radians(self.INIT_ARM_POS[key-1])],
                                [0, math.radians(self.INIT_ARM_POS[key-1])],
                                [0, math.radians(self.INIT_ARM_POS[key-1])],
                    ])
                if value is not None:
                    CP = attribute[str(key)]["CP"]
                    left_CP, right_CP = CP
                    action_data[key].append([
                        [round(keyframe/100, 1), math.radians(value)],
                        [round((keyframe+left_CP[0])/100, 1), math.radians(value+left_CP[1])],
                        [round((keyframe+right_CP[0])/100, 1), math.radians(value+right_CP[1])],
                    ])
        return action_data

    def filter_data(self, action_data):
        filtered_action_data = {}
        for key, frames in action_data.items():
            filtered_frames = []
            found_start = False
            skip_next = False
            for i in range(-1, len(frames)):
                frame = frames[i]
                if i == len(frames) - 1:
                    next_frame = frame
                else:
                    next_frame = frames[i+1]
                end_time = next_frame[0][0]
                if not found_start and end_time >= self.START_FRAME_TIME:
                    found_start = True
                    p0 = np.array([0, self.current_arm_joint_state[key-15]])
                    p3 = np.array([next_frame[0][0] - self.x_shift, next_frame[0][1]])

                    # Calculate control points for smooth transition
                    curve_length = np.linalg.norm(p3 - p0)
                    p1 = p0 + curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the right
                    p2 = p3 - curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the left

                    # Create new frame
                    frame1 = [
                        p0.tolist(),
                        p0.tolist(),
                        p1.tolist()
                    ]

                    # Modify next_frame's left control point
                    next_frame[1] = p2.tolist()

                    filtered_frames.append(frame1)
                    skip_next = True

                if found_start:
                    if skip_next:
                        skip_next = False
                        continue
                    end_point = [round(frame[0][0] - self.x_shift, 1), round(frame[0][1], 1)]
                    left_control_point = [round(frame[1][0] - self.x_shift, 1), round(frame[1][1], 1)]
                    right_control_point = [round(frame[2][0] - self.x_shift, 1), round(frame[2][1], 1)]
                    filtered_frames.append([end_point, left_control_point, right_control_point])

            filtered_action_data[key] = filtered_frames
        return filtered_action_data

    def delayed_publish_action_state(self, delay):
        """
        延时发布动作完成状态。
        :param delay: 延迟时间（秒）
        """
        rospy.loginfo(f"Delaying action completion state for {delay} seconds...")
        rospy.sleep(delay)
        self.running_action = False  # 结束 state=1 的发布
        self.publish_action_state(2)
        self.arm_flag = False
        self.call_change_arm_ctrl_mode_service(1) # 做完动作之后恢复自然摆臂状态

    def publish_running_action_state(self):
        """持续发布 state=1"""
        rate = rospy.Rate(1)  # 每秒发布 2 次
        while self.running_action:
            self.publish_action_state(1)
            rate.sleep()


    def publish_action_state(self, state):
        """
        发布手臂动作状态
        :param state: 动作状态 (0: 失败, 1:执行 2: 成功)
        :param message: 状态描述信息
        """
        state_msg = RobotActionState()
        state_msg.state = state
        self.robot_action_state_pub.publish(state_msg)
        rospy.loginfo(f"Robot action state published: state={state}")

    def create_bezier_request(self, action_data):
        req = planArmTrajectoryBezierCurveRequest()
        for key, value in action_data.items():
            msg = jointBezierTrajectory()
            for frame in value:
                point = bezierCurveCubicPoint()
                point.end_point, point.left_control_point, point.right_control_point = frame
                msg.bezier_curve_points.append(point)
            req.multi_joint_bezier_trajectory.append(msg)
        req.start_frame_time = self.START_FRAME_TIME
        req.end_frame_time = self.END_FRAME_TIME
        req.joint_names = [
            "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch", 
            "l_hand_yaw", "l_hand_pitch", "l_hand_roll", 
            "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch", 
            "r_hand_yaw", "r_hand_pitch", "r_hand_roll"
        ]
        return req

    def plan_arm_trajectory_bezier_curve_client(self, req):
        service_name = '/bezier/plan_arm_trajectory'
        rospy.wait_for_service(service_name)
        try:
            plan_service = rospy.ServiceProxy(service_name, planArmTrajectoryBezierCurve)
            res = plan_service(req)
            return res.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def handle_execute_action(self, req):
        action_name = req.action_name
        self.call_change_arm_ctrl_mode_service(2)
       
        file_path = f"{self.action_files_path}/{action_name}.tact"
        data = self.load_json_file(file_path)
        if not data:
            self.publish_action_state(0)
            return ExecuteArmActionResponse(success=False, message=f"Action file {action_name} not found")

        # 获取初始帧时间
        self.arm_flag = True
        first_value = data.get("first", 0)
        self.START_FRAME_TIME = round(first_value * 0.01, 2)  # 转换为秒，取两位小数
        self.x_shift = self.START_FRAME_TIME  # 动态调整 x_shift

        # 读取动作完成时间
        finish_time = data.get("finish", 0) * 0.01  # 转换为秒
        self.END_FRAME_TIME = finish_time
        
        action_data = self.add_init_frame(data["frames"])
        filtered_data = self.filter_data(action_data)
        bezier_request = self.create_bezier_request(filtered_data)

        rospy.loginfo(f"Planning arm trajectory for action: {action_name}...")
        # 开始执行，持续发布 state=1
        self.running_action = True
        threading.Thread(target=self.publish_running_action_state).start()

        success = self.plan_arm_trajectory_bezier_curve_client(bezier_request)
        # self.call_change_arm_ctrl_mode_service(1)
        if success:
            rospy.loginfo("Arm trajectory planned successfully")
            threading.Thread(target=self.run).start()
            threading.Thread(target=self.delayed_publish_action_state, args=(finish_time,)).start()
            return ExecuteArmActionResponse(success=True, message="Action executed successfully")
        else:
            rospy.logerr("Failed to plan arm trajectory")
            self.publish_action_state(0)
            return ExecuteArmActionResponse(success=False, message="Failed to execute action")

    def run(self):
        rate = rospy.Rate(100)
        # while not rospy.is_shutdown():
        while self.arm_flag:
            try:
                if len(self.joint_state.position) == 0:
                    continue
                # self.call_change_arm_ctrl_mode_service(2)
                self.kuavo_arm_traj_pub.publish(self.joint_state)
                self.control_hand_pub.publish(self.hand_state)
                self.control_head_pub.publish(self.head_state)
            except Exception as e:
                rospy.logerr(f"Failed to publish arm trajectory: {e}")
            except KeyboardInterrupt:
                break
            rate.sleep()


if __name__ == "__main__":
    demo = ArmTrajectoryBezierDemo()
