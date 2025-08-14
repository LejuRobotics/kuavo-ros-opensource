import os, sys
current_dir = os.path.dirname(os.path.abspath(__file__))
data_challenge_simulator_dir = os.path.dirname(current_dir)
if data_challenge_simulator_dir not in sys.path:
    sys.path.insert(0, data_challenge_simulator_dir)
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoPose, KuavoManipulationMpcFrame, KuavoRobotState, KuavoRobotInfo, KuavoRobotArm
import rospy
import time
from utils.gripper_controller import GripperController  # 使用高频版本
from utils.conveyor_controller import ConveyorController
from utils.trajectory_controller import TrajectoryController  # 新增的轨迹控制器
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.utils import Utils
import math
from typing import Tuple
from utils.object_pos import ObjectPose
def main():
    # Initialize SDK
    if not KuavoSDK().Init():
        print("Init KuavoSDK failed, exit!")
        exit(1)
    # Initialize robot
    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    robot_info = KuavoRobotInfo()
    running = True
    # Initialize robot control
    gripper_ctrl = GripperController()
    conveyor_ctrl = ConveyorController()
    obj_pos = ObjectPose()

    target_poses1 = [
        [1.5, [0, 0, 0, 0, 0, 0, 0,   30, 0, 15, -130, 90, 0, 0]],
        [3.0, [0, 0, 0, 0, 0, 0, 0,   -15, 0, 25, -80, 90, 0, 0]],
    ]
    # target_pose1_1_left = Pose.from_euler(pos =(0, 0, -0.270229), euler= (0.0374, -14.5817, 0.0205), frame=Frame.BASE, degrees=True)
    # taregt_pose1_1_right = Pose.from_euler(pos =(0.5,-0.05,0.6), euler= (0,0,0), frame=Frame.BASE, degrees=True)
    # pose1_left = KuavoPose(position=target_pose1_1_left.pos, orientation=target_pose1_1_left.quat)
    # pose1_right = KuavoPose(position=taregt_pose1_1_right.pos, orientation=taregt_pose1_1_right.quat)
    times1 = [pose[0] for pose in target_poses1]
    q_frames1 = [[math.radians(angle) for angle in pose[1]] for pose in target_poses1]
    

    target_poses2 = [
        [1.0, [0, 0, 0, 0, 0, 0, 0,   -15, 15, 25, -120, 90, 0, 0]],

    ]
    
    times2 = [pose[0] for pose in target_poses2]
    q_frames2 = [[math.radians(angle) for angle in pose[1]] for pose in target_poses2]

    target_poses3 = [
        [1.0, [0, 0, 0, 0, 0, 0, 0,   -10, 0, -10, -100, 90, 0, 0]],
    ]
    
    times3 = [pose[0] for pose in target_poses3]
    q_frames3 = [[math.radians(angle) for angle in pose[1]] for pose in target_poses3]


    target_poses4 = [
        [1.0, [0, 0, 0, 0, 0, 0, 0,   20, 0, -10, -130, 90, 0, 0]],
        [5.0, [0, 0, 0, 0, 90, 0, 0,   0, 0, 0, 0, 0, 0, 0]],
    ]
    
    times4 = [pose[0] for pose in target_poses4]
    q_frames4 = [[math.radians(angle) for angle in pose[1]] for pose in target_poses4]

    target_poses5 = [
        [1.0, [30, 0, -15, -130, 90, 0, 0,   0, 0, 0, 0, 0, 0, 0]],
        [2.0, [-22, 0, -27, -72, 90, 10, 0,   0, 0, 0, 0, 0, 0, 0]],
    ]
    
    times5 = [pose[0] for pose in target_poses5]
    q_frames5 = [[math.radians(angle) for angle in pose[1]] for pose in target_poses5]


    target_poses6 = [
        [1.0, [-15, -15, -25, -120, 90, 0, 0,   0, 0, 0, 0, 0, 0, 0]],

    ]
    
    times6 = [pose[0] for pose in target_poses6]
    q_frames6 = [[math.radians(angle) for angle in pose[1]] for pose in target_poses6]


    target_poses7 = [
        [1.0, [-10, 0, 7, -100, 90, 0, 0,   0, 0, 0, -0, 0, 0, 0]],
    ]
    
    times7 = [pose[0] for pose in target_poses7]
    q_frames7 = [[math.radians(angle) for angle in pose[1]] for pose in target_poses7]

    conveyor_ctrl.control_speed(-0.1)
    print("行走到目标区域")
    start_x1 = robot_state.odometry.position[0]
    robot.stance()
    robot.control_command_pose_world(0.9, 0, 0.0, 0)
    time.sleep(4)
    robot.stance()
    forward_distance1 = robot_state.odometry.position[0] - start_x1
    print(f"\033[33mForward distance traveled: {forward_distance1:.3f} m\033[0m")
    
    # robot.control_robot_end_effector_pose(pose1_left,pose1_right,KuavoManipulationMpcFrame.WorldFrame)
    print("开始抓取物体")
    robot.control_arm_joint_trajectory(times1,q_frames1)
    time.sleep(4.0)
    print("关闭夹爪并提起")
    gripper_ctrl.set_gripper_position(0,150)
    time.sleep(0.5)
    robot.control_arm_joint_trajectory(times2,q_frames2)
    time.sleep(1)

    print("行走至放置地区")
    start_x2 = robot_state.odometry.position[0]
    robot.stance()
    robot.control_command_pose(-0.3, 0, 0.0, 0)
    time.sleep(2.5)
    robot.control_command_pose_world(-0.85, 0, 0.0, -3.14)
    time.sleep(12.5)
    robot.stance()
    forward_distance2 = robot_state.odometry.position[0] - start_x2
    print(f"\033[33mForward distance traveled: {forward_distance2:.3f} m\033[0m")

    print("放置于右侧盒子")
    robot.control_arm_joint_trajectory(times3,q_frames3)
    time.sleep(1.5)
    gripper_ctrl.set_gripper_position(0,0)
    time.sleep(0.5)
    robot.control_arm_joint_trajectory(times4,q_frames4)

    print("返回到目标区域")
    start_x3 = robot_state.odometry.position[0]
    robot.stance()
    robot.control_command_pose(-0.3, 0, 0.0, 0)
    time.sleep(2.5)
    robot.control_command_pose_world(0.9, 0, 0.0, 0)
    time.sleep(12.5)
    robot.stance()
    forward_distance3 = robot_state.odometry.position[0] - start_x3
    print(f"\033[33mForward distance traveled: {forward_distance3:.3f} m\033[0m")

    print("左手开始抓取")
    robot.control_arm_joint_trajectory(times5,q_frames5)
    time.sleep(4)
    print("关闭夹爪并提起")
    gripper_ctrl.set_gripper_position(150,0)
    time.sleep(1.0)
    robot.control_arm_joint_trajectory(times6,q_frames6)
    time.sleep(0.5)

    print("行走至放置地区")
    start_x4 = robot_state.odometry.position[0]
    robot.stance()
    robot.control_command_pose(-0.3, 0, 0.0, 0)
    time.sleep(2.5)
    robot.control_command_pose_world(-0.85, 0, 0.0, 3.14)
    time.sleep(12.5)
    robot.stance()
    forward_distance4 = robot_state.odometry.position[0] - start_x4
    print(f"\033[33mForward distance traveled: {forward_distance4:.3f} m\033[0m")


    print("放置于左侧盒子")
    robot.control_arm_joint_trajectory(times7,q_frames7)
    time.sleep(1.5)
    gripper_ctrl.set_gripper_position(0,0)
    time.sleep(1.0)

    pos1 = obj_pos.get_position("item1")

    target_region1 = [
        (-1.535, -1.165),   # x 范围
        (-0.445, -0.155),   # y 范围
        (0.76, 1)  # z 范围
    ]

    pos2 = obj_pos.get_position("item2")
    target_region2 = [
        (-1.535, -1.165),   # x 范围
        (0.155, 0.445),   # y 范围
        (0.76, 1)  # z 范围
    ]

    if Utils.is_in_target_region(pos1, target_region1) and Utils.is_in_target_region(pos2, target_region2):
        print("✅ 任务成功")
        with open("task_result.txt", "w") as f:
            f.write("success")
    else:
        print("❌ 任务失败")
        with open("task_result.txt", "w") as f:
            f.write("fail")


if __name__ == "__main__":
        main()