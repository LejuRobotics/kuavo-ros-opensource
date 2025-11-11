import os, sys
current_dir = os.path.dirname(os.path.abspath(__file__))
data_challenge_simulator_dir = os.path.dirname(current_dir)
if data_challenge_simulator_dir not in sys.path:
    sys.path.insert(0, data_challenge_simulator_dir)
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState
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
from utils.pose_controller import PoseController
def main():
    # Initialize SDK
    if not KuavoSDK().Init(options=KuavoSDK.Options.WithIK):
        print("Init KuavoSDK failed, exit!")
        exit(1)
    # Initialize robot
    robot = KuavoRobot()
    robot_state = KuavoRobotState()

    # Initialize controllers with high-frequency publishing
    conveyor_ctrl = ConveyorController()
    gripper_ctrl = GripperController(publish_frequency=50) 

    obj_pos = ObjectPose()

    randomizer = ObjectRandomizer()
    object_configs = [
        {"name": "item1",
        "position_ranges": {
                    'x': [1.28, 1.38],
                    'y': [-0.6, -0.1],#1-6
                    'z': [0.9, 0.9]
                }},

        {"name": "item2",
        "position_ranges": {
                    'x': [1.28, 1.38],
                    'y': [0.1, 0.6],
                    'z': [0.9, 0.9]
                }},

        {"name": "left_bin_A",
        "position_ranges": {
                    'x': [-1.35, -1.35],
                    'y': [0.15, 0.6],
                    'z': [0.76, 0.76]
                },
        "orientation": {'w': 0.7071, 'x': 0, 'y': 0, 'z': 0.7071}},

        {"name": "left_bin_B",
        "position_ranges": {
                    'x': [-1.35, -1.35],
                    'y': [-0.6, -0.15],
                    'z': [0.76, 0.76]
                },
        "orientation": {'w': 0.7071, 'x': 0, 'y': 0, 'z': 0.7071}},
    ]

    randomizer.randomize_multiple_objects(object_configs)

    conveyor_ctrl.control_speed(-0.1)
    num = 10

    item1_pos = obj_pos.wait_for_position("item1")
    item2_pos = obj_pos.wait_for_position("item2")
    left_bin_A_pos = obj_pos.wait_for_position("left_bin_A")
    left_bin_B_pos = obj_pos.wait_for_position("left_bin_B")

    # #=========================第一段走路与抓取=============================
    try:
        ##robot.set_external_control_arm_mode()
        robot.control_head(yaw=0, pitch=math.radians(12))

        traj_ctrl = TrajectoryController(robot,publish_frequency=150)
        pos_ctrl = PoseController(robot, publish_frequency=1200)

        q0_target1 = [0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 90, 0, 0]
        q0_list1 = Utils.interpolate_joint_trajectory(q0_target1, num = num)
        traj_ctrl.execute_trajectory(q0_list1, sleep_time=0.001)
        time.sleep(0.5)

        print("行走到目标区域")
        # robot.stance()
        offset1 = 0.3

        # #robot.set_external_control_arm_mode()
        pos_ctrl.move_to_target([0.9, item1_pos[1]+offset1, 0, 0],0.1)
        # #robot.set_external_control_arm_mode()

        time.sleep(3.5)
        robot.stance()

        print("开始抓取物体")
        q1_target1 = [0, 0, 0, 0, 0, 0, 0,   30, 0, 0, -130, 90, 0, 0]
        q1_list1 = Utils.interpolate_joint_trajectory(q1_target1,q0_target1, num = num)
        traj_ctrl.execute_trajectory(q1_list1, sleep_time=0.01)

        time.sleep(0.5)
        
        curr_q1 = robot_state.arm_joint_state().position
        l_pose1, r_pose1 = robot.arm_fk(curr_q1)
        r_pose_new1 = [item1_pos[0]+0.01, item1_pos[1], 0.95]
        _, pose1_right1 = Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=r_pose_new1,
            quat_right=r_pose1.orientation,
        )
        
        pose1_left1 = [math.radians(x) for x in q1_target1[0:7]]
        q_target1 = pose1_left1 + pose1_right1
        q_target_deg1 = [math.degrees(x) for x in q_target1]
        q_list_target1 = Utils.interpolate_joint_trajectory(q_target_deg1, q1_target1, num=num)
        traj_ctrl.execute_trajectory(q_list_target1, sleep_time=0.01)

        time.sleep(0.5)
        print("关闭夹爪并提起")
        gripper_ctrl.control_right_gripper(180)
        time.sleep(0.25)


        q1_target2 = [0, 0, 0, 0, 0, 0, 0,   -15, 15, 25, -120, 90, 0, 0]
        q1_list2 = Utils.interpolate_joint_trajectory(q1_target2,q_target_deg1, num = num)
        traj_ctrl.execute_trajectory(q1_list2, sleep_time=0.01)
        time.sleep(0.25)
        # #=========================第一段走路与抓取=============================

        # #=========================第二段走路与放置=============================
        print("行走至放置地区")

        # robot.stance()

        # #robot.set_external_control_arm_mode()
        pos_ctrl.move_to_target([0.6, item1_pos[1]+offset1, 0, 0],0.1)
        # #robot.set_external_control_arm_mode()

        time.sleep(2.25)

        offset2 = -0.3

        # #robot.set_external_control_arm_mode()
        pos_ctrl.move_to_target([-0.85, left_bin_A_pos[1]+offset2, 0, 3.14],0.5)
        # #robot.set_external_control_arm_mode()

        time.sleep(10.5)
        robot.stance()

        print("放置于右侧盒子")

        q1_target3 = [0, 0, 0, 0, 0, 0, 0,   -10, 0, -10, -95, 90, 0, 0]
        q1_list3 = Utils.interpolate_joint_trajectory(q1_target3,q1_target2, num = num)
        traj_ctrl.execute_trajectory(q1_list3, sleep_time=0.01)

        time.sleep(0.5)
        gripper_ctrl.control_right_gripper(0)
        time.sleep(0.25)
        # #=========================第二段走路与放置=============================


        # #=========================第三段走路与抓取=============================
        print("返回到目标区域")
        # robot.stance()
        
        # #robot.set_external_control_arm_mode()
        pos_ctrl.move_to_target([-0.55, left_bin_A_pos[1]+offset2, 0, 3.14],0.1)
        # #robot.set_external_control_arm_mode()

        time.sleep(2.25)
        robot.stance()

        q1_target4 = [0, 0, 0, 0, -90, 0, 0,   0, 0, 0, 0, 0, 0, 0]
        q1_list4 = Utils.interpolate_joint_trajectory(q1_target4,q1_target3, num = num)
        traj_ctrl.execute_trajectory(q1_list4, sleep_time=0.01)
        # robot.arm_reset()
        offset3 = -0.3

        # #robot.set_external_control_arm_mode()
        pos_ctrl.move_to_target([0.9, item2_pos[1]+offset3-0.02, 0, 0],0.5)#-0.0225
        # #robot.set_external_control_arm_mode()
        time.sleep(11)
        robot.stance()

        print("左手开始抓取")
        q1_target5 = [30, 0, 0, -125, -90, 0, 0,   0, 0, 0, 0, 0, 0, 0]
        q1_list5 = Utils.interpolate_joint_trajectory(q1_target5,q1_target4, num = num)
        traj_ctrl.execute_trajectory(q1_list5, sleep_time=0.001)
        error = item2_pos[1]-robot_state.robot_position()[1]-0.3

        time.sleep(0.5)
        
        curr_q2_1 = robot_state.arm_joint_state().position
        l_pose2_1, r_pose2 = robot.arm_fk(curr_q2_1)

        offset_grasp = 0
        if item2_pos[1]>=0.48 and error > 0.01:
            offset_grasp = 0.015
        elif error > 0.01:
            offset_grasp = 0.005
        elif item2_pos[1]>=0.48:
            offset_grasp = 0.01

        l_pose_new2_1 = [item2_pos[0]-0.1, item2_pos[1]+offset_grasp, 0.93]
        l_pose_new2 = [item2_pos[0]+0.01, item2_pos[1]+offset_grasp, 0.93]
                
        pose1_left2_1 , _ = Utils.compute_pose(
            robot,
            robot_state,
            mode="left",
            pos_left=l_pose_new2_1,
            quat_left=l_pose2_1.orientation,
        )
        
        pose1_right2_1 = [math.radians(x) for x in q1_target5[7:14]]
        q_target2_1 = pose1_left2_1 + pose1_right2_1
        q_target_deg2_1 = [math.degrees(x) for x in q_target2_1]
        q_list_target2_1 = Utils.interpolate_joint_trajectory(q_target_deg2_1, q1_target5, num=num)
        traj_ctrl.execute_trajectory(q_list_target2_1, sleep_time=0.01)
        time.sleep(0.1)

        curr_q2 = robot_state.arm_joint_state().position
        l_pose2, r_pose2 = robot.arm_fk(curr_q2)

        pose1_left2 , _ = Utils.compute_pose(
            robot,
            robot_state,
            mode="left",
            pos_left=l_pose_new2,
            quat_left=l_pose2.orientation,
        )
        
        pose1_right2 = [math.radians(x) for x in q1_target5[7:14]]
        q_target2 = pose1_left2 + pose1_right2
        q_target_deg2 = [math.degrees(x) for x in q_target2]
        q_list_target2 = Utils.interpolate_joint_trajectory(q_target_deg2, q_target_deg2_1, num=num)
        traj_ctrl.execute_trajectory(q_list_target2, sleep_time=0.001)

        time.sleep(0.5)
        print("关闭夹爪并提起")
        gripper_ctrl.control_left_gripper(180)

        q1_target6 = [-15, -15, -25, -120, -90, 0, 0,   0, 0, 0, 0, 0, 0, 0]
        q1_list6 = Utils.interpolate_joint_trajectory(q1_target6,q_target_deg2, num = num)
        traj_ctrl.execute_trajectory(q1_list6, sleep_time=0.01)
        time.sleep(0.25)
        # #=========================第三段走路与抓取=============================


        # #=========================第四段走路与放置=============================
        print("行走至放置地区")
        # robot.stance()

        ##robot.set_external_control_arm_mode()
        pos_ctrl.move_to_target([0.6, item2_pos[1]+offset3, 0, 0],0.1)
        ##robot.set_external_control_arm_mode()

        time.sleep(2.25)

        offset4 = 0.3

        ##robot.set_external_control_arm_mode()
        pos_ctrl.move_to_target([-0.85, left_bin_B_pos[1]+offset4, 0, 3.14],0.5)
        ##robot.set_external_control_arm_mode()

        time.sleep(10.5)
        robot.stance()

        print("放置于左侧盒子")
        q1_target7 = [-10, 0, 7, -95, -90, 0, 0,   0, 0, 0, -0, 0, 0, 0]
        q1_list7 = Utils.interpolate_joint_trajectory(q1_target7,q1_target6, num = num)
        traj_ctrl.execute_trajectory(q1_list7, sleep_time=0.01)
        time.sleep(0.5)
        gripper_ctrl.control_left_gripper(0)
        time.sleep(0.5)
        print("\033[91mERROR\033[0m",error)
        #=========================第四段走路与放置=============================

        pos1 = obj_pos.get_position("item1")

        target_region1 = [
            (left_bin_A_pos[0]-0.12, left_bin_A_pos[0]+0.12),   # x 范围
            (left_bin_A_pos[1]-0.14, left_bin_A_pos[1]+0.14),   # y 范围
            (0.76, 0.9)  # z 范围
        ]

        pos2 = obj_pos.get_position("item2")

        target_region2 = [
            (left_bin_B_pos[0]-0.12, left_bin_B_pos[0]+0.12),   # x 范围
            (left_bin_B_pos[1]-0.14, left_bin_B_pos[1]+0.14),   # y 范围
            (0.76, 0.9)  # z 范围
        ]

        if Utils.is_in_target_region(pos1, target_region1) and Utils.is_in_target_region(pos2, target_region2):
            print("✅ 任务成功")
            with open("task_result.txt", "w") as f:
                f.write("success")
        else:
            print("❌ 任务失败")
            with open("task_result.txt", "w") as f:
                f.write("fail")
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序执行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 确保所有控制器正确停止
        print("正在停止控制器...")
        gripper_ctrl.stop()
        traj_ctrl.stop()
        print("控制器已停止")

if __name__ == "__main__":
        main()