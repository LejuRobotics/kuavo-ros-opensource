import os, sys
current_dir = os.path.dirname(os.path.abspath(__file__))
data_challenge_simulator_dir = os.path.dirname(current_dir)
if data_challenge_simulator_dir not in sys.path:
    sys.path.insert(0, data_challenge_simulator_dir)

from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState
import time
from utils.gripper_controller import GripperController
from utils.conveyor_controller import ConveyorController
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.trajectory_controller import TrajectoryController  # 新增的轨迹控制器
from utils.utils import Utils
import math
from typing import Tuple


def main():
    # Initialize SDK
    if not KuavoSDK().Init(options=KuavoSDK.Options.WithIK):
        print("Init KuavoSDK failed, exit!")
        exit(1)
    
    # Initialize robot
    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    
    # Initialize controllers
    conveyor_ctrl = ConveyorController()
    gripper_ctrl = GripperController() 
    traj_ctrl = TrajectoryController(robot)
    obj_pos = ObjectPose()
    num = 20

    try:
        # 随机物体位置
        random_pos = ObjectRandomizer()
        result = random_pos.randomize_object_position(
            object_name='box_grab',
            position_ranges={
                'x': [0.8, 0.8],    # x轴范围
                'y': [0.45, 0.65],   # y轴范围  
                'z': [0.95, 0.95]     # z轴范围
            }
        )
        if result['success']:
            print(f"物体随机化成功！新位置: {result['final_position']}")
        else:
            print(f"物体随机化失败: {result['message']}")

        robot.control_head(yaw=0, pitch=math.radians(12))
        # 预抓位
        q_target1 = [0, 0, 0, -105, -70, 0, 0,   30, 0, 0, -140, 90, 0, 0]
        q_list1 = Utils.interpolate_joint_trajectory(q_target1, num=num) 
        
        q_target2 = [-10, 15, 25, -100, -120, 0, 0,   30, 0, 0, -120, 90, 0, 0]
        q_list2 = Utils.interpolate_joint_trajectory(q_target2, q_target1, num=num)

        q_target3 = [-10, 15, 25, -95, -180, 25, -20,   30, 0, 0, -140, 90, 0, 0]
        q_list3 = Utils.interpolate_joint_trajectory(q_target3, q_target2, num=num)

        conveyor_ctrl.control_speed(-0.1)
        print("双手移动到初始位置,传送带启动")
        
        # 使用轨迹控制器执行轨迹
        traj_ctrl.execute_trajectory(q_list1, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q_list2, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q_list3, sleep_time=0.02)
        time.sleep(0.5)

        # 计算新的目标位置
        curr_q = robot_state.arm_joint_state().position
        l_pose, r_pose = robot.arm_fk(curr_q)
        pos_box = obj_pos.get_position("box_grab")
        offset = 0.03
        if pos_box[1] <= 0.5:
            offset = 0.015
        elif pos_box[1] >= 0.55:
            offset = 0.055

        l_pose_new = [l_pose.position[0], pos_box[1]+offset, l_pose.position[2] + robot_state.robot_position()[2]+0.05]
        pose1_left, _ = Utils.compute_pose(
            robot,
            robot_state,
            mode="left",
            pos_left=l_pose_new,
            quat_left=l_pose.orientation,
        )
        
        pose1_right = [math.radians(x) for x in q_target3[7:14]]
        q_target = pose1_left + pose1_right
        q_target_deg = [math.degrees(x) for x in q_target]
        q_list_target = Utils.interpolate_joint_trajectory(q_target_deg, q_target3, num=num)
        
        # 使用高频轨迹控制器
        traj_ctrl.execute_trajectory(q_list_target, sleep_time=0.02)

        time.sleep(2.5)
        gripper_ctrl.control_left_gripper(140)  # 高频发布
        time.sleep(0.5)

        # 继续后续动作
        q_target4 = [-10, 15, 25, -120, -180, 0, -20,    30, 0, 0, -140, 90, 0, 0]
        q_list4 = Utils.interpolate_joint_trajectory(q_target4, q_target_deg, num=num)

        q_target5 = [-20, 15, -20, -120, -180, 0, -20,    30, 0, 0, -140, 90, 0, 0]
        q_list5 = Utils.interpolate_joint_trajectory(q_target5, q_target4, num=num)

        q_target6 = [-40, -20, -30, -80, -160, 0, -10,  30, 0, 0, -140, 90, 0, 0]
        q_list6 = Utils.interpolate_joint_trajectory(q_target6, q_target5, num=num)

        # 使用高频轨迹控制器
        traj_ctrl.execute_trajectory(q_list4, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q_list5, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q_list6, sleep_time=0.02)
        
        time.sleep(0.5)
        gripper_ctrl.control_left_gripper(0)  # 高频发布
        time.sleep(0.5)

        # 右臂动作
        q_target7 = [-40, -25, -10, -90, -160, 0, -10,     15, 0, 20, -140, 90, 0, 0]
        q_list7 = Utils.interpolate_joint_trajectory(q_target7, q_target6, num=num)

        q_target8 = [-50, 0, 0, -70, 100, 0, -10,         10, 0, 20, -110, 90, 0, -15]
        q_list8 = Utils.interpolate_joint_trajectory(q_target8, q_target7, num=num)

        # 使用高频轨迹控制器
        traj_ctrl.execute_trajectory(q_list7, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q_list8, sleep_time=0.02)

        time.sleep(0.5)

        # 计算右臂新位置
        curr_q1 = robot_state.arm_joint_state().position
        l_pose1, r_pose1 = robot.arm_fk(curr_q1)
        pos_box1 = obj_pos.get_position("box_grab")
        r_pose_new1 = [pos_box1[0]+0.05, pos_box1[1]+0.05, r_pose1.position[2] + robot_state.robot_position()[2]+0.1]
        _, pose1_right = Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=r_pose_new1,
            quat_right=r_pose1.orientation,
        )
        
        pose1_left1 = [math.radians(x) for x in q_target8[0:7]]
        q_target1 = [0,0,0,0,0,0,0] + pose1_right
        q_target_deg1 = [math.degrees(x) for x in q_target1]
        q_list_target1 = Utils.interpolate_joint_trajectory(q_target_deg1, q_target8, num=num)
        
        # 使用高频轨迹控制器
        traj_ctrl.execute_trajectory(q_list_target1, sleep_time=0.02)

        time.sleep(0.5)
        gripper_ctrl.control_right_gripper(140)  # 高频发布
        time.sleep(0.5)

        # 最后的动作
        q_target10 = [-0, 0, 0, 0, 0, 0, 0,                -45, 0, 30, -100, 90, 0, 0]
        q_list10 = Utils.interpolate_joint_trajectory(q_target10, q_target_deg1, num=num)

        q_target11 = [0, 0, 0, 0, 0, 0, 0,                -35, 0, -25, -90, 90, 0, 0]
        q_list11 = Utils.interpolate_joint_trajectory(q_target11, q_target10, num=num)

        # 使用高频轨迹控制器
        traj_ctrl.execute_trajectory(q_list10, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q_list11, sleep_time=0.02)
        
        time.sleep(0.5)
        gripper_ctrl.control_right_gripper(0)  # 高频发布
        time.sleep(0.5)

        # 检查任务完成情况
        pos = obj_pos.get_position("box_grab")
        target_region = [
            (0.36, 0.62),   # x 范围
            (-0.6, -0.32),   # y 范围
            (0.85, 1.5)  # z 范围
        ]
        if Utils.is_in_target_region(pos, target_region):
            print("\033[92m✅ 任务成功\033[0m")
            with open("task_result.txt", "w") as f:
                f.write("success")
        else:
            print("\033[91m❌ 任务失败\033[0m")
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