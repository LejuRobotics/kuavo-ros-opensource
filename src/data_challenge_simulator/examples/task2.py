import os, sys
current_dir = os.path.dirname(os.path.abspath(__file__))
data_challenge_simulator_dir = os.path.dirname(current_dir)
if data_challenge_simulator_dir not in sys.path:
    sys.path.insert(0, data_challenge_simulator_dir)

from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotVision, KuavoManipulationMpcFrame, KuavoRobotState, KuavoPose
import time
from utils.gripper_controller import GripperController  # 使用高频版本
from utils.conveyor_controller import ConveyorController
from utils.trajectory_controller import TrajectoryController  # 新增的轨迹控制器
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.utils import Utils
import math, random
from typing import Tuple
from scipy.spatial.transform import Rotation as R

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
    gripper_ctrl = GripperController() 
    traj_ctrl = TrajectoryController(robot)
    obj_pos = ObjectPose()

    num = 20

    try:
        # 随机物体位置
        y_target = random.uniform(-0.5, -0.2)
        random_pos = ObjectRandomizer()
        result = random_pos.randomize_object_position(
            object_name='box_grab',
            position_ranges={
                'x': [0.4, 0.6],    # x轴范围
                'y': [-1.0, -1.0],   # y轴范围  
                'z': [0.95, 0.95]     # z轴范围
            }
        )
        if result['success']:
            print(f"物体随机化成功！新位置: {result['final_position']}")
        else:
            print(f"物体随机化失败: {result['message']}")
            
        robot.control_head(yaw=0, pitch=math.radians(12))

        # 预抓位1 - 使用高频轨迹控制器
        q_target1 = [0, 0, 0, 0, -90, 0, 0,   80, -30, 0, -130, 45, 0, 0]
        q_list1 = Utils.interpolate_joint_trajectory(q_target1, num=num) 
            
        q_target2 = [0, 0, 0, 0, -90, 0, 0,   20, 0, 0, -120, 90, 0, 0]
        q_list2 = Utils.interpolate_joint_trajectory(q_target2, q_target1, num=num)

        conveyor_ctrl.control_speed(0.1)
        print("双手移动到初始位置,传送带启动")
        
        # 使用高频轨迹控制器执行轨迹
        traj_ctrl.execute_trajectory(q_list1, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q_list2, sleep_time=0.02)
        time.sleep(0.7)

        # 计算俯仰角并调整姿态
        curr_q_rot = robot_state.arm_joint_state().position
        l_pose_rot, r_pose_rot = robot.arm_fk(curr_q_rot)
        pos_box1_rot = obj_pos.get_position("box_grab")

        curr_pos = r_pose_rot.position

        dx = pos_box1_rot[0] - curr_pos[0] + 0.12
        dy = y_target - curr_pos[1]
        pitch_des = math.atan2(dy, dx)

        q_target_pitch = [0, 0, 0, 0, -90, 0, 0,   20, 0, math.degrees(pitch_des)*0.5, -120, 90, 0, -math.degrees(pitch_des)*0.5]
        q_list_pitch = Utils.interpolate_joint_trajectory(q_target_pitch, q_target2, num=5)
        
        # 使用高频轨迹控制器执行快速俯仰调整
        traj_ctrl.execute_trajectory(q_list_pitch, sleep_time=0.01)
        time.sleep(0.7)

        curr_q = robot_state.arm_joint_state().position
        l_pose, r_pose = robot.arm_fk(curr_q)

        # 抓取 - 等待抓取姿态
        pos_box1 = obj_pos.get_position("box_grab")
        _, right_pose = Utils.wait_for_grasp_pose(
            robot=robot,
            robot_state=robot_state,
            y_grasp_right=y_target,
            quat_right=r_pose.orientation,
            v=0.08,
            mode="right",
            obj_name_right="box_grab",
            x_grasp_right=pos_box1[0]+0.12,
            z_grasp_right=pos_box1[2]+0.1,
            time_offset=0,
            move_lead_time=0.6,
            check_interval=0.0001)
        
        left_pose = [math.radians(x) for x in q_target2[0:7]]
        q_target_deg = [math.degrees(x) for x in left_pose+right_pose]
        q_list_target = Utils.interpolate_joint_trajectory(q_target_deg, q_target_pitch, num=15)
        
        # 使用高频轨迹控制器执行抓取准备动作
        traj_ctrl.execute_trajectory(q_list_target, sleep_time=0.02)
        time.sleep(0.5)
        
        # 右夹爪抓取 - 高频发布
        gripper_ctrl.control_right_gripper(140)
        time.sleep(0.5)
        
        # 移动到称 - 使用高频轨迹控制器
        q_target3 = [80, -30, 0, -130, -87, 0, 0,   -10, -5, 25, -130, 87, 0, 0]
        q_list3 = Utils.interpolate_joint_trajectory(q_target3, q_target_deg, num=num) 
        
        q_target4 = [50, 0, -20, -150, -90, 0, 0,   -30, 30, 37, -70, 87, 0, 0]
        q_list4 = Utils.interpolate_joint_trajectory(q_target4, q_target3, num=num)

        print("右手移动至称，打开夹爪")
        # 使用高频轨迹控制器
        traj_ctrl.execute_trajectory(q_list3, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q_list4, sleep_time=0.02)
        time.sleep(0.5)
        
        # 右夹爪释放 - 高频发布
        gripper_ctrl.control_right_gripper(0)

        # 左手开始抓取 - 使用高频轨迹控制器
        q_target5 = [20, 0, -25, -140, -87, 0, 0,   20, 0, 30, -150, 87, 0, 0]
        q_list5 = Utils.interpolate_joint_trajectory(q_target5, q_target4, num=num) 
        
        print("左手抓取")
        traj_ctrl.execute_trajectory(q_list5, sleep_time=0.02)

        time.sleep(0.5)
        
        # 计算左手精确抓取位置
        curr_q1 = robot_state.arm_joint_state().position
        l_pose1, r_pose1 = robot.arm_fk(curr_q1)
        pos_box1 = obj_pos.get_position("box_grab")
        l_pose_new1 = [pos_box1[0]+0.07, pos_box1[1]-0.02, l_pose1.position[2] + robot_state.robot_position()[2]]
        pose1_left, _ = Utils.compute_pose(
            robot,
            robot_state,
            mode="left",
            pos_left=l_pose_new1,
            quat_left=l_pose1.orientation,
        )
        
        pose1_right1 = [math.radians(x) for x in q_target5[7:14]]
        q_target1 = pose1_left + pose1_right1
        q_target_deg1 = [math.degrees(x) for x in q_target1]
        q_list_target1 = Utils.interpolate_joint_trajectory(q_target_deg1, q_target5, num=num)
        
        # 使用高频轨迹控制器执行精确抓取动作
        traj_ctrl.execute_trajectory(q_list_target1, sleep_time=0.02)

        time.sleep(0.5)
        # 左夹爪抓取 - 高频发布
        gripper_ctrl.control_left_gripper(140)

        # 左手放至目标位置 - 使用高频轨迹控制器
        q_target7 = [10, 0, -20, -130, -87, 0, 0,   50, -20, 20, -140, 87, 0, -20]
        q_list7 = Utils.interpolate_joint_trajectory(q_target7, q_target_deg1, num=num) 
        
        q_target8 = [-28, 20, 40, -85, -87, 0, 0,   0, 0, 0, 0, 0, 0, 0]
        q_list8 = Utils.interpolate_joint_trajectory(q_target8, q_target7, num=num)

        print("左手放置到目标位置")
        # 使用高频轨迹控制器
        traj_ctrl.execute_trajectory(q_list7, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q_list8, sleep_time=0.02)
        time.sleep(0.5)
        
        # 左夹爪释放 - 高频发布
        gripper_ctrl.control_left_gripper(0)

        time.sleep(0.5)
        
        # 检查任务完成情况
        pos = obj_pos.get_position("box_grab")

        target_region = [
            (0.26, 0.52),   # x 范围
            (0.48, 0.76),   # y 范围
            (0.85, 1.5)     # z 范围
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