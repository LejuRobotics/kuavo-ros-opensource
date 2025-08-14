import os, sys
current_dir = os.path.dirname(os.path.abspath(__file__))
data_challenge_simulator_dir = os.path.dirname(current_dir)
if data_challenge_simulator_dir not in sys.path:
    sys.path.insert(0, data_challenge_simulator_dir)

from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotVision,KuavoManipulationMpcFrame, KuavoRobotState, KuavoPose
import time
from utils.gripper_controller import GripperController  # 使用高频版本
from utils.conveyor_controller import ConveyorController
from utils.trajectory_controller import TrajectoryController  # 新增的轨迹控制器
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.utils import Utils
import math,random
from typing import Tuple
import mujoco
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
    gripper_ctrl = GripperController()  # 100Hz gripper commands
    traj_ctrl = TrajectoryController(robot)  # 100Hz trajectory commands
    obj_pos = ObjectPose()
    
    num = 50
    # 定义随机范围
    REGIONS = {
        "shampoo1": {"x": (0.18, 0.23), "y": (-0.22, -0.12), "z": (0.72, 0.72)},  # 区域 A
        "shampoo2": {"x": (0.18, 0.23), "y": (-0.38, -0.28), "z": (0.72, 0.72)},  # 区域 B
        "shampoo3": {"x": (0.35, 0.38), "y": (-0.22, -0.12), "z": (0.72, 0.72)},  # 区域 C
        "shampoo4": {"x": (0.35, 0.38), "y": (-0.38, -0.28), "z": (0.72, 0.72)},  # 区域 D
    }
    FRONT = [0.5, -0.5, 0.5, -0.5]
    BACK  = [0.5,  0.5, 0.5,  0.5]

    randomizer = ObjectRandomizer()

    # 随机选择两个物体为正面
    all_objects = list(REGIONS.keys())
    front_objects = random.sample(all_objects, 2)
    back_objects = [obj for obj in all_objects if obj not in front_objects]
    object_configs = []
    for name, region in REGIONS.items():
        r = region
        x, y, z = random.uniform(*r["x"]), random.uniform(*r["y"]), random.uniform(*r["z"]),
        
        if name in front_objects:
            qx, qy, qz, qw = FRONT
        else:
            qx, qy, qz, qw = BACK

        object_configs.append({
            "name": name,
            "position_ranges": {
                "x": list(region["x"]),
                "y": list(region["y"]),
                "z": list(region["z"]),
            },
            "orientation": {  # xyzw 顺序
                "x": qx, "y": qy, "z": qz, "w": qw,
            },
        })

    randomizer.randomize_multiple_objects(object_configs)
    obj1 = back_objects[0]
    obj2 = back_objects[1]

    try:
        robot.control_head(yaw=0, pitch=math.radians(12))
        ### 抓取第一个反面物体
        ##  到达预抓位 - 使用高频轨迹控制器
        q1_target1 = [60, 0, 0, -100, 0, 0, 0,   70, 0, 0, -135, 70, 40, 0]
        q1_list1 = Utils.interpolate_joint_trajectory(q1_target1, num = num) 
            
        q1_target2 = [10, 0, 0, -130, 70, 0, 0,   10, 5, 0, -135, 90, 90, 0]
        q1_list2 = Utils.interpolate_joint_trajectory(q1_target2, q1_target1, num = num)

        print("移动到第一个物体预抓位")
        # 使用高频轨迹控制器
        traj_ctrl.execute_trajectory(q1_list1, sleep_time=0.02)
        time.sleep(0.2)
        traj_ctrl.execute_trajectory(q1_list2, sleep_time=0.02)
        time.sleep(1)

        ## 抓取第一个物体
        # 先到达正上方
        curr_q1 = robot_state.arm_joint_state().position
        l_pose1, r_pose1 = robot.arm_fk(curr_q1)
        pos_obj1 = obj_pos.get_position(obj1)
        offset1_x = 0.065
        offset1_y = -0.016
        offset1_z = 0.05
        r_pose1_new1 = [pos_obj1[0]+offset1_x, pos_obj1[1]+offset1_y, r_pose1.position[2]+robot_state.robot_position()[2]]
        r_pose1_new2 = [pos_obj1[0]+offset1_x, pos_obj1[1]+offset1_y, pos_obj1[2]+offset1_z] 

        [], pose1_right1= Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=r_pose1_new1,
            quat_right=r_pose1.orientation,
        )
        
        pose1_left = [math.radians(x) for x in q1_target2[0:7]]
        q_obj1_1 = pose1_left+pose1_right1
        q_obj1_deg_1 = [math.degrees(x) for x in q_obj1_1]
        q_list_obj1_1 = Utils.interpolate_joint_trajectory(q_obj1_deg_1, q1_target2, num = 50)
        
        print("移动到第一个物体正上方")
        traj_ctrl.execute_trajectory(q_list_obj1_1, sleep_time=0.02)
        time.sleep(0.5)
        
        # 再下去抓
        curr_q1_2 = robot_state.arm_joint_state().position
        l_pose2_2, r_pose1_2 = robot.arm_fk(curr_q1_2)
        [], pose1_right2= Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=r_pose1_new2,
            quat_right=r_pose1_2.orientation,
        )
        
        pose1_left = [math.radians(x) for x in q1_target2[0:7]]
        q_obj1_2 = pose1_left+pose1_right2
        q_obj1_deg_2 = [math.degrees(x) for x in q_obj1_2]
        q_list_obj1_2 = Utils.interpolate_joint_trajectory(q_obj1_deg_2, q_obj1_deg_1, num = 50)

        print("下降到第一个物体抓取位置")
        traj_ctrl.execute_trajectory(q_list_obj1_2, sleep_time=0.02)

        time.sleep(0.5)
        # 右夹爪抓取 - 高频发布
        gripper_ctrl.control_right_gripper(130)
        time.sleep(0.5)

        # 后续动作 - 使用高频轨迹控制器
        q1_target3 = [-10, 20, -40, -110, -40, 0, -50,   -15, 0, 30, -120, 100, 20, -50]
        q1_list3 = Utils.interpolate_joint_trajectory(q1_target3,q_obj1_deg_2, num = num) 
            
        q1_target4 = [-25, 13, -35, -100, -40, 0, -70,   -12, 10, 30, -100, 110, -10, -70]
        q1_list4 = Utils.interpolate_joint_trajectory(q1_target4, q1_target3, num = 30)

        q1_target5 = [-30, 20, -35, -100, -40, 0, -70,   -12, 10, 30, -80, 110, -10, -70]
        q1_list5 = Utils.interpolate_joint_trajectory(q1_target5,q1_target4, num = num) 
            
        q1_target6 = [-50, 20, -10, -50, -40, 0, -70,   60, 0, 10, -100, 110, 10, 0]
        q1_list6 = Utils.interpolate_joint_trajectory(q1_target6, q1_target5, num = num)
            
        q1_target7 = [-40, 20, 10, -30, -150, 0, -70,   70, 0, 0, -135, 70, 40, 0]
        q1_list7 = Utils.interpolate_joint_trajectory(q1_target7, q1_target6, num = num)

        q1_target8 = [10, 0, 0, -130, 70, 0, 0,   70, 0, 0, -135, 70, 40, 0]
        q1_list8 = Utils.interpolate_joint_trajectory(q1_target8, q1_target7, num = num)

        print("第一个物体翻面操作")
        traj_ctrl.execute_trajectory(q1_list3, sleep_time=0.02)
        time.sleep(0.5)
        traj_ctrl.execute_trajectory(q1_list4, sleep_time=0.02)
        time.sleep(1)
        
        print("左手关闭夹爪")
        gripper_ctrl.control_left_gripper(130)
        gripper_ctrl.control_right_gripper(130)
        time.sleep(0.5)
        
        print("右手张开夹爪")
        gripper_ctrl.control_left_gripper(130)
        gripper_ctrl.control_right_gripper(0)

        traj_ctrl.execute_trajectory(q1_list5, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q1_list6, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q1_list7, sleep_time=0.02)
        
        print("放置于目标区域")
        time.sleep(0.5)
        gripper_ctrl.control_left_gripper(0)
        gripper_ctrl.control_right_gripper(0)
        time.sleep(0.5)

        traj_ctrl.execute_trajectory(q1_list8, sleep_time=0.02)
        time.sleep(1)

        # 判定是否正确翻面以及是否在区域内
        all_success = True
        pos1 = obj_pos.get_position(obj1)
        ori1 = obj_pos.get_orientation(obj1)

        target_region = [
            (0.26, 0.60),
            (0.21, 0.58),
            (0.68, 1.00),
        ]

        in_region1 = Utils.is_in_target_region(pos1, target_region)
        is_front1  = Utils.is_front_facing(quat_xyzw=ori1,body_front_axis='x',tol_deg=20)

        if not (in_region1 and is_front1):
            all_success = False

        ### 抓取第二个反面物体
        ##  到达预抓位 - 使用高频轨迹控制器
        q2_target1 = [10, 0, 0, -130, 70, 0, 0,   70, 0, 0, -135, 70, 40, 0]
        q2_list1 = Utils.interpolate_joint_trajectory(q2_target1,q1_target8, num = num) 
            
        q2_target2 = [10, 0, 0, -130, 70, 0, 0,   10, 5, 0, -130, 90, 90, 0]
        q2_list2 = Utils.interpolate_joint_trajectory(q2_target2, q2_target1, num = num)

        print("移动到第二个物体预抓位")
        traj_ctrl.execute_trajectory(q2_list1, sleep_time=0.02)
        time.sleep(0.2)
        traj_ctrl.execute_trajectory(q2_list2, sleep_time=0.02)
        time.sleep(1)

        ## 抓取第二个物体
        curr_q2 = robot_state.arm_joint_state().position
        l_pose2, r_pose2 = robot.arm_fk(curr_q2)
        pos_obj2 = obj_pos.get_position(obj2)

        offset2_x = 0.1
        offset2_y = -0.0175
        offset2_z = 0.046
        r_pose2_new1 = [pos_obj2[0]+offset2_x, pos_obj2[1]+offset2_y, pos_obj2[2]+offset2_z+0.1]
        r_pose2_new2 = [pos_obj2[0]+offset2_x, pos_obj2[1]+offset2_y, pos_obj2[2]+offset2_z] 

        [], pose2_right1= Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=r_pose2_new1,
            quat_right=r_pose2.orientation,
        )

        pose2_left = [math.radians(x) for x in q2_target2[0:7]]
        q_obj2_1 = pose2_left+pose2_right1
        q_obj2_deg_1 = [math.degrees(x) for x in q_obj2_1]
        q_list_obj2_1 = Utils.interpolate_joint_trajectory(q_obj2_deg_1, q2_target2, num = 50)

        #再下去抓
        curr_q2_2 = robot_state.arm_joint_state().position
        l_pose2_2, r_pose2_2 = robot.arm_fk(curr_q2_2)
        [], pose2_right2= Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=r_pose2_new2,
            quat_right=r_pose2_2.orientation,
        )
        
        q_obj2_2 = pose2_left+pose2_right2
        q_obj2_deg_2 = [math.degrees(x) for x in q_obj2_2]
        q_list_obj2_2 = Utils.interpolate_joint_trajectory(q_obj2_deg_2, q_obj2_deg_1, num = 50)

        print("下降到第二个物体抓取位置")
        traj_ctrl.execute_trajectory(q_list_obj2_1, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q_list_obj2_2, sleep_time=0.02)

        time.sleep(0.5)
        # 右夹爪抓取 - 高频发布
        gripper_ctrl.control_right_gripper(130)
        time.sleep(0.5)

        #后续动作 - 使用高频轨迹控制器
        q2_target3 = [-10, 20, -40, -110, -40, 0, -50,   -15, 0, 30, -120, 100, 20, -50]
        q2_list3 = Utils.interpolate_joint_trajectory(q2_target3,q_obj2_deg_2, num = num) 
            
        q2_target4 = [-25, 13, -35, -100, -40, 0, -70,   -12, 10, 30, -100, 110, -10, -70]
        q2_list4 = Utils.interpolate_joint_trajectory(q2_target4, q2_target3, num = 30)

        q2_target5 = [-30, 20, -35, -100, -40, 0, -70,   -12, 10, 30, -80, 110, -10, -70]
        q2_list5 = Utils.interpolate_joint_trajectory(q2_target5,q2_target4, num = num) 
            
        q2_target6 = [-50, 20, -10, -50, -40, 0, -70,   60, 0, 10, -100, 110, 10, 0]
        q2_list6 = Utils.interpolate_joint_trajectory(q2_target6, q2_target5, num = num)
            
        q2_target7 = [-10, 5, 10, -90, -150, -50, -50,   0, 0, 0, 0, 0, 0, 0]
        q2_list7 = Utils.interpolate_joint_trajectory(q2_target7, q2_target6, num = num)

        print("第二个物体翻面操作")
        traj_ctrl.execute_trajectory(q2_list3, sleep_time=0.02)
        time.sleep(0.5)
        traj_ctrl.execute_trajectory(q2_list4, sleep_time=0.02)
        time.sleep(1)
        
        print("左手关闭夹爪")
        gripper_ctrl.control_left_gripper(130)
        gripper_ctrl.control_right_gripper(130)
        time.sleep(0.5)
        
        print("右手张开夹爪")
        gripper_ctrl.control_left_gripper(150)
        gripper_ctrl.control_right_gripper(0)

        traj_ctrl.execute_trajectory(q2_list5, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q2_list6, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q2_list7, sleep_time=0.02)
        
        print("放置于目标区域")
        time.sleep(0.5)
        gripper_ctrl.control_left_gripper(0)
        gripper_ctrl.control_right_gripper(0)
        time.sleep(1)

        # 判定是否正确翻面以及是否在区域内
        pos2 = obj_pos.get_position(obj2)
        ori2 = obj_pos.get_orientation(obj2)
        target_region = [
            (0.26, 0.60), 
            (0.21, 0.58), 
            (0.68, 1.00), 
        ]

        in_region2 = Utils.is_in_target_region(pos2, target_region)
        is_front2  = Utils.is_front_facing(quat_xyzw=ori2,body_front_axis='x',tol_deg=20)

        if not (in_region2 and is_front2):
            all_success = False
        
        # 检查任务完成情况
        if all_success:
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