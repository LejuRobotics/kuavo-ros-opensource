#!/usr/bin/env python3
import time 
import math
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState, KuavoRobotTools, KuavoRobotVision
from kuavo_humanoid_sdk.interfaces.data_types import AprilTagData, PoseQuaternion
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, KuavoManipulationMpcFrame
# 先不使用策略模块,直接通过SDK实现
from kuavo_humanoid_sdk.common.logger import SDKLogger

############################## 手臂控制 #######################################

def control_arm_traj(robot , q0 , q1):
    # reset arm
    robot.arm_reset()
    
    q_list = []
    
    num = 50
    for i in range(num):
        q_tmp = [0.0]*14
        for j in range(14):
            q_tmp[j] = q0[j] + i/float(num)*(q1[j] - q0[j])
        q_list.append(q_tmp)
    for q in q_list:
        # !!! Convert degrees to radians
        robot.control_arm_joint_positions(q)
        time.sleep(0.1)
    time.sleep(5)
    #fixed arm
    #robot.set_fixed_arm_mode()
    #time.sleep(1.0)

############################## 主函数 #######################################

def main():
    # 初始化SDK
    if not KuavoSDK.Init(options=KuavoSDK.Options.Normal, log_level="DEBUG"):
        print("Failed to initialize Kuavo SDK")
        return

    print("初始化机器人...")
   
    # 初始化机器人及相关组件
    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    robot_vision = KuavoRobotVision()

    # robot_position = robot_state.robot_position()
    # robot_orientation = robot_state.robot_orientation()
    # robot_tools = KuavoRobotTools()

    # Stance
    robot.stance()
    # wait for stance state
    if robot_state.wait_for_stance(timeout=100.0):
        print("Robot is in stance state")

############################## Apriltag二维码位置获取 #######################################

    # 获取指定 tag=0 的数据
    time.sleep(0.1)
    print("tag 0 data:")
    data = robot_vision.get_data_by_id(0, "base")
    print(data)
    if data and 'poses' in data and len(data['poses']) > 0:
        detection = data['poses'][0]
        # 提取 position (x, y, z)
        position = [detection.position.x, detection.position.y, detection.position.z]
        # 提取 orientation (x, y, z, w)
        orientation = [
            detection.orientation.x,
            detection.orientation.y,
            detection.orientation.z,
            detection.orientation.w
        ]
        print("Position (x, y, z):", position)
        print("Orientation (x, y, z, w):", orientation)
    else:
        print("No valid data for tag 0.")
############################## ik参数赋值 #######################################

    # Front pose - arms in front, palms facing down
    left_front_position = [0.45,0.25,0.11988012 ]#+0.813]   # Left arm front position
    right_front_position = [0.45,-0.25,0.11988012 ]#+0.813]  # Right arm front position
    left_front_orientation = [0.0,-0.70682518,0.0,0.70738827]
    right_front_orientation = [0.0,-0.70682518,0.0,0.70738827]
    
# ############################## ik求解与执行 #######################################

    res = robot.arm_ik(
        KuavoPose(position=left_front_position, orientation=left_front_orientation),
        KuavoPose(position=right_front_position, orientation=right_front_orientation)
    )
    
    if res:
        # 执行ik结果
        curr_q = robot_state.arm_joint_state().position
        print("curr_q:",curr_q)
        print("res:",res)
        # control_arm_traj(robot , curr_q , list(res))
        
        # !!! arm fk !!!
        curr_q = robot_state.arm_joint_state().position
        l_pose, r_pose = robot.arm_fk(curr_q)
        print(l_pose)
        print(r_pose)

    time.sleep(8)
# ############################## 手臂复位 #######################################
    robot.manipulation_mpc_reset()
    robot.arm_reset()
    time.sleep(1)

if __name__ == "__main__":
    main() 

