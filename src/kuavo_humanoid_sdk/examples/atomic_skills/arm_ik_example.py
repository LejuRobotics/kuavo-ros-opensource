import time
import math
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState
from kuavo_humanoid_sdk import KuavoPose
 
if __name__ == "__main__":
    # !!! Initialize Kuavo SDK with IK module !!!
    if not KuavoSDK.Init(options=KuavoSDK.Options.WithIK): # Init! Important! if U use IK, you must use this option!
        print("Failed to initialize Kuavo SDK")
        exit(1)
        
    robot = KuavoRobot()
    robot.arm_reset()
    robot_state = KuavoRobotState()

    # Front pose - arms in front, palms facing down
    front_left = [0.45, 0.28, 0.25]   # Left arm front position
    front_right = [0.45, -0.20, 0.25]  # Right arm front position
    r_front_orientation = [-0.41158, -0.503073, 0.577546, 0.493919]
    l_front_orientation = [0.38, -0.45, -0.56, 0.57]
    
    arm_q = robot_state.arm_joint_state().position

    res = robot.arm_ik(
        KuavoPose(position=front_left, orientation=l_front_orientation),
        KuavoPose(position=front_right, orientation=r_front_orientation)
    )
    
    if res:
        times = [0.5, 2.5]
        q_frames = [arm_q, res]
        robot.control_arm_joint_trajectory(times, q_frames)
        time.sleep(3.5)
        robot.set_fixed_arm_mode()
        time.sleep(1.0)
        curr_q = robot_state.arm_joint_state().position
        # !!! arm fk !!!
        l_pose, r_pose = robot.arm_fk(curr_q)
        print(l_pose)
        print(r_pose)
        robot.arm_reset()    