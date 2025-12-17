import time
import math
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot,KuavoRobotState
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, KuavoManipulationMpcFrame

if not KuavoSDK().Init(log_level='INFO'):# Init!
    print("Init KuavoSDK failed, exit!")
    exit(1)

robot = KuavoRobot()

def control_arm_traj():
    global robot
     # reset arm
    robot.arm_reset()
    
    q_list = []
    q0 = [0.0]*14
    # arm pose that will collision
    q1 = [-60, 20, -90, -90, 0, 0, 0, -60, -20, 90, -90, 0, 0, 0]

    num = 90
    for i in range(num):
        q_tmp = [0.0]*14
        for j in range(14):
            q_tmp[j] = q0[j] + i/float(num)*(q1[j] - q0[j])
        q_list.append(q_tmp)
    for q in q_list:
        # !!! Convert degrees to radians
        # !!! Convert degrees to radians
        q = [math.radians(angle) for angle in q]
        try:
            robot.control_arm_joint_positions(q)
        except:
            if robot.is_arm_collision():
                robot.wait_arm_collision_complete()
                robot.release_arm_collision_mode()
        
        time.sleep(0.02)
    
    # fixed arm
    robot.set_fixed_arm_mode()
    time.sleep(1.0)
    # back to q0
    for i in range(num):
        q_tmp = [0.0]*14
        for j in range(14):
            q_tmp[j] = q1[j] - i/float(num)*(q1[j] - q0[j])
                # !!! Convert degrees to radians
                # !!! Convert degrees to radians
        q_tmp = [math.radians(angle) for angle in q_tmp]
        try:
            robot.control_arm_joint_positions(q_tmp)
        except:
            if robot.is_arm_collision():
                robot.wait_arm_collision_complete()
                robot.release_arm_collision_mode()
        
        time.sleep(0.02)

    robot.set_auto_swing_arm_mode() # Restore auto arm swing mode

    robot.arm_reset() # Reset arm position


def control_arm_joint_trajectory():
    global robot
    target_poses = [
        [1.0, [20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],
        [2.5, [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]],
        [4.0, [20, 0, 0, -30, 0, 0, 0, -30, 0, 30, -88, 8, -22, -4]],
        [5.5, [20, -90, 0, -30, 0, 0, 0, -30, -25, -54, -15, -6, -22, -4]],
        [7.0, [10, 10, -20, -70, 0, 0, -24, -30, -25, -54, -15, -6, -22, -4]],
        [8.5, [14, 20, 33, -35, 76, -18, 3.5, -30, -25, -54, -15, -6, -22, -4]],
        [10, [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]]
    ]

    times = [pose[0] for pose in target_poses]
    
    # !!! Convert degrees to radians
    # !!! Convert degrees to radians
    q_frames = [[math.radians(angle) for angle in pose[1]] for pose in target_poses]
    
    try:
        if not robot.control_arm_joint_trajectory(times, q_frames):
            print("control_arm_joint_trajectory failed!")
    except:
        if robot.is_arm_collision():
            robot.wait_arm_collision_complete()
            robot.release_arm_collision_mode()

if __name__ == "__main__":

    robot.stance()
    robot_state = KuavoRobotState()

    robot.set_arm_collision_mode(True)
    if not robot_state.wait_for_stance():
        print("change to stance fail!")

    print("Robot stance !!!!!")
    # !!! Move arm trajectory
    control_arm_traj()
    
    control_arm_joint_trajectory()
    time.sleep(12)  # !!! Wait for the arm to reach the target pose
    
    robot.set_arm_collision_mode(False)
    exit()