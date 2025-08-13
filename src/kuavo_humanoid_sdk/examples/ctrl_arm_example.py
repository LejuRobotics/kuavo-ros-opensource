import time
import math
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot,KuavoRobotState

if not KuavoSDK().Init(log_level='DEBUG'):# Init!
    print("Init KuavoSDK failed, exit!")
    exit(1)

robot = KuavoRobot()

def control_arm_traj():
    global robot
     # reset arm
    robot.arm_reset()
    
    q_list = []
    q0 = [0.0]*14
    # open arm
    q1 = [0, 50, 0, 0, 0, 0, 0, 0.0, -50, 0, 0, 0, 0, 0]

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
        robot.control_arm_position(q)
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
        robot.control_arm_position(q_tmp)
        time.sleep(0.02)

    robot.set_auto_swing_arm_mode() # Restore auto arm swing mode

    robot.arm_reset() # Reset arm position


def control_arm_target_poses():
    global robt
    target_poses = [
        [1.0, [20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],
        [2.5, [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]],
        [4.0, [20, 0, 0, -30, 0, 0, 0, -30, 0, 30, -88, 8, -22, -4]],
        [5.5, [20, 0, 0, -30, 0, 0, 0, -30, -25, -54, -15, -6, -22, -4]],
        [7.0, [10, 10, -20, -70, 0, 0, -24, -30, -25, -54, -15, -6, -22, -4]],
        [8.5, [14, 20, 33, -35, 76, -18, 3.5, -30, -25, -54, -15, -6, -22, -4]],
        [10, [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]]
    ]

    times = [pose[0] for pose in target_poses]
    
    # !!! Convert degrees to radians
    # !!! Convert degrees to radians
    q_frames = [[math.radians(angle) for angle in pose[1]] for pose in target_poses]
    
    if not robot.control_arm_target_poses(times, q_frames):
        print("control_arm_target_poses failed!")

if __name__ == "__main__":

    robot.stance()
    robot_state = KuavoRobotState()
    if not robot_state.wait_for_stance():
        print("change to stance fail!")

    print("Robot stance !!!!!")
    # !!! Move arm trajectory
    control_arm_traj()
    
    control_arm_target_poses()
    time.sleep(12)  # !!! Wait for the arm to reach the target pose
    
    robot.arm_reset() # !!! after the arm reaches the target pose, reset the arm position.