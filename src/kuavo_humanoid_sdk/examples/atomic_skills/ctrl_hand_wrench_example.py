import time
import math
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot,KuavoRobotState
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, KuavoManipulationMpcFrame

if not KuavoSDK().Init(log_level='INFO'):# Init!
    print("Init KuavoSDK failed, exit!")
    exit(1)

robot = KuavoRobot()

if __name__ == "__main__":

    robot.stance()
    robot_state = KuavoRobotState()
    if not robot_state.wait_for_stance():
        print("change to stance fail!")
    
    print("robot now is zero wrench")
    left_wrench = [0,0,0,0,0,0]
    right_wrench = [0,0,0,0,0,0]
    robot.control_hand_wrench(left_wrench,right_wrench)
    time.sleep(5)
    
    print("robot now is 10N wrench")
    left_wrench = [0,10,-10,0,0,0]
    right_wrench = [0,-10,-10,0,0,0]
    robot.control_hand_wrench(left_wrench,right_wrench)
    time.sleep(5)
    
    print("robot now is zero wrench")
    left_wrench = [0,0,0,0,0,0]
    right_wrench = [0,0,0,0,0,0]
    robot.control_hand_wrench(left_wrench,right_wrench)
    time.sleep(5)

    print("Robot stance !!!!!")
