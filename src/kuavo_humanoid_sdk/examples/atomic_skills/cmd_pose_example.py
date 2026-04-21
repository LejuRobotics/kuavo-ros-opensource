import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
from kuavo_humanoid_sdk import KuavoRobotState
from kuavo_humanoid_sdk import KuavoRobotVision
def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot = KuavoRobot() 
    robot_state = KuavoRobotState()
    robot_vision = KuavoRobotVision()
    
    # Stance
    robot.stance()
    # Command Pose - Odom
    robot.control_command_pose_world(0.0, 1.0, 0.0, 1.57)
    
    # 等待3s 
    time.sleep(10.0)

    # Stance
    robot.stance()
    # Command Pose - BaseLink
    robot.control_command_pose(-2.0, 0.0, 0.0, -1.57)
    
if __name__ == "__main__":
    main()