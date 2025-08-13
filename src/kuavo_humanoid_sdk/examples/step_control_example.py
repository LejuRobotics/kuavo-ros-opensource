import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot = KuavoRobot() 
    robot_state = KuavoRobotState()

    # Stance
    robot.stance()

    # wait for stance state
    if robot_state.wait_for_stance(timeout=100.0):
        print("Robot is in stance state")
    
    # !!! Warning !!!: step_by_step control can only be used in stance mode
    # 
    # Step by step forward 0.8m
    target_poses = [0.8, 0.0, 0.0, 0.0]
    try:
        if not robot.step_by_step(target_poses):
            print("\033[33mStep by step failed\033[0m")
        else:
            print("\033[32mStep by step succeeded\033[0m")
    except Exception as e:
        print(f"Step by step error: {e}")
    
    # Wait up to 15s for stance state (adjust timeout based on actual needs)
    if robot_state.wait_for_stance(timeout=20.0):
        print("Robot is in stance state")
    else:
        print("Timed out waiting for stance state")
    
    target_poses = [0.2, 0.0, 0.0, 1.57]
    try:
        if not robot.step_by_step(target_poses):
            print("\033[33mStep by step failed\033[0m")
        else:
            print("\033[32mStep by step succeeded\033[0m")
    except Exception as e:
        print(f"Step by step error: {e}")

if __name__ == "__main__":
    main()