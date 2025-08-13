from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
import time

def main():
    # Initialize SDK
    if not KuavoSDK().Init():
        print("Init KuavoSDK failed, exit!")
        exit(1)
        
    robot = KuavoRobot()

    # Reset arm position
    robot.arm_reset()

    # Move to stance position
    robot.stance()

    # Switch to trot gait
    robot.trot()

    # Walk forward for 4 seconds
    duration = 4.0  # seconds
    speed = 0.3     # m/s
    start_time = time.time()
    while (time.time() - start_time < duration):
        robot.walk(linear_x=speed, linear_y=0.0, angular_z=0.0)
        time.sleep(0.1)
    
    robot.stance()
if __name__ == "__main__":
    main()