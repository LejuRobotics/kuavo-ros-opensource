import time
import signal
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState

# Global flag for handling Ctrl+C
running = True

def signal_handler(sig, frame):
    global running
    print('\nCtrl+C pressed. Stopping robot...')
    running = False

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    # Set up Ctrl+C handler
    signal.signal(signal.SIGINT, signal_handler)
    
    robot = KuavoRobot()    
    robot_state = KuavoRobotState()
    
    """ arm reset """
    print("Switching to arm reset mode...")
    robot.arm_reset()
    
    """ stance """
    print("Switching to stance mode...")
    robot.stance()

    """ trot """
    print("Switching to trot mode...")
    robot.trot()
    
    """ walk forward """
    print("Starting forward walk...")
    duration = 8.0  # seconds
    speed = 0.3     # m/s
    start_time = time.time()
    start_x = robot_state.odometry.position[0]
    while running and (time.time() - start_time < duration):
        robot.walk(linear_x=speed, linear_y=0.0, angular_z=0.0)
        print(f"Current x position: {robot_state.odometry.position[0]:.3f} m")
        time.sleep(0.1)  # Small sleep to prevent busy loop
    
    forward_distance = robot_state.odometry.position[0] - start_x
    print(f"\033[33mForward distance traveled: {forward_distance:.3f} m\033[0m")

    if running:
        """ stance """
        print("Switching back to stance mode...")
        robot.stance()

        """ walk back """
        print("Starting backward walk...")
        start_time = time.time()
        start_x = robot_state.odometry.position[0]
        while running and (time.time() - start_time < duration):
            robot.walk(linear_x=-speed, linear_y=0.0, angular_z=0.0)
            print(f"Current x position: {robot_state.odometry.position[0]:.3f} m")
            time.sleep(0.1)  # Small sleep to prevent busy loop
        
        backward_distance = abs(robot_state.odometry.position[0] - start_x)
        print(f"\033[33mBackward distance traveled: {backward_distance:.3f} m\033[0m")
        print(f"\033[33mTotal distance traveled: {forward_distance + backward_distance:.3f} m\033[0m")
        
        """ stance """
        print("Final switch to stance mode...")
        robot.stance()

        """ turn left in place """
        print("Starting left turn in place...")
        start_time = time.time()
        turn_speed = 0.4  # rad/s
        turn_duration = 8.0  # seconds
        while running and (time.time() - start_time < turn_duration):
            robot.walk(linear_x=0.0, linear_y=0.0, angular_z=turn_speed)  # Positive angular_z for left turn
            print(f"Current yaw angle: {robot_state.odometry.orientation[2]:.3f} rad")
            time.sleep(0.1)  # Small sleep to prevent busy loop
        
        """ turn right in place """
        print("Starting right turn in place...")
        start_time = time.time()
        while running and (time.time() - start_time < turn_duration):
            robot.walk(linear_x=0.0, linear_y=0.0, angular_z=-turn_speed)  # Negative angular_z for right turn
            print(f"Current yaw angle: {robot_state.odometry.orientation[2]:.3f} rad")
            time.sleep(0.1)  # Small sleep to prevent busy loop

        """ stance """
        print("Final switch to stance mode...")
        robot.stance()

    """ squat """
    if running:
        print("Starting squat motion...")
        robot.squat(-0.1)
        start_time = time.time()
        while running and (time.time() - start_time < 2.0):
            time.sleep(0.1)
        
        if running:
            print("Returning to original height...")
            robot.squat(0.0)  # Return to original height
            start_time = time.time()
            while running and (time.time() - start_time < 2.0):
                time.sleep(0.1)

if __name__ == "__main__":
    main()