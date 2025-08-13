from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot = KuavoRobot()
    # Control head to move back and forth slowly for multiple cycles
    import time
    
    cycles = 2  # Number of cycles to perform
    interval = 0.1  # Time interval between movements in seconds
    max_pitch = 25  # Maximum pitch angle in degrees
    
    for cycle in range(cycles):
        # Move from 0 to max_pitch degrees
        for pitch in range(0, max_pitch + 1, 2):
            robot.control_head(yaw=0, pitch=pitch * 0.017)  # Convert degrees to radians
            time.sleep(interval)
        
        # Move from max_pitch to -max_pitch degrees
        for pitch in range(max_pitch, -max_pitch - 1, -2):
            robot.control_head(yaw=0, pitch=pitch * 0.017)
            time.sleep(interval)
            
        # Move from -max_pitch back to 0 degrees
        for pitch in range(-max_pitch, 1, 2):
            robot.control_head(yaw=0, pitch=pitch * 0.017)
            time.sleep(interval)

    # Control head to move left and right
    cycles = 2  # Number of cycles to perform
    interval = 0.1  # Time interval between movements in seconds
    max_yaw = 60  # Maximum yaw angle in degrees
    
    for cycle in range(cycles):
        # Move from 0 to max_yaw degrees
        for yaw in range(0, max_yaw + 1, 2):
            robot.control_head(yaw=yaw * 0.0174533, pitch=0)  # Convert degrees to radians
            time.sleep(interval)
        
        # Move from max_yaw to -max_yaw degrees
        for yaw in range(max_yaw, -max_yaw - 1, -2):
            robot.control_head(yaw=yaw * 0.0174533, pitch=0)
            time.sleep(interval)
            
        # Move from -max_yaw back to 0 degrees
        for yaw in range(-max_yaw, 1, 2):
            robot.control_head(yaw=yaw * 0.0174533, pitch=0)
            time.sleep(interval)
if __name__ == "__main__":
    main()
