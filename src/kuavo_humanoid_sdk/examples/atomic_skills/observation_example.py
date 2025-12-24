from kuavo_humanoid_sdk import KuavoSDK, KuavoRobotObservation

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot_obs = KuavoRobotObservation()
    import time
    try:
        print("Printing arm position commands. Press CTRL+C to exit.")
        while True:
            # Get and print the current arm position commands
            arm_positions = robot_obs.arm_position_command
            print("Arm position commands:", arm_positions[0])
            
            # Sleep to avoid flooding the console
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Exiting...")
if __name__ == "__main__":
    main()
