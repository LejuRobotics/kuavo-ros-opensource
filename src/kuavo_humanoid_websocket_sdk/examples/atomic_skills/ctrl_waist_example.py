from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot

def main():
    import argparse 

    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='127.0.0.1', help='Websocket host address')
    parser.add_argument('--port', type=int, default=9090, help='Websocket port')
    args = parser.parse_args()

    if not KuavoSDK().Init(log_level='INFO', websocket_mode=True, websocket_host=args.host, websocket_port=args.port):# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot = KuavoRobot()
    # Control head to move back and forth slowly for multiple cycles
    import time

    # Control head to move left and right
    cycles = 1  # Number of cycles to perform
    interval = 0.4  # Time interval between movements in seconds
    max_yaw = 180  # Maximum yaw angle in degrees
    
    for cycle in range(cycles):
        # Move from 0 to max_yaw degrees
        for yaw in range(0, max_yaw + 1, 2):
            robot.control_waist(yaw=yaw)  # Convert degrees to radians
            time.sleep(interval)
        
        # Move from max_yaw to -max_yaw degrees
        for yaw in range(max_yaw, -max_yaw - 1, -2):
            robot.control_waist(yaw=yaw)
            time.sleep(interval)
            
        # Move from -max_yaw back to 0 degrees
        for yaw in range(-max_yaw, 1, 2):
            robot.control_waist(yaw=yaw)
            time.sleep(interval)
if __name__ == "__main__":
    main()
