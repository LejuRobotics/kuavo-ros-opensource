import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState

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
    robot.step_by_step(target_poses)
    if robot_state.wait_for_step_control(timeout=20.0):
        print("Robot is in step control")
    else:
        print("Timed out waiting for step control")

    # Wait up to 15s for stance state (adjust timeout based on actual needs)
    if robot_state.wait_for_stance(timeout=20.0):
        print("Robot is in stance state")
    else:
        print("Timed out waiting for stance state")
    
    target_poses = [0.2, 0.0, 0.0, 1.57]
    robot.step_by_step(target_poses)
    if robot_state.wait_for_step_control(timeout=20.0):
        print("Robot is in step control")
    else:
        print("Timed out waiting for step control")

if __name__ == "__main__":
    main()