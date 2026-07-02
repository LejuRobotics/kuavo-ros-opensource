import time
import roslibpy
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
from kuavo_humanoid_sdk import KuavoRobotState
from kuavo_humanoid_sdk import KuavoRobotTools
from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK


def _is_wheel_arm_robot():
    """通过 ROS param 判断是否为轮臂机器人 (robot_type==1)"""
    try:
        client = WebSocketKuavoSDK().client
        return roslibpy.Param(client, 'robot_type').get() == 1
    except Exception:
        return False


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
    robot_tools = KuavoRobotTools()

    # 双足需要切 stance，轮臂跳过
    if not _is_wheel_arm_robot():
        robot.stance()
        if robot_state.wait_for_stance(timeout=100.0):
            print("Robot is in stance state")
    else:
        print("轮臂平台，跳过 stance 切换")

    # 获取tf树变换
    time.sleep(0.1)
    print("odom to base_link pose_quaternion:")
    print(robot_tools.get_tf_transform("odom", "base_link", return_type="pose_quaternion"))
    print("odom to base_link homogeneous:")
    print(robot_tools.get_tf_transform("odom", "base_link", return_type="homogeneous"))
    print("base_link to odom pose_quaternion:")
    print(robot_tools.get_base_to_odom(return_type="pose_quaternion"))
    print("base_link to odom homogeneous:")
    print(robot_tools.get_base_to_odom(return_type="homogeneous"))
    print("camera_link to base_link pose_quaternion:")
    print(robot_tools.get_camera_to_base(return_type="pose_quaternion"))
    print("camera_link to base_link homogeneous:")
    print(robot_tools.get_camera_to_base(return_type="homogeneous"))

if __name__ == "__main__":
    main()