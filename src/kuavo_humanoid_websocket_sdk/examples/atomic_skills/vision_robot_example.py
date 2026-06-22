import time
import roslibpy
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
from kuavo_humanoid_sdk import KuavoRobotState
from kuavo_humanoid_sdk import KuavoRobotVision
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
    robot_vision = KuavoRobotVision()

    # 双足需要切 stance，轮臂跳过
    if not _is_wheel_arm_robot():
        robot.stance()
        if robot_state.wait_for_stance(timeout=100.0):
            print("Robot is in stance state")
    else:
        print("轮臂平台，跳过 stance 切换")

    # 等待获取Apriltag数据
    time.sleep(5)
    
    print("Apriltag data from camera:")
    print(robot_vision.apriltag_data_from_camera)
    print("Apriltag data from base:")
    print(robot_vision.apriltag_data_from_base)
    print("Apriltag data from odom:")
    print(robot_vision.apriltag_data_from_odom)
    
    # 识别到的tag数据 - 先检查是否有数据
    odom_data = robot_vision.apriltag_data_from_odom
    if len(odom_data.id) > 0:
        print("first tag data:")
        print(odom_data.id[0])
        print(odom_data.size[0])
        print(odom_data.pose[0].position.x)
        print(odom_data.pose[0].position.y)
        print(odom_data.pose[0].position.z)
        print(odom_data.pose[0].orientation.x)
        print(odom_data.pose[0].orientation.y)
        print(odom_data.pose[0].orientation.z)
        print(odom_data.pose[0].orientation.w)
    else:
        print("No apriltag detected in odom frame")

    if len(odom_data.id) > 1:
        print("second tag data:")
        print(odom_data.id[1])
        print(odom_data.size[1])
        print(odom_data.pose[1].position.x)
        print(odom_data.pose[1].position.y)
        print(odom_data.pose[1].position.z)
        print(odom_data.pose[1].orientation.x)
        print(odom_data.pose[1].orientation.y)
        print(odom_data.pose[1].orientation.z)
        print(odom_data.pose[1].orientation.w)

    # 获取指定tag的数据
    print("tag 0 data:")
    print(robot_vision.get_data_by_id(0, "odom"))
    print("tag 2 data:")
    print(robot_vision.get_data_by_id(2, "odom"))
    print("tag 3 data:")
    print(robot_vision.get_data_by_id(3, "odom"))

    while True:
        time.sleep(0.1)
if __name__ == "__main__":
    main()