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

    # wait for stance state
    if robot_state.wait_for_stance(timeout=100.0):
        print("Robot is in stance state")

    # 获取Apriltag数据
    time.sleep(0.1)
    print("Apriltag data from camera:")
    print(robot_vision.apriltag_data_from_camera)
    print("Apriltag data from base:")
    print(robot_vision.apriltag_data_from_base)
    print("Apriltag data from odom:")
    print(robot_vision.apriltag_data_from_odom)
    
    # 识别到的第一个tag的所有数据
    print(robot_vision.apriltag_data_from_odom.id[0])
    print(robot_vision.apriltag_data_from_odom.size[0])
    print(robot_vision.apriltag_data_from_odom.pose[0].position.x)
    print(robot_vision.apriltag_data_from_odom.pose[0].position.y)
    print(robot_vision.apriltag_data_from_odom.pose[0].position.z)
    print(robot_vision.apriltag_data_from_odom.pose[0].orientation.x)
    print(robot_vision.apriltag_data_from_odom.pose[0].orientation.y)
    print(robot_vision.apriltag_data_from_odom.pose[0].orientation.z)
    print(robot_vision.apriltag_data_from_odom.pose[0].orientation.w)

    # 识别到的第二个tag的所有数据
    print(robot_vision.apriltag_data_from_odom.id[1])
    print(robot_vision.apriltag_data_from_odom.size[1])
    print(robot_vision.apriltag_data_from_odom.pose[1].position.x)
    print(robot_vision.apriltag_data_from_odom.pose[1].position.y)
    print(robot_vision.apriltag_data_from_odom.pose[1].position.z)
    print(robot_vision.apriltag_data_from_odom.pose[1].orientation.x)
    print(robot_vision.apriltag_data_from_odom.pose[1].orientation.y)
    print(robot_vision.apriltag_data_from_odom.pose[1].orientation.z)
    print(robot_vision.apriltag_data_from_odom.pose[1].orientation.w)

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