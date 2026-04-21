import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
from kuavo_humanoid_sdk import KuavoRobotState
from kuavo_humanoid_sdk import KuavoRobotTools

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot = KuavoRobot() 
    robot_state = KuavoRobotState()
    robot_tools = KuavoRobotTools()

    # Stance
    robot.stance()

    # wait for stance state
    if robot_state.wait_for_stance(timeout=100.0):
        print("Robot is in stance state")

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