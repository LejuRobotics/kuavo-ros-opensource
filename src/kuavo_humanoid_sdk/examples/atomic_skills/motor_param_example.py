from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
from kuavo_humanoid_sdk.interfaces.data_types import KuavoMotorParam
import time

def main():
    # Initialize SDK
    if not KuavoSDK().Init():
        print("Init KuavoSDK failed, exit!")
        exit(1)
        
    robot = KuavoRobot()

    success, motor_param = robot.get_motor_param()
    if not success:
        print("Failed to get motor param")

    print("motor_param", motor_param[0])
    # change motor param
    motor_param = []
    motor_param.append(KuavoMotorParam(Kp=221, Kd=2979, id=1))
    
    success, message = robot.change_motor_param(motor_param)
    if not success:
        print(f"Failed to change motor param: {message}")
        exit(1)
    
    success, motor_param = robot.get_motor_param()
    if not success:
        print("Failed to get motor param")

    print("motor_param", motor_param[0])

if __name__ == "__main__":
    main()