<a id="quickstart"></a>

# 快速开始

#### WARNING
在运行任何代码示例之前，请确保已经启动机器人， 否则 SDK 无法正常工作：

- 如果是命令行启动，则请确保类似下面的命令已经执行:
  : - 仿真模式: `roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch` (示例命令)
    - 真实机器人: `roslaunch humanoid_controllers load_kuavo_real.launch` (示例命令)
- 如果是 h12 遥控器等启动方式，也请确保已经让机器人启动(站立)

## 入门指南

在使用 SDK 之前，您必须首先通过调用 `KuavoSDK().Init()` 来初始化它。这是使用任何其他 SDK 功能之前的必要步骤。

以下是一个最小示例：

```python3
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot

# 初始化 SDK - 这是必需的！
if not KuavoSDK().Init():
    print("Init KuavoSDK failed, exit!")
    exit(1)

# 成功初始化后创建机器人实例
robot = KuavoRobot()

# Now you can use the robot object...
```

#### WARNING
在没有首先调用 `KuavoSDK().Init()` 或当它返回 `False` 时调用 SDK 函数是非法的，可能导致未定义的行为。后果是不可预测的，可能很危险。

### 基础示例

让我们分解这个示例：

1. 首先，我们导入所需的模块：
   - `KuavoSDK` - 主要的 SDK 接口
   - `KuavoRobot` - 机器人控制接口
   - `time` - 用于时间控制
2. 在 `main()` 函数中：
   - 我们使用 `KuavoSDK().Init()` 初始化 SDK - 这是关键且必须的第一步
   - 使用 `KuavoRobot()` 创建机器人实例
3. 基本机器人控制方法：
   - `arm_reset()` - 将手臂归位
   - `stance()` - 切换到站立状态模式
   - `trot()` - 切换到小跑步态模式
4. 行走控制：
   - 我们让机器人以 0.3 m/s 的速度行走 4 秒

这个示例演示了初始化 SDK 和控制机器人运动的基本工作流程：

```python
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
import time

def main():
    # Initialize SDK
    if not KuavoSDK().Init():
        print("Init KuavoSDK failed, exit!")
        exit(1)
        
    robot = KuavoRobot()

    # Reset arm position
    robot.arm_reset()

    # Move to stance position
    robot.stance()

    # Switch to trot gait
    robot.trot()

    # Walk forward for 4 seconds
    duration = 4.0  # seconds
    speed = 0.3     # m/s
    start_time = time.time()
    while (time.time() - start_time < duration):
        robot.walk(linear_x=speed, linear_y=0.0, angular_z=0.0)
        time.sleep(0.1)
    
    robot.stance()
if __name__ == "__main__":
    main()
```

以下是一个完整的示例，展示了初始化后获取机器人 apriltag 视觉数据：

```python
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
```
