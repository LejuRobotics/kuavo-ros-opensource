<a id="examples"></a>

# 使用示例

#### WARNING
在运行任何代码示例之前，请确保已经启动机器人， 否则 SDK 无法正常工作：

- 如果是命令行启动，则请确保类似下面的命令已经执行:
  : - 仿真模式: `roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch` (示例命令)
    - 真实机器人: `roslaunch humanoid_controllers load_kuavo_real.launch` (示例命令)
- 如果是 h12 遥控器等启动方式，也请确保已经让机器人启动(站立)

## 获取机器人信息

这个示例展示了如何获取机器人的基本信息。

```python
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobotInfo

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot_info = KuavoRobotInfo()
    print("Robot Type:", robot_info.robot_type)
    print("Robot Version:", robot_info.robot_version)
    print("End Effector Type:", robot_info.end_effector_type)
    print("Joint Names:", robot_info.joint_names)
    print("Total Joint DOF:", robot_info.joint_dof)
    print("Arm Joint DOF:", robot_info.arm_joint_dof)
    print("Arm Joint Names:", robot_info.arm_joint_names)
    print("Head Joint DOF:", robot_info.head_joint_dof)
    print("Head Joint Names:", robot_info.head_joint_names)
    print("End Effector Frame Names:", robot_info.eef_frame_names)
if __name__ == "__main__":
    main()
```

## 机器人运动控制

这个示例展示了如何初始化 SDK 并控制机器人的运动，包括站立，行走，转向，下蹲等。

```python
import time
import signal
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState

# Global flag for handling Ctrl+C
running = True

def signal_handler(sig, frame):
    global running
    print('\nCtrl+C pressed. Stopping robot...')
    running = False

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    # Set up Ctrl+C handler
    signal.signal(signal.SIGINT, signal_handler)
    
    robot = KuavoRobot()    
    robot_state = KuavoRobotState()
    
    """ arm reset """
    print("Switching to arm reset mode...")
    robot.arm_reset()
    
    """ stance """
    print("Switching to stance mode...")
    robot.stance()

    """ trot """
    print("Switching to trot mode...")
    robot.trot()
    
    """ walk forward """
    print("Starting forward walk...")
    duration = 8.0  # seconds
    speed = 0.3     # m/s
    start_time = time.time()
    start_x = robot_state.odometry.position[0]
    while running and (time.time() - start_time < duration):
        robot.walk(linear_x=speed, linear_y=0.0, angular_z=0.0)
        print(f"Current x position: {robot_state.odometry.position[0]:.3f} m")
        time.sleep(0.1)  # Small sleep to prevent busy loop
    
    forward_distance = robot_state.odometry.position[0] - start_x
    print(f"\033[33mForward distance traveled: {forward_distance:.3f} m\033[0m")

    if running:
        """ stance """
        print("Switching back to stance mode...")
        robot.stance()

        """ walk back """
        print("Starting backward walk...")
        start_time = time.time()
        start_x = robot_state.odometry.position[0]
        while running and (time.time() - start_time < duration):
            robot.walk(linear_x=-speed, linear_y=0.0, angular_z=0.0)
            print(f"Current x position: {robot_state.odometry.position[0]:.3f} m")
            time.sleep(0.1)  # Small sleep to prevent busy loop
        
        backward_distance = abs(robot_state.odometry.position[0] - start_x)
        print(f"\033[33mBackward distance traveled: {backward_distance:.3f} m\033[0m")
        print(f"\033[33mTotal distance traveled: {forward_distance + backward_distance:.3f} m\033[0m")
        
        """ stance """
        print("Final switch to stance mode...")
        robot.stance()

        """ turn left in place """
        print("Starting left turn in place...")
        start_time = time.time()
        turn_speed = 0.4  # rad/s
        turn_duration = 8.0  # seconds
        while running and (time.time() - start_time < turn_duration):
            robot.walk(linear_x=0.0, linear_y=0.0, angular_z=turn_speed)  # Positive angular_z for left turn
            print(f"Current yaw angle: {robot_state.odometry.orientation[2]:.3f} rad")
            time.sleep(0.1)  # Small sleep to prevent busy loop
        
        """ turn right in place """
        print("Starting right turn in place...")
        start_time = time.time()
        while running and (time.time() - start_time < turn_duration):
            robot.walk(linear_x=0.0, linear_y=0.0, angular_z=-turn_speed)  # Negative angular_z for right turn
            print(f"Current yaw angle: {robot_state.odometry.orientation[2]:.3f} rad")
            time.sleep(0.1)  # Small sleep to prevent busy loop

        """ stance """
        print("Final switch to stance mode...")
        robot.stance()

    """ squat """
    if running:
        print("Starting squat motion...")
        robot.squat(-0.1)
        start_time = time.time()
        while running and (time.time() - start_time < 2.0):
            time.sleep(0.1)
        
        if running:
            print("Returning to original height...")
            robot.squat(0.0)  # Return to original height
            start_time = time.time()
            while running and (time.time() - start_time < 2.0):
                time.sleep(0.1)

if __name__ == "__main__":
    main()
```

## 末端执行器控制

### LejuClaw 夹爪

这个示例展示了如何控制夹爪，支持整体开合、单侧控制、目标位置设定等多种操作，通过 claw.open()、claw.close()、claw.control_left()、claw.control_right()、claw.control() 等接口实现。

示例代码展示了以下主要功能：

1. 基本开合控制：
   * `claw.close()` 完全闭合夹爪
   * `claw.open()` 完全打开夹爪
   * 每个动作后使用 `wait_for_finish()` 等待动作完成
2. 单侧夹爪控制：
   * `claw.control_left([50])` 控制左侧夹爪到50度位置
   * `claw.control_right([80])` 控制右侧夹爪到80度位置
   * 可以分别精确控制左右两侧夹爪的位置
3. 双侧同步控制：
   * `claw.control([20, 100])` 同时控制左右夹爪到不同位置
   * 第一个参数控制左侧，第二个参数控制右侧
   * 位置范围为0-100度，0表示完全闭合，100表示完全打开

注意事项：

* 每次动作后建议调用 `wait_for_finish()` 等待完成，避免动作叠加
* 可以设置超时时间，如 `wait_for_finish(timeout=2.0)`
* 连续发送指令时要注意等待前一个动作完成，否则可能被丢弃

```python
from kuavo_humanoid_sdk import KuavoSDK, LejuClaw
from kuavo_humanoid_sdk.interfaces.data_types import EndEffectorState

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    claw = LejuClaw()
    
    # close claw
    if claw.close():
        if claw.wait_for_finish(timeout=2.0): # Wait for the claw motion to finish with a 2 second timeout
            print("Claw motion finished successfully")
            
    # open
    if claw.open():
        if claw.wait_for_finish(): # Wait for the claw motion to finish
            print("Open claw motion finished successfully")

    # control_left
    if claw.control_left(target_positions=[50]):
        if claw.wait_for_finish(): # Wait for the claw motion to finish
            print("Left claw motion finished successfully")

    # control_right
    if claw.control_right(target_positions=[80]):
        if claw.wait_for_finish(): # Wait for the claw motion to finish
            print("Right claw motion finished successfully")

    # control_right
    if claw.control(target_positions=[20, 100]):
        if claw.wait_for_finish(): # Wait for the claw motion to finish
            print("Both claw motion finished successfully")

    claw.open()   
    # !!! WARNING: !!!
    # This request may be dropped because the claw may still be executing the previous motion.
    claw.control(target_positions=[10, 10]) 
if __name__ == "__main__":
    main()
```

### Qiangnao 灵巧手

这个示例展示了如何控制灵巧手/触觉灵巧手，支持单侧控制、双侧同步控制、预设手势调用等功能。示例代码展示了：
#. 单侧手指控制：

> * 左手指向”点赞”手势
> * 右手指向”666”手势
1. 预设手势调用：
   * 获取所有支持的手势名称
   * 调用预设手势（如”点赞”和”OK”手势）

```python
from kuavo_humanoid_sdk import KuavoSDK, DexterousHand
from kuavo_humanoid_sdk.interfaces.data_types import EndEffectorState

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    import time
    dex_hand = DexterousHand() # init dex hand

    dex_hand.control_left([5, 5, 95, 95, 95, 95]) # 6 dof. 'thumbs-up' gesture.
    dex_hand.control_right([5, 5, 95, 95, 95, 5]) # 6 dof. '666' gesture.
    time.sleep(1.0)

    # open/reset/release
    dex_hand.open() # both.
    time.sleep(1.0)

    # get gesture names
    gestures = dex_hand.get_gesture_names()
    if gestures is None:
        print("Get gesture names failed!")
        exit(1)
    else:
        for gesture in gestures:
            print("Supported gesture: ", gesture)
    
    # make gesture
    dex_hand.make_gesture(l_gesture_name="thumbs-up", r_gesture_name="ok")
if __name__ == "__main__":
    main()
```

## 手臂运动控制

这个示例展示了如何控制机器人手臂运动，包括轨迹控制和目标姿态控制。

示例代码包含三个主要的控制函数：

1. control_arm_traj()：关节角度插值运动
   * 从初始位置q0开始，通过90步插值运动到目标位置q1
   * q1设置了右臂抬起50度的姿态
   * 每步之间间隔0.02秒，实现平滑过渡
   * 运动完成后恢复到初始位置
   * 最后重置手臂位置
2. control_arm_joint_trajectory()：关节轨迹控制
   * 定义了7个关键时间点的目标姿态
   * 机器人会自动完成关键帧之间的轨迹规划
   * 执行完毕后重置手臂位置
3. control_arm_end_effector_pose()：末端执行器位姿控制
   * 直接指定左右手末端在世界坐标系中的目标位置和姿态
   * 通过逆运动学自动计算所需的关节角度
   * 机器人自动规划轨迹到达目标位姿
   * 完成后重置手臂位置

```python
import time
import math
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot,KuavoRobotState
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, KuavoManipulationMpcFrame

if not KuavoSDK().Init(log_level='INFO'):# Init!
    print("Init KuavoSDK failed, exit!")
    exit(1)

robot = KuavoRobot()

def control_arm_traj():
    global robot
     # reset arm
    robot.arm_reset()
    
    q_list = []
    q0 = [0.0]*14
    # open arm
    q1 = [0, 50, 0, 0, 0, 0, 0, 0.0, -50, 0, 0, 0, 0, 0]

    num = 90
    for i in range(num):
        q_tmp = [0.0]*14
        for j in range(14):
            q_tmp[j] = q0[j] + i/float(num)*(q1[j] - q0[j])
        q_list.append(q_tmp)
    for q in q_list:
        # !!! Convert degrees to radians
        # !!! Convert degrees to radians
        q = [math.radians(angle) for angle in q]
        robot.control_arm_joint_positions(q)
        time.sleep(0.02)
    
    # fixed arm
    robot.set_fixed_arm_mode()
    time.sleep(1.0)

    # back to q0
    for i in range(num):
        q_tmp = [0.0]*14
        for j in range(14):
            q_tmp[j] = q1[j] - i/float(num)*(q1[j] - q0[j])
                # !!! Convert degrees to radians
                # !!! Convert degrees to radians
        q_tmp = [math.radians(angle) for angle in q_tmp]
        robot.control_arm_joint_positions(q_tmp)
        time.sleep(0.02)

    robot.set_auto_swing_arm_mode() # Restore auto arm swing mode

    robot.arm_reset() # Reset arm position


def control_arm_joint_trajectory():
    global robot
    target_poses = [
        [1.0, [20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],
        [2.5, [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]],
        [4.0, [20, 0, 0, -30, 0, 0, 0, -30, 0, 30, -88, 8, -22, -4]],
        [5.5, [20, 0, 0, -30, 0, 0, 0, -30, -25, -54, -15, -6, -22, -4]],
        [7.0, [10, 10, -20, -70, 0, 0, -24, -30, -25, -54, -15, -6, -22, -4]],
        [8.5, [14, 20, 33, -35, 76, -18, 3.5, -30, -25, -54, -15, -6, -22, -4]],
        [10, [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]]
    ]

    times = [pose[0] for pose in target_poses]
    
    # !!! Convert degrees to radians
    # !!! Convert degrees to radians
    q_frames = [[math.radians(angle) for angle in pose[1]] for pose in target_poses]
    
    if not robot.control_arm_joint_trajectory(times, q_frames):
        print("control_arm_joint_trajectory failed!")

def control_arm_end_effector_pose():
    global robot

    robot.control_robot_end_effector_pose(KuavoPose(position=[0.3, 0.4, 0.9], orientation=[0, -0.67566370964, 0, 0.73720997571945]), KuavoPose(position=[0.3, -0.5, 1.0], orientation=[0, -0.67566370964, 0, 0.73720997571945]), KuavoManipulationMpcFrame.LocalFrame)

if __name__ == "__main__":

    robot.stance()
    robot_state = KuavoRobotState()
    if not robot_state.wait_for_stance():
        print("change to stance fail!")

    print("Robot stance !!!!!")
    # !!! Move arm trajectory
    control_arm_traj()
    
    control_arm_joint_trajectory()
    time.sleep(12)  # !!! Wait for the arm to reach the target pose
    
    robot.arm_reset() # !!! after the arm reaches the target pose, reset the arm position.
    control_arm_end_effector_pose()
    time.sleep(3)
    robot.manipulation_mpc_reset()
    robot.arm_reset()
```

## 正向和逆向运动学控制

这个示例展示了如何使用正向运动学 (FK) 从关节角度计算末端执行器位置，以及如何使用逆向运动学 (IK) 计算实现所需末端执行器姿态所需的关节角度。

```python
import time
import math
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState
from kuavo_humanoid_sdk import KuavoPose
 
if __name__ == "__main__":
    # !!! Initialize Kuavo SDK with IK module !!!
    if not KuavoSDK.Init(options=KuavoSDK.Options.WithIK): # Init! Important! if U use IK, you must use this option!
        print("Failed to initialize Kuavo SDK")
        exit(1)
        
    robot = KuavoRobot()
    robot.arm_reset()
    robot_state = KuavoRobotState()

    # Front pose - arms in front, palms facing down
    front_left = [0.45, 0.28, 0.25]   # Left arm front position
    front_right = [0.45, -0.20, 0.25]  # Right arm front position
    r_front_orientation = [-0.41158, -0.503073, 0.577546, 0.493919]
    l_front_orientation = [0.38, -0.45, -0.56, 0.57]
    
    arm_q = robot_state.arm_joint_state().position

    res = robot.arm_ik(
        KuavoPose(position=front_left, orientation=l_front_orientation),
        KuavoPose(position=front_right, orientation=r_front_orientation)
    )
    
    if res:
        times = [0.5, 2.5]
        q_frames = [arm_q, res]
        robot.control_arm_joint_trajectory(times, q_frames)
        time.sleep(3.5)
        robot.set_fixed_arm_mode()
        time.sleep(1.0)
        curr_q = robot_state.arm_joint_state().position
        # !!! arm fk !!!
        l_pose, r_pose = robot.arm_fk(curr_q)
        print(l_pose)
        print(r_pose)
        robot.arm_reset()    
```

## 头部运动控制

这个示例展示了如何控制机器人头部运动，包括点头 (pitch) 和摇头 (yaw) 运动。

1. 头部上下点头运动：
   * 从0度开始，先向上抬到25度
   * 然后从25度向下转到-25度
   * 最后从-25度回到0度
   * 重复2个周期
2. 头部左右摇头运动：
   * 从0度开始，先向左转到60度
   * 然后从60度向右转到-60度
   * 最后从-60度回到0度
   * 重复2个周期

每次运动都以较小的角度增量(2度)变化，并设置了0.1秒的时间间隔，以实现平滑的运动效果。

```python
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
```

## 单步落足控制

这个示例展示了如何通过自定义落足点轨迹控制机器人运动。

1. 首先需要确保机器人在站立状态，才能切换到自定义落足点控制模式
2. 向前步进0.8米
3. 等待机器人回到站立状态
4. 再次步进0.2米并旋转90度

注意事项:

* 步进控制只能在站立模式下使用
* 每次步进后需要等待机器人回到站立状态
* 可以设置超时时间等待状态转换

```python
import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState

def main():
    if not KuavoSDK().Init():# Init!
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
```

## 控制机器人在世界坐标系或基座坐标系中的位姿

通过 control_command_pose_world 和 control_command_pose 函数分别控制机器人在世界坐标系和基座坐标系中的位置和姿态。示例中展示了：

1. 在世界坐标系中控制机器人前进1米并旋转90度
2. 在基座坐标系中控制机器人后退2米并旋转-90度

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
    # Command Pose - Odom
    robot.control_command_pose_world(0.0, 1.0, 0.0, 1.57)
    
    # 等待3s 
    time.sleep(10.0)

    # Stance
    robot.stance()
    # Command Pose - BaseLink
    robot.control_command_pose(-2.0, 0.0, 0.0, -1.57)
    
if __name__ == "__main__":
    main()
```

## 视觉获取 AprilTag 数据

**演示如何获取和处理机器人的视觉数据，特别是AprilTag标记的识别结果。**

通过 KuavoRobotVision 类获取相机识别到的AprilTag数据。示例代码展示了：

1. 获取不同坐标系下的AprilTag数据:
   * 相机坐标系
   * 基座坐标系
   * 里程计坐标系
2. 获取特定标记的详细信息:
   * 标记ID
   * 标记大小
   * 位置和姿态信息
3. 通过ID查询特定标记的数据

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

## 音频功能

这个示例展示了如何使用音频功能，包括播放音频和语音合成。

支持播放指定音频文件、TTS语音合成、停止音乐等操作，通过 KuavoRobotAudio 的 play_audio、text_to_speech、stop_music 等接口实现。

```python
import time
from kuavo_humanoid_sdk import KuavoSDK,KuavoRobotAudio

if not KuavoSDK().Init(log_level='INFO'):# Init!
    print("Init KuavoSDK failed, exit!")
    exit(1)
    
if __name__ == "__main__":
    kuavo_robot = KuavoRobotAudio()
    kuavo_robot.play_audio("2_抱拳.wav")
    time.sleep(3)
    kuavo_robot.text_to_speech("你好，世界")
    time.sleep(3)
    kuavo_robot.text_to_speech("你好，世界")
    kuavo_robot.stop_music()
```

## 电机 Kp/Kd 参数调整

这个示例展示了如何调整电机 Kp/Kd 参数，包括关节角度、关节速度和关节力矩。

**注意**: 该示例仅支持在实物且\`youda\`类型的驱动器电机上运行

```python
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
```

## 获取观测信息（控制指令）等

这个示例展示了如何获取机器人当前的观测信息，如手臂位置指令等。

通过 KuavoRobotObservation 实时获取并打印手臂的目标位置指令，适合用于调试和监控机器人状态。

```python
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
```
