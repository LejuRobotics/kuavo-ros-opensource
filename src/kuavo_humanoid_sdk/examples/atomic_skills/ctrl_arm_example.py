import time
import math
import rospy
from std_msgs.msg import Bool
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot,KuavoRobotState
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, KuavoManipulationMpcFrame

if not KuavoSDK().Init(log_level='INFO'):# Init!
    print("Init KuavoSDK failed, exit!")
    exit(1)

robot = KuavoRobot()

ARM_EXECUTING_TOPIC = "/humanoid/mpc/is_arm_executing"


class ArmExecutionMonitor:
    def __init__(self):
        self.received = False
        self.executing = False
        self.last_logged_state = None
        self.subscriber = None

    def start(self):
        self.subscriber = rospy.Subscriber(ARM_EXECUTING_TOPIC, Bool, self._callback, queue_size=1)

    def stop(self):
        if self.subscriber is not None:
            self.subscriber.unregister()
            self.subscriber = None

    def wait_complete(self, start_timeout, completion_timeout, stable_false_count=3):
        start = time.monotonic()
        executing_start = None
        false_count = 0

        while not rospy.is_shutdown():
            now = time.monotonic()

            if self.executing and executing_start is None:
                executing_start = now

            if executing_start is None:
                if now - start > start_timeout:
                    print("wait arm trajectory start timeout")
                    return False
            elif self.executing:
                false_count = 0
                if now - executing_start > completion_timeout:
                    print("wait arm trajectory complete timeout")
                    return False
            else:
                false_count += 1
                if false_count >= stable_false_count:
                    return True

            time.sleep(0.05)

        return False

    def _callback(self, msg):
        self.received = True
        self.executing = msg.data
        if self.last_logged_state is None or self.last_logged_state != msg.data:
            print(f"Arm executing state changed to {msg.data}")
            self.last_logged_state = msg.data


def control_arm_traj():
    global robot
     # reset arm
    robot.arm_reset()
    time.sleep(0.5)  # Wait for arm reset to complete

    # Switch to external control mode before controlling arm
    robot.set_external_control_arm_mode()
    time.sleep(0.5)  # Wait for mode switch to complete

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
        q = [math.radians(angle) for angle in q]
        robot.control_arm_joint_positions(q)
        time.sleep(0.02)

    # fixed arm
    robot.set_fixed_arm_mode()
    time.sleep(1.0)

    # Switch back to external control mode before controlling arm again
    robot.set_external_control_arm_mode()
    time.sleep(0.5)  # Wait for mode switch to complete

    # back to q0
    for i in range(num):
        q_tmp = [0.0]*14
        for j in range(14):
            q_tmp[j] = q1[j] - i/float(num)*(q1[j] - q0[j])
        q_tmp = [math.radians(angle) for angle in q_tmp]
        robot.control_arm_joint_positions(q_tmp)
        time.sleep(0.02)

    robot.set_auto_swing_arm_mode() # Restore auto arm swing mode

    robot.arm_reset() # Reset arm position


def control_arm_joint_trajectory():
    global robot
    # Switch to external control mode before controlling arm trajectory
    robot.set_external_control_arm_mode()
    time.sleep(0.5)  # Wait for mode switch to complete

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
    q_frames = [[math.radians(angle) for angle in pose[1]] for pose in target_poses]

    if not robot.control_arm_joint_trajectory(times, q_frames):
        print("control_arm_joint_trajectory failed!")
        return None

    return max(times) if times else 0.0

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

    monitor = ArmExecutionMonitor()
    monitor.start()
    action_b_keyframe_time_span = control_arm_joint_trajectory()
    if action_b_keyframe_time_span is None:
        monitor.stop()
        print("control_arm_joint_trajectory failed, skip arm_reset")
        exit(1)

    completed = monitor.wait_complete(
        start_timeout=action_b_keyframe_time_span + 15.0,
        completion_timeout=action_b_keyframe_time_span + 30.0,
    )
    monitor.stop()

    if not completed:
        print("arm trajectory did not report complete, skip arm_reset")
        exit(1)

    print("正在执行: robot.arm_reset() - 手臂复位")
    robot.arm_reset() # !!! after the arm reaches the target pose, reset the arm position.
    time.sleep(2.0)  # Wait for arm reset to complete (arm_reset switches to AutoSwing mode)

    # Switch to external control mode before controlling end effector pose
    print("正在切换到外部控制模式...")
    robot.set_external_control_arm_mode()
    time.sleep(0.5)  # Wait for mode switch to complete

    print("正在执行: control_arm_end_effector_pose() - 手臂末端执行器位姿控制")
    control_arm_end_effector_pose()
    time.sleep(3)
    print("正在执行: robot.manipulation_mpc_reset() - 操作MPC复位")
    robot.manipulation_mpc_reset()
    time.sleep(0.5)  # Wait for MPC reset to complete

    print("正在执行: robot.arm_reset() - 手臂复位")
    robot.arm_reset()  # Reset arm position after MPC reset
    time.sleep(2.0)  # Wait for arm reset to complete
    robot.manipulation_mpc_reset()
    robot.arm_reset()
