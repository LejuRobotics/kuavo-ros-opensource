import time
import signal
from kuavo_humanoid_sdk import KuavoPose
from kuavo_humanoid_sdk.kuavo.robot import KuavoRobot
from kuavo_humanoid_sdk.kuavo.robot_state import KuavoRobotState
from kuavo_humanoid_sdk.kuavo.robot_audio import KuavoRobotAudio
from kuavo_humanoid_sdk.kuavo.robot_climbstair import KuavoRobotClimbStair, set_pitch_limit
from kuavo_humanoid_sdk.interfaces.data_types  import KuavoPose
from kuavo_msgs.srv import planArmTrajectoryBezierCurve, planArmTrajectoryBezierCurveRequest
from kuavo_msgs.msg import planArmState, jointBezierTrajectory, bezierCurveCubicPoint, robotHandPosition, robotHeadMotionData, sensorsData, robotWaistControl
from geometry_msgs.msg import Twist
import asyncio
import math
import json
import os
import rospkg
import rospy
import threading
import tf
import time
import pwd
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from geometry_msgs.msg import Pose, Point, Quaternion

from std_msgs.msg import Float64MultiArray

import inspect
from kuavo_humanoid_sdk.kuavo.core.model_utils.model_utils import *
from kuavo_humanoid_sdk.kuavo.core.model_utils import model_utils as model_utils

from enum import IntEnum
# Global flag for handling Ctrl+C
running = False
robot_version = int(os.environ.get("ROBOT_VERSION", "45"))
ocs2_joint_state = JointState()
ocs2_hand_state = robotHandPosition()
ocs2_head_state = robotHeadMotionData()
ocs2_waist_state = robotWaistControl()
KUAVO_TACT_LENGTH = 28
ROBAN_TACT_LENGTH = 23
if robot_version >= 40:
    INIT_ARM_POS = [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    current_arm_joint_state = [0] * KUAVO_TACT_LENGTH
else:
    INIT_ARM_POS = [22.91831, 0, 0, -45.83662, 22.91831, 0, 0, -45.83662, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] # task.info: shoudler_center: 0.4rad, elbow_center: -0.8rad
    current_arm_joint_state = [0] * ROBAN_TACT_LENGTH


def signal_handler(sig, frame):
    """Signal handler for catching Ctrl+C and safely stopping the robot.

    Args:
        sig: Signal number.
        frame: Current stack frame.
    """
    global running
    print('\nCtrl+C pressed. Stopping robot...')
    running = False
    rospy.signal_shutdown("Ctrl+C pressed")
    exit(0)

def frames_to_custom_action_data(file_path: str):
    """Parse action file and convert frames to custom action data.

    Args:
        file_path (str): Path to the action file.

    Returns:
        dict: Action data for each joint.
    """
    with open(file_path, "r") as f:
        data = json.load(f)
    frames = data["frames"]
    action_data = {}
    for frame in frames:
        servos, keyframe, attribute = frame["servos"], frame["keyframe"], frame["attribute"]
        for index, value in enumerate(servos):
            key = index + 1
            if key not in action_data:
                action_data[key] = []
                if keyframe != 0 and len(action_data[key]) == 0:
                    if key <= len(INIT_ARM_POS):
                        action_data[key].append([
                            [0, INIT_ARM_POS[key-1]],
                            [0, 0],
                            [0, INIT_ARM_POS[key-1]],
                        ])
            if value is not None:
                CP = attribute[str(key)]["CP"]
                left_CP, right_CP = CP
                action_data[key].append([
                    [round(keyframe/100, 1), int(value)],
                    [round((keyframe+left_CP[0])/100, 1), int(value+left_CP[1])],
                    [round((keyframe+right_CP[0])/100, 1), int(value+right_CP[1])],
                ])
    return action_data

def verify_robot_version(file_path: str):
    with open(file_path, "r") as f:
        data = json.load(f)
    
    robot_type_raw = data.get("robotType", None)
    if robot_type_raw is None:
        msg = "Action file missing required field: robotType"
        rospy.logerr(msg)
        return False, msg
    try:
        tact_robot_version = int(robot_type_raw)
    except (TypeError, ValueError):
        msg = f"Invalid robotType in action file: {robot_type_raw}"
        rospy.logerr(msg)
        return False, msg

    # 版本兼容关系映射
    version_compat_map = {
        41: [41],
        42: [42],
        45: [43, 45, 46, 48, 49],
        11: [11, 13, 14],
        13: [11, 13, 14],
        14: [11, 13, 14]
    }
    allowed_robot_versions = version_compat_map.get(tact_robot_version, [tact_robot_version])
    if robot_version not in allowed_robot_versions:
        msg = (
            f"Version mismatch: tact {tact_robot_version} is incompatible with robot {robot_version}"
        )
        rospy.logerr(msg)
        return False, msg
    
    rospy.loginfo(f"Version match: tact {tact_robot_version} is compatible with robot {robot_version}")
    return True, None

def get_start_end_frame_time(file_path: str):
    """Get the start and end frame time from the action file.

    Args:
        file_path (str): Path to the action file.

    Returns:
        tuple: (start_frame_time, end_frame_time)
    """
    with open(file_path, "r") as f:
        data = json.load(f)
    start_frame_time = data["first"] / 100
    end_frame_time = data["finish"] / 100
    return start_frame_time, end_frame_time

def frames_to_custom_action_data_ocs2(file_path: str, start_frame_time: float, current_arm_joint_state: list):
    """Parse action file and convert frames to custom action data for OCS2.

    Args:
        file_path (str): Path to the action file.
        start_frame_time (float): Start frame time.
        current_arm_joint_state (list): Current arm joint state.

    Returns:
        dict: Filtered action data for each joint.
    """
    def filter_data(action_data, start_frame_time, current_arm_joint_state):
        """Filter and adjust action data for OCS2.

        Args:
            action_data (dict): Raw action data.
            start_frame_time (float): Start frame time.
            current_arm_joint_state (list): Current arm joint state.

        Returns:
            dict: Filtered action data.
        """
        filtered_action_data = {}
        x_shift = start_frame_time - 1
        for key, frames in action_data.items():
            filtered_frames = []
            found_start = False
            skip_next = False
            for i in range(-1, len(frames)):
                frame = frames[i]
                if i == len(frames) - 1:
                    next_frame = frame
                else:
                    next_frame = frames[i+1]
                end_time = next_frame[0][0]

                if not found_start and end_time >= start_frame_time:
                    found_start = True

                    p0 = np.array([0, current_arm_joint_state[key-1]])
                    p3 = np.array([next_frame[0][0] - x_shift, next_frame[0][1]])

                    # Calculate control points for smooth transition
                    curve_length = np.linalg.norm(p3 - p0)
                    p1 = p0 + curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the right
                    p2 = p3 - curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the left

                    # Create new frame
                    frame1 = [
                        p0.tolist(),
                        p0.tolist(),
                        p1.tolist()
                    ]

                    # Modify next_frame's left control point
                    next_frame[1] = p2.tolist()

                    filtered_frames.append(frame1)
                    skip_next = True

                if found_start:
                    if skip_next:
                        skip_next = False
                        continue
                    end_point = [round(frame[0][0] - x_shift, 1), round(frame[0][1], 1)]
                    left_control_point = [round(frame[1][0] - x_shift, 1), round(frame[1][1], 1)]
                    right_control_point = [round(frame[2][0] - x_shift, 1), round(frame[2][1], 1)]
                    filtered_frames.append([end_point, left_control_point, right_control_point])

            filtered_action_data[key] = filtered_frames
        return filtered_action_data

    with open(file_path, "r") as f:
        data = json.load(f)
    frames = data["frames"]
    action_data = {}
    for frame in frames:
        servos, keyframe, attribute = frame["servos"], frame["keyframe"], frame["attribute"]
        for index, value in enumerate(servos):
            key = index + 1
            if key not in action_data:
                action_data[key] = []
                if keyframe != 0 and len(action_data[key]) == 0:
                    if key <= len(INIT_ARM_POS):
                        action_data[key].append([
                            [0, math.radians(INIT_ARM_POS[key-1])],
                            [0, math.radians(INIT_ARM_POS[key-1])],
                            [0, math.radians(INIT_ARM_POS[key-1])],
                        ])
            if value is not None:
                CP = attribute[str(key)]["CP"]
                left_CP, right_CP = CP
                action_data[key].append([
                    [round(keyframe/100, 1), math.radians(value)],
                    [round((keyframe+left_CP[0])/100, 1), math.radians(value+left_CP[1])],
                    [round((keyframe+right_CP[0])/100, 1), math.radians(value+right_CP[1])],
                ])
    return filter_data(action_data, start_frame_time, current_arm_joint_state)


class RobotArmControlMode(IntEnum):
    """Enum for robot arm control modes.

    Attributes:
        Fixed: Fixed mode, arm remains stationary.
        AutoSwing: Auto swing mode, arm swings automatically.
        ExternalControl: External control mode, arm is controlled by external signals.
    """
    Fixed = 0           # Fixed mode, arm remains stationary
    AutoSwing = 1       # Auto swing mode, arm swings automatically
    ExternalControl = 2 # External control mode, arm is controlled externally

if robot_version >= 40:
    joint_names = [
        "l_arm_pitch",
        "l_arm_roll",
        "l_arm_yaw",
        "l_forearm_pitch",
        "l_hand_yaw",
        "l_hand_pitch",
        "l_hand_roll",
        "r_arm_pitch",
        "r_arm_roll",
        "r_arm_yaw",
        "r_forearm_pitch",
        "r_hand_yaw",
        "r_hand_pitch",
        "r_hand_roll",
    ]
else:
    joint_names = [
        "zarm_l1_link", "zarm_l2_link", "zarm_l3_link", "zarm_l4_link",
        "zarm_r1_link", "zarm_r2_link", "zarm_r3_link", "zarm_r4_link",
    ]


robot_settings = {
    "kuavo": {
        "plan_arm_trajectory_bezier_service_name": "/plan_arm_trajectory_bezier_curve",
        "stop_plan_arm_trajectory_service_name": "/stop_plan_arm_trajectory",
        "arm_traj_state_topic_name": "/robot_plan_arm_state",
    },
    "ocs2": {
        "plan_arm_trajectory_bezier_service_name": "/bezier/plan_arm_trajectory",
        "stop_plan_arm_trajectory_service_name": "/bezier/stop_plan_arm_trajectory",
        "arm_traj_state_topic_name": "/bezier/arm_traj_state",
    }
}

class RobotControlBlockly:
    def __init__(self):
        """Initialize the robot control class, create robot objects, register signal handlers, and initialize ROS publishers and subscribers."""
        self.robot = KuavoRobot()
        self.robot_state = KuavoRobotState()
        self.robot_audio = KuavoRobotAudio()
        self.climb_stair = KuavoRobotClimbStair()
        # Register signal handler
        signal.signal(signal.SIGINT, signal_handler)
        global g_robot_type
        g_robot_type = "ocs2"
        self.loop = None
        self.init_ros_publishers()
        self.THRESHOLD_HEAD_CONTROL_COUNT = 30
        self.FLOAT_MAX = float('inf')
        self.to_stance()

        package_name = 'planarmwebsocketservice'
        self.package_path = rospkg.RosPack().get_path(package_name)
        sudo_user = os.environ.get("SUDO_USER")
        if sudo_user:
            user_info = pwd.getpwnam(sudo_user)
            home_path = user_info.pw_dir
        else:
            home_path = os.path.expanduser("~")
        self.ACTION_FILE_FOLDER = os.path.join(home_path, '.config', 'lejuconfig', 'action_files')
        self.plan_arm_state_progress = 0
        self.plan_arm_state_status = False

    def set_action_file_path(self, proj_name: str):
        """Set the action file path for a specific project.

        Args:
            proj_name (str): Project name.
        """
        self.ACTION_FILE_FOLDER = self.package_path + "/upload_files/" + proj_name + "/action_files"

    def init_ros_publishers(self):
        """Initialize ROS publishers and subscribers for arm, hand, head control topics, sensor data, and trajectory state."""
        global kuavo_arm_traj_pub, control_hand_pub, control_head_pub, g_robot_type, control_waist_pub
        kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=1, tcp_nodelay=True)
        control_hand_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=1, tcp_nodelay=True)
        control_head_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=1, tcp_nodelay=True)
        control_waist_pub = rospy.Publisher('/robot_waist_motion_data', robotWaistControl, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/dexhand/state', JointState, self.robot_hand_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/sensors_data_raw', sensorsData, self.sensors_data_callback, queue_size=1, tcp_nodelay=True)
        rospy.Timer(rospy.Duration(0.01), self.timer_callback)
        rospy.Subscriber(robot_settings[g_robot_type]["arm_traj_state_topic_name"], planArmState, self.plan_arm_state_callback)
        rospy.Subscriber('/bezier/arm_traj', JointTrajectory, self.traj_callback, queue_size=1, tcp_nodelay=True)

    def timer_callback(self, event):
        """Timer callback for periodically publishing arm, hand, and head state to ROS topics.

        Only publishes when robot type is 'ocs2' and trajectory is not finished.
        """
        global kuavo_arm_traj_pub, control_hand_pub, control_head_pub, g_robot_type
        if g_robot_type == "ocs2" and len(ocs2_joint_state.position) > 0 and self.plan_arm_state_status is False:
            if len(ocs2_joint_state.position) != 0:
                kuavo_arm_traj_pub.publish(ocs2_joint_state)
            if len(ocs2_hand_state.left_hand_position) != 0 or len(ocs2_hand_state.right_hand_position) != 0:
                control_hand_pub.publish(ocs2_hand_state)
            if len(ocs2_head_state.joint_data) != 0:
                control_head_pub.publish(ocs2_head_state)
            if len(ocs2_waist_state.data.data) != 0:
                control_waist_pub.publish(ocs2_waist_state)

    def sensors_data_callback(self, msg):
        """更新关节数据"""
        self._last_joint_msg = msg

        global current_head_joint_state
        current_head_joint_state = msg.joint_data.joint_q[-2:]
        current_head_joint_state = [round(pos, 4) for pos in current_head_joint_state]

        if not hasattr(self, "_last_hand_msg"):
            dummy_hand = JointState()
            dummy_hand.position = [0.0] * 12
            self._last_hand_msg = dummy_hand

        self._update_current_arm_joint_state(self._last_joint_msg, self._last_hand_msg)

    def robot_hand_callback(self, msg):
        """更新手部数据"""
        left = msg.position[:6] if len(msg.position) >= 6 else [0] * 6
        right = msg.position[6:12] if len(msg.position) >= 12 else [0] * 6

        self._last_hand_msg = msg

        if hasattr(self, "_last_joint_msg"):
            self._update_current_arm_joint_state(self._last_joint_msg, self._last_hand_msg)

    def _update_current_arm_joint_state(self, joint_msg, hand_msg):
        """整合 joint_msg 和 hand_msg，更新 current_arm_joint_state"""
        global robot_version, current_arm_joint_state
        if robot_version >= 40:
            arm_part = list(joint_msg.joint_data.joint_q[12:26])
            hand_part = list(hand_msg.position[:12]) if len(hand_msg.position) >= 12 else [0.0] * 12
            head_part = list(joint_msg.joint_data.joint_q[-2:])
            current_arm_joint_state = arm_part + hand_part + head_part

        elif robot_version >= 10 and robot_version < 30:
            # 按照 joint_q 索引顺序定义变量
            hand_part = list(hand_msg.position[:12]) if len(hand_msg.position) >= 12 else [0.0] * 12  # 对应 joint_q[0:12]
            waist_part = [joint_msg.joint_data.joint_q[12]]  # 对应 joint_q[12]
            arm_part = list(joint_msg.joint_data.joint_q[13:21])  # 对应 joint_q[13:21]
            head_part = list(joint_msg.joint_data.joint_q[21:23])  # 对应 joint_q[21:23]
            # 保持最终组合顺序不变：arm_part + hand_part + head_part + waist_part
            current_arm_joint_state = arm_part + hand_part + head_part + waist_part

        current_arm_joint_state = [round(v, 2) for v in current_arm_joint_state]

    def plan_arm_state_callback(self, msg: planArmState):
        """Callback for arm trajectory state, updates global variables for progress and completion.

        Args:
            msg (planArmState): Arm trajectory state message.
        """
        self.plan_arm_state_progress = msg.progress
        self.plan_arm_state_status = msg.is_finished

    def traj_callback(self, msg):
        """Callback for trajectory messages, parses trajectory points and updates global arm, hand, and head state variables.

        Args:
            msg (JointTrajectory): Trajectory message.
        """
        global ocs2_joint_state, ocs2_hand_state, ocs2_head_state, ocs2_waist_state, robot_version
        if len(msg.points) == 0:
            return
        point = msg.points[0]

        if robot_version >= 40:
            ocs2_joint_state.name = joint_names
            ocs2_joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
            ocs2_joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
            ocs2_joint_state.effort = [0] * 14
            ocs2_hand_state.left_hand_position = [int(math.degrees(pos)) for pos in point.positions[14:20]]
            ocs2_hand_state.right_hand_position = [int(math.degrees(pos)) for pos in point.positions[20:26]]
            ocs2_head_state.joint_data = [math.degrees(pos) for pos in point.positions[26:]]

        elif robot_version >= 10 and robot_version < 30:
            ocs2_joint_state.name = joint_names
            ocs2_joint_state.position = [math.degrees(pos) for pos in point.positions[:8]]
            ocs2_joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:8]]
            ocs2_joint_state.effort = [0] * 8
            if len(point.positions) == ROBAN_TACT_LENGTH:
                ocs2_hand_state.left_hand_position = [int(math.degrees(pos)) for pos in point.positions[8:14]]
                ocs2_hand_state.right_hand_position = [int(math.degrees(pos)) for pos in point.positions[14:20]]
                ocs2_head_state.joint_data = [math.degrees(pos) for pos in point.positions[20:22]]
                ocs2_waist_state.header.stamp = rospy.Time.now()
                ocs2_waist_state.data.data = [math.degrees(pos) for pos in point.positions[22:]]

    def arm_reset(self):
        """Reset the arm position to the initial state."""
        try:
            self.robot.arm_reset()
            time.sleep(1)
        except Exception as e:
            print(f"Arm reset failed: {str(e)}")

    def set_auto_swing_arm_mode(self):
        """Set the arm to auto swing mode."""
        try:
            self.robot.set_auto_swing_arm_mode()
            time.sleep(1)
        except Exception as e:
            print(f"Set auto swing arm mode failed: {str(e)}")

    def set_fixed_arm_mode(self):
        """Set the arm to fixed mode."""
        try:
            self.robot.set_fixed_arm_mode()
            time.sleep(1)
        except Exception as e:
            print(f"Set fixed arm mode failed: {str(e)}")

    def set_external_control_arm_mode(self):
        """Set the arm to external control mode."""
        try:
            if self.robot.set_external_control_arm_mode():
                print("Set external control mode succeeded")
            else:
                print("Set external control mode failed")
            time.sleep(1)
        except Exception as e:
            print(f"Set external control arm mode failed: {str(e)}")

    def set_arm_control_mode(self, mode: int):
        """Set the arm control mode.

        Args:
            mode (int): 0 - Fixed, 1 - AutoSwing, 2 - ExternalControl
        """
        try:
            if mode == RobotArmControlMode.Fixed:
                self.set_fixed_arm_mode()
            elif mode == RobotArmControlMode.AutoSwing:
                self.set_auto_swing_arm_mode()
            elif mode == RobotArmControlMode.ExternalControl:
                self.set_external_control_arm_mode()
        except Exception as e:
            print(f"Set arm control mode failed: {str(e)}")

    def _start_loop_in_thread(self):
        """Start a new asyncio event loop in a separate thread."""
        self.loop = asyncio.new_event_loop()
        def run_loop():
            asyncio.set_event_loop(self.loop)
            self.loop.run_forever()
        t = threading.Thread(target=run_loop, daemon=True)
        t.start()

    def walk(self, linear_x: float, linear_y: float, angular_z: float):
        """Start walking with specified velocities.

        Args:
            linear_x (float): Linear velocity in x direction.
            linear_y (float): Linear velocity in y direction.
            angular_z (float): Angular velocity around z axis.
        """
        if self.loop is None:
            self._start_loop_in_thread()
        global running
        # If already walking, stop first
        global global_linear_x, global_linear_y, global_angular_z
        global_linear_x = linear_x
        global_linear_y = linear_y
        global_angular_z = angular_z
        if running:
            self.robot.walk(global_linear_x, global_linear_y, global_angular_z)
        else:
            self.control_robot_height("up", 0.4, 0.0)
            running = True
            async def robot_walk():
                try:
                    while running:
                        self.robot.walk(global_linear_x, global_linear_y, global_angular_z)
                        await asyncio.sleep(0.1)
                    self.robot.walk(0.0, 0.0, 0.0)
                    await asyncio.sleep(0.5)
                    self.robot.stance()
                except asyncio.CancelledError:
                    pass
            asyncio.run_coroutine_threadsafe(robot_walk(), self.loop)

    def walk_angle(self, linear_x: float, linear_y: float, angular_z: float):
        """Start walking with specified velocities.

        Args:
            linear_x (float): Linear velocity in x direction.
            linear_y (float): Linear velocity in y direction.
            angular_z (float): Angular velocity around z axis.
        """
        # 角度转弧度
        angular_z = math.radians(angular_z)
        self.walk(linear_x, linear_y, angular_z)

    def start(self):
        """Start robot movement and Begin trot."""
        self.robot.trot()

    def stop(self):
        """Stop robot movement and set the running flag to False."""
        global running
        running = False
        while True:
            time.sleep(0.01)
            if self.robot_state.is_stance():
                break
        # Wait for gait switch to complete
        time.sleep(0.5)

    def control_arm_position(self, joint_positions: list):
        """Move the arm to the specified joint angles using interpolation for smooth transition.

        Args:
            joint_positions (list): Target joint angles in degrees.
        """
        try:
            self.robot.arm_reset()
            q_list = []
            q0 = [0.0] * 8
            num = 90
            for i in range(num):
                q_tmp = [0.0] * 8
                for j in range(8):
                    q_tmp[j] = q0[j] + i / float(num) * (joint_positions[j] - q0[j])
                q_list.append(q_tmp)
            for q in q_list:
                # Convert degrees to radians
                q = [math.radians(angle) for angle in q]
                self.robot.control_arm_position(q)
                time.sleep(0.02)
        except Exception as e:
            print(f"Arm position control failed: {str(e)}")

    def stance(self):
        """Set the robot to standing posture."""
        try:
            self.robot.stance()
            while True:
                time.sleep(0.01)
                if self.robot_state.is_stance():
                    break
        except Exception as e:
            print(f"Stance adjustment failed: {str(e)}")

    def create_bezier_request(self, action_data, start_frame_time, end_frame_time):
        """Construct a Bezier curve trajectory request message for arm trajectory planning service.

        Args:
            action_data (dict): Joint action data.
            start_frame_time (float): Start frame time.
            end_frame_time (float): End frame time.

        Returns:
            planArmTrajectoryBezierCurveRequest: Bezier curve trajectory request object.
        """
        global robot_version
        req = planArmTrajectoryBezierCurveRequest()
        for key, value in action_data.items():
            msg = jointBezierTrajectory()
            for frame in value:
                point = bezierCurveCubicPoint()
                point.end_point, point.left_control_point, point.right_control_point = frame
                msg.bezier_curve_points.append(point)
            req.multi_joint_bezier_trajectory.append(msg)
        req.start_frame_time = start_frame_time
        req.end_frame_time = end_frame_time
        if robot_version >= 40:
            req.joint_names = [
                "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch", "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
                "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", "r_hand_roll"
            ]
        else:
            req.joint_names = [
                "zarm_l1_link", "zarm_l2_link", "zarm_l3_link", "zarm_l4_link",
                "zarm_r1_link", "zarm_r2_link", "zarm_r3_link", "zarm_r4_link"
            ]
        return req

    def plan_arm_trajectory_bezier_curve_client(self, req):
        """Call the arm Bezier curve trajectory planning service and send the trajectory request.

        Args:
            req: Bezier trajectory request object.

        Returns:
            bool: True if service call succeeded, False otherwise.
        """
        service_name = robot_settings[g_robot_type]["plan_arm_trajectory_bezier_service_name"]
        rospy.wait_for_service(service_name)
        try:
            plan_service = rospy.ServiceProxy(service_name, planArmTrajectoryBezierCurve)
            res = plan_service(req)
            return res.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def excute_action_file(self, action_file: str, proj_name: str = None, music_file: str = None):
        """Execute an action file, parse action frames, and send Bezier trajectory request to the robot.

        Args:
            action_file (str): Action file name.
            proj_name (str, optional): Project name.
        """

        print("⚠️ 警告: 此接口(excute_action_file)即将废弃，请使用新接口execute_action_file")

        global g_robot_type
        g_robot_type = "ocs2"
        self.plan_arm_state_status = False

        action_filename = action_file + ".tact"
        if proj_name is None:
            action_file_path = os.path.expanduser(f"{self.ACTION_FILE_FOLDER}/{action_filename}")
        else:
            action_file_path = self.package_path + "/upload_files/" + proj_name + "/action_files/" + action_filename
        if not os.path.exists(action_file_path):
            print(f"Action file not found: {action_file_path}")
            return

        is_compatible, msg = verify_robot_version(action_file_path)
        if not is_compatible:
            print(msg)
            return

        start_frame_time, end_frame_time = get_start_end_frame_time(action_file_path)

        if g_robot_type == "ocs2":
            action_frames = frames_to_custom_action_data_ocs2(action_file_path, start_frame_time, current_arm_joint_state)
            end_frame_time += 1
            self.set_arm_control_mode(2)
        else:
            action_frames = frames_to_custom_action_data(action_file_path)

        req = self.create_bezier_request(action_frames, start_frame_time, end_frame_time)
        if music_file is not None:
            self.play_music(music_file)
        self.plan_arm_trajectory_bezier_curve_client(req)
        time.sleep(0.5)
        while self.plan_arm_state_status is False:
            time.sleep(0.01)
        print("action file executed")

    def execute_action_file(self, action_file: str, proj_name: str = None, music_file: str = None):
        """Execute an action file, parse action frames, and send Bezier trajectory request to the robot.

        Args:
            action_file (str): Action file name.
            proj_name (str, optional): Project name.
        """

        global g_robot_type
        g_robot_type = "ocs2"
        self.plan_arm_state_status = False

        action_filename = action_file + ".tact"
        if proj_name is None:
            action_file_path = os.path.expanduser(f"{self.ACTION_FILE_FOLDER}/{action_filename}")
        else:
            action_file_path = self.package_path + "/upload_files/" + proj_name + "/action_files/" + action_filename
        if not os.path.exists(action_file_path):
            print(f"Action file not found: {action_file_path}")
            return

        is_compatible, msg = verify_robot_version(action_file_path)
        if not is_compatible:
            print(msg)
            return

        start_frame_time, end_frame_time = get_start_end_frame_time(action_file_path)

        if g_robot_type == "ocs2":
            action_frames = frames_to_custom_action_data_ocs2(action_file_path, start_frame_time, current_arm_joint_state)
            end_frame_time += 1
            self.set_arm_control_mode(2)
        else:
            action_frames = frames_to_custom_action_data(action_file_path)

        req = self.create_bezier_request(action_frames, start_frame_time, end_frame_time)
        if music_file is not None:
            self.play_music(music_file)
        self.plan_arm_trajectory_bezier_curve_client(req)
        time.sleep(0.5)
        while self.plan_arm_state_status is False:
            time.sleep(0.01)
        print("action file executed")


    def control_robot_head(self, yaw: float, pitch: float):
        """Control the robot head rotation.

        Args:
            yaw (float): Yaw angle in degrees.
            pitch (float): Pitch angle in degrees.
        """
        try:
            self.robot.stance()
            yaw = math.radians(yaw)
            pitch = math.radians(pitch)
            self.robot.control_head(yaw, pitch)
            diff_yaw_min = self.FLOAT_MAX
            diff_pitch_min = self.FLOAT_MAX
            times_count = 0
            while True:
                time.sleep(0.01)
                diff_yaw = abs(current_head_joint_state[0] - yaw)
                diff_pitch = abs(current_head_joint_state[1] - pitch)

                if diff_yaw < diff_yaw_min:
                    times_count = 0
                    diff_yaw_min = diff_yaw
                    continue
                if diff_pitch < diff_pitch_min:
                    times_count = 0
                    diff_pitch_min = diff_pitch
                    continue
                times_count += 1
                if times_count > self.THRESHOLD_HEAD_CONTROL_COUNT:
                    break

        except Exception as e:
            print(f"Robot head control failed: {str(e)}")

    def control_robot_head_only_yaw(self, yaw: float):
        """Control the robot head yaw only.

        Args:
            yaw (float): Yaw angle in degrees.
        """
        try:
            self.robot.stance()
            yaw = math.radians(yaw)
            self.robot.control_head(yaw, current_head_joint_state[1])
            times_count = 0
            diff_yaw_min = self.FLOAT_MAX
            while True:
                time.sleep(0.01)
                diff_yaw = abs(current_head_joint_state[0] - yaw)
                if diff_yaw < diff_yaw_min:
                    times_count = 0
                    diff_yaw_min = diff_yaw
                    continue
                times_count += 1
                if times_count > self.THRESHOLD_HEAD_CONTROL_COUNT:
                    break
        except Exception as e:
            print(f"Robot head control failed: {str(e)}")

    def control_robot_head_only_pitch(self, pitch: float):
        """Control the robot head pitch only.

        Args:
            pitch (float): Pitch angle in degrees.
        """
        try:
            self.robot.stance()
            pitch = math.radians(pitch)
            self.robot.control_head(current_head_joint_state[0], pitch)
            times_count = 0
            diff_pitch_min = self.FLOAT_MAX
            while True:
                time.sleep(0.01)
                diff_pitch = abs(current_head_joint_state[1] - pitch)
                if diff_pitch < diff_pitch_min:
                    times_count = 0
                    diff_pitch_min = diff_pitch
                    continue
                times_count += 1
                if times_count > self.THRESHOLD_HEAD_CONTROL_COUNT:
                    break
        except Exception as e:
            print(f"Robot head control failed: {str(e)}")

    def control_robot_height(self, is_up: str, height: float, pitch: float = 0.0):
        """Control the robot's height and body pitch.

        Args:
            is_up (str): "up" to increase height, "down" to decrease height.
            height (float): Target height.
            pitch (float, optional): Body pitch angle. Defaults to 0.0.
        """
        try:
            global com_height
            
            while not self.robot_state.is_stance():
                self.robot.stance()
                time.sleep(0.01)
            now_height = self.robot_state.com_height
            mass_height = now_height - com_height
            if is_up == "up":
                height = mass_height + height
            elif is_up == "down":
                height = mass_height - height
            else:
                raise ValueError("is_up must be 'up' or 'down'")

            if height >= 0.0:
                height = -0.0000000001
            self.robot.squat(height, pitch)
            while True:
                # print(self.robot_state.com_height - com_height - height)
                if abs(self.robot_state.com_height - com_height - height) < 0.1:
                    break
        except Exception as e:
            print(f"Robot height control failed: {str(e)}")

    def to_stance(self):
        """Return the robot to the standard standing posture, including arm reset and head centering."""
        try:
            self.arm_reset()
            self.control_robot_head(0, 0)
            self.control_waist_rotation(0)
            init_hand_state = robotHandPosition()
            init_hand_state.left_hand_position = [0] * 6
            init_hand_state.right_hand_position = [0] * 6
            global control_hand_pub
            control_hand_pub.publish(init_hand_state)
            self.robot._kuavo_core._control.robot_stance()
            global com_height
            while True:
                time.sleep(0.01)
                if self.robot_state.is_stance():
                    break
            time.sleep(0.5)
            com_height = self.robot_state.com_height
        except Exception as e:
            print(f"Stance adjustment failed: {str(e)}")

    def slove_move_time(self, current_pos, target_pos):
        """Calculate the time required for the arm to move to the target position.

        Args:
            current_pos (list): Current position.
            target_pos (list): Target position.

        Returns:
            float: Move time in seconds.
        """
        left_dist = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
        right_dist = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
        max_dist = max(left_dist, right_dist)
        min_time = 1.0
        max_time = 10.0
        # Linear interpolation: 1m -> 10s, 0m -> 1s
        move_time = min_time + (max_time - min_time) * min(max_dist, 1.0)
        return move_time

    def control_arm_target_pose(self, x1, y1, z1, roll1, pitch1, yaw1, x2, y2, z2, roll2, pitch2, yaw2) -> bool:
        """Calculate target joint angles using IK and move the arm to the target pose.

        Args:
            x1, y1, z1, roll1, pitch1, yaw1: Left arm end-effector position and orientation.
            x2, y2, z2, roll2, pitch2, yaw2: Right arm end-effector position and orientation.

        Returns:
            bool: True if successful, False otherwise.
        """
        # Get the transform from zarm_r7_end_effector to base_link using tf
        listener = tf.TransformListener()
        try:
            listener.waitForTransform("base_link", "zarm_r7_end_effector", rospy.Time(0), rospy.Duration(2.0))
            listener.waitForTransform("base_link", "zarm_l7_end_effector", rospy.Time(0), rospy.Duration(2.0))
            (trans_r, rot_r) = listener.lookupTransform("base_link", "zarm_r7_end_effector", rospy.Time(0))
            (trans_l, rot_l) = listener.lookupTransform("base_link", "zarm_l7_end_effector", rospy.Time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Failed to get tf from zarm_r7_end_effector to base_link: ", str(e))
            return False
        global current_arm_joint_state
        front_left = [x1, y1, z1]   # Left arm target position
        front_right = [x2, y2, z2]  # Right arm target position
        # Convert Euler angles to quaternion
        l_front_orientation = tf.transformations.quaternion_from_euler(yaw1, pitch1, roll1)
        r_front_orientation = tf.transformations.quaternion_from_euler(yaw2, pitch2, roll2)

        res = self.robot.arm_ik_free(
            KuavoPose(position=front_left, orientation=l_front_orientation),
            KuavoPose(position=front_right, orientation=r_front_orientation)
        )

        if res:
            # Calculate the spatial distance between current and target end-effector positions
            current_left_pos = trans_l
            current_right_pos = trans_r
            target_left_pos = front_left
            target_right_pos = front_right
            move_time_left = self.slove_move_time(current_left_pos, target_left_pos)
            move_time_right = self.slove_move_time(current_right_pos, target_right_pos)
            move_time = max(move_time_left, move_time_right)
            times = [1.0, move_time]
            q_frames = [current_arm_joint_state[0:8], res]
            self.robot.control_arm_joint_trajectory(times, q_frames)
            time.sleep(move_time + 1.0)
            # self.set_auto_swing_arm_mode()
            # time.sleep(1.0)
            return True
        else:
            return False

    def control_arm_target_pose_by_single(self, hand_type, x, y, z, roll, pitch, yaw) -> bool:
        """Move a single arm to the specified spatial position and orientation.

        Args:
            hand_type (str): "left" or "right".
            x, y, z (float): Target position.
            roll, pitch, yaw (float): Target orientation.

        Returns:
            bool: True if successful, False otherwise.
        """
        # Get the transform from zarm_r7_end_effector to base_link using tf
        listener = tf.TransformListener()
        try:
            listener.waitForTransform("base_link", "zarm_r7_end_effector", rospy.Time(0), rospy.Duration(2.0))
            listener.waitForTransform("base_link", "zarm_l7_end_effector", rospy.Time(0), rospy.Duration(2.0))
            (trans_r, rot_r) = listener.lookupTransform("base_link", "zarm_r7_end_effector", rospy.Time(0))
            (trans_l, rot_l) = listener.lookupTransform("base_link", "zarm_l7_end_effector", rospy.Time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Failed to get tf from zarm_r7_end_effector to base_link: ", str(e))
            return False

        global current_arm_joint_state
        if hand_type == "left":
            front_left = [x, y, z]   # Left arm target position
            front_right = trans_r    # Right arm current position
            # Convert left arm Euler angles to quaternion
            l_front_orientation = tf.transformations.quaternion_from_euler(yaw, pitch, roll)
            r_front_orientation = rot_r
            res = self.robot.arm_ik_free(
                KuavoPose(position=front_left, orientation=l_front_orientation),
                KuavoPose(position=front_right, orientation=r_front_orientation)
            )

            if res:
                # Calculate the spatial distance between current and target end-effector positions
                current_left_pos = trans_l
                current_right_pos = trans_r
                target_left_pos = front_left
                target_right_pos = front_right
                move_time = self.slove_move_time(current_left_pos, target_left_pos)
                times = [1.0, move_time]
                res = list(res)
                res[4:8] = current_arm_joint_state[4:8]
                q_frames = [current_arm_joint_state[0:8], res]
                self.robot.control_arm_joint_trajectory(times, q_frames)
                time.sleep(move_time + 1.0)
                # self.set_auto_swing_arm_mode()
                # time.sleep(1.0)
        elif hand_type == "right":
            front_left = trans_l      # Left arm current position
            front_right = [x, y, z]  # Right arm target position
            # Convert right arm Euler angles to quaternion
            r_front_orientation = tf.transformations.quaternion_from_euler(yaw, pitch, roll)
            l_front_orientation = rot_l
            res = self.robot.arm_ik_free(
                KuavoPose(position=front_left, orientation=l_front_orientation),
                KuavoPose(position=front_right, orientation=r_front_orientation)
            )

            if res:
                # Calculate the spatial distance between current and target end-effector positions
                current_left_pos = trans_l
                current_right_pos = trans_r
                target_left_pos = front_left
                target_right_pos = front_right
                move_time = self.slove_move_time(current_right_pos, target_right_pos)
                times = [1.0, move_time]
                res = list(res)
                res[0:4] = current_arm_joint_state[0:4]
                q_frames = [current_arm_joint_state[0:8], res]
                self.robot.control_arm_joint_trajectory(times, q_frames)
                time.sleep(move_time + 1.0)
        return True

    def play_music(self, music_number: str, volume: int = 100, speed: float = 1.0) -> bool:
        """Play a music file.

        Args:
            music_number (str): The number of the music file to play.
            volume (int, optional): The volume of the music. Defaults to 100.
            speed (float, optional): The speed of the music. Defaults to 1.0.

        Returns:
            bool: True if the music was played successfully, False otherwise.
        """
        return self.robot_audio.play_audio(music_number, volume, speed)
    
    def stop_music(self) -> bool:
        """Stop the currently playing music.

        Returns:
            bool: True if the music was stopped successfully, False otherwise.
        """
        return self.robot_audio.stop_music()

    # ===== Stair Climbing Methods =====
    
    def set_stair_parameters(
        self,
        step_height: float = 0.13,
        step_length: float = 0.28,
        foot_width: float = 0.10,
        stand_height: float = 0.0,
        dt: float = 0.6,
        ss_time: float = 0.5,
    ) -> bool:
        """Set stair climbing parameters with version-specific defaults.

        Args:
            step_height (float): Step height in meters. Defaults to 0.13.
            step_length (float): Step length in meters. Defaults to 0.28.
            foot_width (float): Foot width in meters. Defaults to 0.10.
            stand_height (float): Standing height offset in meters. Defaults to 0.0.
            dt (float): Gait cycle time in seconds. Defaults to 0.6.
            ss_time (float): Single support time ratio. Defaults to 0.5.

        Returns:
            bool: True if parameters were set successfully, False otherwise.
        """
        try:
            # Set default parameters based on robot version
            if robot_version >= 40:
                step_height = step_height or 0.13
                step_length = step_length or 0.28
                foot_width = foot_width or 0.10
                stand_height = stand_height or 0.0
                dt = dt or 0.6
                ss_time = ss_time or 0.5
            else:
                step_height = step_height or 0.08
                step_length = step_length or 0.25
                foot_width = foot_width or 0.10
                stand_height = stand_height or -0.02
                dt = dt or 1.0
                ss_time = ss_time or 0.6
            
            return self.climb_stair.set_stair_parameters(
                step_height=step_height,
                step_length=step_length,
                foot_width=foot_width,
                stand_height=stand_height,
                dt=dt,
                ss_time=ss_time,
            )
        except Exception as e:
            print(f"Set stair parameters failed: {str(e)}")
            return False

    def climb_up_stairs(self, num_steps: int = 4, stair_offset: float = 0.03) -> bool:
        """Plan and add up stairs trajectory to accumulated trajectory.

        Args:
            num_steps (int): Number of steps to climb up. Defaults to 4.

        Returns:
            bool: True if planning was successful, False otherwise.
        """
        try:
            return self.climb_stair.climb_up_stairs(num_steps, stair_offset)
        except Exception as e:
            print(f"Climb up stairs failed: {str(e)}")
            return False

    def climb_down_stairs(self, num_steps: int = 5) -> bool:
        """Plan and add down stairs trajectory to accumulated trajectory.
        
        Note: This functionality is currently disabled.

        Args:
            num_steps (int): Number of steps to climb down. Defaults to 5.

        Returns:
            bool: False (functionality disabled).
        """
        print("⚠ Down stairs functionality is currently disabled (under development)")
        return self.climb_stair.climb_down_stairs(num_steps)

    def stair_move_to_position(
        self,
        dx: float = 0.2,
        dy: float = 0.0,
        dyaw: float = 0.0,
        max_step_x: float = 0.28,
        max_step_y: float = 0.15,
        max_step_yaw: float = 30.0,
    ) -> bool:
        """Plan stair climbing move to position trajectory and add to accumulated trajectory.

        Args:
            dx (float): X direction displacement in meters. Defaults to 0.2.
            dy (float): Y direction displacement in meters. Defaults to 0.0.
            dyaw (float): Yaw angle displacement in degrees. Defaults to 0.0.
            max_step_x (float): Maximum step size in X direction. Defaults to 0.28.
            max_step_y (float): Maximum step size in Y direction. Defaults to 0.15.
            max_step_yaw (float): Maximum yaw step size in degrees. Defaults to 30.0.

        Returns:
            bool: True if planning was successful, False otherwise.
        """
        try:
            return self.climb_stair.move_to_position(
                dx=dx, dy=dy, dyaw=dyaw, max_step_x=max_step_x, max_step_y=max_step_y, max_step_yaw=max_step_yaw
            )
        except Exception as e:
            print(f"Stair move to position failed: {str(e)}")
            return False

    def execute_stair_trajectory(self) -> bool:
        """Execute the complete accumulated stair climbing trajectory.

        Returns:
            bool: True if execution was successful, False otherwise.
        """
        try:
            return self.climb_stair.execute_trajectory()
        except Exception as e:
            print(f"Execute stair trajectory failed: {str(e)}")
            return False

    def simple_up_stair(self, stair_height = 0.25,stair_length = 0.08,stair_num = 4):
        """
        生成简单的上楼梯轨迹
        Args:
            stair_height: 楼梯高度
            stair_length: 楼梯长度
            stair_num: 楼梯级数
        Returns:
            bool: True if successful, False otherwise.
        """
        try:
            return self.climb_stair.simple_up_stairs(stair_height, stair_length, stair_num)
        except Exception as e:
            print(f"Simple up stair failed: {str(e)}")
            return False

    def align_stair(self) -> bool:
        """对齐楼梯，基于视觉 tag 识别控制机器人单步运动到楼梯前方固定的点和朝向。

        从配置文件读取对齐参数（offset_x, offset_y, offset_yaw, tag_id），
        如果配置文件不存在，使用默认值：offset_x=0.80, offset_y=0.30, offset_yaw=0.00

        Returns:
            bool: True if alignment was successful, False otherwise.
        """
        try:
            return self.climb_stair.simple_align_stair()
        except Exception as e:
            print(f"Align stair failed: {str(e)}")
            return False
        
    def control_waist_rotation(self, degree: float = 0):
        """Control the robot's waist rotation.

        Args:
            degree (float): Rotation angle in degrees.
        """
        try:
            global control_waist_pub
            msg = robotWaistControl()
            msg.header.stamp = rospy.Time.now()
            msg.data.data = [degree]
            control_waist_pub.publish(msg)

        except Exception as e:
            print(f"Robot waist control failed: {str(e)}")

    def alignment_target(self, class_name: str, confidence: float = 0.5, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        """
        使机器人对准指定类别到对象，并移动到目标前
        :param class_name: yolo 检测中需要检测的对象名称，和训练模型中的类别名称一致
        :param confidence: yolo 检测置信度，默认 0.5
        :param x: 图像 x 方向的偏移值，物体中心的 x 小于该参数 -x 时，机器人左移，大于该参数 x 时，机器人右移
        :param y: 图像 y 方向的偏移值，物体中心的 y 小于该参数 y 时，机器人前进，否则停止移动
        :param z: 机器人上下蹲的高度控制
        """
        try:
            # 加载 yolo 模型
            yolo_detection = model_utils.yolo_detection
            model = model_utils.model
            is_yolo_init = model_utils.is_yolo_init

            if  not is_yolo_init:
                yolo_detection = YOLO_detection()
                yolo_detection.init_ros_node()
                caller_file_path = self.package_path + "/upload_files/"
                print(f"caller_file_path: {caller_file_path}")
                model_path = os.path.join(os.path.dirname(caller_file_path), 'best.pt')
                # print(f"函数被文件调用: {model_path}")
                model = yolo_detection.load_model(model_path)
                result = yolo_detection.camera_interface.get_camera_image("head")
                if result is None:
                    print("No head camera image...")
                    return
            yolo_detection.set_conf( confidence)

            IMAGE_CENTER_X = yolo_detection.camera_interface.cv_image_shape[1] / 2.0
            IMAGE_CENTER_Y = yolo_detection.camera_interface.cv_image_shape[0] / 2.0
            # rospy.loginfo(f"IMAGE_CENTER_X: {IMAGE_CENTER_X}")
            # rospy.loginfo(f"IMAGE_CENTER_Y: {IMAGE_CENTER_Y}")
            MOVE_SPEED_Y = 0.05  # Y方向移动速度
            MOVE_SPEED_X = 0.10  # X方向移动速度

            while not rospy.is_shutdown():

                results = yolo_detection.get_detections("head", model)
                if not results:
                    continue

                # 存储所有符合条件的目标信息
                targets = []
                for result in results:

                    boxes = result.boxes.cpu().numpy()  # 边界框数据转NumPy数组
                    xywh = boxes.xywh  # [中心x, 中心y, 宽, 高]
                    class_names = [result.names[cls.item()] for cls in result.boxes.cls.int()]  # 类别名称列表

                    # 遍历每个检测框，提取目标信息
                    for i in range(len(xywh)):
                        if class_names[i] == class_name:  # 筛选目标类别
                            targets.append({
                                "class": class_names[i],
                                "x": xywh[i][0],  # 目标中心x坐标
                                "y": xywh[i][1],  # 目标中心y坐标
                                "area": xywh[i][2] * xywh[i][3]  # 目标面积（宽×高）
                            })

                if not targets:
                    continue  # 无目标时继续循环

                # 选择面积最大的目标
                best_target = max(targets, key=lambda t: t["area"])

                # 计算相对于图像中心的偏移量
                offset_x = best_target["x"] - IMAGE_CENTER_X
                offset_y = best_target["y"] - IMAGE_CENTER_Y

                # rospy.loginfo(f"目标中心偏移: x={offset_x:.2f}, y={offset_y:.2f}")

                # 根据偏移量控制机器人移动
                if abs(offset_x) > x:
                    # X方向偏移过大，左右移动调整
                    speed_y = -MOVE_SPEED_Y if offset_x > x else MOVE_SPEED_Y
                    self.walk_angle(0.00, speed_y, 0.00)
                elif offset_y < y:
                    # Y方向偏移过大，向前移动调整
                    self.walk_angle(MOVE_SPEED_X, 0.00, 0.00)
                else:
                    # 偏移量在可接受范围内，停止移动
                    self.stop()

                    # 控制机器人高度
                    if z >= 0:
                        self.control_robot_height("up", z)
                    else:
                        self.control_robot_height("down", -z)

                    break

                time.sleep(0.1)

        except Exception as e:
            print(f"Robot alignment to target failed: {str(e)}")
