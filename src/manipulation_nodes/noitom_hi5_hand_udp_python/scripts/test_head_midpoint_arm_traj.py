#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""不启动 VR，让双手走出不同轨迹，用来看头跟双手中点。

pipeline=auto 时看 /vr_ik/use_velocity_ik_reference：
全身 IK 机器走 /mm/teleop_tcp_target，普通轮臂走 /kuavo_arm_traj。

全身 IK：

    roslaunch wheel_arm_abs_vel load_kuavo_mujoco_velocity_ik.launch \\
        robot_version:=400062 head_control_mode:=track_hand \\
        head_track_target:=both_hands_midpoint
    roslaunch wheel_arm_abs_vel quest_velocity_ik.launch robot_version:=400062
    roslaunch noitom_hi5_hand_udp_python test_head_midpoint_arm_traj.launch

普通轮臂把 pipeline 设成 joint。两种路径都会先切到手臂外部控制（mode 2），
结束或 Ctrl-C 时回到起始姿态并切回保持（mode 0）。
"""

import math
import time

import rospy
import tf
from kuavo_msgs.msg import sensorsData
from kuavo_msgs.srv import changeArmCtrlMode
from sensor_msgs.msg import JointState


# 每只手 7 个关节，单位度。幅度留在轮臂关节限位内。
# 左手走前伸再抬到外侧，每一段更长，所以更慢。
# 右手在身侧来回扫，路点更多、每一段更短，所以更快，轨迹也不和左手对称。
_LEFT_WAYPOINTS_DEG = [
    [-15.0, 30.0, 5.0, -40.0, 0.0, 0.0, 0.0],
    [-60.0, 15.0, -15.0, -90.0, 0.0, 0.0, 0.0],
    [-20.0, 60.0, 30.0, -35.0, 0.0, 0.0, 0.0],
]
_RIGHT_WAYPOINTS_DEG = [
    [-10.0, -20.0, 10.0, -30.0, 0.0, 0.0, 0.0],
    [-25.0, -65.0, -30.0, -50.0, 0.0, 0.0, 0.0],
    [-40.0, -25.0, 20.0, -70.0, 0.0, 0.0, 0.0],
    [-15.0, -45.0, -10.0, -40.0, 0.0, 0.0, 0.0],
]

# 全身 IK 的手目标相对启动姿态的位移，米，腰坐标系。
# 轮臂默认下垂、臂展约 0.71 m，位移要抬到胸前才看得出来，并留在可达半径内。
# 左手慢，先伸到前上方再摆到外侧；右手快，在身侧和前方之间来回扫。
_LEFT_OFFSETS_M = [
    [0.30, 0.08, 0.55],
    [0.20, 0.30, 0.75],
    [0.38, 0.10, 0.30],
]
_RIGHT_OFFSETS_M = [
    [0.15, -0.30, 0.35],
    [0.32, -0.08, 0.70],
    [0.10, -0.28, 0.18],
    [0.35, -0.22, 0.55],
]

_JOINT_NAMES = ["arm_joint_%d" % index for index in range(1, 15)]


def _smoothstep(alpha):
    alpha = max(0.0, min(1.0, alpha))
    return 0.5 - 0.5 * math.cos(math.pi * alpha)


def _lerp(start, end, alpha):
    return [a + (b - a) * alpha for a, b in zip(start, end)]


class HeadMidpointArmTrajTest:
    def __init__(self):
        rospy.init_node("test_head_midpoint_arm_traj")
        self.rate_hz = float(rospy.get_param("~rate", 50.0))
        self.left_segment_sec = float(rospy.get_param("~left_segment_sec", 4.0))
        self.right_segment_sec = float(rospy.get_param("~right_segment_sec", 1.5))
        self.loops = int(rospy.get_param("~loops", 0))
        self.arm_mode_service = rospy.get_param("~arm_mode_service", "/change_arm_ctrl_mode")
        self.velocity_ik = self._select_pipeline(rospy.get_param("~pipeline", "auto"))
        self.torso_frame = rospy.get_param("~torso_frame", "waist_yaw_link")
        self.current_deg = None
        self._mode_client = None
        self._tcp_seq = 0
        if self.velocity_ik:
            from wheel_arm_abs_vel.msg import TeleopTcpTarget
            self._tcp_type = TeleopTcpTarget
            self.listener = tf.TransformListener()
            self.tcp_pub = rospy.Publisher("/mm/teleop_tcp_target", TeleopTcpTarget, queue_size=2)
            rospy.loginfo("Head midpoint test uses whole-body IK target /mm/teleop_tcp_target")
        else:
            rospy.Subscriber("/sensors_data_raw", sensorsData, self._on_sensors, queue_size=1)
            self.pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=2)
            rospy.loginfo("Head midpoint test uses joint trajectory /kuavo_arm_traj")

    @staticmethod
    def _select_pipeline(pipeline):
        name = str(pipeline or "auto").strip().lower()
        if name in ("velocity_ik", "whole_body_ik", "ik"):
            return True
        if name == "joint":
            return False
        return bool(rospy.get_param("/vr_ik/use_velocity_ik_reference", False))

    def _on_sensors(self, msg):
        joint_q = msg.joint_data.joint_q
        if len(joint_q) < 16:
            return
        # 头两个关节在最后，手臂是它前面的 14 个，单位弧度。
        arm_rad = joint_q[-16:-2]
        self.current_deg = [math.degrees(value) for value in arm_rad]

    def _set_arm_mode(self, mode):
        if self._mode_client is None:
            rospy.wait_for_service(self.arm_mode_service, timeout=10.0)
            self._mode_client = rospy.ServiceProxy(self.arm_mode_service, changeArmCtrlMode)
        response = self._mode_client(control_mode=mode)
        if not response.result:
            raise RuntimeError("change arm mode %s failed: %s" % (mode, response.message))
        rospy.loginfo("Arm control mode set to %s (%s)", mode, response.message)

    def _publish(self, position_deg, velocity_deg_s):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(_JOINT_NAMES)
        msg.position = list(position_deg)
        msg.velocity = list(velocity_deg_s)
        self.pub.publish(msg)

    def _step_period(self):
        return 1.0 / (self.rate_hz if self.rate_hz > 0.0 else 50.0)

    def _move(self, start_deg, end_deg, duration_sec):
        steps = max(1, int(duration_sec * self.rate_hz))
        previous = list(start_deg)
        dt = self._step_period()
        for step in range(1, steps + 1):
            alpha = _smoothstep(float(step) / float(steps))
            position = _lerp(start_deg, end_deg, alpha)
            velocity = [(position[index] - previous[index]) / dt for index in range(14)]
            self._publish(position, velocity)
            previous = position
            time.sleep(dt)
        return previous

    def _wait_for_arm_state(self):
        rospy.loginfo("Waiting for /sensors_data_raw arm joints")
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        while not rospy.is_shutdown() and self.current_deg is None and rospy.Time.now() < deadline:
            rospy.sleep(0.05)
        if self.current_deg is None:
            raise RuntimeError("No arm joints in /sensors_data_raw; start the robot first")
        return list(self.current_deg)

    def _lookup_pose(self, child):
        self.listener.waitForTransform(self.torso_frame, child, rospy.Time(0), rospy.Duration(10.0))
        translation, rotation = self.listener.lookupTransform(self.torso_frame, child, rospy.Time(0))
        return list(translation), list(rotation)

    def _capture_tcp_sources(self):
        """把当前末端换成全身 IK 接收的肩部锚点坐标，避免一启动就甩手。"""
        shoulder_x = float(rospy.get_param("/quest3/base_chest_offset_x", 0.0))
        shoulder_z = float(rospy.get_param("/quest3/base_height_offset", 0.42))
        shoulder_y = float(rospy.get_param("/quest3/shoulder_width", 0.15))
        captured = []
        for hand, shoulder, anchor_y in (
            ("zarm_l7_end_effector", "zarm_l1_link", shoulder_y),
            ("zarm_r7_end_effector", "zarm_r1_link", -shoulder_y),
        ):
            hand_pos, hand_quat = self._lookup_pose(hand)
            shoulder_pos, _shoulder_quat = self._lookup_pose(shoulder)
            anchor = [shoulder_x, anchor_y, shoulder_z]
            source = [hand_pos[index] - shoulder_pos[index] + anchor[index] for index in range(3)]
            captured.append((source, hand_quat))
        return captured

    def _publish_tcp(self, left_pos, right_pos, left_quat, right_quat):
        message = self._tcp_type()
        now = rospy.Time.now()
        message.header.seq = self._tcp_seq
        self._tcp_seq = (self._tcp_seq + 1) & 0xFFFFFFFF
        message.header.stamp = now
        message.header.frame_id = "quest_chest_robot_axes"
        message.left_controlled_frame = "left_teleop_tcp"
        message.right_controlled_frame = "right_teleop_tcp"
        message.source = self._tcp_type.SOURCE_QUEST_CONTROLLER
        message.control_state = self._tcp_type.CONTROL_ACTIVE
        message.control_valid = True
        message.left_valid = True
        message.right_valid = True
        message.session_id = 1
        message.generation = 1
        message.body_coordinate_epoch = 1
        message.body_tracking_frame = "quest_tracking_robot_axes"
        message.body_high_confidence = True
        message.chest_valid = True
        message.chest_pose.orientation.w = 1.0
        message.mapper_publish_stamp = now
        message.mapper_receive_stamp = now
        message.mapper_processing_time = 0.0
        message.source_relative_age = 0.0
        for pose, position, quaternion in (
            (message.left_tcp_pose, left_pos, left_quat),
            (message.right_tcp_pose, right_pos, right_quat),
        ):
            pose.position.x, pose.position.y, pose.position.z = position
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        self.tcp_pub.publish(message)

    def _spin_velocity_ik(self):
        if self.tcp_pub.get_num_connections() == 0:
            rospy.sleep(1.0)
        if self.tcp_pub.get_num_connections() == 0:
            raise RuntimeError(
                "No subscriber on /mm/teleop_tcp_target. Start quest_velocity_ik.launch "
                "so absolute_velocity_ik is running."
            )
        self._set_arm_mode(2)
        try:
            time.sleep(0.2)
            (left_source, left_quat), (right_source, right_quat) = self._capture_tcp_sources()
        except Exception:
            try:
                self._set_arm_mode(0)
            except Exception as error:
                rospy.logwarn("Failed to restore arm mode 0: %s", error)
            raise
        rate_hz = self.rate_hz if self.rate_hz > 0.0 else 50.0
        left = _HandSwing([0.0, 0.0, 0.0], _LEFT_OFFSETS_M, self.left_segment_sec, rate_hz)
        right = _HandSwing([0.0, 0.0, 0.0], _RIGHT_OFFSETS_M, self.right_segment_sec, rate_hz)
        rospy.loginfo(
            "Publishing whole-body IK hand targets. left %.2fs, right %.2fs. Ctrl-C returns to the start pose.",
            self.left_segment_sec,
            self.right_segment_sec,
        )
        rate = rospy.Rate(rate_hz)
        left_cycles = 0
        try:
            while not rospy.is_shutdown():
                left_offset, _left_vel, left_cycle = left.advance()
                right_offset, _right_vel, _right_cycle = right.advance()
                self._publish_tcp(
                    [left_source[index] + left_offset[index] for index in range(3)],
                    [right_source[index] + right_offset[index] for index in range(3)],
                    left_quat,
                    right_quat,
                )
                if left_cycle:
                    left_cycles += 1
                    if self.loops > 0 and left_cycles >= self.loops:
                        break
                rate.sleep()
        finally:
            rospy.loginfo("Returning hands to the captured start pose")
            self._return_tcp_offsets(
                left.pose, right.pose, left_source, right_source, left_quat, right_quat
            )
            try:
                self._set_arm_mode(0)
            except Exception as error:
                rospy.logwarn("Failed to restore arm mode 0: %s", error)

    def _return_tcp_offsets(self, left_offset, right_offset, left_source, right_source, left_quat, right_quat):
        duration = max(self.left_segment_sec, self.right_segment_sec)
        steps = max(1, int(duration * self.rate_hz))
        for step in range(1, steps + 1):
            alpha = _smoothstep(float(step) / float(steps))
            left_now = _lerp(left_offset, [0.0, 0.0, 0.0], alpha)
            right_now = _lerp(right_offset, [0.0, 0.0, 0.0], alpha)
            self._publish_tcp(
                [left_source[index] + left_now[index] for index in range(3)],
                [right_source[index] + right_now[index] for index in range(3)],
                left_quat,
                right_quat,
            )
            time.sleep(self._step_period())

    def spin(self):
        if self.velocity_ik:
            self._spin_velocity_ik()
            return
        start_deg = self._wait_for_arm_state()
        self._set_arm_mode(2)
        rate_hz = self.rate_hz if self.rate_hz > 0.0 else 50.0
        left = _HandSwing(start_deg[:7], _LEFT_WAYPOINTS_DEG, self.left_segment_sec, rate_hz)
        right = _HandSwing(start_deg[7:], _RIGHT_WAYPOINTS_DEG, self.right_segment_sec, rate_hz)
        rospy.loginfo(
            "Publishing arm trajectory. left %.2fs x %d poses, right %.2fs x %d poses. Ctrl-C returns to the start pose.",
            self.left_segment_sec,
            len(_LEFT_WAYPOINTS_DEG),
            self.right_segment_sec,
            len(_RIGHT_WAYPOINTS_DEG),
        )
        rate = rospy.Rate(rate_hz)
        left_cycles = 0
        try:
            while not rospy.is_shutdown():
                left_pos, left_vel, left_cycle = left.advance()
                right_pos, right_vel, _right_cycle = right.advance()
                self._publish(left_pos + right_pos, left_vel + right_vel)
                if left_cycle:
                    left_cycles += 1
                    if self.loops > 0 and left_cycles >= self.loops:
                        break
                rate.sleep()
        finally:
            rospy.loginfo("Returning arms to the captured start pose")
            self._move(left.pose + right.pose, start_deg, max(self.left_segment_sec, self.right_segment_sec))
            try:
                self._set_arm_mode(0)
            except Exception as error:
                rospy.logwarn("Failed to restore arm mode 0: %s", error)


class _HandSwing:
    """一只手按自己的路点和段时长循环。段越短，同样的关节位移速度越大。"""

    def __init__(self, start, waypoints, segment_sec, rate_hz):
        self._waypoints = [list(point) for point in waypoints]
        self._from = list(start)
        self.pose = list(start)
        self._index = 0
        self._steps = max(1, int(max(0.05, segment_sec) * rate_hz))
        self._step = 0
        self._dt = 1.0 / rate_hz

    def advance(self):
        self._step += 1
        cycle_done = False
        if self._step > self._steps:
            self._from = list(self.pose)
            next_index = (self._index + 1) % len(self._waypoints)
            cycle_done = next_index == 0
            self._index = next_index
            self._step = 1
        alpha = _smoothstep(float(self._step) / float(self._steps))
        position = _lerp(self._from, self._waypoints[self._index], alpha)
        velocity = [(position[index] - self.pose[index]) / self._dt for index in range(len(position))]
        self.pose = position
        return position, velocity, cycle_done


if __name__ == "__main__":
    try:
        HeadMidpointArmTrajTest().spin()
    except rospy.ROSInterruptException:
        pass
