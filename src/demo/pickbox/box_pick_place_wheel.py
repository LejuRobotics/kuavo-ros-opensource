#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import math
import os
import sys

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

try:
    from apriltag_ros.msg import AprilTagDetectionArray
except ImportError:
    from kuavo_msgs.msg import AprilTagDetectionArray

from kuavo_msgs.msg import ikSolveParam
from kuavo_msgs.msg import robotHeadMotionData
from kuavo_msgs.msg import sensorsData
from kuavo_msgs.msg import twoArmHandPoseCmd
from kuavo_msgs.srv import twoArmHandPoseCmdSrv
from kuavo_msgs.srv import twoArmHandPoseCmdSrvRequest

from box_pick_place import BoxPickPlace as BaseBoxPickPlace
from box_pick_place import BoxPickPlaceError

SCRIPT_ROLE = "wheel_pick_place_flow"


# 轮臂工作空间限制，
safe_space_limits = {
    "x": (0.20, 0.70),
    "y": (-0.25, 0.25),
    "z": (0.08, 0.60),
}
QR_ALIGNMENT_Y_TOLERANCE = 0.05
QR_ALIGNMENT_MAX_PASSES = 3

QR_DETECTION_FRAME = "waist_yaw_link"
WALK_QR_FRAME = "base_link"


class BoxPickPlaceWheel(BaseBoxPickPlace):
    def __init__(self, yaml_path):
        super(BoxPickPlaceWheel, self).__init__(yaml_path)

    def init_ros(self):
        rospy.init_node("box_pick_place_wheel", anonymous=False)

        qr_cfg = self.params["qr"]
        self.tag_sub = rospy.Subscriber(
            qr_cfg["detection_topic"],
            AprilTagDetectionArray,
            self.qr_callback,
            queue_size=10,
        )
        self.sensors_sub = rospy.Subscriber(
            "/sensors_data_raw",
            sensorsData,
            self.sensors_callback,
            queue_size=10,
        )
        self.head_pub = rospy.Publisher(
            "/robot_head_motion_data", robotHeadMotionData, queue_size=10
        )
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.arm_traj_pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        ik_name = "/ik/two_arm_hand_pose_cmd_srv_muli_refer"
        rospy.loginfo("等待双臂IK服务 %s", ik_name)
        rospy.wait_for_service(ik_name)
        self.ik_srv = rospy.ServiceProxy(ik_name, twoArmHandPoseCmdSrv)
        self.ik_service_name = ik_name

        self._wait_for_publishers()
        rospy.loginfo(
            "box_pick_place_wheel 初始化完成"
        )
        rospy.loginfo(
            "轮臂掌心关节偏置 use_palm_joint_bias=%s",
            bool(self.params.get("grasp", {}).get("use_palm_joint_bias", True)),
        )

    def _wait_for_publishers(self):
        super(BoxPickPlaceWheel, self)._wait_for_publishers()

    def set_arm_external_control(self):
        rospy.loginfo("轮臂跳过手臂模式切换")
        return True

    def set_arm_default_control(self):
        rospy.loginfo("轮臂无 /arm_traj_change_mode，跳过恢复默认手臂模式")
        return True

    def set_arm_control_mode(self, mode, label):
        rospy.loginfo("轮臂无 /arm_traj_change_mode，跳过手臂模式切换: %s", label)
        return True
    def enable_base_pitch_limit(self, enable=True):
        rospy.loginfo("轮臂，跳过设置 enable=%s", bool(enable))
        return True, "wheel no-op"

    def disable_base_pitch_limit(self):
        rospy.loginfo("轮臂无 base_pitch_limit 接口，跳过关闭")
        return True

    def run(self):
        self._normalize_wheel_palm_joint_bias_param()
        return super(BoxPickPlaceWheel, self).run()

    def _normalize_wheel_palm_joint_bias_param(self):
        grasp_cfg = self.params.setdefault("grasp", {})
        raw_value = grasp_cfg.get("use_palm_joint_bias", True)
        normalized = self._as_bool(raw_value, default=True)
        grasp_cfg["use_palm_joint_bias"] = normalized
        rospy.loginfo(
            "轮臂run使用掌心关节偏置 use_palm_joint_bias=%s (raw=%s)",
            normalized,
            raw_value,
        )

    @staticmethod
    def _as_bool(value, default=True):
        if isinstance(value, bool):
            return value
        if value is None:
            return bool(default)
        if isinstance(value, (int, float)):
            return bool(value)
        text = str(value).strip().lower()
        if text in ("1", "true", "yes", "y", "on"):
            return True
        if text in ("0", "false", "no", "n", "off"):
            return False
        rospy.logwarn(
            "无法解析布尔配置 use_palm_joint_bias=%s，使用默认值 %s",
            value,
            bool(default),
        )
        return bool(default)

    def stance(self):
        rospy.loginfo("轮臂停止底盘并等待稳定")
        self._stop_cmd_vel()
        rospy.sleep(0.5)

    def walk_to(self, qr, approach_distance, use_qr_y_offset=True):
        walk_qr = self._qr_for_base_link_walk(qr)
        move_x, move_y, relative_yaw = self._approach_target_from_qr_pose(
            walk_qr,
            approach_distance,
        )
        if not use_qr_y_offset:
            move_y = 0.0
        rospy.loginfo(
            "轮臂接近二维码: dx=%.3f dy=%.3f dyaw=%.3f approach=%.3f",
            move_x,
            move_y,
            relative_yaw,
            float(approach_distance),
        )
        if move_x <= 0.0 and abs(move_y) < 1e-3 and abs(relative_yaw) < 1e-3:
            rospy.loginfo("当前位置已满足接近距离要求，跳过行走")
            return

        current_x, current_y, current_yaw = self._get_robot_pose()
        if current_x is None:
            raise BoxPickPlaceError("无法获取 odom->base_link，不能执行轮臂闭环接近")

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        target_x = current_x + move_x * cos_yaw - move_y * sin_yaw
        target_y = current_y + move_x * sin_yaw + move_y * cos_yaw
        target_yaw = self._normalize_angle(current_yaw + relative_yaw)
        rospy.loginfo(
            "轮臂接近目标(odom): 当前=(x=%.3f, y=%.3f, yaw=%.1f°) "
            "base目标=(dx=%.3f, dy=%.3f, dyaw=%.1f°) odom目标=(x=%.3f, y=%.3f, yaw=%.1f°)",
            current_x,
            current_y,
            math.degrees(current_yaw),
            move_x,
            move_y,
            math.degrees(relative_yaw),
            target_x,
            target_y,
            math.degrees(target_yaw),
        )
        if not self._wheel_xy_velocity_walk_to_target(target_x, target_y, target_yaw):
            raise BoxPickPlaceError("轮臂接近二维码失败")

    def _wheel_xy_velocity_walk_to_target(self, target_x, target_y, target_yaw):
        linear_speed = float(self.params.get("walk", {}).get("linear_speed", 0.15))
        lateral_speed = linear_speed
        min_axis_speed = 0.06
        angular_speed = 0.25
        pos_tolerance = 0.05
        axis_tolerance = 0.02
        yaw_tolerance = math.radians(5.0)
        timeout = 30.0
        control_dt = 0.1
        start_time = rospy.Time.now()
        last_print_time = 0.0

        rospy.loginfo("  轮臂阶段: x/y 行走到目标位置...")
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                self._stop_cmd_vel()
                rospy.logwarn("    轮臂行走超时")
                return False

            current_x, current_y, current_yaw = self._get_robot_pose()
            if current_x is None:
                rospy.sleep(control_dt)
                continue

            dx_odom = target_x - current_x
            dy_odom = target_y - current_y
            cos_yaw = math.cos(current_yaw)
            sin_yaw = math.sin(current_yaw)
            err_x = cos_yaw * dx_odom + sin_yaw * dy_odom
            err_y = -sin_yaw * dx_odom + cos_yaw * dy_odom
            distance = math.sqrt(dx_odom * dx_odom + dy_odom * dy_odom)
            yaw_error = self._normalize_angle(target_yaw - current_yaw)

            if elapsed - last_print_time > 0.5:
                rospy.loginfo(
                    "    [轮臂行走] pos=(%.2f, %.2f) err_base=(%.3f, %.3f) "
                    "dist=%.2fm yaw_error=%.1f°",
                    current_x,
                    current_y,
                    err_x,
                    err_y,
                    distance,
                    math.degrees(yaw_error),
                )
                last_print_time = elapsed

            if distance < pos_tolerance and abs(yaw_error) < yaw_tolerance:
                self._stop_cmd_vel()
                rospy.loginfo("    轮臂行走到达目标 (距离=%.2fm)", distance)
                return True

            vx = self._planned_axis_speed(err_x, linear_speed, min_axis_speed, axis_tolerance)
            vy = self._planned_axis_speed(err_y, lateral_speed, min_axis_speed, axis_tolerance)
            vz = 0.0
            if abs(yaw_error) >= yaw_tolerance:
                vz = self._planned_turn_speed(yaw_error, angular_speed)

            self._publish_cmd_vel(vx, vy, vz)
            rospy.sleep(control_dt)

        raise rospy.ROSInterruptException()

    @classmethod
    def _planned_axis_speed(cls, error, max_abs, min_abs, tolerance):
        error = float(error)
        if abs(error) < float(tolerance):
            return 0.0
        speed = cls._clamp_speed(0.8 * error, max_abs)
        if abs(speed) < float(min_abs):
            return float(min_abs) if error > 0.0 else -float(min_abs)
        return speed

    @staticmethod
    def _clamp_speed(value, max_abs):
        return max(-float(max_abs), min(float(max_abs), float(value)))

    def scan_qr_after_walk(self, target_id, pitch_deg=24.0):
        qr = super(BoxPickPlaceWheel, self).scan_qr_after_walk(target_id, pitch_deg)
        place_qr_id = int(self.params.get("qr", {}).get("place_qr_id", -1))
        if int(target_id) == place_qr_id:
            return qr
        return self._align_qr_y_after_walk(target_id, pitch_deg, qr)

    def _enable_qr_y_alignment_after_walk(self):
        return False

    def _align_qr_y_after_walk(self, target_id, pitch_deg, qr):
        for pass_idx in range(1, QR_ALIGNMENT_MAX_PASSES + 1):
            y_error = float(qr["y"])
            if abs(y_error) <= QR_ALIGNMENT_Y_TOLERANCE:
                rospy.loginfo(
                    "轮臂二维码y对齐完成: ID=%s y=%.3f tolerance=%.3f pass=%d",
                    target_id,
                    y_error,
                    QR_ALIGNMENT_Y_TOLERANCE,
                    pass_idx - 1,
                )
                return qr

            adjust_speed = min(float(self.params.get("walk", {}).get("linear_speed", 0.15)), 0.12)
            adjust_timeout = max(1.0, min(4.0, abs(y_error) / max(adjust_speed, 1e-3) + 1.0))
            rospy.logwarn(
                "轮臂二维码y偏差过大: ID=%s y=%.3f tolerance=%.3f，执行第%d次底盘y向微调",
                target_id,
                y_error,
                QR_ALIGNMENT_Y_TOLERANCE,
                pass_idx,
            )
            if not self._cmd_vel_lateral_adjust(
                y_error,
                adjust_speed,
                adjust_timeout,
                pos_tolerance=min(QR_ALIGNMENT_Y_TOLERANCE * 0.5, 0.02),
                min_lateral_speed=0.035,
                log_label="轮臂二维码y",
            ):
                raise BoxPickPlaceError("轮臂二维码y向微调失败: ID=%s y=%.3f" % (target_id, y_error))
            qr = super(BoxPickPlaceWheel, self).scan_qr_after_walk(target_id, pitch_deg)

        if abs(float(qr["y"])) > QR_ALIGNMENT_Y_TOLERANCE:
            raise BoxPickPlaceError(
                "轮臂二维码y对齐失败: ID=%s y=%.3f tolerance=%.3f"
                % (target_id, float(qr["y"]), QR_ALIGNMENT_Y_TOLERANCE)
            )
        return qr

    def _approach_distance_for_qr(self, target_id):
        qr_cfg = self.params.get("qr", {})
        walk_cfg = self.params.get("walk", {})
        if int(target_id) == int(qr_cfg.get("place_qr_id", -1)):
            return float(walk_cfg.get("place_approach_distance", 0.35))
        return float(walk_cfg.get("pick_approach_distance", 0.30))

    def _qr_for_base_link_walk(self, qr):
        try:
            transform = self.tf_buffer.lookup_transform(
                WALK_QR_FRAME,
                QR_DETECTION_FRAME,
                rospy.Time(0),
                rospy.Duration(0.5),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            raise BoxPickPlaceError(
                "无法将二维码坐标从 %s 转到 %s，不能执行底盘接近: %s"
                % (QR_DETECTION_FRAME, WALK_QR_FRAME, exc)
            )

        trans = transform.transform.translation
        rot = transform.transform.rotation
        tf_quat = [float(rot.x), float(rot.y), float(rot.z), float(rot.w)]
        pos_base = self._transform_point(
            [float(qr["x"]), float(qr["y"]), float(qr["z"])],
            tf_quat,
            [float(trans.x), float(trans.y), float(trans.z)],
        )

        walk_qr = dict(qr)
        walk_qr["x"] = pos_base[0]
        walk_qr["y"] = pos_base[1]
        walk_qr["z"] = pos_base[2]
        walk_qr["yaw"] = self._normalize_angle(
            float(qr.get("yaw", 0.0)) + self._yaw_from_quat(rot)
        )
        if "quat" in qr:
            walk_qr["quat"] = self._quaternion_multiply(tf_quat, qr["quat"])
        rospy.loginfo(
            "轮臂底盘接近坐标转换: %s QR=(%.3f, %.3f, %.3f, yaw=%.3f) -> "
            "%s QR=(%.3f, %.3f, %.3f, yaw=%.3f)",
            QR_DETECTION_FRAME,
            float(qr["x"]),
            float(qr["y"]),
            float(qr["z"]),
            float(qr.get("yaw", 0.0)),
            WALK_QR_FRAME,
            walk_qr["x"],
            walk_qr["y"],
            walk_qr["z"],
            walk_qr["yaw"],
        )
        return walk_qr

    @classmethod
    def _transform_point(cls, point, quat_xyzw, translation):
        rotated = cls._rotate_vector_by_quat(point, quat_xyzw)
        return [
            rotated[0] + translation[0],
            rotated[1] + translation[1],
            rotated[2] + translation[2],
        ]

    @staticmethod
    def _rotate_vector_by_quat(vector, quat_xyzw):
        qx, qy, qz, qw = [float(value) for value in quat_xyzw]
        vx, vy, vz = [float(value) for value in vector]
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return [
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        ]

    @staticmethod
    def _quaternion_multiply(q1, q2):
        x1, y1, z1, w1 = [float(value) for value in q1]
        x2, y2, z2, w2 = [float(value) for value in q2]
        return [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]

    def check_safe_space(self, label, center):
        x, y, z = [float(value) for value in center]
        limits = {
            "x": (safe_space_limits["x"][0], safe_space_limits["x"][1], x),
            "y": (safe_space_limits["y"][0], safe_space_limits["y"][1], y),
            "z": (safe_space_limits["z"][0], safe_space_limits["z"][1], z),
        }
        violations = []
        for axis, (lower, upper, value) in limits.items():
            if value < lower or value > upper:
                violations.append("%s=%.3f not in [%.3f, %.3f]" % (axis, value, lower, upper))

        if violations:
            raise BoxPickPlaceError(
                "轮臂safe_space超限[%s]: center=(%.3f, %.3f, %.3f), %s"
                % (label, x, y, z, "; ".join(violations))
            )

        rospy.loginfo(
            "轮臂safe_space检查通过[%s]: center=(%.3f, %.3f, %.3f)",
            label,
            x,
            y,
            z,
        )

    def compute_grasp(self, qr):
        poses = super(BoxPickPlaceWheel, self).compute_grasp(qr)
        self._check_pose_targets_safe("抓取目标", poses)
        return poses

    def compute_place(self, qr):
        poses = super(BoxPickPlaceWheel, self).compute_place(qr)
        self._check_pose_targets_safe("放置目标", poses)
        return poses

    def _axis_has_violation(self, axis, values):
        lower, upper = safe_space_limits[axis]
        return any(value < lower or value > upper for value in values)

    def _check_pose_targets_safe(self, label, poses):
        targets = self._flatten_pose_targets(poses)
        self._log_pose_safe_space_status(label, targets)
        x_bad = self._axis_has_violation("x", [pos[0] for pos in targets])
        z_bad = self._axis_has_violation("z", [pos[2] for pos in targets])
        if x_bad:
            rospy.logwarn(
                "轮臂safe_space x 超限[%s]，当前不做躯干补偿",
                label,
            )
        if z_bad:
            raise BoxPickPlaceError(
                "轮臂safe_space z 超限[%s]: z_bad=%s" % (label, z_bad)
            )

    def _log_pose_safe_space_status(self, label, targets):
        xs = [pos[0] for pos in targets]
        ys = [pos[1] for pos in targets]
        zs = [pos[2] for pos in targets]
        rospy.loginfo(
            "轮臂safe_space目标范围[%s]: x=[%.3f, %.3f] y=[%.3f, %.3f] z=[%.3f, %.3f]",
            label,
            min(xs),
            max(xs),
            min(ys),
            max(ys),
            min(zs),
            max(zs),
        )

    @staticmethod
    def _flatten_pose_targets(poses):
        targets = []
        for _, pair in poses.items():
            targets.extend([pair[0], pair[1]])
        return targets

    def log_end_effector_targets(self, stage, poses):
        pass

    def ik_solve(self, left_xyz, right_xyz, label, q0_joints=None):
        ik_frame = int(self.params.get("wheel", {}).get("ik", {}).get("frame", 0))
        left_quat = [0.06163, -0.70442, -0.06163, 0.70442]
        right_quat = [-0.06163, -0.70442, 0.06163, 0.70442]
        left_elbow, right_elbow = self._ik_elbow_points()


        cmd = twoArmHandPoseCmd()
        cmd.use_custom_ik_param = True
        cmd.joint_angles_as_q0 = False
        cmd.ik_param = self._build_wheel_ik_param()
        cmd.frame = ik_frame
        cmd.hand_poses.left_pose.pos_xyz = list(left_xyz)
        cmd.hand_poses.left_pose.quat_xyzw = left_quat
        cmd.hand_poses.left_pose.elbow_pos_xyz = left_elbow
        cmd.hand_poses.right_pose.pos_xyz = list(right_xyz)
        cmd.hand_poses.right_pose.quat_xyzw = right_quat
        cmd.hand_poses.right_pose.elbow_pos_xyz = right_elbow

        if q0_joints is not None:
            if len(q0_joints) != 14:
                raise BoxPickPlaceError("IK初始关节角必须为14维，当前=%d" % len(q0_joints))
            cmd.joint_angles_as_q0 = True
            cmd.hand_poses.left_pose.joint_angles = [float(value) for value in q0_joints[:7]]
            cmd.hand_poses.right_pose.joint_angles = [float(value) for value in q0_joints[7:]]
            rospy.loginfo(
                "轮臂IK[%s] 使用关节种子(rad): L=%s R=%s",
                label,
                ["%.2f" % float(value) for value in q0_joints[:7]],
                ["%.2f" % float(value) for value in q0_joints[7:]],
            )

        req = twoArmHandPoseCmdSrvRequest()
        req.twoArmHandPoseCmdRequest = cmd
        rospy.loginfo(
            "请求轮臂IK[%s]: frame=%d left_quat=%s right_quat=%s L=%s R=%s elbow_L=%s elbow_R=%s service=%s",
            label,
            ik_frame,
            ["%.4f" % float(value) for value in left_quat],
            ["%.4f" % float(value) for value in right_quat],
            left_xyz,
            right_xyz,
            left_elbow,
            right_elbow,
            self.ik_service_name,
        )
        resp = self.ik_srv(req)
        self._log_ik_response(label, resp)
        if not getattr(resp, "success", False):
            reason = getattr(resp, "error_reason", "")
            raise BoxPickPlaceError("轮臂IK求解失败[%s]: %s" % (label, reason))

        chosen_joints = self._response_hand_pose_joints(resp)
        chosen_source = "hand_poses.joint_angles"
        if chosen_joints is None:
            q_arm_raw = list(getattr(resp, "q_arm", []))
            if len(q_arm_raw) == 14:
                chosen_joints = self.convert_ik_q_arm_for_traj(q_arm_raw)
                chosen_source = "q_arm"
        if chosen_joints is None:
            raise BoxPickPlaceError("轮臂IK返回关节数异常[%s]" % label)

        chosen_joints = [float(value) for value in chosen_joints]
        self.last_ik_raw_joints = self.convert_traj_joints_for_ik_seed(chosen_joints)
        rospy.loginfo(
            "轮臂IK求解成功[%s]: 耗时 %.2fms %s, 使用输出=%s",
            label,
            float(getattr(resp, "time_cost", 0.0)),
            getattr(resp, "error_reason", ""),
            chosen_source,
        )
        rospy.loginfo(
            "轮臂IK[%s] 采用关节(deg)=%s",
            label,
            ["%.3f" % float(value) for value in chosen_joints],
        )
        return chosen_joints, float(getattr(resp, "time_cost", 0.0))

    def move_arms_interpolated(self, target_joints, duration=4.0, steps=20, start_joints=None):
        return super(BoxPickPlaceWheel, self).move_arms_interpolated(
            target_joints,
            duration,
            steps,
            start_joints,
        )

    @staticmethod
    def _log_ik_response(label, resp):
        q_arm = [float(value) for value in list(getattr(resp, "q_arm", []))]
        q_torso = [float(value) for value in list(getattr(resp, "q_torso", []))]
        hand_poses = getattr(resp, "hand_poses", None)
        left_joints = []
        right_joints = []
        if hand_poses is not None:
            left_pose = getattr(hand_poses, "left_pose", None)
            right_pose = getattr(hand_poses, "right_pose", None)
            left_joints = [
                float(value)
                for value in list(getattr(left_pose, "joint_angles", []))
            ]
            right_joints = [
                float(value)
                for value in list(getattr(right_pose, "joint_angles", []))
            ]

        rospy.loginfo(
            "IK响应[%s]: success=%s with_torso=%s time_cost=%.2fms error_reason=%s",
            label,
            bool(getattr(resp, "success", False)),
            bool(getattr(resp, "with_torso", False)),
            float(getattr(resp, "time_cost", 0.0)),
            getattr(resp, "error_reason", ""),
        )
        rospy.loginfo(
            "IK响应[%s] q_arm(len=%d)=%s",
            label,
            len(q_arm),
            ["%.6f" % value for value in q_arm],
        )
        rospy.loginfo(
            "IK响应[%s] q_torso(len=%d)=%s",
            label,
            len(q_torso),
            ["%.6f" % value for value in q_torso],
        )
        rospy.loginfo(
            "IK响应[%s] hand_poses.left_pose.joint_angles(len=%d)=%s",
            label,
            len(left_joints),
            ["%.6f" % value for value in left_joints],
        )
        rospy.loginfo(
            "IK响应[%s] hand_poses.right_pose.joint_angles(len=%d)=%s",
            label,
            len(right_joints),
            ["%.6f" % value for value in right_joints],
        )

    @staticmethod
    def _build_wheel_ik_param():
        param = ikSolveParam()
        param.major_optimality_tol = 4e-3
        param.major_feasibility_tol = 4e-3
        param.minor_feasibility_tol = 4e-3
        param.major_iterations_limit = 100
        param.oritation_constraint_tol = 4e-3
        param.pos_constraint_tol = 4e-3
        param.pos_cost_weight = 1.0
        param.constraint_mode = 1
        return param

    @staticmethod
    def convert_traj_joints_for_ik_seed(joints):
        return [math.radians(float(value)) for value in joints]

    @staticmethod
    def _response_hand_pose_joints(resp):
        values = BoxPickPlaceWheel._raw_response_hand_pose_joints(resp)
        if values is None:
            return None
        return [math.degrees(v) for v in values]

    @staticmethod
    def _raw_response_hand_pose_joints(resp):
        hand_poses = getattr(resp, "hand_poses", None)
        if hand_poses is None:
            return None
        left = list(getattr(getattr(hand_poses, "left_pose", None), "joint_angles", []))
        right = list(getattr(getattr(hand_poses, "right_pose", None), "joint_angles", []))
        values = left + right
        if len(values) != 14:
            return None
        return [float(value) for value in values]


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Kuavo 轮臂夹箱搬运顺序控制")
    parser.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "wheel.yaml"),
        help="wheel.yaml 路径",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(rospy.myargv(argv=sys.argv)[1:])
    try:
        node = BoxPickPlaceWheel(args.config)
        return node.run()
    except BoxPickPlaceError as exc:
        try:
            rospy.logerr("初始化失败: %s", exc)
        except Exception:
            print("初始化失败: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
