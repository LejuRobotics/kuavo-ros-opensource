#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
import math
import os
import sys
import time

try:
    import yaml
except ImportError:
    yaml = None

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolRequest
from std_srvs.srv import Trigger

try:
    from apriltag_ros.msg import AprilTagDetectionArray
except ImportError:
    from kuavo_msgs.msg import AprilTagDetectionArray

from kuavo_msgs.msg import robotHeadMotionData
from kuavo_msgs.msg import sensorsData
from kuavo_msgs.msg import switchGaitByName
from kuavo_msgs.msg import twoArmHandPoseCmd
from kuavo_msgs.srv import changeArmCtrlMode
from kuavo_msgs.srv import changeArmCtrlModeRequest
from kuavo_msgs.srv import twoArmHandPoseCmdSrv
from kuavo_msgs.srv import twoArmHandPoseCmdSrvRequest


class BoxPickPlaceError(RuntimeError):
    pass


class BoxPickPlace(object):
    def __init__(self, yaml_path):
        self.params = self.load_params(yaml_path)
        self.latest_tags = {}
        self.last_arm_joints = None
        self.last_ik_raw_joints = None
        self.current_arm_joints_deg = None
        self.init_ros()

    def load_params(self, yaml_path):
        if yaml is None:
            raise BoxPickPlaceError("缺少 PyYAML，请先安装 python-yaml / python3-yaml")
        if not os.path.exists(yaml_path):
            raise BoxPickPlaceError("参数文件不存在: %s" % yaml_path)
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}
        self.apply_base_link_defaults(data)
        return data

    def apply_base_link_defaults(self, data):
        data.setdefault("qr", {})
        data.setdefault("walk", {})
        data.setdefault("box", {})
        data.setdefault("grasp", {})
        data["qr"]["detection_topic"] = "/robot_tag_info"
        data["box"].setdefault("length_x", 0.40)
        data["box"].setdefault("width_y", 0.30)
        data["box"].setdefault("height_z", 0.25)
        data["box"].setdefault("grasp_offset_x", 0.0)
        data["box"].setdefault("grasp_height_offset_z", -0.05)
        data["box"].setdefault("grasp_clamp_inset_y", 0.02)
        data["box"].setdefault(
            "front_qr_margins",
            {
                "left": float(data["box"]["width_y"]) / 2.0,
                "right": float(data["box"]["width_y"]) / 2.0,
                "top": float(data["box"]["height_z"]) / 2.0,
                "bottom": float(data["box"]["height_z"]) / 2.0,
            },
        )
        data["walk"].setdefault("linear_speed", 0.15)
        data["walk"].setdefault("pick_approach_distance", 0.01)
        data["walk"].setdefault("place_approach_distance", 0.01)
        data["grasp"].setdefault("motion_duration", 2.0)
        data["grasp"].setdefault("use_palm_joint_bias", True)

    def init_ros(self):
        rospy.init_node("box_pick_place_base_link", anonymous=False)

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
        self.gait_pub = rospy.Publisher(
            "/humanoid_switch_gait_by_name", switchGaitByName, queue_size=10
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("等待手臂控制模式服务 /arm_traj_change_mode")
        rospy.wait_for_service("/arm_traj_change_mode")
        self.arm_mode_srv = rospy.ServiceProxy("/arm_traj_change_mode", changeArmCtrlMode)
        self.base_pitch_limit_srv_name = "/humanoid/mpc/enable_base_pitch_limit"

        # IK 默认使用多 seed 服务，成功率更高；这是内部策略，不给客户配置。
        use_multi_seed_ik = True
        ik_name = (
            "/ik/two_arm_hand_pose_cmd_srv_muli_refer"
            if use_multi_seed_ik
            else "/ik/two_arm_hand_pose_cmd_srv"
        )
        rospy.loginfo("等待双臂IK服务 %s", ik_name)
        rospy.wait_for_service(ik_name)
        self.ik_srv = rospy.ServiceProxy(ik_name, twoArmHandPoseCmdSrv)
        self.ik_service_name = ik_name

        self._wait_for_publishers()
        rospy.loginfo("box_pick_place_base_link 初始化完成，二维码和IK均使用 base_link/local frame")

    def sensors_callback(self, msg):
        positions = list(getattr(getattr(msg, "joint_q", None), "position", []))
        if len(positions) < 26:
            return
        arm_rad = positions[12:19] + positions[19:26]
        self.current_arm_joints_deg = [math.degrees(float(value)) for value in arm_rad]

    def get_current_arm_joints_deg(self, timeout=1.0):
        if self.current_arm_joints_deg is not None:
            return list(self.current_arm_joints_deg)
        try:
            msg = rospy.wait_for_message("/sensors_data_raw", sensorsData, timeout=timeout)
            self.sensors_callback(msg)
        except Exception as exc:
            rospy.logwarn("获取当前手臂关节失败，将使用配置的复位姿态作为插值起点: %s", exc)
        if self.current_arm_joints_deg is None:
            return None
        return list(self.current_arm_joints_deg)

    def enable_base_pitch_limit(self, enable):
        try:
            rospy.wait_for_service(self.base_pitch_limit_srv_name, timeout=2.0)
            client = rospy.ServiceProxy(self.base_pitch_limit_srv_name, SetBool)
            req = SetBoolRequest()
            req.data = bool(enable)
            resp = client(req)
            if not getattr(resp, "success", False):
                rospy.logerr("设置 basePitch 限制失败: %s", getattr(resp, "message", ""))
                return False, getattr(resp, "message", "")
            return True, "success"
        except rospy.ROSException as exc:
            rospy.logwarn("等待 basePitch 限制服务超时: %s", exc)
            return False, str(exc)
        except rospy.ServiceException as exc:
            rospy.logwarn("调用 basePitch 限制服务失败: %s", exc)
            return False, str(exc)

    def disable_base_pitch_limit(self):
        rospy.loginfo("重要提示: 关闭 basePitch 限制...")
        for attempt in range(3):
            ok, message = self.enable_base_pitch_limit(False)
            if ok:
                rospy.loginfo("已关闭 basePitch 限制")
                return True
            if attempt < 2:
                rospy.loginfo("第%d次检查 basePitch 限制状态失败，准备重试: %s", attempt + 1, message)
                rospy.sleep(0.5)
        rospy.logwarn("关闭 basePitch 限制失败，继续执行...")
        return False

    def _wait_for_publishers(self):
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        pubs = [
            self.head_pub,
            self.cmd_vel_pub,
            self.arm_traj_pub,
        ]
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if all(pub.get_num_connections() > 0 for pub in pubs):
                return
            rospy.sleep(0.05)

    def qr_callback(self, msg):
        stamp = rospy.Time.now()
        for detection in msg.detections:
            tag_ids_raw = getattr(detection, "id", [])
            tag_ids = (
                list(tag_ids_raw)
                if isinstance(tag_ids_raw, (list, tuple))
                else [tag_ids_raw]
            )
            if not tag_ids:
                continue
            pose = detection.pose.pose.pose
            yaw = self._yaw_from_quat(pose.orientation)
            quat = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
            for tag_id in tag_ids:
                self.latest_tags[int(tag_id)] = {
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z,
                    "yaw": yaw,
                    "quat": quat,
                    "stamp": stamp,
                }

    def scan_qr(
        self,
        target_id,
        yaw_range=5.0,
        yaw_step=2.5,
        pitch_center=0.0,
        pitch_range=10.0,
        pitch_step=5.0,
        initial_yaw=None,
        initial_pitch=None,
    ):
        timeout = 15.0
        hold = 0.5
        pitch_center = float(pitch_center)
        initial_yaw = 0.0 if initial_yaw is None else float(initial_yaw)
        initial_pitch = pitch_center if initial_pitch is None else float(initial_pitch)
        scan_points = self._scan_head_points(
            yaw_range, yaw_step, pitch_center, pitch_range, pitch_step
        )
        start = rospy.Time.now()
        self.latest_tags.pop(int(target_id), None)
        self._publish_head(initial_yaw, initial_pitch)
        rospy.sleep(0.5)
        rospy.loginfo("开始扫描二维码 ID=%s，超时 %.1fs", target_id, timeout)

        idx = 0
        rate = rospy.Rate(max(1.0, 1.0 / max(hold, 0.05)))
        while not rospy.is_shutdown():
            cached = self.latest_tags.get(int(target_id))
            if cached is not None:
                age = (rospy.Time.now() - cached["stamp"]).to_sec()
                if age <= 1.0:
                    self._publish_head(0.0, pitch_center)
                    rospy.loginfo(
                        "扫描到二维码 ID=%s: x=%.3f y=%.3f z=%.3f yaw=%.3f",
                        target_id,
                        cached["x"],
                        cached["y"],
                        cached["z"],
                        cached["yaw"],
                    )
                    return cached

            if (rospy.Time.now() - start).to_sec() > float(timeout):
                self._publish_head(0.0, pitch_center)
                raise BoxPickPlaceError("二维码扫描超时: ID=%s" % target_id)

            yaw_deg, pitch_deg = scan_points[idx % len(scan_points)]
            self._publish_head(yaw_deg, pitch_deg)
            idx += 1
            rate.sleep()

        raise BoxPickPlaceError("ROS 已关闭，扫描二维码中断")

    def scan_qr_after_walk(self, target_id, pitch_deg=24.0):
        # 二次识别时先把头部固定到目标俯仰角，避免行走后头部仍保持搜索姿态。
        secondary_scan_head_yaw_deg = 0.0
        secondary_scan_head_pitch_deg = float(pitch_deg)
        secondary_scan_settle_time = 0.5
        rospy.loginfo(
            "二次识别二维码前调整头部: yaw=%.1f pitch=%.1f",
            secondary_scan_head_yaw_deg,
            secondary_scan_head_pitch_deg,
        )
        self._publish_head(secondary_scan_head_yaw_deg, secondary_scan_head_pitch_deg)
        rospy.sleep(secondary_scan_settle_time)
        qr = self.scan_qr(
            target_id,
            yaw_range=5.0,
            yaw_step=2.5,
            pitch_center=secondary_scan_head_pitch_deg,
            pitch_range=0.0,
            pitch_step=5.0,
            initial_yaw=secondary_scan_head_yaw_deg,
            initial_pitch=secondary_scan_head_pitch_deg,
        )
        place_qr_id = int(self.params.get("qr", {}).get("place_qr_id", -1))
        if int(target_id) == place_qr_id:
            rospy.loginfo(
                "放置阶段跳过二维码y微调: ID=%s y=%.3f",
                target_id,
                float(qr["y"]),
            )
            return qr
        if self._enable_qr_y_alignment_after_walk():
            return self._align_qr_y_after_walk(target_id, secondary_scan_head_pitch_deg, qr)
        return qr

    def _enable_qr_y_alignment_after_walk(self):
        return True

    def _align_qr_y_after_walk(self, target_id, pitch_deg, qr):
        y_tolerance = 0.05
        max_passes = 3
        max_single_step_y = 0.12
        min_adjust_y = 0.02
        adjust_speed = 0.08

        for pass_idx in range(1, max_passes + 1):
            y_error = float(qr["y"])
            if abs(y_error) <= y_tolerance:
                rospy.loginfo(
                    "二维码y对齐完成: ID=%s y=%.3f tolerance=%.3f pass=%d",
                    target_id,
                    y_error,
                    y_tolerance,
                    pass_idx - 1,
                )
                return qr

            step_y = max(-max_single_step_y, min(max_single_step_y, y_error))
            if abs(step_y) < min_adjust_y:
                step_y = min_adjust_y if step_y > 0.0 else -min_adjust_y
            duration = max(1.5, abs(step_y) / max(adjust_speed, 0.01))
            rospy.logwarn(
                "二维码y偏差较大: ID=%s y=%.3f tolerance=%.3f，"
                "第%d次使用/cmd_vel闭环微调y=%.3fm timeout=%.2fs",
                target_id,
                y_error,
                y_tolerance,
                pass_idx,
                step_y,
                duration,
            )
            if not self._cmd_vel_lateral_adjust(step_y, adjust_speed, duration):
                rospy.logwarn(
                    "二维码y闭环微调未确认到位: ID=%s target_y=%.3fm，继续重新扫描",
                    target_id,
                    step_y,
                )
            rospy.sleep(0.5)
            self._publish_head(0.0, pitch_deg)
            rospy.sleep(0.5)
            qr = self.scan_qr(
                target_id,
                yaw_range=5.0,
                yaw_step=2.5,
                pitch_center=float(pitch_deg),
                pitch_range=0.0,
                pitch_step=5.0,
                initial_yaw=0.0,
                initial_pitch=float(pitch_deg),
            )

        if abs(float(qr["y"])) > y_tolerance:
            rospy.logwarn(
                "二维码y多次微调后仍超差: ID=%s y=%.3f tolerance=%.3f，继续后续流程",
                target_id,
                float(qr["y"]),
                y_tolerance,
            )
        return qr

    def stance(self):
        """切换到 stance 站立 gait。

        先停止底盘速度，再发布 gait 切换指令，最后等待姿态稳定。
        """
        rospy.loginfo("  切换到 stance 站立模式")
        self._stop_cmd_vel()

        msg = switchGaitByName()
        msg.header.stamp = rospy.Time.now()
        msg.gait_name = "stance"
        self.gait_pub.publish(msg)
        rospy.loginfo("  已发布 switchGaitByName(stance)")

        rospy.sleep(2.0)
        rospy.loginfo("  stance 切换完成")

    def walk_to(self, qr, approach_distance):
        move_x, move_y, yaw = self._approach_target_from_qr_pose(qr, approach_distance)
        rospy.loginfo(
            "基于 /cmd_vel 闭环接近二维码: dx=%.3f dy=%.3f dyaw=%.3f approach=%.3f",
            move_x,
            move_y,
            yaw,
            float(approach_distance),
        )
        if move_x <= 0.0 and abs(move_y) < 1e-3 and abs(yaw) < 1e-3:
            rospy.loginfo("当前位置已满足接近距离要求，跳过行走")
            return
        target_x, target_y, target_yaw = self._relative_target_to_odom(move_x, move_y, yaw)
        if self._velocity_walk_to_target(target_x, target_y, target_yaw):
            return

        raise BoxPickPlaceError("闭环接近失败")

    def _approach_target_from_qr_pose(self, qr, approach_distance):
        quat = qr["quat"]
        normal = self._rotate_vector_by_quat([0.0, 0.0, 1.0], quat)
        normal_xy_norm = math.sqrt(normal[0] * normal[0] + normal[1] * normal[1])
        if normal_xy_norm < 1e-6:
            rospy.logwarn("二维码法线水平投影过小，回退到旧接近逻辑")
            return max(float(qr["x"]) - float(approach_distance), 0.0), float(qr["y"]), 0.0

        move_x = float(qr["x"]) + float(approach_distance) * normal[0]
        move_y = float(qr["y"]) + float(approach_distance) * normal[1]
        yaw = self._normalize_angle(math.atan2(normal[1], normal[0]) + math.pi)
        rospy.loginfo(
            "按二维码法线计算接近目标: normal=(%.3f, %.3f, %.3f) "
            "base目标=(dx=%.3f, dy=%.3f, dyaw=%.1f°)",
            normal[0],
            normal[1],
            normal[2],
            move_x,
            move_y,
            math.degrees(yaw),
        )
        return move_x, move_y, yaw

    def turn_180(self):
        # 转身闭环内部参数：角速度和容差固定，避免客户误调导致转身不稳。
        turn_settle_time = 4.0
        yaw_tolerance = math.radians(5.0)
        angular_speed = 0.25
        control_dt = 0.1
        rospy.loginfo("转身 180 度")
        current_x, current_y, current_yaw = self._get_robot_pose()
        if current_yaw is None:
            rospy.logwarn("无法获取当前 odom yaw，不能执行闭环转身")
            return False

        target_yaw = self._normalize_angle(current_yaw + math.pi)
        timeout = max(20.0, turn_settle_time + 8.0)
        rospy.loginfo(
            "闭环转身目标: 当前yaw=%.1f° 目标yaw=%.1f°",
            math.degrees(current_yaw),
            math.degrees(target_yaw),
        )
        ok = self._turn_to_yaw(target_yaw, yaw_tolerance, angular_speed, timeout, control_dt)
        self._stop_cmd_vel()
        if not ok:
            rospy.logwarn("闭环转身 180 度超时")


    def set_arm_external_control(self):
        return self.set_arm_control_mode(2, "外部控制模式")

    def set_arm_default_control(self):
        return self.set_arm_control_mode(1, "默认自动摆臂模式")

    def set_arm_control_mode(self, mode, label):
        rospy.loginfo("切换手臂到%s", label)
        req = changeArmCtrlModeRequest()
        req.control_mode = int(mode)
        resp = self.arm_mode_srv(req)
        if not getattr(resp, "result", False):
            raise BoxPickPlaceError(
                "手臂控制模式切换失败(mode=%d): %s"
                % (int(mode), getattr(resp, "message", ""))
            )
        rospy.loginfo("手臂控制模式切换成功(mode=%d): %s", int(mode), getattr(resp, "message", ""))
        return True

    def ik_solve(self, left_xyz, right_xyz, label, q0_joints=None):
        # 末端姿态固定为手心向内、手掌竖直。
        ik_q_arm_output_unit = "rad"
        ik_frame = 2
        # left_quat = [0.0, -0.70682518, 0.0, 0.70738827]
        # right_quat = [0.0, -0.70682518, 0.0, 0.70738827]
        left_quat = [0, -0.70442, 0, 0.70442] 
        right_quat = [0, -0.70442, 0, 0.70442] 
        left_elbow, right_elbow = self._ik_elbow_points()

        cmd = twoArmHandPoseCmd()
        cmd.use_custom_ik_param = False
        cmd.joint_angles_as_q0 = False
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
                "IK[%s] 使用关节种子: L=%s R=%s",
                label,
                ["%.2f" % float(value) for value in q0_joints[:7]],
                ["%.2f" % float(value) for value in q0_joints[7:]],
            )

        req = twoArmHandPoseCmdSrvRequest()
        req.twoArmHandPoseCmdRequest = cmd

        rospy.loginfo(
            "请求IK[%s]: L=%s R=%s elbow_L=%s elbow_R=%s service=%s",
            label,
            left_xyz,
            right_xyz,
            left_elbow,
            right_elbow,
            self.ik_service_name,
        )
        resp = self.ik_srv(req)
        if not getattr(resp, "success", False):
            reason = getattr(resp, "error_reason", "")
            raise BoxPickPlaceError("IK求解失败[%s]: %s" % (label, reason))
        q_arm = list(getattr(resp, "q_arm", []))
        if len(q_arm) != 14:
            raise BoxPickPlaceError("IK返回关节数异常[%s]: %d" % (label, len(q_arm)))
        self.last_ik_raw_joints = [float(value) for value in q_arm]
        q_arm_for_traj = self.convert_ik_q_arm_for_traj(q_arm)
        rospy.loginfo(
            "IK求解成功[%s]: 耗时 %.2fms %s",
            label,
            float(getattr(resp, "time_cost", 0.0)),
            getattr(resp, "error_reason", ""),
        )
        rospy.loginfo(
            "IK关节[%s]: raw(%s) min=%.3f max=%.3f -> /kuavo_arm_traj(deg) min=%.3f max=%.3f",
            label,
            ik_q_arm_output_unit,
            min(q_arm),
            max(q_arm),
            min(q_arm_for_traj),
            max(q_arm_for_traj),
        )
        rospy.loginfo(
            "IK关节[%s] raw(%s)=%s",
            label,
            ik_q_arm_output_unit,
            ["%.6f" % float(value) for value in q_arm],
        )
        rospy.loginfo(
            "IK关节[%s] 转换后(deg)=%s",
            label,
            ["%.3f" % float(value) for value in q_arm_for_traj],
        )
        rospy.loginfo(
            "IK关节[%s] 转换后再核对(rad)=%s",
            label,
            ["%.6f" % math.radians(float(value)) for value in q_arm_for_traj],
        )
        return q_arm_for_traj, float(getattr(resp, "time_cost", 0.0))

    def _ik_elbow_points(self):
        left = getattr(self, "ik_left_elbow_pos_xyz", [0.0, 0.0, 0.0])
        right = getattr(self, "ik_right_elbow_pos_xyz", [0.0, 0.0, 0.0])
        return [float(value) for value in left], [float(value) for value in right]

    @staticmethod
    def convert_ik_q_arm_for_traj(q_arm):
        # IK 服务返回弧度，/kuavo_arm_traj 需要角度。
        ik_q_arm_output_unit = "rad"
        if ik_q_arm_output_unit == "rad":
            return [math.degrees(float(value)) for value in q_arm]
        return [float(value) for value in q_arm]

    @staticmethod
    def convert_traj_joints_for_ik_seed(joints):
        # 轨迹执行使用角度，作为 IK seed 时转回弧度。
        ik_q_arm_output_unit = "rad"
        if ik_q_arm_output_unit == "rad":
            return [math.radians(float(value)) for value in joints]
        return [float(value) for value in joints]

    @staticmethod
    def default_reset_arm_deg():
        # 双臂默认起点角度，单位 deg；仅在读取不到当前关节时作为插值起点使用。
        return [
            20, 0, 0, -30, 0, 0, 0,
            20, 0, 0, -30, 0, 0, 0,
        ]

    @staticmethod
    def default_finish_reset_arm_deg():
        # 流程结束时的双臂复位角度，单位 deg；如需更换结束姿态，只改这里。
        return [
            20, 10, 0, -30, 0, 0, 0,
            20, -10, 0, -30, 0, 0, 0,
        ]

    def move_arms_interpolated(self, target_joints, duration=4.0, steps=20, start_joints=None):
        # 每段插值结束后留 0.5s 给控制器和仿真状态收敛。
        arm_segment_settle_time = 0.5
        if len(target_joints) != 14:
            raise BoxPickPlaceError("手臂目标关节必须为14维，当前=%d" % len(target_joints))
        if start_joints is None:
            start_joints = self.last_arm_joints or self.default_reset_arm_deg()
        if len(start_joints) != 14:
            raise BoxPickPlaceError("插值起始关节必须为14维，当前=%d" % len(start_joints))

        steps = max(1, int(steps))
        duration = float(duration)
        rate = rospy.Rate(max(1.0, steps / max(duration, 0.1)))
        rospy.loginfo("执行关节插值: steps=%d duration=%.2fs", steps, duration)
        for step in range(1, steps + 1):
            if rospy.is_shutdown():
                raise rospy.ROSInterruptException()
            alpha = float(step) / float(steps)
            joints = [
                float(start) + (float(target) - float(start)) * alpha
                for start, target in zip(start_joints, target_joints)
            ]
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = ["arm_joint_%d" % i for i in range(1, 15)]
            msg.position = joints
            self.arm_traj_pub.publish(msg)
            self.last_arm_joints = joints
            rate.sleep()
        rospy.sleep(arm_segment_settle_time)

    def execute_arm_sequence(self, poses, sequence, duration, start_joints=None, seed_joints=None):
        # 抓取/放置动作统一 30 步插值，兼顾平滑性和执行时间。
        steps = 30
        ik_seed = self.convert_traj_joints_for_ik_seed(seed_joints) if seed_joints is not None else None
        current = list(start_joints) if start_joints is not None else None
        q_arm = None

        for label, text in sequence:
            if label not in poses:
                rospy.logerr("Missing arm target label=%s", label)
                return None
            left_xyz, right_xyz = poses[label]
            rospy.loginfo("%s: IK -> /kuavo_arm_traj", text)
            try:
                q_arm, _ = self.ik_solve(left_xyz, right_xyz, label, q0_joints=ik_seed)
            except BoxPickPlaceError as exc:
                rospy.logerr(
                    "IK failed/unreachable label=%s L=(%.3f, %.3f, %.3f) R=(%.3f, %.3f, %.3f): %s",
                    label,
                    left_xyz[0],
                    left_xyz[1],
                    left_xyz[2],
                    right_xyz[0],
                    right_xyz[1],
                    right_xyz[2],
                    exc,
                )
                return None
            self.move_arms_interpolated(q_arm, float(duration), steps, current)
            ik_seed = list(self.last_ik_raw_joints) if self.last_ik_raw_joints is not None else None
            current = None

        return q_arm

    @staticmethod
    def _discard_ros_log(*args, **kwargs):
        pass

    def compute_blind_place_from_current_hands(self, carried_hand_pose):
        z_drop = 0.30
        release_margin_y = 0.08
        box = self.params["box"]
        release_delta_y = release_margin_y + float(box.get("grasp_clamp_inset_y", 0.02))

        if carried_hand_pose is None or len(carried_hand_pose) != 2:
            raise BoxPickPlaceError("缺少当前双手携箱位姿，不能执行复扫失败放置")

        left_hold = [float(value) for value in carried_hand_pose[0]]
        right_hold = [float(value) for value in carried_hand_pose[1]]
        if len(left_hold) != 3 or len(right_hold) != 3:
            raise BoxPickPlaceError("当前双手携箱位姿必须为左右各3维")

        left_place = [left_hold[0], left_hold[1], left_hold[2] - z_drop]
        right_place = [right_hold[0], right_hold[1], right_hold[2] - z_drop]
        left_release = [left_place[0], left_place[1] + release_delta_y, left_place[2]]
        right_release = [right_place[0], right_place[1] - release_delta_y, right_place[2]]
        return {
            "place": (left_place, right_place),
            "release": (left_release, right_release),
        }

    def finish_after_place(self):
        self.back_away_after_place(distance=0.5)
        time.sleep(0.5) 
        self.reset_arms()
        time.sleep(0.5)
        self.set_arm_default_control()
        self.turn_180()

    def place_without_rescan_and_finish(self, carried_hand_pose, ik_seed, duration):
        original_loginfo = rospy.loginfo
        rospy.loginfo = self._discard_ros_log
        try:
            self.stance()
            place_poses = self.compute_blind_place_from_current_hands(carried_hand_pose)
            ik_seed = self.execute_arm_sequence(
                place_poses,
                [
                    ("place", "place"),
                    ("release", "release"),
                ],
                duration,
                seed_joints=ik_seed,
            )
            if ik_seed is None:
                raise BoxPickPlaceError("复扫失败放置动作执行失败")
            self.finish_after_place()
            return 0
        finally:
            rospy.loginfo = original_loginfo

    def move_to_safe_arm_waypoints(self):
        # open 前依次经过三个安全姿态，降低从复位姿态直接伸手的风险。
        arm_poses = {
            "side": [
                20, 50.8, 0.0, -30, 0.0, 0.0, 0.0,
                20, -50.8, 0.0, -30, -0.0, 0.0, 0.0,
            ],
            # "safe": [
            #     7.6, 68.8, 0.0, -90.8, 77.1, -35.0, 0.0,
            #     7.6, -68.8, 0.0, -90.8, -77.1, 35.0, 0.0,
            # ],
             "safe": [
                7.6, 68.8, 0.0, -90.8, 77.1, -25.0, 0.0,
                7.6, -68.8, 0.0, -90.8,  -77.1, 25.0, 0.0,
             ]
        }
        waypoint_names = ["side", "safe"]
        waypoint_duration = 1.5
        waypoint_steps = 20
        current = self.get_current_arm_joints_deg()
        if current is None:
            current = self.default_reset_arm_deg()

        for name in waypoint_names:
            if name not in arm_poses:
                raise BoxPickPlaceError("未知手臂预设姿态: %s" % name)
            rospy.loginfo("open 前经过预设姿态: %s", name)
            target = arm_poses[name]
            self.move_arms_interpolated(target, waypoint_duration, waypoint_steps, current)
            current = target
            rospy.sleep(0.5)
        return current

    def apply_joint_bias_after_clamp(self, ik_seed, joint6_bias_deg=-20.0, joint13_bias_deg=20.0, duration=2.0):
        """在第6和第13关节上施加偏置，先读取传感器真实关节角。

        Args:
            ik_seed: 当前阶段的最终关节角（14维，deg），作为传感器读取失败时的回退基准。
            joint6_bias_deg: 第6关节（左臂 index 5）的偏置角度，单位: 度。
            joint13_bias_deg: 第13关节（右臂 index 12）的偏置角度，单位: 度。
            duration: 偏置动作的持续时间，单位: 秒。

        Returns:
            偏置后的关节角（14维，deg）。
        """
        rospy.loginfo("开始读取当前真实关节角，准备施加掌心偏置...")

        # 尝试从传感器获取当前真实关节角（14维，单位: deg）
        sensor_joints_deg = self.get_current_arm_joints_deg(timeout=1.0)

        if sensor_joints_deg is not None and len(sensor_joints_deg) == 14:
            rospy.loginfo("成功读取传感器关节角，基于真实关节角施加偏置")
            base_joints = list(sensor_joints_deg)
        else:
            rospy.logwarn("无法读取传感器关节角，退回使用 clamp 最终关节角作为基准")
            if ik_seed is None or len(ik_seed) != 14:
                raise BoxPickPlaceError("当前关节角无效，不能执行偏置")
            base_joints = list(ik_seed)

        # 在第6和第13关节上施加偏置 (0-based: index 5 和 12)
        biased_joints = list(base_joints)
        biased_joints[5] += float(joint6_bias_deg)    # 左臂第6关节
        biased_joints[12] += float(joint13_bias_deg)  # 右臂第6关节 (右臂从index 7开始，7+5=12)

        rospy.loginfo(
            "施加关节偏置 index_5=%.2f°(%+.1f°) -> %.2f°, "
            "index_12=%.2f°(%+.1f°) -> %.2f°",
            base_joints[5], float(joint6_bias_deg), biased_joints[5],
            base_joints[12], float(joint13_bias_deg), biased_joints[12],
        )

        # 关节空间插值到偏置后的目标
        self.move_arms_interpolated(biased_joints, float(duration), 30)

        rospy.loginfo("关节偏置调整完成")
        return biased_joints

    def preserve_palm_bias_joints(self, target_joints, bias_joints, label=""):
        if target_joints is None or bias_joints is None:
            return target_joints
        if len(target_joints) != 14 or len(bias_joints) != 14:
            raise BoxPickPlaceError("保留掌心偏置需要14维关节角")

        adjusted = list(target_joints)
        for idx in (5, 12):
            rospy.loginfo(
                "保持掌心偏置[%s] index_%d: IK %.2f° -> bias %.2f°",
                label,
                idx,
                float(adjusted[idx]),
                float(bias_joints[idx]),
            )
            adjusted[idx] = float(bias_joints[idx])
        return adjusted

    def reset_head(self):
        rospy.loginfo("头部回正: yaw=0.0 pitch=0.0")
        self._publish_head(0.0, 0.0)
        rospy.sleep(0.5)

    def reset_arms(self):
        self.reset_head()
        joints = self.default_finish_reset_arm_deg()
        if len(joints) != 14:
            raise BoxPickPlaceError("reset.arm_reset_deg 必须为14维")
        # 复位动作放慢一点，避免任务结束时手臂突然回零。
        duration = 3.0
        steps = 60
        start = self.get_current_arm_joints_deg(timeout=0.5)
        if start is None:
            start = list(self.last_arm_joints) if self.last_arm_joints is not None else None
        if start is None:
            rospy.logwarn("没有可信手臂当前关节，跳过手臂复位，避免直接砸回零位")
            return False
        rospy.loginfo("平滑复位双臂: steps=%d duration=%.2fs", steps, duration)
        self.move_arms_interpolated(joints, duration, steps, start)
        return True

    def terminate_robot(self, reason):
        rospy.logerr("进入终止程序: %s", reason)
        try:
            self._stop_cmd_vel()
        except Exception as exc:
            rospy.logwarn("终止程序: 停止底盘失败: %s", exc)

        try:
            if not self.reset_arms():
                rospy.logwarn("终止程序: 已跳过手臂硬复位")
        except Exception as exc:
            rospy.logwarn("终止程序: 手臂复位失败: %s", exc)

        try:
            self.set_arm_default_control()
        except Exception as exc:
            rospy.logwarn("终止程序: 恢复手臂默认模式失败: %s", exc)

        try:
            ok, message = self.enable_base_pitch_limit(True)
            if ok:
                rospy.loginfo("终止程序: 已恢复 basePitch 限制")
            else:
                rospy.logwarn("终止程序: 恢复 basePitch 限制失败: %s", message)
        except Exception as exc:
            rospy.logwarn("终止程序: 恢复 basePitch 限制异常: %s", exc)

        try:
            rospy.wait_for_service("/humanoid_controller/real_initial_start", timeout=3.0)
            client = rospy.ServiceProxy("/humanoid_controller/real_initial_start", Trigger)
            resp = client()
            if getattr(resp, "success", False):
                rospy.loginfo("终止程序: 全身站立归位完成: %s", getattr(resp, "message", ""))
            else:
                rospy.logwarn("终止程序: 全身站立归位未成功: %s", getattr(resp, "message", ""))
        except Exception as exc:
            rospy.logwarn("终止程序: 调用全身站立归位服务失败: %s", exc)

    def check_safe_space(self, label, center):
        # 抓取中心只允许落在机器人胸前工作空间内，超出时直接中止流程。
        safe_space_limits = {
            "x": (0.20, 0.70),
            "y": (-0.25, 0.25),
            "z": (0.00, 0.60),
        }
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
                "safe_space超限[%s]: center=(%.3f, %.3f, %.3f), %s"
                % (label, x, y, z, "; ".join(violations))
            )

        rospy.loginfo(
            "safe_space检查通过[%s]: center=(%.3f, %.3f, %.3f)",
            label,
            x,
            y,
            z,
        )

    def box_center_from_front_qr(self, qr):
        # 正面二维码边距定义：
        # - 二维码贴在箱子正面，所以箱体中心 x = 二维码 x + 箱子前后长度的一半。
        # - left/right：二维码中心到箱体左/右边缘的距离，左边缘对应 base_link 的 +Y 方向。
        # - top/bottom：二维码中心到箱体上/下边缘的距离，上边缘对应 base_link 的 +Z 方向。
        # 由几何关系可得：箱体中心 = 二维码中心 + length_x/2 * X
        #                              + (左边距-右边距)/2 * Y
        #                              + (上边距-下边距)/2 * Z。
        box = self.params["box"]
        margins = box.get("front_qr_margins", {})
        length_x = float(box.get("length_x", 0.40))
        width_y = float(box.get("width_y", 0.30))
        height_z = float(box.get("height_z", 0.25))

        left = float(margins.get("left", width_y / 2.0))
        right = float(margins.get("right", width_y / 2.0))
        top = float(margins.get("top", height_z / 2.0))
        bottom = float(margins.get("bottom", height_z / 2.0))

        expected_width = left + right
        expected_height = top + bottom
        if abs(expected_width - width_y) > 0.03:
            rospy.logwarn(
                "front_qr_margins 左右和 %.3fm 与 box.width_y %.3fm 不一致，请检查贴码尺寸",
                expected_width,
                width_y,
            )
        if abs(expected_height - height_z) > 0.03:
            rospy.logwarn(
                "front_qr_margins 上下和 %.3fm 与 box.height_z %.3fm 不一致，请检查贴码尺寸",
                expected_height,
                height_z,
            )

        center_x = float(qr["x"]) + length_x / 2.0
        center_y = float(qr["y"]) + (left - right) / 2.0
        center_z = float(qr["z"]) + (top - bottom) / 2.0 
        return center_x, center_y, center_z

    def compute_grasp(self, qr):
        # 抓取阶段内部参数：
        # - open_margin_y：open 阶段左右手额外张开的距离。
        # - contact_margin_y：contact 阶段接近箱体侧面但保留少量余量。
        # - grasp_height_offset_z：客户可在 YAML 中调的抓取 z 高度微调量。
        # - clamp_inset_y：clamp/lift 阶段向箱体内侧收紧的距离，由 YAML 配置。
        # - lift_height：夹紧后向上抬箱的高度。
        open_margin_y = 0.10
        contact_margin_y = 0.01
        lift_height = 0.20

        # 二维码识别结果在 base_link 坐标系下。根据正面二维码到四条边的距离，
        # 反推出箱体中心 y/z；x 方向仍使用二维码前后距离，因为四个平面边距不能推导箱体深度。
        box = self.params["box"]
        grasp_height_offset = float(box.get("grasp_height_offset_z", -0.05))
        cx, cy, cz = self.box_center_from_front_qr(qr)
        # 现场如果发现双手整体在箱体前后方向偏差，可只调
        cx += float(box.get("grasp_offset_x", 0.0))
        cz += grasp_height_offset 

        # 抓取中心必须在机器人胸前安全工作空间内，避免手臂伸到过远或过偏的位置。
        self.check_safe_space("grasp_center", (cx, cy, cz))

        # 左右手以箱体中心 y 为基准，分别偏移半个箱宽，落到箱体两侧。
        half_y = float(box["width_y"]) / 2.0

        open_margin = open_margin_y
        contact_margin = contact_margin_y
        # 夹紧量越大，左右手距离越小：手间距 = box.width_y - 2 * grasp_clamp_inset_y。
        # 现场如果转身或搬运时掉箱子，优先适当增大该参数。
        clamp_inset = float(box.get("grasp_clamp_inset_y", 0.02))
        left_z = cz 
        right_z = cz 

        # 四个抓取阶段：
        # open：双手张开到箱体外侧，给接近动作留空间。
        # contact：双手靠近箱体侧面，仍保留轻微余量。
        # clamp：双手向内收紧，形成夹持。
        # lift：保持夹持的 x/y 不变，只提升 z，抬起箱子。
        poses = {
            "open": ([cx, cy + half_y + open_margin, left_z], [cx, cy - half_y - open_margin, right_z]),
            "contact": ([cx, cy + half_y + contact_margin, left_z], [cx, cy - half_y - contact_margin, right_z]),
            "clamp": ([cx, cy + half_y - clamp_inset, left_z], [cx, cy - half_y + clamp_inset, right_z]),
            "lift": ([cx, cy + half_y - clamp_inset, left_z + lift_height], [cx, cy - half_y + clamp_inset, right_z + lift_height]),
        }
        rospy.loginfo("抓取位姿计算完成: center=(%.3f, %.3f, %.3f)", cx, cy, cz)
        self.log_end_effector_targets("抓取", poses)
        return poses

    def compute_place(self, qr):
        # 放置阶段内部参数：
        # - hand_x_offset：放置点相对二维码/箱体中心的前后补偿。
        # - place_z_offset_from_qr：二维码中心到放置末端高度的 z 轴偏移。
        # - release_margin_y：release 阶段左右手额外向外张开的距离。
        hand_x_offset = 0.0
        place_z_offset_from_qr = 0.45
        release_margin_y = 0.18
        left_z_offset = 0.0
        right_z_offset = 0.0

        # 放置二维码同样在 base_link 坐标系下。根据正面二维码边距修正 y，
        # 得到放置动作使用的平面中心；z 使用二维码中心高度减去固定偏移。
        box = self.params["box"]
        cx, cy, _ = self.box_center_from_front_qr(qr)
        cx += hand_x_offset
        cz = float(qr["z"]) - place_z_offset_from_qr

        # 左右手仍以箱体中心 y 为基准，按半个箱宽分布到箱体两侧。
        half_y = float(box["width_y"]) / 2.0
        release_margin = release_margin_y
        # place 阶段先沿用抓取夹紧量，保证放下前不松手；release 阶段再张开。
        clamp_inset = float(box.get("grasp_clamp_inset_y", 0.02))
        left_z = cz + left_z_offset
        right_z = cz + right_z_offset

        # 两个放置阶段：
        # place：保持夹紧状态下降/移动到放置点。
        # release：左右手向外张开 release_margin，释放箱子。
        poses = {
            "place": ([cx, cy + half_y - clamp_inset, left_z], [cx, cy - half_y + clamp_inset, right_z]),
            "release": ([cx, cy + half_y + release_margin, left_z], [cx, cy - half_y - release_margin, right_z]),
        }
        rospy.loginfo("放置位姿计算完成: center=(%.3f, %.3f, %.3f)", cx, cy, cz)
        self.log_end_effector_targets("放置", poses)
        return poses

    def log_end_effector_targets(self, stage, poses):
        ik_frame = int(self.params.get("wheel", {}).get("ik", {}).get("frame", 2))
        for label, pair in poses.items():
            left_xyz, right_xyz = pair
            rospy.loginfo(
                "%s末端目标[%s] frame=%d L=(%.3f, %.3f, %.3f) R=(%.3f, %.3f, %.3f)",
                stage,
                label,
                ik_frame,
                left_xyz[0],
                left_xyz[1],
                left_xyz[2],
                right_xyz[0],
                right_xyz[1],
                right_xyz[2],
            )

    def run(self):
        try:
            # 行走接近距离由 YAML 配置，方便现场按二维码安装位置微调。
            walk_cfg = self.params["walk"]
            pick_qr_approach_distance = float(walk_cfg.get("pick_approach_distance", 0.01))
            place_qr_approach_distance = float(walk_cfg.get("place_approach_distance", 0.01))
            arm_sequence_duration = 2.0
            qr_cfg = self.params["qr"]
            grasp_cfg = self.params.get("grasp", {})
            use_palm_joint_bias = bool(grasp_cfg.get("use_palm_joint_bias", True))
            grasp_duration = arm_sequence_duration
            place_duration = arm_sequence_duration
            carry_scan_pitch_deg = 24.0
            place_scan_pitch_deg = -24.0
            rospy.loginfo("掌心关节偏置 use_palm_joint_bias=%s", use_palm_joint_bias)

            self.disable_base_pitch_limit()
            pick_qr = self.scan_qr(qr_cfg["pick_qr_id"])
            self.walk_to(pick_qr, pick_qr_approach_distance)
            rospy.loginfo("接近抓取二维码后重新扫描，刷新 base_link 坐标")
            pick_qr = self.scan_qr_after_walk(
                qr_cfg["pick_qr_id"],
                pitch_deg=carry_scan_pitch_deg,
            )

            rospy.loginfo("切换到 stance 站立模式，准备抓取箱子")
            self.stance()

            self.set_arm_external_control()
            rospy.loginfo("先通过 /kuavo_arm_traj 到达安全预设姿态")
            safe_current = self.move_to_safe_arm_waypoints()
            rospy.loginfo("准备已完成，使用该姿态作为抓取IK初始种子")

            grasp_poses = self.compute_grasp(pick_qr)
            ik_seed = self.execute_arm_sequence(
                grasp_poses,
                [
                    ("open", "open"),
                ],
                grasp_duration,
                start_joints=safe_current,
                seed_joints=safe_current,
            )
            if ik_seed is None:
                return 1

            palm_bias_joints = None
            if use_palm_joint_bias:
                # open 后先施加掌心偏置，后续 contact/lift 都保持 5、12 号关节偏置值。
                ik_seed = self.apply_joint_bias_after_clamp(
                    ik_seed,
                    joint6_bias_deg=-20.0,
                    joint13_bias_deg=20.0,
                    duration=2.0,
                )
                palm_bias_joints = list(ik_seed)
            else:
                rospy.loginfo("use_palm_joint_bias=false，跳过掌心关节偏置")

            rospy.loginfo("contact: 调用IK并保持掌心偏置")
            q_contact, _ = self.ik_solve(
                grasp_poses["contact"][0],
                grasp_poses["contact"][1],
                "contact",
                q0_joints=self.convert_traj_joints_for_ik_seed(ik_seed),
            )
            if use_palm_joint_bias and palm_bias_joints is not None:
                q_contact = self.preserve_palm_bias_joints(q_contact, palm_bias_joints, "contact")
            self.move_arms_interpolated(q_contact, grasp_duration, 30)
            ik_seed = q_contact

            rospy.loginfo("clamp: 调用IK并使用偏置后的关节角作为种子")
            q_clamp, _ = self.ik_solve(
                grasp_poses["clamp"][0],
                grasp_poses["clamp"][1],
                "clamp",
                q0_joints=self.convert_traj_joints_for_ik_seed(ik_seed),
            )
            if use_palm_joint_bias and palm_bias_joints is not None:
                q_clamp = self.preserve_palm_bias_joints(q_clamp, palm_bias_joints, "clamp")
            self.move_arms_interpolated(q_clamp, grasp_duration, 30)
            ik_seed = q_clamp

            # lift 单独执行，继续保持掌心偏置关节不被 IK 解回去。
            rospy.loginfo("lift: 调用IK并执行抬起箱子")
            q_lift, _ = self.ik_solve(
                grasp_poses["lift"][0],
                grasp_poses["lift"][1],
                "lift",
                q0_joints=self.convert_traj_joints_for_ik_seed(ik_seed),
            )
            if use_palm_joint_bias and palm_bias_joints is not None:
                q_lift = self.preserve_palm_bias_joints(q_lift, palm_bias_joints, "lift")
            self.move_arms_interpolated(q_lift, grasp_duration, 30)
            ik_seed = q_lift
            carried_hand_pose = (
                list(grasp_poses["lift"][0]),
                list(grasp_poses["lift"][1]),
            )

            self.back_away_after_place(distance=0.5, label="抓取完成后")
            self.turn_180()

            place_qr = self.scan_qr(
                qr_cfg["place_qr_id"],
                pitch_center=place_scan_pitch_deg,
                pitch_range=0.0,
                initial_pitch=place_scan_pitch_deg,
            )
            self.walk_to(place_qr, place_qr_approach_distance)
            rospy.loginfo("接近放置二维码后重新扫描，刷新 base_link 坐标")
            try:
                place_qr = self.scan_qr_after_walk(
                    qr_cfg["place_qr_id"],
                    pitch_deg=place_scan_pitch_deg,
                )
            except BoxPickPlaceError:
                return self.place_without_rescan_and_finish(
                    carried_hand_pose,
                    ik_seed,
                    place_duration,
                )

            rospy.loginfo("切换到 stance 站立模式，准备放下箱子")
            self.stance()

            place_poses = self.compute_place(place_qr)
            ik_seed = self.execute_arm_sequence(
                place_poses,
                [
                    ("place", "place"),
                    ("release", "release"),
                ],
                place_duration,
                seed_joints=ik_seed,
            )
            if ik_seed is None:
                return 1

            self.finish_after_place()
            rospy.loginfo("双臂夹箱搬运流程完成")
            return 0
        except BoxPickPlaceError as exc:
            rospy.logerr("流程失败: %s", exc)
            self.terminate_robot(exc)
            return 1
        except rospy.ROSInterruptException:
            rospy.logerr("ROS中断，流程退出")
            return 1

    def _publish_head(self, yaw_deg, pitch_deg):
        # 头部扫描角度限幅，避免搜索二维码时超过机械安全范围。
        head_yaw_limit_deg = 30.0
        head_pitch_limit_deg = 25.0
        msg = robotHeadMotionData()
        yaw_deg = max(-head_yaw_limit_deg, min(head_yaw_limit_deg, float(yaw_deg)))
        pitch_deg = max(-head_pitch_limit_deg, min(head_pitch_limit_deg, float(pitch_deg)))
        msg.joint_data = [yaw_deg, pitch_deg]
        self.head_pub.publish(msg)

    def _relative_target_to_odom(self, move_x, move_y, yaw):
        current_x, current_y, current_yaw = self._get_robot_pose()
        if current_x is None:
            raise BoxPickPlaceError("无法获取 odom->base_link，不能执行闭环接近")

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        target_x = current_x + float(move_x) * cos_yaw - float(move_y) * sin_yaw
        target_y = current_y + float(move_x) * sin_yaw + float(move_y) * cos_yaw
        target_yaw = self._normalize_angle(current_yaw + float(yaw))
        rospy.loginfo(
            "闭环接近坐标换算: 当前odom=(x=%.3f, y=%.3f, yaw=%.1f°), "
            "base_link目标=(dx=%.3f, dy=%.3f, dyaw=%.1f°), "
            "odom目标=(x=%.3f, y=%.3f, yaw=%.1f°)",
            current_x,
            current_y,
            math.degrees(current_yaw),
            float(move_x),
            float(move_y),
            math.degrees(float(yaw)),
            target_x,
            target_y,
            math.degrees(target_yaw),
        )
        rospy.loginfo(
            "闭环接近目标(odom): x=%.3f y=%.3f yaw=%.1f°",
            target_x,
            target_y,
            math.degrees(target_yaw),
        )
        return target_x, target_y, target_yaw

    def _velocity_walk_to_target(self, target_x, target_y, target_yaw):
        # 行走闭环参数固定在函数内，客户只调 linear_speed。
        linear_speed = float(self.params.get("walk", {}).get("linear_speed", 0.15))
        angular_speed = 0.25
        pos_tolerance = 0.10
        yaw_tolerance = math.radians(5.0)
        turn_timeout = 180.0
        walk_timeout = 30.0
        final_turn_timeout = 60.0
        control_dt = 0.1

        rospy.loginfo("  阶段1: 转向目标方向...")
        ok_turn = self._turn_to_target_direction(
            target_x,
            target_y,
            yaw_tolerance,
            angular_speed,
            turn_timeout,
            control_dt,
        )
        if not ok_turn:
            rospy.logwarn(" 转向目标方向超时，继续尝试行走")
        self._keep_creeping()

        rospy.loginfo("  阶段2: 行走到目标位置...")
        ok_walk = self._walk_straight_to_target(
            target_x,
            target_y,
            pos_tolerance,
            linear_speed,
            walk_timeout,
            control_dt,
        )
        if not ok_walk:
            self._stop_cmd_vel()
            return False
        self._stop_cmd_vel()

        rospy.loginfo("  阶段3: 调整最终朝向...")
        ok_final = self._turn_to_yaw(
            target_yaw,
            yaw_tolerance,
            angular_speed,
            final_turn_timeout,
            control_dt,
        )
        self._stop_cmd_vel()
        if ok_final:
            rospy.loginfo("  闭环接近二维码完成")
        else:
            rospy.logwarn("  最终朝向调整超时")
        return ok_final

    def _cmd_vel_lateral_adjust(
        self,
        target_y,
        lateral_speed,
        timeout,
        pos_tolerance=0.01,
        min_lateral_speed=None,
        log_label="二维码y",
    ):
        start_x, start_y, start_yaw = self._get_robot_pose()
        if start_x is None:
            rospy.logwarn("无法获取 odom->base_link，不能执行/cmd_vel横向闭环微调")
            return False

        target_y = float(target_y)
        lateral_speed = abs(float(lateral_speed))
        timeout = max(float(timeout) + 1.0, 2.0)
        control_dt = 0.1
        pos_tolerance = abs(float(pos_tolerance))
        if min_lateral_speed is None:
            min_lateral_speed = min(0.04, max(lateral_speed * 0.5, 0.02))
        else:
            min_lateral_speed = abs(float(min_lateral_speed))
        start_time = time.time()
        last_print_time = 0.0

        rospy.loginfo(
            "%s /cmd_vel闭环微调开始: target_y=%.3fm speed=%.3fm/s timeout=%.2fs",
            log_label,
            target_y,
            lateral_speed,
            timeout,
        )

        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            current_x, current_y, current_yaw = self._get_robot_pose()
            if current_x is None:
                rospy.sleep(control_dt)
                continue

            dx = current_x - start_x
            dy = current_y - start_y
            moved_y = -dx * math.sin(start_yaw) + dy * math.cos(start_yaw)
            remaining_y = target_y - moved_y

            elapsed = time.time() - start_time
            if elapsed - last_print_time > 0.5:
                rospy.loginfo(
                    "    [%s微调] moved=%.3fm target=%.3fm remaining=%.3fm",
                    log_label,
                    moved_y,
                    target_y,
                    remaining_y,
                )
                last_print_time = elapsed

            if abs(remaining_y) <= pos_tolerance:
                self._stop_cmd_vel()
                rospy.loginfo("%s /cmd_vel闭环微调完成: moved_y=%.3fm", log_label, moved_y)
                return True

            vy = self._planned_axis_speed(
                remaining_y,
                lateral_speed,
                min_lateral_speed,
                pos_tolerance,
            )
            self._publish_cmd_vel(0.0, vy, 0.0)
            rospy.sleep(control_dt)

        self._stop_cmd_vel()
        rospy.logwarn("%s /cmd_vel闭环微调超时: target_y=%.3fm", log_label, target_y)
        return False

    def _turn_to_target_direction(self, target_x, target_y, yaw_tolerance, angular_speed, timeout, control_dt):
        start_time = time.time()
        last_print_time = 0.0
        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            current_x, current_y, current_yaw = self._get_robot_pose()
            if current_x is None:
                rospy.sleep(control_dt)
                continue

            dx = target_x - current_x
            dy = target_y - current_y
            target_dir = math.atan2(dy, dx)
            angle_diff = self._normalize_angle(target_dir - current_yaw)

            elapsed = time.time() - start_time
            if elapsed - last_print_time > 2.0:
                rospy.loginfo(
                    "    [转向] 当前yaw=%.1f° 目标=%.1f° 差=%.1f°",
                    math.degrees(current_yaw),
                    math.degrees(target_dir),
                    math.degrees(angle_diff),
                )
                last_print_time = elapsed

            if abs(angle_diff) < yaw_tolerance:
                rospy.loginfo("    转向完成 (误差=%.1f°)", math.degrees(angle_diff))
                return True

            turn_speed = self._planned_turn_speed(angle_diff, angular_speed)
            self._publish_cmd_vel(0.0, 0.0, turn_speed)
            rospy.sleep(control_dt)
        return False

    def _walk_straight_to_target(self, target_x, target_y, pos_tolerance, linear_speed, timeout, control_dt):
        start_time = time.time()
        last_print_time = 0.0
        max_linear_speed = abs(float(linear_speed))
        min_linear_speed = min(0.05, max_linear_speed)
        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            current_x, current_y, current_yaw = self._get_robot_pose()
            if current_x is None:
                rospy.sleep(control_dt)
                continue

            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.sqrt(dx * dx + dy * dy)
            target_dir = math.atan2(dy, dx)
            angle_diff = self._normalize_angle(target_dir - current_yaw)

            elapsed = time.time() - start_time
            if elapsed - last_print_time > 0.5:
                rospy.loginfo(
                    "    [行走] pos=(%.2f, %.2f) dist=%.2fm angle_diff=%.1f°",
                    current_x,
                    current_y,
                    distance,
                    math.degrees(angle_diff),
                )
                last_print_time = elapsed

            if distance <= pos_tolerance:
                self._publish_cmd_vel(0.0, 0.0, 0.0)
                rospy.loginfo("    到达位置 (距离=%.2fm)", distance)
                return True

            # 使用“到容差边界的剩余距离”降速，避免以固定的 70% 速度穿过目标。
            if distance > 0.5:
                vx = max_linear_speed
            else:
                remaining = max(0.0, distance - pos_tolerance)
                vx = min(max_linear_speed, max(min_linear_speed, 0.8 * remaining))

            if abs(angle_diff) < 0.05:
                vz = 0.0
            elif abs(angle_diff) < 0.15:
                vz = 0.1 if angle_diff > 0.0 else -0.1
            else:
                vz = 0.15 if angle_diff > 0.0 else -0.15

            self._publish_cmd_vel(vx, 0.0, vz)
            rospy.sleep(control_dt)
        rospy.logwarn("    行走超时")
        return False

    def _turn_to_yaw(self, target_yaw, yaw_tolerance, angular_speed, timeout, control_dt):
        start_time = time.time()
        last_print_time = 0.0
        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            current_x, current_y, current_yaw = self._get_robot_pose()
            if current_x is None:
                rospy.sleep(control_dt)
                continue

            angle_diff = self._normalize_angle(target_yaw - current_yaw)
            elapsed = time.time() - start_time
            if elapsed - last_print_time > 2.0:
                rospy.loginfo(
                    "    [朝向] 当前yaw=%.1f° 目标=%.1f° 差=%.1f°",
                    math.degrees(current_yaw),
                    math.degrees(target_yaw),
                    math.degrees(angle_diff),
                )
                last_print_time = elapsed

            if abs(angle_diff) < yaw_tolerance:
                rospy.loginfo("    朝向调整完成 (误差=%.1f°)", math.degrees(angle_diff))
                return True

            turn_speed = self._planned_turn_speed(angle_diff, angular_speed)
            self._publish_cmd_vel(0.0, 0.0, turn_speed)
            rospy.sleep(control_dt)
        return False

    def back_away_after_place(self, distance=1.0, label="放置后"):
        """保持手臂不动，身体按当前朝向倒退一段距离。"""
        distance = abs(float(distance))
        if distance <= 0.0:
            return True

        start_x, start_y, start_yaw = self._get_robot_pose()
        if start_x is None:
            rospy.logwarn("无法获取 odom->base_link，不能执行闭环后退 %.2fm", distance)
            return False

        walk_cfg = self.params.get("walk", {})
        back_speed = min(0.12, abs(float(walk_cfg.get("linear_speed", 0.15))))
        angular_speed = 0.18
        pos_tolerance = 0.07
        control_dt = 0.1
        timeout = max(12.0, distance / max(back_speed, 0.05) + 5.0)
        target_x = start_x - distance * math.cos(start_yaw)
        target_y = start_y - distance * math.sin(start_yaw)
        back_dir_x = -math.cos(start_yaw)
        back_dir_y = -math.sin(start_yaw)

        rospy.loginfo(
            "%s保持手臂不动，准备倒退 %.2fm: 当前odom=(%.3f, %.3f, %.1f°) "
            "目标odom=(%.3f, %.3f)",
            label,
            distance,
            start_x,
            start_y,
            math.degrees(start_yaw),
            target_x,
            target_y,
        )

        start_time = time.time()
        last_print_time = 0.0
        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            current_x, current_y, current_yaw = self._get_robot_pose()
            if current_x is None:
                rospy.sleep(control_dt)
                continue

            dx = target_x - current_x
            dy = target_y - current_y
            remaining = math.sqrt(dx * dx + dy * dy)
            traveled = (
                (current_x - start_x) * back_dir_x
                + (current_y - start_y) * back_dir_y
            )
            heading_error = self._normalize_angle(start_yaw - current_yaw)

            elapsed = time.time() - start_time
            if elapsed - last_print_time > 0.5:
                rospy.loginfo(
                    "    [后退] traveled=%.2fm remaining=%.2fm heading_error=%.1f°",
                    traveled,
                    remaining,
                    math.degrees(heading_error),
                )
                last_print_time = elapsed

            if traveled >= distance - pos_tolerance or remaining < pos_tolerance:
                self._stop_cmd_vel()
                rospy.loginfo("%s倒退完成: traveled=%.2fm remaining=%.2fm", label, traveled, remaining)
                return True

            if remaining < 0.25:
                vx = -max(0.06, back_speed * 0.6)
            else:
                vx = -back_speed

            if abs(heading_error) < math.radians(3.0):
                vz = 0.0
            else:
                vz = self._planned_turn_speed(heading_error, angular_speed)
            self._publish_cmd_vel(vx, 0.0, vz)
            rospy.sleep(control_dt)

        self._stop_cmd_vel()
        rospy.logwarn("%s倒退超时", label)
        return False

    def _keep_creeping(self):
        # 阶段切换时保持极小前进，避免控制器从行走状态突然掉到站立状态。
        creep_speed = 0.05
        creep_time = 1.0
        control_dt = 0.1
        if creep_speed <= 0.0 or creep_time <= 0.0:
            return
        steps = max(1, int(creep_time / max(control_dt, 0.02)))
        rospy.loginfo("    保持微小前进速度，准备进入下一阶段")
        for _ in range(steps):
            if rospy.is_shutdown():
                raise rospy.ROSInterruptException()
            self._publish_cmd_vel(creep_speed, 0.0, 0.0)
            rospy.sleep(control_dt)


    def _stop_cmd_vel(self, duration=0.6, control_dt=0.05):
        start_time = time.time()
        duration = max(float(duration), float(control_dt))
        while not rospy.is_shutdown() and time.time() - start_time < duration:
            self._publish_cmd_vel(0.0, 0.0, 0.0)
            rospy.sleep(control_dt)

    def _publish_cmd_vel(self, vx, vy, vz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = 0.0
        msg.angular.z = float(vz)
        self.cmd_vel_pub.publish(msg)

    def _get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom",
                "base_link",
                rospy.Time(0),
                rospy.Duration(0.5),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(2.0, "获取 odom->base_link 失败: %s", exc)
            return None, None, None

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        yaw = self._yaw_from_quat(transform.transform.rotation)
        return x, y, yaw

    @staticmethod
    def _planned_turn_speed(angle_diff, angular_speed):
        abs_diff = abs(angle_diff)
        max_speed = abs(float(angular_speed))
        min_speed = min(0.06, max_speed * 0.3)
        if abs_diff > 0.35:
            turn_speed = max_speed
        elif abs_diff > 0.12:
            turn_speed = min_speed + (abs_diff - 0.12) / (0.35 - 0.12) * (max_speed - min_speed)
        else:
            turn_speed = max(min_speed, abs_diff * 0.5)
        return turn_speed if angle_diff > 0.0 else -turn_speed


    @classmethod
    def _planned_axis_speed(cls, error, max_abs, min_abs, tolerance):
        error = float(error)
        if abs(error) <= float(tolerance):
            return 0.0
        speed = cls._clamp_abs(0.8 * error, max_abs)
        if abs(speed) < float(min_abs):
            return float(min_abs) if error > 0.0 else -float(min_abs)
        return speed

    @staticmethod
    def _clamp_abs(value, max_abs):
        return max(-float(max_abs), min(float(max_abs), float(value)))

    @staticmethod
    def _scan_yaws(scan_range, scan_step):
        scan_range = abs(float(scan_range))
        scan_step = abs(float(scan_step)) or scan_range or 1.0
        values = [0.0]
        step = scan_step
        while step <= scan_range + 1e-6:
            values.extend([step, -step])
            step += scan_step
        return values

    @classmethod
    def _scan_head_points(cls, yaw_range, yaw_step, pitch_center, pitch_range, pitch_step):
        yaws = cls._scan_yaws(yaw_range, yaw_step)
        pitch_offsets = cls._scan_yaws(pitch_range, pitch_step)
        return [
            (yaw, float(pitch_center) + pitch_offset)
            for pitch_offset in pitch_offsets
            for yaw in yaws
        ]

    @staticmethod
    def _yaw_from_quat(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

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
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Kuavo 双臂夹箱搬运顺序控制")
    parser.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "box_pick_place.yaml"),
        help="box_pick_place.yaml 路径",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(rospy.myargv(argv=sys.argv)[1:])
    try:
        node = BoxPickPlace(args.config)
        return node.run()
    except BoxPickPlaceError as exc:
        try:
            rospy.logerr("初始化失败: %s", exc)
        except Exception:
            print("初始化失败: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
