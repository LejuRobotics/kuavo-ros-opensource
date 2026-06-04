#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rospy
import rospkg
from kuavo_msgs.msg import ikSolveParam, sensorsData, twoArmHandPose, twoArmHandPoseCmd
from kuavo_msgs.srv import fkSrv, twoArmHandPoseCmdSrv
from sensor_msgs.msg import JointState


def quat_angle_deg(q_target_xyzw, q_actual_xyzw) -> float:
    qt = np.asarray(q_target_xyzw, dtype=float)
    qa = np.asarray(q_actual_xyzw, dtype=float)
    nt = np.linalg.norm(qt)
    na = np.linalg.norm(qa)
    if nt < 1e-9 or na < 1e-9:
        return float("nan")
    qt = qt / nt
    qa = qa / na
    dot_val = float(np.dot(qt, qa))
    dot_val = max(-1.0, min(1.0, dot_val))
    return 2.0 * math.degrees(math.acos(abs(dot_val)))


class ArmsIKApi(object):
    def __init__(self, node_name: str, wait_secs: float = 2.0):
        rospy.init_node(node_name, anonymous=True)
        self.joint_pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
        self._ik_result_msg: Optional[twoArmHandPose] = None
        self.ik_result_sub = rospy.Subscriber("/ik/result", twoArmHandPose, self._ik_result_cb)
        self._arm_q_rad = np.zeros(14, dtype=float)
        self._arm_q_valid = False
        self._arm_slice_start, self._arm_slice_end = self._load_arm_joint_slice_indices()
        self._sensors_sub = rospy.Subscriber(
            "/sensors_data_raw", sensorsData, self._sensors_data_raw_cb, queue_size=1
        )
        self._warned_no_arm_feedback = False
        time.sleep(wait_secs)

    @staticmethod
    def _load_arm_joint_slice_indices() -> Tuple[int, int]:
        """从 kuavo.json 计算 joint_q 中双臂段起止下标。

        与 ``WheelArmControlBaseROS`` 一致：
        ``start = NUM_JOINT - NUM_HEAD_JOINT - NUM_ARM_JOINT``（轮臂如 20-2-14=4）；
        双足长向量布局下与 ``12 + NUM_WAIST_JOINT`` 等价（如 29-2-14=13）。
        """
        try:
            rp = rospkg.RosPack()
            assets = rp.get_path("kuavo_assets")
            if rospy.has_param("/robot_version"):
                rv = str(int(rospy.get_param("/robot_version")))
            else:
                rv = os.environ.get("ROBOT_VERSION", "40")
            if rv == "15":
                rv = "14"
            cfg_path = os.path.join(assets, "config", "kuavo_v{}".format(rv), "kuavo.json")
            with open(cfg_path, "r") as f:
                cfg = json.load(f)
            num_arm = int(cfg.get("NUM_ARM_JOINT", 14))
            num_head = int(cfg.get("NUM_HEAD_JOINT", 2))
            num_total = int(cfg.get("NUM_JOINT", 28))
            start = num_total - num_head - num_arm
            if start < 0:
                raise ValueError("invalid NUM_JOINT/HEAD/ARM: {} {} {}".format(num_total, num_head, num_arm))
            return start, start + num_arm
        except Exception as e:
            rospy.logwarn("[arms_ik_api] 无法从 kuavo.json 读取手臂切片，使用默认 12:26: %s", e)
            return 12, 26

    def _sensors_data_raw_cb(self, msg: sensorsData):
        q = msg.joint_data.joint_q
        if len(q) < self._arm_slice_end:
            return
        self._arm_q_rad = np.asarray(q[self._arm_slice_start : self._arm_slice_end], dtype=float)
        self._arm_q_valid = True

    def get_current_arm_joints_rad(self) -> np.ndarray:
        """当前双臂关节角 (14,) rad；若尚未收到传感器则尽量短暂等待。"""
        if not self._arm_q_valid:
            deadline = time.time() + 2.0
            while not rospy.is_shutdown() and time.time() < deadline:
                if self._arm_q_valid:
                    break
                rospy.sleep(0.05)
        if not self._arm_q_valid and not self._warned_no_arm_feedback:
            self._warned_no_arm_feedback = True
            rospy.logwarn(
                "[arms_ik_api] 未收到 /sensors_data_raw 中的手臂关节，"
                "joint_angles 仍用零向量；请确认仿真/真机已发布该话题。"
            )
        return np.array(self._arm_q_rad, copy=True)

    def _ik_result_cb(self, msg: twoArmHandPose):
        self._ik_result_msg = msg

    @staticmethod
    def default_ik_param(constraint_mode: int = 0) -> ikSolveParam:
        p = ikSolveParam()
        p.major_optimality_tol = 4e-3
        p.major_feasibility_tol = 4e-3
        p.minor_feasibility_tol = 4e-3
        p.major_iterations_limit = 100
        p.oritation_constraint_tol = 4e-3
        p.pos_constraint_tol = 4e-3
        p.pos_cost_weight = 10.0
        p.constraint_mode = int(constraint_mode)
        return p

    def build_two_arm_cmd(
        self,
        left_pos_xyz,
        left_quat_xyzw,
        right_pos_xyz,
        right_quat_xyzw,
        ik_param: Optional[ikSolveParam] = None,
        use_custom_ik_param: bool = True,
        joint_angles_as_q0: bool = False,
        left_joint_angles=None,
        right_joint_angles=None,
        left_elbow_pos_xyz=None,
        right_elbow_pos_xyz=None,
        frame: int = 0,
    ) -> twoArmHandPoseCmd:
        cmd = twoArmHandPoseCmd()
        cmd.use_custom_ik_param = bool(use_custom_ik_param)
        cmd.joint_angles_as_q0 = bool(joint_angles_as_q0)
        cmd.ik_param = ik_param if ik_param is not None else ArmsIKApi.default_ik_param()
        cmd.frame = int(frame)

        q_arm = self.get_current_arm_joints_rad()
        if left_joint_angles is None:
            left_joint_angles = q_arm[:7]
        if right_joint_angles is None:
            right_joint_angles = q_arm[7:14]
        if left_elbow_pos_xyz is None:
            left_elbow_pos_xyz = np.zeros(3)
        if right_elbow_pos_xyz is None:
            right_elbow_pos_xyz = np.zeros(3)

        cmd.hand_poses.left_pose.pos_xyz = np.asarray(left_pos_xyz, dtype=float)
        cmd.hand_poses.left_pose.quat_xyzw = np.asarray(left_quat_xyzw, dtype=float)
        cmd.hand_poses.left_pose.joint_angles = np.asarray(left_joint_angles, dtype=float)
        cmd.hand_poses.left_pose.elbow_pos_xyz = np.asarray(left_elbow_pos_xyz, dtype=float)

        cmd.hand_poses.right_pose.pos_xyz = np.asarray(right_pos_xyz, dtype=float)
        cmd.hand_poses.right_pose.quat_xyzw = np.asarray(right_quat_xyzw, dtype=float)
        cmd.hand_poses.right_pose.joint_angles = np.asarray(right_joint_angles, dtype=float)
        cmd.hand_poses.right_pose.elbow_pos_xyz = np.asarray(right_elbow_pos_xyz, dtype=float)
        return cmd

    def call_fk(self, q: List[float]):
        rospy.wait_for_service("/ik/fk_srv")
        srv = rospy.ServiceProxy("/ik/fk_srv", fkSrv)
        return srv(q)

    def call_ik_srv(self, cmd: twoArmHandPoseCmd):
        rospy.wait_for_service("/ik/two_arm_hand_pose_cmd_srv")
        srv = rospy.ServiceProxy("/ik/two_arm_hand_pose_cmd_srv", twoArmHandPoseCmdSrv)
        return srv(cmd)

    def call_ik_multi_ref_srv(self, cmd: twoArmHandPoseCmd):
        rospy.wait_for_service("/ik/two_arm_hand_pose_cmd_srv_muli_refer")
        srv = rospy.ServiceProxy("/ik/two_arm_hand_pose_cmd_srv_muli_refer", twoArmHandPoseCmdSrv)
        return srv(cmd)

    def publish_ik_topic(self, cmd: twoArmHandPoseCmd):
        pub = rospy.Publisher("/ik/two_arm_hand_pose_cmd", twoArmHandPoseCmd, queue_size=10)
        time.sleep(0.3)
        pub.publish(cmd)

    def reset_ik_result_topic_buffer(self):
        """清空缓存的 /ik/result，避免 case 里读到上一次逆解结果。"""
        self._ik_result_msg = None

    def wait_ik_result_topic(self, timeout_sec: float = 2.0) -> Optional[twoArmHandPose]:
        start = time.time()
        while (time.time() - start) < timeout_sec and not rospy.is_shutdown():
            if self._ik_result_msg is not None:
                return self._ik_result_msg
            time.sleep(0.02)
        return None

    def publish_arm_traj(self, q_arm_rad: List[float], stamp_now: bool = True):
        msg = JointState()
        msg.name = ["arm_joint_{}".format(i) for i in range(1, 15)]
        if stamp_now:
            msg.header.stamp = rospy.Time.now()
        msg.position = (180.0 / math.pi * np.asarray(q_arm_rad, dtype=float)).tolist()
        self.joint_pub.publish(msg)

    @staticmethod
    def calc_pose_error(target_pose, solved_pose) -> Dict[str, float]:
        lp = np.asarray(solved_pose.left_pose.pos_xyz) - np.asarray(target_pose.left_pose.pos_xyz)
        rp = np.asarray(solved_pose.right_pose.pos_xyz) - np.asarray(target_pose.right_pose.pos_xyz)
        lmm = float(np.linalg.norm(lp) * 1000.0)
        rmm = float(np.linalg.norm(rp) * 1000.0)
        lo = quat_angle_deg(target_pose.left_pose.quat_xyzw, solved_pose.left_pose.quat_xyzw)
        ro = quat_angle_deg(target_pose.right_pose.quat_xyzw, solved_pose.right_pose.quat_xyzw)
        return {
            "left_pos_mm": lmm,
            "right_pos_mm": rmm,
            "left_ori_deg": lo,
            "right_ori_deg": ro,
            "max_pos_mm": max(lmm, rmm),
            "max_ori_deg": max(lo, ro),
        }

    @staticmethod
    def print_ik_summary(tag: str, success: bool, time_cost_ms: Optional[float], err: Optional[Dict[str, float]] = None):
        if err is None:
            print("[{}] success={}, time_cost_ms={}".format(tag, success, time_cost_ms))
            return
        print(
            "[{}] success={}, time_cost_ms={:.2f}, "
            "pos_error(mm): L={:.2f} R={:.2f} max={:.2f}, "
            "ori_error(deg): L={:.3f} R={:.3f} max={:.3f}".format(
                tag,
                success,
                float(time_cost_ms) if time_cost_ms is not None else float("nan"),
                err["left_pos_mm"],
                err["right_pos_mm"],
                err["max_pos_mm"],
                err["left_ori_deg"],
                err["right_ori_deg"],
                err["max_ori_deg"],
            )
        )
