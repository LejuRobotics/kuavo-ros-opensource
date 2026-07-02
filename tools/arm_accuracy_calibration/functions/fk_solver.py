# -*- coding: utf-8 -*-
"""
正运动学：调用 /ik/fk_srv_with_refer_frame 计算肘关节和末端在 base/shoulder 下的位姿
"""
import numpy as np
from typing import Optional, Tuple, Dict, Any

# 最新 sensor_data，由 sensor_data_callback 更新
_sensor_data = None


def sensor_data_callback(msg):
    """订阅 sensor_data_raw 的回调，用于更新关节角"""
    global _sensor_data
    _sensor_data = msg


def get_arm_fk_with_refer_frame(
    hand_side: int,
) -> Optional[Dict[str, Any]]:
    """
    调用 /ik/fk_srv_with_refer_frame 计算肘关节和末端在 base、shoulder 下的位姿。

    参数:
        hand_side: 0=左臂, 1=右臂

    返回:
        {
            "elbow_base": (pos_mm, quat),
            "end_base": (pos_mm, quat),
            "elbow_shoulder": (pos_mm, quat),
            "end_shoulder": (pos_mm, quat),
        }
        失败返回 None。pos 单位 mm，quat [x,y,z,w]
    """
    global _sensor_data
    if _sensor_data is None:
        return None
    if not hasattr(_sensor_data, "joint_data") or not hasattr(_sensor_data.joint_data, "joint_q"):
        return None
    joint_q = list(_sensor_data.joint_data.joint_q)
    # 服务需要14个手臂关节角度（索引 12:26）
    if len(joint_q) < 26:
        return None

    # 只使用14个手臂关节角度（双臂）
    arm_joints = [float(x) for x in joint_q[13:27]]

    # 根据左右手选择 link 名称
    if hand_side == 0:
        shoulder_link = "zarm_l1_ref_link"
        elbow_link = "zarm_l4_link"
        end_link = "zarm_l7_end_effector"
    else:
        shoulder_link = "zarm_r1_link"
        elbow_link = "zarm_r4_link"
        end_link = "zarm_r7_end_effector"

    base_link = "base_link"

    try:
        import rospy
        from kuavo_msgs.srv import fkSrvWithReferFrame, fkSrvWithReferFrameRequest

        rospy.wait_for_service("/ik/fk_srv_with_refer_frame", timeout=1.0)
        fk_client = rospy.ServiceProxy("/ik/fk_srv_with_refer_frame", fkSrvWithReferFrame)

        def call_fk(base_frame: str, end_frame: str, q_list) -> Optional[Tuple[np.ndarray, np.ndarray]]:
            req = fkSrvWithReferFrameRequest()
            req.q = q_list
            req.base_frame = base_frame
            req.end_effector_frame = end_frame
            req.hand_side = hand_side
            res = fk_client(req)
            if not res.success:
                return None
            pose = res.hand_poses.left_pose if hand_side == 0 else res.hand_poses.right_pose
            pos_m = np.array(pose.pos_xyz, dtype=float)
            quat = np.array(pose.quat_xyzw, dtype=float)
            return pos_m * 1000.0, quat  # 米 -> 毫米

        # 只使用14个手臂关节角度
        elbow_base = call_fk(base_link, elbow_link, arm_joints)
        end_base = call_fk(base_link, end_link, arm_joints)
        elbow_shoulder = call_fk(shoulder_link, elbow_link, arm_joints)
        end_shoulder = call_fk(shoulder_link, end_link, arm_joints)

        if elbow_base is None or end_base is None or elbow_shoulder is None or end_shoulder is None:
            return None

        return {
            "elbow_base": elbow_base,
            "end_base": end_base,
            "elbow_shoulder": elbow_shoulder,
            "end_shoulder": end_shoulder,
        }
    except Exception:
        return None
