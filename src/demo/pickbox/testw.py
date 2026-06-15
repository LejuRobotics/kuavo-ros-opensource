#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
轮臂版本抓取轨迹测试脚本
复用当前 BoxPickPlaceWheel 的抓取阶段：
open -> 可选掌心偏置 -> contact -> clamp -> lift
"""

from __future__ import print_function

import os
import sys

import rospy
import tf2_ros

from box_pick_place_wheel import BoxPickPlaceWheel, BoxPickPlaceError

# ============================================================
# 测试参数 — 直接修改这里即可
# ============================================================

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "wheel.yaml")

# 模拟箱子正面二维码在 waist_yaw_link 下的位置 (m)
QR_X = 0.55  # 或 0.50
QR_Y = 0.00
QR_Z = 0.40
# 抓取动作持续时间 (s)，设为 None 则与当前搬运主流程一致使用 2.0s
GRASP_DURATION = None

# 是否先执行安全位姿到达 (side -> safe -> prepare_high)
SKIP_SAFE_WAYPOINTS = False

# 抓取测试 IK 肘部约束点（wheel.yaml 中 wheel.ik.frame 对应的 IK frame）
LEFT_ELBOW = [0.24, 0.30, 0.28]
RIGHT_ELBOW = [0.24, -0.30, 0.28]

# open 后是否执行掌心关节偏置；None 表示读取 YAML 的 grasp.use_palm_joint_bias。
TEST_USE_PALM_JOINT_BIAS = None

# 末端位姿查询的参考系列表
REFERENCE_FRAMES = ("waist_yaw_link", "base_link", "odom")

# 末端执行器候选 TF frame 名
END_EFFECTOR_FRAMES = {
    "left": ["zarm_l7_end_effector", "zarm_l7_link"],
    "right": ["zarm_r7_end_effector", "zarm_r7_link"],
}


def lookup_eef_in_frame(tf_buffer, reference_frame, child_frames, timeout=0.2):
    """查询末端在指定参考系下的位姿。"""
    for child_frame in child_frames:
        try:
            t = tf_buffer.lookup_transform(
                reference_frame, child_frame,
                rospy.Time(0), rospy.Duration(float(timeout)),
            )
            tr = t.transform.translation
            rot = t.transform.rotation
            return {
                "child_frame": child_frame,
                "pos": [float(tr.x), float(tr.y), float(tr.z)],
                "quat": [float(rot.x), float(rot.y), float(rot.z), float(rot.w)],
            }
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            continue
    return None


def log_eef_poses_all_frames(tf_buffer, label="", ik_targets=None):
    """打印左右末端在 waist_yaw_link / base_link / odom 下的位置及与 IK 目标的误差。

    Args:
        tf_buffer: tf2_ros.Buffer 实例。
        label: 日志标签。
        ik_targets: 上一次 IK 目标 {"label", "left": [x,y,z], "right": [x,y,z]}。
    """
    targets = ik_targets or {}
    target_label = targets.get("label", "")

    for ref_frame in REFERENCE_FRAMES:
        for side in ("left", "right"):
            pose = lookup_eef_in_frame(
                tf_buffer, ref_frame, END_EFFECTOR_FRAMES[side]
            )
            if pose is None:
                rospy.logwarn("  %s %s: TF 查询失败 ref=%s", side, ref_frame, ref_frame)
                continue

            pos = pose["pos"]
            quat = pose["quat"]
            rospy.loginfo(
                "  末端[%s] ref=%s child=%s pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
                label if label else target_label,
                ref_frame,
                pose["child_frame"],
                pos[0], pos[1], pos[2],
                quat[0], quat[1], quat[2], quat[3],
            )

            # 仅在 waist_yaw_link 参考系下对比 IK 目标（因为 IK 输入是 waist_yaw_link 坐标）
            target = targets.get(side)
            if ref_frame == "waist_yaw_link" and target is not None and len(target) == 3:
                dx = pos[0] - float(target[0])
                dy = pos[1] - float(target[1])
                dz = pos[2] - float(target[2])
                dist = (dx * dx + dy * dy + dz * dz) ** 0.5
                rospy.loginfo(
                    "  末端误差[%s] %s vs IK_target: dx=%.4f dy=%.4f dz=%.4f dist=%.4f",
                    target_label, side, dx, dy, dz, dist,
                )


def main():
    try:
        node = BoxPickPlaceWheel(CONFIG)
    except BoxPickPlaceError as exc:
        rospy.logerr("初始化 BoxPickPlaceWheel 失败: %s", exc)
        return 1
    node.ik_left_elbow_pos_xyz = list(LEFT_ELBOW)
    node.ik_right_elbow_pos_xyz = list(RIGHT_ELBOW)
    node._normalize_wheel_palm_joint_bias_param()
    rospy.loginfo(
        "轮臂抓取测试使用IK肘部约束: left=%s right=%s",
        node.ik_left_elbow_pos_xyz,
        node.ik_right_elbow_pos_xyz,
    )

    try:
        node.disable_base_pitch_limit()
        node.set_arm_external_control()
    except BoxPickPlaceError as exc:
        rospy.logerr("初始化控制模式失败: %s", exc)
        node.terminate_robot(exc)
        return 1

    grasp_duration = (
        GRASP_DURATION
        if GRASP_DURATION is not None
        else 2.0
    )
    use_palm_joint_bias = (
        bool(node.params.get("grasp", {}).get("use_palm_joint_bias", True))
        if TEST_USE_PALM_JOINT_BIAS is None
        else bool(TEST_USE_PALM_JOINT_BIAS)
    )

    pick_qr = {"x": QR_X, "y": QR_Y, "z": QR_Z}
    rospy.loginfo(
        "轮臂模拟 QR: (%.3f, %.3f, %.3f) | skip_safe=%s | grasp_duration=%.2f | use_palm_joint_bias=%s",
        QR_X, QR_Y, QR_Z, SKIP_SAFE_WAYPOINTS,
        grasp_duration,
        use_palm_joint_bias,
    )

    try:
        if SKIP_SAFE_WAYPOINTS:
            rospy.loginfo("跳过安全位姿到达，直接计算抓取位姿")
            safe_current = None
        else:
            rospy.loginfo("先通过 /kuavo_arm_traj 到达安全预设姿态")
            safe_current = node.move_to_safe_arm_waypoints()
            rospy.loginfo("prepare_high 完成，使用该姿态作为 open 阶段起始参考")

        # ★ 打印 safe waypoint 后的末端位置
        rospy.loginfo("===== safe waypoint 后末端位置 =====")
        log_eef_poses_all_frames(node.tf_buffer, label="safe_waypoint")

        grasp_poses = node.compute_grasp(pick_qr)

        # open 阶段直接复用当前搬运代码的执行入口。
        ik_seed = node.execute_arm_sequence(
            grasp_poses,
            [("open", "open")],
            grasp_duration,
            start_joints=safe_current,
            seed_joints=safe_current,
        )
        if ik_seed is None:
            raise BoxPickPlaceError("open 阶段执行失败")

        rospy.loginfo("===== open 后末端位置 =====")
        log_eef_poses_all_frames(
            node.tf_buffer,
            label="open",
            ik_targets={
                "label": "open",
                "left": [float(v) for v in grasp_poses["open"][0]],
                "right": [float(v) for v in grasp_poses["open"][1]],
            },
        )

        palm_bias_joints = None
        if use_palm_joint_bias:
            rospy.loginfo("open 完成，施加当前搬运流程使用的掌心关节偏置")
            ik_seed = node.apply_joint_bias_after_clamp(
                ik_seed,
                joint6_bias_deg=-20.0,
                joint13_bias_deg=20.0,
                duration=2.0,
            )
            palm_bias_joints = list(ik_seed)
        else:
            rospy.loginfo("use_palm_joint_bias=false，跳过掌心关节偏置")

        # contact、clamp、lift 与当前搬运主流程保持一致：
        # 每个阶段使用上一阶段关节角作为 IK 种子，再执行关节空间插值。
        for label, text in [
            ("contact", "末端靠近接触箱面"),
            ("clamp", "双臂向内夹紧箱子"),
            ("lift", "保持夹紧并抬起箱子"),
        ]:
            rospy.loginfo("%s: 调用IK并执行", text)
            q_arm, _ = node.ik_solve(
                grasp_poses[label][0],
                grasp_poses[label][1],
                label,
                q0_joints=node.convert_traj_joints_for_ik_seed(ik_seed),
            )
            if use_palm_joint_bias and palm_bias_joints is not None:
                q_arm = node.preserve_palm_bias_joints(
                    q_arm, palm_bias_joints, label
                )
            node.move_arms_interpolated(q_arm, grasp_duration, 30)
            ik_seed = q_arm

            rospy.loginfo("===== %s 后末端位置 =====", label)
            log_eef_poses_all_frames(
                node.tf_buffer,
                label=label,
                ik_targets={
                    "label": label,
                    "left": [float(v) for v in grasp_poses[label][0]],
                    "right": [float(v) for v in grasp_poses[label][1]],
                },
            )

        rospy.loginfo("轮臂抓取测试完成，已执行 open -> contact -> clamp -> lift")
        rospy.sleep(4.0)
        node.terminate_robot("轮臂抓取测试完成")
    except BoxPickPlaceError as exc:
        rospy.logerr("抓取流程失败: %s", exc)
        node.terminate_robot(exc)
        return 1
    except rospy.ROSInterruptException:
        rospy.logerr("ROS 中断，流程退出")
        node.terminate_robot("ROS interrupt")
        return 1
    except KeyboardInterrupt:
        rospy.logerr("KeyboardInterrupt")
        node.terminate_robot("KeyboardInterrupt")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
