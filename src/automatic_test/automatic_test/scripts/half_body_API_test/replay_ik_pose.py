#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import time
import math
import random
import argparse
from typing import List, Dict

import rospy
import tf
import signal
from kuavo_msgs.msg import armTargetPoses
from kuavo_msgs.msg import twoArmHandPoseCmd
from kuavo_msgs.srv import twoArmHandPoseCmdSrv
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
import rospkg

DEFAULT_IN = rospkg.RosPack().get_path('automatic_test') + "/scripts/half_body_API_test/actions/ik_pose_samples.jsonl"
DEFAULT_OUT = rospkg.RosPack().get_path('automatic_test') + "/scripts/half_body_API_test/actions/ik_pose_replay_list.jsonl"

def read_jsonl(file_path: str) -> List[Dict]:
    samples: List[Dict] = []
    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                samples.append(json.loads(line))
            except Exception as e:
                rospy.logwarn(f"跳过无法解析的行: {e}")
    return samples


def get_total_count_by_last_index(file_path: str) -> int:
    """读取最后一行的 index 字段，返回条目总数（index+1）。若不存在则回退为行数。"""
    last_obj = None
    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                last_obj = json.loads(line)
            except Exception:
                continue
    if isinstance(last_obj, dict) and "index" in last_obj:
        try:
            return int(last_obj["index"]) + 1
        except Exception:
            pass
    # 回退为行数
    return sum(1 for _ in open(file_path, 'r', encoding='utf-8') if _.strip())


def set_arm_mode(mode: int) -> bool:
    srv_name = "/arm_traj_change_mode"
    try:
        rospy.wait_for_service(srv_name, timeout=3.0)
    except rospy.ROSException:
        rospy.logerr(f"服务 {srv_name} 不可用")
        return False
    try:
        cli = rospy.ServiceProxy(srv_name, changeArmCtrlMode)
        req = changeArmCtrlModeRequest()
        req.control_mode = mode
        res = cli(req)
        if res.result:
            rospy.loginfo(f"切换手臂控制模式为 {mode} 成功")
            return True
        rospy.logwarn(f"切换手臂控制模式为 {mode} 失败: {res.message}")
    except Exception as e:
        rospy.logerr(f"调用 {srv_name} 失败: {e}")
    return False


def publish_arm_target(values_deg, duration_s=5.0, wait_time=4.0):
    pub = rospy.Publisher('kuavo_arm_target_poses', armTargetPoses, queue_size=1)
    start = time.time()
    while pub.get_num_connections() == 0 and (time.time() - start) < 2.0 and not rospy.is_shutdown():
        rospy.sleep(0.05)
    msg = armTargetPoses()
    # times 设为总等待时间减去预留的收敛/评估时间
    run_time = max(0.0, float(duration_s) - float(wait_time))
    msg.times = [run_time]
    msg.values = list(values_deg)
    pub.publish(msg)


def call_ik_with_pose(left_pos, left_quat, right_pos, right_quat):
    srv_name = "/ik/two_arm_hand_pose_cmd_srv"
    try:
        rospy.wait_for_service(srv_name, timeout=3.0)
    except rospy.ROSException:
        rospy.logerr(f"服务 {srv_name} 不可用")
        return None
    try:
        cli = rospy.ServiceProxy(srv_name, twoArmHandPoseCmdSrv)
        req = twoArmHandPoseCmd()
        req.use_custom_ik_param = False
        req.joint_angles_as_q0 = False
        req.hand_poses.left_pose.pos_xyz = list(left_pos)
        req.hand_poses.left_pose.quat_xyzw = list(left_quat)
        req.hand_poses.left_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
        req.hand_poses.right_pose.pos_xyz = list(right_pos)
        req.hand_poses.right_pose.quat_xyzw = list(right_quat)
        req.hand_poses.right_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
        res = cli(req)
        return res
    except Exception as e:
        rospy.logerr(f"调用 {srv_name} 失败: {e}")
        return None


def radians_list_to_degrees(rad_list):
    return [math.degrees(x) for x in rad_list]


def normalize_quaternion_xyzw(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [x/n, y/n, z/n, w/n]


def quat_angle_error_deg(q_sample_xyzw, q_tf_xyzw):
    qs = normalize_quaternion_xyzw(list(q_sample_xyzw))
    qt = normalize_quaternion_xyzw(list(q_tf_xyzw))
    dot = abs(qs[0]*qt[0] + qs[1]*qt[1] + qs[2]*qt[2] + qs[3]*qt[3])
    dot = max(-1.0, min(1.0, dot))
    angle = 2.0 * math.degrees(math.acos(dot))
    if angle > 180.0:
        angle = 360.0 - angle
    return angle


def lookup_pose(listener: tf.TransformListener, target_frame: str, source_frame: str, timeout_s: float = 0.5):
    start = time.time()
    while (time.time() - start) < timeout_s and not rospy.is_shutdown():
        try:
            listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.1))
            (trans, rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            return (list(trans), list(rot))
        except Exception:
            rospy.sleep(0.02)
    return None


# Ctrl+C 友好退出支持
_STOP_REQUESTED = False


def _sigint_handler(signum, frame):
    global _STOP_REQUESTED
    _STOP_REQUESTED = True
    try:
        rospy.signal_shutdown("SIGINT")
    except Exception:
        pass


def safe_sleep(seconds: float):
    end_t = time.time() + float(seconds)
    step = 0.05
    while time.time() < end_t and not rospy.is_shutdown() and not _STOP_REQUESTED:
        time.sleep(min(step, max(0.0, end_t - time.time())))


# 终端彩色输出
_C_RESET = "\033[0m"
_C_RED = "\033[31m"
_C_GREEN = "\033[32m"
_C_YELLOW = "\033[33m"
_C_CYAN = "\033[36m"


def parse_args():
    parser = argparse.ArgumentParser(description="读取 JSONL 末端位姿并调用 IK 使手臂运动")
    parser.add_argument("--file", type=str, default=DEFAULT_IN, help="输入 JSONL 文件路径")
    parser.add_argument("--index", type=int, default=-1, help="使用的样本索引（-1 表示随机）")
    parser.add_argument("--ratio", type=float, default=0.05, help="随机抽取比例 (0,1]，用于生成回放清单并逐条执行")
    parser.add_argument("--out", type=str, default=DEFAULT_OUT, help="抽样结果输出 JSONL 文件")
    parser.add_argument("--sleep", type=float, default=10.0, help="每次运动后停留时间（秒）")
    parser.add_argument("--wait_time", type=float, default=4.0, help="在总等待时间中预留的收敛/评估时间（秒）")
    parser.add_argument("--base_frame", type=str, default="base_link", help="基坐标系名")
    parser.add_argument("--right_ee_frame", type=str, default="zarm_r7_end_effector", help="右手末端坐标系名")
    parser.add_argument("--left_ee_frame", type=str, default="zarm_l7_end_effector", help="左手末端坐标系名")
    parser.add_argument("--pos_err_thresh", type=float, default=0.10, help="位置误差阈值(米)")
    parser.add_argument("--ori_err_thresh_deg", type=float, default=10.0, help="姿态误差阈值(度)")
    parser.add_argument("--check_ori", action="store_true", help="开启后在达成判定中比较姿态误差；默认关闭仅比较位置误差")
    parser.add_argument("--ik_req_rate", type=float, default=0.90, help="IK 成功率阈值 (0~1)，用于总体判定")
    parser.add_argument("--reach_req_rate", type=float, default=0.90, help="到达成功率阈值 (0~1)，基于 IK 成功样本")
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


if __name__ == "__main__":
    rospy.init_node("replay_ik_pose_node", anonymous=True)
    args = parse_args()
    tf_listener = tf.TransformListener()
    # 捕获 Ctrl+C
    try:
        signal.signal(signal.SIGINT, _sigint_handler)
    except Exception:
        pass

    assert os.path.exists(args.file), f"文件不存在: {args.file}"
    all_samples = read_jsonl(args.file)
    assert len(all_samples) > 0, f"文件中没有可用样本: {args.file}"

    # 判定是否为回放清单：文件中任意一行包含 ik_success 字段即认为是回放清单
    is_replay_list = any(isinstance(obj, dict) and ("ik_success" in obj) for obj in all_samples)

    def write_selected(fn: str, data: List[Dict]):
        os.makedirs(os.path.dirname(fn), exist_ok=True)
        with open(fn, 'w', encoding='utf-8') as wf:
            for rec in data:
                wf.write(json.dumps(rec, ensure_ascii=False) + "\n")

    if is_replay_list:
        # 直接使用回放清单，跳过可能存在的汇总头部行（不含 ik_success）
        selected: List[Dict] = []
        for i, obj in enumerate(all_samples):
            if not (isinstance(obj, dict) and ("ik_success" in obj)):
                continue
            idx_val = int(obj.get("index", i))
            new_obj = {
                "index": idx_val,
                "ik_success": bool(obj.get("ik_success", False)),
            }
            if "q_arm" in obj:
                new_obj["q_arm"] = obj["q_arm"]
            if "left" in obj:
                new_obj["left"] = obj["left"]
            if "right" in obj:
                new_obj["right"] = obj["right"]
            if "right_pos_err" in obj:
                new_obj["right_pos_err"] = obj["right_pos_err"]
            if "right_ori_err_deg" in obj:
                new_obj["right_ori_err_deg"] = obj["right_ori_err_deg"]
            if "right_in_limit" in obj:
                new_obj["right_in_limit"] = obj["right_in_limit"]
            if "left_pos_err" in obj:
                new_obj["left_pos_err"] = obj["left_pos_err"]
            if "left_ori_err_deg" in obj:
                new_obj["left_ori_err_deg"] = obj["left_ori_err_deg"]
            if "left_in_limit" in obj:
                new_obj["left_in_limit"] = obj["left_in_limit"]
            selected.append(new_obj)
        args.out = args.file
        rospy.loginfo(f"读取回放清单，共 {len(selected)} 条（已跳过汇总头部行，如存在）：{args.out}")
    else:
        # 从采集文件抽样生成回放清单
        total_lines = len(all_samples)
        # 也参考末行 index，但以实际行数为准以避免越界
        total_by_last = get_total_count_by_last_index(args.file)
        total = min(total_lines, total_by_last)
        ratio = max(0.0, min(1.0, float(args.ratio)))
        if ratio <= 0.0:
            rospy.logwarn("ratio <= 0，调整为 1.0（使用全部数据）")
            ratio = 1.0
        k = max(1, int(total * ratio))
        # 若显式指定了 index，则严格只跑该索引；越界直接报错退出
        if args.index is not None and args.index >= 0:
            if not (0 <= args.index < total):
                rospy.logerr(f"--index {args.index} 超出范围 [0, {total-1}]，退出")
                sys.exit(1)
            indices = [args.index]
        else:
            indices = random.sample(range(total), k)

        selected = []
        for i in indices:
            obj = all_samples[i]
            idx_val = int(obj.get("index", i))
            new_obj = {
                "index": idx_val,
                "ik_success": False,
            }
            if "q_arm" in obj:
                new_obj["q_arm"] = obj["q_arm"]
            if "left" in obj:
                new_obj["left"] = obj["left"]
            if "right" in obj:
                new_obj["right"] = obj["right"]
            selected.append(new_obj)

        write_selected(args.out, selected)
        rospy.loginfo(f"抽样 {len(selected)}/{total} 条，已写入: {args.out}")

    # 切换为外部控制模式
    set_arm_mode(2)

    # 逐条执行 IK 并更新 ik_success
    for rec in selected:
        if rospy.is_shutdown() or _STOP_REQUESTED:
            break
        left_pos = rec.get("left", {}).get("pos_xyz", None)
        left_quat = rec.get("left", {}).get("quat_xyzw", None)
        right_pos = rec.get("right", {}).get("pos_xyz", None)
        right_quat = rec.get("right", {}).get("quat_xyzw", None)

        if not (left_pos and left_quat and right_pos and right_quat):
            rospy.logwarn(f"索引 {rec['index']} 数据不完整，跳过并发布全 0")
            publish_arm_target([0.0]*14, duration_s=args.sleep, wait_time=args.wait_time)
            safe_sleep(args.sleep)
            continue

        rospy.loginfo(f"使用样本索引 {rec['index']} 调用 IK 服务")
        res = call_ik_with_pose(left_pos, left_quat, right_pos, right_quat)
        # print(res)

        if res is not None and res.success:
            q_arm_deg = radians_list_to_degrees(res.q_arm)
            publish_arm_target(q_arm_deg, duration_s=args.sleep, wait_time=args.wait_time)
            rec["ik_success"] = True
            rospy.loginfo("IK 成功，已发布目标关节角")
        else:
            rospy.logwarn("IK 失败，发布全 0 关节角")
            publish_arm_target([0.0]*14, duration_s=args.sleep, wait_time=args.wait_time)

        # 等待运动完成
        safe_sleep(args.sleep)

        # TF 误差评估（与样本 pose 对比）
        try:
            # 右手
            tf_pose_r = lookup_pose(tf_listener, args.base_frame, args.right_ee_frame, timeout_s=0.8)
            if tf_pose_r is not None and "right" in rec:
                (r_trans, r_rot) = tf_pose_r
                sample_r = rec["right"]
                pos_xyz_sample = sample_r.get("pos_xyz", None)
                quat_xyzw_sample = sample_r.get("quat_xyzw", None)
                if pos_xyz_sample and quat_xyzw_sample:
                    rx = r_trans[0] - pos_xyz_sample[0]
                    ry = r_trans[1] - pos_xyz_sample[1]
                    rz = r_trans[2] - pos_xyz_sample[2]
                    r_pos_err = math.sqrt(rx*rx + ry*ry + rz*rz)
                    r_ori_err_deg = quat_angle_error_deg(quat_xyzw_sample, r_rot)
                    rec["right_pos_err"] = float(r_pos_err)
                    rec["right_ori_err_deg"] = float(r_ori_err_deg)
                    r_pos_ok = (r_pos_err <= args.pos_err_thresh)
                    r_ori_ok = (r_ori_err_deg <= args.ori_err_thresh_deg)
                    rec["right_in_limit"] = bool(r_pos_ok and (r_ori_ok if args.check_ori else True))

            # 左手
            tf_pose_l = lookup_pose(tf_listener, args.base_frame, args.left_ee_frame, timeout_s=0.8)
            if tf_pose_l is not None and "left" in rec:
                (l_trans, l_rot) = tf_pose_l
                sample_l = rec["left"]
                pos_xyz_sample = sample_l.get("pos_xyz", None)
                quat_xyzw_sample = sample_l.get("quat_xyzw", None)
                if pos_xyz_sample and quat_xyzw_sample:
                    lx = l_trans[0] - pos_xyz_sample[0]
                    ly = l_trans[1] - pos_xyz_sample[1]
                    lz = l_trans[2] - pos_xyz_sample[2]
                    l_pos_err = math.sqrt(lx*lx + ly*ly + lz*lz)
                    l_ori_err_deg = quat_angle_error_deg(quat_xyzw_sample, l_rot)
                    rec["left_pos_err"] = float(l_pos_err)
                    rec["left_ori_err_deg"] = float(l_ori_err_deg)
                    l_pos_ok = (l_pos_err <= args.pos_err_thresh)
                    l_ori_ok = (l_ori_err_deg <= args.ori_err_thresh_deg)
                    rec["left_in_limit"] = bool(l_pos_ok and (l_ori_ok if args.check_ori else True))
        except Exception as e:
            rospy.logwarn(f"TF 误差评估失败: {e}")

        # 每次更新一次输出文件（将成功状态与误差落盘）
        write_selected(args.out, selected)

    # 可选：恢复模式 1
    set_arm_mode(1)

    # 运行结束后统计并将汇总写入输出文件首行（到达率仅统计 IK 成功的样本）
    try:
        total = len(selected)
        ik_succ = sum(1 for rec in selected if rec.get("ik_success", False))
        # 仅在 IK 成功的样本中统计到达情况
        reach_total = ik_succ
        reach_succ = sum(
            1 for rec in selected
            if rec.get("ik_success", False) and rec.get("right_in_limit", False) and rec.get("left_in_limit", False)
        )
        ik_rate = (float(ik_succ) / float(total)) if total > 0 else 0.0
        reach_rate = (float(reach_succ) / float(reach_total)) if reach_total > 0 else 0.0
        overall_pass = (ik_rate >= float(args.ik_req_rate)) and (reach_rate >= float(args.reach_req_rate))
        summary = {
            "summary": {
                "total": int(total),
                "ik_success": int(ik_succ),
                "ik_success_rate": ik_rate,
                "reach_success": int(reach_succ),
                "reach_total": int(reach_total),
                "reach_success_rate": reach_rate,
                "ik_req_rate": float(args.ik_req_rate),
                "reach_req_rate": float(args.reach_req_rate),
                "test_pass": bool(overall_pass),
            }
        }

        # 读取现有内容并在首行插入汇总
        try:
            with open(args.out, 'r', encoding='utf-8') as rf:
                lines = rf.readlines()
        except Exception:
            lines = []
        with open(args.out, 'w', encoding='utf-8') as wf:
            wf.write(json.dumps(summary, ensure_ascii=False) + "\n")
            for ln in lines:
                if ln.strip():
                    wf.write(ln if ln.endswith('\n') else (ln + '\n'))

        # 控制台彩色汇总输出
        failed_reach_indices = [
            int(rec.get("index", -1))
            for rec in selected
            if rec.get("ik_success", False) and not (rec.get("right_in_limit", False) and rec.get("left_in_limit", False))
        ]
        print(_C_CYAN + "===== 回放汇总 =====" + _C_RESET)
        print((_C_GREEN if ik_rate >= float(args.ik_req_rate) else _C_YELLOW) +
              f"IK 成功: {ik_succ}/{total}  (rate={ik_rate:.2%}, 要求>={float(args.ik_req_rate):.0%})" + _C_RESET)
        print((_C_GREEN if reach_rate >= float(args.reach_req_rate) and reach_total>0 else _C_YELLOW) +
              f"到达成功(双手在限, 仅计 IK 成功样本): {reach_succ}/{reach_total}  (rate={reach_rate:.2%}, 要求>={float(args.reach_req_rate):.0%})" + _C_RESET)
        print((_C_GREEN if overall_pass else _C_RED) + ("测试结果: 通过" if overall_pass else "测试结果: 未通过") + _C_RESET)
        if failed_reach_indices:
            print(_C_RED + "IK 成功但未到达的 index: " + ", ".join(map(str, failed_reach_indices)) + _C_RESET)
        else:
            print(_C_GREEN + "所有 IK 成功样本均到达阈值内" + _C_RESET)
    except Exception as e:
        rospy.logwarn(f"写入汇总行失败: {e}")

    

