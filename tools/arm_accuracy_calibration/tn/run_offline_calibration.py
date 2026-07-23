#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline arm accuracy calibration using fitted YAML data.

Requires ROS master and IK service running.

Usage:
    python3 run_offline_calibration.py fitting_data.yaml
"""

import os
import sys
import argparse
from typing import Dict, Tuple

import yaml
import numpy as np
import rospy

_script_dir = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_script_dir)
if _parent not in sys.path:
    sys.path.insert(0, _parent)

from arm_accuracy_calibration.functions import (
    load_config,
    solve_all_joints_in_base,
    solve_all_joints_in_shoulder,
    print_accuracy_table,
    get_arm_fk_with_refer_frame,
)
import arm_accuracy_calibration.functions.fk_solver as fk_module


def load_offline_data(yaml_path: str) -> Tuple[Dict[str, Dict[str, np.ndarray]], list]:
    """从 YAML 文件加载离线数据"""
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    rigid_body_poses: Dict[str, Dict[str, np.ndarray]] = {}
    for name, pose_data in data.get("rigid_body_poses", {}).items():
        rigid_body_poses[name] = {
            "pos": np.array(pose_data["pos"], dtype=float),
            "quat": np.array(pose_data["quat"], dtype=float),
        }

    arm_joint_angles = data.get("arm_joint_angles_14dof", [])
    return rigid_body_poses, arm_joint_angles


def resolve_yaml_path(arg_path: str) -> str:
    """
    解析 YAML 路径：
    - 如果是绝对路径或现有相对路径，直接使用
    - 否则尝试在 functions/offline_data/ 下查找
    """
    if os.path.isabs(arg_path) or os.path.exists(arg_path):
        return arg_path

    candidate = os.path.join(
        _script_dir, "functions", "offline_data", arg_path
    )
    if os.path.exists(candidate):
        return candidate

    # 最后退回原始路径（方便用户看到更明确的报错）
    return arg_path


def create_mock_sensor_data(joint_angles_14dof):
    """创建模拟的 sensor_data 对象，用于 FK 计算"""
    full_joint_q = [0.0] * 12 + list(joint_angles_14dof) + [0.0] * max(0, 26 - 12 - len(joint_angles_14dof))

    class MockJointData:
        def __init__(self, joint_q):
            self.joint_q = joint_q

    class MockSensorsData:
        def __init__(self, joint_q):
            self.joint_data = MockJointData(joint_q)

    return MockSensorsData(full_joint_q)


def main() -> None:
    parser = argparse.ArgumentParser(description="Offline arm accuracy calibration")
    parser.add_argument("yaml_file", help="Offline data YAML file")
    args = parser.parse_args()

    yaml_path = resolve_yaml_path(args.yaml_file)
    if not os.path.exists(yaml_path):
        print(f"[ERROR] YAML file not found: {yaml_path}")
        sys.exit(1)

    rospy.init_node("arm_accuracy_calibration_offline", anonymous=True)

    all_config = load_config()
    rigid_body_poses, arm_joint_angles = load_offline_data(yaml_path)

    result_base = solve_all_joints_in_base(all_config, rigid_body_poses)
    result_shoulder = solve_all_joints_in_shoulder(result_base)

    fk_module._sensor_data = create_mock_sensor_data(arm_joint_angles)
    fk_poses = get_arm_fk_with_refer_frame(hand_side=all_config.hand_side)

    print_accuracy_table(result_base, result_shoulder=result_shoulder, fk_poses=fk_poses)


if __name__ == "__main__":
    main()


