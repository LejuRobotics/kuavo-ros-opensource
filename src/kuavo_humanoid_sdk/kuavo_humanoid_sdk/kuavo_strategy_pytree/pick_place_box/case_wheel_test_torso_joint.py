import os
import sys
import time

import numpy as np
import py_trees

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import NodeTorsoJoint
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import TorsoAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK

# === Demo 配置 ===
# 定义目标关节角度：4个关节（下肢关节）的目标位置（度）
TARGET_JOINT_ANGLES = [14.90, -32.01, 18.03, -90.0]  # 目标关节角度（度）
# TARGET_JOINT_ANGLES = [0.0, 0.0, 0.0, 0.0]  # 目标关节角度（度）

TOTAL_TIME = 3.0  # 总执行时间：8秒


def make_tree():
    robot_sdk = RobotSDK()
    torso_api = TorsoAPI(robot_sdk=robot_sdk)

    move_torso_joint = NodeTorsoJoint(
        name="move_torso_joint_demo",
        torso_api=torso_api,
        joint_trajectory=TARGET_JOINT_ANGLES,  # 传递单组关节角度
        total_time=TOTAL_TIME,
    )

    root = py_trees.composites.Sequence(name="torso_joint_demo", memory=True)
    root.add_children([move_torso_joint])
    return root


if __name__ == '__main__':
    print("=== 躯干关节控制示例 ===")
    print(f"目标关节角度: {TARGET_JOINT_ANGLES}")
    print(f"关节名称: ['joint1', 'joint2', 'joint3', 'joint4']")
    print("将发布到话题: /lb_leg_traj")
    print()

    root = make_tree()
    tree = py_trees.trees.BehaviourTree(root)

    while True:
        tree.tick()
        status = root.status
        print(py_trees.display.unicode_tree(root, show_status=True))

        if status == py_trees.common.Status.SUCCESS:
            print("躯干关节运动完成成功。")
            break
        if status == py_trees.common.Status.FAILURE:
            print("躯干关节运动失败。")
            break

        time.sleep(0.1)
