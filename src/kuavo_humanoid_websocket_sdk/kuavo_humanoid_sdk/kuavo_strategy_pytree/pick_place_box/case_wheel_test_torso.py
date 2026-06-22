import os
import sys
import time

import numpy as np
import py_trees

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import NodeTorsoPose
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import TorsoAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Frame

# === Demo 配置 ===
TORso_TARGET_POS = (0.0, 0.0, 1.2)  # x = 0.0m, z = 1.2m
TORso_TARGET_EULER = (
    0.0,  # roll
    np.deg2rad(0.0),  # pitch
    np.deg2rad(90.0),  # yaw
)

# 躯干归位
# TORso_TARGET_POS = (0.0, 0.0, 0.789919)  # x = 0.0m, z = 0.8m
# TORso_TARGET_EULER = (
#     0.0,  # roll
#     np.deg2rad(0.0),  # pitch
#     np.deg2rad(0.0),  # yaw
# )

def make_tree():
    robot_sdk = RobotSDK()
    torso_api = TorsoAPI(robot_sdk=robot_sdk)

    torso_target_pose = Pose.from_euler(
        pos=TORso_TARGET_POS,
        euler=TORso_TARGET_EULER,
        frame=Frame.BASE,
        degrees=False,
    )

    move_torso = NodeTorsoPose(
        name="move_torso_demo",
        torso_api=torso_api,
        target_pose=torso_target_pose,
        total_time=3.0,
    )

    root = py_trees.composites.Sequence(name="torso_demo", memory=True)
    root.add_children([move_torso])
    return root


if __name__ == '__main__':
    root = make_tree()
    tree = py_trees.trees.BehaviourTree(root)

    while True:
        tree.tick()
        status = root.status
        print(py_trees.display.unicode_tree(root, show_status=True))

        if status == py_trees.common.Status.SUCCESS:
            print("Torso motion completed successfully.")
            break
        if status == py_trees.common.Status.FAILURE:
            print("Torso motion failed.")
            break

        time.sleep(0.1)
