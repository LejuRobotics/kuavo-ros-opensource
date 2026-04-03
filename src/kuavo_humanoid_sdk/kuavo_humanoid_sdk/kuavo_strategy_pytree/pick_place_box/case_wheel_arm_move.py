"""
轮臂上肢关节运动控制案例

使用 NodeWheelMoveTimedCmd 通过 /mobile_manipulator_timed_single_cmd 服务发送上肢关节角度。
14 维关键点（左臂 7 + 右臂 7）在 API 层拆为左臂 planner 8、右臂 planner 9 各发一次。
"""
import os
import sys
import time
import math
from typing import List, Optional

import py_trees

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import NodeWheelMoveTimedCmd, NodeFuntion
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import TimedCmdAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK


def generate_arm_joint_keypoints(arm_poses: List[dict]):
    """
    生成上肢关键点列表
    
    Args:
        arm_poses: 位姿列表，每个包含 time, joints(14个关节角度，度数)
    Returns:
        (keypoints, times): 关节角度列表（弧度）、时间列表
    """
    keypoints = []
    times = []
    for pose in arm_poses:
        joints_rad = [math.radians(angle) for angle in pose['joints']]
        keypoints.append(joints_rad)
        times.append(pose['time'])
    return keypoints, times


def make_tree(timed_cmd_api, arm_poses: Optional[List[dict]] = None):
    """构建行为树"""
    poses = arm_poses if arm_poses is not None else ARM_POSES
    keypoints, times = generate_arm_joint_keypoints(poses)
    
    def set_keypoints_to_blackboard():
        bb = py_trees.blackboard.Client(name="set_arm_joint_keypoints")
        bb.register_key("arm_joint_keypoints", py_trees.common.Access.WRITE)
        bb.register_key("arm_joint_keypoint_times", py_trees.common.Access.WRITE)
        bb.arm_joint_keypoints = keypoints
        bb.arm_joint_keypoint_times = times
        print(f"设置 {len(keypoints)} 个上肢关键点")
        return True
    
    root = py_trees.composites.Sequence(name="arm_joint_demo", memory=True)
    root.add_children([
        NodeFuntion(name="set_arm_joint_keypoints", fn=set_keypoints_to_blackboard),
        NodeWheelMoveTimedCmd(name="move_arm_joint", timed_cmd_api=timed_cmd_api, cmd_type='arm'),
    ])
    return root


if __name__ == '__main__':
    # 上肢关键点：time, joints(14个关节角度，度数，左臂7个+右臂7个)
    ARM_POSES = [
        {'time': 2.0, 'joints': [-30, 20, 15, -45, 25, 10, -35,
                                  -30, -20, -15, -45, -25, -10, -35]},
        {'time': 2.0, 'joints': [-20, 30, -25, -20, 40, -15, 25,
                                  -20, -30, 25, -20, -40, 15, 25]},
        {'time': 2.0, 'joints': [0.0] * 14},
    ]

    timed_cmd_api = TimedCmdAPI()
    root = make_tree(timed_cmd_api)
    tree = py_trees.trees.BehaviourTree(root)
    
    print("开始上肢关节控制...")
    while True:
        tree.tick()
        status = root.status
        print(py_trees.display.unicode_tree(root, show_status=True))
        
        if status == py_trees.common.Status.SUCCESS:
            print("上肢关节运动完成!")
            break
        if status == py_trees.common.Status.FAILURE:
            print("上肢关节运动失败.")
            break
        time.sleep(0.1)
