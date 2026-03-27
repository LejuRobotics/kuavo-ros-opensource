"""
轮臂下肢运动控制案例

使用 NodeWheelMoveTimedCmd 通过 /mobile_manipulator_timed_single_cmd 服务发送下肢关节角度。
planner_index=3，4个关节。
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


def generate_leg_keypoints(leg_poses: List[dict]):
    """
    生成下肢关键点列表
    
    Args:
        leg_poses: 位姿列表，每个包含 time, joints(4个关节角度，度数)
    Returns:
        (keypoints, times): 关节角度列表（弧度）、时间列表
    """
    keypoints = []
    times = []
    for pose in leg_poses:
        joints_rad = [math.radians(angle) for angle in pose['joints']]
        keypoints.append(joints_rad)
        times.append(pose['time'])
    return keypoints, times


def make_tree(timed_cmd_api, leg_poses: Optional[List[dict]] = None):
    """构建行为树"""
    poses = leg_poses if leg_poses is not None else LEG_POSES
    keypoints, times = generate_leg_keypoints(poses)
    
    def set_keypoints_to_blackboard():
        bb = py_trees.blackboard.Client(name="set_leg_keypoints")
        bb.register_key("leg_keypoints", py_trees.common.Access.WRITE)
        bb.register_key("leg_keypoint_times", py_trees.common.Access.WRITE)
        bb.leg_keypoints = keypoints
        bb.leg_keypoint_times = times
        print(f"设置 {len(keypoints)} 个下肢关键点")
        return True
    
    root = py_trees.composites.Sequence(name="leg_demo", memory=True)
    root.add_children([
        NodeFuntion(name="set_leg_keypoints", fn=set_keypoints_to_blackboard),
        NodeWheelMoveTimedCmd(name="move_leg", timed_cmd_api=timed_cmd_api, cmd_type='leg'),
    ])
    return root


if __name__ == '__main__':
    # 下肢关键点：time, joints(4个关节角度，度数)
    LEG_POSES = [
        {'time': 2.0, 'joints': [14.90, -32.01, 18.03, 0.0]},
        {'time': 2.0, 'joints': [14.90, -32.01, 18.03, 30.0]},
        {'time': 2.0, 'joints': [14.90, -32.01, 18.03, -30.0]},
        {'time': 2.0, 'joints': [14.90, -32.01, 18.03, 0.0]},
        {'time': 2.0, 'joints': [0.0, 0.0, 0.0, 0.0]},
    ]

    timed_cmd_api = TimedCmdAPI()
    root = make_tree(timed_cmd_api)
    tree = py_trees.trees.BehaviourTree(root)
    
    print("开始下肢控制...")
    while True:
        tree.tick()
        status = root.status
        print(py_trees.display.unicode_tree(root, show_status=True))
        
        if status == py_trees.common.Status.SUCCESS:
            print("下肢运动完成!")
            break
        if status == py_trees.common.Status.FAILURE:
            print("下肢运动失败.")
            break
        time.sleep(0.1)
