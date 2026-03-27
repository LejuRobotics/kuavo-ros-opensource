"""
轮臂底盘运动控制案例

使用 NodeWheelMoveTimedCmd 通过 /mobile_manipulator_timed_single_cmd 服务发送底盘位姿。
支持 chassis_local (planner_index=1) 和 chassis_world (planner_index=0)。
"""
import os
import sys
import time
from typing import List, Optional

import py_trees

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import NodeWheelMoveTimedCmd, NodeFuntion
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import TimedCmdAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK


def generate_chassis_keypoints(chassis_poses: List[dict]):
    """
    生成底盘关键点列表
    
    Args:
        chassis_poses: 位姿列表，每个包含 time, pose([x, y, yaw]，米/弧度)
    Returns:
        (keypoints, times): 位姿列表、时间列表
    """
    keypoints = []
    times = []
    for pose in chassis_poses:
        keypoints.append(pose['pose'])
        times.append(pose['time'])
    return keypoints, times


def make_tree(timed_cmd_api, chassis_poses: Optional[List[dict]] = None, cmd_type: str = 'chassis_local'):
    """构建行为树"""
    poses = chassis_poses if chassis_poses is not None else CHASSIS_POSES
    keypoints, times = generate_chassis_keypoints(poses)
    
    keypoints_key = f"{cmd_type}_keypoints"
    times_key = f"{cmd_type}_keypoint_times"
    
    def set_keypoints_to_blackboard():
        bb = py_trees.blackboard.Client(name="set_chassis_keypoints")
        bb.register_key(keypoints_key, py_trees.common.Access.WRITE)
        bb.register_key(times_key, py_trees.common.Access.WRITE)
        setattr(bb, keypoints_key, keypoints)
        setattr(bb, times_key, times)
        print(f"设置 {len(keypoints)} 个底盘关键点 ({cmd_type})")
        return True
    
    root = py_trees.composites.Sequence(name="chassis_demo", memory=True)
    root.add_children([
        NodeFuntion(name="set_chassis_keypoints", fn=set_keypoints_to_blackboard),
        NodeWheelMoveTimedCmd(name="move_chassis", timed_cmd_api=timed_cmd_api, cmd_type=cmd_type),
    ])
    return root


if __name__ == '__main__':
    # 底盘关键点：time, pose([x, y, yaw]，米/弧度)
    CHASSIS_POSES = [
        {'time': 5.0, 'pose': [-0.3, 0.0, 0.0]},   # 前进0.3米
        {'time': 5.0, 'pose': [0.3, 0.0, 0.0]},  # 右移0.3米 + 旋转90度
        {'time': 5.0, 'pose': [-0.3, 0.0, 0.0]},  # 继续旋转90度
        {'time': 5.0, 'pose': [0.0, 0.0, 0.0]},  # 回原点方向
    ]
    CMD_TYPE = 'chassis_local'  # 或 'chassis_world'

    timed_cmd_api = TimedCmdAPI()
    root = make_tree(timed_cmd_api, cmd_type=CMD_TYPE)
    tree = py_trees.trees.BehaviourTree(root)
    
    print(f"开始底盘控制 ({CMD_TYPE})...")
    while True:
        tree.tick()
        status = root.status
        print(py_trees.display.unicode_tree(root, show_status=True))
        
        if status == py_trees.common.Status.SUCCESS:
            print("底盘运动完成!")
            break
        if status == py_trees.common.Status.FAILURE:
            print("底盘运动失败.")
            break
        time.sleep(0.1)
