"""
轮臂手臂末端控制案例：支持局部坐标系和世界坐标系

使用 NodeWheelMoveTimedCmd 通过 /mobile_manipulator_timed_single_cmd 服务发送手臂末端位姿。
命令格式 12 维：[左臂 x,y,z,yaw,pitch,roll, 右臂 x,y,z,yaw,pitch,roll]，API 内拆为左/右单臂（planner 4/5 世界系或 6/7 局部系）各发一次。
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
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import WheelArmFrame


def euler_to_ypr(euler_deg):
    """将欧拉角(度) [roll, pitch, yaw] 转换为 [yaw, pitch, roll]（弧度）"""
    roll, pitch, yaw = [math.radians(a) for a in euler_deg]
    return [yaw, pitch, roll]


def generate_arm_ee_keypoints(arm_poses: List[dict]):
    """
    生成手臂末端关键点列表
    
    Args:
        arm_poses: 位姿列表，每个包含 time, pos_left, euler_left, pos_right, euler_right
    Returns:
        (keypoints, times): 命令向量列表、时间列表
    """
    keypoints = []
    times = []
    for pose in arm_poses:
        left_ypr = euler_to_ypr(pose['euler_left'])
        right_ypr = euler_to_ypr(pose['euler_right'])
        cmd_vec = [*pose['pos_left'], *left_ypr, *pose['pos_right'], *right_ypr]
        keypoints.append(cmd_vec)
        times.append(pose['time'])
    return keypoints, times


def make_tree(timed_cmd_api, arm_poses: Optional[List[dict]] = None, frame: Optional[WheelArmFrame] = None):
    """构建行为树"""
    poses = arm_poses if arm_poses is not None else ARM_POSES
    arm_frame = frame if frame is not None else FRAME_ARM
    
    cmd_type = 'arm_ee_world' if arm_frame == WheelArmFrame.ODOM else 'arm_ee_local'
    keypoints_key = f'{cmd_type}_keypoints'
    keypoints, times = generate_arm_ee_keypoints(poses)
    
    def set_keypoints_to_blackboard():
        bb = py_trees.blackboard.Client(name="set_arm_ee_keypoints")
        bb.register_key(keypoints_key, py_trees.common.Access.WRITE)
        bb.register_key("arm_ee_keypoint_times", py_trees.common.Access.WRITE)
        setattr(bb, keypoints_key, keypoints)
        bb.arm_ee_keypoint_times = times
        print(f"设置 {len(keypoints)} 个手臂末端关键点 ({cmd_type})")
        return True
    
    root = py_trees.composites.Sequence(name="arm_ee_demo", memory=True)
    root.add_children([
        NodeFuntion(name="set_arm_ee_keypoints", fn=set_keypoints_to_blackboard),
        NodeWheelMoveTimedCmd(name="move_arm_ee", timed_cmd_api=timed_cmd_api, cmd_type=cmd_type),
    ])
    return root


if __name__ == '__main__':
    # 手臂关键点：time, pos_left(x,y,z), euler_left(roll,pitch,yaw度), pos_right, euler_right
    ARM_POSES = [
        {'time': 2.0, 'pos_left': (0.1, 0.4, 0.7), 'euler_left': (0, 0, 0),
                      'pos_right': (0.1, -0.4, 0.7), 'euler_right': (0, 0, 0)},
        {'time': 2.0, 'pos_left': (0.3, 0.4, 0.7), 'euler_left': (0, -90, 0),
                      'pos_right': (0.3, -0.4, 0.7), 'euler_right': (0, -90, 0)},
        {'time': 2.0, 'pos_left': (0.3, 0.2, 0.7), 'euler_left': (0, -90, 0),
                      'pos_right': (0.3, -0.2, 0.7), 'euler_right': (0, -90, 0)},
        {'time': 2.0, 'pos_left': (0.5, 0.2, 0.7), 'euler_left': (0, -90, 0),
                      'pos_right': (0.5, -0.2, 0.7), 'euler_right': (0, -90, 0)},
        {'time': 2.0, 'pos_left': (0.5, 0.2, 0.85), 'euler_left': (0, -90, 0),
                      'pos_right': (0.5, -0.2, 0.85), 'euler_right': (0, -90, 0)},
        {'time': 4.0, 'pos_left': (1.2, 0.2, 0.85), 'euler_left': (0, -90, 0),
                      'pos_right': (1.2, -0.2, 0.85), 'euler_right': (0, -90, 0)},
        {'time': 4.0, 'pos_left': (0.5, 0.2, 0.85), 'euler_left': (0, -90, 0),
                      'pos_right': (0.5, -0.2, 0.85), 'euler_right': (0, -90, 0)},
    ]
    FRAME_ARM = WheelArmFrame.ODOM  # 世界坐标系

    timed_cmd_api = TimedCmdAPI()
    root = make_tree(timed_cmd_api)
    tree = py_trees.trees.BehaviourTree(root)
    
    print("开始手臂末端控制...")
    while True:
        tree.tick()
        status = root.status
        print(py_trees.display.unicode_tree(root, show_status=True))
        
        if status == py_trees.common.Status.SUCCESS:
            print("手臂末端运动完成!")
            break
        if status == py_trees.common.Status.FAILURE:
            print("手臂末端运动失败.")
            break
        time.sleep(0.1)
