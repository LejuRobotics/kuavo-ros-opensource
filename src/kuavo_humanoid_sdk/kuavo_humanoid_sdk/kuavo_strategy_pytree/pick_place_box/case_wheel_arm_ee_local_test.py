"""
轮臂手臂末端控制案例：支持局部坐标系和世界坐标系

使用 NodeWheelMoveTimedCmd 通过 /mobile_manipulator_timed_single_cmd 服务发送手臂末端位姿。
命令格式：[左臂x,y,z,yaw,pitch,roll, 右臂x,y,z,yaw,pitch,roll]
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


def euler_rad_to_ypr(euler_rad):
    """将欧拉角(弧度) [roll, pitch, yaw] 转换为 [yaw, pitch, roll]（弧度）"""
    roll, pitch, yaw = euler_rad[0], euler_rad[1], euler_rad[2]
    return [yaw, pitch, roll]


def generate_arm_ee_keypoints(arm_poses: List[dict]):
    """
    生成手臂末端关键点列表

    Args:
        arm_poses: 位姿列表，每个包含 time 和 pose。
                  pose 为 12 维向量：[左手x,y,z, 左手roll,pitch,yaw(弧度), 右手x,y,z, 右手roll,pitch,yaw(弧度)]
    Returns:
        (keypoints, times): 命令向量列表、时间列表
    """
    keypoints = []
    times = []
    for item in arm_poses:
        pose = item['pose']  # 12 维
        left_pos = pose[0:3]
        left_euler_rad = pose[3:6]
        right_pos = pose[6:9]
        right_euler_rad = pose[9:12]
        left_ypr = euler_rad_to_ypr(left_euler_rad)
        right_ypr = euler_rad_to_ypr(right_euler_rad)
        cmd_vec = [*left_pos, *left_ypr, *right_pos, *right_ypr]
        keypoints.append(cmd_vec)
        times.append(item['time'])
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
    # 手臂关键点：每项 time + pose(12维向量)。
    # pose 顺序：左手位置(3) + 左手姿态 roll,pitch,yaw(弧度)(3) + 右手位置(3) + 右手姿态 roll,pitch,yaw(弧度)(3)
    _r90 = math.radians(-90)
    ARM_POSES = [
        {'time': 0.5, 'pose': [0.6626714800214014, 0.42901996207391835, 0.8817291969288493, 0, -1.5657004383511728, 0, 0.6958128321301926, -0.3702828909660909, 0.8858034947212338, 0, -1.5657004383511728, 0]},
        {'time': 0.5, 'pose': [0.6709568180485991, 0.22919424881391603, 0.8827477713769455, 0, -1.5657004383511728, 0, 0.6875274941029949, -0.17045717770608848, 0.8847849202731377, 0, -1.5657004383511728, 0]},
        {'time': 0.5, 'pose': [0.6709495240717068, 0.23021339599070445, 1.0827451745746617, 0, -1.5657004383511728, 0, 0.6875202001261026, -0.16943803052930018, 1.084782323470854, 0, -1.5657004383511728, 0]},
        {'time': 0.5, 'pose': [0.6, 0.2, 1.0, 0.0, -1.5707963267948966, 0.0, 0.6, -0.2, 1.0, 0.0, -1.5707963267948966, 0.0]},
        # {'time': 2.0, 'pose': [0.5, 0.2, 0.85, 0.0, _r90, 0.0, 0.5, -0.2, 0.85, 0.0, _r90, 0.0]},
        # {'time': 4.0, 'pose': [1.2, 0.2, 0.85, 0.0, _r90, 0.0, 1.2, -0.2, 0.85, 0.0, _r90, 0.0]},
        # {'time': 4.0, 'pose': [0.5, 0.2, 0.85, 0.0, _r90, 0.0, 0.5, -0.2, 0.85, 0.0, _r90, 0.0]},
    ]
    FRAME_ARM = WheelArmFrame.BASE  # 局部坐标系

    timed_cmd_api = TimedCmdAPI()
    root = make_tree(timed_cmd_api)
    tree = py_trees.trees.BehaviourTree(root)
    
    print("开始手臂末端控制...")
    actual_times = []
    while True:
        tree.tick()
        status = root.status
        print(py_trees.display.unicode_tree(root, show_status=True))
        
        if status == py_trees.common.Status.SUCCESS:
            print("手臂末端运动完成!")
            bb = py_trees.blackboard.Client(name="read_actual_times")
            bb.register_key("arm_ee_actual_times", py_trees.common.Access.READ)
            actual_times = getattr(bb, "arm_ee_actual_times", [])
            print("----- 每个关键点实际执行时间(s) -----")
            for i, t in enumerate(actual_times):
                print(f"  关键点 {i + 1}: {t:.3f}s")
            if actual_times:
                print(f"  总耗时: {sum(actual_times):.3f}s")
            break
        if status == py_trees.common.Status.FAILURE:
            print("手臂末端运动失败.")
            break
        time.sleep(0.1)
