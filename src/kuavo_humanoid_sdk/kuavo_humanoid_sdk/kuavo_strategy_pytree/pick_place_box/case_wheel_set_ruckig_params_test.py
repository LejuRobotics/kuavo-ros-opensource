"""
轮臂Ruckig规划器参数设置案例

使用 NodeSetRuckigParams 节点设置不同规划器的运动参数（速度、加速度、急动度等）。
可以在运动控制前设置规划器参数，以调整运动特性。
"""
import os
import sys
import time
from typing import List, Optional

import numpy as np
import py_trees

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import NodeSetRuckigParams, NodeWheelMoveTimedCmd, NodeFuntion
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import TimedCmdAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Frame


def _yaw_diff_rad(current: float, target: float) -> float:
    """yaw 角度差（弧度），归一化到 [-pi, pi]。"""
    d = current - target
    while d > np.pi:
        d -= 2 * np.pi
    while d < -np.pi:
        d += 2 * np.pi
    return d


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


def make_tree(
    timed_cmd_api,
    chassis_poses: Optional[List[dict]] = None,
    cmd_type: str = 'chassis_world',
    scale: float = 1.0,
    is_sync: bool = True,
):
    """构建行为树。

    Args:
        is_sync: True=三轴时间同步，同时开始同时结束；False=非同步，给定时间为「最慢轴」时间，
                 旋转等较快轴会先于该时间完成（例如总时间 5s 时 yaw 可能 3s 就到位）。
    """
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
    
    # 根据cmd_type确定planner_index（臂类为左右单臂：4/5 世界系、6/7 局部系、8/9 关节）
    planner_index_map = {
        'chassis_world': 0,
        'chassis_local': 1,
        'torso': 2,
        'leg': 3,
        'arm_ee_world': 4,   # 左臂世界系，右臂为 5
        'arm_ee_local': 6,   # 左臂局部系，右臂为 7
        'arm': 8,            # 左臂关节，右臂为 9
    }
    planner_index = planner_index_map.get(cmd_type, 0)
    
    # 基础规划器参数 [x, y, yaw]
    base_velocity_max = [0.8, 1.3, 2.0]   # 速度上限为（2.0， 2.0， 2.1）
    base_acceleration_max = [0.9, 2.0, 5.0]  #实机底盘x, y, yaw 3个自由度的最大加速度 (2.0m/s², 2.0m/s², 5.0rad/s²)
    base_jerk_max = [3.0, 50.0, 70.0]

    # 线性缩放后的规划器参数
    velocity_max = [v * scale for v in base_velocity_max]
    acceleration_max = [a * scale for a in base_acceleration_max]
    jerk_max = [j * scale for j in base_jerk_max]

    # 设置规划器参数节点
    # is_sync=False 时：给定时间由「最慢轴」用满，旋转等较快轴会提前到位
    set_params_node = NodeSetRuckigParams(
        name="set_ruckig_params",
        timed_cmd_api=timed_cmd_api,
        planner_index=planner_index,
        is_sync=is_sync,
        velocity_max=velocity_max,  # 底盘x, y, yaw 3个自由度的最大速度 (m/s, m/s, rad/s)
        acceleration_max=acceleration_max,  # 最大加速度 (m/s², m/s², rad/s²)
        jerk_max=jerk_max,  # 最大急动度 (m/s³, m/s³, rad/s³)
    )

    root = py_trees.composites.Sequence(name="chassis_with_params_demo", memory=True)
    root.add_children([
        set_params_node,  # 先设置规划器参数
        NodeFuntion(name="set_chassis_keypoints", fn=set_keypoints_to_blackboard),
        NodeWheelMoveTimedCmd(name="move_chassis", timed_cmd_api=timed_cmd_api, cmd_type=cmd_type),
    ])
    return root


if __name__ == '__main__':
    # 底盘关键点：time, pose([x, y, yaw]，米/弧度)
    # 若 IS_SYNC=False：time 为该段总时间，最慢轴（通常 x/y）用满该时间，yaw 在限幅内会提前到位
    CHASSIS_POSES = [
        {'time': 3.0, 'pose': [0.0, -0.0, 3.14]},  
        {'time': 1.0, 'pose': [0.0, -0.0, 3.14]},  
        {'time': 3.0, 'pose': [0.0, -0.0, 0]},   
        {'time': 3.5, 'pose': [0.0, -1.0, -1.57]},   
        {'time': 2.0, 'pose': [0.0, -1.0, -1.57]},   
        {'time': 3.5, 'pose': [0.0, 1.0, 1.57]}, 
        {'time': 2.0, 'pose': [0.0, 1.0, 1.57]}, 
        {'time': 3.0, 'pose': [0.0, 0.0, 0.0]},  # 回到原点
    ]
    CMD_TYPE = 'chassis_world'  # 或 'chassis_local'
    SCALE = 0.5  # 规划参数缩放系数
    IS_SYNC = False  # False=旋转可先于位移完成（给定时间为最慢轴时间）

    timed_cmd_api = TimedCmdAPI()
    root = make_tree(timed_cmd_api, cmd_type=CMD_TYPE, scale=SCALE, is_sync=IS_SYNC)
    tree = py_trees.trees.BehaviourTree(root)
    
    print(f"开始底盘控制 ({CMD_TYPE})，已设置规划器参数...")
    while True:
        tree.tick()
        status = root.status
        print(py_trees.display.unicode_tree(root, show_status=True))
        
        if status == py_trees.common.Status.SUCCESS:
            print("底盘运动完成!")
            # 获取期望位姿（最后一个关键点）
            last_pose = CHASSIS_POSES[-1]["pose"]
            expected_x, expected_y, expected_yaw = float(last_pose[0]), float(last_pose[1]), float(last_pose[2])
            # 获取实际底盘位姿
            robot_sdk = RobotSDK()
            cur = robot_sdk.tools.get_link_pose(link_name="base_link", reference_frame=Frame.ODOM)
            if cur is not None:
                actual_x = float(cur.position[0])
                actual_y = float(cur.position[1])
                cur_pose = Pose(pos=cur.position, quat=cur.orientation, frame=Frame.ODOM)
                actual_yaw = float(cur_pose.get_euler(degrees=False)[2])
                err_x = actual_x - expected_x
                err_y = actual_y - expected_y
                err_yaw = _yaw_diff_rad(actual_yaw, expected_yaw)
                print("---------- 底盘位姿与误差 ----------")
                print(f"  实际: x={actual_x:.4f} m, y={actual_y:.4f} m, yaw={np.rad2deg(actual_yaw):.4f}°")
                print(f"  期望: x={expected_x:.4f} m, y={expected_y:.4f} m, yaw={np.rad2deg(expected_yaw):.4f}°")
                print(f"  误差: dx={err_x:.4f} m, dy={err_y:.4f} m, dyaw={np.rad2deg(err_yaw):.4f}°")
                print("----------------------------------")
            else:
                print("警告: 无法获取底盘位姿，跳过误差打印")
            break
        if status == py_trees.common.Status.FAILURE:
            print("底盘运动失败.")
            break
        time.sleep(0.1)

