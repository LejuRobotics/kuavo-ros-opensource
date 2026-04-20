"""
轮臂Ruckig规划器参数设置案例

使用 NodeSetRuckigParams 节点设置不同规划器的运动参数（速度、加速度、急动度等）。
可以在运动控制前设置规划器参数，以调整运动特性。
"""
import os
import sys
import time
from typing import List, Optional

import py_trees

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import NodeSetRuckigParams, NodeWheelMoveTimedCmd, NodeFuntion
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


def make_tree(timed_cmd_api, chassis_poses: Optional[List[dict]] = None, cmd_type: str = 'chassis_world'):
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
    
    # 设置规划器参数节点
    # 示例：设置底盘世界系规划器参数
    set_params_node = NodeSetRuckigParams(
        name="set_ruckig_params",
        timed_cmd_api=timed_cmd_api,
        planner_index=planner_index,
        is_sync=True,  # 同步模式
        velocity_max=[0.2, 0.2, 0.6],  # 底盘x, y, yaw 3个自由度的最大速度 (m/s, m/s, rad/s)
        acceleration_max=[4.0, 4.0, 4],  # 最大加速度 (m/s², m/s², rad/s²)
        jerk_max=[20.0, 15.0, 12.0],  # 最大急动度 (m/s³, m/s³, rad/s³)
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
    CHASSIS_POSES = [
        {'time': 2.0, 'pose': [0.3, 0.0, 0.0]},   # 前进0.3米
        {'time': 2.0, 'pose': [0.0, 0.3, 1.57]},  # 右移0.3米 + 旋转90度
        {'time': 2.0, 'pose': [0.0, 0.0, 1.57]},  # 继续旋转90度
        {'time': 2.0, 'pose': [0.0, 0.0, 0.0]},  # 回原点方向
    ]
    CMD_TYPE = 'chassis_world'  # 或 'chassis_local'

    timed_cmd_api = TimedCmdAPI()
    root = make_tree(timed_cmd_api, cmd_type=CMD_TYPE)
    tree = py_trees.trees.BehaviourTree(root)
    
    print(f"开始底盘控制 ({CMD_TYPE})，已设置规划器参数...")
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

