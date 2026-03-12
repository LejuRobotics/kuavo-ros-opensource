"""
案例：移动和手臂控制同步执行
使用 Parallel 节点并行执行移动和手臂控制，通过延时节点调整手臂启动时间
"""
import argparse
import time

import numpy as np
import py_trees

from kuavo_humanoid_sdk import KuavoSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Tag, Frame
from kuavo_humanoid_sdk.kuavo_strategy_pytree.configs.config_sim import config
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import (
    transform_pose_from_tag_to_world, 
    TorsoAPI, 
    ArmAPI
)
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import (
    NodeFuntion, 
    NodeDelay, 
    NodeWheelWalk,
    NodeWheelArm,
    NodeTagToArmGoal
)
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.funcs import arm_generate_pick_keypoints
from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcCtrlMode


# === 配置 ===
TAG_ID = config.pick.tag_id  # tag ID
CONTROL_MODES = ['cmd_pos_world', 'cmd_pos', 'cmd_vel']

# 虚假的 tag 位置（在 ODOM 坐标系下）
FAKE_TAG_POS = (0.50, 0.0, 0.75)  # x, y, z (米)
FAKE_TAG_EULER = (90, 0, -90)  # roll, pitch, yaw (度)

# 手臂控制类型
ARM_CONTROL_TYPE = 'joint'  # eef_world, eef_base, joint

# 手臂启动延时（秒）- 用于调整手臂相对于移动的启动时间
ARM_START_DELAY = 1.0


def set_fake_tag_to_blackboard():
    """设置虚假 tag 到 blackboard"""
    fake_tag = Tag(
        id=TAG_ID,
        pose=Pose.from_euler(
            pos=FAKE_TAG_POS,
            euler=FAKE_TAG_EULER,
            frame=Frame.ODOM,
            degrees=True
        )
    )
    
    bb = py_trees.blackboard.Client(name="set_fake_tag")
    bb.register_key(f"latest_tag_{TAG_ID}", py_trees.common.Access.WRITE)
    bb.register_key(f"latest_tag_{TAG_ID}_version", py_trees.common.Access.WRITE)
    setattr(bb, f"latest_tag_{TAG_ID}", fake_tag)
    setattr(bb, f"latest_tag_{TAG_ID}_version", 1)
    
    print(f"✅ 设置虚假 tag {TAG_ID} 位置: {FAKE_TAG_POS}")
    return True


def set_walk_goal_from_tag():
    """从检测到的 tag 设置行走目标到黑板"""
    bb = py_trees.blackboard.Client(name="tag_goal_setter")
    
    # 注册黑板键
    for k in [f'latest_tag_{TAG_ID}', 'walk_goal', 'is_walk_goal_new']:
        bb.register_key(key=k, access=py_trees.common.Access.READ)
        bb.register_key(key=k, access=py_trees.common.Access.WRITE)
    
    # 从黑板获取 tag 信息
    tag_key = f'latest_tag_{TAG_ID}'
    tag = getattr(bb, tag_key, None)
    
    if tag is None:
        print(f"❌ 黑板上没有找到 tag {TAG_ID}")
        return False
    
    # 打印 tag 信息
    print(f"✅ 成功找到 tag {TAG_ID}!")
    print(f"📍 tag {TAG_ID} 位置: {tag.pose.pos}")
    print(f"   姿态(quat): {tag.pose.quat}")
    
    # 使用配置文件中的站立位置和姿态
    stand_in_tag_pos = config.pick.stand_in_tag_pos
    stand_in_tag_euler = config.pick.stand_in_tag_euler
    print(f"stand_in_tag_pos: {stand_in_tag_pos}, stand_in_tag_euler: {stand_in_tag_euler}")
    
    # 创建站立位置的 Pose（在 tag 坐标系下）
    stand_pose_in_tag = Pose.from_euler(
        pos=stand_in_tag_pos,
        euler=stand_in_tag_euler,
        frame=Frame.TAG,
        degrees=False
    )
    
    # 转换到世界坐标系
    target_pose = transform_pose_from_tag_to_world(tag, stand_pose_in_tag)
    
    # 设置到黑板
    bb.walk_goal = target_pose
    bb.is_walk_goal_new = True
    
    print(f"🎯 设置行走目标: 位置=[{target_pose.pos[0]:.3f}, {target_pose.pos[1]:.3f}, {target_pose.pos[2]:.3f}]")
    print(f"   使用配置: stand_in_tag_pos={stand_in_tag_pos}, stand_in_tag_euler={np.rad2deg(stand_in_tag_euler)}")
    
    return True


def build_tree(walk_mode: str = 'cmd_pos_world', arm_delay: float = ARM_START_DELAY) -> py_trees.behaviour.Behaviour:
    """
    构建完整的行为树：移动和手臂控制同步执行
    
    行为树结构:
        root (Sequence)
        ├── 1_set_fake_tag           # 设置虚假 tag
        ├── 2_set_walk_goal          # 设置行走目标
        ├── 3_tag_to_arm_goal        # 计算手臂轨迹
        └── 4_parallel_execute       # 并行执行
            ├── walk_branch (Sequence)
            │   └── walk_node        # 执行移动
            └── arm_branch (Sequence)
                ├── arm_delay        # 延时等待
                └── move_arm         # 执行手臂运动
    
    Args:
        walk_mode: 移动控制模式，可选 'cmd_pos_world', 'cmd_pos', 'cmd_vel'
        arm_delay: 手臂启动延时（秒），用于调整手臂相对于移动的启动时间
    
    Returns:
        行为树根节点
    """
    if walk_mode not in CONTROL_MODES:
        raise ValueError(f"Unsupported walk_mode: {walk_mode}")
    
    # 创建 API 实例
    robot_sdk = RobotSDK()
    torso_api = TorsoAPI(robot_sdk)
    arm_api = ArmAPI(robot_sdk=robot_sdk)
    
    # ========== 准备阶段 ==========
    
    # 1. 设置虚假 tag 到 blackboard
    set_fake_tag_node = NodeFuntion(
        name="1_set_fake_tag",
        fn=set_fake_tag_to_blackboard
    )
    
    # 2. 根据 tag 设置行走目标
    set_walk_goal_node = NodeFuntion(
        name="2_set_walk_goal",
        fn=set_walk_goal_from_tag
    )
    
    # 3. 生成手臂关键点并计算手臂目标轨迹
    left_arm_relative_keypoints, right_arm_relative_keypoints = arm_generate_pick_keypoints(
        box_width=config.common.box_width,
        box_behind_tag=config.pick.box_behind_tag,
        box_beneath_tag=config.pick.box_beneath_tag,
        box_left_tag=config.pick.box_left_tag,
    )
    
    tag_to_arm_goal_node = NodeTagToArmGoal(
        name='3_tag_to_arm_goal',
        arm_api=arm_api,
        tag_id=TAG_ID,
        control_type=ARM_CONTROL_TYPE,
        left_arm_relative_keypoints=left_arm_relative_keypoints,
        right_arm_relative_keypoints=right_arm_relative_keypoints,
        enable_joint_mirroring=True,
    )
    
    # ========== 并行执行阶段 ==========
    
    # 移动分支 - 使用 BaseArm 模式，支持同时控制底盘和手臂
    walk_node = NodeWheelWalk(
        name=f"walk_{walk_mode}",
        torso_api=torso_api,
        walk_mode=walk_mode,
        mpc_ctrl_mode=KuavoManipulationMpcCtrlMode.BaseArm
    )
    
    walk_branch = py_trees.composites.Sequence(name="walk_branch", memory=True)
    walk_branch.add_children([walk_node])
    
    # 手臂分支：延时 + 手臂运动
    arm_delay_node = NodeDelay(
        duration=arm_delay,
        name=f"arm_delay_{arm_delay}s"
    )
    
    # 手臂控制 - 使用 BaseArm 模式，支持同时控制底盘和手臂
    move_arm_node = NodeWheelArm(
        name="move_arm",
        arm_api=arm_api,
        control_type=ARM_CONTROL_TYPE,
        direct_to_wbc=False,
        back_default=False,
        mpc_ctrl_mode=KuavoManipulationMpcCtrlMode.BaseArm
    )
    
    arm_branch = py_trees.composites.Sequence(name="arm_branch", memory=True)
    arm_branch.add_children([arm_delay_node, move_arm_node])
    
    # 并行节点：同时执行移动和手臂控制
    # policy=SuccessOnAll: 所有子节点都成功才算成功
    parallel_node = py_trees.composites.Parallel(
        name="4_parallel_execute",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    parallel_node.add_children([walk_branch, arm_branch])
    
    # ========== 构建完整行为树 ==========
    root = py_trees.composites.Sequence(name="move_and_arm_sync_demo", memory=True)
    root.add_children([
        # 准备阶段
        set_fake_tag_node,
        set_walk_goal_node,
        tag_to_arm_goal_node,
        # 并行执行阶段
        parallel_node,
    ])
    
    return root


def run_tree(root: py_trees.behaviour.Behaviour):
    """运行行为树"""
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=5)
    
    print("=" * 60)
    print("🚀 启动 PyTree: 移动和手臂同步执行")
    print("=" * 60)
    
    while True:
        tree.tick()
        status = root.status
        print(py_trees.display.unicode_tree(root, show_status=True))
        
        if status == py_trees.common.Status.SUCCESS:
            print("=" * 60)
            print("✅ 任务完成：移动和手臂控制同步执行成功!")
            print("=" * 60)
            break
        if status == py_trees.common.Status.FAILURE:
            print("=" * 60)
            print("❌ 任务失败")
            print("=" * 60)
            break
        
        time.sleep(0.1)


def main():

    # 初始化 SDK（确保 KuavoRobotCore.initialize 已完成，否则 cmd_vel/walk 会因缺少 _robot_version_major 崩溃）
    KuavoSDK.Init(log_level="INFO")

    parser = argparse.ArgumentParser(description="PyTree 案例：移动和手臂控制同步执行")
    parser.add_argument(
        "--walk-mode",
        choices=CONTROL_MODES,
        default="cmd_pos_world",
        help="选择移动控制模式 (默认: cmd_pos_world)"
    )
    parser.add_argument(
        "--arm-delay",
        type=float,
        default=ARM_START_DELAY,
        help=f"手臂启动延时(秒)，用于调整手臂相对于移动的启动时间 (默认: {ARM_START_DELAY})"
    )
    args = parser.parse_args()
    
    print(f"🎮 移动控制模式: {args.walk_mode}")
    print(f"🦾 手臂控制类型: {ARM_CONTROL_TYPE}")
    print(f"⏱️ 手臂启动延时: {args.arm_delay} 秒")
    print(f"🏷️ 目标 Tag ID: {TAG_ID}")
    print()
    
    # 构建并运行行为树
    root = build_tree(walk_mode=args.walk_mode, arm_delay=args.arm_delay)
    run_tree(root)
    
    print("🎉 案例执行完毕")


if __name__ == '__main__':
    main()

