import argparse
import time

import numpy as np
import py_trees

from kuavo_humanoid_sdk import KuavoSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Tag, Frame
from kuavo_humanoid_sdk.kuavo_strategy_pytree.configs.config_sim import config
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import transform_pose_from_tag_to_world, TorsoAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import NodeFuntion, NodeDelay, NodeWheelWalk, NodePercep

CONTROL_MODES = ['cmd_pos_world', 'cmd_pos', 'cmd_vel']


def set_walk_goal_from_tag(tag_id: int = 1, use_real_data: bool = False):
    """从检测到的tag设置行走目标到黑板"""
    bb = py_trees.blackboard.Client(name="tag_goal_setter")
    
    # 注册黑板键
    for k in [f'latest_tag_{tag_id}', 'walk_goal', 'is_walk_goal_new']:
        bb.register_key(key=k, access=py_trees.common.Access.READ)
        bb.register_key(key=k, access=py_trees.common.Access.WRITE)
    
    if use_real_data:
        # 使用你提供的真实tag数据
        print(f"✅ 成功找到 tag {tag_id}!")
        print(f"📍 tag {tag_id} 位置: [    0.01365     -1.7599     0.89368]")
        print(f"   姿态(quat): [  0.0035202     0.57765     0.81626   0.0049414]")
        
        # 创建tag对象
        tag = Tag(
            id=tag_id,
            pose=Pose(
                pos=[0.5, 0.0, 0.89368],
                quat=[0.0, 0.0, 0.0, 1.0],
                frame=Frame.ODOM
            )
        )
    else:
        # 从黑板获取tag信息
        tag_key = f'latest_tag_{tag_id}'
        tag = getattr(bb, tag_key, None)
        
        if tag is None:
            print(f"❌ 黑板上没有找到 tag {tag_id}")
            return False
        
        # 打印tag信息
        print(f"✅ 成功找到 tag {tag_id}!")
        print(f"📍 tag {tag_id} 位置: {tag.pose.pos}")
        print(f"   姿态(quat): {tag.pose.quat}")
    
    # 使用配置文件中的站立位置和姿态
    stand_in_tag_pos = config.pick.stand_in_tag_pos
    stand_in_tag_euler = config.pick.stand_in_tag_euler
    print(f"stand_in_tag_pos: {stand_in_tag_pos}, stand_in_tag_euler: {stand_in_tag_euler}")
    
    # 创建站立位置的Pose（在tag坐标系下）
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


def build_tree_with_node_walk(control_mode: str) -> py_trees.behaviour.Behaviour:
    """
    使用NodeWheelWalk构建移动树
    """
    if control_mode not in CONTROL_MODES:
        raise ValueError(f"Unsupported control_mode: {control_mode}")
    
    # 创建节点实例
    robot_sdk = RobotSDK()
    torso_api = TorsoAPI(robot_sdk)
    
    set_goal_node = NodeFuntion(
        name="set_walk_goal_from_tag",
        fn=lambda: set_walk_goal_from_tag(tag_id=1, use_real_data=True)
    )
    
    walk_node = NodeWheelWalk(
        name=f"walk_{control_mode}",
        torso_api=torso_api,
        walk_mode=control_mode
    )
    
    wait_node = NodeDelay(duration=1.0, name="wait_after_move")
    
    # 构建序列：直接设置目标 → 行走 → 等待
    root = py_trees.composites.Sequence(name=f"move_to_tag_demo_{control_mode}", memory=True)
    root.add_children([set_goal_node, walk_node, wait_node])
    
    return root


def run_tree(root: py_trees.behaviour.Behaviour):
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=5)

    print("🧠 启动 PyTree ...")
    while True:
        tree.tick()
        status = root.status
        print(py_trees.display.unicode_tree(root, show_status=True))

        if status == py_trees.common.Status.SUCCESS:
            print("✅ 移动示例完成")
            break
        if status == py_trees.common.Status.FAILURE:
            print("❌ 移动示例执行失败")
            break

        time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description="PyTree移动示例：使用NodeWheelWalk节点")
    parser.add_argument(
        "--control-mode",
        choices=CONTROL_MODES,
        default="cmd_pos_world",
        help="选择移动控制模式"
    )
    args = parser.parse_args()

    print(f"🚀 控制模式：{args.control_mode}")

    # 初始化 SDK（确保 KuavoRobotCore.initialize 已完成，否则 cmd_vel/walk 会因缺少 _robot_version_major 崩溃）
    KuavoSDK.Init(log_level="INFO")

    # 直接使用你提供的tag数据
    root = build_tree_with_node_walk(args.control_mode)
    
    run_tree(root)
    print("🎉 示例执行完毕，可根据需要多次运行并切换不同模式")


if __name__ == '__main__':
    main()
