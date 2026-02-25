import os
import sys
import time

import numpy as np
import py_trees

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import (
    NodeWalk, NodePercep, NodeTagToNavGoal, NodeWaitForBlackboard,
    NodeFuntion, NodeArm, NodeTagToArmGoal, NodeDelay
)
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import ArmAPI, TorsoAPI, transform_pose_from_tag_to_world
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.configs.config_sim import config
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Frame, Tag
from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcCtrlMode
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.funcs import (
    arm_generate_pick_keypoints, arm_generate_place_keypoints
)

# === 配置参数 ===
HEAD_SEARCH_YAWS = [np.deg2rad(85), 0, np.deg2rad(-85)]  # 头部搜索的偏航角度
HEAD_SEARCH_PITCHES = [np.deg2rad(-10), np.deg2rad(0), np.deg2rad(10)]  # 头部搜索的俯仰角度
control_type = 'eef_world'  # 手臂控制模式：'joint', 'eef_world', 'eef_base'
walk_mode = 'cmd_pos_world'  # 移动控制模式：'cmd_pos_world', 'cmd_pos', 'cmd_vel'（与case_test_move.py一致）


def set_walk_goal_from_tag(tag_id: int):
    """从检测到的tag设置行走目标到黑板（参考case_test_move.py）"""
    bb = py_trees.blackboard.Client(name="tag_goal_setter")

    # 注册黑板键
    for k in [f'latest_tag_{tag_id}', 'walk_goal', 'is_walk_goal_new']:
        bb.register_key(key=k, access=py_trees.common.Access.READ)
        bb.register_key(key=k, access=py_trees.common.Access.WRITE)

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

    # 根据tag_id选择配置
    if tag_id == config.pick.tag_id:
        stand_in_tag_pos = config.pick.stand_in_tag_pos
        stand_in_tag_euler = config.pick.stand_in_tag_euler
    elif tag_id == config.place.tag_id:
        stand_in_tag_pos = config.place.stand_in_tag_pos
        stand_in_tag_euler = config.place.stand_in_tag_euler
    else:
        print(f"❌ 未知的tag_id: {tag_id}")
        return False

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

    return True


def make_head_search_sequence(robot_sdk, tag_id):
    """创建头部搜索序列（参考case_test_head.py）"""
    HEAD_SEARCH = py_trees.composites.Sequence(name=f"head_search_tag_{tag_id}", memory=True)

    # 创建多个头部控制节点，在不同角度搜索
    head_move_nodes = []
    for yaw in HEAD_SEARCH_YAWS:
        for pitch in HEAD_SEARCH_PITCHES:
            # 使用闭包确保正确捕获变量
            def make_head_control(y, p, rsdk):
                return NodeFuntion(
                    name=f"head_move_yaw_{np.rad2deg(y):.0f}_pitch_{np.rad2deg(p):.0f}",
                    fn=lambda: rsdk.control.control_head(y, p) or True
                )
            head_move_node = make_head_control(yaw, pitch, robot_sdk)
            head_move_nodes.append(head_move_node)
            # 每个头部移动后等待一下，让感知有时间识别
            delay_node = NodeDelay(duration=0.5, name=f"delay_after_head_move")
            head_move_nodes.append(delay_node)

    # 等待找到 tag 的条件节点
    wait_for_tag = NodeWaitForBlackboard(key=f"latest_tag_{tag_id}")
    HEAD_SEARCH.add_children(head_move_nodes + [wait_for_tag])

    return HEAD_SEARCH


def make_tree(robot_sdk, arm_api, torso_api):
    """
    构建完整的行为树：寻找tag-移动-抓取-放置
    组合 case_test_head.py, case_test_move.py, case_test_arm.py 的功能
    """
    # ========== 感知节点 - 持续感知两个tag ==========
    PERCEP = NodePercep(
        name='PERCEP',
        robot_sdk=robot_sdk,
        tag_ids=[config.pick.tag_id, config.place.tag_id]
    )

    # ========== 1. 寻找pick tag（使用头部搜索） ==========
    search_pick_tag_HEAD = make_head_search_sequence(robot_sdk, config.pick.tag_id)

    search_pick_tag_SETGOAL = NodeFuntion(
        name="search_pick_tag_SETGOAL",
        fn=lambda: set_walk_goal_from_tag(tag_id=config.pick.tag_id)
    )

    search_pick_tag = py_trees.composites.Sequence(name="search_pick_tag", memory=True)
    search_pick_tag.add_children([search_pick_tag_HEAD, search_pick_tag_SETGOAL])

    # ========== 2. 走到pick位置（参考case_test_move.py的简单Sequence结构） ==========
    walk_to_pick_SETGOAL = NodeFuntion(
        name="walk_to_pick_SETGOAL",
        fn=lambda: set_walk_goal_from_tag(tag_id=config.pick.tag_id)
    )

    walk_to_pick_WALK = NodeWalk(
        name='walk_to_pick_WALK',
        torso_api=torso_api,
        walk_mode=walk_mode
    )

    walk_to_pick = py_trees.composites.Sequence(name="walk_to_pick", memory=True)
    walk_to_pick.add_children([walk_to_pick_SETGOAL, walk_to_pick_WALK])

    # ========== 3. 抓取箱子并后退 ==========
    left_arm_relative_keypoints, right_arm_relative_keypoints = arm_generate_pick_keypoints(
        box_width=config.common.box_width,
        box_behind_tag=config.pick.box_behind_tag,
        box_beneath_tag=config.pick.box_beneath_tag,
        box_left_tag=config.pick.box_left_tag,
    )

    pick_box_TAG2GOAL = NodeTagToArmGoal(
        name='pick_box_TAG2GOAL',
        arm_api=arm_api,
        tag_id=config.pick.tag_id,
        control_type=control_type,
        left_arm_relative_keypoints=left_arm_relative_keypoints,
        right_arm_relative_keypoints=right_arm_relative_keypoints,
        enable_joint_mirroring=True,
    )

    pick_box_ARM = NodeArm(
        name='pick_box_ARM',
        arm_api=arm_api,
        control_type=control_type,
        control_base=False,
        direct_to_wbc=False,
        back_default=False,
    )

    # 后退（统一使用BASE坐标系，因为后退操作使用cmd_pos模式）
    def set_backward_goal():
        bb = py_trees.blackboard.Client(name="backward_goal_setter")
        bb.register_key("walk_goal", py_trees.common.Access.WRITE)
        bb.register_key("is_walk_goal_new", py_trees.common.Access.WRITE)

        # 获取当前机器人位置（用于调试）
        current_pos = robot_sdk.state.robot_position()
        print(f"📍 当前位置: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")

        # 统一使用BASE坐标系进行相对移动（向后移动）
        # 后退操作使用cmd_pos模式，支持BASE坐标系
        backward_pose = Pose(
            pos=(-config.common.step_back_distance, 0., 0.),
            quat=(0, 0, 0, 1),
            frame=Frame.BASE
        )

        bb.walk_goal = backward_pose
        bb.is_walk_goal_new = True
        print(f"🎯 设置后退目标: 相对位置=[{backward_pose.pos[0]:.3f}, {backward_pose.pos[1]:.3f}, {backward_pose.pos[2]:.3f}], frame={backward_pose.frame}")
        return True

    pick_box_SETWALKGOAL = NodeFuntion(
        name="pick_box_SETWALKGOAL",
        fn=set_backward_goal
    )

    # 后退操作使用cmd_pos模式（支持BASE坐标系，更可靠）
    # cmd_pos_world模式可能不响应，所以后退时改用cmd_pos
    pick_box_WALK = NodeWalk(
        name='pick_box_WALK',
        torso_api=torso_api,
        walk_mode='cmd_pos'  # 后退时强制使用cmd_pos模式
    )

    # 重置行走状态，确保从cmd_pos模式切换回cmd_pos_world模式
    def reset_walk_after_pick():
        torso_api.stop_walk()
        # 切换到基座模式（BaseOnly），确保状态机正确重置
        robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseOnly)
        time.sleep(0.5)  # 短暂延迟，让状态机有时间重置
        print("🔄 pick_box完成后重置行走状态，切换到BaseOnly模式，准备切换到cmd_pos_world模式")
        return True

    pick_box_RESET = NodeFuntion(
        name="pick_box_RESET",
        fn=reset_walk_after_pick
    )

    pick_box = py_trees.composites.Sequence(name="pick_box", memory=True)
    pick_box.add_children([
        pick_box_TAG2GOAL,
        pick_box_ARM,
        pick_box_SETWALKGOAL,
        pick_box_WALK,
        pick_box_RESET  # 重置状态，确保后续cmd_pos_world模式能正常工作
    ])

    # ========== 4. 寻找place tag（使用头部搜索） ==========
    search_place_tag_HEAD = make_head_search_sequence(robot_sdk, config.place.tag_id)

    search_place_tag_SETGOAL = NodeFuntion(
        name="search_place_tag_SETGOAL",
        fn=lambda: set_walk_goal_from_tag(tag_id=config.place.tag_id)
    )

    search_place_tag = py_trees.composites.Sequence(name="search_place_tag", memory=True)
    search_place_tag.add_children([search_place_tag_HEAD, search_place_tag_SETGOAL])

    # ========== 5. 走到place位置（参考case_test_move.py的简单Sequence结构） ==========
    walk_to_place_SETGOAL = NodeFuntion(
        name="walk_to_place_SETGOAL",
        fn=lambda: set_walk_goal_from_tag(tag_id=config.place.tag_id)
    )

    walk_to_place_WALK = NodeWalk(
        name='walk_to_place_WALK',
        torso_api=torso_api,
        walk_mode=walk_mode
    )

    def delay_after_walk_to_place():
        torso_api.stop_walk()
        # 切换到基座模式（BaseOnly），确保状态机正确重置
        robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
        time.sleep(2)  # 短暂延迟，让状态机有时间重置
        print("🔄 walk_to_place完成后重置行走状态，切换到ArmOnly模式，准备切换到cmd_pos_world模式")
        return True

    delay_after_walk_to_place_node = NodeFuntion(
        name="delay_after_walk_to_place",
        fn=delay_after_walk_to_place
    )

    walk_to_place = py_trees.composites.Sequence(name="walk_to_place", memory=True)
    walk_to_place.add_children([walk_to_place_SETGOAL, walk_to_place_WALK, delay_after_walk_to_place_node])

    # ========== 6. 放置箱子并后退 ==========
    left_arm_relative_keypoints, right_arm_relative_keypoints = arm_generate_place_keypoints(
        box_width=config.common.box_width,
        box_behind_tag=config.place.box_behind_tag,
        box_beneath_tag=config.place.box_beneath_tag,
        box_left_tag=config.place.box_left_tag,
    )

    place_box_TAG2GOAL = NodeTagToArmGoal(
        name='place_box_TAG2GOAL',
        arm_api=arm_api,
        tag_id=config.place.tag_id,
        control_type=control_type,
        left_arm_relative_keypoints=left_arm_relative_keypoints,
        right_arm_relative_keypoints=right_arm_relative_keypoints,
        enable_joint_mirroring=True,
    )

    place_box_ARM = NodeArm(
        name='place_box_ARM',
        arm_api=arm_api,
        control_type=control_type,
        control_base=False,
        direct_to_wbc=False,
        back_default=False,
    )

    place_box_SETWALKGOAL = NodeFuntion(
        name="place_box_SETWALKGOAL",
        fn=set_backward_goal
    )

    # 后退操作使用cmd_pos模式（支持BASE坐标系，更可靠）
    # cmd_pos_world模式可能不响应，所以后退时改用cmd_pos
    place_box_WALK = NodeWalk(
        name='place_box_WALK',
        torso_api=torso_api,
        walk_mode='cmd_pos'  # 后退时强制使用cmd_pos模式
    )

    place_box = py_trees.composites.Sequence(name="place_box", memory=True)
    place_box.add_children([
        place_box_TAG2GOAL,
        place_box_ARM,
        place_box_SETWALKGOAL,
        place_box_WALK
    ])

    # ========== 构建主行为树 ==========
    ACTION = py_trees.composites.Sequence(name="ACTION", memory=True)
    ACTION.add_children([
        search_pick_tag,
        walk_to_pick,
        pick_box,
        search_place_tag,
        walk_to_place,
        place_box
    ])

    # 根节点：并行执行感知和动作
    root = py_trees.composites.Parallel(
        name="root",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    root.add_children([ACTION, PERCEP])

    return root


def run_tree(root: py_trees.behaviour.Behaviour):
    """
    运行行为树（参考case_test_*.py的编写方式）
    """
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=5)

    print("🧠 启动 PyTree - 寻找tag-移动-抓取-放置流程...")

    while True:
        tree.tick()
        status = root.status
        # print(py_trees.display.unicode_tree(root, show_status=True))

        if status == py_trees.common.Status.SUCCESS:
            print("✅ 完整流程执行完成")
            break
        if status == py_trees.common.Status.FAILURE:
            print("❌ 流程执行失败")
            break

        time.sleep(0.1)


if __name__ == '__main__':
    # 初始化
    robot_sdk = RobotSDK()
    arm_api = ArmAPI(robot_sdk=robot_sdk)
    torso_api = TorsoAPI(robot_sdk=robot_sdk)

    # 初始化头部位置
    robot_sdk.control.control_head(0, np.deg2rad(-10))

    # 构建并运行行为树
    root = make_tree(robot_sdk, arm_api, torso_api)
    run_tree(root)

    print("🎉 程序执行完毕")
