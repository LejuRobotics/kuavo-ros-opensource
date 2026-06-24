"""
案例：头部搜tag + 搬起箱子 + 放下箱子（PyTree版本）

流程概览：
1) 头部扫描找到抓取用的tag并缓存到黑板
2) 根据tag生成行走目标 & 手臂关键点
3) 并行移动到底盘站位并执行抓取
4) 头部扫描找到放置用的tag并缓存到黑板
5) 根据tag生成行走目标 & 手臂关键点
6) 并行移动到底盘站位并执行放置
"""

import argparse
import sys
import time
from functools import partial

import numpy as np
import py_trees

from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcCtrlMode
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Tag, Frame
from kuavo_humanoid_sdk.kuavo_strategy_pytree.configs.config_sim import config
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import (
    transform_pose_from_tag_to_world,
    TorsoAPI,
    ArmAPI,
)
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import (
    NodePercep,
    NodeWaitForBlackboard,
    NodeFuntion,
    NodeDelay,
    NodeWheelWalk,
    NodeWheelArm,
    NodeTagToArmGoal,
)


# === 配置 ===
PICK_TAG_ID = config.pick.tag_id
PLACE_TAG_ID = config.place.tag_id
CONTROL_MODES = ["cmd_pos_world", "cmd_pos", "cmd_vel"]

# 手臂控制类型
ARM_CONTROL_TYPE = "joint"  # eef_world, eef_base, joint

# 手臂启动延时（秒）
PICK_ARM_DELAY = 5
PLACE_ARM_DELAY = 8


def generate_pick_keypoints(
        box_width: float,
        box_behind_tag: float,  # 箱子在tag后面的距离，单位米
        box_beneath_tag: float,  # 箱子在tag下方的距离，单位米
        box_left_tag: float,  # 箱子在tag左侧的距离，单位米
        hand_pitch_degree: float = 0.0,  # 手臂pitch角度（相比水平, 下倾是正），单位度
):
    pick_left_arm_poses = [

        # # 1. 预抓取点位
        Pose.from_euler(pos=(0.3, box_width * 3 / 2 - box_left_tag, 0.1), euler=(0, -90 + hand_pitch_degree, 0),
                        degrees=True,
                        frame=Frame.BASE),

        # # 1. 预抓取点位
        Pose.from_euler(pos=(0.5, box_width * 3 / 2 - box_left_tag, 0.2), euler=(0, -90 + hand_pitch_degree, 0),
                        degrees=True,
                        frame=Frame.BASE),
        # 2. 并拢点位
        Pose.from_euler(pos=(0.5, box_width / 2 - box_left_tag, 0.2), euler=(0, -90 + hand_pitch_degree, 0),
                        degrees=True,
                        frame=Frame.BASE),
        # 4. 收臂点位
        Pose.from_euler(pos=(0.5, box_width / 2, 0.4), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                        frame=Frame.BASE)
    ]

    pick_right_arm_poses = [
        Pose.from_euler(pos=(0.3, -box_width * 3 / 2 - box_left_tag, 0.1), euler=(0, -90 + hand_pitch_degree, 0),
                        degrees=True,
                        frame=Frame.BASE),

        Pose.from_euler(pos=(0.5, -box_width * 3 / 2 - box_left_tag, 0.2), euler=(0, -90 + hand_pitch_degree, 0),
                        degrees=True,
                        frame=Frame.BASE),
        # 2. 并拢点位
        Pose.from_euler(pos=(0.5, -box_width / 2 - box_left_tag, 0.2), euler=(0, -90 + hand_pitch_degree, 0),
                        degrees=True,
                        frame=Frame.BASE),
        # 4. 收臂点位
        Pose.from_euler(pos=(0.5, -box_width / 2, 0.4), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                        frame=Frame.BASE),
    ]

    return pick_left_arm_poses, pick_right_arm_poses


def generate_place_keypoints(
        box_width: float,
        box_behind_tag: float,  # 箱子在tag后面的距离，单位米
        box_beneath_tag: float,  # 箱子在tag下方的距离，单位米
        box_left_tag: float,  # 箱子在tag左侧的距离，单位米
):
    place_left_arm_poses = [
        # 1. 收臂点位
        Pose.from_euler(pos=(0.4, box_width * 3 / 2, 0.1), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
    ]
    place_right_arm_poses = [
        # 1. 收臂点位
        Pose.from_euler(pos=(0.4, -box_width * 3 / 2, 0.1), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
    ]  # 手臂关键点数据，假设为空列表

    return place_left_arm_poses, place_right_arm_poses


def set_walk_goal_from_tag(tag_id: int, stand_pos, stand_euler):
    """从黑板上的 tag 生成 walk_goal"""
    bb = py_trees.blackboard.Client(name=f"tag_goal_setter_{tag_id}")

    for k in [f"latest_tag_{tag_id}", "walk_goal", "is_walk_goal_new"]:
        bb.register_key(key=k, access=py_trees.common.Access.READ)
        bb.register_key(key=k, access=py_trees.common.Access.WRITE)

    tag_key = f"latest_tag_{tag_id}"
    tag = getattr(bb, tag_key, None)
    if tag is None:
        print(f"❌ 黑板上没有找到 tag {tag_id}")
        return False

    stand_pose_in_tag = Pose.from_euler(
        pos=stand_pos,
        euler=stand_euler,
        frame=Frame.TAG,
        degrees=False,
    )
    target_pose = transform_pose_from_tag_to_world(tag, stand_pose_in_tag)

    bb.walk_goal = target_pose
    bb.is_walk_goal_new = True

    print(
        f"🎯 设置行走目标(tag {tag_id}): 位置=[{target_pose.pos[0]:.3f}, {target_pose.pos[1]:.3f}, {target_pose.pos[2]:.3f}]"
    )
    return True


def log_tag_from_bb(tag_id: int):
    """模仿 case_test_head：从黑板读取 tag 并打印出来"""
    bb = py_trees.blackboard.Client(name=f"tag_logger_{tag_id}")
    bb.register_key(key=f"latest_tag_{tag_id}", access=py_trees.common.Access.READ)
    tag = getattr(bb, f"latest_tag_{tag_id}", None)
    if tag is None or not hasattr(tag, "pose"):
        print(f"⚠️ 无法在黑板上读取 latest_tag_{tag_id}")
        sys.exit(1)  # 直接退出当前案例
    pos = tag.pose.pos if hasattr(tag.pose, "pos") else tag.pose
    quat = tag.pose.quat if hasattr(tag.pose, "quat") else None
    print(f"📍 tag {tag_id} 位置: {pos}")
    if quat is not None:
        print(f"   姿态(quat): {quat}")
    return True


def build_head_search_tree(
    tag_id: int,
    robot_sdk: RobotSDK,
    yaws,
    pitches,
    delay_after_move: float = 0.4,
):
    """创建头部搜索子树，与感知并行运行直到发现目标tag"""
    percep = NodePercep(name=f"PERCEP_tag_{tag_id}", robot_sdk=robot_sdk, tag_ids=[tag_id])

    head_seq = py_trees.composites.Sequence(name=f"head_search_tag_{tag_id}", memory=True)
    head_move_nodes = []
    for yaw in yaws:
        for pitch in pitches:
            def make_head_node(y, p, rsdk):
                return NodeFuntion(
                    name=f"head_move_yaw_{np.rad2deg(y):.0f}_pitch_{np.rad2deg(p):.0f}",
                    fn=lambda y=y, p=p: rsdk.control.control_head(y, p) or True,
                )

            head_move_nodes.append(make_head_node(yaw, pitch, robot_sdk))
            head_move_nodes.append(
                NodeDelay(duration=delay_after_move, name="delay_after_head_move")
            )

    wait_for_tag = NodeWaitForBlackboard(key=f"latest_tag_{tag_id}")
    head_seq.add_children(head_move_nodes + [wait_for_tag])

    root = py_trees.composites.Parallel(
        name=f"head_search_parallel_tag_{tag_id}",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )
    root.add_children([head_seq, percep])
    return root


def build_walk_and_arm_parallel(
    name: str,
    torso_api: TorsoAPI,
    arm_api: ArmAPI,
    walk_mode: str,
    arm_delay: float,
):
    """并行节点：行走到底盘位姿 + 手臂执行轨迹"""
    walk_node = NodeWheelWalk(
        name=f"{name}_walk_{walk_mode}",
        torso_api=torso_api,
        walk_mode=walk_mode,
        mpc_ctrl_mode=KuavoManipulationMpcCtrlMode.BaseArm,
    )

    arm_delay_node = NodeDelay(duration=arm_delay, name=f"{name}_arm_delay_{arm_delay}s")
    move_arm_node = NodeWheelArm(
        name=f"{name}_move_arm",
        arm_api=arm_api,
        control_type=ARM_CONTROL_TYPE,
        direct_to_wbc=False,
        back_default=False,
        mpc_ctrl_mode=KuavoManipulationMpcCtrlMode.BaseArm,
    )

    walk_branch = py_trees.composites.Sequence(name=f"{name}_walk_branch", memory=True)
    walk_branch.add_children([walk_node])

    arm_branch = py_trees.composites.Sequence(name=f"{name}_arm_branch", memory=True)
    arm_branch.add_children([arm_delay_node, move_arm_node])

    parallel_node = py_trees.composites.Parallel(
        name=f"{name}_parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    parallel_node.add_children([walk_branch, arm_branch])
    return parallel_node


def build_tree(
    walk_mode: str = "cmd_pos_world",
    pick_arm_delay: float = PICK_ARM_DELAY,
    place_arm_delay: float = PLACE_ARM_DELAY,
) -> py_trees.behaviour.Behaviour:
    """
    行为树结构（先扫描两种tag，再抓取、再放置）:
        pick_and_place_demo (Sequence)
        ├── head_search_pick_parallel        # 扫描抓取tag，写 latest_tag_{pick_id}
        ├── head_search_place_parallel       # 扫描放置tag，写 latest_tag_{place_id}
        ├── set_pick_walk_goal               # 读抓取tag → walk_goal
        ├── tag_to_arm_goal_pick             # 生成抓取手臂轨迹       
        ├── pick_parallel                    # 并行走位+抓取
        │   ├── pick_walk_branch (Sequence)
        │   │   └── pick_walk_<walk_mode>
        │   └── pick_arm_branch (Sequence)
        │       ├── pick_arm_delay_<arm_delay>s
        │       └── pick_move_arm
        ├── set_place_walk_goal              # 读放置tag → walk_goal
        ├── tag_to_arm_goal_place            # 生成放置手臂轨迹
        └── place_parallel                   # 并行走位+放置
            ├── place_walk_branch (Sequence)
            │   └── place_walk_<walk_mode>
            └── place_arm_branch (Sequence)
                ├── place_arm_delay_<arm_delay>s
                └── place_move_arm
    """
    if walk_mode not in CONTROL_MODES:
        raise ValueError(f"Unsupported walk_mode: {walk_mode}")

    robot_sdk = RobotSDK()
    torso_api = TorsoAPI(robot_sdk)
    arm_api = ArmAPI(robot_sdk=robot_sdk)

    # ===== 先扫描并缓存抓取 / 放置 tag =====
    head_search_pick = build_head_search_tree(
        tag_id=PICK_TAG_ID,
        robot_sdk=robot_sdk,
        yaws=config.common.head_search_yaws,
        pitches=config.common.head_search_pitchs,
    )
    head_search_place = build_head_search_tree(
        tag_id=PLACE_TAG_ID,
        robot_sdk=robot_sdk,
        yaws=config.common.head_search_yaws,
        pitches=config.common.head_search_pitchs,
    )

    log_pick_tag_node = NodeFuntion(
        name="log_pick_tag",
        fn=partial(log_tag_from_bb, tag_id=PICK_TAG_ID),
    )
    log_place_tag_node = NodeFuntion(
        name="log_place_tag",
        fn=partial(log_tag_from_bb, tag_id=PLACE_TAG_ID),
    )

    # ===== 抓取阶段节点 =====
    set_pick_walk_goal_node = NodeFuntion(
        name="set_pick_walk_goal",
        fn=partial(
            set_walk_goal_from_tag,
            tag_id=PICK_TAG_ID,
            stand_pos=config.pick.stand_in_tag_pos,
            stand_euler=config.pick.stand_in_tag_euler,
        ),
    )

    pick_left_kp, pick_right_kp = generate_pick_keypoints(
        box_width=config.common.box_width,
        box_behind_tag=config.pick.box_behind_tag,
        box_beneath_tag=config.pick.box_beneath_tag,
        box_left_tag=config.pick.box_left_tag,
    )
    tag_to_arm_goal_pick = NodeTagToArmGoal(
        name="tag_to_arm_goal_pick",
        arm_api=arm_api,
        tag_id=PICK_TAG_ID,
        control_type=ARM_CONTROL_TYPE,
        left_arm_relative_keypoints=pick_left_kp,
        right_arm_relative_keypoints=pick_right_kp,
        enable_joint_mirroring=True,
    )
    pick_parallel = build_walk_and_arm_parallel(
        name="pick",
        torso_api=torso_api,
        arm_api=arm_api,
        walk_mode=walk_mode,
        arm_delay=pick_arm_delay,
    )

    # ===== 放置阶段节点 =====
    set_place_walk_goal_node = NodeFuntion(
        name="set_place_walk_goal",
        fn=partial(
            set_walk_goal_from_tag,
            tag_id=PLACE_TAG_ID,
            stand_pos=config.place.stand_in_tag_pos,
            stand_euler=config.place.stand_in_tag_euler,
        ),
    )
    place_left_kp, place_right_kp = generate_place_keypoints(
        box_width=config.common.box_width,
        box_behind_tag=config.place.box_behind_tag,
        box_beneath_tag=config.place.box_beneath_tag,
        box_left_tag=config.place.box_left_tag,
    )
    tag_to_arm_goal_place = NodeTagToArmGoal(
        name="tag_to_arm_goal_place",
        arm_api=arm_api,
        tag_id=PLACE_TAG_ID,
        control_type=ARM_CONTROL_TYPE,
        left_arm_relative_keypoints=place_left_kp,
        right_arm_relative_keypoints=place_right_kp,
        enable_joint_mirroring=True,
    )
    place_parallel = build_walk_and_arm_parallel(
        name="place",
        torso_api=torso_api,
        arm_api=arm_api,
        walk_mode=walk_mode,
        arm_delay=place_arm_delay,
    )

    # ===== 构建总树 =====
    root = py_trees.composites.Sequence(name="pick_and_place_demo", memory=True)
    root.add_children(
        [
            head_search_pick,
            log_pick_tag_node,
            head_search_place,
            log_place_tag_node,
            set_pick_walk_goal_node,
            tag_to_arm_goal_pick,
            pick_parallel,
            set_place_walk_goal_node,
            tag_to_arm_goal_place,
            place_parallel,
        ]
    )
    return root


def run_tree(root: py_trees.behaviour.Behaviour):
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=5)

    print("=" * 60)
    print("🚀 启动 PyTree: 搬箱子 & 放箱子")
    print("=" * 60)

    while True:
        tree.tick()
        status = root.status
        print(py_trees.display.unicode_tree(root, show_status=True))

        if status == py_trees.common.Status.SUCCESS:
            print("=" * 60)
            print("✅ 任务完成：搬起并放下箱子!")
            print("=" * 60)
            break
        if status == py_trees.common.Status.FAILURE:
            print("=" * 60)
            print("❌ 任务失败")
            print("=" * 60)
            break

        time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description="PyTree 案例：搬箱子+放箱子")
    parser.add_argument(
        "--walk-mode",
        choices=CONTROL_MODES,
        default="cmd_pos_world",
        help="选择移动控制模式 (默认: cmd_pos_world)",
    )
    parser.add_argument(
        "--pick-arm-delay",
        type=float,
        default=PICK_ARM_DELAY,
        help=f"抓取阶段手臂启动延时(秒)，用于调整手臂相对于移动的启动时间 (默认: {PICK_ARM_DELAY})",
    )
    parser.add_argument(
        "--place-arm-delay",
        type=float,
        default=PLACE_ARM_DELAY,
        help=f"放置阶段手臂启动延时(秒)，用于调整手臂相对于移动的启动时间 (默认: {PLACE_ARM_DELAY})",
    )
    args = parser.parse_args()

    print(f"🎮 移动控制模式: {args.walk_mode}")
    print(f"🦾 手臂控制类型: {ARM_CONTROL_TYPE}")
    print(f"⏱️ 抓取手臂启动延时: {args.pick_arm_delay} 秒")
    print(f"⏱️ 放置手臂启动延时: {args.place_arm_delay} 秒")
    print(f"🏷️ 抓取 Tag ID: {PICK_TAG_ID}, 放置 Tag ID: {PLACE_TAG_ID}")
    print()

    root = build_tree(
        walk_mode=args.walk_mode,
        pick_arm_delay=args.pick_arm_delay,
        place_arm_delay=args.place_arm_delay,
    )
    run_tree(root)
    print("🎉 案例执行完毕")


if __name__ == "__main__":
    main()


