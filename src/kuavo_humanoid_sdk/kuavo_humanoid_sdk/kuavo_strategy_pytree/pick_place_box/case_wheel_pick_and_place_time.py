"""
案例：头部搜 tag + 搬箱子 + 放箱子（PyTree + mobile_manipulator_timed_single_cmd）

使用 /mobile_manipulator_timed_single_cmd 服务执行底盘与手臂动作：
- 底盘：walk_goal 转为 chassis_world_keypoints，NodeWheelMoveTimedCmd(chassis_world)
- 手臂：抓取用 lb_arm_generate_pick_keypoints，放置用 lb_arm_generate_place_keypoints，
       底盘到位后经 NodeTagToBase 将 TAG 系关键点转为 BASE 系，写入 arm_ee_local_keypoints 后执行

流程：一次头部扫描同时检测 pick/place 两个 tag → 设置 walk_goal → 底盘到位 → TagToBase 转换 → 执行手臂
"""

import sys
import time
from functools import partial

import numpy as np
import py_trees
import rospy

from kuavo_humanoid_sdk import KuavoSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.configs.config_wheel_real import config  # 仿真改用 config_wheel_sim
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import TimedCmdAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.funcs import (
    lb_arm_generate_pick_before,
    lb_arm_generate_pick_keypoints,
    lb_arm_liftUp_keypoint,
    lb_arm_generate_pre_place_keypoints,
    lb_arm_generate_place_keypoints,
    set_arm_ee_local_keypoints_from_poses,
)

from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import (
    NodeFuntion,
    NodeWheelMoveTimedCmd,
    NodeSetRuckigParams,
    NodeTagToBase,
    NodeHeadSearchTag,
    NodeChassisMoveToTag,
    NodeArmForce,
)

# === 配置 ===
PICK_TAG_ID = config.pick.tag_id
PLACE_TAG_ID = config.place.tag_id

USER_DEBUG_MODE = False

# 抓取阶段到底盘点位时间
CHASSIS_PICK_TIME = 3.0
# 放置阶段到底盘点位时间
CHASSIS_PLACE_TIME = 3.0

# 手臂关键点总执行时间（秒）
ARM_PICK_TIME = 0.7 # 抓取时间
ARM_PICK_LIFT_TIME = 0.7 # 抬升时间
ARM_PRE_PLACE_TIME = 0.7 # 放置前移动时间
ARM_PLACE_TIME = 0.7 # 放置时间

# 抓取前手臂预动作执行时间（秒），与底盘运动并行
ARM_PRE_PICK_TIME = 2.0

# 头部搜索范围（度转弧度在 build_head_search_tree 中）
HEAD_SEARCH_YAWS = [-90, 90]
HEAD_SEARCH_PITCHS = [10, 0, -10]

TAG_STABLE_COUNT = 40
TAG_POS_STD_THRESHOLD = 0.02

# 底盘到位判定：xy 位置误差 < 0.05m，yaw 误差 < 5°
CHASSIS_POS_TOL = 0.15
CHASSIS_YAW_TOL_DEG = 10.0

# 底盘 Ruckig 规划器默认参数（来自 humanoid_wheel_interface 的 task.info：
# referencekinematicLimit.wheel_move.max_vel / max_acc / max_jerk）
# 维度为 [x, y, yaw]，单位：速度 m/s、rad/s，加速度 m/s²、rad/s²，急动度 m/s³、rad/s³
CHASSIS_RUCKIG_DEFAULT = {
    "velocity_max": [0.8, 1.1, 2.0],      # 速度上限为（2.0， 2.0， 1.57）
    "acceleration_max": [1.5, 2.0, 5.0],   #实机底盘x, y, yaw 3个自由度的最大加速度 (2.0m/s², 2.0m/s², 2.0rad/s²)
    "jerk_max": [40.0, 50.0, 90],
}
# 底盘运动参数缩小为默认的 0.5 倍（更平滑、更保守）
CHASSIS_RUCKIG_SCALE = 1.0
chassis_ruckig_move_params = {
    "velocity_max": [v * CHASSIS_RUCKIG_SCALE for v in CHASSIS_RUCKIG_DEFAULT["velocity_max"]],
    "acceleration_max": [a * CHASSIS_RUCKIG_SCALE for a in CHASSIS_RUCKIG_DEFAULT["acceleration_max"]],
    "jerk_max": [j * CHASSIS_RUCKIG_SCALE for j in CHASSIS_RUCKIG_DEFAULT["jerk_max"]],
}


# --------------- 底盘世界系关键点（通用接口）---------------
def set_chassis_world_keypoints(
    x: float,
    y: float,
    yaw: float,
    time: float = 7.0,
) -> bool:
    """将底盘世界系单点 (x, y, yaw) 及执行时间写入黑板，供 NodeWheelMoveTimedCmd(chassis_world) 使用。
    用户可传入任意 (x, y, yaw) 和 time。"""
    bb = py_trees.blackboard.Client(name="set_chassis_world_keypoints")
    for k in ["chassis_world_keypoints", "chassis_world_keypoint_times"]:
        bb.register_key(key=k, access=py_trees.common.Access.WRITE)
    bb.chassis_world_keypoints = [[float(x), float(y), float(yaw)]]
    bb.chassis_world_keypoint_times = [float(time)]
    rospy.loginfo(f"📌 底盘关键点: [x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}], time={time}s")
    return True


def build_tree() -> py_trees.behaviour.Behaviour:
    """
    行为树：一次头部扫描同时检测 pick/place 两个 tag → 抓取（底盘+手臂预动作并行→手臂抓取）→ 放置（底盘→手臂）
    抓取阶段：底盘运动与手臂预动作（arm_generate_pick_before，2s）并行；底盘到位后将 TAG 系关键点
    转为 BASE 系再执行抓取手臂。放置阶段仍使用本地 BASE 关键点。
    config.fake_tags.enable 为 True 时跳过视觉识别。
    """
    robot_sdk = RobotSDK()
    timed_cmd_api = TimedCmdAPI()

    head_search_tag_node = NodeHeadSearchTag(
        name="head_search_tag",
        robot_sdk=robot_sdk,
        tag_ids=[PICK_TAG_ID, PLACE_TAG_ID],
        config=config,
        head_search_yaws=np.deg2rad(HEAD_SEARCH_YAWS),
        head_search_pitchs=np.deg2rad(HEAD_SEARCH_PITCHS),
        tag_wait_timeout=1.0,             # 等待 tag 识别超时时间（秒），超时则报错并退出
        tag_stable_count=TAG_STABLE_COUNT,              # 连续 40 帧位置方差小于阈值才写入黑板
        tag_pos_std_threshold=TAG_POS_STD_THRESHOLD,       # 位置标准差阈值(m)
    )

    # 底盘世界系规划器参数：使用 0.5 倍默认速度/加速度/急动度
    set_chassis_ruckig_node = NodeSetRuckigParams(
        name="set_chassis_ruckig_params",
        timed_cmd_api=timed_cmd_api,
        planner_index=0,  # 0: 底盘世界系
        is_sync=config.common.is_chassis_move_sync,
        velocity_max=chassis_ruckig_move_params["velocity_max"],
        acceleration_max=chassis_ruckig_move_params["acceleration_max"],
        jerk_max=chassis_ruckig_move_params["jerk_max"],
    )

    def set_pick_arm_pre_keypoints_fn():
        left_poses, right_poses = lb_arm_generate_pick_before()
        return set_arm_ee_local_keypoints_from_poses(left_poses, right_poses, ARM_PRE_PICK_TIME)

    set_pick_arm_pre_keypoints_node = NodeFuntion(
        name="set_pick_arm_pre_keypoints",
        fn=set_pick_arm_pre_keypoints_fn,
    )
    pick_arm_pre_move_node = NodeWheelMoveTimedCmd(name="pick_arm_pre_move", timed_cmd_api=timed_cmd_api, cmd_type="arm_ee_local")

    pick_chassis_node = NodeChassisMoveToTag(
        name="pick_chassis_to_tag",
        robot_sdk=robot_sdk,
        timed_cmd_api=timed_cmd_api,
        tag_id=PICK_TAG_ID,
        stand_pos=config.pick.stand_in_tag_pos,
        stand_euler=config.pick.stand_in_tag_euler,
        chassis_time=CHASSIS_PICK_TIME,
        pos_tol=CHASSIS_POS_TOL,
        yaw_tol_deg=CHASSIS_YAW_TOL_DEG,
        prepare_nodes=set_pick_arm_pre_keypoints_node,
        parallel_with=pick_arm_pre_move_node,
    )

    # 抓取阶段：前两个点位（预抓取、并拢）
    pick_left_kp, pick_right_kp = lb_arm_generate_pick_keypoints(
        box_width=config.common.box_width,
        box_behind_tag=config.pick.box_behind_tag,
        box_beneath_tag=config.pick.box_beneath_tag,
        box_left_tag=config.pick.box_left_tag,
        hand_pitch_degree=0.0,
    )
    convert_pick_arm_keypoints_node = NodeTagToBase(
        name="convert_pick_arm_keypoints",
        robot_sdk=robot_sdk,
        tag_id=PICK_TAG_ID,
        left_poses=pick_left_kp,
        right_poses=pick_right_kp,
        total_time=ARM_PICK_TIME,
    )
    pick_arm_move_node = NodeWheelMoveTimedCmd(
        name="pick_arm_move", timed_cmd_api=timed_cmd_api, cmd_type="arm_ee_local"
    )

    # 抓取后的抬升点位：与期望力并行执行
    lift_left_kp, lift_right_kp = lb_arm_liftUp_keypoint(
        box_width=config.common.box_width,
        box_behind_tag=config.pick.box_behind_tag,
        box_beneath_tag=config.pick.box_beneath_tag,
        box_left_tag=config.pick.box_left_tag,
    )
    convert_pick_lift_keypoints_node = NodeTagToBase(
        name="convert_pick_lift_keypoint",
        robot_sdk=robot_sdk,
        tag_id=PICK_TAG_ID,
        left_poses=lift_left_kp,
        right_poses=lift_right_kp,
        total_time=ARM_PICK_LIFT_TIME,
    )
    pick_lift_move_node = NodeWheelMoveTimedCmd(
        name="pick_lift_move", timed_cmd_api=timed_cmd_api, cmd_type="arm_ee_local"
    )

    # 搬箱期望力节点：抓取后抬升阶段开启，在放置打开点位后关闭
    arm_force_enable_node = NodeArmForce(
        name="enable_arm_force",
        box_weight_kg=config.common.box_weight_kg,
        enable=True,
        transition_time=config.common.contact_force_weight_transition_time,
        interpolation_speed=config.common.contact_force_interpolation_speed,
    )

    # 抬升与期望力并行：先由 TAG→BASE 生成抬升关键点，再与施加期望力同时执行
    lift_and_force_parallel = py_trees.composites.Parallel(
        name="lift_and_force_parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    lift_sequence = py_trees.composites.Sequence(
        name="lift_sequence",
        memory=True,
    )
    lift_sequence.add_children(
        [
            convert_pick_lift_keypoints_node,
            pick_lift_move_node,
        ]
    )
    lift_and_force_parallel.add_children(
        [
            arm_force_enable_node,
            lift_sequence,
        ]
    )

    # 放置阶段：
    pre_place_left, pre_place_right = lb_arm_generate_pre_place_keypoints(
        box_width=config.common.box_width,
        box_behind_tag=config.place.box_behind_tag,
        box_beneath_tag=config.place.box_beneath_tag,
        box_left_tag=config.place.box_left_tag,
    )
    place_left, place_right = lb_arm_generate_place_keypoints(
        box_width=config.common.box_width,
        box_behind_tag=config.place.box_behind_tag,
        box_beneath_tag=config.place.box_beneath_tag,
        box_left_tag=config.place.box_left_tag,
    )

    place_chassis_node = NodeChassisMoveToTag(
        name="place_chassis_to_tag",
        robot_sdk=robot_sdk,
        timed_cmd_api=timed_cmd_api,
        tag_id=PLACE_TAG_ID,
        stand_pos=config.place.stand_in_tag_pos,
        stand_euler=config.place.stand_in_tag_euler,
        chassis_time=CHASSIS_PLACE_TIME,
        pos_tol=CHASSIS_POS_TOL,
        yaw_tol_deg=CHASSIS_YAW_TOL_DEG,
    )
    convert_place_pre_move_node = NodeTagToBase(
        name="convert_place_pre_move_keypoints",
        robot_sdk=robot_sdk,
        tag_id=PLACE_TAG_ID,
        left_poses=pre_place_left,
        right_poses=pre_place_right,
        total_time=ARM_PRE_PLACE_TIME,
    )
    place_arm_pre_move_node = NodeWheelMoveTimedCmd(
        name="place_arm_pre_move", timed_cmd_api=timed_cmd_api, cmd_type="arm_ee_local"
    )
    convert_place_node = NodeTagToBase(
        name="convert_place_keypoint",
        robot_sdk=robot_sdk,
        tag_id=PLACE_TAG_ID,
        left_poses=place_left,
        right_poses=place_right,
        total_time=ARM_PLACE_TIME,
    )
    place_arm_move_node = NodeWheelMoveTimedCmd(
        name="place_arm_move", timed_cmd_api=timed_cmd_api, cmd_type="arm_ee_local"
    )

    arm_force_disable_node = NodeArmForce(
        name="disable_arm_force",
        box_weight_kg=config.common.box_weight_kg,
        enable=False,
        transition_time=config.common.contact_force_weight_transition_time,
        interpolation_speed=config.common.contact_force_interpolation_speed,
    )

    # 第三个点位（收臂）与撤销期望力并行执行
    place_and_force_disable_parallel = py_trees.composites.Parallel(
        name="place_and_force_disable_parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    place_sequence = py_trees.composites.Sequence(name="place_sequence", memory=True)
    place_sequence.add_children(
        [
            convert_place_node,
            place_arm_move_node,
        ]
    )
    place_and_force_disable_parallel.add_children(
        [
            arm_force_disable_node,
            place_sequence,
        ]
    )

    # 回到原点：底盘回到 (0,0,0)，手臂复位
    set_back_chassis_keypoints_node = NodeFuntion(
        name="set_back_chassis_keypoints",
        fn=partial(
            set_chassis_world_keypoints,
            0.0, 0.0, 0.0,
            5.0,
        ),
    )
    back_chassis_move_node = NodeWheelMoveTimedCmd(name="back_chassis_move", timed_cmd_api=timed_cmd_api, cmd_type="chassis_world")
    back_arm_reset_node = NodeFuntion(
        name="back_arm_reset",
        fn=lambda: robot_sdk.control.wheel_control.reset_and_set_external() or True,
    )
    # 底盘与手臂复位并行执行：先设置底盘关键点，再同时跑底盘移动和手臂复位
    back_parallel = py_trees.composites.Parallel(
        name="back_chassis_and_arm_parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    back_parallel.add_children([back_arm_reset_node])
    back_to_origin_node = py_trees.composites.Sequence(
        name="back_to_origin",
        memory=True,
    )
    back_to_origin_node.add_children([set_back_chassis_keypoints_node, back_parallel])


## =================================== 构建PyTree搬箱流程 ===================================================
    
    root = py_trees.composites.Sequence(name="pick_and_place_demo", memory=True)
    # 头部搜索tag、设置底盘运动参数
    children = [
        head_search_tag_node,
        set_chassis_ruckig_node,
    ]
    if 1:
        children.append(NodeFuntion(name="wait_before_pick", fn=lambda: (input("准备执行【抓取阶段】，按回车继续...\n") or True)))
    
    # 底盘运动到pick位置并抓取：底盘运动与手臂预动作并行（预动作 2s），底盘到位后执行抓取手臂
    children.extend(
        [
            pick_chassis_node,                  # 底盘运动到抓取点位
            convert_pick_arm_keypoints_node,    # 将TAG系抓取点位转换为BASE系抓取点位
            pick_arm_move_node,                 # 执行抓取手臂
            lift_and_force_parallel,            # 设置期望力并抬升
        ]
    )
    if USER_DEBUG_MODE:
        children.append(NodeFuntion(name="wait_before_place", fn=lambda: (input("准备执行【放置阶段】，按回车继续...\n") or True)))
    
    # 底盘运动到place位置并放置：先移动到放置点位，然后执行手臂移动到放置位置，最后撤销期望力并收臂
    children.extend(
        [
            place_chassis_node,                     # 底盘运动到放置点位
            convert_place_pre_move_node,           # 将TAG系放置点位转换为BASE系放置点位
            place_arm_pre_move_node,          # 执行手臂移动到放置位置
            place_and_force_disable_parallel, # 撤销期望力并收臂
        ]
    )

    # if USER_DEBUG_MODE:
    #     children.append(
    #         NodeFuntion(name="back_to_origin", fn=lambda: (input("准备执行【复位阶段】，按回车继续...\n") or True), )
    #     )

    # 回到原点：底盘回 (0,0,0)，手臂复位
    # children.append(back_to_origin_node)
    
    if 1:
        children.append(NodeFuntion(name="back_to_origin", fn=lambda: (input("搬箱结束【开始下一轮】，按回车继续...\n") or True)))

    root.add_children(children)
    return root


def run_tree(root: py_trees.behaviour.Behaviour):

    root_looping = py_trees.decorators.Repeat(name="RepeatRoot", child=root, num_success=10)
    
    tree = py_trees.trees.BehaviourTree(root_looping)
    tree.setup(timeout=5)
    rospy.loginfo("=" * 60)
    rospy.loginfo("🚀 启动 PyTree: 搬箱子 (timed_cmd)")
    rospy.loginfo("=" * 60)

    while True:
        tree.tick()
        status = root_looping.status  # 查看根节点的状态
        # 打印根节点状态
        # rospy.loginfo(py_trees.display.unicode_tree(root, show_status=True))  
        time.sleep(0.002)


def main():

    KuavoSDK.Init(log_level="INFO")

    if not rospy.core.is_initialized():
        rospy.init_node("pick_place_time_case_node", anonymous=True, disable_signals=True)

    rospy.loginfo(f"🦾 手臂控制: arm_ee_local 局部系")
    rospy.loginfo(
        f"⏱️ 底盘时间: 抓取 {CHASSIS_PICK_TIME}s, 放置 {CHASSIS_PLACE_TIME}s; "
        f"手臂时间: 抓取 {ARM_PICK_TIME}s, 放置 {ARM_PLACE_TIME}s"
    )
    rospy.loginfo(f"🏷️ 抓取 Tag: {PICK_TAG_ID}, 放置 Tag: {PLACE_TAG_ID}")
    if config.fake_tags.enable:
        rospy.loginfo(f"📌 虚假 tag: pick={config.fake_tags.pick_pos}, place={config.fake_tags.place_pos}")
    rospy.loginfo("")

    root = build_tree()
    run_tree(root)
    rospy.loginfo("🎉 案例执行完毕")


if __name__ == "__main__":
    main()
