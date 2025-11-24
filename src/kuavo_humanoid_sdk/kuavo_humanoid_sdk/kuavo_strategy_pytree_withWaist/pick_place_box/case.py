from kuavo_humanoid_sdk.kuavo_strategy_pytree_withWaist.nodes.nodes import NodeWalk, NodePercep, NodeTagToNavGoal, \
    NodeWaitForBlackboard, NodeFuntion, NodeArm, NodeTagToArmGoal, NodeWaist
from kuavo_humanoid_sdk.kuavo_strategy_pytree_withWaist.nodes.api import ArmAPI, TorsoAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree_withWaist.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree_withWaist.configs.config_sim import config
from kuavo_humanoid_sdk.kuavo_strategy_pytree_withWaist.common.data_type import Pose, Tag, Frame
from kuavo_humanoid_sdk.kuavo_strategy_pytree_withWaist.nodes.funcs import update_walk_goal, update_tag_guess, switch__mpc_control_flow, move_head, move_arm_to_zero
from kuavo_humanoid_sdk.kuavo_strategy_pytree_withWaist.nodes.funcs import arm_generate_pick_keypoints, \
    arm_generate_place_keypoints

from kuavo_humanoid_sdk.kuavo_strategy_pytree_withWaist.nodes.black_board import BlackBoardManager
import py_trees
import numpy as np

# 初始化API
robot_sdk = RobotSDK()
arm_api = ArmAPI(
    robot_sdk=robot_sdk,
)
torso_api = TorsoAPI(
    robot_sdk=robot_sdk,
)

# 控制模式选择: "cmd_vel" 或 "cmd_pose_world"
WALK_CONTROL_MODE = "cmd_pose_world"

# 先构建树
search_pick_tag_WALK = NodeWalk(name='search_pick_tag_WALK', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold)
search_pick_tag_TAG2GOAL = NodeTagToNavGoal(name='search_pick_tag_TAG2GOAL',
                                            tag_id=config.pick.tag_id,
                                            stand_in_tag_pos=config.pick.stand_in_tag_pos,
                                            stand_in_tag_euler=config.pick.stand_in_tag_euler)

PERCEP = NodePercep(name='PERCEP', robot_sdk=robot_sdk, tag_ids=[config.pick.tag_id, config.place.tag_id])
ACTION = py_trees.composites.Sequence(name="ACTION", memory=True)
root = py_trees.composites.Parallel(name="root", policy=py_trees.common.ParallelPolicy.SuccessOnOne())
root.add_children([ACTION, PERCEP])

# 1. 寻找箱子
move_head_to_find_tag = NodeFuntion(name="move_head_to_find_tag",
                        fn=lambda: move_head(flag=True, yaw=-20, pitch=-10))

search_pick_tag_GUESS = NodeFuntion(name="search_pick_tag_GUESS",
                                    fn=lambda: update_tag_guess(tag_id=config.pick.tag_id,
                                                                tag_pos_world=config.pick.tag_pos_world,
                                                                tag_euler_world=config.pick.tag_euler_world))
search_pick_tag_CONDITION = NodeWaitForBlackboard(key=f"latest_tag_{config.pick.tag_id}")

search_pick_tag = py_trees.composites.Sequence(name="search_pick_tag", memory=True)
search_pick_tag.add_children(
    [move_head_to_find_tag, search_pick_tag_GUESS, search_pick_tag_WALK, search_pick_tag_CONDITION, search_pick_tag_TAG2GOAL])

# 2. 走过去到箱子的位置，同时中途持续识别
walk_to_pick_WALk = NodeWalk(name='walk_to_pick_WALk', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold)
walk_to_pick_TAG2GOAL = py_trees.decorators.SuccessIsRunning(name="walk_to_pick_TAG2GOAL",
                                                             child=NodeTagToNavGoal(name='walk_to_pick_TAG2GOAL_',
                                                                                    tag_id=config.pick.tag_id,
                                                                                    stand_in_tag_pos=config.pick.stand_in_tag_pos,
                                                                                    stand_in_tag_euler=config.pick.stand_in_tag_euler))

# 转腰控制节点 - 在走向箱子时并行执行，距离小于0.3米时开始转腰
walk_to_pick_WAIST = NodeWaist(name='walk_to_pick_WAIST',
                               robot_sdk=robot_sdk,
                               waist_pos=config.pick.waist_pos,
                               distance_threshold=0.3)

walk_to_pick = py_trees.composites.Parallel(name="walk_to_pick",
                                            policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
                                                children=[walk_to_pick_WALk, walk_to_pick_WAIST]))
if config.common.enable_percep_when_walking==True:
    walk_to_pick.add_children([walk_to_pick_WALk, walk_to_pick_TAG2GOAL, walk_to_pick_WAIST])
else:
    walk_to_pick.add_children([walk_to_pick_WALk, walk_to_pick_WAIST])

# 3. 拿起箱子并后退
left_arm_relative_keypoints, right_arm_relative_keypoints = arm_generate_pick_keypoints(
    box_width=config.common.box_width,
    tag_dx=config.pick.tag_dx,  # 箱子在tag坐标系x方向偏移的距离，单位米
    tag_dy=config.pick.tag_dy,  # 箱子在tag坐标系y方向偏移的距离，单位米
    tag_dz=config.pick.tag_dz,  # 箱子在tag坐标系z方向偏移的距离，单位米
    step_back_distance=-config.common.step_back_distance+0.1
)

# 前三个点的关键点
left_arm_first_three_keypoints = left_arm_relative_keypoints[:3]
right_arm_first_three_keypoints = right_arm_relative_keypoints[:3]

# 第四个点的关键点
left_arm_fourth_keypoint = [left_arm_relative_keypoints[3]]
right_arm_fourth_keypoint = [right_arm_relative_keypoints[3]]

# 前三个点的执行
pick_box_first_three_TAG2GOAL = NodeTagToArmGoal(name='pick_box_first_three_TAG2GOAL',
                                                  arm_api=arm_api,
                                                  tag_id=config.pick.tag_id,
                                                  left_arm_relative_keypoints=left_arm_first_three_keypoints,
                                                  right_arm_relative_keypoints=right_arm_first_three_keypoints)

pick_box_first_three_ARM = NodeArm(name='pick_box_first_three_ARM', arm_api=arm_api)

# 第四个点的执行
pick_box_fourth_TAG2GOAL = NodeTagToArmGoal(name='pick_box_fourth_TAG2GOAL',
                                             arm_api=arm_api,
                                             tag_id=config.pick.tag_id,
                                             left_arm_relative_keypoints=left_arm_fourth_keypoint,
                                             right_arm_relative_keypoints=right_arm_fourth_keypoint)

pick_box_fourth_ARM = NodeArm(name='pick_box_fourth_ARM', arm_api=arm_api)

# KMPC控制切换到全身控制模式
switch_to_full_body_control = NodeFuntion(name="switch_to_full_body_control",
                                           fn=lambda: switch__mpc_control_flow(flag=True))

# 后退动作 - 拿箱子后向后移动
pick_box_SETWALKGOAL = NodeFuntion(name="pick_box_SETWALKGOAL",
                                   fn=lambda: update_walk_goal(target_pose=Pose(
                                       pos=(-config.common.step_back_distance, config.pick.side_step_distance, 0.),  # 向后平移+侧移
                                       quat=(0, 0, 0, 1),  # 保持姿态不变
                                       frame=Frame.BASE  # 使用基座坐标系
                                   )))
pick_box_WALK = NodeWalk(name='pick_box_WALK', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold)

# 前三个点顺序执行
pick_box_first_three = py_trees.composites.Sequence(name="pick_box_first_three", memory=True)
pick_box_first_three.add_children([pick_box_first_three_TAG2GOAL, pick_box_first_three_ARM])

# 转腰回正
waist_to_zero = NodeWaist(name='waist_to_zero',
                               robot_sdk=robot_sdk,
                               waist_pos=0.0,
                               distance_threshold=0.3)

# 转腰顺序执行
waistToZero = py_trees.composites.Sequence(name="waistToZero", memory=True)
waistToZero.add_children([waist_to_zero, switch_to_full_body_control])

# 后退动作顺序执行
pick_box_walk_back = py_trees.composites.Sequence(name="pick_box_walk_back", memory=True)
pick_box_walk_back.add_children([pick_box_SETWALKGOAL, pick_box_WALK])

# 第四个点与后退并行执行
pick_box_fourth_and_walk = py_trees.composites.Parallel(name="pick_box_fourth_and_walk",
                                                         policy=py_trees.common.ParallelPolicy.SuccessOnAll())
pick_box_fourth_and_walk.add_children([waistToZero, pick_box_walk_back])

# 整个拿起箱子的流程：前三个点 -> (第四个点 并行 后退)
pick_box = py_trees.composites.Sequence(name="pick_box", memory=True)
pick_box.add_children([pick_box_first_three, pick_box_fourth_and_walk])

# 4. 找到放置点

move_head_to_find_tag = NodeFuntion(name="move_head_to_find_tag",
                        fn=lambda: move_head(flag=True, yaw=20, pitch=-10))
search_place_tag_GUESS = NodeFuntion(name="search_place_tag_GUESS",
                                     fn=lambda: update_tag_guess(tag_id=config.place.tag_id,
                                                                 tag_pos_world=config.place.tag_pos_world,
                                                                 tag_euler_world=config.place.tag_euler_world))
search_place_tag_CONDITION = NodeWaitForBlackboard(key=f"latest_tag_{config.place.tag_id}")
search_place_tag_TAG2GOAL = NodeTagToNavGoal(name='search_place_tag_TAG2GOAL',
                                             tag_id=config.place.tag_id,
                                             stand_in_tag_pos=config.place.stand_in_tag_pos,
                                             stand_in_tag_euler=config.place.stand_in_tag_euler)
search_place_tag_WALK = NodeWalk(name='search_place_tag_WALK', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold)

search_place_tag = py_trees.composites.Sequence(name="search_place_tag", memory=True)
search_place_tag.add_children(
    [move_head_to_find_tag, search_place_tag_GUESS, search_place_tag_WALK, search_place_tag_CONDITION, search_place_tag_TAG2GOAL])

# 5. 走去放置点，同时中途持续识别
walk_to_place_WALk = NodeWalk(name='walk_to_place_WALk', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold)
walk_to_place_TAG2GOAL = py_trees.decorators.SuccessIsRunning(name="walk_to_place_TAG2GOAL",
                                                              child=NodeTagToNavGoal(name='walk_to_place_TAG2GOAL_',
                                                                                     tag_id=config.pick.tag_id,
                                                                                     stand_in_tag_pos=config.pick.stand_in_tag_pos,
                                                                                     stand_in_tag_euler=config.pick.stand_in_tag_euler))

# 转腰控制节点 - 在走向放置点时并行执行，距离小于0.3米时开始转腰
walk_to_place_WAIST = NodeWaist(name='walk_to_place_WAIST',
                                robot_sdk=robot_sdk,
                                waist_pos=config.place.waist_pos,
                                distance_threshold=0.3)

# 改为并行结构，以行走结束为退出条件
walk_to_place = py_trees.composites.Parallel(name="walk_to_place",
                                             policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
                                                 children=[walk_to_place_WALk, walk_to_place_WAIST]))
if config.common.enable_percep_when_walking==True:
    walk_to_place.add_children([walk_to_place_WALk, walk_to_place_TAG2GOAL, walk_to_place_WAIST])
else:
    walk_to_place.add_children([walk_to_place_WALk, walk_to_place_WAIST])

# 6. 放置箱子并向后移动
left_arm_relative_keypoints, right_arm_relative_keypoints = arm_generate_place_keypoints(
    box_width=config.common.box_width,
    tag_dx=config.place.tag_dx,  # 箱子在tag坐标系x方向偏移的距离，单位米
    tag_dy=config.place.tag_dy,  # 箱子在tag坐标系y方向偏移的距离，单位米
    tag_dz=config.place.tag_dz,  # 箱子在tag坐标系z方向偏移的距离，单位米
    step_back_distance=-config.common.step_back_distance+0.1
)

# 前三个点的关键点
left_arm_place_first_three_keypoints = left_arm_relative_keypoints[:3]
right_arm_place_first_three_keypoints = right_arm_relative_keypoints[:3]

# 第四个点的关键点
left_arm_place_fourth_keypoint = [left_arm_relative_keypoints[3]]
right_arm_place_fourth_keypoint = [right_arm_relative_keypoints[3]]

# 前三个点的执行
place_box_first_three_TAG2GOAL = NodeTagToArmGoal(name='place_box_first_three_TAG2GOAL',
                                                   arm_api=arm_api,
                                                   tag_id=config.place.tag_id,
                                                   left_arm_relative_keypoints=left_arm_place_first_three_keypoints,
                                                   right_arm_relative_keypoints=right_arm_place_first_three_keypoints)

place_box_first_three_ARM = NodeArm(name='place_box_first_three_ARM', arm_api=arm_api)

# 第四个点的执行
place_box_fourth_TAG2GOAL = NodeTagToArmGoal(name='place_box_fourth_TAG2GOAL',
                                              arm_api=arm_api,
                                              tag_id=config.place.tag_id,
                                              left_arm_relative_keypoints=left_arm_place_fourth_keypoint,
                                              right_arm_relative_keypoints=right_arm_place_fourth_keypoint)

place_box_fourth_ARM = NodeArm(name='place_box_fourth_ARM', arm_api=arm_api)

# KMPC控制切换到全身控制模式
switch_to_full_body_control1 = NodeFuntion(name="switch_to_full_body_control1",
                                           fn=lambda: switch__mpc_control_flow(flag=True))

# 后退+侧移动作 - 放箱子后向后移动并侧移到中间位置
place_box_SETWALKGOAL = NodeFuntion(name="place_box_SETWALKGOAL",
                                    fn=lambda: update_walk_goal(target_pose=Pose(
                                        pos=(-config.common.step_back_distance, config.place.side_step_distance, 0.),  # 向后平移+侧移
                                        quat=(0, 0, 0, 1),  # 保持姿态不变
                                        frame=Frame.BASE  # 使用基座坐标系
                                    )))
place_box_WALK = NodeWalk(name='place_box_WALK', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold)

# 前三个点顺序执行
place_box_first_three = py_trees.composites.Sequence(name="place_box_first_three", memory=True)
place_box_first_three.add_children([place_box_first_three_TAG2GOAL, place_box_first_three_ARM])

# 转腰回正
waist_to_zero2 = NodeWaist(name='waist_to_zero2',
                               robot_sdk=robot_sdk,
                               waist_pos=0.0,
                               distance_threshold=0.3)

# 第四个点顺序执行
# place_box_fourth = py_trees.composites.Sequence(name="place_box_fourth", memory=True)
# place_box_fourth.add_children([place_box_fourth_TAG2GOAL, place_box_fourth_ARM, switch_to_full_body_control1])

# 后退+侧移动作顺序执行
place_box_walk_back = py_trees.composites.Sequence(name="place_box_walk_back", memory=True)
place_box_walk_back.add_children([place_box_SETWALKGOAL, place_box_WALK])

# 手臂复位动作
move_arm_to_zero_ = NodeFuntion(name="move_arm",
                                       fn=lambda: move_arm_to_zero())
back_to_origin_ARM_RESET1 = NodeFuntion(name="back_to_origin_ARM_RESET",
                                       fn=lambda: robot_sdk.control.arm_reset())
place_walkBack_armToZero = py_trees.composites.Parallel(name="walkBack_and_armToZero",
                                                        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
                                                            children=[place_box_walk_back]))
place_walkBack_armToZero.add_children([place_box_walk_back, move_arm_to_zero_])

# 整个放置箱子的流程：前三个点 -> 后退+侧移 -> 第四个点（顺序执行）
place_box = py_trees.composites.Sequence(name="place_box", memory=True)
place_box.add_children([place_box_first_three, switch_to_full_body_control1, place_walkBack_armToZero, back_to_origin_ARM_RESET1, waist_to_zero2])

# 7. 回到初始位置
back_to_origin_SETGOAL = NodeFuntion(name="back_to_origin_SETGOAL",
                                     fn=lambda: update_walk_goal(target_pose=Pose.from_euler(
                                         pos=(0, 0, 0),
                                         euler=(0, 0, 0),  # 只旋转yaw角度
                                         frame=Frame.ODOM,  # 使用里程计坐标系
                                         degrees=False
                                     )))
back_to_origin_WALK = NodeWalk(name='walk_to_origin_WALK', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold)


back_to_origin = py_trees.composites.Sequence(name="back_to_origin", memory=True)
back_to_origin.add_children([back_to_origin_SETGOAL, back_to_origin_WALK])

# 创建暂停节点
pause1 = NodeFuntion(name="pause1", fn=lambda: pause_for_next_step("1.寻找箱子", config.common.enable_step_pause))
pause2 = NodeFuntion(name="pause2", fn=lambda: pause_for_next_step("2.走过去到箱子的位置", config.common.enable_step_pause))
pause3 = NodeFuntion(name="pause3", fn=lambda: pause_for_next_step("3.拿起箱子并后退", config.common.enable_step_pause))
pause4 = NodeFuntion(name="pause4", fn=lambda: pause_for_next_step("4.找到放置点", config.common.enable_step_pause))
pause5 = NodeFuntion(name="pause5", fn=lambda: pause_for_next_step("5.走去放置点", config.common.enable_step_pause))
pause6 = NodeFuntion(name="pause6", fn=lambda: pause_for_next_step("6.放置箱子并向后移动", enable_pause=False))
pause7 = NodeFuntion(name="pause7", fn=lambda: pause_for_next_step("7.回到初始位置", enable_pause=False))

ACTION.add_children([search_pick_tag, pause1, walk_to_pick, pause2, pick_box, pause3,
                     search_place_tag, pause4, walk_to_place, pause5, place_box, pause6,
                     back_to_origin, pause7])

#
# /_/ root [*]
#     {-} ACTION [*]
#         {-} search_pick_tag [✓]
#             --> search_pick_tag_GUESS [✓]
#             --> search_pick_tag_WALK [✓]
#             --> WaitFor(latest_tag_1) [✓]
#             --> search_pick_tag_TAG2GOAL [✓]
#         /_/ walk_to_pick [✓]
#             --> walk_to_pick_WALk [✓]
#             -^- walk_to_pick_TAG2GOAL [-] -- success is running []
#                 --> walk_to_pick_TAG2GOAL_ [-]
#         {-} pick_box [✓]
#             --> pick_box_TAG2GOAL [✓]
#             --> pick_box_ARM [✓]
#             --> pick_box_SETWALKGOAL [✓]
#             --> pick_box_WALK [✓]
#         {-} search_place_tag [✓]
#             --> search_place_tag_GUESS [✓]
#             --> search_place_tag_WALK [✓]
#             --> WaitFor(latest_tag_0) [✓]
#             --> search_place_tag_TAG2GOAL [✓]
#         {-} walk_to_place [✓]
#             --> walk_to_place_WALk [✓]
#         {-} place_box [✓]
#             --> place_box_TAG2GOAL [✓]
#             --> place_box_ARM [✓]
#             --> place_box_SETWALKGOAL [✓]
#             --> place_box_WALK [✓]
#         {-} back_to_origin [*]
#             --> back_to_origin_SETGOAL [✓]
#             --> walk_to_origin_WALK [*]
#     --> PERCEP [*]

import time

tick = time.time()

# 步骤间暂停函数
def pause_for_next_step(step_name, enable_pause=None):
    print(f"\n=== 完成步骤: {step_name} ===")
    if enable_pause is None:
        enable_pause = config.common.enable_step_pause
    if enable_pause:
        input("按Enter键继续下一步...")
    return True

if __name__ == '__main__':    
    # 用 Repeat 包裹，让它无限循环
    num_repeats = 10
    looping_root = py_trees.decorators.Repeat(name="RepeatRoot", child=root, num_success=num_repeats)

    tree = py_trees.trees.BehaviourTree(looping_root)

    while True:
        tree.tick()  # 注意是 tree，不是 root
        status = looping_root.status  # 查看根节点的状态
        if status != py_trees.common.Status.RUNNING:
            print("Tree finished:", status)
            break
        # BlackBoardManager.print_blackboard()

        print(py_trees.display.unicode_tree(
            root,
            show_status=True,  # 打印状态
        ))

    print(f'============== 时间 {time.time() - tick} ==============')
