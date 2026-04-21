import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
print(sys.path)
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import NodeWalk, NodePercep, NodeTagToNavGoal, \
    NodeWaitForBlackboard, NodeFuntion, NodeArm, NodeTagToArmGoal
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import ArmAPI, TorsoAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.configs.config_sim import config
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Tag, Frame
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.funcs import update_walk_goal, update_tag_guess
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.funcs import arm_generate_pick_keypoints, \
    arm_generate_place_keypoints

from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.black_board import BlackBoardManager
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

# 先构建树
search_pick_tag_WALK = NodeWalk(name='search_pick_tag_WALK', torso_api=torso_api)
search_pick_tag_TAG2GOAL = NodeTagToNavGoal(name='search_pick_tag_TAG2GOAL',
                                            tag_id=config.pick.tag_id,
                                            stand_in_tag_pos=config.pick.stand_in_tag_pos,
                                            stand_in_tag_euler=config.pick.stand_in_tag_euler)

PERCEP = NodePercep(name='PERCEP', robot_sdk=robot_sdk, tag_ids=[config.pick.tag_id, config.place.tag_id])
ACTION = py_trees.composites.Sequence(name="ACTION", memory=True)
root = py_trees.composites.Parallel(name="root", policy=py_trees.common.ParallelPolicy.SuccessOnOne())
root.add_children([ACTION, PERCEP])

# 1. 寻找箱子
search_pick_tag_GUESS = NodeFuntion(name="search_pick_tag_GUESS",
                                    fn=lambda: update_tag_guess(tag_id=config.pick.tag_id,
                                                                tag_pos_world=config.pick.tag_pos_world,
                                                                tag_euler_world=config.pick.tag_euler_world))
search_pick_tag_CONDITION = NodeWaitForBlackboard(key=f"latest_tag_{config.pick.tag_id}")

search_pick_tag = py_trees.composites.Sequence(name="search_pick_tag", memory=True)
search_pick_tag.add_children(
    [search_pick_tag_GUESS, search_pick_tag_WALK, search_pick_tag_CONDITION, search_pick_tag_TAG2GOAL])

# 2. 走过去到箱子的位置，同时中途持续识别
walk_to_pick_WALk = NodeWalk(name='walk_to_pick_WALk', torso_api=torso_api)
walk_to_pick_TAG2GOAL = py_trees.decorators.SuccessIsRunning(name="walk_to_pick_TAG2GOAL",
                                                             child=NodeTagToNavGoal(name='walk_to_pick_TAG2GOAL_',
                                                                                    tag_id=config.pick.tag_id,
                                                                                    stand_in_tag_pos=config.pick.stand_in_tag_pos,
                                                                                    stand_in_tag_euler=config.pick.stand_in_tag_euler))

# walk_to_pick = py_trees.composites.Sequence(name="walk_to_pick", memory=True)
walk_to_pick = py_trees.composites.Parallel(name="walk_to_pick",
                                            policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
                                                children=[walk_to_pick_WALk]))

walk_to_pick.add_children([walk_to_pick_WALk, walk_to_pick_TAG2GOAL])

# 3. 拿起箱子并后退
left_arm_relative_keypoints, right_arm_relative_keypoints = arm_generate_pick_keypoints(
    box_width=config.common.box_width,
    box_behind_tag=config.pick.box_behind_tag,  # 箱子在tag后面的距离，单位米
    box_beneath_tag=config.pick.box_beneath_tag,  # 箱子在tag下方的距离，单位米
    box_left_tag=config.pick.box_left_tag,  # 箱子在tag左侧的距离，单位米
)

pick_box_TAG2GOAL = NodeTagToArmGoal(name='pick_box_TAG2GOAL',
                                     arm_api=arm_api,
                                     tag_id=config.pick.tag_id,
                                     left_arm_relative_keypoints=left_arm_relative_keypoints,
                                     right_arm_relative_keypoints=right_arm_relative_keypoints)

pick_box_ARM = NodeArm(name='pick_box_ARM', arm_api=arm_api)
pick_box_SETWALKGOAL = NodeFuntion(name="pick_box_SETWALKGOAL",
                                   fn=lambda: update_walk_goal(target_pose=Pose(
                                       pos=(-config.common.step_back_distance, 0., 0.),  # 向后平移
                                       quat=(0, 0, 0, 1),  # 保持姿态不变
                                       frame=Frame.BASE  # 使用基座坐标系
                                   )))
pick_box_WALK = NodeWalk(name='pick_box_WALK', torso_api=torso_api)

pick_box = py_trees.composites.Sequence(name="pick_box", memory=True)
pick_box.add_children([pick_box_TAG2GOAL, pick_box_ARM, pick_box_SETWALKGOAL, pick_box_WALK])

# 4. 找到放置点
search_place_tag_GUESS = NodeFuntion(name="search_place_tag_GUESS",
                                     fn=lambda: update_tag_guess(tag_id=config.place.tag_id,
                                                                 tag_pos_world=config.place.tag_pos_world,
                                                                 tag_euler_world=config.place.tag_euler_world))
search_place_tag_CONDITION = NodeWaitForBlackboard(key=f"latest_tag_{config.place.tag_id}")
search_place_tag_TAG2GOAL = NodeTagToNavGoal(name='search_place_tag_TAG2GOAL',
                                             tag_id=config.place.tag_id,
                                             stand_in_tag_pos=config.place.stand_in_tag_pos,
                                             stand_in_tag_euler=config.place.stand_in_tag_euler)
search_place_tag_WALK = NodeWalk(name='search_place_tag_WALK', torso_api=torso_api)

search_place_tag = py_trees.composites.Sequence(name="search_place_tag", memory=True)
search_place_tag.add_children(
    [search_place_tag_GUESS, search_place_tag_WALK, search_place_tag_CONDITION, search_place_tag_TAG2GOAL])

# 5. 走去放置点，同时中途持续识别
walk_to_place_WALk = NodeWalk(name='walk_to_place_WALk', torso_api=torso_api)
walk_to_place_TAG2GOAL = py_trees.decorators.SuccessIsRunning(name="walk_to_place_TAG2GOAL",
                                                              child=NodeTagToNavGoal(name='walk_to_place_TAG2GOAL_',
                                                                                     tag_id=config.pick.tag_id,
                                                                                     stand_in_tag_pos=config.pick.stand_in_tag_pos,
                                                                                     stand_in_tag_euler=config.pick.stand_in_tag_euler))

walk_to_place = py_trees.composites.Sequence(name="walk_to_place", memory=True)
walk_to_place.add_children([walk_to_place_WALk])

# 6. 放置箱子并向后移动
left_arm_relative_keypoints, right_arm_relative_keypoints = arm_generate_place_keypoints(
    box_width=config.common.box_width,
    box_behind_tag=config.place.box_behind_tag,  # 箱子在tag后面的距离，单位米
    box_beneath_tag=config.place.box_beneath_tag,  # 箱子在tag下方的距离，单位米
    box_left_tag=config.place.box_left_tag,  # 箱子在tag左侧的距离，单位米
)

place_box_TAG2GOAL = NodeTagToArmGoal(name='place_box_TAG2GOAL',
                                      arm_api=arm_api,
                                      tag_id=config.place.tag_id,
                                      left_arm_relative_keypoints=left_arm_relative_keypoints,
                                      right_arm_relative_keypoints=right_arm_relative_keypoints)

place_box_ARM = NodeArm(name='place_box_ARM', arm_api=arm_api)
place_box_SETWALKGOAL = NodeFuntion(name="place_box_SETWALKGOAL",
                                    fn=lambda: update_walk_goal(target_pose=Pose(
                                        pos=(-config.common.step_back_distance, 0., 0.),  # 向后平移
                                        quat=(0, 0, 0, 1),  # 保持姿态不变
                                        frame=Frame.BASE  # 使用基座坐标系
                                    )))
place_box_WALK = NodeWalk(name='place_box_WALK', torso_api=torso_api)

place_box = py_trees.composites.Sequence(name="place_box", memory=True)
place_box.add_children([place_box_TAG2GOAL, place_box_ARM, place_box_SETWALKGOAL, place_box_WALK])

# 7. 回到初始位置
back_to_origin_SETGOAL = NodeFuntion(name="back_to_origin_SETGOAL",
                                     fn=lambda: update_walk_goal(target_pose=Pose.from_euler(
                                         pos=(0, 0, 0),
                                         euler=(0, 0, 0),  # 只旋转yaw角度
                                         frame=Frame.ODOM,  # 使用里程计坐标系
                                         degrees=False
                                     )))
back_to_origin_WALK = NodeWalk(name='walk_to_origin_WALK', torso_api=torso_api)
back_to_origin = py_trees.composites.Sequence(name="back_to_origin", memory=True)
back_to_origin.add_children([back_to_origin_SETGOAL, back_to_origin_WALK])

ACTION.add_children([search_pick_tag, walk_to_pick, pick_box, search_place_tag, walk_to_place, place_box])

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

if __name__ == '__main__':
    robot_sdk.control.control_head(0, np.deg2rad(-10))
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
