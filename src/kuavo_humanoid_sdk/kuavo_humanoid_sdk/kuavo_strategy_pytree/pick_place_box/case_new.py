from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import NodeWalk, NodePercep, NodeTagToNavGoal, \
    NodeWaitForBlackboard, NodeFuntion, NodeArm, NodeTagToArmGoal, NodeWaist, NodeHead, NodeDirectToArmGoal, \
    NodeWalkWithDistanceMonitor
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import ArmAPI, TorsoAPI, HeadAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
# 根据实际环境选择配置，仿真使用：config_sim，实机使用：config_real
from kuavo_humanoid_sdk.kuavo_strategy_pytree.configs.config_real import config
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Frame
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.funcs import update_walk_goal, update_tag_guess
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.funcs import arm_generate_pick_keypoints, \
    arm_generate_place_keypoints_new, arm_reset, arm_generate_pick_before, get_current_pick_tag_id, update_round_and_tag_id_fn
from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcFrame

from kuavo_humanoid_sdk import KuavoSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.black_board import BlackBoardManager
import py_trees
import time


# 初始化API
robot_sdk = RobotSDK()
arm_api = ArmAPI(
    robot_sdk=robot_sdk,
)
torso_api = TorsoAPI(
    robot_sdk=robot_sdk,
)
head_api = HeadAPI(
    robot_sdk=robot_sdk,
)

# 控制模式选择: "cmd_vel" 或 "cmd_pose_world"
WALK_CONTROL_MODE = "cmd_vel"

# 先构建树

# 处理 config.pick.tag_id 可能是列表的情况
if isinstance(config.pick.tag_id, list):
    pick_tag_id = config.pick.tag_id[0]
else:
    pick_tag_id = config.pick.tag_id

search_pick_tag_WALK = NodeWalk(name='search_pick_tag_WALK', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold)
search_pick_tag_TAG2GOAL = NodeTagToNavGoal(name='search_pick_tag_TAG2GOAL',
                                            tag_id=pick_tag_id,
                                            stand_in_tag_pos=config.pick.stand_in_tag_pos,
                                            stand_in_tag_euler=config.pick.stand_in_tag_euler)

PERCEP = NodePercep(name='PERCEP', robot_sdk=robot_sdk, tag_ids=[pick_tag_id, config.place.tag_id])
ACTION = py_trees.composites.Sequence(name="ACTION", memory=True)
root = py_trees.composites.Parallel(name="root", policy=py_trees.common.ParallelPolicy.SuccessOnOne())
root.add_children([ACTION, PERCEP])

# 1. 寻找箱子
search_pick_tag_GUESS = NodeFuntion(name="search_pick_tag_GUESS",
                                    fn=lambda: update_tag_guess(tag_id=get_current_pick_tag_id(config),
                                                                tag_pos_world=config.pick.tag_pos_world,
                                                                tag_euler_world=config.pick.tag_euler_world))

# 头部搜索节点（内置检查黑板，如果已识别到会直接返回SUCCESS）
search_pick_tag_HEAD = NodeHead(
    name='search_pick_tag_HEAD',
    head_api=head_api,
    head_search_yaws=config.common.head_search_yaws,
    head_search_pitchs=config.common.head_search_pitchs,
    tag_id=pick_tag_id,  # 传入tag_id，用于检查是否识别到
    check_interval=0.3  # 每次转头后等待0.5秒，给视觉识别时间
)

# 等待识别结果节点
search_pick_tag_CONDITION = NodeWaitForBlackboard(key=f"latest_tag_{pick_tag_id}")

# 并行执行：头部搜索 || 等待识别
search_pick_tag_HEAD_AND_WAIT = py_trees.composites.Parallel(
    name="search_pick_tag_HEAD_AND_WAIT",
    policy=py_trees.common.ParallelPolicy.SuccessOnOne()  # 任一成功就退出
)
search_pick_tag_HEAD_AND_WAIT.add_children([search_pick_tag_HEAD, search_pick_tag_CONDITION])

search_pick_tag = py_trees.composites.Sequence(name="search_pick_tag", memory=True)
search_pick_tag.add_children(
    [search_pick_tag_GUESS, search_pick_tag_WALK, search_pick_tag_HEAD_AND_WAIT, search_pick_tag_TAG2GOAL])

# 2. 走到箱子位置，中途持续识别并执行手臂预动作
walk_to_pick_WALk = NodeWalk(name='walk_to_pick_WALk', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold)
walk_to_pick_TAG2GOAL = py_trees.decorators.SuccessIsRunning(name="walk_to_pick_TAG2GOAL",
                                                             child=NodeTagToNavGoal(name='walk_to_pick_TAG2GOAL_',
                                                                                    tag_id=pick_tag_id,
                                                                                    stand_in_tag_pos=config.pick.stand_in_tag_pos,
                                                                                    stand_in_tag_euler=config.pick.stand_in_tag_euler))

left_arm_preaction_poses, right_arm_preaction_poses = arm_generate_pick_before()
walk_to_pick_ARM_GOAL = NodeDirectToArmGoal(
    name='walk_to_pick_ARM_GOAL',
    arm_api=arm_api,
    left_arm_poses=left_arm_preaction_poses,
    right_arm_poses=right_arm_preaction_poses,
    frame=KuavoManipulationMpcFrame.LocalFrame  # 使用LocalFrame
)

walk_to_pick_ARM = NodeArm(
    name='walk_to_pick_ARM',
    arm_api=arm_api,
    control_base=False,
    total_time=1.0, 
    frame=KuavoManipulationMpcFrame.LocalFrame,
    arm_pos_threshold=config.common.arm_pos_threshold,
    arm_angle_threshold=config.common.arm_angle_threshold,
    arm_error_detect=config.common.arm_error_detect
)

# 手臂预动作
pre_pick_arm = py_trees.composites.Sequence(name="pre_pick_arm", memory=True)
pre_pick_arm.add_children([walk_to_pick_ARM_GOAL, walk_to_pick_ARM])

walk_to_pick = py_trees.composites.Parallel(name="walk_to_pick",
                                            policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
                                                children=[walk_to_pick_WALk]))

walk_to_pick.add_children([walk_to_pick_WALk, walk_to_pick_TAG2GOAL, pre_pick_arm])

# 3. 拿起箱子
left_arm_relative_keypoints, right_arm_relative_keypoints = arm_generate_pick_keypoints(
    box_width=config.common.box_width,
    box_behind_tag=config.pick.box_behind_tag,  # 箱子在tag后面的距离，单位米
    box_beneath_tag=config.pick.box_beneath_tag,  # 箱子在tag下方的距离，单位米
    box_left_tag=config.pick.box_left_tag,  # 箱子在tag左侧的距离，单位米
)

# 手臂关键点的执行
pick_box_TAG2GOAL = NodeTagToArmGoal(name='pick_box_TAG2GOAL',
                                                  arm_api=arm_api,
                                                  tag_id=pick_tag_id,
                                                  left_arm_relative_keypoints=left_arm_relative_keypoints,
                                                  right_arm_relative_keypoints=right_arm_relative_keypoints)

pick_box_ARM = NodeArm(name='pick_box_ARM', arm_api=arm_api, control_base=config.common.arm_control_base, total_time=config.pick.arm_total_time, frame=KuavoManipulationMpcFrame.WorldFrame,
                        arm_pos_threshold=config.common.arm_pos_threshold, arm_angle_threshold=config.common.arm_angle_threshold, arm_error_detect=config.common.arm_error_detect)

pick_box = py_trees.composites.Sequence(name="pick_box", memory=True)
pick_box.add_children([pick_box_TAG2GOAL, pick_box_ARM])

# 4. 拿箱子后转腰180度
turn_waist_180 = NodeWaist(name='turn_waist_180',
                           robot_sdk=robot_sdk,
                           waist_pos=config.pick.waist_degree)

# 后退动作
pick_box_SETWALKGOAL = NodeFuntion(name="pick_box_SETWALKGOAL",
                                   fn=lambda: update_walk_goal(target_pose=Pose(
                                       pos=(-config.pick.step_back_distance, 0., 0.),  # 向后平移
                                       quat=(0, 0, 0, 1),  # 保持姿态不变
                                       frame=Frame.BASE  # 使用基座坐标系
                                   )))
pick_box_WALK = NodeWalk(name='pick_box_WALK', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold)

walk_and_turn_waist = py_trees.composites.Parallel(name="walk_and_turn_waist",
                                                    policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
                                                        children=[turn_waist_180]))
walk_and_turn_waist.add_children([pick_box_SETWALKGOAL, pick_box_WALK, turn_waist_180])

# 5. 找到放置点
search_place_tag_GUESS = NodeFuntion(name="search_place_tag_GUESS",
                                     fn=lambda: update_tag_guess(tag_id=config.place.tag_id,
                                                                 tag_pos_world=config.place.tag_pos_world,
                                                                 tag_euler_world=config.place.tag_euler_world))

# 头部搜索节点（内置检查黑板，如果已识别到会直接返回SUCCESS）
search_place_tag_HEAD = NodeHead(
    name='search_place_tag_HEAD',
    head_api=head_api,
    head_search_yaws=config.common.head_search_yaws,
    head_search_pitchs=config.common.head_search_pitchs,
    tag_id=config.place.tag_id,  # 传入tag_id，用于检查是否识别到
    check_interval=0.5  # 每次转头后等待0.5秒，给视觉识别时间
)

# 等待识别结果节点
search_place_tag_CONDITION = NodeWaitForBlackboard(key=f"latest_tag_{config.place.tag_id}")

# 并行执行：头部搜索 || 等待识别
search_place_tag_HEAD_AND_WAIT = py_trees.composites.Parallel(
    name="search_place_tag_HEAD_AND_WAIT",
    policy=py_trees.common.ParallelPolicy.SuccessOnOne()  # 任一成功就退出
)
search_place_tag_HEAD_AND_WAIT.add_children([search_place_tag_HEAD, search_place_tag_CONDITION])

search_place_tag_TAG2GOAL = NodeTagToNavGoal(name='search_place_tag_TAG2GOAL',
                                             tag_id=config.place.tag_id,
                                             stand_in_tag_pos=config.place.stand_in_tag_pos,
                                             stand_in_tag_euler=config.place.stand_in_tag_euler)
search_place_tag_WALK = NodeWalk(name='search_place_tag_WALK', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold, backward_mode=True)

search_place_tag = py_trees.composites.Sequence(name="search_place_tag", memory=True)
search_place_tag.add_children(
    [search_place_tag_GUESS, search_place_tag_WALK, search_place_tag_HEAD_AND_WAIT, search_place_tag_TAG2GOAL])

# 6. 走去放置点，同时中途持续识别
walk_to_place_WALk = NodeWalk(name='walk_to_place_WALk', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold, backward_mode=True)
walk_to_place_TAG2GOAL = py_trees.decorators.SuccessIsRunning(name="walk_to_place_TAG2GOAL",
                                                              child=NodeTagToNavGoal(name='walk_to_place_TAG2GOAL_',
                                                                                     tag_id=config.place.tag_id,
                                                                                     stand_in_tag_pos=config.place.stand_in_tag_pos,
                                                                                     stand_in_tag_euler=config.place.stand_in_tag_euler))

walk_to_place = py_trees.composites.Parallel(name="walk_to_place",
                                            policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
                                                children=[walk_to_place_WALk]))
walk_to_place.add_children([walk_to_place_WALk, walk_to_place_TAG2GOAL])

# 7. 放置箱子并恢复状态
left_arm_relative_keypoints, right_arm_relative_keypoints = arm_generate_place_keypoints_new(
    box_width=config.common.box_width,
    box_behind_tag=config.place.box_behind_tag,  # 箱子在tag后面的距离，单位米
    box_beneath_tag=config.place.box_beneath_tag,  # 箱子在tag下方的距离，单位米
    box_left_tag=config.place.box_left_tag,  # 箱子在tag左侧的距离，单位米
)

# 手臂动作关键点
left_arm_place_keypoints = left_arm_relative_keypoints
right_arm_place_keypoints = right_arm_relative_keypoints

place_box_TAG2GOAL = NodeTagToArmGoal(name='place_box_TAG2GOAL',
                                                   arm_api=arm_api,
                                                   tag_id=config.place.tag_id,
                                                   left_arm_relative_keypoints=left_arm_place_keypoints,
                                                   right_arm_relative_keypoints=right_arm_place_keypoints)

place_box_ARM = NodeArm(name='place_box_ARM', arm_api=arm_api, control_base=config.common.arm_control_base, total_time=config.place.arm_total_time, frame=KuavoManipulationMpcFrame.WorldFrame,
                        arm_pos_threshold=config.common.arm_pos_threshold, arm_angle_threshold=config.common.arm_angle_threshold, arm_error_detect=config.common.arm_error_detect)

place_box = py_trees.composites.Sequence(name="place_box", memory=True)
place_box.add_children([place_box_TAG2GOAL, place_box_ARM])

#8. 手臂复位
back_to_origin_ARM_RESET1 = NodeFuntion(name="back_to_origin_ARM_RESET",
                                       fn=lambda: arm_reset())

# 后退动作（带距离监控，用于触发转腰）
place_box_SETWALKGOAL = NodeFuntion(name="place_box_SETWALKGOAL",
                                    fn=lambda: update_walk_goal(target_pose=Pose(
                                        pos=(-config.place.step_back_distance, 0., 0.),  # 向后平移
                                        quat=(0, 0, 0, 1),  # 保持姿态不变
                                        frame=Frame.BASE  # 使用基座坐标系
                                    )))

# 行走距离检测，后退trigger_distance距离后得到key=“walk_distance_trigger_reached”
place_box_WALK = NodeWalkWithDistanceMonitor(
    trigger_distance=0.1,  # 后退0.1m后触发转腰
    name='place_box_WALK',
    torso_api=torso_api,
    control_mode=WALK_CONTROL_MODE,
    pos_threshold=config.common.walk_pos_threshold
)

turn_waist_0 = NodeWaist(name='turn_waist_0',
                         robot_sdk=robot_sdk,
                         waist_pos=config.place.waist_degree)

# 等待后退达到0.1m触发条件
wait_for_distance_trigger = NodeWaitForBlackboard(key='walk_distance_trigger_reached')

# 创建Sequence：等待触发后开始转腰（memory=False确保每次都重新执行）
turn_waist_after_trigger = py_trees.composites.Sequence(name="turn_waist_after_trigger", memory=False)
turn_waist_after_trigger.add_children([wait_for_distance_trigger, turn_waist_0])

# 后退、手臂重置和转腰并行执行，后退完成时整体完成
walk_and_turn = py_trees.composites.Parallel(name="walk_and_turn",
                                              policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
                                                  children=[place_box_WALK, turn_waist_after_trigger]))
walk_and_turn.add_children([back_to_origin_ARM_RESET1, place_box_SETWALKGOAL, place_box_WALK, turn_waist_after_trigger])

# 9. 回到初始位置
back_to_origin_SETGOAL = NodeFuntion(name="back_to_origin_SETGOAL",
                                     fn=lambda: update_walk_goal(target_pose=Pose.from_euler(
                                         pos=(0, 0, 0),
                                         euler=(0, 0, -90),  # 只旋转yaw角度
                                         frame=Frame.ODOM,  # 使用里程计坐标系
                                         degrees=False
                                     )))
back_to_origin_WALK = NodeWalk(name='walk_to_origin_WALK', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold)

back_to_origin = py_trees.composites.Sequence(name="back_to_origin", memory=True)
back_to_origin.add_children([back_to_origin_SETGOAL, back_to_origin_WALK])

# 创建更新轮次和 tag_id 的函数（在所有节点创建后）
update_round_and_tag_id_fn = update_round_and_tag_id_fn(
    config, search_pick_tag_TAG2GOAL, search_pick_tag_HEAD, pick_box_TAG2GOAL,
    walk_to_pick_TAG2GOAL, PERCEP, search_pick_tag_HEAD_AND_WAIT
)
update_round_node = NodeFuntion(name="update_round_node", fn=update_round_and_tag_id_fn)

# 创建暂停节点，方便调试
pause1 = NodeFuntion(name="pause1", fn=lambda: pause_for_next_step("1.寻找箱子", config.common.enable_step_pause))
pause2 = NodeFuntion(name="pause2", fn=lambda: pause_for_next_step("2.走到箱子位置，中途持续识别并执行手臂预动作", config.common.enable_step_pause))
pause3 = NodeFuntion(name="pause3", fn=lambda: pause_for_next_step("3.拿起箱子并后退", config.common.enable_step_pause))
pause4 = NodeFuntion(name="pause4", fn=lambda: pause_for_next_step("4.拿箱子后转腰180度", config.common.enable_step_pause))
pause5 = NodeFuntion(name="pause5", fn=lambda: pause_for_next_step("5.找到放置点", config.common.enable_step_pause))
pause6 = NodeFuntion(name="pause6", fn=lambda: pause_for_next_step("6.走去放置点，同时中途持续识别", config.common.enable_step_pause))
pause7 = NodeFuntion(name="pause7", fn=lambda: pause_for_next_step("7.放置箱子并恢复状态", config.common.enable_step_pause))
pause8 = NodeFuntion(name="pause8", fn=lambda: pause_for_next_step("8.手臂与腰部复位以及后退", config.common.enable_step_pause))
pause9 = NodeFuntion(name="pause9", fn=lambda: pause_for_next_step("9.回到初始位置", enable_pause=True))
pause10 = NodeFuntion(name="pause9", fn=lambda: pause_for_next_step("10.完成一轮搬箱子", enable_pause=config.common.enable_round_stop))

ACTION.add_children([update_round_node, search_pick_tag, pause1, walk_to_pick, pause2, pick_box, pause3, walk_and_turn_waist, pause4,
                     search_place_tag, pause5, walk_to_place, pause6, place_box, pause7,walk_and_turn, pause8, pause10])
# 行为树
# /_/ root [*]
#     {-} ACTION [*]
#         {-} search_pick_tag [✓]
#             --> search_pick_tag_GUESS [✓]
#             --> search_pick_tag_WALK [✓]
#             /_/ search_pick_tag_HEAD_AND_WAIT [✓]
#                 --> search_pick_tag_HEAD [✓]
#                 --> WaitFor(latest_tag_1) [✓]
#             --> search_pick_tag_TAG2GOAL [✓]
#         --> pause1 [✓]
#         /_/ walk_to_pick [✓]
#             --> walk_to_pick_WALk [✓]
#             -^- walk_to_pick_TAG2GOAL [-] -- success is running []
#                 --> walk_to_pick_TAG2GOAL_ [-]
#             {-} pre_pick_arm [✓]
#                 --> walk_to_pick_ARM_GOAL [✓]
#                 --> walk_to_pick_ARM [✓]
#         --> pause2 [✓]
#         {-} pick_box [✓]
#             --> pick_box_TAG2GOAL [✓]
#             --> pick_box_ARM [✓]
#         --> pause3 [✓]
#         /_/ walk_and_turn_waist [✓]
#             --> pick_box_SETWALKGOAL [✓]
#             --> pick_box_WALK [✓]
#             --> turn_waist_180 [✓]
#         --> pause4 [✓]
#         {-} search_place_tag [✓]
#             --> search_place_tag_GUESS [✓]
#             --> search_place_tag_WALK [✓]
#             /_/ search_place_tag_HEAD_AND_WAIT [✓]
#                 --> search_place_tag_HEAD [✓]
#                 --> WaitFor(latest_tag_0) [✓]
#             --> search_place_tag_TAG2GOAL [✓]
#         --> pause5 [✓]
#         /_/ walk_to_place [✓]
#             --> walk_to_place_WALk [✓]
#             -^- walk_to_place_TAG2GOAL [-] -- success is running []
#                 --> walk_to_place_TAG2GOAL_ [-]
#         --> pause6 [✓]
#         {-} place_box [✓]
#             --> place_box_TAG2GOAL [✓]
#             --> place_box_ARM [✓]
#         --> pause7 [✓]
#         /_/ walk_and_turn [*]
#             --> back_to_origin_ARM_RESET [✓]
#             --> place_box_SETWALKGOAL [✓]
#             --> place_box_WALK [✓]
#             [-] turn_waist_after_trigger [*]
#                 --> WaitFor(walk_distance_trigger_reached) [✓]
#                 --> turn_waist_0 [*]
#         --> pause8 [-]
#         --> pause9 [-]
#     --> PERCEP [*]

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
    # robot_sdk.control.control_head(0, np.deg2rad(-10))
    KuavoSDK.Init(log_level="INFO")
    # 用 Repeat 包裹，让它无限循环
    num_repeats = config.common.grab_box_num
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
