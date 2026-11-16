from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import NodeWalk, NodePercep, NodeTagToNavGoal, \
    NodeWaitForBlackboard, NodeFuntion, NodeArm, NodeTagToArmGoal, NodeWaist, NodeHead, NodeDirectToArmGoal
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import ArmAPI, TorsoAPI, HeadAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
# 根据环境变量 KUAVO_REAL 自动选择配置：默认实机使用 config_real，传 KUAVO_REAL=false 使用 config_sim
import os
_is_real = os.environ.get('KUAVO_REAL', 'true').lower() == 'true'
if _is_real:
    from kuavo_humanoid_sdk.kuavo_strategy_pytree.configs.config_real import config
else:
    from kuavo_humanoid_sdk.kuavo_strategy_pytree.configs.config_sim import config
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Frame
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.funcs import update_walk_goal
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.funcs import arm_generate_pick_keypoints, \
    arm_generate_place_keypoints_new, arm_reset, arm_generate_pick_before, update_round_and_tag_id_fn
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

search_pick_tag_TAG2GOAL = NodeTagToNavGoal(name='search_pick_tag_TAG2GOAL',
                                            tag_id=pick_tag_id,
                                            stand_in_tag_pos=config.pick.stand_in_tag_pos,
                                            stand_in_tag_euler=config.pick.stand_in_tag_euler)

PERCEP = NodePercep(name='PERCEP', robot_sdk=robot_sdk, tag_ids=[pick_tag_id, config.place.tag_id])
ACTION = py_trees.composites.Sequence(name="ACTION", memory=True)
root = py_trees.composites.Parallel(name="root", policy=py_trees.common.ParallelPolicy.SuccessOnOne())
root.add_children([ACTION, PERCEP])

# 1. 寻找箱子

# 头部搜索节点（内置检查黑板，如果已识别到会直接返回SUCCESS）
search_pick_tag_HEAD = NodeHead(
    name='search_pick_tag_HEAD',
    head_api=head_api,
    head_search_yaws=config.common.head_search_yaws,
    head_search_pitchs=config.common.head_search_pitchs,
    tag_id=pick_tag_id,  # 传入tag_id，用于检查是否识别到
    check_interval=0.3  # 每次转头后等待0.5秒，给视觉识别时间
)

# 等待识别结果节点（第一轮扫描使用）
search_pick_tag_CONDITION = NodeWaitForBlackboard(key=f"latest_tag_{pick_tag_id}")

# 并行执行：头部搜索 || 等待识别
search_pick_tag_HEAD_AND_WAIT = py_trees.composites.Parallel(
    name="search_pick_tag_HEAD_AND_WAIT",
    policy=py_trees.common.ParallelPolicy.SuccessOnOne()  # 任一成功就退出
)
search_pick_tag_HEAD_AND_WAIT.add_children([search_pick_tag_HEAD, search_pick_tag_CONDITION])

# scan_direct: 从当前位置直接扫描 tag
search_pick_tag_SCAN_DIRECT = py_trees.composites.Sequence(name="search_pick_tag_SCAN_DIRECT", memory=True)
search_pick_tag_SCAN_DIRECT.add_children([search_pick_tag_HEAD_AND_WAIT])

# detect_tag: 只本地扫描，扫描不到不走路
search_pick_tag_DETECT = py_trees.composites.Selector(name="search_pick_tag_DETECT", memory=True)
search_pick_tag_DETECT.add_children([search_pick_tag_SCAN_DIRECT])

# search_pick_tag: 检测到 tag 后设定导航目标
search_pick_tag = py_trees.composites.Sequence(name="search_pick_tag", memory=True)
search_pick_tag.add_children([search_pick_tag_DETECT, search_pick_tag_TAG2GOAL])

# 2. 走到箱子位置，中途持续识别并执行手臂预动作
walk_to_pick_WALk = NodeWalk(name='walk_to_pick_WALk', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold, max_vel_x=config.common.walk_max_vel_x, max_vel_y=config.common.walk_max_vel_y)
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
pick_box_WALK = NodeWalk(name='pick_box_WALK', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold, max_vel_x=config.common.walk_max_vel_x, max_vel_y=config.common.walk_max_vel_y)

walk_and_turn_waist = py_trees.composites.Parallel(name="walk_and_turn_waist",
                                                    policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
                                                        children=[turn_waist_180]))
walk_and_turn_waist.add_children([pick_box_SETWALKGOAL, pick_box_WALK, turn_waist_180])

# 5. 找到放置点

# 头部搜索节点（内置检查黑板，如果已识别到会直接返回SUCCESS）
search_place_tag_HEAD = NodeHead(
    name='search_place_tag_HEAD',
    head_api=head_api,
    head_search_yaws=config.common.head_search_yaws,
    head_search_pitchs=config.common.head_search_pitchs,
    tag_id=config.place.tag_id,  # 传入tag_id，用于检查是否识别到
    check_interval=0.5  # 每次转头后等待0.5秒，给视觉识别时间
)

# 导航节点
search_place_tag_TAG2GOAL = NodeTagToNavGoal(name='search_place_tag_TAG2GOAL',
                                             tag_id=config.place.tag_id,
                                             stand_in_tag_pos=config.place.stand_in_tag_pos,
                                             stand_in_tag_euler=config.place.stand_in_tag_euler)

# 等待识别结果节点（第一轮扫描使用）
search_place_tag_CONDITION = NodeWaitForBlackboard(key=f"latest_tag_{config.place.tag_id}")

# 并行执行：头部搜索 || 等待识别
search_place_tag_HEAD_AND_WAIT = py_trees.composites.Parallel(
    name="search_place_tag_HEAD_AND_WAIT",
    policy=py_trees.common.ParallelPolicy.SuccessOnOne()
)
search_place_tag_HEAD_AND_WAIT.add_children([search_place_tag_HEAD, search_place_tag_CONDITION])

# scan_direct: 从当前位置直接扫描 tag
search_place_tag_SCAN_DIRECT = py_trees.composites.Sequence(name="search_place_tag_SCAN_DIRECT", memory=True)
search_place_tag_SCAN_DIRECT.add_children([search_place_tag_HEAD_AND_WAIT])

# detect_tag: 只本地扫描，扫描不到不走路
search_place_tag_DETECT = py_trees.composites.Selector(name="search_place_tag_DETECT", memory=True)
search_place_tag_DETECT.add_children([search_place_tag_SCAN_DIRECT])

# search_place_tag: 检测到 tag 后设定导航目标
search_place_tag = py_trees.composites.Sequence(name="search_place_tag", memory=True)
search_place_tag.add_children([search_place_tag_DETECT, search_place_tag_TAG2GOAL])

# 6. 走去放置点，同时中途持续识别
walk_to_place_WALk = NodeWalk(name='walk_to_place_WALk', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold, backward_mode=True, max_vel_x=config.common.walk_max_vel_x, max_vel_y=config.common.walk_max_vel_y, ramp_duration=10.0)
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

# 7.5 放箱后身体后退（远离桌子，给手臂复位留出空间）
place_body_step_back_SETWALKGOAL = NodeFuntion(name="place_body_step_back_SETWALKGOAL",
                                               fn=lambda: update_walk_goal(target_pose=Pose(
                                                   pos=(config.place.body_step_back_distance, 0., 0.),
                                                   quat=(0, 0, 0, 1),
                                                   frame=Frame.BASE
                                               )))
place_body_step_back_WALK = NodeWalk(name='place_body_step_back_WALK', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold, max_vel_x=config.common.walk_max_vel_x, max_vel_y=config.common.walk_max_vel_y)
place_body_step_back = py_trees.composites.Sequence(name="place_body_step_back", memory=True)
place_body_step_back.add_children([place_body_step_back_SETWALKGOAL, place_body_step_back_WALK])

# 8. 手臂复位，完成后转腰
back_to_origin_ARM_RESET1 = NodeFuntion(name="back_to_origin_ARM_RESET",
                                       fn=lambda: arm_reset())

turn_waist_0 = NodeWaist(name='turn_waist_0',
                         robot_sdk=robot_sdk,
                         waist_pos=config.place.waist_degree)

walk_and_turn = py_trees.composites.Sequence(name="walk_and_turn", memory=True)
walk_and_turn.add_children([back_to_origin_ARM_RESET1, turn_waist_0])

# 9. 回到初始位置（启动时记录的位姿）
_origin_pose = None  # 在 main 中初始化

def _get_origin_pose():
	return _origin_pose
back_to_origin_SETGOAL = NodeFuntion(name="back_to_origin_SETGOAL",
                                     fn=lambda: update_walk_goal(target_pose=_get_origin_pose()))
back_to_origin_WALK = NodeWalk(name='walk_to_origin_WALK', torso_api=torso_api, control_mode=WALK_CONTROL_MODE, pos_threshold=config.common.walk_pos_threshold, max_vel_x=config.common.walk_max_vel_x, max_vel_y=config.common.walk_max_vel_y)

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
                     search_place_tag, pause5, walk_to_place, pause6, place_box, place_body_step_back, pause7, walk_and_turn, pause8,
                     back_to_origin, pause10])
# 行为树
# /_/ root [*]
#     {-} ACTION [*]
#         {-} search_pick_tag [✓]
#             /~ search_pick_tag_DETECT [✓]  -- Selector: 只本地扫描, 扫不到直接FAIL
#                 {-} search_pick_tag_SCAN_DIRECT [✓]
#                     /_/ search_pick_tag_HEAD_AND_WAIT [✓]
#                         --> search_pick_tag_HEAD [✓]
#                         --> WaitFor(latest_tag_1) [✓]
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
#             /~ search_place_tag_DETECT [✓]  -- Selector: 只本地扫描, 扫不到直接FAIL
#                 {-} search_place_tag_SCAN_DIRECT [✓]
#                     /_/ search_place_tag_HEAD_AND_WAIT [✓]
#                         --> search_place_tag_HEAD [✓]
#                         --> WaitFor(latest_tag_0) [✓]
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
#         {-} place_body_step_back [✓]
#             --> place_body_step_back_SETWALKGOAL [✓]
#             --> place_body_step_back_WALK [✓]
#         --> pause7 [✓]
#         {-} walk_and_turn [*]
#             --> back_to_origin_ARM_RESET [✓]
#             /_/ walk_turn_parallel [*]
#                 --> place_box_SETWALKGOAL [✓]
#                 --> place_box_WALK [✓]
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
    # WSSDK 必须：初始化 WebSocket 连接
    if not KuavoSDK.Init(log_level="INFO", websocket_mode=True):
        print("Init KuavoSDK failed, exit!")
        exit(1)

    # 修复: robot_sdk 在模块导入时创建，rosbridge 未就绪时订阅 /sensors_data_raw 失败，
    # SDK 回退订阅了 /sensors_data_raw_shm（无数据）。这里强制重订阅正确话题。
    import roslibpy
    _core = robot_sdk.state._rs_core
    _new_topic = roslibpy.Topic(_core.websocket.client, '/sensors_data_raw', 'kuavo_msgs/sensorsData')
    _new_topic.subscribe(_core._sensors_data_raw_callback)
    print("[FIX] 重新订阅 /sensors_data_raw 完成")

    # robot_sdk.control.control_head(0, np.deg2rad(-10))

    # 记录当前 ODOM 位姿作为返回原点的目标
    odom = robot_sdk.state.odometry
    _origin_pose = Pose(
        pos=odom.position,
        quat=odom.orientation,
        frame=Frame.ODOM,
    )
    print(f"[INIT] 记录原点位姿: pos={odom.position}, ori={odom.orientation}")
    # 用 Repeat 包裹，让它无限循环
    num_repeats = config.common.grab_box_num
    looping_root = py_trees.decorators.Repeat(name="RepeatRoot", child=root, num_success=num_repeats)

    tree = py_trees.trees.BehaviourTree(looping_root)

    tick_count = 0
    while True:
        tree.tick()
        tick_count += 1
        status = looping_root.status
        if status != py_trees.common.Status.RUNNING:
            print("Tree finished:", status)
            break

        time.sleep(0.1)

    print(f'============== 时间 {time.time() - tick} ==============')
