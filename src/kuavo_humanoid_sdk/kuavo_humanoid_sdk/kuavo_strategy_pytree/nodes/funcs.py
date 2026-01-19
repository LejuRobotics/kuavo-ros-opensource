from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import ArmAPI, TorsoAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Tag, Frame
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import NodeWaitForBlackboard
import py_trees
from py_trees.common import Access
import numpy as np
import time
# 初始化API
robot_sdk = RobotSDK()
arm_api = ArmAPI(
    robot_sdk=robot_sdk,
)
torso_api = TorsoAPI(
    robot_sdk=robot_sdk,
)


def update_walk_goal(target_pose: Pose):
    # 启动前：写入初始值
    bb = py_trees.blackboard.Client(name="update_walk_goal")
    bb.register_key("tag_id", Access.WRITE)
    bb.register_key("walk_goal", Access.WRITE)
    bb.register_key("is_walk_goal_new", Access.WRITE)

    bb.walk_goal = target_pose
    bb.is_walk_goal_new = True

    return True


def update_tag_guess(
        tag_id,
        tag_pos_world,
        tag_euler_world
):
    init_tag_guess = Tag(
        id=tag_id,  # 假设目标箱子的ID为1
        pose=Pose.from_euler(
            pos=tag_pos_world,  # 初始位置猜测，单位米
            euler=tag_euler_world,  # 初始姿态猜测，单位四元数
            frame=Frame.ODOM,  # 使用里程计坐标系
            degrees=False
        )
    )

    robot_pose = Pose(
        pos=robot_sdk.state.robot_position(),
        quat=robot_sdk.state.robot_orientation()
    )

    tag_pose = init_tag_guess.pose

    # 计算目标相对于机器人的位置向量
    dx = tag_pose.pos[0] - robot_pose.pos[0]
    dy = tag_pose.pos[1] - robot_pose.pos[1]
    target_direction = np.arctan2(dy, dx)

    target_pose = Pose.from_euler(
        pos=robot_sdk.state.robot_position(),
        euler=(0, 0, target_direction),  # 只旋转yaw角度
        frame=Frame.ODOM,  # 使用里程计坐标系
        degrees=False
    )

    update_walk_goal(target_pose)

    return True

def arm_generate_pick_before():
    # 手臂预抓取动作
    # 这里以机器人坐标系为基准，这样手臂动作总是相对于机器人，机器人坐标系位于基座在地面的投影点
    pick_left_arm_poses = [
        Pose.from_euler(pos=(0.4, 0.4, 1.1), euler=(0, -90, 0),
                        degrees=True,
                        frame=Frame.ROBOT)]

    pick_right_arm_poses = [
        Pose.from_euler(pos=(0.4, -0.4, 1.1), euler=(0, -90, 0),
                        degrees=True,
                        frame=Frame.ROBOT)]

    return pick_left_arm_poses, pick_right_arm_poses

def arm_generate_pick_boxes_before():
    # 手臂预抓取动作
    # 这里以机器人坐标系为基准，这样手臂动作总是相对于机器人，机器人坐标系位于基座在地面的投影点
    pick_left_arm_poses = [
        Pose.from_euler(pos=(-0.4, -0.4, 1.1), euler=(0, -90, 180),
                        degrees=True,
                        frame=Frame.ROBOT)]

    pick_right_arm_poses = [
        Pose.from_euler(pos=(-0.4, 0.4, 1.1), euler=(0, -90, 180),
                        degrees=True,
                        frame=Frame.ROBOT)]

    return pick_left_arm_poses, pick_right_arm_poses

def arm_generate_pick_keypoints(
        box_width: float,
        box_behind_tag: float,  # 箱子在tag后面的距离，单位米
        box_beneath_tag: float,  # 箱子在tag下方的距离，单位米
        box_left_tag: float,  # 箱子在tag左侧的距离，单位米
        hand_pitch_degree: float = 0.0,  # 手臂pitch角度（相比水平, 下倾是正），单位度
):
    pick_left_arm_poses = [
        # # 1. 预抓取点位
        Pose.from_euler(pos=(-box_width * 3 / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 3. 抬升点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 4. 收臂点位
        Pose.from_euler(pos=(0.4, box_width / 2, 0.25), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                        frame=Frame.BASE)]

    pick_right_arm_poses = [
        Pose.from_euler(pos=(box_width * 3 / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 3. 抬升点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 4. 收臂点位
        Pose.from_euler(pos=(0.4, -box_width / 2, 0.25), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                        frame=Frame.BASE),
    ]

    return pick_left_arm_poses, pick_right_arm_poses


def arm_generate_place_keypoints_new(
        box_width: float,
        box_behind_tag: float,  # 箱子在tag后面的距离，单位米
        box_beneath_tag: float,  # 箱子在tag下方的距离，单位米
        box_left_tag: float,  # 箱子在tag左侧的距离，单位米
):
    place_left_arm_poses = [
        # 1. 上方点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 3. 打开点位
        Pose.from_euler(pos=(-box_width * 3 / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag),
                        euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 4. 收臂点位
        Pose.from_euler(pos=(-0.4, -0.4, 0.1), euler=(0, -90, 180), degrees=True, frame=Frame.BASE),
    ]
    place_right_arm_poses = [
        # 1. 上方点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 3. 打开点位
        Pose.from_euler(pos=(box_width * 3 / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag),
                        euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 4. 收臂点位
        Pose.from_euler(pos=(-0.4, 0.4, 0.1), euler=(0, -90, 180), degrees=True, frame=Frame.BASE),
    ]  # 手臂关键点数据，假设为空列表

    return place_left_arm_poses, place_right_arm_poses

def arm_generate_pick_boxes_keypoints(
        box_width: float,
        box_behind_tag: float,  # 箱子在tag后面的距离，单位米
        box_beneath_tag: float,  # 箱子在tag下方的距离，单位米
        box_left_tag: float,  # 箱子在tag左侧的距离，单位米
        hand_pitch_degree: float = 0.0,  # 手臂pitch角度（相比水平, 下倾是正），单位度
):
    pick_left_arm_poses = [
        # # 1. 预抓取点位
        Pose.from_euler(pos=(-box_width * 3.2 / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 3. 抬升点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 4. 收臂点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag + 0.35, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG)
        ]

    pick_right_arm_poses = [
        Pose.from_euler(pos=(box_width * 3.2 / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 3. 抬升点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 4. 收臂点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag + 0.35, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG)
    ]

    return pick_left_arm_poses, pick_right_arm_poses


def arm_generate_place_boxes_keypoints(
        box_width: float,
        box_behind_tag: float,  # 箱子在tag后面的距离，单位米
        box_beneath_tag: float,  # 箱子在tag下方的距离，单位米
        box_left_tag: float,  # 箱子在tag左侧的距离，单位米
):
    place_left_arm_poses = [
        # 1. 上方点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 3. 打开点位
        Pose.from_euler(pos=(-box_width * 3 / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag),
                        euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 4. 收臂点位
        # Pose.from_euler(pos=(0.4, 0.4, 0.1), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
    ]
    place_right_arm_poses = [
        # 1. 上方点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 3. 打开点位
        Pose.from_euler(pos=(box_width * 3 / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag),
                        euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 4. 收臂点位
        # Pose.from_euler(pos=(0.4, -0.4, 0.1), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
    ]  # 手臂关键点数据，假设为空列表

    return place_left_arm_poses, place_right_arm_poses

def arm_generate_place_keypoints(
        box_width: float,
        box_behind_tag: float,  # 箱子在tag后面的距离，单位米
        box_beneath_tag: float,  # 箱子在tag下方的距离，单位米
        box_left_tag: float,  # 箱子在tag左侧的距离，单位米
):
    place_left_arm_poses = [
        # 1. 上方点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 3. 打开点位
        Pose.from_euler(pos=(-box_width * 3 / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag),
                        euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 4. 收臂点位
        Pose.from_euler(pos=(0.4, 0.4, 0.1), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
    ]
    place_right_arm_poses = [
        # 1. 上方点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 3. 打开点位
        Pose.from_euler(pos=(box_width * 3 / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag),
                        euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 4. 收臂点位
        Pose.from_euler(pos=(0.4, -0.4, 0.1), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
    ]  # 手臂关键点数据，假设为空列表

    return place_left_arm_poses, place_right_arm_poses

def time_sleep(seconds: float):
    time.sleep(seconds)
    return True

def arm_reset():
    """Reset arms with simple retry until success."""
    max_retry = 5
    for _ in range(max_retry):
        status = robot_sdk.control.arm_reset()
        if status:
            return True
        time.sleep(0.2)
    return False

def robot_stance():
    for i in range(1):
        status = robot_sdk.control.stance()
        time.sleep(0.05)
    print("stance_status:",status)
    return status

def get_current_pick_tag_id(config):
    """从 BlackBoard 读取当前轮次，返回对应的 tag_id"""
    bb = py_trees.blackboard.Blackboard()
    try:
        round_index = bb.get('current_round')
    except KeyError:
        round_index = 0
    tag_id_list = config.pick.tag_id if isinstance(config.pick.tag_id, list) else [config.pick.tag_id]
    return tag_id_list[round_index % len(tag_id_list)]


def update_round_and_tag_id_fn(config, search_pick_tag_TAG2GOAL, search_pick_tag_HEAD, 
                                      pick_box_TAG2GOAL, walk_to_pick_TAG2GOAL, PERCEP, 
                                      search_pick_tag_HEAD_AND_WAIT):
    """
    更新轮次和 tag_id 的函数
    
    Returns:
        一个函数，每次调用时更新轮次和所有相关节点的 tag_id
    """
    def update_round_and_tag_id():
        bb = py_trees.blackboard.Blackboard()
        try:
            current_round = bb.get('current_round')
        except KeyError:
            current_round = -1
        new_round = current_round + 1
        bb.set('current_round', new_round)
        
        # 获取当前轮次对应的 tag_id
        tag_id_list = config.pick.tag_id if isinstance(config.pick.tag_id, list) else [config.pick.tag_id]
        # 使用取余数确保当轮次超过列表长度时，循环使用列表中的值
        # 例如：tag_id=[1,2]，grab_box_num=10，则循环使用 [1,2,1,2,1,2,1,2,1,2]
        current_tag_id = tag_id_list[new_round % len(tag_id_list)]
        
        print(f"========== 开始第 {new_round + 1} 轮，使用 tag_id: {current_tag_id} ==========")
        
        # 更新所有相关节点的 tag_id
        search_pick_tag_TAG2GOAL.tag_id = current_tag_id
        search_pick_tag_HEAD.tag_id = current_tag_id
        pick_box_TAG2GOAL.tag_id = current_tag_id
        
        # 更新 walk_to_pick_TAG2GOAL 内部节点的 tag_id
        # walk_to_pick_TAG2GOAL 是 SuccessIsRunning 装饰器，使用 decorated 属性访问子节点
        if hasattr(walk_to_pick_TAG2GOAL, 'decorated') and hasattr(walk_to_pick_TAG2GOAL.decorated, 'tag_id'):
            print(f"更新前 walk_to_pick_TAG2GOAL.decorated.tag_id = {walk_to_pick_TAG2GOAL.decorated.tag_id}")
            walk_to_pick_TAG2GOAL.decorated.tag_id = current_tag_id  # type: ignore
            print(f"更新后 walk_to_pick_TAG2GOAL.decorated.tag_id = {walk_to_pick_TAG2GOAL.decorated.tag_id}")
        
        # 更新 PERCEP 的 tag_ids
        if hasattr(PERCEP, 'tag_ids') and len(PERCEP.tag_ids) > 0:
            PERCEP.tag_ids[0] = current_tag_id
        
        # NodeWaitForBlackboard 节点的 key 在初始化时确定，无法运行时修改，需要移除旧节点，创建新节点
        old_condition = None
        # 在 search_pick_tag_HEAD_AND_WAIT 的 children 中查找 NodeWaitForBlackboard 节点
        for child in search_pick_tag_HEAD_AND_WAIT.children:
            if isinstance(child, NodeWaitForBlackboard):
                old_condition = child
                break
        # 如果找到了旧节点，先移除它
        if old_condition is not None:
            search_pick_tag_HEAD_AND_WAIT.remove_child(old_condition)
        # 创建新的 NodeWaitForBlackboard 节点，使用新的 tag_id
        search_pick_tag_HEAD_AND_WAIT.add_child(NodeWaitForBlackboard(key=f"latest_tag_{current_tag_id}"))
        
        return True
    
    return update_round_and_tag_id
