from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import ArmAPI, TorsoAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Tag, Frame

import py_trees
from py_trees.common import Access
import numpy as np

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
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag + 0.3, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 4. 收臂点位
        Pose.from_euler(pos=(0.5, box_width / 2, 0.4), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
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
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag + 0.3, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 4. 收臂点位
        Pose.from_euler(pos=(0.5, -box_width / 2, 0.4), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                        frame=Frame.BASE),
    ]

    return pick_left_arm_poses, pick_right_arm_poses


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
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag + 0.03, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 3. 打开点位
        Pose.from_euler(pos=(-box_width * 3 / 2 - 0.2 - box_left_tag, -box_beneath_tag, -box_behind_tag),
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
        Pose.from_euler(pos=(box_width / 2 - box_left_tag - 0.03, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 3. 打开点位
        Pose.from_euler(pos=(box_width * 3 / 2 + 0.2 - box_left_tag, -box_beneath_tag, -box_behind_tag),
                        euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 4. 收臂点位
        Pose.from_euler(pos=(0.4, -0.4, 0.1), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
    ]  # 手臂关键点数据，假设为空列表

    return place_left_arm_poses, place_right_arm_poses
