from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import ArmAPI, TorsoAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Tag, Frame
from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcControlFlow
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


def arm_generate_pick_keypoints(
        box_width: float,
        tag_dx: float,  # 箱子在tag坐标系x方向偏移的距离，单位米
        tag_dy: float,  # 箱子在tag坐标系y方向偏移的距离，单位米
        tag_dz: float,  # 箱子在tag坐标系z方向偏移的距离，单位米
        hand_pitch_degree: float = 0.0,  # 手臂pitch角度（相比水平, 下倾是正），单位度
        step_back_distance: float = 0.0,  # 步行后退距离，单位米
):

    pick_left_arm_poses_new = [
        # # 1. 预抓取点位
        Pose.from_euler(pos=(tag_dx, tag_dy, -box_width * 2.5 / 2 + tag_dz), euler=(-90, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(tag_dx, tag_dy, -box_width / 2 + tag_dz), euler=(-90, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
         # 3. 抬升点位
        Pose.from_euler(pos=(tag_dx, tag_dy + 0.3, -box_width / 2 + tag_dz), euler=(-90, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 4. 收臂点位
        Pose.from_euler(pos=(0.5 + step_back_distance, box_width / 2, 0.4), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                        frame=Frame.BASE)
        ]

    pick_right_arm_poses_new = [
        Pose.from_euler(pos=(tag_dx, tag_dy, box_width * 2.5 / 2 + tag_dz), euler=(-90, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(tag_dx, tag_dy, box_width / 2 + tag_dz), euler=(-90, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 3. 抬升点位
        Pose.from_euler(pos=(tag_dx, tag_dy + 0.3, box_width / 2 + tag_dz), euler=(-90, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 4. 收臂点位
        Pose.from_euler(pos=(0.5 + step_back_distance, -box_width / 2, 0.4), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                        frame=Frame.BASE),
        ]

    return pick_left_arm_poses_new, pick_right_arm_poses_new


def arm_generate_place_keypoints(
        box_width: float,
        tag_dx: float,  # 箱子在tag坐标系x方向偏移的距离，单位米
        tag_dy: float,  # 箱子在tag坐标系y方向偏移的距离，单位米
        tag_dz: float,  # 箱子在tag坐标系z方向偏移的距离，单位米
        step_back_distance: float = 0.0
):

    place_left_arm_poses_new = [
        # 1. 上方点位
        Pose.from_euler(pos=(-tag_dx, tag_dy + 0.1, box_width / 2 + tag_dz), euler=(90, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 2. 并拢点位
    Pose.from_euler(pos=(-tag_dx, tag_dy, box_width / 2 + tag_dz), euler=(90, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
    # 3. 打开点位
    Pose.from_euler(pos=(-tag_dx, tag_dy, box_width * 3 / 2 + 0.15 + tag_dz),
                        euler=(90, 0, 90), degrees=True,
                        frame=Frame.TAG),
    # 4. 收臂点位
    Pose.from_euler(pos=(0.4 + step_back_distance, 0.4, 0.1), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
    ]
    place_right_arm_poses_new = [
    # 1. 上方点位
        Pose.from_euler(pos=(-tag_dx, tag_dy + 0.1, -box_width / 2 + tag_dz), euler=(90, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
    # 2. 并拢点位
        Pose.from_euler(pos=(-tag_dx, tag_dy, -box_width / 2 + tag_dz), euler=(90, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
    # 3. 打开点位
        Pose.from_euler(pos=(-tag_dx, tag_dy, -box_width * 3 / 2 - 0.15 + tag_dz),
                        euler=(90, 0, 90), degrees=True,
                        frame=Frame.TAG),
    # 4. 收臂点位
        Pose.from_euler(pos=(0.4 + step_back_distance, -0.4, 0.1), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
    ] # 手臂关键点数据，假设为空列表

    return place_left_arm_poses_new, place_right_arm_poses_new

def switch__mpc_control_flow(flag: bool):
    if flag:
        robot_sdk.control.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.ThroughFullBodyMpc)
    return True

def move_head(flag: bool, yaw: float, pitch: float):
    if flag:
        robot_sdk.control.control_head(np.deg2rad(yaw), np.deg2rad(pitch))
        time.sleep(0.5)
    return True

def arm_reset():
    robot_sdk.control.arm_reset()
    time.sleep(1.5)

def move_arm_to_zero():
    current_joint_positions = robot_sdk.state.arm_joint_state().position  # 手臂关节位置
    target_joint_positions = [0.0] * 14  # 目标零位
    print(f"当前手臂关节位置：{current_joint_positions}")
    # 设置插值步数，控制运动速度
    interpolation_steps = 30
    # robot_sdk.control.stance()
    robot_sdk.control.control_arm_joint_positions(
            joint_positions=current_joint_positions
        )
    for i in range(interpolation_steps + 1):
        # 线性插值计算中间位置
        alpha = i / interpolation_steps
        interpolated_positions = [
            current_pos + alpha * (target_pos - current_pos)
            for current_pos, target_pos in zip(current_joint_positions, target_joint_positions)
        ]
        
        # 控制手臂到插值位置
        robot_sdk.control.control_arm_joint_positions(
            joint_positions=interpolated_positions
        )
        
        # 短暂延时确保运动平滑
        time.sleep(0.05)
    return True