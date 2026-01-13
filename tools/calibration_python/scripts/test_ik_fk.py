#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IK/FK 测试脚本
测试关键点在 base_link 坐标系下的期望位姿的IK逆解结果，
并使用正运动学验证与期望位姿的误差
只使用IK/FK服务，不加载完整SDK
"""

import os
import sys
import numpy as np
import rospy
import rospkg
import json

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Frame
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose
# 直接使用 kuavo_msgs 包的导入，避免md5sum不匹配
from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam
from kuavo_msgs.srv import twoArmHandPoseCmdSrv, fkSrv
from visualization_msgs.msg import Marker
from function.marker_utils import publish_marker
from function.motion_utils import euler_to_quaternion

# ==================== 服务/对齐参数（与 verify_fk_ik_offline_by_pose.py 对齐） ====================
# offline 脚本的 param2/param3 使用这个服务；如果你想对齐 param2 的结果，必须调用同一个服务
IK_SERVICE_NAME = "/ik/two_arm_hand_pose_cmd_srv"
FK_SERVICE_NAME = "/ik/fk_srv"

# offline 脚本用于 build_dual_arm 的默认关节（用于“只评估单臂”时固定另一只手）
INIT_ARM_POS_DEG = [20, 0, 0, -30, 0, 0, 0,
                    20, 0, 0, -30, 0, 0, 0]

# 是否固定使用 INIT_ARM_POS_DEG 作为 q0（更容易复现 offline 的结果；否则会用“上一帧解”做 q0）
USE_FIXED_INIT_Q0 = True

# 添加 motion_capture_ik 包的路径以导入 Quest3ArmInfoTransformer
current_file_dir = os.path.dirname(os.path.abspath(__file__))
motion_capture_ik_path = None
try:
    rospack = rospkg.RosPack()
    motion_capture_ik_path = rospack.get_path("motion_capture_ik")
    sys.path.insert(0, os.path.join(motion_capture_ik_path, "scripts"))
    from tools.quest3_utils import Quest3ArmInfoTransformer
except Exception as e:
    print(f"Warning: Could not import Quest3ArmInfoTransformer: {e}")
    Quest3ArmInfoTransformer = None

# IK求解参数

# IK_SOLVE_PARAM_1 = {
#     "major_optimality_tol": 9e-3,
#     "major_feasibility_tol": 9e-3,
#     "minor_feasibility_tol": 9e-3,
#     "major_iterations_limit": 20000,
#     "oritation_constraint_tol": 19e-3,
#     "pos_constraint_tol": 9e-3,
#     "pos_cost_weight": 10.0,
#     "constraint_mode": 1,
# }

IK_SOLVE_PARAM_1 = {
    'major_optimality_tol': 1e-3,
    'major_feasibility_tol': 1e-3,
    'minor_feasibility_tol': 3e-3,
    'major_iterations_limit': 10000,
    'oritation_constraint_tol': 0.05,
    'pos_constraint_tol': 1e-3,
    'pos_cost_weight': 10000,
    "constraint_mode": 6,
}

# IK_SOLVE_PARAM_1 = {
#     'major_optimality_tol': 1e-3,
#     'major_feasibility_tol': 1e-3,
#     'minor_feasibility_tol': 3e-3,
#     'major_iterations_limit': 10000,
#     'oritation_constraint_tol': 0.0001,
#     'pos_constraint_tol': 1e-3,
#     'pos_cost_weight': 0.0001,
#     "constraint_mode": 2,
# }


def calculate_elbow_y(target_y: float, is_left: bool) -> float:
    """
    计算手肘在Y方向上的偏置，防止双臂过于贴近身体。
    """
    if abs(target_y) < 0.4:
        return 0.4 if is_left else -0.4
    return target_y + 0.05 if is_left else target_y - 0.05


def call_ik_service(left_pose: KuavoPose, right_pose: KuavoPose,
                    left_elbow_pos_xyz: list, right_elbow_pos_xyz: list,
                    arm_q0: list = None) -> list:
    """
    调用IK服务进行逆解
    
    Returns:
        list: 关节角度列表（14维），失败返回None
    """
    try:
        # 创建服务代理（不需要每次都wait_for_service，因为已经在test_ik_fk中检查过了）
        ik_srv = rospy.ServiceProxy(IK_SERVICE_NAME, twoArmHandPoseCmdSrv)
        
        eef_pose_msg = twoArmHandPoseCmd()
        
        # 设置初始关节角度
        if arm_q0 is None:
            eef_pose_msg.joint_angles_as_q0 = False
            # 重要：即使不使用 q0，也要把 joint_angles 填成长度=7 的零数组（对齐 data_utils.py 行为）
            eef_pose_msg.hand_poses.left_pose.joint_angles = [0.0] * 7
            eef_pose_msg.hand_poses.right_pose.joint_angles = [0.0] * 7
        else:
            eef_pose_msg.joint_angles_as_q0 = True
            eef_pose_msg.hand_poses.left_pose.joint_angles = arm_q0[:7]
            eef_pose_msg.hand_poses.right_pose.joint_angles = arm_q0[7:]
        
        # 使用自定义IK参数
        ik_solve_param = ikSolveParam()
        ik_solve_param.major_optimality_tol = IK_SOLVE_PARAM_1["major_optimality_tol"]
        ik_solve_param.major_feasibility_tol = IK_SOLVE_PARAM_1["major_feasibility_tol"]
        ik_solve_param.minor_feasibility_tol = IK_SOLVE_PARAM_1["minor_feasibility_tol"]
        ik_solve_param.major_iterations_limit = IK_SOLVE_PARAM_1["major_iterations_limit"]
        ik_solve_param.oritation_constraint_tol = IK_SOLVE_PARAM_1["oritation_constraint_tol"]
        ik_solve_param.pos_constraint_tol = IK_SOLVE_PARAM_1["pos_constraint_tol"]
        ik_solve_param.pos_cost_weight = IK_SOLVE_PARAM_1["pos_cost_weight"]
        ik_solve_param.constraint_mode = IK_SOLVE_PARAM_1["constraint_mode"]
        eef_pose_msg.ik_param = ik_solve_param
        # 必须开启，否则 IK 节点会忽略 ik_param（包括 constraint_mode）
        eef_pose_msg.use_custom_ik_param = True
        
        # 设置左手臂位姿
        eef_pose_msg.hand_poses.left_pose.pos_xyz = left_pose.position
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = left_pose.orientation
        # 肘约束：传全0表示不启用（与 data_utils.py / offline 脚本一致）
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = left_elbow_pos_xyz
        
        # 设置右手臂位姿
        eef_pose_msg.hand_poses.right_pose.pos_xyz = right_pose.position
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = right_pose.orientation
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = right_elbow_pos_xyz
        
        # 调用服务
        res = ik_srv(eef_pose_msg)
        
        if res.success:
            # multi_refer 服务会在 error_reason 里说明使用了哪个参考解（pinv/midpoint/user_q0/default_q0/last_valid）
            try:
                if hasattr(res, "error_reason") and res.error_reason:
                    print(f"  [IK] {res.error_reason}")
            except Exception:
                pass
            return res.hand_poses.left_pose.joint_angles + res.hand_poses.right_pose.joint_angles
        else:
            return None
            
    except rospy.ServiceException as e:
        print(f"  ❌ IK服务调用失败: {e}")
        return None
    except Exception as e:
        print(f"  ❌ IK服务调用异常: {e}")
        return None


def call_fk_service(q: list) -> tuple:
    """
    调用FK服务进行正运动学计算
    
    Returns:
        tuple: (left_pose, right_pose) 都是 KuavoPose，失败返回 (None, None)
    """
    try:
        # 创建服务代理（不需要每次都wait_for_service，因为已经在test_ik_fk中检查过了）
        fk_srv = rospy.ServiceProxy(FK_SERVICE_NAME, fkSrv)
        
        res = fk_srv(q)
        
        if res.success:
            left_pose = KuavoPose(
                position=res.hand_poses.left_pose.pos_xyz,
                orientation=res.hand_poses.left_pose.quat_xyzw
            )
            right_pose = KuavoPose(
                position=res.hand_poses.right_pose.pos_xyz,
                orientation=res.hand_poses.right_pose.quat_xyzw
            )
            return left_pose, right_pose
        else:
            return None, None
            
    except rospy.ServiceException as e:
        print(f"  ❌ FK服务调用失败: {e}")
        return None, None
    except Exception as e:
        print(f"  ❌ FK服务调用异常: {e}")
        return None, None


def test_ik_fk():
    """测试IK逆解和正运动学验证"""
    
    # 初始化ROS节点
    print("=" * 80)
    print("初始化ROS节点...")
    try:
        rospy.init_node('test_ik_fk', anonymous=True)
        print("✅ ROS节点初始化完成\n")
    except rospy.ROSException as e:
        if "already been initialized" in str(e):
            print("⚠️ ROS节点已初始化，继续使用现有节点\n")
        else:
            print(f"❌ ROS节点初始化失败: {e}")
            return
    
    # 检查服务是否可用
    print("检查IK/FK服务可用性...")
    try:
        rospy.wait_for_service(IK_SERVICE_NAME, timeout=5.0)
        print(f"✅ IK服务可用: {IK_SERVICE_NAME}")
    except rospy.ROSException:
        print(f"❌ IK服务不可用: {IK_SERVICE_NAME}")
        print("   请确保IK服务节点正在运行")
        return
    
    try:
        rospy.wait_for_service(FK_SERVICE_NAME, timeout=5.0)
        print(f"✅ FK服务可用: {FK_SERVICE_NAME}")
    except rospy.ROSException:
        print(f"❌ FK服务不可用: {FK_SERVICE_NAME}")
        print("   请确保IK服务节点正在运行")
        return
    print()
    
    # 初始化可视化marker发布器
    quest3_arm_info_transformer = None
    marker_pub_cmd_left = None
    marker_pub_cmd_right = None
    marker_pub_res_left = None
    marker_pub_res_right = None
    # 保存最后一个marker，用于持续发布
    last_markers = {
        'left_target': None,
        'right_target': None,
        'left_result': None,
        'right_result': None
    }
    
    if Quest3ArmInfoTransformer is not None:
        try:
            # 获取模型路径和配置
            rospack = rospkg.RosPack()
            kuavo_assets_path = rospack.get_path("kuavo_assets")
            robot_version = os.environ.get('ROBOT_VERSION', '40')
            if rospy.has_param("robot_version"):
                robot_version = rospy.get_param("robot_version")
            
            model_path = kuavo_assets_path + "/models/biped_s" + str(robot_version)
            model_config_file = kuavo_assets_path + f"/config/kuavo_v{robot_version}/kuavo.json"
            
            with open(model_config_file, 'r') as f:
                model_config = json.load(f)
            eef_visual_stl_files = model_config["eef_visual_stl_files"]
            
            quest3_arm_info_transformer = Quest3ArmInfoTransformer(model_path, eef_visual_stl_files=eef_visual_stl_files)
            
            # 创建marker发布器
            marker_pub_cmd_left = rospy.Publisher("/test_ik_fk_target_marker_left", Marker, queue_size=10)
            marker_pub_cmd_right = rospy.Publisher("/test_ik_fk_target_marker_right", Marker, queue_size=10)
            marker_pub_res_left = rospy.Publisher("/test_ik_fk_result_marker_left", Marker, queue_size=10)
            marker_pub_res_right = rospy.Publisher("/test_ik_fk_result_marker_right", Marker, queue_size=10)
            
            print("✅ Marker发布器初始化完成")
            print(f"   - 期望位姿话题: /test_ik_fk_target_marker_left, /test_ik_fk_target_marker_right")
            print(f"   - 实际位姿话题: /test_ik_fk_result_marker_left, /test_ik_fk_result_marker_right")
            rospy.sleep(0.5)  # 等待发布器注册
        except Exception as e:
            print(f"⚠️ Marker发布器初始化失败: {e}")
            print("   继续测试，但不显示可视化marker")
    else:
        print("⚠️ Quest3ArmInfoTransformer不可用，跳过marker初始化")
    print()
    
    # 定义关键点（在 base_link/BASE 坐标系下）
    # 根据终端输出的关键点位姿定义
    # 重要：这里的欧拉角转换必须和 verify_fk_ik_offline_by_pose.py 保持一致。
    # verify 脚本使用 function.motion_utils.euler_to_quaternion（ZYX 顺序），
    # 而 SDK 的 Pose.from_euler 使用 scipy 的 'xyz'，两者不一致会导致“看起来同样的欧拉角”实际对应不同四元数，
    # 从而出现旋转误差 170~180° 这种现象。
    keypoints = [
        {
            'name': '关键点 1',
            'left_pose': Pose(
                pos=(0.2999, 0.5600, 0.1002),
                quat=euler_to_quaternion(roll_deg=0.0, pitch_deg=-89.96, yaw_deg=0.0),
                frame=Frame.BASE,
            ),
            'right_pose': Pose(
                pos=(0.2999, -0.5600, 0.1002),
                quat=euler_to_quaternion(roll_deg=0.0, pitch_deg=-89.96, yaw_deg=0.0),
                frame=Frame.BASE,
            )
        },
        {
            'name': '关键点 2',
            'left_pose': Pose(
                pos=(0.4998, 0.4200, 0.2003),
                quat=euler_to_quaternion(roll_deg=0.0, pitch_deg=-89.97, yaw_deg=0.0),
                frame=Frame.BASE,
            ),
            'right_pose': Pose(
                pos=(0.4998, -0.4200, 0.2003),
                quat=euler_to_quaternion(roll_deg=0.0, pitch_deg=-89.97, yaw_deg=0.0),
                frame=Frame.BASE,
            )
        },
        {
            'name': '关键点 3',
            'left_pose': Pose(
                pos=(0.4998, 0.1400, 0.2001),
                quat=euler_to_quaternion(roll_deg=0.0, pitch_deg=-89.98, yaw_deg=0.0),
                frame=Frame.BASE,
            ),
            'right_pose': Pose(
                pos=(0.4998, -0.1400, 0.2001),
                quat=euler_to_quaternion(roll_deg=0.0, pitch_deg=-89.98, yaw_deg=0.0),
                frame=Frame.BASE,
            )
        },
        {
            'name': '关键点 4',
            'left_pose': Pose(
                pos=(0.359, 0.1400, 0.5500),
                quat=euler_to_quaternion(roll_deg=0.0, pitch_deg=-89.99, yaw_deg=0.0),
                frame=Frame.BASE,
            ),
            'right_pose': Pose(
                pos=(0.359, -0.1400, 0.5500),
                quat=euler_to_quaternion(roll_deg=0.0, pitch_deg=-89.99, yaw_deg=0.0),
                frame=Frame.BASE,
            )
        }
    ]
    # 每两个关键点之间的暂停时间（秒），便于观察/记录
    KEYPOINT_DELAY = 2.0

    # 初始关节角度（设为None，让IK服务使用默认值）
    if USE_FIXED_INIT_Q0:
        current_joints = np.radians(np.array(INIT_ARM_POS_DEG, dtype=float))
        print("初始关节角度: 固定使用 INIT_ARM_POS_DEG 作为 q0（更容易对齐 offline 结果）\n")
    else:
        current_joints = None
        print("初始关节角度: 使用IK服务默认值/上一帧解\n")

    print("=" * 80)
    print("开始IK/FK测试\n")
    
    # 对每个关键点进行测试
    for idx, kp in enumerate(keypoints):
        print(f"\n{'=' * 80}")
        print(f"{kp['name']} (base_link坐标系)")
        print(f"{'=' * 80}")
        
        left_pose_base = kp['left_pose']
        right_pose_base = kp['right_pose']
        
        # 打印期望位姿
        left_euler = left_pose_base.get_euler(degrees=True)
        right_euler = right_pose_base.get_euler(degrees=True)
        
        print(f"\n期望位姿:")
        print(f"  左臂:")
        print(f"    位置: [{left_pose_base.pos[0]:.4f}, {left_pose_base.pos[1]:.4f}, {left_pose_base.pos[2]:.4f}]")
        print(f"    姿态(欧拉角): roll={left_euler[0]:.2f}°, pitch={left_euler[1]:.2f}°, yaw={left_euler[2]:.2f}°")
        print(f"  右臂:")
        print(f"    位置: [{right_pose_base.pos[0]:.4f}, {right_pose_base.pos[1]:.4f}, {right_pose_base.pos[2]:.4f}]")
        print(f"    姿态(欧拉角): roll={right_euler[0]:.2f}°, pitch={right_euler[1]:.2f}°, yaw={right_euler[2]:.2f}°")
        
        # 转换为 KuavoPose
        left_target_kuavo_pose = KuavoPose(
            position=list(left_pose_base.pos),
            orientation=list(left_pose_base.quat)
        )
        right_target_kuavo_pose = KuavoPose(
            position=list(right_pose_base.pos),
            orientation=list(right_pose_base.quat)
        )
        
        # 计算肘关节约束（暂时不用，注释掉）
        left_elbow_y = calculate_elbow_y(left_target_kuavo_pose.position[1], True)
        right_elbow_y = calculate_elbow_y(right_target_kuavo_pose.position[1], False)
        
        # 简化的肘关节位置（不使用get_link_position）
        x_offset = 0.05
        z_offset = 0.0
        left_elbow = [x_offset, 0.3, z_offset]   # 左肘默认y=0.3
        right_elbow = [x_offset, -0.3, z_offset] # 右肘默认y=-0.3
        
        # 如果计算出的elbow_y与默认值不同，使用计算值（暂时关闭）
        if abs(left_elbow_y - 0.3) > 0.01:
            left_elbow[1] = left_elbow_y
        if abs(right_elbow_y - (-0.3)) > 0.01:
            right_elbow[1] = right_elbow_y
        
        # print(f"\n肘关节约束:")
        # print(f"  左肘: {left_elbow}")
        # print(f"  右肘: {right_elbow}")
        
        # IK逆解
        print(f"\n进行IK逆解...")
        target_joint_positions = None
        for retry in range(5):
            target_joint_positions = call_ik_service(
                left_pose=left_target_kuavo_pose,
                right_pose=right_target_kuavo_pose,
                left_elbow_pos_xyz=[0.0, 0.0, 0.0],
                right_elbow_pos_xyz=[0.0, 0.0, 0.0],
                arm_q0=current_joints.tolist() if current_joints is not None else None
            )
            
            if target_joint_positions is not None:
                print(f"  ✅ IK逆解成功 (重试次数: {retry})")
                break
        
        if target_joint_positions is None:
            print(f"  ❌ IK逆解失败")

            # 发布期望位姿（红色）以及失败占位（蓝色，原点）到结果话题
            if quest3_arm_info_transformer is not None:
                try:
                    # 期望位姿 marker（红色）——目标话题
                    if marker_pub_cmd_left is not None:
                        publish_marker(
                            quest3_arm_info_transformer,
                            marker_pub_cmd_left,
                            left_pose_base.pos,
                            left_pose_base.quat,
                            "left",
                            rgba=[1.0, 0.0, 0.0, 0.9],
                            marker_id=0,
                            namespace="test_ik_fk_target",
                        )
                    if marker_pub_cmd_right is not None:
                        publish_marker(
                            quest3_arm_info_transformer,
                            marker_pub_cmd_right,
                            right_pose_base.pos,
                            right_pose_base.quat,
                            "right",
                            rgba=[1.0, 0.0, 0.0, 0.9],
                            marker_id=0,
                            namespace="test_ik_fk_target",
                        )

                    # 失败占位 marker（蓝色，原点，单位四元数）——结果话题
                    failed_pos = [0.0, 0.0, 0.0]
                    failed_quat = [0.0, 0.0, 0.0, 1.0]
                    if marker_pub_res_left is not None:
                        publish_marker(
                            quest3_arm_info_transformer,
                            marker_pub_res_left,
                            failed_pos,
                            failed_quat,
                            "left",
                            rgba=[0.0, 0.0, 1.0, 0.9],
                            marker_id=0,
                            namespace="test_ik_fk_failed",
                        )
                    if marker_pub_res_right is not None:
                        publish_marker(
                            quest3_arm_info_transformer,
                            marker_pub_res_right,
                            failed_pos,
                            failed_quat,
                            "right",
                            rgba=[0.0, 0.0, 1.0, 0.9],
                            marker_id=0,
                            namespace="test_ik_fk_failed",
                        )
                except Exception as e:
                    print(f"  ⚠️ 发布失败marker时出错: {e}")

            continue
        
        target_joint_positions = np.array(target_joint_positions, dtype=float)
        print(f"  关节角度 (度): {np.rad2deg(target_joint_positions)}")
        
        # 正运动学验证
        print(f"\n进行正运动学验证...")
        try:
            left_fk_pose, right_fk_pose = call_fk_service(target_joint_positions)
            
            if left_fk_pose is None or right_fk_pose is None:
                print(f"  ❌ 正运动学计算失败")
                continue
            
            # 转换为 Pose 对象
            left_fk_pose_base = Pose(
                pos=left_fk_pose.position,
                quat=left_fk_pose.orientation,
                frame=Frame.BASE
            )
            right_fk_pose_base = Pose(
                pos=right_fk_pose.position,
                quat=right_fk_pose.orientation,
                frame=Frame.BASE
            )
            
            # 计算误差
            left_pos_error = left_pose_base.position_l2_norm(left_fk_pose_base)
            right_pos_error = right_pose_base.position_l2_norm(right_fk_pose_base)
            left_rot_error = left_pose_base.angle(left_fk_pose_base)
            right_rot_error = right_pose_base.angle(right_fk_pose_base)

            # ==================== offline式误差（对齐 verify_fk_ik_offline_by_pose.py 的统计口径） ====================
            # offline 脚本对每只手分别做 IK，然后 FK 时把“另一只手”的关节固定在 INIT_ARM_POS_DEG，
            # 因此它的 left/right 误差并不是“同一套14维关节同时满足双臂目标”的误差。
            init_left = np.radians(np.array(INIT_ARM_POS_DEG[:7], dtype=float))
            init_right = np.radians(np.array(INIT_ARM_POS_DEG[7:], dtype=float))

            q_left_off = np.concatenate([target_joint_positions[:7], init_right])
            q_right_off = np.concatenate([init_left, target_joint_positions[7:]])

            left_fk_off, _ = call_fk_service(q_left_off)
            _, right_fk_off = call_fk_service(q_right_off)

            left_rot_error_off_deg = None
            right_rot_error_off_deg = None
            if left_fk_off is not None:
                left_fk_off_pose = Pose(pos=left_fk_off.position, quat=left_fk_off.orientation, frame=Frame.BASE)
                left_rot_error_off_deg = np.rad2deg(left_pose_base.angle(left_fk_off_pose))
            if right_fk_off is not None:
                right_fk_off_pose = Pose(pos=right_fk_off.position, quat=right_fk_off.orientation, frame=Frame.BASE)
                right_rot_error_off_deg = np.rad2deg(right_pose_base.angle(right_fk_off_pose))
            
            # 打印实际位姿
            left_fk_euler = left_fk_pose_base.get_euler(degrees=True)
            right_fk_euler = right_fk_pose_base.get_euler(degrees=True)
            
            print(f"\n实际位姿 (正运动学结果):")
            print(f"  左臂:")
            print(f"    位置: [{left_fk_pose_base.pos[0]:.4f}, {left_fk_pose_base.pos[1]:.4f}, {left_fk_pose_base.pos[2]:.4f}]")
            print(f"    姿态(欧拉角): roll={left_fk_euler[0]:.2f}°, pitch={left_fk_euler[1]:.2f}°, yaw={left_fk_euler[2]:.2f}°")
            print(f"  右臂:")
            print(f"    位置: [{right_fk_pose_base.pos[0]:.4f}, {right_fk_pose_base.pos[1]:.4f}, {right_fk_pose_base.pos[2]:.4f}]")
            print(f"    姿态(欧拉角): roll={right_fk_euler[0]:.2f}°, pitch={right_fk_euler[1]:.2f}°, yaw={right_fk_euler[2]:.2f}°")
            
            # 打印误差分析
            print(f"\n误差分析:")
            print(f"  左臂:")
            print(f"    位置误差: {left_pos_error*1000:.2f} mm")
            print(f"    旋转误差: {np.rad2deg(left_rot_error):.2f}°")
            print(f"  右臂:")
            print(f"    位置误差: {right_pos_error*1000:.2f} mm")
            print(f"    旋转误差: {np.rad2deg(right_rot_error):.2f}°")

            if left_rot_error_off_deg is not None or right_rot_error_off_deg is not None:
                print(f"\n[对齐offline统计口径]（另一只手固定为 INIT_ARM_POS_DEG）:")
                if left_rot_error_off_deg is not None:
                    print(f"  左臂旋转误差(offline口径): {left_rot_error_off_deg:.2f}°")
                if right_rot_error_off_deg is not None:
                    print(f"  右臂旋转误差(offline口径): {right_rot_error_off_deg:.2f}°")
            
            # 判断是否满足精度要求
            pos_threshold_mm = 10.0  # 10mm
            rot_threshold_deg = 5.0  # 5度
            
            left_pos_ok = left_pos_error * 1000 < pos_threshold_mm
            left_rot_ok = np.rad2deg(left_rot_error) < rot_threshold_deg
            right_pos_ok = right_pos_error * 1000 < pos_threshold_mm
            right_rot_ok = np.rad2deg(right_rot_error) < rot_threshold_deg
            
            print(f"\n精度评估 (阈值: 位置<{pos_threshold_mm}mm, 旋转<{rot_threshold_deg}°):")
            print(f"  左臂: 位置{'✅' if left_pos_ok else '❌'}, 旋转{'✅' if left_rot_ok else '❌'}")
            print(f"  右臂: 位置{'✅' if right_pos_ok else '❌'}, 旋转{'✅' if right_rot_ok else '❌'}")
            
            if left_pos_ok and left_rot_ok and right_pos_ok and right_rot_ok:
                print(f"  ✅ {kp['name']} 通过精度测试")
            else:
                print(f"  ⚠️ {kp['name']} 未完全通过精度测试")
            
            # 发布可视化marker
            if quest3_arm_info_transformer is not None and marker_pub_cmd_left is not None:
                try:
                    # 发布期望位姿（红色，目标）
                    marker_left_target = quest3_arm_info_transformer.construct_marker(
                        left_pose_base.pos, 
                        left_pose_base.quat, 
                        rgba=[1, 0, 0, 0.9], 
                        side="Left", 
                        marker_id=0
                    )
                    marker_right_target = quest3_arm_info_transformer.construct_marker(
                        right_pose_base.pos, 
                        right_pose_base.quat, 
                        rgba=[1, 0, 0, 0.9], 
                        side="Right", 
                        marker_id=0
                    )
                    # 设置lifetime为永不过期（或者设置一个较长的时间）
                    if marker_left_target is not None:
                        # 注意：Quest3ArmInfoTransformer.construct_marker 默认 frame_id=base_link。
                        # 由于不存在 waist_yaw_link，这里显式统一为 base_link，确保 RViz 可显示。
                        marker_left_target.header.frame_id = "base_link"
                        marker_left_target.ns = "test_ik_fk_target"
                        marker_left_target.id = 0
                        marker_left_target.lifetime = rospy.Duration()  # 0表示永不过期
                        marker_pub_cmd_left.publish(marker_left_target)
                    if marker_right_target is not None:
                        marker_right_target.header.frame_id = "base_link"
                        marker_right_target.ns = "test_ik_fk_target"
                        marker_right_target.id = 1
                        marker_right_target.lifetime = rospy.Duration()  # 0表示永不过期
                        marker_pub_cmd_right.publish(marker_right_target)
                    
                    # 发布实际位姿（蓝色，结果）
                    marker_left_result = quest3_arm_info_transformer.construct_marker(
                        left_fk_pose.position, 
                        left_fk_pose.orientation, 
                        rgba=[0, 0, 1, 0.9], 
                        side="Left", 
                        marker_id=0
                    )
                    marker_right_result = quest3_arm_info_transformer.construct_marker(
                        right_fk_pose.position, 
                        right_fk_pose.orientation, 
                        rgba=[0, 0, 1, 0.9], 
                        side="Right", 
                        marker_id=0
                    )
                    if marker_left_result is not None:
                        marker_left_result.header.frame_id = "base_link"
                        marker_left_result.ns = "test_ik_fk_result"
                        marker_left_result.id = 2
                        marker_left_result.lifetime = rospy.Duration()  # 0表示永不过期
                        marker_pub_res_left.publish(marker_left_result)
                    if marker_right_result is not None:
                        marker_right_result.header.frame_id = "base_link"
                        marker_right_result.ns = "test_ik_fk_result"
                        marker_right_result.id = 3
                        marker_right_result.lifetime = rospy.Duration()  # 0表示永不过期
                        marker_pub_res_right.publish(marker_right_result)
                    
                    # 连续发布几次确保rviz能接收到，每次更新时间戳
                    for i in range(5):
                        if marker_left_target is not None:
                            marker_left_target.header.stamp = rospy.Time.now()
                            marker_pub_cmd_left.publish(marker_left_target)
                        if marker_right_target is not None:
                            marker_right_target.header.stamp = rospy.Time.now()
                            marker_pub_cmd_right.publish(marker_right_target)
                        if marker_left_result is not None:
                            marker_left_result.header.stamp = rospy.Time.now()
                            marker_pub_res_left.publish(marker_left_result)
                        if marker_right_result is not None:
                            marker_right_result.header.stamp = rospy.Time.now()
                            marker_pub_res_right.publish(marker_right_result)
                        rospy.sleep(0.02)
                    print(f"  ✅ Marker已发布到rviz话题 (每个关键点发布5次)")
                    # 保存最后一个marker用于持续发布
                    last_markers['left_target'] = marker_left_target
                    last_markers['right_target'] = marker_right_target
                    last_markers['left_result'] = marker_left_result
                    last_markers['right_result'] = marker_right_result
                except Exception as e:
                    print(f"  ⚠️ 发布marker时出错: {e}")
                    import traceback
                    traceback.print_exc()
            
        except Exception as e:
            print(f"  ❌ 正运动学验证时出错: {e}")
            import traceback
            traceback.print_exc()
        
        # 更新当前关节角度用于下一个关键点
        current_joints = target_joint_positions

        # 在关键点之间插入延时，便于观察
        if KEYPOINT_DELAY > 0.0:
            rospy.sleep(KEYPOINT_DELAY)
    
    print(f"\n{'=' * 80}")
    print("IK/FK测试完成")
    print(f"{'=' * 80}")
    
    # 持续发布marker一段时间，确保rviz能接收到并保持话题可见
    if quest3_arm_info_transformer is not None and marker_pub_cmd_left is not None:
        print("\n保持marker发布30秒，以便在rviz中查看...")
        print("  提示：在rviz中添加Marker显示，订阅以下话题：")
        print("    - /test_ik_fk_target_marker_left (红色，期望位姿-左手)")
        print("    - /test_ik_fk_target_marker_right (红色，期望位姿-右手)")
        print("    - /test_ik_fk_result_marker_left (蓝色，实际位姿-左手)")
        print("    - /test_ik_fk_result_marker_right (蓝色，实际位姿-右手)")
        print("  (按Ctrl+C提前退出)")
        
        rate = rospy.Rate(10)  # 10Hz
        start_time = rospy.Time.now()
        duration = rospy.Duration(30.0)  # 持续30秒
        
        try:
            while not rospy.is_shutdown() and (rospy.Time.now() - start_time) < duration:
                # 重新发布最后一个marker，更新时间戳
                if last_markers['left_target'] is not None:
                    last_markers['left_target'].header.stamp = rospy.Time.now()
                    marker_pub_cmd_left.publish(last_markers['left_target'])
                if last_markers['right_target'] is not None:
                    last_markers['right_target'].header.stamp = rospy.Time.now()
                    marker_pub_cmd_right.publish(last_markers['right_target'])
                if last_markers['left_result'] is not None:
                    last_markers['left_result'].header.stamp = rospy.Time.now()
                    marker_pub_res_left.publish(last_markers['left_result'])
                if last_markers['right_result'] is not None:
                    last_markers['right_result'].header.stamp = rospy.Time.now()
                    marker_pub_res_right.publish(last_markers['right_result'])
                rate.sleep()
            print("  ✅ Marker发布结束")
        except KeyboardInterrupt:
            print("\n  用户中断marker发布")
        except Exception as e:
            print(f"  持续发布marker时出错: {e}")


if __name__ == '__main__':
    try:
        test_ik_fk()
    except KeyboardInterrupt:
        print("\n\n用户中断测试")
    except Exception as e:
        print(f"\n\n测试出错: {e}")
        import traceback
        traceback.print_exc()

