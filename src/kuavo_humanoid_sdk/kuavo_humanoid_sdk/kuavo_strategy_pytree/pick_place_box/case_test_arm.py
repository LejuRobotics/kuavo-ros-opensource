import os
import sys
import time

import numpy as np
import py_trees

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import NodeArm, NodeFuntion, NodeTagToArmGoal
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import ArmAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Frame, Tag
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.funcs import arm_generate_pick_keypoints
from kuavo_humanoid_sdk.kuavo_strategy_pytree.configs.config_sim import config

# === Demo 配置 ===
TAG_ID = config.pick.tag_id  # tag ID
# 虚假的 tag 位置（在 ODOM 坐标系下）
FAKE_TAG_POS = (0.50, 0.0, 0.75)  # x, y, z (米)
FAKE_TAG_EULER = (90, 0, -90)  # roll, pitch, yaw (度)
control_type = 'joint'   # eef_world, eef_base, joint


def make_tree(robot_sdk, arm_api):
    # 1. 设置虚假 tag 到 blackboard
    def set_fake_tag():
        fake_tag = Tag(
            id=TAG_ID,
            pose=Pose.from_euler(
                pos=FAKE_TAG_POS,
                euler=FAKE_TAG_EULER,
                frame=Frame.ODOM,
                degrees=True
            )
        )
        
        bb = py_trees.blackboard.Client(name="set_fake_tag")
        bb.register_key(f"latest_tag_{TAG_ID}", py_trees.common.Access.WRITE)
        bb.register_key(f"latest_tag_{TAG_ID}_version", py_trees.common.Access.WRITE)
        setattr(bb, f"latest_tag_{TAG_ID}", fake_tag)
        setattr(bb, f"latest_tag_{TAG_ID}_version", 1)
        
        print(f"✅ 设置虚假 tag {TAG_ID} 位置: {FAKE_TAG_POS}")
        return True
    
    set_fake_tag_node = NodeFuntion(
        name="set_fake_tag",
        fn=set_fake_tag
    )
    
    # 2. 生成手臂关键点（相对于 tag）
    left_arm_relative_keypoints, right_arm_relative_keypoints = arm_generate_pick_keypoints(
        box_width=config.common.box_width,
        box_behind_tag=config.pick.box_behind_tag,
        box_beneath_tag=config.pick.box_beneath_tag,
        box_left_tag=config.pick.box_left_tag,
    )
    
    # 3. 根据 tag 计算手臂目标轨迹
    tag_to_arm_goal = NodeTagToArmGoal(
        name='tag_to_arm_goal',
        arm_api=arm_api,
        tag_id=TAG_ID,
        control_type=control_type,
        left_arm_relative_keypoints=left_arm_relative_keypoints,
        right_arm_relative_keypoints=right_arm_relative_keypoints,
        enable_joint_mirroring=True,
    )
    
    # 4. 执行手臂运动
    move_arm = NodeArm(
        name="move_arm_demo",
        arm_api=arm_api,
        control_type=control_type,
        direct_to_wbc=False,
        total_time=3.0,     # 设置手臂执行时间
        back_default=False,
    )
    
    # 5. 构建行为树
    root = py_trees.composites.Sequence(name="arm_demo", memory=True)
    root.add_children([set_fake_tag_node, tag_to_arm_goal, move_arm])
    
    return root


def arm_reset(robot_sdk):
    """手臂归位"""
    print("🔄 手臂归位中...")
    result = robot_sdk.control.arm_reset()
    if result:
        print("✅ 手臂归位成功")
    else:
        print("❌ 手臂归位失败")
    return result


if __name__ == '__main__':
    robot_sdk = RobotSDK()
    arm_api = ArmAPI(robot_sdk=robot_sdk)
    
    root = make_tree(robot_sdk, arm_api)
    tree = py_trees.trees.BehaviourTree(root)
    
    print("开始手臂控制...")
    while True:
        tree.tick()
        status = root.status
        print(py_trees.display.unicode_tree(root, show_status=True))
        
        if status == py_trees.common.Status.SUCCESS:
            print("✅ 手臂运动完成!")
            break
        if status == py_trees.common.Status.FAILURE:
            print("❌ 手臂运动失败.")
            break
        
        time.sleep(0.1)
    
    time.sleep(2)
    # 手臂节点执行完后进行手臂归位
    arm_reset(robot_sdk)
