import os
import sys
import time

import numpy as np
import py_trees

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import NodePercep, NodeWaitForBlackboard, NodeFuntion, NodeDelay
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.configs.config_sim import config

# === Demo 配置 ===
TAG_ID = 0  # 要寻找的 tag ID
HEAD_SEARCH_YAWS = [np.deg2rad(85), 0, np.deg2rad(-85)]  # 头部搜索的偏航角度
HEAD_SEARCH_PITCHES = [np.deg2rad(-10), np.deg2rad(0), np.deg2rad(10)]  # 头部搜索的俯仰角度
TIMEOUT_SECONDS = 5.0  # 超时时间（秒）


def make_tree(robot_sdk):
    # 1. 感知节点 - 持续感知 tag（在 Parallel 中运行）
    PERCEP = NodePercep(
        name='PERCEP',
        robot_sdk=robot_sdk,
        tag_ids=[TAG_ID]
    )

    # 2. 头部搜索序列
    HEAD_SEARCH = py_trees.composites.Sequence(name="head_search", memory=True)

    # 创建多个头部控制节点，在不同角度搜索
    head_move_nodes = []
    for yaw in HEAD_SEARCH_YAWS:
        for pitch in HEAD_SEARCH_PITCHES:
            # 使用闭包确保正确捕获变量
            def make_head_control(y, p, rsdk):
                return NodeFuntion(
                    name=f"head_move_yaw_{np.rad2deg(y):.0f}_pitch_{np.rad2deg(p):.0f}",
                    fn=lambda: rsdk.control.control_head(y, p) or True
                )
            head_move_node = make_head_control(yaw, pitch, robot_sdk)
            head_move_nodes.append(head_move_node)
            # 每个头部移动后等待一下，让感知有时间识别
            delay_node = NodeDelay(duration=0.5, name=f"delay_after_head_move")
            head_move_nodes.append(delay_node)

    # 3. 等待找到 tag 的条件节点
    wait_for_tag = NodeWaitForBlackboard(key=f"latest_tag_{TAG_ID}")

    HEAD_SEARCH.add_children(head_move_nodes + [wait_for_tag])

    # 4. 主行为树 - Parallel 结构，同时运行感知和头部搜索
    root = py_trees.composites.Parallel(
        name="root",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    root.add_children([HEAD_SEARCH, PERCEP])

    return root


if __name__ == '__main__':
    # 初始化头部位置（保存为初始位置，用于后续回位）
    INITIAL_HEAD_YAW = 0
    INITIAL_HEAD_PITCH = np.deg2rad(-10)
    
    robot_sdk = RobotSDK()
    robot_sdk.control.control_head(INITIAL_HEAD_YAW, INITIAL_HEAD_PITCH)

    root = make_tree(robot_sdk)
    tree = py_trees.trees.BehaviourTree(root)

    print("开始头部搜索 tag...")
    start_time = time.time()  # 记录开始时间

    while True:
        # 检查超时
        elapsed_time = time.time() - start_time
        if elapsed_time >= TIMEOUT_SECONDS:
            print(f"⏰ 超时退出：在 {TIMEOUT_SECONDS} 秒内未找到 tag {TAG_ID}")
            # 超时时也回到初始位置
            print("🔄 头部回到初始位置...")
            robot_sdk.control.control_head(INITIAL_HEAD_YAW, INITIAL_HEAD_PITCH)
            time.sleep(0.5)  # 等待头部移动完成
            print("✅ 头部已回到初始位置")
            break

        tree.tick()
        status = root.status
        print(py_trees.display.unicode_tree(root, show_status=True))

        if status == py_trees.common.Status.SUCCESS:
            print(f"✅ 成功找到 tag {TAG_ID}!")
            bb_client = py_trees.blackboard.Client(name="tag_printer")
            bb_client.register_key(key=f"latest_tag_{TAG_ID}", access=py_trees.common.Access.READ)
            latest_tag = getattr(bb_client, f"latest_tag_{TAG_ID}", None)
            if latest_tag is not None and hasattr(latest_tag, "pose"):
                pos = latest_tag.pose.pos if hasattr(latest_tag.pose, "pos") else latest_tag.pose
                quat = latest_tag.pose.quat if hasattr(latest_tag.pose, "quat") else None
                print(f"📍 tag {TAG_ID} 位置: {pos}")
                if quat is not None:
                    print(f"   姿态(quat): {quat}")
            else:
                print(f"⚠️ 无法在黑板上读取 latest_tag_{TAG_ID}")
            
            # 找到 tag 后，头部回到初始位置
            print("🔄 头部回到初始位置...")
            robot_sdk.control.control_head(INITIAL_HEAD_YAW, INITIAL_HEAD_PITCH)
            time.sleep(0.5)  # 等待头部移动完成
            print("✅ 头部已回到初始位置")
            break
        if status == py_trees.common.Status.FAILURE:
            print("❌ 头部搜索失败.")
            # 失败时也回到初始位置
            print("🔄 头部回到初始位置...")
            robot_sdk.control.control_head(INITIAL_HEAD_YAW, INITIAL_HEAD_PITCH)
            time.sleep(0.5)  # 等待头部移动完成
            print("✅ 头部已回到初始位置")
            break

        time.sleep(0.1)