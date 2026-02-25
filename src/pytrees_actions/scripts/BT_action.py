#!/usr/bin/env python3
import py_trees
import py_trees.composites
import json
import rospy
import sys
import inspect
import os
import importlib.util
import actionlib
import traceback
from py_trees import common
# 导入自动生成的 Action 消息
from pytrees_actions.msg import GetBtStateAction, GetBtStateFeedback, GetBtStateResult

# 全局变量
BT_REAL_TIME_STATE = {}
BT_ROOT_NODE = None
BT_ACTION_SERVER = None

# 节点类映射
NODE_CLASSES = {
    "Sequence": py_trees.composites.Sequence,
    "Parallel": py_trees.composites.Parallel,
    "Selector": py_trees.composites.Selector,
}

def auto_import_node_dir_modules(node_dir_path):
    global NODE_CLASSES
    rospy.loginfo(f"[自动导入] 扫描节点目录：{node_dir_path}")
    if not os.path.exists(node_dir_path) or not os.path.isdir(node_dir_path):
        rospy.logerr(f"[自动导入失败] 目录不存在：{node_dir_path}")
        return

    for file_name in os.listdir(node_dir_path):
        if not file_name.endswith(".py") or file_name.startswith("_"):
            continue
        file_abs_path = os.path.join(node_dir_path, file_name)
        module_name = os.path.splitext(file_name)[0]

        try:
            spec = importlib.util.spec_from_file_location(module_name, file_abs_path)
            if spec is None:
                rospy.logwarn(f"[自动导入] 无法创建模块：{file_name}")
                continue
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)

            for member_name, member_obj in inspect.getmembers(module):
                if (inspect.isclass(member_obj)
                    and issubclass(member_obj, py_trees.behaviour.Behaviour)
                    and not member_obj.__module__.startswith("py_trees.")
                ):
                    node_class_name = member_obj.__name__
                    NODE_CLASSES[node_class_name] = member_obj
                    rospy.loginfo(f"[自动导入] 注册节点：{node_class_name}")
        except Exception as e:
            rospy.logerr(f"[自动导入失败] 处理 {file_name}：{str(e)}")
            traceback.print_exc()
            continue

def build_behavior_tree(json_data):
    global BT_ROOT_NODE
    def create_node(node_data):
        # 1. 提取JSON中的纯Label和节点类型
        node_name = node_data.get("name", "Unnamed")
        json_label = node_data.get("label", node_name)
        params = node_data.get("params", {})

        # 2. 节点类型校验（增强调试信息）
        if node_name not in NODE_CLASSES:
            rospy.logerr(f"[创建节点失败] 未知类型：{node_name}（Label：{json_label}），已注册类型：{list(NODE_CLASSES.keys())}")
            return None
        node_class = NODE_CLASSES[node_name]

        # 3. 创建节点实例（增强容错处理）
        node = None
        try:
            if node_name in ["Sequence", "Selector"]:
                # 优化memory参数处理（支持布尔值和字符串）
                memory_val = params.get("memory", "True")
                if isinstance(memory_val, bool):
                    memory = memory_val
                else:
                    memory = memory_val.strip().lower() == "true"
                node = node_class(name=json_label, memory=memory)
                rospy.logdebug(f"[创建节点成功] {node_name}（Label：{json_label}），memory：{memory}")

            elif node_name == "Parallel":
                # 核心优化：Parallel策略容错处理
                policy_map = {
                    "SuccessOnAll": common.ParallelPolicy.SuccessOnAll,
                    "SuccessOnOne": common.ParallelPolicy.SuccessOnOne,
                    "SuccessOnSelected": common.ParallelPolicy.SuccessOnSelected
                }
                # 标准化策略字符串（忽略大小写和空格）
                policy_str = str(params.get("policy", "SuccessOnAll")).strip().lower()
                matched_policy_cls = None
                for key, cls in policy_map.items():
                    if key.lower() == policy_str:
                        matched_policy_cls = cls
                        break
                # 策略匹配失败时使用默认值并报警
                if not matched_policy_cls:
                    rospy.logwarn(f"[Parallel策略错误] Label：{json_label}，未知策略：{policy_str}，使用默认策略：SuccessOnAll")
                    matched_policy_cls = common.ParallelPolicy.SuccessOnAll
                # 创建策略实例
                policy = matched_policy_cls()
                node = node_class(name=json_label, policy=policy)
                rospy.logdebug(f"[创建节点成功] Parallel（Label：{json_label}），策略：{matched_policy_cls.__name__}")

            else:  # 叶子节点（Action/Test/Custom）
                node = node_class(name=json_label, label=json_label, params=params if params else {})
                rospy.logdebug(f"[创建节点成功] 叶子节点（Label：{json_label}），类型：{node_name}")

            # 绑定核心属性
            node.json_label = json_label
            node.node_type_raw = node_name

        except Exception as e:
            rospy.logerr(f"[创建节点失败] {node_name}（Label：{json_label}）：{str(e)}")
            traceback.print_exc()
            return None

        # 4. 递归添加子节点（增强错误处理）
        if node_name in ["Sequence", "Parallel", "Selector"]:
            children_data = node_data.get("childs", [])
            # 校验子节点数据类型
            if not isinstance(children_data, list):
                rospy.logerr(f"[子节点格式错误] Label：{json_label}，childs必须是列表，当前类型：{type(children_data).__name__}")
                children_data = []

            added_child_count = 0
            for idx, child_data in enumerate(children_data):
                child_node = create_node(child_data)
                if child_node:
                    node.add_child(child_node)
                    added_child_count += 1
                    rospy.logdebug(f"[添加子节点成功] 父节点：{json_label} -> 子节点{idx+1}：{child_node.json_label}")
                else:
                    rospy.logwarn(f"[添加子节点失败] 父节点：{json_label}，子节点{idx+1}配置无效")

            # 报警：组合节点无子节点
            if added_child_count == 0:
                rospy.logwarn(f"[组合节点无子节点] {node_name}（Label：{json_label}）未添加任何子节点")

        return node

    try:
        # 校验JSON结构
        if "tree" not in json_data:
            rospy.logerr(f"[构建行为树失败] JSON中无'tree'字段，当前结构：{list(json_data.keys())}")
            return None

        BT_ROOT_NODE = create_node(json_data["tree"])
        if BT_ROOT_NODE:
            child_count = len(BT_ROOT_NODE.children) if hasattr(BT_ROOT_NODE, "children") else 0
            rospy.loginfo(f"[构建行为树成功] 根节点：{BT_ROOT_NODE.json_label}（类型：{BT_ROOT_NODE.node_type_raw}），子节点数量：{child_count}")
            # 根节点无子节点报警
            if child_count == 0:
                rospy.logerr(f"[根节点无子节点] 请检查JSON中根节点的childs配置")
        return BT_ROOT_NODE
    except Exception as e:
        rospy.logerr(f"[构建行为树失败] {str(e)}")
        traceback.print_exc()
        raise

def update_bt_state():
    global BT_REAL_TIME_STATE, BT_ROOT_NODE
    if not BT_ROOT_NODE:
        rospy.logwarn("[更新状态失败] 行为树根节点未初始化")
        return

    # 重置状态存储
    BT_REAL_TIME_STATE = {}
    execution_logs = []

    # 递归收集节点信息
    def collect_node_info(node, depth=0):
        # 存储节点状态
        node_status = str(node.status).replace("Status.", "")
        BT_REAL_TIME_STATE[node.json_label] = {
            "label": node.json_label,
            "type": node.node_type_raw,
            "status": node_status,
            "depth": depth,
            "timestamp": rospy.Time.now().to_sec()
        }
        # 收集叶子节点日志
        if node.node_type_raw not in ["Sequence", "Parallel", "Selector"]:
            log_msg = f"{node.json_label} (类型：{node.node_type_raw}) 状态：{node_status}"
            execution_logs.append(log_msg)

        # 递归处理子节点
        if hasattr(node, "children") and node.children:
            for child in node.children:
                collect_node_info(child, depth + 1)

    # 执行收集并打印日志
    rospy.loginfo("\n" + "-"*50 + " 节点执行日志 " + "-"*50)
    collect_node_info(BT_ROOT_NODE)

    if execution_logs:
        for log in execution_logs:
            rospy.loginfo(log)
    else:
        rospy.logwarn("[节点执行日志] 未检测到叶子节点")

    # 打印层级状态
    rospy.loginfo("\n" + "-"*50 + " 行为树层级状态 " + "-"*50)
    rospy.loginfo(f"[{rospy.Time.now().to_sec():.3f}] 共 {len(BT_REAL_TIME_STATE)} 个节点")
    for node in sorted(BT_REAL_TIME_STATE.values(), key=lambda x: x["depth"]):
        indent = "  " * node["depth"]
        rospy.loginfo(f"{indent}- {node['label']} 【{node['type']}】: {node['status']}")
    rospy.loginfo("-"*106 + "\n")

def bt_state_action_execute_cb(goal):
    global BT_REAL_TIME_STATE, BT_ROOT_NODE
    feedback = GetBtStateFeedback()
    result = GetBtStateResult()
    result.total_node_count = len(BT_REAL_TIME_STATE)
    result.root_node_status = str(BT_ROOT_NODE.status).replace("Status.", "") if BT_ROOT_NODE else "Unknown"

    if goal.query_type == "single":
        py_teset()
        update_bt_state()
        if goal.target_node_label.strip():
            target = goal.target_node_label.strip()
            result.all_nodes_state = json.dumps({target: BT_REAL_TIME_STATE.get(target, {})})
            result.message = f"查询节点：{target}"
        else:
            result.all_nodes_state = json.dumps(BT_REAL_TIME_STATE)
            result.message = f"查询所有节点（共 {len(BT_REAL_TIME_STATE)} 个）"

        # -------------------------- 新增修复 --------------------------
        result.success = True  # 单次查询成功，显式设为 True
        # ---------------------------------------------------------------

        BT_ACTION_SERVER.set_succeeded(result)
        return
    elif goal.query_type == "realtime_feedback":
        start_time = rospy.Time.now()
        prev_state = json.dumps(BT_REAL_TIME_STATE)
        while not rospy.is_shutdown():
            if BT_ACTION_SERVER.is_preempt_requested():
                result.success = False
                result.message = "已取消实时反馈"
                BT_ACTION_SERVER.set_preempted(result)
                return
            if goal.timeout > 0 and (rospy.Time.now() - start_time).to_sec() > goal.timeout:
                break
            update_bt_state()
            feedback.root_node_status = result.root_node_status
            feedback.updated_node_count = len(BT_REAL_TIME_STATE)
            current_state = json.dumps(BT_REAL_TIME_STATE)
            if current_state != prev_state:
                latest_label = max(BT_REAL_TIME_STATE.keys(), key=lambda x: BT_REAL_TIME_STATE[x]["timestamp"])
                feedback.latest_node_state = f"{latest_label}: {BT_REAL_TIME_STATE[latest_label]['status']}"
                BT_ACTION_SERVER.publish_feedback(feedback)
                prev_state = current_state
            rospy.sleep(0.5)
        result.all_nodes_state = json.dumps(BT_REAL_TIME_STATE)
        result.message = f"实时反馈结束（超时 {goal.timeout} 秒）"
        BT_ACTION_SERVER.set_succeeded(result)
        return

    else:
        result.success = False
        result.message = "无效查询类型（仅支持 single/realtime_feedback）"
        BT_ACTION_SERVER.set_succeeded(result)
        return

def init_bt_state_action_server():
    global BT_ACTION_SERVER
    BT_ACTION_SERVER = actionlib.SimpleActionServer(
        "get_bt_state", GetBtStateAction, execute_cb=bt_state_action_execute_cb, auto_start=False
    )
    BT_ACTION_SERVER.start()
    rospy.loginfo("Action 服务端已启动：/get_bt_state")

def main():
    # 初始化节点时开启DEBUG日志
    rospy.init_node("behavior_tree_main", log_level=rospy.INFO)


    # 初始化服务端 + 启动行为树
    init_bt_state_action_server()

    # 3. 🔥🔥🔥 直接在这里运行行为树，无需等待 Action  goal
    # py_teset()

    rospy.spin()



def py_teset():
    # JSON 路径（请确认与实际路径一致）
    json_path = "/home/lab/garb_box/kuavo-ros-control/src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/pytrees_actions/tree_node/test/py_tree.json"
    try:
        with open(json_path, "r") as f:
            json_data = json.load(f)
        rospy.loginfo(f"成功读取 JSON 配置：{json_path}")
        rospy.logdebug(f"JSON配置预览：{str(json_data)[:100]}...")
    except Exception as e:
        rospy.logerr(f"读取 JSON 失败：{str(e)}")
        traceback.print_exc()
        return

    # 自动导入节点模块 + 构建行为树
    auto_import_node_dir_modules("/home/lab/garb_box/kuavo-ros-control/src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/pytrees_actions/node")
    root_node = build_behavior_tree(json_data)
    if not root_node:
        rospy.logerr("行为树构建失败，退出程序")
        return
    bt = py_trees.trees.BehaviourTree(root=root_node)
    rate = rospy.Rate(1)  # 1Hz 更新频率
    rospy.loginfo("行为树已启动，开始输出日志...")

    while not rospy.is_shutdown():
        bt.tick()  # 触发行为树更新
        update_bt_state()  # 更新并打印状态
        rate.sleep()


if __name__ == "__main__":
    main()