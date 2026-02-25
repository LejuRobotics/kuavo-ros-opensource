#!/usr/bin/env python3
import rospy
# 导入自动生成的 Action 消息
from blackboard_utils import apply_blackboard_data_from_json
from py_trees.blackboard import Client

# 导入重构后的模块
from behavior_tree_factory import BehaviorTreeFactory
from behavior_tree_controller import BehaviorTreeController

from pytrees_actions.msg import GetBtStateAction, GetBtStateFeedback, GetBtStateResult
import json
import actionlib

BT_REAL_TIME_STATE = {}
BT_ROOT_NODE = None
BT_ACTION_SERVER = None

def bt_state_action_execute_cb(goal):
    result = GetBtStateResult()
    if goal.query_type == "single":
        pytree_begin()
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


def pytree_begin():
    blackboard_client = Client(name="main_tree_blackboard", namespace="/")
    blackboard_json = "/home/lab/xwy/kuavo-ros-control/src/pytrees_actions/board.json"
    print("apply keyboard")
    apply_blackboard_data_from_json(blackboard_client, blackboard_json)
    # 全局实例用于兼容现有代码调用
    bt_core = BehaviorTreeFactory(blackboard_client)
    bt_controller = BehaviorTreeController(bt_core)
    tree_json_file = "/home/lab/xwy/kuavo-ros-control/src/pytrees_actions/py_tree.json"
    bt_controller.init_services()
    bt_controller.start_behavior_tree(tree_json_file)


def main():
    """主函数"""
    rospy.init_node("behavior_tree_main", log_level=rospy.INFO)
    init_bt_state_action_server()

    rospy.spin()

if __name__ == "__main__":
    main()
