#!/usr/bin/env python3
import rospy
# 导入自动生成的 Action 消息
from blackboard_utils import apply_blackboard_data_from_json
from py_trees.blackboard import Client

# 导入重构后的模块
from behavior_tree_factory import BehaviorTreeFactory
from behavior_tree_controller import BehaviorTreeController

blackboard_client = Client(name="main_tree_blackboard", namespace="/")
blackboard_json = "/home/lab/lpf/kuavo-ros-control/src/pytrees_actions/board.json"
# 全局实例用于兼容现有代码调用
bt_core = BehaviorTreeFactory(blackboard_client)
bt_controller = BehaviorTreeController(bt_core)


def main():
    """主函数"""
    # 初始化节点时开启DEBUG日志
    rospy.init_node("behavior_tree_main", log_level=rospy.INFO)
    tree_json_file = "/home/lab/lpf/kuavo-ros-control/src/pytrees_actions/py_tree_mes.json"

    bt_controller.init_services()
    print(f"开始解析board.json")
    apply_blackboard_data_from_json(blackboard_client, blackboard_json)
    bt_controller.start_behavior_tree(tree_json_file)

    rospy.spin()

if __name__ == "__main__":
    main()