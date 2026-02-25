import json
import os
import sys
import py_trees
import importlib
from py_trees.blackboard import Blackboard
from py_trees.common import ParallelPolicy
from typing import Dict, Any, Optional
from concurrent.futures import ThreadPoolExecutor, as_completed


class ParamsWrapper:
    """参数包装类，支持通过点分隔的路径获取嵌套值"""
    def __init__(self, params: Dict[str, Any]):
        self.params = self._flatten_params(params)

    def _flatten_params(self, params: Dict[str, Any], parent_key: str = '', sep: str = '.') -> Dict[str, Any]:
        """将嵌套的参数展平为扁平字典"""
        items = []
        for k, v in params.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            if isinstance(v, dict):
                items.extend(self._flatten_params(v, new_key, sep=sep).items())
            else:
                items.append((new_key, v))
        return dict(items)

    def get(self, key: str, default: Any = None) -> Any:
        """通过点分隔的路径获取值"""
        return self.params.get(key, default)

    def __getitem__(self, key: str) -> Any:
        return self.params[key]

    def __contains__(self, key: str) -> bool:
        return key in self.params


class BehaviorTreeFactory:
    """行为树工厂类，支持子树构建、参数解析和 namespace 管理"""

    def __init__(self, blackboard_client, tree_dir: str = None, enable_parallel_loading: bool = True):
        self.tree_dir = tree_dir or os.path.dirname(__file__)
        self.node_dir = os.path.join(os.path.dirname(__file__), 'node')
        self.global_blackboard = blackboard_client
        self.tree = None
        # 缓存已导入的模块，避免重复导入
        self._module_cache = {}
        # 并行加载配置
        self.enable_parallel_loading = enable_parallel_loading
        self._executor = None
        self.subtree_json = os.path.join(self.tree_dir, 'py_tree_child_lb.json')
        print(f"self.subtree_json: {self.subtree_json}")

        with open(self.subtree_json) as f:
            self.subtree_config = json.load(f)

    def load_tree_from_json(self, json_file: str) -> py_trees.trees.BehaviourTree:
        """
        从 JSON 文件加载行为树
        Args:
            json_file (str): JSON 文件路径
        Returns:
            py_trees.trees.BehaviourTree: 构建的行为树
        """
        with open(json_file) as f:
            tree_config = json.load(f)

        if "tree" not in tree_config:
            raise ValueError("Invalid JSON structure: missing 'tree' key")

        root_node = self._build_tree_recursive(tree_config["tree"], parent_namespace=None)
        self.tree = py_trees.trees.BehaviourTree(root=root_node)
        return self.tree

    def _build_tree_recursive(self, node_config: Dict[str, Any], parent_namespace: Optional[str]) -> py_trees.behaviour.Behaviour:
        """
        递归构建行为树
        Args:
            node_config (dict): 当前节点的配置
            parent_namespace (str): 父级 namespace
        Returns:
            py_trees.behaviour.Behaviour: 构建的行为树节点
        """
        node_name = node_config.get("name", "UnnamedNode")
        node_label = node_config.get("label", node_name)
        node_params = node_config.get("params", {})
        childs = node_config.get("childs", [])

        # 生成当前节点的 namespace
        namespace = parent_namespace

        # 如果是子树，处理子树逻辑
        if node_name.endswith(".json"):
            namespace = f"{parent_namespace}__{node_label}" if parent_namespace else node_label
            return self._handle_subtree(node_name, node_label, node_params, namespace)

        # 解析参数
        parsed_params = self._parse_params(node_params, namespace)

        # 创建当前节点
        if node_name in ["Sequence", "Selector", "Parallel"]:
            return self._create_composite_node(node_name, node_label, parsed_params, namespace, childs)
        else:
            return self._create_node_instance(node_name, node_label, namespace, parsed_params)

    def _handle_subtree(self, json_file: str, label: str, params: Dict[str, Any], namespace: str) -> py_trees.behaviour.Behaviour:
        """
        处理子树逻辑
        Args:
            json_file (str): 子树 JSON 文件名
            label (str): 子树的标签
            params (dict): 子树的参数
            namespace (str): 子树的 namespace
        Returns:
            py_trees.behaviour.Behaviour: 子树的根节点
        """
        # 构建子树的 namespace
        subtree_namespace = f"{json_file[:-5]}__{label}"

        # 将 custom 类型的参数写入 namespace 对应的黑板
        namespace_blackboard = py_trees.blackboard.Client(name=subtree_namespace)
        for key, value in params.items():
            if isinstance(value, dict) and value.get("source") == "CUSTOM":
                namespace_blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
                namespace_blackboard.set(key, value.get("value"))

        # 加载子树 config
        subtree_config = self.subtree_config[json_file]

        # 递归构建子树
        subtree_root = self._build_tree_recursive(subtree_config["tree"], parent_namespace=subtree_namespace)
        return subtree_root

    def _parse_params(self, params: Dict[str, Any], namespace: str) -> ParamsWrapper:
        """
        解析参数
        Args:
            params (dict): 参数字典
            namespace (str): 当前节点的 namespace
        Returns:
            ParamsWrapper: 包含解析后参数的包装类
        """
        parsed_params = {}
        for key, value in params.items():
            if isinstance(value, dict) and value.get("source") == "CUSTOM":
                parsed_params[key] = value.get("value")
            elif isinstance(value, dict) and value.get("source") == "read_board":
                # 从全局黑板读取值
                self.global_blackboard.register_key(key=key, access=py_trees.common.Access.READ)
                if self.global_blackboard.exists(key):
                    parsed_params[key] = self.global_blackboard.get(key)
            else:
                parsed_params[key] = value
        return ParamsWrapper(parsed_params)

    def _create_composite_node(self, node_type: str, label: str, params: ParamsWrapper, namespace: str, childs: list) -> py_trees.composites.Composite:
        """
        创建复合节点（Sequence、Selector、Parallel）
        """
        if node_type == "Sequence":
            memory = params.get("memory", False)
            node = py_trees.composites.Sequence(name=label, memory=memory)
        elif node_type == "Selector":
            memory = params.get("memory", False)
            node = py_trees.composites.Selector(name=label, memory=memory)
        elif node_type == "Parallel":
            policy = params.get("policy", "success_on_one")
            if policy == "success_on_one":
                policy_enum = ParallelPolicy.SuccessOnOne()
            else:
                policy_enum = ParallelPolicy.SuccessOnAll()
            node = py_trees.composites.Parallel(name=label, policy=policy_enum)
        else:
            raise ValueError(f"Unknown composite node type: {node_type}")

        # 并行构建子节点（如果启用且子节点数量较多）
        if self.enable_parallel_loading and len(childs) > 1:
            child_nodes = self._build_children_parallel(childs, namespace)
        else:
            # 顺序构建子节点
            child_nodes = []
            for child_config in childs:
                child_node = self._build_tree_recursive(child_config, parent_namespace=namespace)
                child_nodes.append(child_node)

        # 按顺序添加子节点（保持原有顺序）
        for child_node in child_nodes:
            node.add_child(child_node)

        return node

    def _build_children_parallel(self, child_configs: list, namespace: str) -> list:
        """
        并行构建子节点
        """
        child_nodes = [None] * len(child_configs)  # 预分配列表，保持顺序

        # 使用线程池并行构建
        with ThreadPoolExecutor(max_workers=min(len(child_configs), 8)) as executor:
            # 提交所有任务
            future_to_index = {
                executor.submit(self._build_tree_recursive, child_config, namespace): i
                for i, child_config in enumerate(child_configs)
            }

            # 收集结果
            for future in as_completed(future_to_index):
                index = future_to_index[future]
                try:
                    child_nodes[index] = future.result()
                except Exception as e:
                    raise RuntimeError(f"Failed to build child node at index {index}: {e}")

        return child_nodes

    def _create_node_instance(self, node_name: str, label: str, namespace: str, params: ParamsWrapper) -> py_trees.behaviour.Behaviour:
        """
        动态创建行为树节点实例
        """
        # 动态导入模块（使用缓存避免重复导入）
        try:
            # 检查缓存
            if node_name in self._module_cache:
                node_class = self._module_cache[node_name]
            else:
                # 添加节点目录到Python路径
                if self.node_dir not in sys.path:
                    sys.path.insert(0, os.path.dirname(self.node_dir))

                # 直接导入模块（不使用包名）
                module = importlib.import_module(f"node.{node_name}")
                node_class = getattr(module, node_name)
                # 缓存模块类
                self._module_cache[node_name] = node_class

        except AttributeError:
            raise ValueError(f"Node class '{node_name}' not found in py_trees.behaviours")

        print(f"node_name = {node_name}")
        return node_class(name=node_name, label=label, namespace = namespace, params=params)

    def tick(self):
        """执行行为树的一次tick操作"""
        if not self.tree:
            print("[执行tick失败] 行为树实例未初始化")
            return False

        try:
            # 如果启用性能监控，记录节点执行时间
            if hasattr(self, 'enable_node_performance') and self.enable_node_performance:
                self._record_node_execution_times()

            # 执行行为树的一次tick
            self.tree.tick()

            # 状态更新
            self.update_bt_state()

            if self._check_for_failure(self.tree.root):
                print("[行为树失败] 检测到节点失败状态，停止执行")
                return False

            return True
        except Exception as e:
            print(f"[执行tick出错] {e}")
            return False

    def _record_node_execution_times(self):
        """记录每个节点的执行时间（通过遍历节点访问时间戳）"""
        if not self.tree or not self.tree.root:
            return

        # 遍历所有节点，记录执行时间
        # 注意：这里记录的是节点状态变化的时间，实际执行时间需要在节点内部记录
        self._traverse_and_record_node_time(self.tree.root)

    def _traverse_and_record_node_time(self, node, parent_start_time=None):
        """递归遍历节点并记录时间"""
        if parent_start_time is None:
            parent_start_time = time.time()

        node_name = getattr(node, 'name', 'Unknown')

        # 如果节点有自定义的执行时间属性，使用它
        if hasattr(node, '_node_execution_time'):
            exec_time = getattr(node, '_node_execution_time', 0)
            if exec_time > 0:
                if not hasattr(self, 'node_performance'):
                    self.node_performance = {}
                if node_name not in self.node_performance:
                    self.node_performance[node_name] = []
                self.node_performance[node_name].append(exec_time)

        # 递归处理子节点
        if hasattr(node, "children") and node.children:
            for child in node.children:
                self._traverse_and_record_node_time(child, parent_start_time)
        elif hasattr(node, "child") and node.child:
            self._traverse_and_record_node_time(node.child, parent_start_time)

    def update_bt_state(self):
        """更新行为树状态，打印所有节点的状态"""
        if not self.tree or not self.tree.root:
            print("[Update Failed] Behavior tree is not initialized.")
            return

        # 禁用状态输出
        # print("\n[Behavior Tree State]")
        # self._traverse_and_print_node_status(self.tree.root, 0)

    def _traverse_and_print_node_status(self, node: py_trees.behaviour.Behaviour, level: int):
        """递归遍历并打印节点状态"""
        indent = "  " * level
        # print(f"{indent}{node.name}: {node.status}")

        if hasattr(node, "children") and node.children:
            for child in node.children:
                self._traverse_and_print_node_status(child, level + 1)

    def _check_for_failure(self, node):
        """递归检查节点及其子节点是否有失败状态

        Args:
            node: 要检查的节点

        Returns:
            bool: 如果任何节点返回失败状态则为True，否则为False
        """
        if not node:
            return False

        # 检查当前节点是否失败
        if hasattr(node, "status"):
            from py_trees import common
            # 检查status是否为FAILURE（可能需要根据实际的Status枚举进行调整）
            if str(node.status) == "Status.FAILURE" or getattr(node.status, "value", None) == 3:  # 3通常是FAILURE的值
                print(f"[节点失败] {node.name}: {node.status}")
                return True

        # 递归检查子节点
        if hasattr(node, "children") and node.children:
            for child in node.children:
                if self._check_for_failure(child):
                    return True
        elif hasattr(node, "child") and node.child:
            # 处理具有单个子节点的节点类型
            return self._check_for_failure(node.child)

        return False
