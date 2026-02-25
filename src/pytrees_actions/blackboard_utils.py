import os
import json
import py_trees
from typing import Dict, Any, List, Optional

def apply_blackboard_data_from_json(blackboard_client: py_trees.blackboard.Client, json_file_path: str) -> None:
    """
    从JSON文件加载键值对，并应用到blackboard客户端
    支持将分组和键解析为"group.key"格式，例如 common.box_length
    """
    if not os.path.exists(json_file_path):
        print(f"警告: JSON文件不存在: {json_file_path}")
        return

    with open(json_file_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    # 预先收集所有需要注册的键
    all_keys = []
    if isinstance(data, dict):
        for group_name, group_items in data.items():
            if isinstance(group_items, list):
                for kv_item in group_items:
                    if isinstance(kv_item, dict) and 'key' in kv_item and 'value' in kv_item:
                        full_key = f"{group_name}_{kv_item['key']}"
                        all_keys.append(full_key)

    # 批量注册所有键的写入权限
    for key in all_keys:
        try:
            # 使用不带前导斜杠的键名
            blackboard_client.register_key(key=key, access=py_trees.common.Access.WRITE)
        except Exception as e:
            print(f"注册键 {key} 的写入权限失败: {e}")

    # 现在设置所有键值对
    if isinstance(data, dict):
        # 遍历每个分组（如common、head等）
        for group_name, group_items in data.items():
            if isinstance(group_items, list):
                # 处理分组内的每个key-value对象
                for kv_item in group_items:
                    if isinstance(kv_item, dict) and 'key' in kv_item and 'value' in kv_item:
                        # 构建完整的键名：group.key
                        full_key = f"{group_name}_{kv_item['key']}"
                        value = kv_item['value']
                        print(f"设置黑板键 {full_key} 为 {value}")

                        # 处理嵌套键路径（如pick_pose.stand_position_in_tag）
                        if '.' in kv_item['key']:
                            _apply_nested_dict_to_blackboard(blackboard_client, full_key, value)
                        else:
                            # 设置简单的键值对
                            try:
                                blackboard_client.set(full_key, value)
                            except Exception as e:
                                        print(f"设置黑板键 {full_key} 失败: {e}")
#    elif isinstance(data, dict):
#        # 保持对普通字典格式的支持
#        for key, value in data.items():
#            _apply_nested_dict_to_blackboard(blackboard_client, key, value)

def _apply_nested_dict_to_blackboard(blackboard_client: py_trees.blackboard.Client, prefix: str, data: Any) -> None:
    """
    递归地将嵌套字典应用到黑板上
    """
    if isinstance(data, dict):
        for key, value in data.items():
            # 构建嵌套键路径
            nested_key = f"{prefix}.{key}"
            _apply_nested_dict_to_blackboard(blackboard_client, nested_key, value)
    else:
        # 设置最终的键值对
        try:
            # 尝试注册键（如果尚未注册）
            try:
                blackboard_client.register_key(key=prefix, access=py_trees.common.Access.WRITE)
            except:
                pass  # 如果已经注册过，忽略错误

            blackboard_client.set(prefix, data)
        except Exception as e:
            print(f"设置嵌套黑板键 {prefix} 失败: {e}")

def create_blackboard_client_with_json(json_file_path: str, namespace: Optional[str] = None) -> py_trees.blackboard.Client:
    """
    创建一个新的blackboard客户端，并从JSON文件加载数据
    """
    # 使用更通用的客户端名称
    client = py_trees.blackboard.Client(name="main_blackboard_client", namespace=namespace)
    apply_blackboard_data_from_json(client, json_file_path)
    return client

def get_nested_value(data: Any, keys: str) -> Any:
    """
    从嵌套字典或列表中获取值，支持以点分隔的键路径
    特别处理列表中包含 {'key': 'xxx', 'value': 'xxx'} 格式的情况
    例如: get_nested_value(data, "head.percep_half_fov")
    """
    keys_list = keys.split('.')
    value = data

    for key in keys_list:
        if isinstance(value, dict):
            if key in value:
                value = value[key]
            else:
                return None
        elif isinstance(value, list):
            # 处理列表中包含 {'key': 'xxx', 'value': 'xxx'} 格式的情况
            found = False
            for item in value:
                if isinstance(item, dict) and item.get('key') == key:
                    value = item.get('value')
                    found = True
                    break
            if not found:
                return None
        else:
            return None

    return value
