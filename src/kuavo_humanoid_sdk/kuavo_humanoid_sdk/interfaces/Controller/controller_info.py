from dataclasses import dataclass


@dataclass
class ControllerListInfo:
    """表示控制器列表信息的数据类"""
    controller_names: list
    """可用控制器名称列表"""
    count: int
    """控制器数量"""
    current_controller: str
    """当前控制器名称"""
    success: bool
    """获取是否成功"""
    message: str
    """返回消息"""


@dataclass
class ControllerResult:
    """表示控制器操作结果的数据类"""
    success: bool
    """操作是否成功"""
    message: str
    """返回消息"""


@dataclass
class SwitchControllerResult:
    """表示控制器切换结果的数据类"""
    success: bool
    """切换是否成功"""
    message: str
    """返回消息"""
    current_controller: str
    """当前控制器名称"""
    target_controller: str
    """目标控制器名称（下一个/上一个）"""
    current_index: int
    """当前控制器索引"""
    target_index: int
    """目标控制器索引"""
