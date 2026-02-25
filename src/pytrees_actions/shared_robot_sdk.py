"""
共享的 RobotSDK 实例管理器
避免每个节点重复创建 RobotSDK 实例，提升启动速度
"""
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK

_shared_robot_sdk = None

def get_shared_robot_sdk():
    """获取共享的 RobotSDK 实例（单例模式）"""
    global _shared_robot_sdk
    if _shared_robot_sdk is None:
        _shared_robot_sdk = RobotSDK()
    return _shared_robot_sdk

def reset_shared_robot_sdk():
    """重置共享的 RobotSDK 实例（用于测试或重新初始化）"""
    global _shared_robot_sdk
    _shared_robot_sdk = None

