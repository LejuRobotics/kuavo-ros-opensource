# Copyright (c) 2024 lejurobot
# Author: lejurobot
# Brief: Pico VR configuration management module

import yaml
import os
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass


@dataclass
class HandWrenchConfig:
    """
    手部力矩配置数据类
    
    Attributes:
        default (bool): 是否为默认配置
        description (str): 配置描述信息
        itemMass (float): 物品质量 (kg)
        lforceX (float): X方向力 (N)
        lforceY (float): Y方向力 (N)
        lforceZ (float): Z方向力 (N)
    """
    default: bool
    description: str
    itemMass: float
    lforceX: float
    lforceY: float
    lforceZ: float


class PicoVrConfig:
    """
    Pico VR 配置管理类
    
    负责读取和管理 pico_vr_config.yaml 配置文件，提供手部力矩配置的
    读取、查询和管理功能。
    """
    
    def __init__(self, config_path: Optional[str] = None):
        """
        初始化 Pico VR 配置管理器
        
        Args:
            config_path (Optional[str]): 配置文件路径，如果为 None 则使用默认路径
        """
        if config_path is None:
            # 首先检查用户配置目录
            user_config_path = os.path.expanduser('~/.config/lejuconfig/pico_vr_config.yaml')
            if os.path.exists(user_config_path):
                config_path = user_config_path
                print(f"Using user config file: {user_config_path}")
            else:
                # 如果用户配置文件不存在，使用当前目录的配置文件
                current_dir = os.path.dirname(__file__)
                print(f"current_dir: {current_dir}")
                config_path = os.path.join(current_dir, '..', '..', '..', 'config', 'pico_vr_config.yaml')
                print(f"User config file not found, using default config file: {config_path}")
        
        print(f"config_path: {config_path}")

        self.config_path = config_path
        self._config_data = None
        self._load_config()
    
    def _load_config(self):
        """
        加载配置文件
        
        Raises:
            FileNotFoundError: 配置文件不存在
            ValueError: YAML 解析错误
        """
        try:
            with open(self.config_path, 'r', encoding='utf-8') as file:
                self._config_data = yaml.safe_load(file)
        except FileNotFoundError:
            print(f"Configuration file not found: {self.config_path}")
        except yaml.YAMLError as e:
            print(f"Error parsing YAML configuration: {e}")
    
    def get_hand_wrench_config(self, case_name: str) -> Optional[HandWrenchConfig]:
        """
        获取指定名称的手部力矩配置
        
        Args:
            case_name (str): 配置案例名称
            
        Returns:
            Optional[HandWrenchConfig]: 手部力矩配置对象，如果不存在则返回 None
        """
        if not self._config_data or 'hand_wrench' not in self._config_data:
            return None
        
        hand_wrench_data = self._config_data['hand_wrench']
        if case_name not in hand_wrench_data:
            return None
        
        case_data = hand_wrench_data[case_name]
        return HandWrenchConfig(
            default=case_data.get('default', False),
            description=case_data.get('description', ''),
            itemMass=case_data.get('itemMass', 0.0),
            lforceX=case_data.get('lforceX', 0.0),
            lforceY=case_data.get('lforceY', 0.0),
            lforceZ=case_data.get('lforceZ', 0.0)
        )
    
    def get_default_hand_wrench_config(self) -> Optional[HandWrenchConfig]:
        """
        获取默认的手部力矩配置
        
        Returns:
            Optional[HandWrenchConfig]: 默认手部力矩配置对象，如果不存在则返回 None
        """
        if not self._config_data or 'hand_wrench' not in self._config_data:
            return None
        
        hand_wrench_data = self._config_data['hand_wrench']
        for case_name, case_data in hand_wrench_data.items():
            print(f"case_name: {case_name}, case_data: {case_data}")
            if case_data.get('default', False):
                return self.get_hand_wrench_config(case_name)
        
        return None
    
    def get_all_hand_wrench_cases(self) -> Tuple[bool, str, Dict[str, HandWrenchConfig]]:
        """
        获取所有手部力矩配置案例
        
        Returns:
            Tuple[bool, str, Dict[str, HandWrenchConfig]]: (success, errmsg, 所有手部力矩配置的字典)
        """
        try:
            if not self._config_data or 'hand_wrench' not in self._config_data:
                return False, "配置数据不存在或缺少hand_wrench配置", {}
            
            hand_wrench_data = self._config_data['hand_wrench']
            result = {}
            for case_name in hand_wrench_data.keys():
                config = self.get_hand_wrench_config(case_name)
                if config:
                    result[case_name] = config
            
            return True, "", result
        except Exception as e:
            return False, f"获取手部力矩配置失败: {str(e)}", {}
    
    def get_raw_config(self) -> Dict[str, Any]:
        """
        获取原始配置数据
        
        Returns:
            Dict[str, Any]: 原始配置字典
        """
        return self._config_data or {}
    
    def reload_config(self):
        """
        重新加载配置文件
        
        当配置文件发生变化时，可以调用此方法重新加载配置
        """
        self._load_config()
    
    def __str__(self) -> str:
        """
        返回配置管理器的字符串表示
        
        Returns:
            str: 配置管理器的字符串描述
        """
        return f"PicoVrConfig(config_path='{self.config_path}')"


def main():
    """
    测试 Pico VR 配置管理器的主函数
    """
    print("=== Pico VR 配置管理器测试 ===")
    
    try:
        # 初始化配置管理器
        config = PicoVrConfig()
        print(f"配置文件路径: {config.config_path}")
        print(f"配置管理器: {config}")
        
        # 测试获取默认配置
        print("\n--- 测试获取默认配置 ---")
        default_config = config.get_default_hand_wrench_config()
        if default_config:
            print(f"默认配置:")
            print(f"  描述: {default_config.description}")
            print(f"  物品质量: {default_config.itemMass} kg")
            print(f"  力 X: {default_config.lforceX} N")
            print(f"  力 Y: {default_config.lforceY} N")
            print(f"  力 Z: {default_config.lforceZ} N")
        else:
            print("未找到默认配置")
        
        # 测试获取所有配置
        print("\n--- 测试获取所有配置 ---")
        success, errmsg, all_configs = config.get_all_hand_wrench_cases()
        if success:
            print(f"找到 {len(all_configs)} 个配置:")
            for case_name, case_config in all_configs.items():
                print(f"  {case_name}:")
                print(f"    描述: {case_config.description}")
                print(f"    默认: {case_config.default}")
                print(f"    物品质量: {case_config.itemMass} kg")
                print(f"    力: X={case_config.lforceX}, Y={case_config.lforceY}, Z={case_config.lforceZ} N")
        else:
            print(f"获取配置失败: {errmsg}")
        
        # 测试获取特定配置
        print("\n--- 测试获取特定配置 ---")
        test_cases = ["case1_for_changchun_yiqi_box", "case2", "nonexistent_case"]
        for case_name in test_cases:
            case_config = config.get_hand_wrench_config(case_name)
            if case_config:
                print(f"配置 {case_name} 存在:")
                print(f"  描述: {case_config.description}")
                print(f"  物品质量: {case_config.itemMass} kg")
            else:
                print(f"配置 {case_name} 不存在")
        
        # 测试重新加载配置
        print("\n--- 测试重新加载配置 ---")
        config.reload_config()
        print("配置重新加载完成")
        
        print("\n=== 测试完成 ===")
        
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()