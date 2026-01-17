#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quest3 VR 配置管理模块
"""

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


class Quest3VrConfig:
    """
    Quest3 VR 配置管理类
    
    负责读取和管理 quest_vr_config.yaml 配置文件
    """
    
    def __init__(self, config_path: Optional[str] = None):
        """初始化配置管理器"""
        if config_path is None:
            # 默认配置文件路径
            current_dir = os.path.dirname(__file__)
            config_path = os.path.join(current_dir, '../config/quest_vr_config.yaml')
        
        self.config_path = config_path
        self._config_data = None
        self._load_config()
    
    def _load_config(self):
        """加载配置文件"""
        try:
            with open(self.config_path, 'r', encoding='utf-8') as file:
                self._config_data = yaml.safe_load(file)
                print(f"✓ 加载配置文件: {self.config_path}")
        except FileNotFoundError:
            print(f"⚠️  配置文件不存在: {self.config_path}")
            self._config_data = {'hand_wrench': {}}
        except yaml.YAMLError as e:
            print(f"✗ YAML 解析错误: {e}")
            self._config_data = {'hand_wrench': {}}
    
    def save_config(self):
        """保存配置到文件"""
        try:
            os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
            with open(self.config_path, 'w', encoding='utf-8') as file:
                yaml.safe_dump(self._config_data, file, allow_unicode=True, default_flow_style=False)
            print(f"✓ 配置已保存到: {self.config_path}")
            return True
        except Exception as e:
            print(f"✗ 保存配置失败: {e}")
            return False
    
    def get_hand_wrench_config(self, case_name: str) -> Optional[HandWrenchConfig]:
        """获取指定名称的手部力矩配置"""
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
    
    def set_hand_wrench_config(self, case_name: str, config: HandWrenchConfig) -> bool:
        """设置手部力矩配置"""
        if not self._config_data:
            self._config_data = {}
        
        if 'hand_wrench' not in self._config_data:
            self._config_data['hand_wrench'] = {}
        
        self._config_data['hand_wrench'][case_name] = {
            'default': config.default,
            'description': config.description,
            'itemMass': config.itemMass,
            'lforceX': config.lforceX,
            'lforceY': config.lforceY,
            'lforceZ': config.lforceZ
        }
        
        return self.save_config()
    
    def get_default_hand_wrench_config(self) -> Optional[HandWrenchConfig]:
        """获取默认的手部力矩配置"""
        if not self._config_data or 'hand_wrench' not in self._config_data:
            return None
        
        hand_wrench_data = self._config_data['hand_wrench']
        for case_name, case_data in hand_wrench_data.items():
            if case_data.get('default', False):
                return self.get_hand_wrench_config(case_name)
        
        return None
    
    def get_all_hand_wrench_cases(self) -> Tuple[bool, str, Dict[str, HandWrenchConfig]]:
        """获取所有手部力矩配置案例"""
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
    
    def reload_config(self):
        """重新加载配置文件"""
        self._load_config()


if __name__ == "__main__":
    # 测试代码
    config_manager = Quest3VrConfig()
    
    # 获取所有配置
    success, errmsg, configs = config_manager.get_all_hand_wrench_cases()
    if success:
        print("\n所有配置:")
        for name, config in configs.items():
            print(f"  {name}: {config.description}, 质量={config.itemMass}kg")
    
    # 获取默认配置
    default_config = config_manager.get_default_hand_wrench_config()
    if default_config:
        print(f"\n默认配置: {default_config.description}")

