#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN总线设备配置管理工具
- 可以批量修改所有电机的ignore状态
-    可供折叠臂磨线测试使用
"""

import yaml
import os

def load_config(config_path):
    """加载YAML配置文件"""
    if not os.path.exists(config_path):
        print(f"错误：文件不存在 - {config_path}")
        return None
    
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def save_config(config, config_path):
    """保存YAML配置文件"""
    with open(config_path, 'w', encoding='utf-8') as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

def modify_ignore_status(config, target_status):
    """
    修改所有设备的ignore状态
    :param config: 配置字典
    :param target_status: 目标状态 (True/False)
    :return: 修改的设备数量
    """
    changed_count = 0
    
    for key in config.keys():
        if key.endswith('_devices'):
            devices = config.get(key, [])
            for device in devices:
                # 只修改电机设备
                if device.get('class') == 'motor' and device.get('ignore') != target_status:
                    device['ignore'] = target_status
                    changed_count += 1
                    status_str = "禁用" if target_status else "启用"
                    print(f"  [{key}] {device.get('name', '未知设备')} -> ignore: {target_status} ({status_str})")
    
    return changed_count

def main():
    # 目标文件路径
    config_path = '/home/lab/.config/lejuconfig/canbus_device_cofig.yaml'
    
    # 加载配置
    config = load_config(config_path)
    if config is None:
        return
    
    # 显示菜单
    print("=" * 50)
    print("      CAN总线设备配置管理工具")
    print("=" * 50)
    print("  1. 禁用所有手臂电机 适用于折叠臂磨线 (ignore: false -> true)")
    print("  2. 启用所有手臂电机 适用于磨线后恢复 (ignore: true -> false)")
    print("  q. 退出")
    print("=" * 50)
    
    # 获取用户选择
    while True:
        choice = input("请输入选择 (1-2,q): ").strip()
        if choice in ['q', '1', '2']:
            break
        print("无效输入，请输入 q、1 或 2")
    
    if choice == 'q':
        print("已退出")
        return
    
    # 执行操作
    print("\n正在处理...")
    if choice == '1':
        changed = modify_ignore_status(config, True)
        action = "禁用"
    else:
        changed = modify_ignore_status(config, False)
        action = "启用"
    
    # 保存配置
    save_config(config, config_path)
    
    # 显示结果
    print(f"\n{action}完成！共修改 {changed} 个电机设备")
    print(f"配置文件已保存: {config_path}")

if __name__ == '__main__':
    main()
