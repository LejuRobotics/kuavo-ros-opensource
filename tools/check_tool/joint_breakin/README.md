# Leg_Breakin_Tools

一个基于 EC_Master 的 EtherCAT 驱动器调试工具，支持机器人腿部磨线测试功能，适用于不同型号机器人的关节磨合。

## 🎯 项目简介

- 支持多种机器人型号的腿部磨线测试
- 自动检测机器人版本并选择对应的磨线脚本
- 支持Roban2、Kuavo4、Kuavo5不同机器人

## 🤖 支持的机器人平台

| 版本号 | 机器人类型 | EC从站数量 | 磨线脚本 |
|--------|------------|------------|----------|
| 13-14 | Roban2 | 13个 | `roban2_leg_breakin.py` |
| 40-49 | Kuavo4 | 14个 | `kuavo4_leg_breakin.py` |
| 50-52 | Kuavo5 | 15个 | `kuavo5_leg_breakin.py` |

## ⚙️ 配置说明
- 驱动器类型：配置文件 `~/.config/lejuconfig/EcMasterType.ini` 可指定驱动器类型

## 🔧 磨线测试流程

### 准备工作
1. 将机器人吊起，确保无干涉
2. 将手臂和腿部摆到零点位置
3. 确保周围无障碍物

### 执行步骤
运行程序：`sudo python3 Hardware_tool.py`，其中的 o、m 功能。
