---
title: "任务模拟环境"
---
# 文档说明
## launch
任务场景启动文件与任务动作启动文件

## model
### assets
物品模型与texture贴图

### biped_s45
kuavo模型（ROBOT_VERSION = 45, 配有Robotiq 2f-85夹爪），任务场景xml文件
# 如何使用
## 启动模拟场景
cd至launch文件夹
```bash
python3 launch.py
```
选择要启动的场景, 等待场景启动后，terminal里会提示启动任务，输入s并Enter来启动相应的任务(任务结束后需要手动点击mujoco窗口的关闭)

## 数据采集脚本
cd至launch文件夹
```bash
python3 automation.py
```
运行后选择任务，再选择是否录制rosbag，若不录制，则只会执行相应任务后关闭
若录制，选择采集的次数，录制的rosbag会保存在bags文件夹内，同时录制完成后会打印成功次数，失败的采集不会保存rosbag