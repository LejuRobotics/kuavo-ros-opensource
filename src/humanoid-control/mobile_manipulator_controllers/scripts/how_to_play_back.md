# 移动操作器回放模式使用指南

## 快速开始

### 1. 录制数据
使用机器人正常录制的rosbag。

### 2. 启动回放模式
```bash
# 终端1：启动控制器
roslaunch mobile_manipulator_controllers play_back.launch

# 终端2：播放数据
rosbag play xxx.bag --topics \
  /humanoid_wbc_observation \
  /humanoid_mpc_observation \
  /humanoid_mpc_policy \
  /sensors_data_raw \
  /mm/control_type \
  /mm/end_effector_trajectory
```

### 3. 使用自动化脚本（推荐）
```bash
# 快速播放
./scripts/playback_kmpc.sh ~/.ros/2025-07-18-13-53-32_0.bag

# 带时间戳的播放
./scripts/playback_kmpc.sh ~/.ros/2025-07-18-13-53-32_0.bag 12 # 从12s开始回放
```
