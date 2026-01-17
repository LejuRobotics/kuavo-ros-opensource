# motion_capture_calibration_arm 使用说明与运行指南

## 重点文件

- `read_bag_topics.py`
  - 作用：交互式读取 bag，按时间段抽取各话题数据，计算均值；对关节话题调用 FK 服务得到 EE；导出 txt统计数据。

- `utils/load_keyframes.py`
  - 作用：读取/补齐关键帧（YAML）。

- `ik_kmpc_example/`
  - `fk_text.py`
    - 从 `/sensors_data_raw` 获取 14 维手臂关节（或用代码内示例值），调用 `/ik/fk_srv` 输出左右 EE 位姿；
    - 追加写入 `config/target_pose.yaml`（便于手工采样关键帧）。
  - `ik_from_config.py`
    - 读取 `config/target_pose.yaml` 中的关键帧（末端位姿+时间）；
    - 仅在关键帧时刻调用双臂 IK 得到关节角关键帧，然后沿时间线性插值关节角；
    - 以 `JointState` 形式发布到 `/kuavo_arm_traj`（角度单位：度）。
  - `kmpc_from_config.py`
    - 读取关键帧后对末端位姿进行时间插值（位置 Catmull-Rom、姿态 Slerp）；
    - 将插值后的末端目标发布到 `/mm/two_arm_hand_pose_cmd`，并通过 `call_mm_mpc_control_service(1)` 切换 ArmOnly 模式。

## 如何运行bag解析脚本read_bag_topics.py

1) 准备 bag 文件
- 将待分析的 `.bag` 文件放入：`src/demo/motion_capture_calibration_arm/bag/`
- 例：`src/demo/motion_capture_calibration_arm/bag/2025-10-31-20-28-01_8.bag`

2) 启动 FK 服务（若未运行）
```bash
#进入容器，启动mujoco仿真/实物会同时启动服务：
catkin build humanoid_controllers
source devel/setup.zsh   
export ROBOT_VERSION=47    
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch
//如果是实物，请替换为load_kuavo_real.launch   
```

3) 再开一个终端，进入容器
```bash
catkin build kuavo_msgs   
source devel/setup.zsh 
python3 src/demo/motion_capture_calibration_arm/read_bag_topics.py
```

4) 查看输出与图表
- 误差统计表：`summary_keyframe_errors_aligned.txt`
- 数据统计：`summary_keyframe_topic_stats_aligned.txt`

 
## 示例程序运行方式（ik_kmpc_example）

## 进入容器，启动mujoco仿真/实物会同时启动服务
编译消息包并配置环境
```bash
catkin build kuavo_msgs   
source devel/setup.zsh 
```
1) FK 示例：读取传感器或使用内置关节角，调用 `/ik/fk_srv`，可得到左右手末端位姿并写入 `config/target_pose.yaml`，然后自己去补充时间字段

```bash
python3 src/demo/motion_capture_calibration_arm/ik_kmpc_example/fk_text.py
```

2) IK 示例：从配置关键帧解 IK，线性插值关节角并发布到 `/kuavo_arm_traj`

```bash（wbc）
python3 src/demo/motion_capture_calibration_arm/ik_kmpc_example/ik_from_config.py
```

```bash（mpc+wbc）
python3 src/demo/motion_capture_calibration_arm/ik_kmpc_example/ik_from_config.py _enable_wbc_arm_trajectory_control:=false
```
3) MPC 示例：末端位姿插值并发布到 `/mm/two_arm_hand_pose_cmd`

```bash
python3 src/demo/motion_capture_calibration_arm/ik_kmpc_example/kmpc_from_config.py
```

参数说明：
- `rate_hz`：发布/插值频率，默认 100Hz。
- `config_path`：关键帧 YAML 文件路径，默认 `config/target_pose.yaml`。
- 其他：`play_loop_count`：播放循环次数，默认 3。
