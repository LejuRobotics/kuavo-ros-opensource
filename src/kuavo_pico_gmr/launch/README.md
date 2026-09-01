# PICO Motion Retarget Launch 文件使用说明

## 简介

`pico_motion_retarget.launch` 文件用于同时启动 PICO 数据接收节点和运动重定向节点，实现从 PICO 设备到机器人的实时运动映射。

## 功能

该 launch 文件会启动以下两个节点：

1. **pico_comm_minimal** - PICO 数据接收器
   - 接收 PICO 设备通过 UDP 发送的 Protobuf 数据
   - 解析骨骼数据（25 个骨骼点）
   - 转换坐标系并发布到 `/pico/world_bone_poses` topic
   - 可选的 IP 广播功能（让 PICO 设备自动发现服务器）

2. **pico_realtime_retarget** - 运动重定向节点
   - 订阅 `/pico/world_bone_poses` topic
   - 进行运动重定向（GMR pipeline）
   - 发布重定向结果到 `/pico/retargeted_pose` topic
   - 可选的 Mujoco 可视化

## 基本使用

### 启动（带可视化）

```bash
roslaunch kuavo_pico_gmr pico_motion_retarget.launch
```

### 启动（不带可视化）

```bash
roslaunch kuavo_pico_gmr pico_motion_retarget.launch visualize:=false
```

## 参数说明

### visualize (默认: true)
是否启用 Mujoco 可视化

```bash
roslaunch kuavo_pico_gmr pico_motion_retarget.launch visualize:=true
roslaunch kuavo_pico_gmr pico_motion_retarget.launch visualize:=false
```

### robot_type (默认: kuavo_s45)
目标机器人类型

```bash
roslaunch kuavo_pico_gmr pico_motion_retarget.launch robot_type:=kuavo_s45
```

### human_height (默认: 1.32)
估计的人体身高（米）

```bash
roslaunch kuavo_pico_gmr pico_motion_retarget.launch human_height:=1.75
```

### udp_port (默认: 12345)
PICO 数据接收端口

```bash
roslaunch kuavo_pico_gmr pico_motion_retarget.launch udp_port:=12345
```

### publish_tf (默认: true)
是否发布 TF 变换

```bash
roslaunch kuavo_pico_gmr pico_motion_retarget.launch publish_tf:=true
```

### enable_ip_broadcast (默认: true)
是否启用 IP 广播（让 PICO 设备发现服务器）

```bash
roslaunch kuavo_pico_gmr pico_motion_retarget.launch enable_ip_broadcast:=true
```

## 组合使用示例

### 示例 1：不带可视化，自定义身高

```bash
roslaunch kuavo_pico_gmr pico_motion_retarget.launch visualize:=false human_height:=1.75
```

### 示例 2：不启用 IP 广播和 TF 发布

```bash
roslaunch kuavo_pico_gmr pico_motion_retarget.launch enable_ip_broadcast:=false publish_tf:=false
```

### 示例 3：自定义端口和机器人类型

```bash
roslaunch kuavo_pico_gmr pico_motion_retarget.launch udp_port:=8888 robot_type:=kuavo_s45
```

## 发布的 ROS Topics

### 输出 Topics

- `/pico/world_bone_poses` (picoPoseInfoList)
  - PICO 骨骼位姿数据（世界坐标系）
  
- `/pico/raw_bone_poses` (picoPoseInfoList)
  - PICO 原始骨骼位姿数据（PICO 坐标系）

- `/pico/retargeted_pose` (picoPoseRetarget)
  - 重定向后的机器人姿态数据

## 注意事项

1. 确保已经安装所有依赖包：
   ```bash
   pip install numpy rospy geometry_msgs tf kuavo_msgs protobuf netifaces
   ```

2. 确保 PICO 设备与运行 launch 文件的计算机在同一网络

3. 如果启用可视化，需要图形界面支持（不能在纯终端环境运行）

4. 两个节点都设置了 `required="true"`，如果任一节点崩溃，整个 launch 会停止

## 故障排查

### 问题：找不到 kuavo_pico_gmr 包

**解决方案：** 确保已经 source 了工作空间的 setup.bash：
```bash
cd /home/zhongxu/work/kuavo_pico_gmr
source devel/setup.bash
```

### 问题：无法接收 PICO 数据

**解决方案：**
1. 检查网络连接
2. 确认 UDP 端口未被占用
3. 检查防火墙设置

### 问题：可视化窗口无法打开

**解决方案：**
1. 使用 `visualize:=false` 禁用可视化
2. 确保有图形界面支持
3. 检查 Mujoco 是否正确安装

## 相关文件

- `kuavo_gmr/pico_comm_minimal.py` - PICO 数据接收器
- `scripts/pico_realtime_to_robot.py` - 运动重定向节点
- `package.xml` - ROS 包配置文件

## PICO 手柄按键（`pico_comm_minimal`）

PICO 协议**无 Quest 的 touch（触摸）语义**，组合键均基于**按下（pressed）**与**上升沿**触发。

### 按键缩写

| 缩写 | 含义 |
|------|------|
| LT / RT | 左 / 右扳机（trigger ≥ 0.5） |
| LG / RG | 左 / 右握把边键（grip_button 或 grip ≥ 0.5） |
| X / Y | 左手柄面键 |
| A / B | 右手柄面键 |

### `pico_comm_minimal` 组合键

| 组合 | 功能 | 备注 |
|------|------|------|
| RT + Y | 暂停 VMP 推流 | 调 `/vmp/pico_stream_control` |
| RT + X | 恢复 VMP 推流 | |
| RT + B | GMR 半身校准 | `/pico/gmr_calibrate` |
| LT + B | GMR 全身校准 | `/pico/gmr_calibrate_whole` |
| **RG + A** | **下一个 RL 控制器** | `/humanoid_controller/switch_to_next_controller` |
| **RG + B** | **上一个 RL 控制器** | `/humanoid_controller/switch_to_previous_controller`；与 RT+B 互斥 |

控制器按 `rl_controllers.yaml` 中 `enabled: true` 的列表循环：`mpc → RL1 → … → mpc`。切换受 `RLControllerManager` 保护（stance、转腰回中等），失败时见节点日志。

与 Quest3（`QuestControlFSM`）对比：Quest 用 **X+Y 触摸 + A** 切下一个，且**无**手柄切上一个；PICO 用 **RG + A/B** 双向切换。

## 联系方式

如有问题，请联系 KUAVO 团队。

