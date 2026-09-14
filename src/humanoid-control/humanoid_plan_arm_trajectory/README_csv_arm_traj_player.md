# CSV 手臂、腰部、头部与灵巧手轨迹播放器使用说明

本文档说明 `script/csv_arm_traj_player.py` 的使用方法。该脚本用于从 CSV 文件按固定帧率同步发布：

- 手臂关节轨迹：`/kuavo_arm_traj`
- 腰部控制指令：`/robot_waist_motion_data`
- 头部控制指令：`/robot_head_motion_data`
- 灵巧手控制指令：`/control_robot_hand_position`

脚本本身只负责读取 CSV、切换手臂控制模式、按帧发布 ROS 话题，不修改底层控制器。

## 运行前准备

先启动仿真或机器人控制器：

```bash
cd /root/kuavo_ws
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch
```

另开一个终端，进入同一个工作区并 source 环境：

```bash
cd /root/kuavo_ws
source devel/setup.bash
```

如果你使用 zsh，也可以执行：

```bash
source devel/setup.zsh
```

## 推荐运行命令

### 等待手柄触发播放

适用于加入头部后的 `*_dexhand_fk.csv`。默认约定：

- `7:34` 是 27 个身体关节位置
- 倒数第 14、13 列是头部 `yaw, pitch`
- 最后 12 列是已经重映射好的灵巧手控制值
- 灵巧手 12 个值顺序为：左手 6 个，右手 6 个

```bash
rosrun humanoid_plan_arm_trajectory csv_arm_traj_player.py \
  --csv src/humanoid-control/humanoid_controllers/config/kuavo_v54/rl/IMG_9977_bvh_50fps_dexhand_fk.csv \
  --delimiter ',' \
  --has-header false \
  --position-columns 7:34 \
  --fps 50 \
  --position-unit rad \
  --publish-waist \
  --publish-head \
  --head-unit rad \
  --publish-hand \
  --wait-for-trigger \
  --trigger-button-name BUTTON_RB
```

按一次 `BUTTON_RB` 后，脚本会切换手臂到外部轨迹控制模式并从头播放 CSV；再次按下会停止发布并切回 RL/自动摆臂模式。

### 不等待手柄，直接播放一次

去掉 `--wait-for-trigger` 和触发按键参数即可：

```bash
rosrun humanoid_plan_arm_trajectory csv_arm_traj_player.py \
  --csv src/humanoid-control/humanoid_controllers/config/kuavo_v54/rl/IMG_9977_bvh_50fps_dexhand_fk.csv \
  --delimiter ',' \
  --has-header false \
  --position-columns 7:34 \
  --fps 50 \
  --position-unit rad \
  --publish-waist \
  --publish-head \
  --head-unit rad \
  --publish-hand
```

### 只播放手臂和腰部

不传 `--publish-head` 和 `--publish-hand` 即可：

```bash
rosrun humanoid_plan_arm_trajectory csv_arm_traj_player.py \
  --csv src/humanoid-control/humanoid_controllers/config/kuavo_v54/rl/IMG_9977_bvh_50fps_dexhand_fk.csv \
  --delimiter ',' \
  --has-header false \
  --position-columns 7:34 \
  --fps 50 \
  --position-unit rad \
  --publish-waist \
  --wait-for-trigger \
  --trigger-button-name BUTTON_RB
```

## CSV 列说明

加入头部后的 CSV 默认布局如下：

| 列范围 | 含义 |
| --- | --- |
| `0:7` | root/body 状态 |
| `7:34` | 27 个身体关节位置 |
| `34:54` | 手指原始或中间数据，当前脚本默认不使用 |
| `54:56` | 头部 `yaw, pitch` |
| `56:68` | 12 个重映射后的灵巧手控制值 |

脚本支持从行尾倒数选列。默认值已经按“头部在手指前两列，手指在最后 12 列”设置：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `--head-columns` | `-14:-12` | 最后 12 列手指前面的 2 列头部 |
| `--hand-columns` | `-12:` | 最后 12 列灵巧手 |

如果在命令行里显式传负数列范围，建议使用等号形式，避免 shell 或 argparse 把它当成新的参数名：

```bash
--head-columns=-14:-12 --hand-columns=-12:
```

`--position-columns 7:34` 选出的 27 个关节中，脚本默认使用：

| 选中位置块内索引 | 含义 |
| --- | --- |
| `12` | 腰部 `waist_yaw_joint` |
| `13:27` | 14 个手臂关节 |

如果你的 CSV 不是这个布局，可以通过参数显式指定：

```bash
--arm-indices ...
--waist-indices ...
--head-columns ...
--hand-columns ...
```

## 发布话题

### 手臂

```text
话题：/kuavo_arm_traj
类型：sensor_msgs/JointState
```

该话题期望手臂角度单位为度。脚本会根据 `--position-unit` 和 `--velocity-unit` 自动把弧度转换为度。

### 腰部

```text
话题：/robot_waist_motion_data
类型：kuavo_msgs/robotWaistControl
```

使用 `--publish-waist` 后启用。腰部单位默认跟随 `--position-unit`，也可以用 `--waist-unit` 单独指定。

### 头部

```text
话题：/robot_head_motion_data
类型：kuavo_msgs/robotHeadMotionData
```

使用 `--publish-head` 后启用。`--head-columns` 必须选中 2 列：

```text
第 1 个 -> yaw
第 2 个 -> pitch
```

该话题期望头部角度单位为度。脚本会根据 `--head-unit` 自动转换：

```text
--head-unit rad  CSV 头部值是弧度，发布前转成度
--head-unit deg  CSV 头部值已经是度，直接发布
不填写          默认跟随 --position-unit
```

### 灵巧手

```text
话题：/control_robot_hand_position
类型：kuavo_msgs/robotHandPosition
```

使用 `--publish-hand` 后启用。`--hand-columns` 必须选中 12 列：

```text
前 6 个 -> left_hand_position
后 6 个 -> right_hand_position
```

灵巧手值会被转换成整数并限制在 `0-100` 范围内：

```text
0   表示完全打开
100 表示完全闭合
```

## 常用参数

| 参数 | 说明 |
| --- | --- |
| `--csv` | CSV 文件路径 |
| `--delimiter` | CSV 分隔符，逗号分隔文件使用 `','` |
| `--has-header` | 是否有表头，可选 `auto/true/false` |
| `--position-columns` | 关节位置列，默认 `7:34` |
| `--velocity-columns` | 关节速度列，可不填 |
| `--position-unit` | 位置单位，可选 `rad/deg` |
| `--velocity-unit` | 速度单位，可选 `rad/deg` |
| `--publish-waist` | 发布腰部轨迹 |
| `--waist-indices` | 腰部在位置块内的索引，默认自动取 `12` |
| `--waist-unit` | 腰部单位，不填则跟随 `--position-unit` |
| `--publish-head` | 发布头部轨迹 |
| `--head-columns` | 2 列头部 yaw/pitch，默认 `-14:-12` |
| `--head-unit` | 头部单位，不填则跟随 `--position-unit` |
| `--publish-hand` | 发布灵巧手轨迹 |
| `--hand-columns` | 12 列灵巧手控制值，默认 `-12:` |
| `--set-mode` | 播放前是否调用手臂模式服务，默认 `true` |
| `--external-mode` | 外部手臂轨迹控制模式，默认 `2` |
| `--rl-mode` | 停止后恢复的手臂模式，默认 `1` |
| `--wait-for-trigger` | 等待手柄按键触发 |
| `--trigger-button-name` | 触发按键名，默认 `BUTTON_RB` |
| `--trigger-button-index` | 手动指定触发按键 index |
| `--loop` | 循环播放 CSV |

## 常见问题

### column spec 7:34 out of range for csv width 1

通常是分隔符传错了。逗号分隔 CSV 要使用：

```bash
--delimiter ','
```

不要使用：

```bash
--delimiter $'\t'
```

否则整行会被当成 1 列。

### CSV 第一行是数字但传了 `--has-header true`

如果 CSV 没有表头，应使用：

```bash
--has-header false
```

也可以使用默认的：

```bash
--has-header auto
```

### 头部没有动

依次检查：

```bash
rostopic echo /robot_head_motion_data
rostopic info /robot_head_motion_data
```

确认运行命令中包含：

```bash
--publish-head
```

如果 CSV 里的头部是弧度，确认使用：

```bash
--head-unit rad
```

如果头部列不是默认的“最后 12 列手指前面的 2 列”，需要显式指定：

```bash
--head-columns=<yaw_col>:<pitch_col_plus_1>
```

### 灵巧手没有动

依次检查：

```bash
rostopic echo /control_robot_hand_position
rostopic info /control_robot_hand_position
rostopic echo /dexhand/state
```

确认运行命令中包含：

```bash
--publish-hand
```

并确认 CSV 的最后 12 列，或你通过 `--hand-columns` 指定的 12 列，确实是 `0-100` 的 12 个重映射灵巧手值。
