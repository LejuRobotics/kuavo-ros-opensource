# 手臂碰撞检测包 kuavo_arm_collision_check

### 更新记录
#### 2025.11.05更新
- 删除原本测试用的meshes文件夹,默认从kuavo_assets中根据$ROBOT_VERSION读取stl文件进行膨胀
- 使用yaml配置文件对膨胀系数进行管理,支持手动调整每个关节的膨胀系数
- 使用yaml配置文件管理膨胀检测需要跳过的link对
- 默认存在s45和s49两个yaml文件,以便区分
- 添加src/kuavo_sdk/sdk/04_use_arm/use_target_control.py文件,用于测试碰撞检测保护效果
- 因49版本灵巧手结构较为复杂,因此直接在函数中添加处理逻辑,防止单只灵巧手自身结构的干涉

**暂时不支持半身**

用于检测机器人手臂的碰撞情况。当启用手臂碰撞后自动归位时，发生碰撞时移动手臂到 3 秒前的状态。

**注意，如果需要启动碰撞归位功能，需要将原本发送到 `kuavo_arm_traj` 和 `kuavo_arm_target_poses`改为发送到`/arm_collision/kuavo_arm_traj`和`/arm_collision/kuavo_arm_target_poses`，否则发送碰撞归位时会发送冲突**

未启用手臂碰撞后自动归位，只发布发送碰撞的两个关节信息。

## 启动方式

### 基本启动
```bash
roslaunch kuavo_arm_collision_check arm_collision_check.launch
```

### 启动时开启碰撞自动归位
```bash
roslaunch kuavo_arm_collision_check arm_collision_check.launch arm_collision_enable_arm_moving:=true
```


### VR 遥操启用手臂碰撞检测归位
```bash
roslaunch kuavo_arm_collision_check arm_collision_check.launch arm_collision_enable_arm_moving:=true
```
```bash
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_arm_collision:=true
```
当启用手臂碰撞后自动归位时，VR 外部手臂控制时发生碰撞移动手臂到 3 秒前的状态，手臂回到 3 秒前状态后 VR 手柄再次按下 XA 可恢复 VR 外部手臂控制模式。


## 功能说明

该节点启动后根据 `ROBOT_VERSION` 环境变量读取 URDF 文件，并加载相应的 STL 模型。模型加载优先级如下：

1. **cache 文件夹**：优先使用缓存模型（经过处理的模型）
2. **meshes 文件夹(已删除)**：使用经过减面和内部掏空处理的曲面模型（适用于长手灵巧手机器人版本）
3. **kuavo_assets**：如果上述文件夹中没有 STL 文件，则使用 kuavo_assets 内的原始 STL 文件

**注意**：当前仅支持 `ROBOT_VERSION=45` 和 `ROBOT_VERSION=49`，其他版本可能导致关节不匹配问题。

## 配置文件说明

- 文件路径:$(find kuavo_arm_collision_check)/config/s$(arg robot_version)_collision_config.yaml

#### 1. 膨胀系数（Inflation）

- 说明:每个关节或 STL 模型的膨胀系数（单位：米）。膨胀系数用于在碰撞检测时，对模型进行微小放大，以提高安全性和鲁棒性。

```yaml
inflation:
  torso_STL: 0.020
  camera_STL: 0.020
  ...
  r_hand_pitch_STL: 0.020
  head_yaw_STL: 0.020
  head_pitch_STL: 0.020
```

- 注意事项：
  - 为了避免 ROS 参数名称不合法，原文件名中的 . 或 - 已经被替换成 _。
  - 可以针对不同关节设置不同膨胀系数，例如手臂关节可以略大于腿部关节以增加碰撞安全边界。
  - 修改膨胀系数时，需要保证数值在合理范围（一般 0.01~0.05 米），过大可能导致虚假碰撞，过小可能漏检。

#### 2. 碰撞过滤规则（Collision Filter）

- 说明:定义不进行碰撞检测的关节对（joint pairs），用于屏蔽机械臂自身相邻关节或特定关节的碰撞检测。

```yaml
collision_filter:
  - [r_hand_tripod, zarm_r5_link]
  - [l_hand_tripod, zarm_l5_link]
```

- 注意事项：
  - 列表中每一项是一个关节对 [linkA, linkB]。
  - 关节名必须与机器人模型加载时使用的名称一致。
  - 当检测到 linkA 与 linkB 时，将跳过该碰撞对。
  - 可以根据不同机器人版本或手型添加新的屏蔽规则

## 缓存机制

程序首次加载时会使用 OpenMesh 库处理 STL 模型，并将处理后的文件存储在 `cache` 文件夹内。后续启动将直接使用缓存模型以提高性能。

**注意**：编译时会清空 `cache` 文件夹，以防止代码修改导致缓存模型失效。

## 工作流程

1. 加载 STL 文件
2. 将 STL 文件内的模型添加为 FCL 的碰撞体
3. 订阅 TF，实时更新碰撞体的位姿
4. 按照设定频率（默认 5Hz）计算碰撞检测
5. 发生碰撞时发布碰撞信息
6. 持续记录传感器数据（默认 3 秒）

## 发布的话题

- `/arm_collision/info`：碰撞信息，包含涉及碰撞的关节名
  - 消息类型：`kuavo_msgs/armCollisionCheckInfo`
- `/arm_collision/check_duration`：碰撞检测耗时（毫秒）
- `/arm_collision/markers`：用于 RViz 显示的碰撞标记（可选，启用会增大耗时）
  - 消息类型：`kuavo_msgs/sensorsData`
- `/kuavo_arm_target_poses`：手臂目标位姿
  - 消息类型：`kuavo_msgs/armTargetPoses`
- `/kuavo_arm_traj`：手臂轨迹数据
  - 消息类型：`sensor_msgs/JointState`

## 订阅的话题

- `/arm_collision/kuavo_arm_traj`：手臂轨迹数据
  - 消息类型：`sensor_msgs/JointState`
- `/arm_collision/kuavo_arm_target_poses`：手臂目标位姿
  - 消息类型：`kuavo_msgs/armTargetPoses`
- `/sensors_data_raw`：原始传感器数据
  - 消息类型：`kuavo_msgs/sensorsData`

## 服务

- `/arm_collision/wait_complete`：等待碰撞回归完成
  - 服务类型：`std_srvs/SetBool`
  - 用于同步操作，等待当前操作完成
- `/arm_collision/set_arm_moving_enable`：设置手臂移动检测启用状态
  - 服务类型：`std_srvs/SetBool`
  - 动态启用或禁用手臂发生碰撞时移动归位功能

## 节点参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `/arm_collision/check_freq` | double | 5.0 | 碰撞检测频率（Hz） |
| `/arm_collision/publish_collision_markers` | bool | false | 是否发布碰撞标记用于 RViz 显示 |
| `/arm_collision/enable_arm_moving` | bool | false | 是否启用手臂碰撞后自动归位 |
| `/arm_collision/arm_move_diff` | double | 0.1 | 手臂移动差异阈值 |

## 依赖项

- ROS
- FCL (Flexible Collision Library)
- OpenMesh
- kuavo_msgs
- kuavo_common
- tf2_ros
- sensor_msgs
- visualization_msgs

## 使用示例

1. 启动碰撞检测节点：
   ```bash
   roslaunch kuavo_arm_collision_check arm_collision_check.launch
   ```

2. 查看碰撞信息：
   ```bash
   rostopic echo /arm_collision/info
   ```

3. 在 RViz 中显示碰撞标记：
   ```bash
   roslaunch kuavo_arm_collision_check arm_collision_check.launch arm_collision_publish_collision_markers:=true
   ```
   （默认禁用，需要手动启用）

4. 调整检测频率：
   ```bash
   roslaunch kuavo_arm_collision_check arm_collision_check.launch arm_collision_check_freq:=10.0
   ```

5. 禁用手臂移动检测：
   ```bash
   roslaunch kuavo_arm_collision_check arm_collision_check.launch arm_collision_enable_arm_moving:=false
   ```

## 故障排除

- 如果碰撞检测频率过低，可以调整 `arm_collision_check_freq` 参数
- 如果模型加载失败，检查 URDF 文件路径和 STL 文件是否存在
- 如果缓存出现问题，可以手动删除 `cache` 文件夹重新生成
- 如果手臂移动检测不工作，检查 `enable_arm_moving_check_` 参数是否正确设置
- 确保 `ROBOT_VERSION` 环境变量已正确设置（当前支持版本 45）
- 如果传感器数据记录异常，检查 `/sensors_data_raw` 话题是否正常发布
- 如果服务调用失败，检查服务是否正常启动和参数是否正确