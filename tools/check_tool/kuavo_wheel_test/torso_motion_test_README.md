# 躯干往复/位置测试程序

## 功能描述

该程序用于测试机器人躯干在可工作空间内的关键位置与姿态，重点验证：
- Z轴最低/最高高度
- X轴最前/最后位置
- 在上述位置分别测试多种 Pitch 前倾角（0°/正向若干步）
- 自动切换手臂控制模式，发布一次手臂目标位姿以保证安全空间
- 自动回到 base 初始位置

> 当前实现采用“位置枚举+停留”的方式进行与边界覆盖，不再做连续往复插值扫描。

## 使用方法

### 1. 直接运行
```bash
cd /home/user/kuavo-ros-control/tools/check_tool/kuavo_wheel_test
# 可选：指定机器人版本（将用于选择 kuavo.json）
export ROBOT_VERSION=60
python3 torso_motion_test.py
```

### 2. ROS 环境下运行（推荐仍使用直接运行方式）
脚本为独立可执行的 ROS 节点脚本，满足依赖后直接运行即可。若需用 rosrun，请确保已封装为 ROS 包再调用。

## 配置参数

程序会尝试通过 rospack 定位 `kuavo_assets` 包，并读取如下路径中的 `kuavo.json`：
- `$(rospack find kuavo_assets)/config/kuavo_v${ROBOT_VERSION}/kuavo.json`

如未设置 `ROBOT_VERSION` 或配置文件不存在，将使用内置默认值。

### 在 kuavo.json 中的配置格式

在 `kuavo.json` 中添加如下配置段以自定义运动范围与初始位置（单位见注释）：

```json
{
  "kuavo_wheel_torso_limit": {
    "z_range": [0.60, 0.80],   // Z 轴高度范围 (米) [min, max]
    "x_range": [0.00, 0.10],   // X 轴位置范围 (米) [min, max]
    "pitch_range": [-20, 20],  // Pitch 角度范围 (度) [min, max]
    "base_position": [0.0, 0.0, 0.74] // 初始 base_link 位置 (米)
  }
}
```

说明：
- `z_range`、`x_range`、`pitch_range` 为区间 [min, max]；程序会在最小值与最大值（以及若干等步长角度）进行测试。
- `base_position` 为初始姿态所用的 base_link 坐标。

### 默认配置

若未找到配置或缺少对应字段，将使用以下默认值：
- `z_range`: [0.74, 0.94] (米)
- `x_range`: [0.10, 0.20] (米)
- `pitch_range`: [0.00, 20.00] (度)
- `base_position`: [0.05, 0.0, 0.74]
- `base_orientation`: [1.0, 0.0, 0.0, 0.0] (四元数，固定为水平)
- `motion_duration`: 5.0 (秒，单个位置停留时间)

> 角度使用度制；内部换算时按弧度参与 IK 计算。

## 运行流程
1. 启动节点，加载配置
2. 切换手臂控制模式到“外部控制模式”(2)
3. 发布一次 `/kuavo_arm_traj` 目标（14 关节，度制），将双臂收回到安全位姿
4. 先移动到 base 初始位置
5. 计算并测试以下位置序列（每个位置停留 `motion_duration` 秒）：
   - Z 轴最低、最高高度
   - 对于每个 Z 高度，测试 X 轴最前、最后位置
   - 在每个位置测试若干个正向 Pitch 角（含 0°）
6. 全部位置完成后，回到 base 初始位置
7. 切换手臂控制模式回“正常模式”(1)

## 依赖服务与话题

必须可用：
- 服务 `/lb_leg_ik_srv`：腿部逆运动学求解（请求类型 `lbBaseLinkPoseCmdSrv`）
- 服务 `/lb_leg_control_srv`：腿部控制执行（请求类型 `lbLegControlSrv`）
- 服务 `/change_arm_ctrl_mode`：切换手臂控制模式（请求类型 `changeArmCtrlMode`）
- 话题 `/sensors_data_raw`：用于读取当前手臂关节角（消息类型 `kuavo_msgs/sensorsData`）

程序发布：
- 话题 `/kuavo_arm_traj`（`sensor_msgs/JointState`）：在开始阶段线性插值发布一次手臂目标轨迹（单位：度）