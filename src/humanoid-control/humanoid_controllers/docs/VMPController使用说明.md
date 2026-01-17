# VMP 控制器使用说明

## 概述

VMP 控制器是一种基于运动原语的强化学习控制器，用于执行预定义的复杂动作轨迹（如武术动作、舞蹈等）。它通过 VAE 编码器将参考运动轨迹编码为潜在空间表示，结合实时观测数据，由策略网络生成关节控制命令。

### 核心特性

- **多轨迹管理**：支持预加载多条轨迹，运行时动态切换
- **平滑过渡**：轨迹前后自动添加静止帧，支持插值过渡
- **OpenVINO 推理**：使用 OpenVINO 进行高效神经网络推理
- **ROS 服务接口**：提供完整的 ROS 服务用于轨迹查询和执行
- **高频控制**：推理频率 100Hz，控制频率 1000Hz

## 架构

### 控制流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                        VMPController                                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────┐  │
│  │ 轨迹数据管理  │───▶│ VAE 编码器   │───▶│ 潜在向量 (latent)    │  │
│  │ (task_data)  │    │ (Encoder)    │    │                      │  │
│  └──────────────┘    └──────────────┘    └──────────┬───────────┘  │
│                                                       │              │
│  ┌──────────────┐    ┌──────────────┐                │              │
│  │ 传感器数据   │───▶│ 观测构建     │───▶ observation │              │
│  │ (IMU/关节)   │    │              │                │              │
│  └──────────────┘    └──────────────┘                │              │
│                                                       ▼              │
│  ┌──────────────┐    ┌──────────────────────────────────────────┐  │
│  │ 参考运动     │───▶│ 策略网络 (Policy)                        │  │
│  │ (ref_motion) │    │ Input: [obs, ref_motion, latent]         │  │
│  └──────────────┘    │ Output: action (26-dim)                   │  │
│                       └────────────────────┬─────────────────────┘  │
│                                            │                         │
│                                            ▼                         │
│                       ┌──────────────────────────────────────────┐  │
│                       │ PD 控制 + 关节命令生成                    │  │
│                       │ (力矩/位置命令)                           │  │
│                       └────────────────────┬─────────────────────┘  │
│                                            │                         │
└────────────────────────────────────────────┼─────────────────────────┘
                                             ▼
                                      ┌──────────────┐
                                      │  关节命令    │
                                      │ (jointCmd)   │
                                      └──────────────┘
```

### 关键组件

| 组件 | 说明 |
|------|------|
| `VMPController` | VMP 控制器主类，继承自 `RLControllerBase` |
| `VMPConfig` | VMP 配置参数结构体，包含 VAE 和参考运动配置 |
| `TrajectoryData` | 轨迹数据结构，包含原始和处理后的轨迹数据 |
| VAE Encoder | 将参考运动窗口编码为潜在向量 |
| Policy Network | 策略网络，输入观测+参考运动+潜在向量，输出动作 |

## 配置说明

### 1. 在 rl_controllers.yaml 中注册

位置：`config/kuavo_vXX/rl_controllers.yaml`

```yaml
controllers:
  - name: "vmp_controller"
    class: "BASE_CONTROLLER"
    type: "VMP_CONTROLLER"
    config_file: "rl/vmp_param.info"
    enabled: true
```

### 2. VMP 参数配置文件

位置：`config/kuavo_vXX/rl/vmp_param.info`

#### 模型配置

```info
vmpPolicyModelFile   Nov02_13-15-33_v3_nominal.onnx  ; 策略模型文件
vmpEncoderModelFile  policy_vae_64.onnx              ; VAE 编码器模型
vmpRefDataDir        vmp_ref_data/                   ; 参考数据目录
```

#### 轨迹列表配置

```info
trajectories
{
    [0]
    {
        name        "yc_126_spec"           ; 轨迹名称
        data_file   "yc_126_spec_data.bin"  ; 轨迹数据文件
    }
    [1]
    {
        name        "yc_01_front_kick"
        data_file   "task_data_yc_01.bin"
    }
    ; ... 更多轨迹
}
```

#### 静止帧配置

```info
preStandingFrames       80      ; 轨迹前添加的静止帧数量
postStandingFrames      80      ; 轨迹后添加的静止帧数量
standingHeight          0.87    ; 静止帧高度值

preInterpolationFrames   20     ; 前置插值帧数
postInterpolationFrames  50     ; 后置插值帧数
```

## ROS 接口

### 服务 (Services)

VMP 控制器提供以下 ROS 服务（命名空间：`/humanoid_controllers/vmp_controller/`）：

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/trajectory/list` | `kuavo_msgs/GetStringList` | 获取可用轨迹列表 |
| `/trajectory/execute` | `kuavo_msgs/SetString` | 执行指定轨迹 |
| `/trajectory/stop` | `std_srvs/Trigger` | 停止当前轨迹 |

### 话题 (Topics)

| 话题名 | 类型 | 频率 | 说明 |
|--------|------|------|------|
| `/trajectory/state` | `kuavo_msgs/VMPTrajectoryState` | 50Hz | 轨迹播放状态 |

#### VMPTrajectoryState 消息结构

```
Header header
int32 current_index      # 当前轨迹索引
string current_name      # 当前轨迹名称
int32 current_frame      # 当前帧号
int32 total_frames       # 总帧数
float32 progress         # 进度 (0.0 ~ 1.0)
bool completed           # 是否播放完成
```

### 控制器切换

使用主控制器管理服务进行切换：

```bash
# 切换到 VMP 控制器
rosservice call /humanoid_controller/switch_controller "controller_name: 'vmp_controller'"

# 获取当前控制器列表
rosservice call /humanoid_controller/get_controller_list
```

## 命令行工具

使用前需要先 source 工作空间：

```bash
source devel/setup.bash
```

### 1. 查看轨迹列表

```bash
rosrun humanoid_controllers vmp_list_trajectories.py
```

输出示例：
```
==================================================
VMP轨迹列表
==================================================
  [0] yc_126_spec
  [1] yc_01_front_kick
  [2] yc_02_side_kick
  [3] yc_06_straight_punch
==================================================
共 4 条轨迹
```

### 2. 执行轨迹（交互式）

```bash
rosrun humanoid_controllers vmp_execute_trajectory.py
```

交互式选择轨迹：
```
==================================================
可用轨迹列表:
==================================================
  [0] yc_126_spec
  [1] yc_01_front_kick
  [2] yc_02_side_kick
  [3] yc_06_straight_punch
--------------------------------------------------
  [q] 退出
==================================================

请选择轨迹编号 (或输入轨迹名称): 1

执行轨迹: yc_01_front_kick
```

### 3. 切换到 VMP 控制器

```bash
rosrun humanoid_controllers vmp_switch_controller.py
```

### 4. 使用 rosservice 直接调用

```bash
# 获取轨迹列表
rosservice call /humanoid_controllers/vmp_controller/trajectory/list

# 执行指定轨迹
rosservice call /humanoid_controllers/vmp_controller/trajectory/execute "data: 'yc_01_front_kick'"

# 停止轨迹
rosservice call /humanoid_controllers/vmp_controller/trajectory/stop
```

## 添加新轨迹

### 1. 准备轨迹数据文件

轨迹数据文件格式：二进制浮点数数组，每帧包含 `in_c`（默认77）个浮点数。

数据结构（每帧77维）：
- `[0:1)` - 高度 (h)
- `[1:7)` - 6D 旋转表示 (theta)
- `[7:13)` - 速度 (v)
- `[13:39)` - 关节位置 (q, 26维)
- `[39:65)` - 关节速度 (q_dot, 26维)
- `[65:77)` - 末端位置 (p, 4个追踪点 x 3维)

### 2. 放置数据文件

将 `.bin` 文件放置到参考数据目录：
```
{network_model_dir}/../vmp_ref_data/
```

### 3. 更新配置文件

在 `vmp_param.info` 的 `trajectories` 部分添加新条目：

```info
trajectories
{
    ; ... 现有轨迹

    [N]  ; 新轨迹索引
    {
        name        "new_trajectory_name"
        data_file   "new_trajectory_data.bin"
    }
}
```

## 轨迹播放机制

### 静止帧与插值

VMP 控制器在轨迹前后自动添加静止帧，实现平滑过渡：

```
┌─────────────┬──────────────────────────┬─────────────┐
│ 前置静止帧   │      原始轨迹数据         │ 后置静止帧   │
│ (插值过渡)  │                          │ (插值过渡)  │
└─────────────┴──────────────────────────┴─────────────┘
     ↑                                           ↑
 preInterpolationFrames                 postInterpolationFrames
```

- **前置静止帧**：机器人从站立姿态平滑过渡到轨迹起始姿态
- **后置静止帧**：轨迹结束后平滑回到站立姿态
- **插值区域**：使用线性插值实现关节位置的平滑过渡

### 播放状态

| 状态 | 说明 |
|------|------|
| `STOPPED` | 已停止，等待执行命令 |
| `PLAYING` | 正在播放轨迹 |
| `COMPLETED` | 播放完成 |

### 使用注意

> **注意**：VMP 控制器不适合长时间静止站立，在做完动作后应尽快切换到常规控制器（如 MPC 或 AMP）。

## 真实机器人注意事项

### 电机参数切换

在真实机器人上，VMP 控制器会自动切换电机参数：

- **激活时** (`resume`)：切换到 `vmp_yongchun_kpkd`（VMP 专用电机参数）
- **暂停时** (`pause`)：切换回 `normal_kpkd`（正常电机参数）

### 关节限速修改

因为咏春策略所需速度较大，默认的关节速度限制有可能会触发底层的超速保护，因此建议把全部关节的速度限制放大或关闭。

腿部电机速度限制可通过数码管修改，建议设置如下（rpm）：

**前四个关节：**

| 操作指令 | 建议速度限制 (rpm) |
|----------|-------------------|
| 0x3E07   | 2500              |
| 0x3E06   | 2400              |
| 0x3401   | 2300              |

**脚踝关节：**

| 操作指令 | 建议速度限制 (rpm) |
|----------|-------------------|
| 0x3E07   | 5000              |
| 0x3E06   | 4500              |
| 0x3401   | 3400              |

### 机况检查

实物运行对机况要求比较高，运行前请确保：

1. **脚踝检查**：确保脚踝没有松动
2. **十字轴承检查**：检查十字轴承状态良好

### 灵巧手设置

执行咏春动作前，建议先让灵巧手握拳，防止动作过程中双手互相打到造成损坏。

### 安全建议

1. **首次测试**：建议先在仿真环境中测试新轨迹
2. **低速验证**：可通过调整 `actionScale` 降低动作幅度进行验证
3. **急停准备**：确保急停开关可用
4. **环境清理**：执行动作前确保机器人周围无障碍物
5. **灵巧手保护**：执行咏春等涉及双手交互的动作前，先让灵巧手握拳

