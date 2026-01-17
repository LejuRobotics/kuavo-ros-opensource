# Rosbag到TACT文件转换工具使用说明

## 概述

`rosbag_to_act_frames.py` 是一个交互式工具，用于录制机器人手臂、头部和手指的rosbag数据，并将其转换为TACT格式的动作文件。该工具使用贝塞尔曲线规划器生成平滑的轨迹。

## 主要功能

1. **录制Rosbag数据**：录制 `/sensors_data_raw` 和 `/dexhand/state` 话题的数据
2. **转换Rosbag为TACT文件**：使用贝塞尔曲线规划器将rosbag数据转换为TACT格式的动作文件

## 依赖项

### Python包
- `rosbag` (ROS)
- `rospy` (ROS)
- `questionary` - 交互式命令行界面
- `rich` - 美化终端输出
- `numpy`
- `scipy`

### ROS话题要求
- `/sensors_data_raw` - 手臂和头部关节数据
- `/dexhand/state` - 手指传感器数据

### ROS节点要求
- **`humanoid_plan_arm_trajectory` 节点** - 手臂轨迹规划节点（**转换时必须运行**）
  - 提供 `/bezier/plan_arm_trajectory` 服务
  - 发布 `/bezier/arm_traj` 和 `/bezier/arm_traj_state` 话题
  - 启动命令：
    ```bash
    roslaunch humanoid_plan_arm_trajectory humanoid_plan_arm_trajectory.launch
    ```

### 相关脚本
- `rosbag_to_bezier_planner.py` - 必须与 `rosbag_to_act_frames.py` 在同一目录下

## 使用方法

### 1. 启动脚本

```bash
cd /path/to/scripts
python3 rosbag_to_act_frames.py
```

### 2. 主菜单选项

启动后会显示主菜单，包含以下选项：

```
欢迎使用Rosbag到Tact文件转换工具
主菜单
  1. 录制手臂头部手指rosbag数据
  2. 将rosbag数据转成tact文件
  ─────────────────────────────
  3. 退出
```

### 3. 录制Rosbag数据

选择选项 `1` 后：

1. **输入文件名**：输入要保存的rosbag文件名（不需要包含 `.bag` 后缀）
   - 例如：输入 `my_trajectory`，将生成 `my_trajectory.bag`

2. **开始录制**：
   - 脚本会自动开始录制以下话题：
     - `/sensors_data_raw` - 手臂和头部关节数据
     - `/dexhand/state` - 手指传感器数据
   - 按 `Ctrl+C` 停止录制

3. **完成**：录制完成后，rosbag文件会保存在当前目录

**示例**：
```
请输入要保存的rosbag文件名（不包含.bag后缀）：my_arm_trajectory
开始录制Rosbag...
录制的话题: /sensors_data_raw /dexhand/state
按Ctrl+C停止录制
^C
录制已停止
Rosbag文件已保存为: my_arm_trajectory.bag
```

### 4. 转换Rosbag为TACT文件

**⚠️ 重要提示：在进行转换之前，必须确保 `humanoid_plan_arm_trajectory` 节点已经启动！**

#### 4.0 启动手臂规划节点（转换前必须完成）

在进行转换之前，请先检查并启动手臂规划节点：

1. **检查节点是否运行**：
   ```bash
   rosnode list | grep humanoid_plan_arm_trajectory
   ```

2. **检查服务是否可用**：
   ```bash
   rosservice list | grep bezier
   ```
   应该能看到 `/bezier/plan_arm_trajectory` 服务

3. **如果节点未运行，启动节点**：
   ```bash
   roslaunch humanoid_plan_arm_trajectory humanoid_plan_arm_trajectory.launch
   ```

4. **验证节点启动成功**：
   - 等待节点完全启动（通常需要几秒钟）
   - 再次检查服务列表，确认 `/bezier/plan_arm_trajectory` 服务可用

选择选项 `2` 后：

#### 4.1 选择Rosbag文件

- 如果当前目录下有 `.bag` 文件，会显示文件列表供选择
- 如果没有，需要输入完整的文件路径

#### 4.2 设置转换参数

脚本会依次询问以下参数：

1. **数据预处理采样率**（默认：0.1秒）
   - 用于对原始rosbag数据进行降采样
   - 较小的值会保留更多数据点，但处理时间更长
   - 建议值：0.05 ~ 0.2 秒

2. **贝塞尔曲线平滑因子**（默认：0.3）
   - 控制贝塞尔曲线的平滑程度
   - 范围：0.0 ~ 1.0
   - 较小的值：曲线更接近原始数据，但可能不够平滑
   - 较大的值：曲线更平滑，但可能与原始数据偏差较大
   - 建议值：0.2 ~ 0.4

3. **输出目录**（默认：`./bezier_results`）
   - TACT文件的保存目录
   - 如果目录不存在，会自动创建

4. **是否保存采样结果为TACT文件**（默认：是）
   - 选择是否对规划后的轨迹进行自定义采样并保存

5. **自定义采样率**（如果选择了保存采样结果，默认：0.5秒）
   - 对规划后的轨迹进行重新采样的间隔
   - 留空则不进行采样
   - 建议值：0.1 ~ 1.0 秒

#### 4.3 转换过程

转换过程包括以下步骤：

1. **加载Rosbag数据**：从rosbag文件中提取关节数据
2. **数据预处理**：根据采样率进行降采样
3. **生成贝塞尔控制点**：为每个关节生成平滑的贝塞尔曲线控制点
4. **调用C++规划器**：使用ROS服务 `/bezier/plan_arm_trajectory` 生成轨迹
5. **保存TACT文件**：将生成的轨迹保存为TACT格式

**示例**：
```
请选择要转换的rosbag文件：my_arm_trajectory.bag
开始使用贝塞尔曲线规划器转换 rosbag...
请输入数据预处理采样率（秒，默认0.1）: 0.1
请输入贝塞尔曲线平滑因子（默认0.3）: 0.3
请输入输出目录（默认./bezier_results）: ./bezier_results
是否保存采样结果为TACT文件？ (Y/n): y
请输入自定义采样率（秒，默认0.5，留空则不采样）: 0.5
转换完成！TACT文件已保存到 ./bezier_results
```

## 输出文件

转换完成后，会在输出目录中生成以下文件：

- `sampling_trajectory_YYYYMMDD_HHMMSS.tact` - 采样后的轨迹TACT文件
  - 文件名包含时间戳，避免覆盖
  - 包含28个关节的完整轨迹数据（14个手臂关节 + 12个手指关节 + 2个头部关节）

## 参数说明

### 数据预处理采样率 (`sampling_rate`)

- **作用**：对原始rosbag数据进行降采样，减少数据点数量
- **单位**：秒
- **建议值**：
  - 快速运动：0.05 ~ 0.1 秒
  - 慢速运动：0.1 ~ 0.2 秒
- **影响**：较小的值会保留更多细节，但处理时间更长

### 贝塞尔曲线平滑因子 (`smoothing_factor`)

- **作用**：控制贝塞尔曲线控制点的位置，影响曲线的平滑程度
- **范围**：0.0 ~ 1.0
- **建议值**：0.2 ~ 0.4
- **影响**：
  - 较小值（0.2）：曲线更接近原始数据，但可能不够平滑
  - 较大值（0.4）：曲线更平滑，但可能与原始数据偏差较大

### 自定义采样率 (`custom_sampling_rate`)

- **作用**：对规划后的轨迹进行重新采样，进一步减少关键帧数量
- **单位**：秒
- **建议值**：0.1 ~ 1.0 秒
- **影响**：较小的值会保留更多关键帧，TACT文件更大；较大的值会减少关键帧，文件更小但可能丢失细节

## 注意事项

1. **ROS环境**：
   - 确保ROS环境已正确配置
   - 确保 `/sensors_data_raw` 和 `/dexhand/state` 话题正在发布
   - **转换前必须启动 `humanoid_plan_arm_trajectory` 节点**：
     ```bash
     roslaunch humanoid_plan_arm_trajectory humanoid_plan_arm_trajectory.launch
     ```
   - 确保 `/bezier/plan_arm_trajectory` 服务可用

2. **文件位置**：
   - `rosbag_to_bezier_planner.py` 必须与 `rosbag_to_act_frames.py` 在同一目录下

3. **数据对齐**：
   - 如果两个话题的数据长度不一致，工具会自动用最后一个值填充较短的话题
   - 建议在录制时确保两个话题的数据同步

4. **手指数据单位**：
   - 手指数据（`/dexhand/state`）的范围是 0~100（百分比）
   - 工具会自动处理单位转换

5. **转换时间**：
   - 转换时间取决于rosbag文件的大小和采样率
   - 较大的文件和较小的采样率会导致更长的处理时间

6. **内存使用**：
   - 处理大型rosbag文件时可能需要较多内存
   - 如果遇到内存不足，可以增大采样率

## 故障排除

### 问题1：无法导入 RosbagToBezierPlanner

**错误信息**：
```
错误：无法导入 RosbagToBezierPlanner 类
请确保 rosbag_to_bezier_planner.py 在同一目录下
```

**解决方法**：
- 确保 `rosbag_to_bezier_planner.py` 与 `rosbag_to_act_frames.py` 在同一目录
- 检查文件权限

### 问题2：找不到ROS话题

**错误信息**：
```
Failed to load rosbag data
```

**解决方法**：
- 检查rosbag文件是否包含 `/sensors_data_raw` 和 `/dexhand/state` 话题
- 使用 `rosbag info <bag_file>` 查看rosbag内容

### 问题3：规划器服务不可用

**错误信息**：
```
Service not available
```

**解决方法**：
- **首先启动手臂规划节点**：
  ```bash
  roslaunch humanoid_plan_arm_trajectory humanoid_plan_arm_trajectory.launch
  ```
- 等待节点完全启动（通常需要几秒钟）
- 检查 `/bezier/plan_arm_trajectory` 服务是否可用：
  ```bash
  rosservice list | grep bezier
  ```
- 如果服务仍然不可用，检查节点日志：
  ```bash
  rosnode info /humanoid_plan_arm_trajectory
  ```

### 问题4：转换超时

**错误信息**：
```
trajectory generation timed out
```

**解决方法**：
- 检查规划器节点是否正常运行
- 尝试增大采样率，减少数据点数量
- 检查ROS网络连接

## 示例工作流程

### 完整示例

1. **启动手臂规划节点**（转换前必须完成）：
   ```bash
   roslaunch humanoid_plan_arm_trajectory humanoid_plan_arm_trajectory.launch
   ```
   在新终端中运行，保持节点运行状态

2. **启动脚本**：
   ```bash
   python3 rosbag_to_act_frames.py
   ```

3. **录制数据**：
   - 选择选项 `1`
   - 输入文件名：`test_trajectory`
   - 执行动作（移动手臂和手指）
   - 按 `Ctrl+C` 停止录制

4. **转换数据**：
   - 选择选项 `2`
   - 选择 `test_trajectory.bag`
   - 使用默认参数（直接按回车）
   - 等待转换完成

4. **查看结果**：
   ```bash
   ls -lh ./bezier_results/
   # 应该看到类似 sampling_trajectory_20251203_103447.tact 的文件
   ```

## 相关文档

- `rosbag_to_bezier_planner.py` - 核心转换逻辑
- TACT文件格式说明
- 贝塞尔曲线规划器文档

## 更新日志

- **2025-12-03**: 初始版本
  - 支持录制rosbag数据
  - 支持转换为TACT文件
  - 使用贝塞尔曲线规划器生成平滑轨迹

