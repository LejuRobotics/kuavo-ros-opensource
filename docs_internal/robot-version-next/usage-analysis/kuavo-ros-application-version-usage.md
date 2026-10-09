# kuavo_ros_application 中 ROBOT_VERSION 使用分析

## 概述

kuavo_ros_application 对 `ROBOT_VERSION` 的使用相对集中，主要在 SLAM 模块的 Launch 文件和 demo 测试脚本中，涉及文件约 10 个。

---

## 使用场景

### 1. Launch 文件中的版本判断（Livox 雷达配置）

5 代机器人反转了雷达安装方向，需要根据版本号选择不同的配置文件。

**文件**：
- `src/kuavo_slam_ws/src/livox_ros_driver2/launch_ROS1/msg_MID360.launch`
- `src/kuavo_slam_ws/src/livox_ros_driver2/launch_ROS1/rviz_MID360.launch`

**用法**：
```xml
<arg name="robot_version" default="$(optenv ROBOT_VERSION 45)"/>
<arg name="is_kuavo5" default="$(eval int(arg('robot_version')) >= 50)"/>

<!-- 根据版本选择配置文件 -->
<arg name="config_file" default="MID360_config_kuavo5.json" if="$(arg is_kuavo5)"/>
<arg name="config_file" default="MID360_config.json" unless="$(arg is_kuavo5)"/>
```

**迁移方式**：`>= 50` → `is_kuavo5()` 布尔函数。

---

### 2. Python 脚本中的版本读取和校验

**文件**：`demo_test/action_scripts/custom_action.py`

读取环境变量并与 .tact 文件中的 robotType 比较，校验兼容性：

```python
env_robot_version = os.environ.get('ROBOT_VERSION')
# 检查 tact 文件中的版本是否匹配当前机器人
```

**迁移方式**：改为 `RobotVersion.from_env()`，通过 `robot_id` 比较。

---

### 3. 版本号映射字典

**文件**：`demo_test/action_scripts/factory_reset_action.py`

```python
ROBOT_VERSION_MAP = {
    "41": "4代标准",
    "42": "4pro短手",
    "45": "4pro长手"
}
```

注意：这个映射表和 kuavo-ros-control 中的 `factory_reset_action.py` 是同一个文件的副本，映射内容更少（只有 3 个版本）。

**迁移方式**：删除字典，改用 `rv.robot_id`。

---

### 4. Shell 脚本版本检测

**文件**：`demo_test/action_scripts/kuavo_tools.sh`

```bash
if [ -n "$ROBOT_VERSION" ]; then
    echo "ROBOT_VERSION: $ROBOT_VERSION"
else
    echo "错误: 未设置 ROBOT_VERSION 环境变量"
fi
```

仅做存在性检查和显示，不做版本判断。

**迁移方式**：无需改动，或改为调用 `RobotVersion.from_env()` 校验格式。

---

### 5. Docker 环境变量

**文件**：
- `src/kuavo_slam_ws/docker/Dockerfile`：`echo 'export ROBOT_VERSION=45' >> /root/.bashrc`
- `src/kuavo_slam_ws/docker/run.sh`：`-e ROBOT_VERSION=$ROBOT_VERSION`
- `src/kuavo_slam_ws/docker/run_with_gpu.sh`：`-e ROBOT_VERSION=$ROBOT_VERSION`

**迁移方式**：过渡期不动。最终态改为新格式字符串。

---

### 6. 硬编码的 URDF 模型路径（biped_s3 / biped_s4 / biped_s42）

多个 Python 脚本和 Launch 文件中硬编码了旧版本的模型路径：

| 模型 | 使用位置 |
|------|---------|
| `biped_s3` | pick_basket/、pick_apriltag_sponge_demo/、kuavo30_moveit_config/ |
| `biped_s4` | kuavo40_moveit_config/ |
| `biped_s42` | ros_robotModel/ |

这些是**静态资源引用**（MoveIt 配置、IK 脚本等），不通过 `ROBOT_VERSION` 动态拼接，而是直接硬编码模型名。

**迁移方式**：这些路径不受 `ROBOT_VERSION` 格式变更影响，不需要迁移。但如果将来这些模型目录重命名为新格式，需要同步更新。

---

## 影响总结

| 场景 | 文件数 | 迁移难度 |
|------|:------:|:--------:|
| Launch 版本判断（`>= 50`） | 2 | 低 |
| Python 版本读取和校验 | 2 | 低 |
| Shell 版本检测 | 1 | 低 |
| Docker 环境变量 | 3 | 低（过渡期不动） |
| 硬编码 URDF 路径 | ~10 | 不需要迁移 |

**总体影响很小**，核心改动只有 4 个文件（2 个 Launch + 2 个 Python），且模式简单（`>= 50` 判断 + 环境变量读取）。
