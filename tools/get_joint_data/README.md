# 机器人标定数据生成脚本

## 功能概述

`generate_cali_data.py` 是一个用于生成机器人标定信息的Python脚本。该脚本能够自动读取机器人的零点配置文件、URDF模型文件和电机配置信息，并生成标准化的JSON格式标定数据文件。

## 支持的关节

脚本处理以下28个关节的标定数据：

### 腿部关节 (12个)
- **左腿**: `leg_l1_joint` 到 `leg_l6_joint`
- **右腿**: `leg_r1_joint` 到 `leg_r6_joint`

### 肩部关节 (2个)
- **左肩**: `zarm_l1_joint`
- **右肩**: `zarm_r1_joint`

### 手臂关节 (12个)
- **左臂**: `zarm_l2_joint` 到 `zarm_l7_joint`
- **右臂**: `zarm_r2_joint` 到 `zarm_r7_joint`

### 头部关节 (2个)
- **头部**: `zhead_1_joint`, `zhead_2_joint`

## 使用方法

### 1. 环境准备
确保以下文件存在于指定路径：
```bash
# URDF模型文件
~/kuavo-ros-opensource/src/kuavo_assets/models/biped_s{ROBOT_VERSION}/urdf/biped_s{ROBOT_VERSION}.urdf

# 零点配置文件（出厂时下位机自带）
~/.config/lejuconfig/arms_zero.yaml
~/.config/lejuconfig/offset.csv
~/.config/lejuconfig/config.yaml
```

### 2. 运行脚本
```bash
cd tools/get_joint_data
python3 generate_cali_data.py
```

### 3. 输出文件
脚本将在用户主目录下生成 `robot_calibration.json` 文件。

## 配置文件说明

### arms_zero.yaml
包含手臂和颈部关节的零点位置数据，共14个值：
- 索引0-5: 左臂关节 (zarm_l2到zarm_l7)
- 索引6-11: 右臂关节 (zarm_r2到zarm_r7)
- 索引12-13: 头部关节 (zhead_1, zhead_2)

### offset.csv
包含腿部关节的零点偏移数据，每行一个角度值（度），共14个值：
- 索引0-5: 左腿关节 (leg_l1到leg_l6)
- 索引6-11: 右腿关节 (leg_r1到leg_r6)
- 索引12: 左肩关节 (zarm_l1)
- 索引13: 右肩关节 (zarm_r1)

### config.yaml
包含电机反转配置信息，通过 `negtive_address` 字段指定需要反转的电机ID。

## 输出格式

生成的JSON文件包含以下结构：
```json
{
  "joint_name": {
    "id": 0,
    "drive_mode": 0,
    "homing_offset": 0.0,
    "range_min": -3.14,
    "range_max": 3.14
  }
}
```

### 字段说明
- **id**: 关节ID，从0开始递增
- **drive_mode**: 驱动模式，0表示正向，1表示反向
- **homing_offset**: 零点偏移值（弧度）
- **range_min**: 关节运动下限（弧度）
- **range_max**: 关节运动上限（弧度）

## 注意事项

### 重要提醒
1. **机器差异**: 每个机器的零点略有差异，需要在每个机器上单独运行脚本生成标定文件
2. **配置更新**: 如果机器人零点文件有改动，必须重新生成JSON文件
3. **路径确认**: 使用前需要确认仓库路径，修改脚本中的 `urdf_path` 变量
