# VMP轨迹文件工具使用文档

## 概述

`vmp_trajectory_tool.py` 是一个用于查看和编辑 VMP (Variational Motion Prior) 二进制轨迹文件的命令行工具。

## 文件格式

VMP轨迹文件 (`.bin`) 是连续存储的 float32 数组，每帧包含 77 维特征：

| 特征 | 索引 | 维度 | 说明 |
|------|------|------|------|
| h (高度) | [0:1] | 1 | 机器人质心高度 (m) |
| theta (6D方向) | [1:7] | 6 | 旋转矩阵前两列 |
| v (速度) | [7:13] | 6 | 2个3D速度向量 |
| q (关节位置) | [13:39] | 26 | 所有关节角度 (rad) |
| q_dot (关节速度) | [39:65] | 26 | 所有关节角速度 |
| p (末端位置) | [65:77] | 12 | 4个追踪点的3D位置 |

文件大小计算：`帧数 × 77 × 4 bytes`

## 安装

无需安装，直接运行即可。依赖：
- Python 3.6+
- NumPy

## 命令

### 1. 查看轨迹信息 (`info`)

显示轨迹文件的基本信息。

```bash
python3 vmp_trajectory_tool.py info <文件路径> [选项]
```

**参数：**
| 参数 | 说明 |
|------|------|
| `<文件路径>` | VMP轨迹文件路径 (.bin) |
| `-v, --verbose` | 显示详细信息（关节位置、末端位置等） |

**示例：**

```bash
# 基本信息
python3 vmp_trajectory_tool.py info task_data_yc_06.bin

# 详细信息
python3 vmp_trajectory_tool.py info task_data_yc_06.bin -v
```

**输出示例：**

```
============================================================
VMP轨迹文件: task_data_yc_06.bin
============================================================
文件路径: task_data_yc_06.bin
文件大小: 662,200 bytes (646.68 KB)
特征维度: 77
总帧数: 2150
轨迹时长: 21.50 秒 (100Hz)

特征维度分布:
----------------------------------------
  h (高度): [0:1], 维度 1
  theta (6D方向): [1:7], 维度 6
  v (速度): [7:13], 维度 6
  q (关节位置): [13:39], 维度 26
  q_dot (关节速度): [39:65], 维度 26
  p (末端位置): [65:77], 维度 12

高度 (h) 统计:
----------------------------------------
  初始: 0.8831 m
  最小: 0.8342 m
  最大: 0.9151 m
  平均: 0.8760 m
  末尾: 0.8724 m
```

---

### 2. 裁剪轨迹 (`trim`)

裁剪轨迹文件的前后帧数。

```bash
python3 vmp_trajectory_tool.py trim <输入文件> <输出文件> [选项]
```

**参数：**
| 参数 | 说明 |
|------|------|
| `<输入文件>` | 输入轨迹文件路径 |
| `<输出文件>` | 输出轨迹文件路径 |
| `--trim-start N` | 去掉开头的 N 帧 (默认: 0) |
| `--trim-end N` | 去掉结尾的 N 帧 (默认: 0) |
| `--start-frame N` | 起始帧号 (与 --trim-start 互斥) |
| `--end-frame N` | 结束帧号 (与 --trim-end 互斥) |

**示例：**

```bash
# 去掉最后 70 帧
python3 vmp_trajectory_tool.py trim input.bin output.bin --trim-end 70

# 去掉前 100 帧
python3 vmp_trajectory_tool.py trim input.bin output.bin --trim-start 100

# 同时去掉前 100 帧和后 50 帧
python3 vmp_trajectory_tool.py trim input.bin output.bin --trim-start 100 --trim-end 50

# 只保留第 200-800 帧 (不含第800帧)
python3 vmp_trajectory_tool.py trim input.bin output.bin --start-frame 200 --end-frame 800

# 从第 500 帧开始到结尾
python3 vmp_trajectory_tool.py trim input.bin output.bin --start-frame 500
```

**输出示例：**

```
原始轨迹: 2150 帧
裁剪范围: [0, 2080)
去掉前: 0 帧
去掉后: 70 帧
新轨迹: 2080 帧 (20.80 秒)
已保存: output.bin
  帧数: 2080
  文件大小: 640640 bytes
```

---

## 常用工作流

### 查看所有轨迹文件信息

```bash
for f in model/vmp_ref_data/*.bin; do
    echo ">>> $f"
    python3 scripts/vmp_trajectory_tool.py info "$f"
    echo
done
```

### 批量裁剪

```bash
# 对所有轨迹文件去掉最后 50 帧
for f in model/vmp_ref_data/task_data_*.bin; do
    base=$(basename "$f" .bin)
    python3 scripts/vmp_trajectory_tool.py trim "$f" "model/vmp_ref_data/${base}_trimmed.bin" --trim-end 50
done
```

### 提取中间片段

```bash
# 提取第 500-1500 帧作为新轨迹
python3 vmp_trajectory_tool.py trim input.bin segment.bin --start-frame 500 --end-frame 1500
```

---

## 时间与帧数换算

轨迹默认采样率为 **100Hz**（每秒100帧）。

| 帧数 | 时间 |
|------|------|
| 100 帧 | 1.0 秒 |
| 500 帧 | 5.0 秒 |
| 1000 帧 | 10.0 秒 |
| 2150 帧 | 21.5 秒 |

---

## 关节顺序

26个关节的顺序：

| 索引 | 关节名 | 索引 | 关节名 |
|------|--------|------|--------|
| 0 | leg_l1 | 13 | arm_l1 |
| 1 | leg_l2 | 14 | arm_l2 |
| 2 | leg_l3 | 15 | arm_l3 |
| 3 | leg_l4 | 16 | arm_l4 |
| 4 | leg_l5 | 17 | arm_l5 |
| 5 | leg_l6 | 18 | arm_l6 |
| 6 | leg_r1 | 19 | arm_l7 |
| 7 | leg_r2 | 20 | arm_r1 |
| 8 | leg_r3 | 21 | arm_r2 |
| 9 | leg_r4 | 22 | arm_r3 |
| 10 | leg_r5 | 23 | arm_r4 |
| 11 | leg_r6 | 24 | arm_r5 |
| 12 | - | 25 | arm_r6 |
| - | - | 26 | arm_r7 |

---

## 追踪点

4个末端追踪点：

| 索引 | 链接名 | 说明 |
|------|--------|------|
| 0 | zarm_l5_link | 左手末端 |
| 1 | zarm_r5_link | 右手末端 |
| 2 | leg_l6_link | 左脚末端 |
| 3 | leg_r6_link | 右脚末端 |

---

## 错误处理

| 错误信息 | 原因 | 解决方法 |
|----------|------|----------|
| `文件不存在` | 文件路径错误 | 检查文件路径是否正确 |
| `文件为空或格式错误` | 文件损坏或不是VMP格式 | 确认文件是有效的VMP轨迹 |
| `无效的裁剪范围` | 起始帧 >= 结束帧 | 检查裁剪参数 |

---

## 相关文件

- 轨迹文件目录: `model/vmp_ref_data/`
- 配置文件: `config/kuavo_vXX/rl/vmp_param.info`
- 加载代码: `src/rl/vmp/vmp_trajectory_loader.hpp`
