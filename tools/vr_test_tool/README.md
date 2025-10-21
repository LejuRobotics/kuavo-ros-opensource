# Quest3 设备手臂数据分析工具

## 功能介绍

此工具用于分析 Quest3 设备的手臂数据一致性，通过比较多个设备的数据来验证设备间的一致性。

## 使用方法

```bash
python quest3_analyze_arm_data.py [-h] [-o OUTPUT] bag_files [bag_files ...]
```

### 参数说明

- `bag_files`: 要分析的 bag 文件路径（至少一个），bag 时长需在20秒以上，且 10-20秒的区间保持固定的动作
- `-h`, `--help`: 显示帮助信息
- `-o OUTPUT`, `--output OUTPUT`: 输出报告文件路径，默认为脚本目录下的 `arm_analysis_report.json`

### 示例

```bash
# 分析单个 bag 文件
python quest3_analyze_arm_data.py ./data/device1.bag

# 分析多个 bag 文件
python quest3_analyze_arm_data.py ./data/device1.bag ./data/device2.bag ./data/device3.bag

# 分析多个 bag 文件并指定输出报告路径
python quest3_analyze_arm_data.py -o ./reports/analysis_result.json ./data/device1.bag ./data/device2.bag
```

## 输出结果

正在分析 device1 (forward_stretch_device1.bag) 的数据...
device1 左臂数据点数: xxxx
device1 右臂数据点数: xxxx

正在分析 device2 (forward_stretch_device2.bag) 的数据...
device2 左臂数据点数: xxxx
device2 右臂数据点数: xxxx

正在分析 device3 (forward_stretch_device3.bag) 的数据...
device3 左臂数据点数: xxxx
device3 右臂数据点数: xxxx

脚本执行后会生成一个 JSON 格式的报告文件，包含以下信息：

1. 设备统计数据：

   - 每个设备的左臂和右臂数据统计信息
   - 包括均值、标准差、最小值、最大值等
2. 设备间一致性比较：

   - 左臂和右臂在不同设备间的一致性比较
   - 主要关注大手和小臂的比例（distance_ratio）
