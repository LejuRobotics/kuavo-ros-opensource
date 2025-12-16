# Kuavo Voice Control ROS Node

## 1. 概述

`voice_control_node` 是一个ROS功能包，提供通过语音关键词来控制机器人行为的能力。

本节点基于 [FunASR](https://github.com/alibaba-damo-academy/FunASR) 实现语音识别，通过识别特定的关键词（如“往前走”、“打招呼”）来触发机器人的相应动作，例如执行单步移动或预设的手臂动作。

## 2. 功能特性

- **实时语音识别**: 采用 FunASR 的 Paraformer-large ASR 模型进行实时的中文语音识别。
- **语音活动检测 (VAD)**: 使用 FSMN VAD 模型检测语音活动，降低计算资源消耗。
- **关键词匹配**: 可在 `scripts/key_words.json` 中自定义关键词及其对应的机器人动作。
- **动作执行**: 集成了机器人底层控制器，可执行基本移动和手臂动作。
- **ROS 集成**: 作为标准的 ROS 节点运行，通过 ROS 话题与其他节点通信。

## 3. 依赖

在运行此节点前，请确保已安装以下软件和依赖项。

### 3.1. Python 依赖

主要的 Python 依赖定义在 `requirements.txt` 文件中，包括：
- `funasr`: 语音识别框架。
- `modelscope==1.10.0`: 用于下载和管理 FunASR 模型。
- `torch` & `torchaudio`: FunASR 的核心计算和音频处理依赖。
- `numpy<=1.24`: 版本限制以避免 API 兼容性问题。
- `psutil`: 系统工具库。
- `uvicorn` & `fastapi`: `modelscope` 运行所需的额外Web服务依赖。
- `umap-learn<=0.5.5`: `funasr` 的一个间接依赖，高版本可能存在兼容性问题。

### 3.2. ROS 依赖

本功能包依赖于以下 ROS 包：
- `kuavo_msgs`: 定义了自定义消息类型（例如 `/micphone_data` 话题）。
- `humanoid_controllers`: 机器人底层控制器，用于执行动作。

## 4. 安装与编译

### 4.1. 使用安装脚本（推荐）

为了确保所有依赖和模型都正确安装，建议使用项目提供的安装脚本。该脚本会自动执行以下操作：
- 安装 `requirements.txt` 中定义的所有 Python 依赖。
- 检查并修复 `modelscope` 和 `umap-learn` 的版本兼容性问题。
- 预先下载并缓存 FunASR 的 **ASR** 和 **VAD** 模型，避免首次运行时下载。
  - ASR模型: `iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch`
  - VAD模型: `iic/speech_fsmn_vad_zh-cn-16k-common-pytorch`

**执行脚本:**
```bash
# 在 voice_control_node 功能包根目录下执行
# 授予脚本执行权限
chmod +x deploy/voice_control_install.sh

# 运行安装脚本
# 如果 ROS 环境需要 sudo 权限，请使用 sudo 运行
sudo bash deploy/voice_control_install.sh
```

### 4.2. 手动安装

如果希望手动安装，请按照以下步骤操作。

#### 步骤 1: 安装 Python 依赖

```bash
# 在 voice_control_node 功能包根目录下执行
pip3 install -r requirements.txt
# 如果需要启动机器人控制节点(humanoid_controllers下的load_kuavo_real.launch)，请使用 sudo
sudo pip3 install -r requirements.txt
```
**注意**: 手动安装可能无法解决 `modelscope` 的潜在版本冲突，并且不会预下载模型。

#### 步骤 2: 编译

在您的 Catkin 工作空间根目录下执行编译命令：
```bash
catkin build voice_control_node
```

## 5. 使用方法

### 5.1. 启动节点

推荐使用 `launch` 文件启动本节点，它会自动加载所需的控制器。

**注意**: 启动前请确保已使用 `sudo su` 切换到 root 用户，这是因为 `humanoid_controllers` 包中的 `load_kuavo_real.launch` 需要 root 权限。

```bash
# 1. 切换到 root 用户
sudo su

# 2. Source 工作空间
source devel/setup.bash

# 3. 启动 launch 文件
roslaunch voice_control_node voice_control.launch
```

`voice_control.launch` 文件会同时启动 `humanoid_controllers` 包中的 `load_kuavo_real.launch`，确保机器人控制器已准备就绪。

### 5.2. 依赖节点

直接运行本节点前，请确保以下节点已经启动：
- **麦克风节点**: 负责采集音频并发布到 `/micphone_data` 话题。

如果程序启动后没有显示语音识别结果，请检查麦克风节点是否正常启动。并且麦克风是否正常连接。

```bash
# 检查麦克风节点是否存在
rosnode list | grep /audio_receiver_node
```

如果返回结果为空，则说明麦克风节点未启动。请检查麦克风节点的启动情况。

## 6. 配置

可以修改 `scripts/key_words.json` 文件来定义或修改关键词和动作的映射关系。

文件结构如下：
```json
{
  "move_forward": {
    "type": "SINGLE_STEP",
    "keywords": ["往前走", "往前", "向前走", "朝前走"],
    "data": {
      "direction": "前",
      "step": 1
    }
  },
  "greet": {
    "type": "ARM_ACTION",
    "keywords": ["打招呼", "挥手", "挥挥手", "挥一下手", "挥个手"],
    "data": "右手打招呼"
  }
}
```
- `type`: 定义动作类型，例如 `SINGLE_STEP`（单步移动）或 `ARM_ACTION`（手臂动作）。
- `keywords`: 触发该动作的关键词列表。
- `data`: 传递给动作执行器的具体参数。

## 7. ROS API

### 7.1. 订阅的话题

- **/micphone_data** (`kuavo_msgs/AudioReceiverData`)
  - 从麦克风节点接收原始音频数据。

### 7.2. 节点

- **`voice_control_node`**
  - 本功能包的主节点，负责所有语音控制逻辑。