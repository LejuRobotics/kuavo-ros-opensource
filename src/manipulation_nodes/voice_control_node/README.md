# voice_control_node

语音控制 ROS 包，提供基于关键词识别的语音控制功能。

## 功能说明

本包实现了基于 FunASR 的语音识别和关键词匹配功能，可以：
- 订阅麦克风音频数据话题
- 使用 VAD（语音活动检测）和 ASR（自动语音识别）进行语音识别
- 根据关键词匹配执行相应的动作

## 依赖

# 语音识别相关
modelscope==1.10.0 # 兼容python3.8，用于下载模型
umap-learn<=0.5.5 # funasr的依赖，过高版本会依赖不存在的scikit-learn
funasr>=1.0.0
torch>=1.10.0
torchaudio

# 数据处理
numpy<=1.24 # 过高版本可能引起np.bool错误

# 系统工具
psutil>=5.8.0

## 安装

1. 确保已安装所有python依赖（root用户可以访问）
```bash
sudo su
cd src/manipulation_nodes/kuavo_voice_control
pip3 install -r requirements.txt
```
2. 在 catkin workspace 中编译：
```bash
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release # Important! 

catkin build # 全量构建（会重新扫描所有包）
# 或者是
catkin build voice_control_node # 也会编译 humanoid_controllers 包
```

## 使用方法

### 使用 launch 文件启动

** 注意： ** 由于需要顺带启动humanoid_controllers包中的load_kuavo_real.launch，需要先**切换到root用户**执行以下命令

```bash
sudo su
source devel/setup.bash
roslaunch voice_control_node voice_control.launch
```

### 直接运行节点

```bash
sudo su
source devel/setup.bash
rosrun voice_control_node main.py
```

## 配置

配置文件位于 `config/key_words.json`，定义了关键词到动作的映射关系。

## 话题

- 订阅：`/micphone_data` (kuavo_msgs/AudioReceiverData) - 麦克风音频数据

## 节点

- `voice_control_node` - 主语音控制节点

