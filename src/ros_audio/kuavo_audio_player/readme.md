# kuavo_audio_player 使用说明

## 1. 系统架构说明

kuavo_audio_player 全机型统一为单节点 `audio_player_node`（`audio_player.py`）：提供 `/play_music` 服务播放本地音频文件，订阅 `/audio_data` 接收外部 PCM 流，统一经单环形缓冲 + 单 PortAudio 回调输出到声卡。

**启动闸门**（`init_node` 之前执行，任一不过则进程退出，由 launch `respawn` 周期重检）：

1. **声卡检测**：无播音设备（如 5W 下位机）则退出，不注册任何服务/话题，避免抢占对机的 `/play_music`；
2. **唯一提供者**：`/play_music` 已被对机提供（`lookupService` + TCP 活性探测，能识别 kill -9 残留的陈旧注册）则退出。上下位机共享 master 时保证全网只有一个音频节点。

热插拔喇叭、对机节点死亡后的自动接管均由 respawn 重检自然覆盖。

## 2. 服务与话题接口

### 2.1 播放本地音频文件

#### 服务接口：`/play_music`
- **服务类型**：`kuavo_msgs/playmusic`
- **请求参数**：
  - `music_number` (str)：音频文件名（如 `test.wav`、`test.mp3`），需放在 `music_path` 指定的目录下
  - `volume` (int)：音量，范围 0–100。内部映射为 ffmpeg `volume` 滤镜的 dB 增益：`100` 为原声（0 dB），`0` 近静音（约 −60 dB），对数缩放避免线性乘法溢出破音
- **响应**：`success_flag` (bool)
- **语义**：追加到缓冲，按队列顺序播放，不打断当前

#### 服务接口：`/play_music_immediate`
- **服务类型**：`kuavo_msgs/PlayMusicImmediate`
- **请求参数**：同 `/play_music`
- **响应**：`success_flag` (bool)
- **语义**：原子化「打断当前 + 立即播新」。先同步停止当前播放（关流丢弃 ALSA DMA）再加载新文件

#### 示例调用：
```bash
# 追加播放（默认，排队）
rosservice call /play_music "music_number: '1_挥手.mp3'
volume: 80"

# 打断当前并立即播放新文件
rosservice call /play_music_immediate "music_number: '进入搬运模式可安全移动.wav'
volume: 80"
```

### 2.2 音频流播放接口

#### 话题：`/audio_data`
- **话题类型**：`std_msgs/Int16MultiArray`
- **数据格式**：
  - `data`：16 位有符号整数（INT16）PCM 采样数组
  - `layout.dim`：可选，通过 `dim.label="sample_rate"`、`dim.size=采样率` 指定源采样率
- **采样率**：默认 16000 Hz；若消息带 `sample_rate` 字段则按其重采样至声卡采样率，未带则按 16000 Hz 处理
- **过载策略**：缓冲溢出时丢弃积压旧帧，保留最新数据，保证实时语音（TTS）优先

#### 示例代码：
```python
from std_msgs.msg import Int16MultiArray, MultiArrayDimension

msg = Int16MultiArray()
msg.data = audio_chunk.tolist()  # audio_chunk: numpy int16 数组

# 非默认采样率时附带采样率信息
dim = MultiArrayDimension()
dim.label = "sample_rate"
dim.size = 44100
dim.stride = 1
msg.layout.dim.append(dim)

audio_publisher.publish(msg)
```

### 2.3 停止播放接口

停止播放提供服务与话题两种入口：

#### 服务：`/stop_music`（推荐）
- **服务类型**：`std_srvs/Trigger`
- **功能**：同步停止当前所有音频，返回时已静音
- **示例**：
```bash
rosservice call /stop_music
```

#### 话题：`/stop_music`（兼容，建议迁移到服务）
- **话题类型**：`std_msgs/Bool`
- **功能**：发送 `True` 停止播放；fire-and-forget，不返回停止结果
- **注意**：`/play_music` / `/play_music_immediate` 播放成功后的 0.3 秒内，通过话题到达的停止请求会被丢弃（避免「先停后播」调用序列中话题延迟造成的时序反转把刚加载的音频抹掉）。需要可靠停止请使用服务
- **示例**：
```bash
rostopic pub /stop_music std_msgs/Bool "data: true" -1
```

> 两种入口底层处理一致：置位 abort → 回调返回 `paAbort` 终止流 → 清空环形缓冲 → 关流丢弃 ALSA DMA 残留 → 重建流。回调式播放使停止即时生效，无残留音频。

### 2.4 状态查询接口

#### 服务：`/audio_status`
- **服务类型**：`kuavo_audio_player/audio_status`
- **响应**：`is_playing` (bool)，当前是否正在播放

#### 服务：`/get_used_audio_buffer_size`
- **服务类型**：`std_srvs/Trigger`
- **响应**：`message` (str)，当前环形缓冲剩余字节数

#### 话题：`/audio_playback_status`
- **话题类型**：`kuavo_msgs/AudioPlaybackStatus`
- **字段**：`header`、`playing` (bool)、`message` (str)、`buffer_size` (int32)
- **频率**：10 Hz 周期发布

## 3. 软件依赖

- `pyaudio`：音频回调播放（`sudo apt-get install python3-pyaudio -y`）
- `scipy`：音频重采样与处理（`pip install scipy`）
- `ffmpeg`：音频格式转换与音量缩放（`sudo apt-get install ffmpeg -y`）
- 依赖缺失时节点会尝试自动安装

## 4. 音频格式支持

- 支持 MP3、WAV、OGG、AAC、FLAC 等常见格式，由 ffmpeg 统一转码为 16 位 PCM
- 支持音量调节（dB 对数缩放）与实时重采样，适配不同声卡设备

> 音频格式支持基于 ffmpeg，请确保系统已安装

## 5. 使用流程

### 5.1 启动节点
```bash
roslaunch kuavo_audio_player play_music.launch
```

### 5.2 播放本地音频文件
```bash
rosservice call /play_music "music_number: '1_挥手.mp3'
volume: 80"
```

### 5.3 发布音频流测试
```bash
rosrun kuavo_audio_player audio_stream_test.py
```

### 5.4 停止当前播放
```bash
rosservice call /stop_music
```

## 注意
避免播放过大音频文件，否则会导致音频播放卡顿。
