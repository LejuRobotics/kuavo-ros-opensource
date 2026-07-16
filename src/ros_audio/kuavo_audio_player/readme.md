# kuavo_audio_player 使用说明

## 1. 系统架构说明

kuavo_audio_player 按机型提供两种部署形态（`play_music.launch` 按 `ROBOT_VERSION` 自动分流）：

- **v17 — 单节点 `audio_player_node`**：提供 `/play_music` 服务播放本地音频文件，订阅 `/audio_data` 接收外部 PCM 流，统一经单环形缓冲 + 单 PortAudio 回调输出到声卡。
- **非 v17 — 旧两节点**：`loundspeaker.py`（文件服务）+ `audio_stream_player.py`（流播放）。

**接口原则**：旧接口定义冻结；新功能只通过新接口名提供，不修改旧定义。

## 2. 服务与话题接口

### 2.1 播放本地音频文件

#### 服务接口：`/play_music`
- **服务类型**：`kuavo_msgs/playmusic`（旧接口）
- **请求参数**：
  - `music_number` (str)：音频文件名（如 `test.wav`、`test.mp3`），需放在 `music_path` 指定的目录下
  - `volume` (int)：音量，范围 0–100。内部映射为 ffmpeg `volume` 滤镜的 dB 增益：`100` 为原声（0 dB），`0` 近静音（约 −60 dB），对数缩放避免线性乘法溢出破音
- **响应**：`success_flag` (bool)
- **语义**：追加到缓冲，按队列顺序播放，不打断当前

#### 服务接口：`/play_music_immediate`（新接口，仅单节点提供）
- **服务类型**：`kuavo_msgs/PlayMusicImmediate`
- **请求参数**：同 `/play_music`
- **响应**：`success_flag` (bool)
- **语义**：原子化「打断当前 + 立即播新」。先同步停止当前播放（关流丢弃 ALSA DMA）再加载新文件
- **兼容性**：旧两节点不提供此服务。调用方应探测其存在（`wait_for_service` 短超时 + 缓存），不存在时降级为 `/stop_music` 同步服务 → `/stop_music` topic → `/play_music`。

#### 示例调用：
```bash
# 追加播放（默认，排队）
rosservice call /play_music "music_number: '1_挥手.mp3'
volume: 80"

# 立即打断当前并播放新文件（搬运语音切换用，仅单节点机型）
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

停止播放提供服务与话题两种入口，均立即生效：

#### 服务：`/stop_music`（推荐）
- **服务类型**：`std_srvs/Trigger`
- **功能**：同步停止当前所有音频，返回时已静音
- **示例**：
```bash
rosservice call /stop_music
```

#### 话题：`/stop_music`（兼容，建议迁移到服务）
- **话题类型**：`std_msgs/Bool`
- **功能**：发送 `True` 立即停止；fire-and-forget，不返回停止结果
- **示例**：
```bash
rostopic pub /stop_music std_msgs/Bool "data: true" -1
```

> 两种入口底层都走「置位 abort → 回调返回 `paAbort` 终止流 → 清空环形缓冲 → 关流丢弃 ALSA DMA 尾巴 → 重建流」。旧实现用阻塞模式 `stream.write` 单次喂约 4s 音频块，停止只清软件队列、无法打断进行中的阻塞 write，停止后旧音频会持续播放至该块排尽（残留可达约 4 秒）；本节点改用回调模式，停止可即时打断，已修复。

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
