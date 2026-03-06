# VR录制接口文档

## WebSocket接口

### 1. 获取VR状态 - `get_vr_status`

**请求**：
```json
{
  "cmd": "get_vr_status",
  "data": {}
}
```

**响应**：
```json
{
  "cmd": "get_vr_status",
  "data": {
    "code": 0,
    "vr_connected": true,
    "vr_nodes_running": true,
    "vr_state": "connected",
    "recording_state": "idle",
    "recording_duration": null
  }
}
```

---

### 2. 开始录制 - `start_vr_record`

**请求**：
```json
{
  "cmd": "start_vr_record",
  "data": {}
}
```

**响应**：
```json
{
  "cmd": "start_vr_record",
  "data": {
    "code": 0,
    "message": "Recording started"
  }
}
```

---

### 3. 停止录制并转换 - `stop_vr_record`

**请求**：
```json
{
  "cmd": "stop_vr_record",
  "data": {
    "tact_filename": "my_action"
  }
}
```

**响应**：
```json
{
  "cmd": "stop_vr_record",
  "data": {
    "code": 0,
    "message": "Recording stopped, converting to my_action.tact"
  }
}
```

---

### 4. 取消录制 - `cancel_vr_record`

**请求**：
```json
{
  "cmd": "cancel_vr_record",
  "data": {}
}
```

**响应**：
```json
{
  "cmd": "cancel_vr_record",
  "data": {
    "code": 0,
    "message": "Recording cancelled successfully"
  }
}
```

---

## 辅助接口

### 获取机器人信息 - `get_robot_info`

**响应**：
```json
{
  "cmd": "get_robot_info",
  "data": {
    "code": 0,
    "robot_type": "45",
    "vr_recording_path": "~/.config/lejuconfig/vr_recordings",
    "workspace_setup_path": "/kuavo-ros-control/devel/setup.bash"
  }
}
```