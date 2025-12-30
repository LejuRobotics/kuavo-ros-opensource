# Websocket SDK 示例说明文档
> 按照下面文档操作以后，只要在局域网以内，就可以通过示例代码控制机器人，而不用通过ROS
## 启动 Websocket 

### Websocket 服务器启动

使用 Weboskcet SDK 需要在上位机先启动 Websocket 服务器：
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

### SDK 初始化

在使用SDK之前，需要先初始化 WebSocket 连接。初始化时需要指定以下参数：

```python
KuavoSDK.Init(
    websocket_mode=True,              # 启用 WebSocket 模式
    websocket_host='127.0.0.1'        # WebSocket 服务器IP地址，需要对应修改,也就是上位机IP
)
```

参数说明：
- `websocket_mode`: 是否启用WebSocket模式，必须设置为True
- `websocket_host`: WebSocket服务器地址，默认为'127.0.0.1'