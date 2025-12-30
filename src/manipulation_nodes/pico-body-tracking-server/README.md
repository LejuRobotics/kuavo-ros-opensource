# Pico-Body-Tracking-Server


## Getting started

1. 运行服务器

先运行 roscore
```bash
roscore
```

再运行服务器
```bash
python3 body_tracking_server.py
```

2. 启动 PICO VR 中 **Pico-Body-Tracking-Demo** 项目

    2.1 按照指示校准传感器（如果校准完进入APP后没有任何UI反馈，请重新进入 **Pico-Body-Tracking-Demo** APP）

    2.2 进入 **Pico-Body-racking-Demo** APP 后，在左侧 UI 在线列表选择已上线的机器人服务器

    2.3 点击左侧 UI 中的 **连接** 按钮

    2.4 连接后，点击左侧 UI 中的 **开始遥操作** 按钮，此时 APP 会把 full-body-tracking-pose 发送到机器人服务器


3. 可视化

```bash
rviz
```

启动 `rviz` 后，更改 `Global Options` 中的 `Fixed Frame` 为 `world`，然后添加 `TF` 选项，就可以看到 full-body-tracking-pose **的实时可视化**

## full-body-tracking-pose 说明


人体参考图：

![human-body-bone](./assets/imgs/human-body-bone.png)


full-body-tracking-pose 的骨骼节点说明：
```python
body_tracker_role = [    
    "Pelvis",
    "LEFT_HIP",
    "RIGHT_HIP",
    "SPINE1",
    "LEFT_KNEE",
    "RIGHT_KNEE",
    "SPINE2",
    "LEFT_ANKLE",
    "RIGHT_ANKLE",
    "SPINE3",
    "LEFT_FOOT",
    "RIGHT_FOOT",
    "NECK",
    "LEFT_COLLAR",
    "RIGHT_COLLAR",
    "HEAD",
    "LEFT_SHOULDER",
    "RIGHT_SHOULDER",
    "LEFT_ELBOW",
    "RIGHT_ELBOW",
    "LEFT_WRIST",
    "RIGHT_WRIST",
    "LEFT_HAND",
    "RIGHT_HAND",
]
```



