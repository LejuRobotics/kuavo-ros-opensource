# 相关资源

md5 c287aeb8a1df0f152fdc164d29ceb2fe https://kuavo.lejurobot.com/Quest_apks/leju_kuavo_hand-0.0.1-147-g85b5c38.apk

[Quest3 激活和安装程序说明](./docs/Quest3_激活和安装说明.md)

# 使用方法 

1. 在 Quest3 上安装 leju_kuavo_hand.apk
2. 查看 Quest3 设备的 IP
3. 运行 ROS 节点的机器需要和 Quest3 在同一个局域网内, 需要安装的依赖在 requirements.txt
4. 启动监听脚本，比如： python ./scripts/monitor_quest3.py 10.10.20.120 这个 IP 需要根据实际情况更新
5. 带上 Quest3 点击安装好的程序，启动。然后就会有一个 topic: `/leju_quest_bone_poses` 发布所有关节的信息。这些信息是一个序列，顺序请参考: bone_name_to_index

# Motion Capture IK 使用方法

闭源：
[使用方法](https://www.lejuhub.com/highlydynamic/motion_capture_ik/-/blob/develop/README.md)

已经打包好：
[使用方法](https://www.lejuhub.com/highlydynamic/motion_capture_ik_packaged/-/blob/develop/README.md)

# ROSBAG 工具

添加工具用于录制和回放VR手臂数据和相机数据。

```bash
python3 scripts/rosbag_tool.py

# 当前录制的话题如下
# "record_topics": [
#   "/kuavo_arm_traj",
#   "/control_robot_hand_position",
#   "/robot_head_motion_data",
#   "/camera/depth/image_rect_raw",
#   "/camera/depth/camera_info",
#   "/camera/depth/color/points",
#   "/camera/color/image_raw",
#   "/camera/color/camera_info",
#   "/camera/aligned_depth_to_color/image_raw",
#   "/camera/aligned_depth_to_color/camera_info"
# ],
```

配置录制话题有哪些的 json 文件在 `scripts/record_topics.json` 中。

# 更新记录

1. 添加头和脖子的 Bone 的数据
2. 添加了两个手柄的数据
