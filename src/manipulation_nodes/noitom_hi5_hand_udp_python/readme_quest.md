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

# VR头部控制系统

- 配置文件位置：`src/manipulation_nodes/noitom_hi5_hand_udp_python/scripts/config.json`
- 支持四种控制模式：
  - `fixed`：固定模式，将头部yaw和pitch设置为0（正前方），平滑移动到目标位置
  - `auto_track_active`：自动跟踪主动手模式，自动检测并跟踪移动的手
  - `fixed_main_hand`：固定主手模式，跟踪指定的手（left/right）
  - `vr_follow`：VR随动模式，由VR设备直接控制
- 可配置参数：
  - `joint_limits`：头部关节角度限制（yaw/pitch，单位：度）
  - `smoothing_factor`：平滑滤波系数（0-1，越大越平滑）
  - `active_hand_threshold`：主动手检测阈值（单位：米，越小越敏感）
- 系统会自动从TF树获取头部和手部的实时位置进行计算，确保坐标系一致性
- **运行时动态切换模式（ROS服务）**：
  - 服务名称：`/quest3/set_head_control_mode`
  - 服务类型：`kuavo_msgs/SetHeadControlMode`
  - 调用方式：
    ```bash
    # 固定模式（将头部移动到正前方）
    rosservice call /quest3/set_head_control_mode "{mode: 'fixed', fixed_hand: ''}"
    
    # 自动跟踪主动手模式
    rosservice call /quest3/set_head_control_mode "{mode: 'auto_track_active', fixed_hand: ''}"
    
    # 固定主手模式（跟踪左手）- fixed_hand参数必需
    rosservice call /quest3/set_head_control_mode "{mode: 'fixed_main_hand', fixed_hand: 'left'}"
    
    # 固定主手模式（跟踪右手）- fixed_hand参数必需
    rosservice call /quest3/set_head_control_mode "{mode: 'fixed_main_hand', fixed_hand: 'right'}"
    
    # VR随动模式
    rosservice call /quest3/set_head_control_mode "{mode: 'vr_follow', fixed_hand: ''}"
    ```
  - 参数说明：
    - `mode`：控制模式字符串，必需参数
    - `fixed_hand`：固定主手模式时的手（"left" 或 "right"），仅在 `mode` 为 `"fixed_main_hand"` 时必需，其他模式可传入空字符串 `""`
  - 响应说明：
    - `success`：是否成功设置模式
    - `message`：操作结果消息
    - `current_mode`：当前生效的模式


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
