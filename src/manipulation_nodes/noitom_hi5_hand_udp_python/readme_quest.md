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

# 头部控制

头部指令由 `head_control_node` 发布到 `/robot_head_motion_data`。轮臂机器启动时默认拉起这个节点，Quest 节点不再发头角。

启动参数：

- `head_control_mode`，默认 `vr_follow`
  - `vr_follow`：跟随 Quest 骨骼 `Chest` 相对 `Head`。没有这组 TF 时不发布头角，头保持不动
  - `track_hand`：跟踪机器人自己的手，再由 `head_track_target` 指定目标
  - `fixed`：平滑回到正前方（yaw = 0，pitch = 0）
- `head_track_target`，仅 `track_hand` 使用，默认 `both_hands_midpoint`
  - `both_hands_midpoint`：双手连线。`scripts/config.json` 的 `hand_bias` 默认 0.5，即正中间；0 始终看左手，1 始终看右手。一只手更活跃时再按 `velocity_bias` 偏向它
  - `left_hand` / `right_hand`：固定跟踪指定手
  - `active_hand`：跟踪正在移动的手

手和头都在 `waist_yaw_link` 下取值。目标在头部后方时保持当前头角，回到前方至少 0.03 m 后继续跟随。

```bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch head_control_mode:=track_hand head_track_target:=both_hands_midpoint
```

不需要这个节点时传 `with_head_control_node:=false`。PICO 轮臂启动会关掉它，头改由 PICO 自己的节点发布。

运行时切换服务是 `/head_control_node/set_head_control_mode`（`kuavo_msgs/SetHeadControlMode`）：

```bash
rosservice call /head_control_node/set_head_control_mode "{mode: track_hand, fixed_hand: both_hands_midpoint}"
rosservice call /head_control_node/set_head_control_mode "{mode: track_hand, fixed_hand: right_hand}"
rosservice call /head_control_node/set_head_control_mode "{mode: left_hand, fixed_hand: ''}"
rosservice call /head_control_node/set_head_control_mode "{mode: fixed, fixed_hand: ''}"
rosservice call /head_control_node/set_head_control_mode "{mode: vr_follow, fixed_hand: ''}"
```

`mode` 为 `track_hand` 时，`fixed_hand` 是注视目标；留空保持当前目标。`vr_follow` 和 `fixed` 忽略 `fixed_hand`。返回的 `current_mode` 是实际生效的名字，例如 `both_hands_midpoint`，而不是笼统的 `track_hand`。

角度限制、平滑和换手阈值在 `scripts/config.json` 的 `head_control`。`smoothing_factor` 越小越平滑，越大越跟手。launch 传入的模式优先于配置文件。


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
