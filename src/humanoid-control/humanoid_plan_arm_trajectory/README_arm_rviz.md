# humanoid_arm_trajectort_rviz 使用说明

## 功能简介

该 launch 脚本和配套 Python 脚本可实现：
- 加载指定 URDF 机器人模型
- 启动 robot_state_publisher、静态 TF、RViz
- 启动 arm_trajectory_bezier_rviz.py 脚本，将关节轨迹桥接到 joint_states 并可自动触发指定动作
- 支持通过参数灵活切换动作文件路径和名称

## 启动方法

1. **准备动作文件**
   - 将你的动作文件（如 `welcome.tact`）放入某个目录，例如 `humanoid_plan_arm_trajectory/script/action_files`。如果在其他路径，可通过launch参数指定。

2. **启动 launch**
   
   ```bash
   roslaunch humanoid_plan_arm_trajectory humanoid_arm_trajectort_rviz.launch \
     action_files_path:=/home/youruser/my_actions \
     auto_action_name:=welcome
   ```
   - `action_files_path`：指定动作文件目录
   - `auto_action_name`：指定启动后自动执行的动作（不带扩展名）


