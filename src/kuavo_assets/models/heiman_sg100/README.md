# 黑漫 SG100 手部模型

`410062` / `410063` 为采用 SG100 V4 手的轮臂版本，分别基于
`400062` / `400063` 增加；原版本和本目录原有 V3 资源继续保留。

V4 资源位于 `v4/meshes` 和 `v4/urdf`，来自供应商的
`SG100V4.0_L_3D_URDF` / `SG100V4.0_R_3D_URDF`。导入时给左右手的
link/joint 添加 `l_` / `r_` 前缀，将 `base_link` 改为 `l_hand_base` /
`r_hand_base`，并改写网格路径。网格文件内容保持供应商原样。
`v4/source_manifest.json` 记录源 URDF 及网格 SHA-256，便于追溯。
手部 URDF 是与原目录相同形式的 XML 片段，需要嵌入机器人 `<robot>` 中使用。

新版本主 URDF、Gazebo URDF、Drake 简化 URDF 和 MuJoCo XML 同步使用 V4
的质量、质心、完整惯量、关节变换、轴和限位。Drake 保持原来的固定手指关节及
碰撞简化方式。MuJoCo 使用 `fullinertia` 保留惯量非对角项。左右手质量分别为
0.2808614 kg 和 0.28133254558633874 kg。

腕部安装关节继续采用原轮臂模型的安装变换，关节与末端帧名称保持兼容。
新版本的 `meshes` 是指向对应旧版本身体网格的相对符号链接，V4 手网格显式引用
本目录的 `v4/meshes`；复制/打包模型时应保留符号链接及其目标目录。
新版本副本中同时修正了继承的 Drake 网格文件名和 Gazebo 启动路径。

启动时选择完整版本号，例如：

```bash
export ROBOT_VERSION=410062  # S63 使用 410063
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch
```

版本注册表的显示键直接使用 `410062` / `410063`，避免指定未经确认的新显示别名。
版本数字的通用解析规则未更改；轮臂运行入口对新 ID 显式兼容。

验证命令（不启动硬件）：

```bash
python3 -m unittest discover -s src/kuavo_assets/tests -v
# 以下检查需要 source /opt/ros/noetic/setup.bash 和工作空间 devel/setup.bash
python3 -m unittest discover -s src/humanoid-control/humanoid_plan_arm_trajectory/test -v
roslaunch --dump-params humanoid_controllers load_kuavo_real_wheel.launch robot_version:=410062
roslaunch --dump-params humanoid_controllers load_kuavo_mujoco_sim_wheel.launch robot_version:=410063
```

模型测试检查网格哈希、URDF 树与资源、各格式手部参数、身体惯性及原关节兼容，
并在安装 MuJoCo Python 绑定时核对零位/非零位手部 FK、编译后的完整惯量和短步仿真。
实机安装方向、末端工具帧标定与运动效果仍需在对应 V4 硬件上确认。

## 遥操作版本适配

Quest 单机、双机和视频回传入口将 `400062/400063/410062/410063` 的
默认末端类型设为 `heiman`，可用 `ee_type:=...` 显式覆盖。新版本默认启用轮臂
IK；C++ 增量模式的 `/end_effector_type` 和 `/control_finger_type` 在全局设置，
与手部控制器读取路径一致。绝对式速度 IK 使用新版本自己的完整模型与配置，
并保留各入口既有的轮臂限速；不继承旧机型专用的放宽容差。
更新绝对式控制器的版本校验后，需要重新编译 `wheel_arm_abs_vel` 才能使二进制生效。

仅解析启动配置的回归测试（先加载 ROS 和当前工作空间环境）：

```bash
python3 -m unittest discover -s src/manipulation_nodes/noitom_hi5_hand_udp_python/tests -p test_heiman_v4_launch.py -v
```
