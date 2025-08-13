# Beta

## Breaking Changes

## 文档相关

## 新增功能

## 修复问题

## 其他改进

# 1.2.1

## Breaking Changes
- 无

## 文档相关
- 重新调整运动控制文档结构并删除无用接口，[新的文档链接](./docs/运动控制API.md)
- 新增 Kuavo Humanoid SDK 原子技能 v2 版搬箱子策略模块文档，[文档链接](./src/kuavo_humanoid_sdk/docs/markdown/pages/kuavo_strategy_v2.md)
- 新增运动学 MPC VR 遥操作使用文档， [文档链接](./src/manipulation_nodes/motion_capture_ik/README_VR_MPC.md)
- 上楼梯案例适配奥比中光相机以及使用文档，[文档链接](./src/humanoid-control/humanoid_controllers/scripts/上楼梯案例说明.md)
- 新增行为树搬箱子 README 说明文档，[文档链接](./src/demo/grab_box/README.md)
- Kuavo Humanoid SDK 更新使用文档描述搬箱子策略模块需要关闭 `basePitchLimits`，[文档链接](./src/kuavo_humanoid_sdk/README.md)
- Kuavo Humanoid SDK 更新使用示例和搬箱子案例文档中文描述，[文档链接](./src/kuavo_humanoid_sdk/README.md)
- Kuavo Humanoid SDK 更新所有使用文档为中文格式，[文档链接](./src/kuavo_humanoid_sdk/README.md)
- 示教功能文档补充说明示教模式启动机器人需要按‘o’后才开始记录，[文档链接](./src/manipulation_nodes/teach_pendant/readme.md)
- 运动控制 API 文档新增电机Kp/Kd 参数 ROS 服务接口描述，[文档链接](./docs/运动控制API.md)
- Kuavo Humanoid SDK 增加搬箱子策略模块使用文档，[文档链接](./src/kuavo_humanoid_sdk/README.md)
- 增加日志上传 coScene 工具使用文档，[文档链接](./tools/upload_log/doc/readme.md)

## 新增功能
- Kuavo Humanoid SDK 在原子技能的手部运动控制同步接口上增加碰撞保护的功能, [使用示例](./src/kuavo_humanoid_sdk/examples/atomic_skills/ctrl_arm_example_protected.py)
- 运动学 KMPC 新增 play_back 回放功能，[文档链接](./src/humanoid-control/mobile_manipulator_controllers/scripts/how_to_play_back.md)
- 新增运动学 MPC 的异常检测机制
- 45 和 49 版本机器人 URDF 新增手腕相机模型
- 新增关节保护功能，使用电机原始的速度峰值和扭矩峰值进行数据拦截，避免电机异常运动损坏
- 新增 VR 运动学 mpc 增量式遥操功能，启动launch文件见 [链接](./src/manipulation_nodes/motion_capture_ik/launch/kinematic_mpc_vr_incremental_real.launch)
- 新增手臂碰撞检测模块，VR 外部控制时检测到碰撞回到手臂 3秒前的状态， [文档链接](./src/kuavo_arm_collision_check/readme.md)
- 新增 H12 遥控器支持控制头部关节，按键文档见 [文档](./docs/5功能案例/通用案例/H12遥控器使用开发案例.md)
- 运动学 MPC 增加末端轨迹的输入接口、mpc mrt 求解器重置服务
- 新增置速度和扭矩超过峰值截断保护功能
- 一键标定零点脚本工具增加对相机(realsense，奥比中光)和 IP 的适配，[文档链接](./scripts/joint_cali/README_One_button_start.md)
- RUIWO CXX SDK 支持 kp kd 参数动态控制
- 新增 rosbag 扭矩转化工具，读取 rosbag 中的 /sensors_data_raw 话题，将电流值转换为扭矩值，[工具链接](tools/bag_tools/current_to_torque_converter.py)
- Kuavo Humanoid SDK 新增 v2 搬箱子策略模块，基于事件模型提供闭环的策略编排控制，[使用文档](./src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_v2/README.md)
- 新增上楼梯功能案例，[使用文档](src/humanoid-control/humanoid_controllers/scripts/上楼梯案例说明.md)
- 新增 ROS 服务接口 `/humanoid_mpc_gait_change`用于步态切换, 支持 `walk` 和 `stance`
- 新增 VR 可以控制头部运动，默认开启头部控制模式，可通过`enable_head_control:=false`关闭
- 新增手臂碰撞检测功能包 `kuavo_arm_collision_check`，用于检测手臂碰撞
- 非触觉灵巧手新增 ROS 控制话题:`/dexhand/command`, `/dexhand/left/command`, `/dexhand/right/command`, 支持位置和速度控制模式， [详细文档](./docs/运动控制API.md)
- 非触觉灵巧手新增更改握力级别 ROS 服务接口:`/dexhand/change_force_level` `, [详细文档](./docs/运动控制API.md)
- 新增非触觉灵巧手 Protobuf SDK 替换旧版 hand_sdk 支持位置/速度控制，握力级别变更，获取手指真实状态等接口 
- Kuavo Humanoid SDK 原子技能增加 BasePitchLimits 开启/关闭和获取状态接口，避免执行搬箱子策略过程中触发保护而摔倒
- Kuavo Humanoid SDK Websocket 版新增 SDK 启动并站立和停止机器人功能
- Kuavo Humanoid SDK 新增搬箱子策略模块测试案例， gazebo 仿真场景默认开启灯光
- Kuavo Humanoid SDK 原子技能接口增加末端力控制，搬运箱子策略模块放下箱子后添加安全后退保护，搬运箱子案例添加末端力控制夹紧箱子
- 新增数据截断和关节保护使能参数,默认不启动
- 新增电机跟随性测试工具，[使用文档](./tools/check_tool/selfCheckScripts/README.md)
- 新增太极，舞蹈动作案例以及全身控制接口，案例[使用文档](./src/demo/full_body_demo/readme.md)
- Kuavo Humanoid SDK websocket 版新增适配视觉模块，使用方法与非websocket版本一致
- Kuavo Humanoid SDK 原子策略模块抓取相关策略添加安全保护模组
- 新增关节保护数据截断功能，扭矩峰值保护从瞬时修改为时间窗口触发
- 新增视觉零点标定工具，[使用文档链接](./scripts/joint_cali/README_One_button_start.md)
- Kuavo Humanoid SDK 拆分 websocket 为单独模块 Kuavo Humanoid Websocket SDK，原先 SDK 将不支持在 Windows 系统下运行
- 新增机器人 collision 碰撞对，实现横向移动保护
- Kuavo Humanoid SDK 新增修改和获取电机Kp/Kd 参数接口以及使用示例，目前仅支持`youda`驱动类型的电机
- 新增修改和获取电机Kp/Kd 参数 ROS 服务接口，目前仅支持`youda`驱动类型的电机
- Kuavo Humanoid SDK 新增搬箱子策略模块以及 gazebo 仿真策略使用示例

## 修复问题
- 修复 KMPC 控制模式切换到 None 之外的其他模式时，收到 `mm/two_hand` 的话题数据不执行的问题
- 修复不安全的多线程访问数据可能会导致段错误而引起机器摔倒的问题
- 修复硬件自检脚本无法绘图的问题，原因是保存数据中索引越界
- 运动学 MPC 功能包中对不必要的ros包的依赖以及添加误差状态四元数卡尔曼滤波器
- 修复 VR 遥操作时，手臂弯曲的问题
- 修复 pick_and_place 案例功能包，右臂进行抓取时, 左臂动作异常问题
- 修复音频模块安装 samplerate 库失败的问题
- 修复 VR 控制手臂动作过快可能导致摔倒的问题，在VR控制时关闭 pullup 保护
- 修复 `/cmd_pose_world` 接口中单步抽搐会导致机器摔倒问题
- 修复初始化和运行过程中的手柄的型号发生改变无法控制机器人问题
- 修复半身模式手臂 VR 跟踪时无法回正
- 修复 TF 树中 odom->base_link 时间戳为 0 和 base link 相对于 dummy link 平移和旋转不为 0 的问题 
- 修复示教功能中小臂不会动的问题
- 修复手臂和头部电机获取速度接口未赋值就返回导致速度值一直为 0 的问题
- Kuavo Humanoid SDK 修复在非原点处执行`ctrl_arm_example.py`示例手臂会异常问题，原因是示例中展示的运动学控制坐标系为 odom 系，更正示例为局部系
- 修复所有版本机器人的 gazebo urdf 中 friction 参数错误问题，更正为 0.0 防止手臂控制有静差
- 修复 pick_and_place 功能包, 运行时无法控制灵巧手问题
- 修复半身VR XA切换控制模式手臂不插值，导致机器人手臂抽搐
- 修复 h12 遥控器以及 root 用户启动时无法收集到 git 仓库信息问题
- 修复行为树搬箱子案例中，在控制手臂的情况下，初始状态会出现抖动的问题
- Kuavo Humanoid SDK 修复音频模块的消息格式错误问题
- 修复半身/轮臂机器人会触发 WBC pullup 问题
- 修复手臂轨迹规划demo不能控制手指和头部关节
- 修复播放音频的时候流式播放和音频文件播放设备占用冲突的问题
- 修复 humanoid_estimation 包中 ymal 配置缺失问题
- 修复 v47-v49 机器人无法使用遥操作的问题
- 修复 cmd_pose_world 接口时转弯出现躯干不动的现象
- 修复 mujoco 仿真关节名称不存在时计算关节组大小错误问题
- 增加运动学 yaw 滤波以修复 IMU 偏航角偏移问题
- 修复 kuavo.json 中头部错误的位置阈值
- kuavo Humanoid SDK 搬箱子策略修复下蹲情况下抓箱子位置错误的问题, 抓取动作全部由局部系改为世界系
- 修复 VR 手柄无法控制灵巧手手指的问题
- 修复 h12 自启服务启动时贝塞尔插值器获取参数失败问题
- 增加运动学 yaw 滤波，修复 imu yaw 角度漂移问题
- 修复桌面软件自启动服务存在`urdfFile`参数配置找不到的问题
- 修复桌面软件连接机器人热点后，无法显示机器人 IP 地址和无法连接问题
- 修复桌面软件上传文件存放路径错误的问题，存放路径为 `$HOME/.config/lejuconfig/action_files`
- 修复半身控制模式下，启动机器人时手臂突然抽搐的错误
- 修复触觉手控制器初始化两遍的错误，可能会影响灵巧手控制

## 其他改进
- 添加工具类用于获取头和手腕相机到机器人底座的位置和转换，[工具使用文档](./tools/extract_camera_pose/howto-kuavo-pose-calculator.md)
- 更新手臂末端三个电机和腿部的飞车保护阈值
- VR 机器人 VR 节点新增广播机器人信息用于等待 VR 头盔连接
- 增加可视化运动学 MPC 补充双手末端执行器的轨迹姿态
- 新增 USB WIFI 信号稳定性测试工具和 VR 传输数据用量测试工具， [使用文档](./tools/wifi_benchmark/README.md)
- 增加时频域分析脚本，用于分析手臂末端的运动跟踪性能，从时域和频域的角度定量的分析跟踪效果、各环节间的延迟
- 优化上下位机 DHCP 配置工具，默认下位机为DHCP分配主机并增加声卡 udev 规则配置
- 取消 MobileManipulatorController 启动时一直打印等待`com_height`参数的日志
- 取消 MobileManipulatorController 节点启动时一直刷屏提示等待接收观测数据

# 1.1.5

## Breaking Changes
- 迁移`motion_capture_ik`包中的定义的消息到`kuavo_msgs`中统一维护

## 文档相关
- VR 遥操作增加如何查看 Quest3 和 Kuavo 的网络延迟的视频， [文档链接](./docs/Quest3_VR_basic.md)
- [kuavo humanoid sdk] 更新安装文档和 API 文档，详情见 [文档链接](./src/kuavo_humanoid_sdk/README.md)

## 新增功能
- 新增一键启动 Kuavo Humanoid SDK 依赖的所有功能包启动文件
- 单步控制接口增加支持完全指定腾空相轨迹功能
- Kuavo Humanoid SDK 新增 KuavoRobotObservation 类，用于获取机器人当前的控制指令
- Kuavo Humanoid SDK 新增 websocket 模式使用示例以及适配音频模块接口
- Kuavo Humanoid SDK 原子技能新增 运动学 MPC 接口，找 tag 和 接近 tag 策略模块，搬箱子基础框架
- Ruiwo 电机新增 C++ SDK 控制并支持参数切换C++/Python SDK，可通过参数`rui_cxx_sdk`指定，默认值为`true`
- 硬件节点默认打开关节保护检测功能，并取消触发关节保护之后的动作
- [kuavo humanoid sdk] 新增视觉接口和工具类接口，详情见[使用案例](./src/kuavo_humanoid_sdk/examples/vision_robot_example.py)
- 新增机器人 weboscket 节点配置成开机启动的 systemctl 服务
- 新增版本号为 49 的机器人模型， 特点相机模型为 orbbec 
- 下位机胸部 NUC 增加音频播放 ROS 服务
- 新增一键自检功能：IMU硬件连接，手臂电机通讯响应，灵巧手抓握测试，遥控器硬件信号，上下位机通信检查，相机/雷达/音响检查, [文档链接](tools/check_tool/selfCheckScripts/README.md)
- [kuavo humanoid sdk] 不依赖下位机的环境和kuavo_msgs消息包, 支持安装在上位机运行，[最新版本](https://pypi.org/project/kuavo-humanoid-sdk/1.1.2a924/) 
- [kuavo humanoid sdk] 启动时只检测依赖的节点是否存在，如果不存在则提示用户启动
- [kuavo humanoid sdk] 支持控制触觉灵巧手以及获取对应的触觉状态
- 路径跟踪案例添加`/cmd_pose`控制机器人跟踪路径，见[参考链接](/src/demo/trace_path/scripts/trace_path/mpc_client_example.py)
- 新增站立时上拉保护，拉起之后，切换到挂起模式保持不动
- 新增下蹲离地保护功能，下蹲到一定程度自动切换到挂起模式，即使没有按back退出也始终保持姿态
- 新增挂起模式，通过监测总接触力挂起模式，使用进入挂起模式的初始位置作为关节参考
- 站立保护功能支持站立失败缩腿后重新使用键盘或北通遥控器重复尝试站立

## 修复问题
- 修复由于触觉手 SDK 存在的非法越界访问内存错误导致的程序崩溃问题，已更新 SDK 版本到最新的`0.4.4`
- 修复手眼标定功能包及手腕相机抓取功能包的部分问题
- 修复 Kuavo Humanoid SDK 增加音频视觉模块后导致的不兼容的问题
- 修复对贝塞尔曲线插值器得出的关节角度值未进行限位处理导致的插值结果存在错误的控制命令问题
- 修复 h12 遥控器控制机器人站立下蹲等动作的阈值，防止出现摔倒等情况
- 修复 CTRL+C 等方式退出程序时，手臂电机未正常解锁问题
- 修复示教模式下未关闭站立保护功能的错误
- 修复命令启动机器人程序时，h12 遥控器控制逻辑和开机自启动方式不一致问题
- 修复 `/cmd_pose_world`话题存在无法导致重新回到原点的问题
- 修复执行手臂电机辨识相序脚本报错找不到文件的问题
- 修复 4pro 进阶版机器人半身控制时，H12播放手臂动作异常问题
- 修复末端为夹爪时程序启动崩溃的问题，原因是：初始化夹爪 ROS timer 顺序比硬件夹爪资源要早，导致夹爪 timer callback 访问了非法的资源
- 修复 HardwareTool.py 工具校准程序报错，手臂电机设置零点时提示找不到相关文件问题
- 修复 ROBOT_VERSION 为 49 的 kuavo.json 配置文件缺少电流保护相关的配置`joint_current_limits`字段
- 修复 websocket 广播机器人信息接口缺失`robot_action_file_folder`字段
- 修复手眼标定 launch 文件中相机话题名称错误的问题
- 修复终端日志频繁刷音频节点检测不到设备的问题
- 修复 h12 遥控器开机自启动和使用终端启动（joystick_type=h12）存在不一致的问题 
- 修复 ROBOT_VERSION 为 49 时缺少一些配置文件错误
- 改进关节保护手部速度阈值以及关节保护电机力矩阈值
- 改进关节保护堵转检测时间窗口从 0.1 增加到 0.2
- 修复机器人半身状态下/轮臂机器人初始手臂不弯曲
- 修复仿真和实物的键盘控制，请求原地旋转(j or l)10%无法执行的问题
- 修复当前机器人不支持步态设置连续的单腿浮空状态的问题
- 修复isaac-sim仿真环境未使用最新的`sensordata_msg`导致的程序启动失败问题
- 修复正常退出机器人程序手臂电机不解锁和 rosbag 包出现 active 的问题
- 修复关节保护模式下，现行腿部控制方式可能导致的电机堵转问题
- 修复 48 版本 kuavo.json 配置文件缺少`joint_current_limits`导致无法启动程序问题

## 其他改进
- 硬件检查工具支持扫描和测试触觉灵巧手功能，使用文档见 [文档链接](./tools/check_tool/readme.md)
- 更换音频播放节点 pip 源为 aliyun 避免安装等待过长时间
- 仿真环境时跳过机器人自动蹲起站立过程
- 新增 KuavoCrashReport 工具，用于收集机器人故障信息，并上传反馈给乐聚人员，[使用文档链接](./tools/crash-report/README.md)
- 升级了 Quest3 的 APK 程序 //kuavo.lejurobot.com/Quest_apks/leju_kuavo_hand-0.0.1-147-g85b5c38.apk ，显示出识别的骨骼效果，能方便的查看到 Quest3 识别出错的情况。以及更新 Meta SDK 到 0.74 改进识别的稳定性。

# 1.1.2
## Breaking Changes
- 无

## 文档相关
- 新增 NVIDIA Jetson AGX Orin 镜像备份和还原指南文档， [文档链接](./docs_internal/NVIDIA_Jetson_AGX_Orin_backup_restore_tutor/NVIDIA%20Jetson%20AGX%20Orin%20backup%20restore%20tutor.md)
- 新增上下位机 NUC DHCP 分配 IP配置说明文档，[文档链接](./docs/others/SET_HEAD_DHCP/头部%20NUC%20DHCP配置说明.md)
- 新增 ROS_MASTER_URI 主从配置工具文档，[文档链接](./docs/others/CHANGE_ROS_MASTER_URI/修改ROS_MASTER_URI说明.md)
- 更新运动控制接口文档，新增触觉灵巧手控制和状态话题，[文档链接](./docs/运动控制API.md)
- 新增 kuavo-humanoid-sdk 文档， [文档链接](src/kuavo_humanoid_sdk/docs/markdown/index.md)
- Kuavo 文档中心新增轮臂机器人介绍, 导航案例, 正逆解案例使用说明
- 更新 README 中全身控制器参数与 kuavo 配置参数的说明, [文档链接](./docs/info文件说明.md), [文档链接](./docs/kuavo_json文档说明.md)
- 运动控制接口新增乐聚自研夹爪控制接口`/control_robot_leju_claw`, 使用方法见 [文档链接](./docs/运动控制API.md)
- 新增 kuavo IK 正逆解模块使用说明, [文档链接](./src/manipulation_nodes/motion_capture_ik/how-to-use-kuavo-ik.md)
- 补充`/joint_cmd`控制话题中 control_mode 参数详细描述 [文档链接](./docs/运动控制API.md)
- 更新如何实时查看到 Quest3 投屏的屏幕使用文档, [文档链接](./docs/Quest3_VR_basic.md)
- 🎉🎉🎉 : 新增 Kuavo 产品介绍, 快速开始, 开发接口,功能案例等文档, [内测版文档网站链接](https://kuavo.lejurobot.com/beta_manual/basic_usage/kuavo-ros-control/docs/1%e4%ba%a7%e5%93%81%e4%bb%8b%e7%bb%8d/%e4%ba%a7%e5%93%81%e4%bb%8b%e7%bb%8d/index.html), [正式版文档网站链接](https://kuavo.lejurobot.com/manual/basic_usage/kuavo-ros-control/docs/1%e4%ba%a7%e5%93%81%e4%bb%8b%e7%bb%8d/%e4%ba%a7%e5%93%81%e4%bb%8b%e7%bb%8d/index.html)
- 补充运动控制接口文档中`/sensor_data_raw`话题的详细说明和数据示例 [文档链接](./docs/运动控制API.md)

## 新增功能
- 新增 isaac-sim 仿真环境, 使用说明请参考[文档链接](./src/kuavo-isaac-sim/README.md)
- 新增触觉灵巧手控制话题`/dexhand/command`和左右单独控制话题`/dexhand/left_command`和`/dexhand/right_command`，以及 `/dexhand/touch_state`话题, 并兼容原先非触觉手的ROS接口
- 新增触觉灵巧手硬件识别与测试工具，工具路径:`tools/check_tool/touch_dexhand_test.sh`
- 新增触觉灵巧手功能，末端类型为`qiangnao_touch`，在kuavo.json中修改 EndEffectorType 字段即可使用
- 新增灵巧手状态话题接口`/dexhand/state`
- 新增机器人初始化站立时的检测保护功能，在一定情况下防止机器人在未站立或站立异常时的抽搐
- 新增 kuavo-humanoid-sdk 应用层 SDK 用于控制机器人，pypi 项目：https://pypi.org/project/kuavo-humanoid-sdk
- 新增接收 Pico 发布脚部位置的节点，待算法部接入遥操作
- 新增 ROS话题`leju_claw_command`用于控制自研二指夹爪
- 新增夹爪手臂机器人模型，版本为47
- 新增一键安装脚本,可在刷完镜像的机器人上通过 wget 下载并执行完成自动安装环境等操作
- 新增实现通过按键控制手臂末端逆解和躯干逆解的案例，[文档链接](./docs/5功能案例/通用案例/按键控制躯干逆解.md)
- 完善URDF模型中的camera_base说明并添加全局静态TF转换以连接odom与像素坐标系，并新增元数据虚拟相机帧数据发布
- 新增机器人启动时无需人工搀扶辅助，可从双脚悬空或贴地状态切到站立状态的功能
- 新增kuavo_assets 各个版本机器人可视化的 launch 文件和 rviz 配置
- Gazebo 仿真器支持 Realsense 实感摄像头
- 新增支持 Gazebo 仿真器和基于共享内存的中间件读写 ROS 控制接口以减低通信延迟
- 新增机器人搬箱子应用案例
- 新增足部末端高度可调节功能
- 新增单步大转向+正常走，不平地面和斜坡行走功能
- 话题`/cmd_pose`和`/cmd_pose_world` 支持控制躯干 pitch
- 新增话题`/humanoid/single_step_mode` 用于发布机器人当前是否为单步控制状态
- VR 功能支持在结束录制和启动的时候可以指定手臂的关节位置
- 更新 kuavo_assets 包中机器人模型文件, 统一命名关节名称并添加关节限位, 扭矩限制, 速度限制等约束
- kuavo_sdk 新增正逆解使用说明, 单步控制使用说明, 位姿控制使用说明
- VR: Quest3 末端执行器支持乐聚自研夹爪, 可通过手柄上扳机或食指捏合控制夹爪
- 末端执行器支持乐聚自研夹爪, 可通过修改 kuavo.json 配置生效, 控制接口`/control_robot_leju_claw`
- 新增一些机器人校准动作方便在校准模式下检查电机是否正常
- VR: 新增 Quest3/Vision Pro 视频流功能
- 支持手柄控制头部运动, RT+左摇杆控制头部
- 支持通过修改 kuavo.json 配置文件`only_half_up_body`为 true 只使能上半身电机, 并适配了半身轮臂机器人, 使用见 [REAME文档](./readme.md)
- 工具: 添加支持同时开启热点和连接WIFI工具, WIFI名称`$ROBOT_NAME的热点`, 密码`kuavo123456`[使用文档链接](./tools/linux_wifi_hotspot/readme.md)
- IK 服务增加工作空间检查和可选打印求解信息提示

## 修复问题
- 修复潜在的内存非法访问导致的段错误程序崩溃问题
- 修复只使用上半身和轮臂机器人手抖问题
- 修复 VNC 无法切换 WIFI 问题，解决方法见[文档](./docs/6常用工具/修复VNC无法切换wifi的问题.md)
- 修复播报 WIFI IP 地址工具安装脚本的提示消息错误
- 修复使用 VR 时需要额外添加末端执行器类型的冗余操作
- 修复 system info 节点未初始化就发布话题数据的问题
- 修复手臂电机CAN通信频率为250Hz
- 修复手臂控制硬件层存在问题，降低控制频率防止手臂指令阻塞
- 修复路径跟踪示例到达终点后会超出一些距离问题，已通过到达终点前缓慢降低速度解决
- 修复`leju_claw_state` 夹爪状态话题消息无消息头时间戳问题
- 修复非 v42 版本机器人站立时存在抖动问题
- 新增长手臂电机配置文件 long_arm_config.yaml
- 修复地面高度估计对单步控制的影响导致摔倒问题
- 修复`/cmd_pose`控制指令的自动启停触发问题，现在可以正确触发启停并支持被`/cmd_vel`指令覆盖
- 修复多次单步控制后容易摔倒的问题, 已通过使用规划值作为 init_target_state 的起点来解决
- 修复在一些电机出问题的机器上启动时存在电机异响的问题, 通过增大站立控制器的关节跟踪权重来避免异响
- 修复 42 版本行走时躯干 pitch limit 晃动幅度变大问题
- 修复单步控制模式下未对高度进行补偿导致的高度变化问题
- 修复 Gazebo 仿真器中相机数据格式不完整问题, 补充使用仿真时间戳, 相机内参矩阵, 畸变参数和旋转矩阵等信息
- 修复 45 版本机器人由于 mesh 文件路径错误导致 VR 启动报错, 无法遥操作控制手臂问题
- 修复 autogait 阈值和键盘控制 10%的行程重合导致行走和站立频繁切换的问题
- 修复后退时对斜面的处理导致踩脚,增加斜坡规划生效的阈值
- 修复在半身模式下使用 VR 推摇杆会导致程序挂掉问题
- 依据实体测量数据更新 KUAVO 4Pro 总质量配置，实现仿真系统与物理实体参数一致性
- 修复机械臂关节扭矩反馈系统的电流-扭矩转换(C2T)系数校准错误
- 修复 h12pro 遥控器没有正确加载`~/.bashrc`中定义的 ROS 环境变量的问题
- 修复机器人站立瞬间容易受外力影响导致的瞬间异响电流问题
- 修复 43 版本的 URDF mesh 文件路径错误
- 修复在 40，41等版本只使用上半身功能是手臂抽搐问题
- 修复由于 kuavo_assets 包中 drake 相关的 URDF 文件的 mesh 文件名称不对，导致无法正常使用 VR 和 IK 功能
- 修复机器人原地踏步抽搐无法行走，VR无法启动跟随手势的问题
- 修复 URDF 更改未重新编译 cppad 导致异常不生效的问题, 已通过严格检查 URDF 文件的 MD5 校验来实现
- 修复由于缺少 `lusb` 而导致的编译错误, 已通过在编译时先检查或安装`libusb-1.0-0-dev`来解决
- 修复 h12pro 遥控器无法控制机器人站立, 行走等问题
- 修复潜在的执行`sudo apt install ros-noetic-Pinocchio -y`而升级版本导致的函数接口不兼容编译报错问题
- 修复程序结束后手臂电机未正常掉使能的问题
- 修复由于`/humanoid_wbc_observation` 话题数据维度减少修改导致 h12 遥控器播放动作失败问题
- 更正  biped_s42 机器人的 URDF 文件, 新增雷达的 TF, 调整 mesh，惯量和限位
- 修复 biped_s42 机器人头部 yaw 电机方向问题, 已根据右手定更正

## 其他改进
- ROS_MASTER_URI 主从配置工具给所有用户都配置ROS环境
- 新增 ROS_MASTER_URI 主从配置工具，[工具链接](./docs/others/CHANGE_ROS_MASTER_URI/修改ROS_MASTER_URI说明.md)
- 修改遥控器头部控制快捷键为RT+右侧摇杆，可以同时走路
- 优化graspBox案例，增加Move的运动类型，使用bt_config.yaml文件box_holdon_pose配置，用于规划抓取后手臂的运动，坐标系是机器人坐标系。
- 限制 pitch limit 速度上下限，避免机器人弯腰时姿态调整过于剧烈
- 优化 cppad 缓存机制，将缓存目录改为基于 URDF 哈希值的文件夹，支持保留多个版本的 cppad 缓存，实现不同 URDF 版本间的快速切换而无需重新编译
- 统一 42、45 等各个版本机器人的 URDF 中的质量
- 去除头部关机限位硬编码，更正为从 URDF 中获取
- 优化手臂电机模块，提升手臂电机的刷新速率至 300 Hz
- IK 模块去除求解时的关节限制， 更正为在 URDF 中定义上下限位
- 优化 VR 控制乐聚夹爪的速度为 90 以提高夹爪控制反馈速度
- 添加 ROS snapshot 源用于指定安装`ros-noetic-pinocchio=2.6.21-1focal.20240830.092123`避免升级 pinocchio 版本导致的编译错误
- 提高遥控器指令截止频率以提高灵敏度
- 优化下肢电机控制模块，提高走路，站立等动作的稳定性
- 添加 hardware tools 用于硬件测试故障排查
- 遥控器控制改进: 使用五阶低通滤波对 cmdVel 进行滤波，截止频率1hz
- 头部 yaw 关节软限位修正, 放宽至 +- 80°

# 1.0.0

## Breaking Changes

## 文档相关
- 更新文档说明如何检测手臂电机运动方向, [文档链接](./docs/硬件基本设置/README.md)
- 更新当前机器人发布和订阅话题的详细描述, 数据单位与物理含义, [文档链接](/docs/运动控制API.md)
- 增加出厂流程文档, [文档链接](./docs/硬件基本设置/README.md)
- 更新 motion_capture_ik 使用文档和示例, [文档链接](./src/manipulation_nodes/motion_capture_ik/README.md)
- 更新 Quest3 VR 使用文档, 补充如何去除空间限制步骤  [文档链接](./ docs/Quest3_VR_basic.md)
- 更新 README 添加了开源版本容器镜像下载链接和使用指南 [文档链接](readme.md) 
- 更新运动控制 API 接口文档, 新增`/gesture/list`, `/gesture/execute` 手势相关服务接口 [文档链接](./docs/运动控制API.md)

## 新增功能
- 新增长手臂4.3版本机器人, 增加对应的 URDF 文件
- 新增手臂电机 CAN 模块识别与绑定功能, 避免与夹爪模块冲突
- 适配 MPC 不同手臂自由度的机器人
- 新增辅助校准功能用于快速调整手臂零点圈数回零, 实现校准圈数、以当前位置作为零点、使能掉使能等方便调试
- 新增遥控器按键启动 VR 遥操作功能
- 新增贝塞尔曲线插值和三次样曲线插值手臂轨迹规划功能和对应的 websocket 播放服务, [文档链接](src/humanoid-control/humanoid_plan_arm_trajectory/README.md)
- 新增 VR 启动时允许指定上扳机只控制拇指+食指 or 除拇指虎口方向外的全部手指自由度
- 新增`/cmd_pose`位置控制接口
- 新增使用基于 MPC 控制机器人行走正方形/圆形/S曲线的示例 [文档链接](src/demo/trace_path/README.md)
- 新增电机 cali 模式下辅助校准功能, 增加准备阶段按`h`可查看当前操作流程提示功能
- 新增开机自启动遥控器控制机器人的功能, 目前支持 h12pro controller, [文档链接](./src/humanoid-control/h12pro_controller_node/ocs2_README.md)
- 新增简单案例: 抓取案例, 使用`/cmd_vel`控制走正方形/圆形/S曲线, 按键控制手臂微调 [文档链接](src/humanoid-control/humanoid_arm_control/README.md),[文档链接1](src/humanoid-control/humanoid_arm_control/docs/arm-control-keyboard.md)
- 兼容 4代、4pro版本的机器人, 新增单步控制功能
- 新增使用 ROBOT_VERSION 环境变量设置机器人版本号, 根据版本号选择对应的配置文件
- 新增自动根据机器人总质量修改对应模型和编译 cppad 功能
- 使用 ROS 标准方式重构开源版本的编译安装方式
- 新增 4.0 与 4.2 仿真环境的头部模型和控制, 并发布头部关节到 TF 与 Rviz 可视化
- 新增 4.1 版本机器人模型与配置文件
- 新增实物 call_leg 参数用于校准腿部零点, 允许直接设置当前腿部位置为零点
- 新增 h12 遥控器启停录制 VR 的手臂轨迹和摄像头图像的功能
- 新增 Quest3 手势识别功能, 支持'握拳', '点赞', 'OK', '666' 等手势, 支持在一键启动中通过`predict_gesture`参数开启
- 新增 ROS 手势执行和获取手势列表服务接口, 接口详情和支持的手势列表见[文档](./docs/运动控制API.md)

## 修复问题 
- 修复 rosparam 获取`/mpc/mpcArmsDof`和`/armRealDof`参数时等待条件错误问题
- 在校准腿部时不检查关节限位, 在跳圈或者编码器和零点位置相差较大时不触发保护挂掉,而是允许进入cali_leg模式校准
- 修复py和shell脚本安装之后没有可执行权限  会导致开源仓库无法rosrun执行脚本,无法使用键盘控制脚本等现象
- 修复humanoid_wbc_observation时间和mode没有更新, 可能导致运行时间久了动作不执行
- 修复 4.2 版本各个 URDF 中的机器人初始质量不一致问题
- 修复全自由度buffer未初始化问题, 导致手臂切换到非遥操作mode会崩溃
- 修复启动时quest3中有几率姿态或者手柄消息为None导致的报错
- 修复 VR 卡顿问题, 原因是增加手势识别功能在每个周期都会查询 TF 树多次, 导致循环卡顿, 已通过修改为每个周期调用一遍解决
- 修复踝关节转换中v和t错误的问题, 用上个周期的电机位置和通过电机位置换算得到的关节位置作为程序的输入，得到期望电机速度和期望电机电流
- 消除长手臂平举时无法伸直的问题, 适配所有手臂, Quest手臂长度改为实时计算
- 修复切换手臂模式时跳变问题, 原因是简化的手臂维度在切换手臂模式时没有经过mpc的插值
- 修复启动时 Quest3 中偶现姿态或者手柄消息为 None 导致的报错
- 修复 手势识别导致的 VR 卡顿问题
- 修复手臂电机默认的相序辨别配置错误问题
- 修复硬件模块捕获不到 signalint 导致手臂电机无法掉使能的问题
- 修复由于 VRHandCommandNode 未初始化可能会阻塞 Ctrl+C 退出
- 修复 4pro mujoco 模型手臂高度错误导致仿真中需要修改 pitch 增益才能运行问题
- 修改手臂电机使能之后存在自动回零的问题，通过发送当前位置一次之后可以避免自动回零解决
- 修复 VR 使用提示无法找到 xx_ik.py 问题, 以及手指控制跳变问题
- 修复开机自启动遥控器控制机器人功能中错误的包路径导致的安装失败问题
- 修复 youda 驱动器版本机器人行走出现全身抖动问题, 原因是缺少髋关节力控、腿部楼空版本质量、手臂末端的零速度约束...
- 修复脚本文件无可执行权限问题, 已通过在 cmake install 中追加权限解决
- 修复开源仓库硬件包查找路径错误问题, 使用 rospack 获取而不是通过维护环境变量
- 修复由于适配驱动器引入的 CST, CSV 下索引错误问题
- 修复由于 IMU 校准导致机器人抖动的问题
- 修复 EcMaster 写入零点文件后导致文件权限和所有者变更问题
- 修复 ROS 接口读取头部位置数据无变化问题, 原因是 hardware 模块没有上传头部电机数据
- 修复容器脚本启动时报错提示`不支持 robot_version 34 版本`, 已更新启动容器脚步中 robot_version 环境变量解决
- 修复遥控器启动 VR 程序时用户无法感知切换手臂控制模式的反馈, 已通过简化操作步骤, 长按遥控器扳机键解决

## 性能优化
- 轻量化 kuavo_assets 的模型文件, 从 1.1G 减小为 130+M
- 修复头部动作执行时不断读取 json 降低效率问题

## 其他改进
- 程序启动时自动清除日志文件, 避免磁盘被占满
- kuavo_assets 更新各个版本机器人的模型快照
- 增加遥控器topic维度不对时进行提示, 防止自动触发开始发生危险
- trot步态去除SS相
- 临时禁用不正确的 quest 头部控制, 等待后续修复
- ik的末端和手肘位置存放在配置文件中, 无需按照原来的link名字的手臂urdf也能使用
- motion_capture_ik IK 节点统一使用 kuavo_asserts包中的模型文件
- 简化 4 代机器人头部和躯干模型的面数,统一 xml 的引入格式为 obj
- 优化 launch 中通过 gnome-terminal 开启的节点作为 launch 的子进程,避免 launch 退出时没有关闭节点
- 根据版本、自由度、质心模型存放 cppad 缓存, 避免混淆
- dockerfile 增加 ros-noetic-rqt-graph ROS 包安装步骤
- 重构手臂电机零点调整功能, 使用单独的零点文件 arms_zero.yaml，不存在时自动获取一次当前位置作为零点
- 移除废弃的 GPU dockerfile 文件
- 整理 URDF 模型文件, mujoco 模型文件和硬件相关的配置文件到 kuavo_assets 包中统一进行管理
- 使用 config 配置目录的 EcMasterType.ini 来指定驱动器类型, 并在编译时提示选择驱动器版本(4.2版本之后生效)
- 添加 CPU 温度、频率、占用率记录和发布方便观测与调试
- 优化日志打印提示, 消除编译告警和补充开源仓库缺失的一些脚本和节点
- 移除废弃的代码,脚本和编译选项
- 新增 VR 相关节点运行状况监控功能, S如果异常会在终端打印提示用户
- 运行环境为容器时取消系统信息发布, 原因是在容器里无法读取温度传感器等信息，同时容器的性能也不能作为系统性能的参考
- 改进容器启动脚本, 自动使用最新的镜像, 每个文件夹启动的容器名字使用不同hash区分
