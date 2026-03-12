---
title: "kuavo nMPC 基本使用(实物与仿真)"
---

# 接口文档

[readme_topics.md](./docs/readme_topics.md)

[运动控制接口文档](./docs/运动控制API.md)

[全身控制器相关参数说明](./docs/info文件说明.md)

[kuavo_asset配置文档说明](./docs/kuavo_json文档说明.md)

[运动控制接口及topic补充文档](./docs/补充工程说明文档.md)

[ROBOT_VERSION 说明](./docs/robot_version版本号说明.md)

# VR 端软件使用

[Quest3 相关使用](./docs/Quest3_VR_basic.md)

# 如何使用

## 克隆代码
```shell
# ssh
git clone --depth=1 ssh://git@www.lejuhub.com:10026/highlydynamic/kuavo-ros-control.git

# 或者https
git clone --depth=1 https://www.lejuhub.com/highlydynamic/kuavo-ros-control.git
```

根据需要选择某个分支(一般稳定一些为beta)，然后更新子仓库
```shell
git checkout dev
git submodule update --init --recursive
```

## 确认机器人版本和总质量
#### 机器人版本
- 机器人版本通过环境变量`$ROBOT_VERSION`设置，版本号涉及不同机器人模型、硬件设置等, 需要和自己的机器人匹配。
- 在终端执行`echo $ROBOT_VERSION`查看当前设置的版本号，如果没有设置，通过以下设置版本号(其中的40代表4.0版本，根据实际情况修改)：

   1. 在当前终端执行(临时设置): 

     `export ROBOT_VERSION=45`

   2. 将其添加到你的 `~/.bashrc` 或者 `~/.zshrc` 终端配置文件中:
    如执行: 

        `echo 'export ROBOT_VERSION=45' >> ~/.bashrc `

    添加到 `~/.bashrc` 文件(bash终端)末尾，重启终端后生效

#### 机器人质量
- 由于每台机器人的选配不同，质量也不同，需要确认机器人的总质量，确保模型准确。(出厂时的质量会修改正确一次)
- 机器人总质量存储于`~/.config/lejuconfig/TotalMassV${ROBOT_VERSION}`文件中(${ROBOT_VERSION}为上述设置的版本号)，编译时会自动读取该文件，校准仓库中的模型质量。
- 机器人称重之后，将总质量写入`~/.config/lejuconfig/TotalMassV${ROBOT_VERSION}`文件中即可。
- ocs2中使用了cppad自动微分库，cppad的缓存与模型相关
  - 因此每次修改总质量文件时，会`自动`删除缓存目录`/var/ocs2/biped_v${ROBOT_VERSION}`, 下一次运行时会自动重新编译cppad模型(大概4分钟)
  - 如果手动修改了仓库中的模型，会导致缓存失效，再次运行时会重新编译cppad模型


## 编译

##### docker环境
在没有机器人运行环境的情况下，可以使用docker环境进行编译和仿真使用。

- docker镜像可以自行根据后续章节使用`./docker/Dockerfile`构建，或者下载已经编译好的镜像：

```bash  
wget https://kuavo.lejurobot.com/kuavo_research_editiion/docker_images/kuavo_opensource_mpc_wbc_img_v1.3.0.tar.gz
```

- 执行以下命令导入容器镜像：
```bash
docker load -i kuavo_opensource_mpc_wbc_img_v1.3.0.tar.gz
```
- 执行`./docker/run.sh`进入容器后，默认在仓库的映射目录`/root/kuavo_ws`，执行以下命令开始编译：

```bash
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release # Important! 
# -DCMAKE_ASM_COMPILER=/usr/bin/as 为配置了ccache必要操作，否则可能出现找不到编译器的情况
source installed/setup.zsh # 加载一些已经安装的ROS包依赖环境，包括硬件包等
catkin build humanoid_controllers #会编译所有依赖项
```
> 注意：容器镜像内部默认使用zsh


##### 实机环境

- kuavo实机镜像如果较旧，需要手动安装一些依赖项：
```bash
# 提供了一个脚本用于快速在旧的kuavo实机镜像进行安装依赖
./docker/install_env_in_kuavoimg.sh
```
- 如果是干净的发行版的ubuntu20.04，可以直接通过脚本安装所有依赖：
```bash
./scripts/kuavo_environment_setup.sh
```

- 实物编译
```bash
cd kuavo-ros-control #仓库目录
sudo su # 实物需要在root用户下运行，重要！
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release # Important! 
catkin build  humanoid_controllers
# 如果使用 DDS 通信(可选)
catkin clean -y # 如果编译过，则需要清理，否则请忽略
catkin build humanoid_controllers --cmake-args -DUSE_DDS=ON  

# 如果使用 LEJU DDS 通信(可选)
catkin clean -y # 如果编译过，则需要清理，否则请忽略
catkin build humanoid_controllers --cmake-args -DUSE_LEJU_DDS=ON  
```

## 运行

##### 仿真运行
* 使用mujoco仿真器
```bash
source devel/setup.bash # 如果使用docker环境，则使用source devel/setup.zsh
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch # 启动控制器、mpc、wbc、mujoco仿真器
```
* 使用gazebo仿真器
```bash
catkin build humanoid_controllers gazebo_sim # 需要编译gazebo_sim包
source devel/setup.bash # 如果使用docker环境，则使用source devel/setup.zsh
roslaunch humanoid_controllers load_kuavo_gazebo_sim.launch # 启动控制器、mpc、wbc、gazebo仿真器
```
* 使用isaac-sim仿真器
```bash
catkin build humanoid_controllers isaac_sim # 需要编译isaac_sim包
source devel/setup.bash # 如果使用docker环境，则使用source devel/setup.zsh
roslaunch humanoid_controllers load_kuavo_isaac_sim.launch  # 启动控制器、mpc、wbc、isaac_sim仿真器
```
##### kuavo_humaniod_sdk测试
大致步骤如下，具体可查看[Kuavo_Humaniod_SDK搬箱子案例](src/kuavo_humanoid_sdk/README.md)：

- 编译
```
catkin build humanoid_controllers kuavo_msgs gazebo_sim ar_control
```

- 运行:
```
# 启动gazebo场景
roslaunch humanoid_controllers load_kuavo_gazebo_manipulate.launch joystick_type:=bt2pro

# 启动ar_tag转换码操作和virtual操作
roslaunch ar_control robot_strategies.launch  

# 运行搬箱子案例
python3 grasp_box_example.py 
```
##### 实物运行

##### 末端执行器配置
在运行实物之前, 您需要先修改`src/kuavo_assets/config/kuavo_v$ROBOT_VERSION/kuavo.json`中EndEffectorType为您实物机器人的执行器类型:
- `none`: 无末端执行器或者需要屏蔽末端执行器时使用
- `qiangnao` : 灵巧手, 默认值
- `lejuclaw` : 二指夹爪
- `qiangnao_touch` : 触觉灵巧手
- `revo2`: revo2 二代灵巧手， **仅支持 Roban** !

在运行实物时，您可以通过指定`ruiwo_cxx_sdk`参数来选择手臂电机使用 C++ SDK 还是 Python SDK：
- 默认值为`true`表示使用 C++ SDK
- `false`表示使用 Python SDK, 比如`roslaunch humanoid_controllers load_kuavo_real.launch ruiwo_cxx_sdk:=true`


> 实物运行时，开机第一次建议先在cali模式下运行一次，确认机器人姿态和位置正确(机器人所有关节回到零位)
- 实物运行命令：
```bash
sudo su # 实物需要在root用户下运行，非常重要！
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real.launch cali:=true # 以校准模式启动
```

- 零点标定(当校准模式下发现零点不对时才需要进行)
   - 机器人的零点是指所有电机在零位置时的位置, 此时机器人姿态处于完全伸直的状态；
   - 零点标定在机器人出厂时会完成一次，除非更换电机或者编码器，其他情况只需要进行`重启之后的实物零点校准`流程即可。
   - 电机的零点存储有两个地方:
      1. 腿部和肩膀两个电机的零点位置存储于`~/.config/lejuconfig/offset.csv`文件中, 每个关节对应一行, 顺序为从躯干出发,左腿、右腿、左肩、右肩的电机;
         - 腿部零点手动标定过程：
            - 将机器人所有关节摆到零位(可以使用工装等硬件设备进行固定)
            - 打开机器人急停按键给机器人电机上电
            - 运行程序（增加cali:=true参数），在使能完腿部电机后(打印出如下图的位置之后), 即可关闭程序(零点校准之前电机运动可能会超出限位)
            - ![alt text](./images/ecmaster-zero.png)
            - 其中的`Slave xx actual position 5.1770019`即这个电机的实际位置
            - 将需要标零的电机位置复制到`offset.csv`文件中对应的行中即可, 注意不要有多余的空行
         - 腿部零点辅助标定过程：
            - 将机器人所有关节摆到零位(可以使用工装等硬件设备进行固定)
            - 打开机器人急停按键给机器人电机上电
            - 运行程序（增加**cali_leg:=true**参数）
            - 腿部电机进入使能状态之后，按'c'键，自动保存腿部**当前位置**作为零点
      2. 手臂的零点位置存储于`~/.config/lejuconfig/arms_zero.yaml`文件中,如需微调某个关节可以调整
            - 手臂校准在**root**下执行`rosrun hardware_node setZero.sh`将会以当前手臂位置作为零点并保存到配置文件中;
            - 多圈的情况下手臂后续无需校准, 没有多圈记忆功能的机器**重启之后**需要按下述手臂校准流程校准手臂零点所在圈数
   - **电机的运动方向**：
     - 电机的旋转方向和机器人的坐标系正方向一致，符合右手定则。用右手握住电机旋转轴，让拇指指向旋转轴的正方向（旋转轴正方向和机器人的坐标系正方向一致）其余四指的弯曲方向表示旋转的正方向。
     - 机器人正方向：往前->x轴正方向，往左->y轴正方向，往上->z轴正方向。
     - 比如1号髋关节roll轴心与机器人x轴平行，则该电机根据右手定则，绕着机器人x轴旋转的方向即为正方向。
     - 再如：踝关节是由并联杆控制，两个电机轴心与机器人y轴平行(4代)，则该电机根据右手定则，绕着机器人y轴旋转的方向即为正方向。
     - > 注意：膝关节比较特殊，膝关节正方向判断应该看关节的旋转方向而不是看电机的旋转方向(4代上无影响，4pro上的反关节会有差异)
  - 腿部和手臂零点校准方法
    - **腿部校准**：
       1. 手动校准
          - 开机时手动掰机器人各个关节的电机回到零位所在圈数内，打开机器人急停按键给机器人电机上电
          - 运行程序，机器人进入全身伸直姿态，看所有关节是否都回到零位，如果不在零位处，则单独反方向掰动该关节，关闭再重新打开急停按键，重复运行程序步骤直到所有关节都回到零位
       2. 工装校准(有工装时，推荐的方式)：
          - 机器人腿部电机都回到零点位置，插上工装
          - 启动程序luanch时，传入cali_leg:=true参数，根据提示，按`c`,程序会自动将腿部电机的位置保存到配置文件中
       3. 辅助校准，在`cali:=true`模式下, 提供了一个辅助校准流程：
          - 运行程序，机器人进入全身伸直姿态，看所有关节是否都回到零位
          - 如果需要校准，则根据终端提示输入'c'进入腿部校准模式, 输入'v'进入手部校准模式，输入`a`进入手臂限位自动校准的模式
          - 校准腿部的模式下：
                1. 输入 'l' 或 'r' 选择要校准的左腿或右腿。
                2. 输入 1-6 选择要校准的电机。
                3. 输入 'w' 增加编码器圈数，输入 's' 减少编码器圈数（注意：电机会移动！）
                4. 输入 'c' 保存校准结果(断电之前都无需重新校准)，输入 'q' 退出校准。
          - 校准手部的模式下：
                1. 输入 'l' 或 'r' 选择要校准的左手或右手。
                2. 输入 2-7 选择要校准的电机(只校准瑞沃关节的电机)。
                3. 输入 'w' 增加编码器圈数，输入 's' 减少编码器圈数（注意：电机会移动！）
                4. 输入 'd' 将手臂电机掉使能(此时可以掰动手臂电机), 输入 'e' 重新使能
                5. 输入 'f' 以当前手臂位置作为零点并保存
                6. 输入 'c' 保存校准结果(断电之前都无需重新校准)，输入 'q' 退出校准。
          
          - 根据需要通过上述按键组合逐圈调整电机, 直到所有关节都回到零位所在圈。
          - > 注意：按`w`和`s`键时，请仔细确认运动方向正确，不要反方向调节，否则会触发电机的限位保护，程序退出。
    - **手臂校准**：
       1. 手臂的位置需求不那么准确时，可以通过`cali`模式下再传入一个`cali_arm=true`，将会以当前位置作为手臂的零点，并保存到配置文件中。
       2. 手臂限位自动零点校准(限位准确无遮挡时推荐使用)：
          - 输入 'a' 进入手臂限位自动校准模式
          - 选择校准模式：
            1. 输入 '1' 选择自动校准模式（所有关节组自动依次校准）
            2. 输入 '2' 选择逐个校准模式（每组关节需要手动确认后运动）
          - 校准过程：
            1. 机器人会先将手臂移动到一个安全姿态
            2. 从末端关节组开始逐组向近端校准（左右对称关节同时校准）
            3. 每组关节会自动移动到预设的限位位置
            4. 自动检测限位（基于速度、位置方差、超时等条件）
            5. 检测到限位后立即记录位置并计算零点偏移，更新零点
            6. 完成一组后移回安全姿态，继续下一组
            7. 所有组校准完成后，手臂移动到零位姿态验证校准效果
          - 校准完成后：
            1. 输入 's' 保存校准结果到配置文件
            2. 输入其他键放弃保存（EC电机零点会恢复到原始值）
          - 配置参数（在 kuavo.json 中可调）：
            - `arm_calibration_velocity`: 校准移动速度（默认15度/秒）
            - `arm_calibration_timeout`: 单个关节校准超时时间（默认20秒）
            - `arm_calibration_position_variance_time`: 位置方差检测时间窗口（默认0.4秒）
            - `arm_calibration_position_variance_threshold`: 位置方差阈值（默认0.1度²）
            - `calibration_safe_pose`: 校准过程中的安全姿态
            - `arm_calibration_limits`: 各关节的目标限位位置
            - `arm_calibration_directions`: 各关节的校准运动方向
          - > 注意：限位校准会让关节主动运动到机械限位，请确保机器人周围无障碍物且处于安全状态！
> 注意：没有多圈编码器记忆功能的版本(4.2版本以上都有该功能)，电机断电之后，开机都需要手动校准一次;腿部的辅助校准功能只用于关机重启之后**校准编码器圈数**, 即需要确认此前的腿部零点是正常的, 否则请先手动校准一次。
  
- 运行程序之后，机器人会进入待站立阶段，确认无误之后，准备扶住机器人，然后按`o`启动机器人，机器人回开始运动到站立状态，并开启反馈控制。

##### 实物运行-只使能上半身
> 即不控制机器人的下肢关节, 只控制机器人的上半身(手臂和头部关节), 方便在只使用手臂和头部关节的场景快速开发和调试.

1. 修改配置文件
修改`src/kuavo_assets/config/kuavo_v$ROBOT_VERSION/kuavo.json`配置文件中的`only_half_up_body`配置项, 将其设置为`true`.
```json
// 大约在 38 行
"only_half_up_body":true, 
```
2. 确认一下胸部 NUC 的 CPU 型号, 可以执行以下命令查看:
```bash
lscpu |grep  Intel
```
如果输出如下所示, 说明 NUC 的 CPU 型号为`i9`:
```bash
Vendor ID:                            GenuineIntel
Model name:                           13th Gen Intel(R) Core(TM) i9-13900H
```
如果输出如下所示, 说明 NUC 的 CPU 型号为`i7`:
```bash
厂商 ID：                             GenuineIntel
型号名称：                            12th Gen Intel(R) Core(TM) i7-12700
```
3. 运行
> 注意: 不同类型的 CPU 型号, launch 启动命令不同(性能有差异).

对于`i9`型号的 CPU, 执行以下命令启动机器人:
```bash
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real.launch 
```
对于`i7`型号的 CPU, 执行以下命令启动机器人:
```bash
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real_half_up_body.launch
```
其他操作步骤和 **实物运行**章节一样, 您可阅读该章节进行操作.

运行程序之后， 根据终端中的提示(会提示按`o`启动机器人)，然后按`o`启动机器人。

##### 实物运行-轮臂机器人
1. 修改配置文件
修改`src/kuavo_assets/config/kuavo_v$ROBOT_VERSION/kuavo.json`配置文件中的`only_half_up_body`配置项, 将其设置为`true`.
```json
// 大约在 38 行
"only_half_up_body":true, 
```
修改配置文件中的 `MOTOR_TYPE` , 为前 12 个电机追加 _none 来屏蔽下肢关节(轮臂机器人无腿部关节), 如下图.
```json
 "MOTORS_TYPE":[
        "PA100_18_none", "PA100_none", "PA100_none", "PA100_18_none", "CK_none", "CK_none",
        "PA100_18_none", "PA100_none", "PA100_none", "PA100_18_none", "CK_none", "CK_none",
        "PA100", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo",
        "PA100", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo"],
```
2. 运行
本步骤和 **实物运行-只使能上半身**章节一样, 您可阅读该章节进行操作.

## 手柄控制
> 遥控器型号通过运行时launch参数，joystick_type指定，在`src/humanoid-control/humanoid_controllers/launch/joy`目录指定了按键映射关系，新增遥控器类型可以直接添加自己的按键映射关系到json文件中，运行时通过`joystick_type:=bt2pro`传递相应文件名即可
- joystick_type:=bt2
   使用的手柄型号为"北通阿修罗2无线版"，参考的遥控器键位如下，其他型号需要自行修改遥控器节点：
   - ![遥控器](./images/遥控器.png)

   - 字母键切换gait
      - A: STANCE
      - B: TROT
      - X: RL/MPC
      - Y: WALK

   - 摇杆控制腿部运动
      - 左摇杆控制前后左右
      - 右摇杆控制左右转和上下蹲
   - 按钮发送固定target
   - start键实物控制时用于从悬挂准备阶段切换到站立
   - back键用于退出所有节点
   - ABY键切换步态模式，X键切换MPC控制器和RL控制器
   - RL模式下：
     - 推摇杆即走、放摇杆即停
     - 按Y手动切换到RL的原地踏步,推摇杆即走、放摇杆维持原地踏步(注意有的RL控制器如AMP没有手动切换行走功能，只需要推摇杆即走)
     - 按A手动切换到RL的站立
   - MPC模式下：
     - 推摇杆即走、放摇杆即停
     - 按Y手动切换到MPC的原地踏步,推摇杆即走、放摇杆维持原地踏步
     - 按A手动切换到MPC的站立
   - RT+右摇杆可以控制头部
   > 在五代、roban2等机型上，存在腰部yaw自由度，可以通过LT+右摇杆实现腰部控制
   - 手动触发倒地状态(全身掉使能)（仅roban2.1）
     - RB+B键，注意按下之后会全身掉使能，所以手动触发时可以先提着机器人
   - 倒地起身（仅roban2.1）
     - 倒地之后(无论是初始传入init_fall_down_state从倒地状态起来还是手动触发倒地)，按下RB+X键，机器人会首先插值到起身姿态
     - 再次按下RB+X键，自动起身，并进入MPC控制器
     - 前后倒地起身会自动识别
     - 起身过程如果再次按下RB+X键，会结束起身过程，回到全身掉使能状态
- joystick_type:=h12
   使用的手柄型号为"H12pro"，参考的遥控器键位如下
   - ![h12](./images/h12.jpg)
   - 摇杆控制腿部运动
      - 左摇杆控制前后左右
      - 右摇杆控制左右转和上下蹲
   - 实物start开关掰到最中间位置可以结束悬挂准备阶段，进入站立状态
   - 左侧开关掰到最中间终止程序
   - 切换MPC/RL控制器，站立状态下按C键

`HumanoidAutoGaitJoyCommandNodeVel`节点(默认)
- 发送/cmd_vel消息给MPC
- 摇杆往前推，自动切换到walk行走，到达target之后自动停止
- 按钮发送固定target也会自动切换walk行走，自动停止
- 在stance状态时手动切换gait之后，会变成手动模式，通过摇杆可以控制运动，不会自动停止，直到重新切换回stance

`HumanoidJoyCommandNode`节点
- 没有自动切换gait的遥控器节点


> note: 手柄控制实物的`load_kuavo_real.launch`默认打开手柄控制，插上手柄接收器即可使用，仿真的launch文件，需要传入`use_joystick:=true`参数开启

> 开机自启动遥控器就可以控制机器人的功能，文档 [link](./src/humanoid-control/h12pro_controller_node/ocs2_README.md), 目前支持 h12pro controller 并且按键功能与上述 h12 不完全一致，请仔细阅读使用文档
## QUEST3 VR控制
- 按照前面的步骤正常启动机器人
  

- 单独启动VR节点
   - 运行
  
  > 旧版镜像如果没有包含VR相关依赖，需要手动安装：`cd src/manipulation_nodes/noitom_hi5_hand_udp_python && pip install -r requirements.txt && cd -`
  
  ```bash
   source devel/setup.bash

   # VR先和机器人连到同一局域网, VR 会广播 自身IP 到局域网中
   roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch

   # 可选配置参数：use_cpp_ik
   # 启动python版本的ik
   roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_ik:=false

   # 启动C++版本的ik
   roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_ik:=true

  # 可选配置参数：use_incremental_ik(仅当use_cpp_ik:=true 时，可选是否启用增量式IK)
   roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_ik:=true use_incremental_ik:=true
  ```
  > 如果希望同时映射躯干的运动（上下蹲和弯腰），可以增加选项`control_torso:=1`，使用前**务必在站立状态下长按VR右手柄的meta键**以标定躯干高度。

  > 默认控制双手，如果需要控制单手，可以增加选项`ctrl_arm_idx:=0`, 其中0，1，2分别对应左手，右手，双手

  > 如果手动输入VR的IP地址, 在启动命令后追加参数 `ip_address:=192.168.3.32`(替换成VR的实际IP地址)

  > 现在 VR 头盔中的 APP 会自动广播自身IP，启动节点不需要手动输入 ip，但是假如 VR 节点程序关掉了，你需要在 VR 头盔中重新打开 VR 程序，才会重新广播IP

  > 现在 vr 控制默认启动头部控制，如果不需要控制头部，可以增加选项`enable_head_control:=false`

- 同时启动VR节点和机器人
  - 运行

  ```bash
  sudo su
  source devel/setup.bash
  roslaunch humanoid_controllers load_kuavo_real_with_vr.launch
  ```

- 全程使用VR的手柄控制即可
  - 启动时按A键站立(从启动等待开始状态站立，相当于kuavo中的按o)；
  - 停止机器人，同时按下左侧XY两个键，停止机器人
  - 自动模式下，推摇杆即走，松摇杆自动立即停止
  - 按下A（stance）、B（walk）也可以手动切换gait
  - 扳机控制手指开合，Y键用于锁定或解锁手指控制
  - 默认摇杆左摇杆控制前后，右摇杆控制左右转；
    - 当手放到一侧的两个按钮上时(只接触不按下)，切换为对侧为控制左右或者高度
    - 如手指覆盖住左侧的XY键，则右侧摇杆切换为高度控制
    - 手贴在左侧XY键，右侧摇杆会自然地变为高度控制，按下去即可关闭程序；
  - x键为模式切换辅助键，按住x键之后,其他按键的作用如下：
    - A:手臂模式切换为外部控制/自动摆手，这两种模式切换之后会有一个平滑同步到当前规划轨迹的过程
    - B:手臂模式切换为保持姿态
  - Y 键为末端质量设置辅助键，按住 Y 键之后，其他按键的作用如下：
    - A:机器人末端应用设置的末端质量以及对应方向的施力
    - B:机器人末端释放末端质量以及相关方向的力
  - VR低时延模式
    - 在使用有线连接情况下，建议使用低时延模式，可以实现更快速的动作
    - 启动VR之后，按下左边前扳机+X，开启低时延模式；按下左边侧扳机+X，关闭低时延模式
  - VR映射躯干的运动（上下蹲和弯腰）使用前**务必在站立状态下长按VR右手柄的meta键**以标定躯干高度然后同时按下两个手柄前扳机键启动手臂控制
    - 通过给launch_quest3_ik.launch文件传入参数`control_torso:=true`才能开启躯干控制
    - 启动VR之后，先长按手柄左右扳机，解锁手臂和躯干跟踪映射
    - 左手大拇指同时触摸XY两个按键（注意不要按下）+同时右手按下B键，开启映射躯干模式；再次左手大拇指同时触摸XY两个按键（注意不要按下）+同时右手按下B键，关闭映射躯干模式
    - **开启或者关闭均需在站立情况下进行**
    > 在五代、roban2等机型上，存在腰部yaw自由度，VR可以通过触摸Y+右摇杆实现腰部控制
  - VR切换控制器（MPC/RL控制器）
    - 左手大拇指同时触摸XY两个按键（注意不要按下），同时右手按下A键, 可切换到下一个控制器
    - 两种控制器都是推摇杆即走，松摇杆自动立即停止
    - 按B切换到手动原地踏步模式，A切换站立
  
  - VR右摇杆单步转向
    - 需要在`roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch`启用时需在启动时传入参数`use_step_turning:=true`
    - 目前分为 4 档转向: 15°, 30°, 45°,60°, 大约占比摇杆区间行程的:0~15%, 15~45%,45~75%,75~100%
    - 注意只有当左摇杆不动时（即没有控制前后左右运动），右摇杆左右操作才会进行单独转向，否则会采用普通模式进行转向
    - 正常站立时，右手摇杆左右控制单步转身
    - VR按 B 进入踏步状态后，右手摇杆左右控制踏步转身	
 
 > 开启手势识别，可以增加选项 `predict_gesture:=true`，利用神经网络预测手势，灵巧手会直接根据手势预测结果进行运动，目前支持的手势有（只有当预测结果同时满足：高置信度（>80%）明显优于第二预测（差值>0.3）预测分布集中（熵值<0.8）才会返回具体的手势类别。否则会认为预测失败，灵巧手会采用原来的方式控制）：

  > - 单指点（外展式）：[`finger-pointing-opposed`](./src/manipulation_nodes/motion_capture_ik/imgs/finger-pointing-opposed.png)
  > - 五指抓取：[`cylindrical-grip`](./src/manipulation_nodes/motion_capture_ik/imgs/cylindrical-grip.png)
  > - 666手势：[`shaka-sign`](./src/manipulation_nodes/motion_capture_ik/imgs/shaka-sign.png)
  > - 两只捏（外展式）：[`precision-pinch-unopposed`](./src/manipulation_nodes/motion_capture_ik/imgs/precision-pinch-unopposed.png)
  > - 握拳：`fist`
  > - 点赞：`thumbs-up`
  > - 五指张开：[`palm-open`](./src/manipulation_nodes/motion_capture_ik/imgs/palm-open.png)
  > - 三指捏：[`tripod-pinch-unopposed`](./src/manipulation_nodes/motion_capture_ik/imgs/tripod-pinch-unopposed.png)
  > - 兔子头手势：[`rock-and-roll`](./src/manipulation_nodes/motion_capture_ik/imgs/rock-and-roll.png)
  > - 二指夹（外展式）：[`two-finger-spread-unopposed`](./src/manipulation_nodes/motion_capture_ik/imgs/two-finger-spread-unopposed.png)

> 如果希望从机器人双手平放开始控制，可以运行`rosrun motion_capture_ik scripts/tools/lay_robot_hands_flat.py`，机器人会自动将双手平放

## QUEST3 视频流

- 在上位机（带有摄像头的） clone 本仓库后，克隆以下仓库并且编译

```
cd <kuavo-ros-control>
sudo apt install v4l-utils
git clone --depth=1 https://github.com/ros-perception/image_common.git  --branch noetic-devel src/image_common
git clone --depth=1 https://github.com/ros-perception/image_pipeline.git  --branch noetic src/image_pipeline
git clone --depth=1 https://github.com/ros-drivers/usb_cam.git --branch develop src/usb_cam
cd <kuavo-ros-control>
catkin build usb_cam noitom_hi5_hand_udp_python
```

- 在上位机运行 `source devel/setup.bash` 后，运行 `roslaunch noitom_hi5_hand_udp_python usb_cam_node.launch` 即可

- 在下位机运行 `roslaunch noitom_hi5_hand_udp_python launch_quest3_ik_videostream_usb_cam.launch` 即可


 ## 键盘控制
 - 如没有手柄或者VR等，可以使用键盘控制节点进行控制, 如运行仿真器并且使用键盘控制节点
 
 ```bash
 source devel/setup.bash   
 roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch joystick_type:=sim 
 ```
 - 运行后将会启动仿真，并且在另外一个终端窗口打开键盘控制窗口，按键控制机器人运动，具体按键如下：
```
WASD: Left stick, control forward/backward, left/right
IKJL/QE: Right stick, up/down, turn left/right
R: walk, C: stance, T: trot
B: BUTTON_BACK, O/F: BUTTON_START
<space>: Reset all axes to zero
Press Ctrl-C to exit
```
- 键盘控制节点也单独可以手动开启，可以运行多个
```bash
source devel/setup.bash
rosrun humanoid_interface_ros joystickSimulator.py 
```

## 示例

- 基于 MPC 控制机器人行走特定轨迹(直线，正方形，圆形，S 曲线) [示例](./src/demo/trace_path/README.md)
- `/cmd_vel`控制机器人行走特定轨迹(正方形, 圆形, S 曲线) [示例](./src/humanoid-control/humanoid_interface_ros/docs/walk-trajectory.md)
- 键盘控制机器人手臂运动[示例](./src/humanoid-control/humanoid_arm_control/README.md)
- 基于 apriltag 抓取[示例](./src/humanoid-control/humanoid_arm_control/README.md)
- **倒地起身**[说明文档](./src/humanoid-control/humanoid_controllers/docs/倒地起身操作说明.md)（目前仅支持roban2.1）

## 工具

### Kuavo 开机播报 WIFI 工具

本工具提供机器人开机后, 语音提示已连接的 WIFI 信息(名称和IP). 若在一定时间内未连接WIFI,则会自动切换 AP 热点模式, 您可以使用 VNC或SSH连接机器人, 切换到对应的WIFI.

更多详细安装和使用步骤请阅读[README文档](./tools/announce_wifi/readme.md).

### Kuavo 热点工具
本工具提供机器人同时连接 WIFI 并开启热点功能, 方便用户连接到机器人.

更多详细安装和使用步骤请阅读[README文档](./tools/linux_wifi_hotspot/readme.md).

# Build docker image&container for Kuavo-MPC-WBC
## 1. Install Docker
Follow the instructions on the official Docker website to install Docker on your system. 
## 2. Build docker imags with a dockerfile
We provide a dockerfile for Kuavo-MPC-WBC. You can use it to build a docker image for Kuavo-MPC-WBC. You just need to run the following command, which will build the docker image `kuavo_mpc_wbc_img:0.3` from the `./docker/Dockerfile`.
```bash
./docker/build.sh
``` 

# run docker container
## 1. Run docker container
You can run a docker container with the following command:
```bash
docker run -it --net host  --name kuavo_container  --privileged  -v /dev:/dev  -v "${HOME}/.ros:/root/.ros"  -v "./.ccache:/root/.ccache"  -v "./:/root/kuavo_ws"  -v "${HOME}/.config/lejuconfig:/root/.config/lejuconfig"  --group-add=dialout  --ulimit rtprio=99  --cap-add=sys_nice  -e DISPLAY=$DISPLAY  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  kuavo_mpc_wbc_img:0.3  bash
```

## 2. for GPU Version
If you want to use GPU version of Kuavo-MPC-WBC, you just need to add the `--gpus all` command to the `docker run` command.

## 3. we provide a script to run the docker container
You can run the docker container with the following command:
```bash
./docker/run.sh
```
> This script will automatically find the exisiting container and restart it, or create a new container if it does not exist, and run the container with the correct parameters. 
