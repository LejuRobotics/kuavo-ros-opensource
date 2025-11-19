# Kuavo Data Challenge Simulator - 安装教程

## 1. 克隆代码仓库
```bash
git clone https://github.com/LejuRobotics/kuavo-ros-opensource.git
cd kuavo-ros-opensource
git checkout opensource/kuavo-data-challenge
````

## 2. 启动 Docker 环境
- docker镜像可以使用`./docker/Dockerfile`构建，或者下载已经编译好的镜像(推荐)：

```bash  
wget https://kuavo.lejurobot.com/docker_images/kuavo_opensource_mpc_wbc_img_v0.6.1.tar.gz
```

- 执行以下命令导入容器镜像：
```bash
docker load -i kuavo_opensource_mpc_wbc_img_v0.6.1.tar.gz
```
- 执行`./docker/run.sh`进入容器后（此处推荐使用gpu版本的run_with_gpu.sh,需要相应的nvidia container toolkit），默认在仓库的映射目录`/root/kuavo_ws`

## 3. 设置机器人版本环境变量

注意，运行推理脚本前请确保机器人版本是45，可用```echo $ROBOT_VERSION```查看当前设置的版本号
```bash
export ROBOT_VERSION=45
```

## 4. 添加密钥

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys AD19BAB3CBF125EA
```

## 5. 安装缺失依赖

```bash
sudo apt-get update
sudo apt-get install ros-noetic-geographic-msgs
```

## 6. 编译工程

```bash
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release
source installed/setup.zsh
catkin build humanoid_controllers data_challenge_simulator
source devel/setup.zsh
```

## 7. 安装 SDK

```bash
cd src/kuavo_humanoid_sdk
./install.sh
```

## 8. 运行 MuJoCo 仿真

```bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch
```

- 等待机器人加载完成后(！！重要！！加载完成后机器人会正常站立，且Mujoco上不会有pause)退出仿真。若能正常运行则说明环境配置成功。若运行时出现机器人摔倒或者闪退情况，可以多试几次或尝试将launch文件中的taskset -c 18改成其他数字（可以尝试1）。

- 后续可在kuavo_ws目录下进入./zshrc，```nano ~/.zshrc```添加
```bash
export ROBOT_VERSION=45
source devel/setup.zsh
```
就不需要每次都设置机器人版本和source了

# 内容说明
## 如何使用
### 模型推理测试

⚠️ 模型推理需要同时开启仿真环境和推理环境，此文档只包含推理时仿真侧的操作，推理侧具体请查看kuavo_data_challenge仓库的README文档
开始推理后，可以在kuavo_data_challenge仓库的log文件夹下查看log/kuavo_deploy/deploy.log来确认报错原因

进入src/data_challenge_simulator/examples/deploy文件夹，运行
```bash
python3 deploy.py
```
选择此次推理的任务

仿真侧会自动打开对应场景文件，并将机器人移动到初始预抓位，此时等待推理侧响应，开始推理（注意，第一次会自动被reset并结束所有进程）

推理过程中，若完成任务或者推理结束，都会自动结束此次推理并结束所有进程，开始下一次推理过程

每一轮的得分和目前的平均分会在terminal中显示，只取有效的数据计入总成绩

Note: 这里如果运行deploy.py时，仿真环境没有启动，一直报错：Timeout waiting for marker1, 解决方案如下：
1. 先运行```roslaunch data_challenge_simulator load_kuavo_mujoco_sim1.launch```, 检查是否能正常启动仿真环境
2. 若可以正常运行，打开src/data_challenge_simulator/examples/deploy/deploy.py文件，在第385行的sleep处，改为5s或者更长(根据需要调整)
3. 如果不能正常运行，请先检查第8步是否正常

## 数据与任务说明
### 数据说明
- 采集的rosbag包含如下topic：
```text
'/gripper/command',                                # 夹爪action      (type: JointState, 仅使用position:0(打开) - 255(闭合)) [left_gripper, right_gripper]
'/gripper/state',                                  # 夹爪state       (type: JointState, 仅使用position:0(打开) - 0.8(闭合)) [left_gripper, right_gripper]
'/sensors_data_raw',                               # 手臂state       (包含joint_data,imu_data,end_effort_data，其中手臂state为joint_data.joint_q,单位为弧度)
'/kuavo_arm_traj',                                 # 手臂action (type: JointState, 仅使用position:单位:角度, [0:7]:左臂七个关节角度指令，[7:14]:右臂七个关节角度)

'/joint_cmd',                                      # 一般不使用

'/cam_h/color/image_raw/compressed',               # 头部彩色相机     （格式：jpg， 640x480）
'/cam_l/color/image_raw/compressed',               # 左腕彩色相机     （格式：jpg， 640x480）
'/cam_r/color/image_raw/compressed',               # 右腕彩色相机     （格式：jpg， 640x480）

'/cam_h/depth/image_raw/compressedDepth',          # 头部深度相机     （格式：png， 640x480）
'/cam_r/depth/image_rect_raw/compressedDepth',     # 左腕深度相机     （格式：png， 640x480）
'/cam_l/depth/image_rect_raw/compressedDepth',     # 右腕深度相机     （格式：png， 640x480）

'/cmd_pose_world',                                 # 机器人位置指令(仅任务四)    (type：Twist, 包含linear.x,y,z和angular.x,y,z，分别对应机器人在世界坐标系中的位置和方向)
```

### 任务说明(后续可能会有修改)
#### 任务一
此任务中，机器人左臂将左侧传送带上的物体抓取并放置到桌面指定区域1，右臂再抓取此物体并放置到区域2

其中，环境光照强度，颜色，物体的颜色，在传送带上的位置，区域1与区域2的位置与颜色，均会在一定范围内随机

##### 评分标准：
- a. 成功抓住传送带上物体并放到桌面上指定区域，加30分，保持方向正确，加10分

- b. 成功抓住放置到桌面物体并放到最终目标区域，加30分，保持方向正确，加10分

- c. 在指定时间N秒内完成，加20分，超出N秒，每一秒扣2分

- 关于位置和方向的判定：

物体必须在桌面上，且中心位置必须位于区域色块内才视为位置正确，仅位置成功时才会触发方向正确加分，物体z轴与世界z轴偏差不超过10度视为方向正确


#### 任务二
此任务中，机器人右臂将正面传送带上的移动物体抓取并放置到桌面的称(具有一定高度)上，称重后左臂再将物体从称抓取放置到目标区域

其中，环境光照强度，颜色，物体的颜色，在传送带上的位置，称与目标区域的位置与颜色，均会在一定范围内随机

##### 评分标准：
- a. 成功抓住传送带上物体并放到称，加30分，保持方向正确，加10分

- b. 成功抓住称重后物体并放到最终目标区域，加30分，保持方向正确，加10分

- c. 在指定时间N秒内完成，加20分，超出N秒，每一秒扣2分

- 关于位置和方向的判定：

物体必须在桌面上，且中心位置必须位于区域色块内才视为位置正确，仅位置成功时才会触发方向正确加分，物体z轴与世界z轴偏差不超过10度视为方向正确


#### 任务三
此任务中，机器人需要将桌面上的三个物体中反面的物体进行翻面并运送到目标区域，正面物体不翻面仅移动到目标区域。

其中，三个物体中会有随机的两个物体是反面，一个物体是正面，三个物体的位置会在一定范围内随机

##### 评分标准
- a. 成功将两个反面物体运送到目标区域，每一个加10分，保持正面朝上，每一个加25分

- b. 成功将一个正面物体运送到目标区域，加10分，保持正面朝上，加10分

- c. 在指定时间N秒内完成，加10分，超出N秒，每一秒扣1分

- 关于位置和方向的判定：

物体必须在桌面上，且中心位置必须位于区域内才视为位置正确，仅位置成功时才会触发方向正确加分，物体正面轴(-y)与世界z轴偏差不超过10度视为方向正确


#### 任务四
此任务中，机器人需要先移动到前方货架，右臂抓取物体1，转身并移动到后方货架，将物体1放置于料盘A。再从后方货架移动到前方货架，左臂抓取物体2，返回后方货架并将物体2放置于料盘B

其中，物体1，2的位置，料盘A，B的位置会在一定范围内随机

##### 评分标准
- a. 准确移动到目标区域，每一次加10分，共4次

- b. 成功将两个物体放置于指定料盘，每一个加20分

- c. 在指定时间N秒内完成，加20分，超出N秒，每一秒扣1分

##### 任务四说明
任务四涉及腿部的动作，而处于安全考虑，机器人只有在站立时手臂才能移动，所以两者话题相互独立，我们做了以下修改：
1. 手臂的action话题 ```/kuavo_arm_traj```更新为```/kuavo_arm_traj``` + ```/kuavo_arm_traj_synced```, 其中```/kuavo_arm_traj``` 跟之前并无差异，但由于机器人移动时手臂话题会被阻塞，导致话题中间会有间断，为了保证数据的时间连续性，我们引入```/kuavo_arm_traj_synced```，此话题为```/kuavo_arm_traj```的映射，将话题的间断部分填充为间断前上一时刻的关节action，其余部分完全相同。更新后，```/kuavo_arm_traj_synced```作为手臂的action话题，```/kuavo_arm_traj```由于存在断点，可以很好的用于判断机器人当前状态，我们保留这个话题用作标志位，来判断当前机器人应该执行的动作

2. 由于使用IL训练腿部数据不太现实，我们这里引入机器人位置的话题```/cmd_pose_world```，这个话题同上面介绍的，指定了机器人在世界坐标系中的位姿，向这个话题发送msg会调用内置的方法让机器人移动，所以此话题将作为aciton的一部分。同时，由于上面提到的断点问题，我们引入连续版本的```/cmd_pose_world_synced```作为模型学习的机器人位置的action

3. 相应的数据转换、训练、推理代码都已同步更新，支持以上改动，流程和之前一致，但请大家注意在进行task4的数转和推理时修改config，尤其是only_arm参数设置为False，如果数转的时候没有设置，训练完了是无法推理的。

4. 任务四流程较长，数据文件比较大，大家如果硬件有限可以选择先下载少量数据，训练时也可以选择提前结束

5. 任务四本身任务较难，而且下肢运动的实现并不是精确和稳定的(不论是训练数据还是IL的方法)，各位选手如果任务四分数较低或者效果不太理想，请记住这是非常正常的，大家可以把任务四当成一个“附加题”去看待，不需要要求太高。