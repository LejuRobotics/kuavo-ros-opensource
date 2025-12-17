# VR 测试说明文档
## 一键模拟 VR 测试
### 配置
- 安装依赖:
```bash
   sudo pip install tqdm && sudo pip install questionary
```
- 编译:
```bash
    cd <kuavo-ros-control>
    catkin build automatic_test
```
- 加载环境变量
```bash
    source devel/setup.bash
```
### 启动测试
- 参数说明
```bash 
    "test_round": 测试轮数,即每一个动作执行多少次.
```
- 启动脚本
```bash
    ./start_VR_test.sh test_round
```
### 测试结束后,会统计误差以及判断动作是否执行标准,如果有不标准的执行,认为此次动作失败,会以红色注明.

## 关于阈值的说明
## 误差计算方法
1. 逆解的逐关节误差：
- 比较每一个关节的每一个逆解结果，两个值作差值，将整个逆解结果的差值进行求平方差，并且将所得的平方差求平均,获得平均平方差.
2. 机器人实际运动关节对比：
- 由于每一次的运动，逆解并不一定是唯一解。因此还读取了实际的关节电机读数，可参考电机是否达到目标位置。同样使用逐关节误差的平方求平均.
## 阈值
- 经过仿真和实机测试,当前阈值设置为:
```bash  
    文件目录:
    kuavo-ros-control/src/automatic_test/automatic_test/scripts/automatic_VR_test/VR_test.py
    # 关节误差阈值
    JOINT_ERROR_THRESHOLD = 0.015  
    # 逆解轨迹误差阈值
    IK_ERROR_THRESHOLD = 9.0  
```
## 动作录制
- 启动机器人仿真，启动 VR
```bash
   source ~/kuavo-ros-control/devel/setup.bash
   roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch 
   roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch 

   # 可选配置参数：use_cpp_ik
   # 启动python版本的ik
   roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_ik:=false 

   # 启动C++版本的ik
   roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_ik:=true
   
   # 可选配置参数：use_incremental_ik(仅当use_cpp_ik:=true 时，可选是否启用增量式IK)
   roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_ik:=true use_incremental_ik:=true
```
- 接入 VR
- 运行程序，开始录制：
```python
    python3 record_bag.py
```
- 解锁手臂,开始跟随运动.
- 执行待测试动作，大约在 5 秒以内
- 录制完毕后，录制的 rosbag 文件在以下目录，同时脚本也会有输出：
```bash
   ~/rosbag_records/session_2025*.bag
```
## bag 拆分
- 将录制的 bag 拆分成两个待使用的bag：
1. 包含实际参考的逆解轨迹以及电机值的 bag：
```bash
    session_2025*_group1.bag
```
2. 包含 VR 发布的末端的姿态和手肘的姿态的骨骼信息
```bash
    session_2025*_group2.bag
```
- 拷贝待拆分的 rosbag 文件到当前目录：
```bash
    cp ~/rosbag_records/session_2025*.bag ./
```
- 运行脚本自动拆分
``` bash
    ./filter_bag.sh session_2025*.bag
```
- 拆分出来后的 rosbag 包保存在当前目录