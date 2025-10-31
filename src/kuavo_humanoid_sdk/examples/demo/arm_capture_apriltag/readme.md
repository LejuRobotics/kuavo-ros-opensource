# Kuavo Humanoid SDK - 抓取与递水示例

本项目演示如何使用 **Kuavo Humanoid SDK** 控制机器人完成 **识别 AprilTag → 抓取物体 → 递水 → 复位** 的动作流程。  
脚本通过 SDK 接口直接控制机器人头部、手臂、灵巧手，并结合相机识别结果进行抓取姿态规划。

---

## 功能特点
- ✅ **初始化与站立**：调用 SDK 使机器人进入站立模式  
- ✅ **相机识别**：获取 AprilTag (`id=0`) 的位姿信息  
- ✅ **位姿规划**：根据 AprilTag 位置计算抓取点并转换为四元数姿态  
- ✅ **逆运动学 (IK)**：调用 `robot.arm_ik()` 计算手臂轨迹  
- ✅ **灵巧手控制**：支持 `open` / `close` / `zero` 姿态  
- ✅ **抓取与递水**：完整执行 **张手 → 抓取 → 递水 → 松手 → 复位** 动作序列  

---

## 使用方法

1. **SDK环境构建**
  - 运行`pip show kuavo-humanoid-sdk-ws`或`pip show kuavo-humanoid-sdk`查看本地的sdk版本
  - 若已存在新版本的sdk,可以跳过此步骤
  - 若未存在,运行`pip install kuavo-humanoid-sdk-ws`安装最新稳定版本sdk
  - 注意:若`kuavo-humanoid-sdk-ws`与`kuavo-humanoid-sdk`二者互相冲突,不能同时存在,可参考`uninstall_sdk.md`进行卸载

2. **下位机 使机器人站立**
  - 仿真：`roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch with_mm_ik:=True`
  - 实物：`roslaunch humanoid_controllers load_kuavo_real.launch with_mm_ik:=True cali:=true`

3. **下位机 启动ik求解服务**
  - **注意: 部分版本的ik逆解服务会在上一步启动机器人时自动启动,注意不要重复启动**
  - 判断方式: 终端输入`rosnode list | grep ik`
  - 若已存在`/arms_ik_node`, 则跳过此步
  - 若不存在`/arms_ik_node`, 则运行:
    - 在lab目录下新开一个终端执行 `cd <kuavo-ros-opensource>` ，
    - 执行 `sudo su` 进入root用户，
    - 执行 `source devel/setup.bash` ，
    - 执行 `roslaunch motion_capture_ik ik_node.launch ` ，

      ```bash
      cd kuavo-ros-opensource  # 进入下位机工作空间
      sudo su
      source devel/setup.bash
      roslaunch motion_capture_ik ik_node.launch 
      ```
4. **发送tag信息（`仿真运行`和`实机运行`二选一，不要同时运行）**
- `仿真运行 下位机` 启动tag信息mock工具
  - 新开一个终端执行 `cd kuavo-ros-opensource` ，
  - 执行 `sudo su` 进入root用户，
  - 执行 `source devel/setup.bash` ， 
  - 执行 `python3 src/demo/arm_capture_apriltag/mock_tag_publisher.py`

- `实机运行 上位机` 启动传感器
  - 新开一个终端执行 `cd kuavo_ros_application` ，
  - 执行 `source /opt/ros/noetic/setup.bash` 切换到ros1环境 ，
  - 如果是远程连接上位机桌面 
    - 若上位机为`i7`,执行 `export DISPLAY=:1.0`
    - 若上位机为`ORIN-NX`或`AGX-Orin`,执行 `export DISPLAY=:2.0`
  - 执行 `source devel/setup.bash` ， 
  - 根据实际机器人版本启动程序:
  ```bash
  # 旧版4代, 4Pro
  roslaunch dynamic_biped load_robot_head.launch
  # 标准版, 进阶版, 展厅版, 展厅算力版
  roslaunch dynamic_biped load_robot_head.launch use_orbbec:=true
  # Max版
  roslaunch dynamic_biped load_robot_head.launch use_orbbec:=true enable_wrist_camera:=true
  ```
5. **检测下位机是否能收到标签信息**
- `下位机` 执行 `rostopic list | grep tag`
- 如果存在 `/robot_tag_info`
  - 执行 `rostopic echo /robot_tag_info`
  - 观察是否存在标签的坐标信息
- 注意事项:
  - 如果在实物上运行，需测量得到的坐标信息是否准确
  - 要启动下位机程序后，上位机才能检测到机器人各关节的角度，以计算出基于机器人坐标系的结果

6. **下位机 启动二维码抓取流程**
- 在lab目录下新开一个终端执行 `cd <kuavo-ros-opensource>` ，
- 执行 `sudo su` 进入root用户，
- 执行 `source devel/setup.bash` ， 
- 机器人实物运行
  - 执行 `python3 src/kuavo_humanoid_sdk/examples/demo/arm_capture_apriltag/arm_capture_example.py --offset_start True` 
- 机器人仿真运行  
  - 执行 `python3 src/kuavo_humanoid_sdk/examples/demo/arm_capture_apriltag/arm_capture_example.py --offset_start False` 
 
---

## 动作流程
1. 初始化 SDK → 机器人进入 **站立状态**  
2. 机器人 **低头**，相机识别 **AprilTag(id=0)**  
3. 根据识别结果 + 偏移量计算抓取点，并进行 **IK 求解**  
4. 手臂移动至准备位置，**右手张开**  
5. 执行 IK 结果 → **右手闭合抓取**  
6. 通过 **正运动学 (FK)** 校验手部位置  
7. **递水动作** → 手臂将物体递出 → **右手张开**  
8. 手臂回到初始位置，头部抬起，**右手归零**  
9. **复位**：执行 `robot.manipulation_mpc_reset()` 与 `robot.arm_reset()`  

---

## 注意事项
- 确保机器人 SDK 已正确安装，并且机器人头部与手臂零点已标定   
- 不同机器人型号（42/45/48/49）对应不同的初始零点，请确认版本号  
- 建议在调试时设置安全区域，避免手臂运动范围内有人员或障碍物  

---

## 示例运行效果
运行后机器人会完成以下动作：
1. 识别瓶盖上的 **AprilTag 目标**  
2. 手臂伸出，右手张开并移动至目标位置  
3. 闭合手指抓取目标  
4. 将目标递出，随后松手  
5. 回到初始姿态并复位  

