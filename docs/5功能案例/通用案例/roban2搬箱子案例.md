## 搬箱子案例运行步骤：

### 编译：  
```commandline
cd ~/kuavo-ros-control
catkin build  humanoid_controllers
catkin build  humanoid_plan_arm_trajectory kuavo_sdk
catkin build roban2_move_boxes kuavo_msgs
```
### 启动：  在 2 个终端分别打开  

仿真启动机器人  
```commandline
sudo su
export ROBOT_VERSION=13
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch 
```
真机启动机器人  
```commandline
sudo su
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real.launch
```

启动手臂规划程序
```commandline
source devel/setup.bash
roslaunch humanoid_plan_arm_trajectory humanoid_plan_arm_trajectory.launch arm_restore_flag:=False
```

## 配置文件路径
动作文件需要放在该路径下
`/home/lab/.config/lejuconfig/action_files`

## 注意事项
需要在上位机启动相应的搬箱子节点