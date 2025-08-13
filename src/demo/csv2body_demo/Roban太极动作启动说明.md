## Roban太极动作启动说明

太极动作的执行需要读取动作文件`actions\taiji_step_roban_stable.csv`, 使用脚本`step_player_csv_ocs2.py`进行读取、处理和ROS话题发布

csv数据格式以第一行举例

0.01,0,0.0,0.0,0.633582,0.0,0.023596,0.153187,-0.10927,0.015454,-0.13749,-0.004997,0.007705,-0.012599,-0.000956,-0.010388,0.008096,0.00014,0.001207,-0.010142

时间(1)+接触相(1)+躯干维度(4)+左脚维度(3)+右脚维度(3)+手臂维度(8)

接触相0为双支撑相，1为右腿摆动，2为左腿摆动，躯干为(x,y,z,yaw)，脚部为(x,y,yaw)，手臂为关节角度

在脚本中动作发布分为两部分：

- 单步控制腿部话题 humanoid_mpc_foot_pose_target_trajectories
- 手臂话题 kuavo_arm_target_poses

## 启动方法

- 编译控制器

`catkin build humanoid_controllers`

`source devel/setup.zsh`或者setup.bash

- 运行仿真或实物

实物版本`roslaunch humanoid_controllers load_kuavo_real.launch`

仿真版本`roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch`

- 运行脚本

`python3 ./src/demo/csv2body_demo/step_player_csv_ocs2.py`



## 注意事项

提供了轨迹编辑脚本`kuavo-ros-control/src/demo/csv2body_demo/scripts/motion_editor_simple.py`可进行简单的落地点编辑和手臂关节调整，注意由于采用了单步控制接口，躯干和腿的轨迹在腾空相只有最后一帧有效，仅调整最后一帧即可，手臂轨迹不受影响

由于灵巧手的安装问题，实物上不同机器执行可能存在打手问题，如果出现可适当将手臂零点往外打，或者调整对应时刻的手臂轨迹