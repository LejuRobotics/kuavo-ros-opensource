## Roban上楼梯仿真及实物启动说明

## 启动方法

`catkin build humanoid_controllers gazebo_sim`

`source devel/setup.zsh`或者setup.bash

- 仿真版本

`roslaunch humanoid_controllers load_kuavo_gazebo_sim.launch`

`python3 ./src/humanoid-control/humanoid_controllers/scripts/stairClimbPlanner-roban-sim.py`

- 实物版本

`roslaunch humanoid_controllers load_kuavo_real.launch`

`python3 ./src/humanoid-control/humanoid_controllers/scripts/stairClimbPlanner-roban.py`

## 注意事项

`offset_x = [-0.01, 0.03, 0.0, 0.0, 0.0]`  ->  `offset_x = [0.02, 0.03, 0.0, 0.0, 0.0]`

目前仿真和实物脚本仅存在一处差异，即offset_x的第一步偏移，仿真为-0.01因为楼梯设置较近，而实物为0.02因为每次放置位置不同需要保证第一步稳定跨上台阶

实物启动时需将机器人放置在台阶前方3-5cm的位置，不能太近也不能太远

## 仿真问题

目前gazebo仿真存在初始状态加载可能发散的问题，暂时不好定位，多开几次脚本就可以，或者在`kuavo-ros-control/src/gazebo/gazebo-sim/launch/gazebo-sim.launch`里编辑

`<arg name="paused" value="true"/>`

这样会暂停gazebo，在界面里点击开始或者空格键就基本能正常启动

## 功能调试

添加了具有前走、转弯、连续上楼梯的测试脚本`continuousStairClimber.py `，在终端中输入数字进行指令发布，其可以使用三次样条插值和三角插值两种方法进行上楼梯的轨迹插值,插值方法添加在 `trajectory_interpolator.py `，其中默认五步上楼梯，与`stairClimbPlanner-roban.py`相同，如果输入步数为2可以只上一阶，检查当前机器人是否能正常运行

- 脚本启动方法

`python3 ./src/humanoid-control/humanoid_controllers/scripts/continuousStairClimber.py`