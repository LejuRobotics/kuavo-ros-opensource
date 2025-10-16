## Roban上楼梯仿真及实物启动说明

## 启动方法

`catkin build humanoid_controllers gazebo_sim`

`source devel/setup.zsh`或者setup.bash

- 实物版本

`roslaunch humanoid_controllers load_kuavo_real.launch`

`python3 ./src/humanoid-control/humanoid_controllers/scripts/continuousStairClimber-roban.py`

- 仿真版本

`roslaunch humanoid_controllers load_kuavo_gazebo_sim.launch load_stairs:=true`

`python3 ./src/humanoid-control/humanoid_controllers/scripts/continuousStairClimber-roban-sim.py`



```
# 使用终端交互的楼梯脚本，可以执行前走、转向、上楼梯，可定义上楼梯步数和楼梯偏置距离
python3 ./src/humanoid-control/humanoid_controllers/scripts/continuousStairClimber-roban.py
# 一次执行五步的固定脚本，需固定机器人距离楼梯的距离
python3 ./src/humanoid-control/humanoid_controllers/scripts/stairClimbPlanner-roban.py
```

- 交互脚本

使用终端交互版本启动的话，会看到下面的界面，可以使用3/4调整机器人位移到台阶前3cm的位置，之后使用1的默认设置就可以执行上楼梯

在启动站立之后至少需要运行前进脚本一次，来保证躯干和脚的位置同步,之后脚和台阶之间的距离才是实际应该在脚本规划里输入的偏置

```
==================================================
连续楼梯控制器
==================================================
请选择要执行的功能:
┌────────────────────────────────────────────────┐
│ 1. 连续上楼梯 (up_stairs)                │
│ 2. 连续下楼梯 (down_stairs)              │
│ 3. 前进/后退 (forward)                   │
│ 4. 转身 (turn)                           │
│ 5. 切换轨迹方法 (method)                 │
│ 6. 退出 (quit)                           │
└────────────────────────────────────────────────┘

请输入选择 (1-6): 1

执行连续上楼梯
请输入离楼梯的偏置距离 (默认0.01m): 
请输入迈步次数 (默认5步): 
连续上楼梯执行成功

==================================================
```

- 服务启动

如果使用服务启动的方法，可以调用服务启动`src/kuavo-ros-control/src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo/robot_climbstair.py`，功能与`stairClimbPlanner-roban.py`一致

## 注意事项

当前Roban实物进行单步移动时会存在每步向后移大约1cm的问题，经过排查应该是躯干的问题，所以实物进行移动时暂时添加了躯干的偏移来补偿后移 `temp_x_offset`，其其作用的方法是在上楼梯规划中每一步执行前叠加到躯干上，而由于单步接口基于躯干来规划，腿的位置也会同步前移

`self.temp_x_offset = 0.002  # 临时x方向偏置，每步叠加`

`current_torso_pos[0] += self.temp_x_offset * (step + 1)`

目前仿真版本不存在这个问题，可以正常启动

启动固定脚本`robot_climbstair.py`和`stairClimbPlanner-roban.py`时需将机器人放置在台阶前方约3cm的位置，不能太近也不能太远

## 仿真问题

目前gazebo仿真存在初始状态加载可能发散的问题，暂时不好定位，多开几次脚本就可以，或者在`kuavo-ros-control/src/gazebo/gazebo-sim/launch/gazebo-sim.launch`里编辑

`<arg name="paused" value="true"/>`

这样会暂停gazebo，在界面里点击开始或者空格键就基本能正常启动
