## 键盘控制机器人运动 参数说明

  - 关于启动`load_kuavo_mujoco_sim.launch`及`load_kuavo_real.launch`时，参数`joystick_type`的选择说明

### 对于仿真
  - **参数的选择**：`load_kuavo_mujoco_sim.launch` 文件启动时参数 `joystick_type` 可选择 `h12` `bt2` `bt2pro` `sim` 分别对应h12遥控器，bt2遥控器，bt2pro遥控器和键盘控制。
    - 这里默认参数为`sim`，如果直接运行会弹出新终端用来从键盘输入运动控制指令。
    - 如果启动该launch文件时`joystick_type`参数选择了`h12`或`bt2`，可以新开一个终端，依次执行`cd kuavo-ros-control`，`sudo su`,`source devel/setup.bash`,`python3 src/kuavo_sdk/scripts/keyboard_control/robot_keyboard_control.py`，在此终端中也可以用键盘输入运动控制指令，此时遥控器和键盘均奏效。
    - 如果启动该launch文件时`joystick_type`参数选择了`bt2pro`，就不能使用键盘控制，因为json配置文件中按键映射不一致会导致键盘控制实失效，只能用遥控器控制。

### 对于实机
  - **参数的选择**： `load_kuavo_real.launch` 文件启动时参数 `joystick_type` 可选择 `h12` `bt2` `bt2pro` `sim` 分别对应h12遥控器，bt2遥控器，bt2pro遥控器和键盘控制。
    - 这里默认参数为`bt2`，如果启动该launch文件时`joystick_type`参数选择了`h12`或`bt2`，可以新开一个终端，依次执行`cd kuavo-ros-control`，`sudo su`,`source devel/setup.bash`,`python3 src/kuavo_sdk/scripts/keyboard_control/robot_keyboard_control.py`，在此终端中也可以用键盘输入运动控制指令，此时遥控器和键盘均奏效。
    - 如果启动launch文件时`joystick_type`参数选择了`sim`，会弹出新终端用来从键盘输入运动控制指令。
    - 如果启动该launch文件时`joystick_type`参数选择了`bt2pro`，就不能使用键盘控制，因为json配置文件中按键映射不一致会导致键盘控制实失效，只能用遥控器控制。
  - **特别**：
    - 当`joystick_type`参数不是`sim`时，不论使用上述哪种方式启动键盘控制，都不要在机器人身上插着北通手柄的接受器。