# PICO Motion Retarget Launch 使用说明

## 功能说明

这个 launch 文件启动了两个 ROS 节点：
1. **pico_comm_minimal** - PICO 数据接收器
2. **pico_realtime_retarget** - 运动重定向与可视化节点（在 gmr_venv 虚拟环境中运行）

## 启动方式

### 1. 基本启动（启用可视化，默认）

```bash
source devel/setup.bash
roslaunch kuavo_pico_gmr pico_motion_retarget.launch
```

### 2. 禁用可视化

```bash
roslaunch kuavo_pico_gmr pico_motion_retarget.launch visualize:=false
```

### 3. 启用可视化（显式指定）

```bash
roslaunch kuavo_pico_gmr pico_motion_retarget.launch visualize:=true
```

## 参数说明

### visualize (bool)
- **默认值**: `true`
- **作用**: 控制是否启用 Mujoco 可视化
- **实现**: 
  - 当 `visualize:=true` 时，不传递任何额外参数给 Python 脚本
  - 当 `visualize:=false` 时，传递 `--no-viz` 参数给 Python 脚本

## 技术实现

### Wrapper 脚本

`pico_realtime_to_robot_wrapper.sh` 包装了 Python 脚本：
1. 自动激活 `gmr_venv` 虚拟环境
2. 传递所有命令行参数到 Python 脚本
3. 输出调试信息（脚本目录、包目录、传入参数）

### Launch 文件参数传递

使用 ROS launch 的条件表达式：
```xml
args="$(eval '--no-viz' if not arg('visualize') else '')"
```
- 如果 `visualize=false`，则 `args="--no-viz"`
- 如果 `visualize=true`，则 `args=""`（空字符串，不传递参数）

## 调试

启动后，wrapper 脚本会输出以下调试信息：
- `SCRIPT_DIR`: 脚本所在目录
- `PACKAGE_DIR`: 功能包根目录
- `传入参数`: 从 launch 文件接收到的所有参数

查看这些信息可以帮助确认参数是否正确传递。

## 注意事项

1. 确保已编译功能包：`catkin build kuavo_pico_gmr`
2. 确保已 source 工作空间：`source devel/setup.bash`
3. 确保 `gmr_venv` 虚拟环境已正确安装所有依赖
4. wrapper 脚本必须有可执行权限（已配置）

