# craic_simulator

CRAIC 仿真功能包，结构参考 `data_challenge_simulator`。

## 结构

- **package.xml** - 包描述与依赖
- **CMakeLists.txt** - 构建与安装（Python 脚本、launch）
- **launch/** - 启动文件
  - `load_kuavo_mujoco_craic.launch` - 加载 MuJoCo 仿真（默认 ROBOT_VERSION=47）
- **config/** - 场景配置
  - `craic_room.yaml` - CRAIC Lab 场景配置（房间、物体、材质、灯光等）
  - `README.md` - 配置文件结构详解
- **utils/** - Python 工具
  - `scene_builder.py` - 场景生成器（从 craic_room.yaml 生成 MuJoCo XML）
  - `topic_repeater.py` - 话题同步节点（/cmd_pose_world、/kuavo_arm_traj 带时间戳转发）
  - `lidar_mid360_node.py` - mid360 雷达仿真节点（订阅 /mujoco/qpos，发布 /lidar/points PointCloud2）
- **scripts/** - 辅助脚本
  - `patch_tibvh.py` - 修复 tibvh 库与 PyTorch 的 uint32 兼容性问题（启用 GPU 后端时需运行）
- **models/biped_s47/xml/** - 场景 XML
  - `my_scene.xml` - 仿真加载的场景（由 scene_builder 从 craic_room.yaml 生成）

## 编译与使用

### 1. 安装 MuJoCo-LiDAR（编译前需先安装）

雷达仿真依赖 MuJoCo-LiDAR，编译前需先安装：

```bash
# Docker 内执行
cd /root/kuavo_ws/src/MuJoCo-LiDAR
pip install -e ".[taichi]"
```

**启用 GPU 后端（可选）**：由于 `tibvh` 库与 PyTorch 存在兼容性问题，使用 GPU 后端前需运行 patch：

```bash
python3 /root/kuavo_ws/src/craic_simulator/scripts/patch_tibvh.py
```

> patch 只需运行一次。如果不需要 GPU 加速，使用默认的 `cpu` 后端即可。

### 2. 编译

```bash
# 若 build 报路径不一致，可先清理再编译
# catkin clean -b
catkin build craic_simulator
source devel/setup.bash
```

### 3. 启动仿真

```bash
# 启动 MuJoCo CRAIC 场景
roslaunch craic_simulator load_kuavo_mujoco_craic.launch

# 启动键盘控制（虚拟手柄/键盘，需依赖 joy_control_sim.launch，仿真模式）
roslaunch craic_simulator joy_control_sim.launch

# 运行话题同步节点（可选）
rosrun craic_simulator topic_repeater.py
```

### 4. 场景配置与生成

仿真使用的场景为 `models/biped_s47/xml/my_scene.xml`，由 **scene_builder** 从 `config/craic_room.yaml` 生成。修改场景需：

1. 编辑 `config/craic_room.yaml`（房间、物体、材质、灯光等）
2. 运行 scene_builder 生成 my_scene.xml

```bash
cd src/craic_simulator/utils
# 生成到 my_scene.xml（launch 加载此文件）
python3 /root/kuavo_ws/src/craic_simulator/utils/scene_builder.py /root/kuavo_ws/src/craic_simulator/config/craic_room.yaml
```

**scene_builder 用法**：
- `python3 scene_builder.py` - 使用默认 craic_room.yaml，输出到 `my_scene.xml`
- `python3 scene_builder.py <config>` - 指定配置文件路径
- `python3 scene_builder.py <config> -o <output>` - 指定配置和输出路径
- `python3 scene_builder.py --seed <N>` - 指定随机种子
- `python3 scene_builder.py --robot-version <ver>` - 指定机器人版本（默认从环境变量 ROBOT_VERSION 读取）
- `python3 scene_builder.py --help` - 查看帮助

**craic_room.yaml 主要配置项**：
- `scene_name` - 场景名称
- `camera` - 相机视角 (center, extent)
- `materials` - 材质（地面、墙面等）
- `ground` / `lights` - 地面与光源
- `rooms` - U 型走廊（外围房间 + 内部墙体）
- `objects` - 物体列表，支持：
  - `floor_marker` - 地面标记（起/终/暂停点）
  - `include` - 引用模型（托盘、篮子、手表、印章等）
    - `movable: true` - 可抓取，需配合 `mass` 与 `friction`
    - `mass` - 质量（kg），影响物理仿真
    - `friction` - 摩擦系数（格式 `"滑移 扭转 滚动"`），如 `"1.5 0.8 0.02"` 便于抓取
  - `slope` / `stairs` - 斜坡、楼梯
  - `gravel_road` / `speed_bump` / `traffic_cone` - 石子路、减速带、路障
  - `pillar` / `button_sphere` - 立柱、按钮

详细配置说明见 `config/README.md`。

### 修改机器人初始位置

**生效的是 launch 里的 `robot_init_state_param`**，不是 scene XML 的 keyframe。MuJoCo 节点会读取该 ROS 参数并覆盖场景中的初始姿态。

在 `load_kuavo_mujoco_craic.launch` 中修改：

```xml
<rosparam param="robot_init_state_param">[x, y, z, qw, qx, qy, qz, 关节...]</rosparam>
```

- 前 3 个数：世界系下位置 **(x, y, z)**
- 第 4–7 个数：姿态四元数 **(qw, qx, qy, qz)**，无旋转为 `1, 0, 0, 0`
- 后面为各关节角度，保持与当前 launch 中数量一致即可

例如把机器人放到 (2, 1)，高度 0.84：把数组前 3 个改成 `2, 1, 0.84414`，四元数保持 `0.99864, 0.0, 0.05215, 0.0` 不变。

## MuJoCo-LiDAR 雷达

biped_s47 模型在头部雷达位置已添加 **lidar_site**（与 radar mesh 同位置），可与 [MuJoCo-LiDAR](https://github.com/TATP-233/MuJoCo-LiDAR) 配合使用。

**代码示例**（按需在脚本中调用）：

```python
from mujoco_lidar import MjLidarWrapper, scan_gen

# 加载场景（craic_simulator 的 my_scene.xml）
mj_model = mujoco.MjModel.from_xml_path("path/to/my_scene.xml")
mj_data = mujoco.MjData(mj_model)

# 使用头部雷达 site，排除机身避免自检
exclude_body_id = mj_model.body("base_link").id
lidar = MjLidarWrapper(
    mj_model,
    site_name="lidar_site",
    backend="cpu",  # 或 "taichi"
    cutoff_dist=50.0,
    args={"bodyexclude": exclude_body_id},
)
rays_theta, rays_phi = scan_gen.generate_grid_scan_pattern(64, 16)
lidar.trace_rays(mj_data, rays_theta, rays_phi)
points = lidar.get_hit_points()
```

雷达 site 位于 `models/biped_s47/xml/biped_s47.xml` 的 `zhead_2_link` 下，与 radar 几何同 pose。

**仿真内 mid360 接入**：使用 `load_kuavo_mujoco_craic.launch` 时会自动启动 `lidar_mid360` 节点。仿真节点发布 `/mujoco/qpos`，雷达节点据此同步状态并用 MuJoCo-LiDAR 做射线追踪：
- **点云话题**：`/lidar/points`（`sensor_msgs/PointCloud2`）
- **坐标系**：点云在雷达坐标系下发布，同时广播 TF `parent_frame -> lidar_frame`
- **RViz 使用**：Fixed Frame 设为 `odom` 或 `world`，添加 PointCloud2 订阅 `/lidar/points` 即可看到点云

**可配置参数**（launch 或 rosparam）：
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `~lidar_frame` | `lidar` | 雷达坐标系名称 |
| `~parent_frame` | `world` | 父坐标系名称 |
| `~rate` | `10` | 发布频率（Hz） |
| `~backend` | `cpu` | 后端：`cpu`（稳定）或 `taichi`（GPU） |
| `~cutoff_dist` | `50.0` | 探测距离（米） |
| `~use_geomgroup` | `true` | 排除机器人自身：`true` 用 geomgroup 过滤（只检测 group=0），`false` 用 bodyexclude |

依赖 MuJoCo-LiDAR 的 `LivoxGenerator("mid360")` 与 `MjLidarWrapper`。

## 依赖

- catkin
- 使用 launch 时需：`nodelet`、`mujoco_cpp`、`kuavo_assets`
- 使用 topic_repeater 时需：`sensor_msgs`、`geometry_msgs`（ROS 标准）
- 使用 MuJoCo-LiDAR 时需：`mujoco`、`mujoco-lidar`（见 MuJoCo-LiDAR 仓库）
