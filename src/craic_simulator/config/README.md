# MuJoCo 场景配置文件说明

## 概述

这个目录包含用于生成 MuJoCo 仿真场景的 YAML 配置文件。通过简单的配置文件，你可以快速创建复杂的场景，包括房间、墙壁、物体等。

## 使用方法

### 方法1：生成默认 CRAIC 场景（推荐）

```bash
cd src/craic_simulator
python3 utils/scene_builder.py
```

这会自动读取 `config/craic_room.yaml`，生成场景到 `models/<robot_version>/xml/my_scene.xml`。
机器人版本从环境变量 `ROBOT_VERSION` 读取。

### 方法2：指定配置文件和输出路径

```bash
cd src/craic_simulator
python3 utils/scene_builder.py config/craic_room.yaml -o models/biped_s52/xml/my_scene.xml --robot-version 52
```

### 方法3：同时生成两个版本

```bash
cd src/craic_simulator
python3 utils/scene_builder.py config/craic_room.yaml -o models/biped_s52/xml/my_scene.xml --robot-version 52
python3 utils/scene_builder.py config/craic_room.yaml -o models/biped_s47/xml/my_scene.xml --robot-version 47
```

### 方法4：指定随机种子和机器人版本

```bash
python3 scene_builder.py --seed 42 --robot-version biped_s52
```

### 方法5：查看帮助

```bash
python3 scene_builder.py --help
```

### 方法5：编程方式生成

```python
from scene_builder import SceneBuilder

builder = SceneBuilder()
builder.load_config('config/craic_room.yaml')
builder.build()
builder.save('output/scene.xml')
```

## 配置文件结构

### 基本结构

```yaml
scene_name: "场景名称"

visual:
  # 视觉设置
  headlight:
    diffuse: "0.6 0.6 0.6"
    ambient: "0.3 0.3 0.3"
  azimuth: 160
  elevation: -20

materials:
  # 材质定义
  material_name:
    rgba: "r g b a"
    reflectance: 0.1

ground:
  # 地面设置
  enabled: true
  material: "ground_mat"

lights:
  # 光源列表
  - name: "light1"
    pos: "x y z"
    dir: "dx dy dz"

rooms:
  # 房间定义
  room_name:
    points:
      - [x1, y1]
      - [x2, y2]
      # ...
    height: 3.0
    wall_thickness: 0.05

objects:
  # 物体列表
  - type: "box"
    name: "object1"
    pos: "x y z"
    # ...

keyframe:
  # 机器人初始位置
  name: "home"
  qpos: "x y z qw qx qy qz ..."
```

## 材质与围墙贴图

### 方式 1：每面墙单独贴图（一墙一图，拉伸铺满）

每面墙使用不同图片，`texrepeat="1 1"` 拉伸铺满整面墙。贴图仅显示在墙壁内侧（面向房间内部），外侧为纯色材质。

```yaml
rooms:
  outer_room:
    points: [[6.25,4.5], [-6.25,4.5], [-6.25,-4.5], [6.25,-4.5]]
    texture_files:  # 与 points 边顺序对应：上|左|下|右
      - "models/wall_textures/1.png"
      - "models/wall_textures/2.png"
      - "models/wall_textures/3.png"
      - "models/wall_textures/4.png"
  inner_top_wall:
    points: [[-3.25, 1.5], [3.25, 1.5]]
    texture_file: "models/wall_textures/inner_top.png"  # 单面墙用 texture_file
```

实现原理：每面贴图墙由两个 geom 组成——主体 box（纯色材质，负责碰撞和外侧视觉）+ 内侧极薄装饰 box（贴图，仅视觉无碰撞）。

### 方式 2：材质级贴图（所有墙共用，可平铺）

```yaml
materials:
  walls_mat:
    rgba: "0.85 0.75 0.60 1"
    texture_file: "models/wall_textures/wall.png"
    texrepeat: "4 4"   # 贴图平铺次数
```

将贴图放入 `models/wall_textures/` 目录，路径以 `craic_simulator` 为根。若文件不存在，则回退为纯色。

## 房间定义详解

### 矩形房间

通过4个点定义矩形房间：

```yaml
rooms:
  my_room:
    points:
      - [5, 5]    # 右上角
      - [-5, 5]   # 左上角
      - [-5, -5]  # 左下角
      - [5, -5]   # 右下角
    height: 3.0
    wall_thickness: 0.05
    material: "walls_mat"
```

这会自动生成4面墙，围成一个 10m x 10m 的房间。

### U型房间

通过外围矩形 + 内部矩形形成U型走廊：

```yaml
rooms:
  # 外围大房间
  outer_room:
    points:
      - [5, 4]      # 右上
      - [-5, 4]     # 左上
      - [-5, -4]    # 左下
      - [5, -4]     # 右下
    height: 3.0
    wall_thickness: 0.05
  
  # 内部矩形障碍物（靠近上边）
  inner_block:
    points:
      - [3, 3]      # 右上（靠近外围）
      - [-3, 3]     # 左上（靠近外围）
      - [-3, 1]     # 左下
      - [3, 1]      # 右下
    height: 2.5
    wall_thickness: 0.05
```

这会形成一个U型走廊：
```
外围房间边界
┌─────────────────┐
│  ┌───────────┐  │  ← 内部矩形障碍物
│  │           │  │
│  │           │  │
│  └───────────┘  │
│                 │  ← U型走廊
│                 │
│       ↑         │  ← 开口（机器人入口）
└─────────────────┘
```

走廊宽度 = (外围尺寸 - 内部尺寸) / 2

### 复杂多边形房间

可以定义任意多边形：

```yaml
rooms:
  hexagon:
    points:
      - [2, 0]
      - [1, 1.732]
      - [-1, 1.732]
      - [-2, 0]
      - [-1, -1.732]
      - [1, -1.732]
    height: 3.0
    wall_thickness: 0.05
```

## 物体类型

### 基础几何体

#### 盒子 (box)

```yaml
- type: "box"
  name: "box1"
  pos: "1 0 0.5"
  size: "0.2 0.2 0.2"  # 半尺寸
  rgba: "0.8 0.2 0.2 1"
  mass: 1.0
  movable: true  # 是否可移动
```

#### 圆柱 (cylinder)

```yaml
- type: "cylinder"
  name: "cylinder1"
  pos: "0 0 0.5"
  size: "0.1 0.2"  # 半径 高度
  rgba: "0.2 0.8 0.2 1"
  mass: 0.5
  movable: true
```

#### 球体 (sphere)

```yaml
- type: "sphere"
  name: "ball1"
  pos: "0 0 1.0"
  radius: 0.1
  rgba: "0.2 0.2 0.8 1"
  mass: 0.3
  movable: true
```

#### 桌子 (table)

```yaml
- type: "table"
  name: "table1"
  pos: "1 0 0.85"
  top_size: [0.4, 0.8, 0.02]  # 长 宽 厚
  leg_radius: 0.03
  leg_height: 0.4
  top_material: "table_ceramic"
  leg_material: "table_legs_metal"
```

### 地形类

#### 石子路 (gravel_road / hfield)

高度场地形，模拟不平整路面。

```yaml
- type: "gravel_road"
  name: "gravel_road_1"
  pos: "2.0 0.0 -0.03"            # 底面位置，z 略低于地面
  euler: "0 0 1.5708"             # 旋转角度（弧度）
  size: "1.25 1.05 0.05 0.1"      # 半宽 半长 最大高度 基础高度
  geom_size: "1.25 1.05 1"        # geom 尺寸
  rgba: "0.5 0.5 0.5 1"           # 颜色
  friction: "1 0.1 0.1"           # 摩擦系数
  hfield_file: "models/gravel_road/mujoco_assets/road_heightfield.png"
```

#### 斜坡 (slope)

```yaml
- type: "slope"
  name: "ramp_1"
  pos: "3.0 0.0 0.0"
  euler: "0 0 0"
  rgba: "0.6 0.6 0.6 1"
  mesh_file: "models/slope/xml/xiepo.stl"
```

#### 楼梯 (stairs)

使用 box 模式（仿 kuavo scene），踏面 0.28m×1.5m，台阶高 0.13m，4级上+平台1m+4级下。

```yaml
- type: "stairs"
  name: "stairs"
  pos: "4.75 -3.5 0"
  euler: "0 0 3.14159"
  rgba: "0.76 0.60 0.42 1"
```

### 障碍物类

#### 减速带 (speed_bump)

```yaml
- type: "speed_bump"
  name: "speed_bump_1"
  pos: "2.0 0.0 0.0"
  quat: "0.5 0.5 0.5 0.5"         # 旋转（平躺且长度沿 X 轴）
  target_size: "0.35 2.0 0.05"    # 长(沿道路) 宽(横跨道路) 高(凸起)
  rgba: "0.3 0.3 0.3 1"           # 深灰色
  friction: "1 0.5 0.01"
  mesh_file: "models/speed_bump/uploads_files_2559840_Lezachiq.obj"
```

#### 锥形路障 (traffic_cone)

```yaml
- type: "traffic_cone"
  name: "traffic_cone_1"
  pos: "3.0 2.0 0.0"
  euler: "-1.5708 0 0"            # 绕 X 轴 -90°，让模型立起来
  target_size: "0.33 0.6"         # 底部直径 高度（米）
  rgba: "1.0 0.4 0.0 1"           # 橙色
  mesh_file: "models/traffic_cone/uploads_files_3223633_traffic+cone.obj"
```

#### 立柱 (pillar / column)

```yaml
- type: "pillar"
  name: "pillar_1"
  pos: "0.0 0.0 0.0"              # 底面中心，z=0 表示贴地
  target_size: "0.4 0.4 1.0"      # 长 宽 高（米）
  rgba: "0.45 0.45 0.5 1"         # 灰白色
```

### 交互类

#### 按钮半球 (button_sphere / hemisphere)

用于交互任务，机器人按压后会触发状态变化（亮→暗）。

```yaml
- type: "button_sphere"
  name: "button_green"
  pos: "0.0 0.0 1.0"              # 球心位置（放在立柱顶部时 z = 立柱高度）
  diameter: "0.1"                  # 直径（米）
  rgba: "0 1 0 1"                  # 绿色
  bright: true                     # 初始状态：亮(true) 或 暗(false)
```

**工作原理：**
- 半球是一个完整的球体，下半部分嵌入立柱（视觉上呈半球）
- 亮态：高 emission（发光），暗态：低亮度
- 任何物体施加超过阈值的力（默认 2.5N）会触发亮→暗
- 力阈值可在 launch 文件中配置：`<param name="button_force_threshold" value="2.5"/>`

#### 二维码/AprilTag (qrcode)

```yaml
- type: "qrcode"
  name: "apriltag_1"
  pos: "3.0 1.5 0.07"
  size: "0.07 0.07 0.005"         # 宽 高 厚度
  image_file: "models/slope/xml/april_36h11-1.png"
```

### 外部模型 (include)

从外部 XML 文件加载复杂模型（如托盘、篮子、手表、印章等）。可抓取物体需设置 `movable: true`，并配合 `mass`、`friction` 控制物理特性。

```yaml
- type: "include"
  name: "tray_1"
  file: "models/tray/model.xml"
  pos: "-0.10 -2.0 0.75"
  euler: "0 1.5708 0"
  target_size: "0.2 0.03"         # 目标尺寸（可选）
  movable: true                   # 可移动/可抓取
  mass: 0.5                       # 质量（kg），影响物理仿真
  friction: "1.5 0.8 0.02"        # 摩擦系数（滑移 扭转 滚动），增大便于抓取
```

**可抓取物体的物理参数**：
- `mass` - 质量（kg），如托盘 0.5、篮子 0.3、小件 0.05
- `diaginertia` - 惯性对角元（可选），如 `"0.0001 0.0001 0.0001"`；不指定时按 `mass * 0.02` 估算
- `friction` - 字符串格式 `"滑移 扭转 滚动"`，如手表 `"1.5 0.8 0.02"`、印章 `"1.8 1.0 0.03"`
- `solref` - 接触求解器参考（可选），默认 `"0.02 1"`，提高抓取稳定性
- `solimp` - 接触求解器阻抗（可选），默认 `"0.9 0.95 0.001"`

## 机器人初始位置

通过 `keyframe` 设置机器人初始位置和姿态：

```yaml
keyframe:
  name: "home"
  qpos: "x y z qw qx qy qz joint1 joint2 ..."
```

- 前3个数：位置 (x, y, z)
- 第4-7个数：四元数姿态 (qw, qx, qy, qz)
- 后续数字：各关节角度

### 示例

```yaml
# 机器人在原点，高度0.98m，无旋转
qpos: "0 0 0.98 1 0 0 0 0 0 0 0 ..."

# 机器人在 (2, 3) 位置
qpos: "2 3 0.98 1 0 0 0 0 0 0 0 ..."

# 机器人绕Z轴旋转90度
qpos: "0 0 0.98 0.7071 0 0 0.7071 0 0 0 ..."
```

## 示例场景

### craic_room.yaml

CRAIC 机器人标准仿真场景，包含：
- U型走廊布局
- 桌子、托盘、篮子
- 可抓取物体（手表、印章等）
- 地形障碍（石子路、减速带、斜坡、楼梯）
- 交互按钮（立柱 + 半球按钮）
- 锥形路障

这是默认的标准场景配置。

## 旋转说明

MuJoCo 使用**弧度制**的欧拉角：

| 角度 | 弧度 |
|------|------|
| 90°  | 1.5708 (π/2) |
| 180° | 3.1416 (π) |
| -90° | -1.5708 |

```yaml
# 绕 Z 轴旋转 90°
euler: "0 0 1.5708"

# 绕 X 轴旋转 -90°（常用于让 Y-up 模型立起来）
euler: "-1.5708 0 0"

# 组合旋转：绕 X 轴 90° + 绕 Z 轴 90°（用四元数更精确）
quat: "0.5 0.5 0.5 0.5"
```

## 按钮交互系统

场景中的 `button_sphere` 类型物体支持力感应交互：

1. **配置按钮**：在 YAML 中添加 `button_sphere`，设置 `bright: true/false`
2. **触发条件**：任何物体施加的力超过阈值（默认 2.5N）
3. **效果**：亮的按钮变暗（emission 降低，颜色变深）
4. **调整阈值**：在 launch 文件中修改 `button_force_threshold`

```xml
<param name="button_force_threshold" value="2.5"/>
```

## 提示

1. **墙壁厚度**：建议使用 0.05m (5cm)
2. **房间高度**：通常 2.5-5.0m
3. **点的顺序**：按顺时针或逆时针顺序定义点
4. **坐标系**：X轴向前，Y轴向左，Z轴向上
5. **单位**：所有距离单位为米(m)
6. **旋转**：euler 使用弧度制，复杂旋转建议用 quat

## 获取模型尺寸

对于 OBJ/STL 模型，可以使用辅助脚本获取原始尺寸：

```bash
cd src/craic_simulator/models/speed_bump
python3 get_obj_size.py
```

输出示例：
```
OBJ: uploads_files_2559840_Lezachiq.obj
  X: -0.32 ~ 0.32  宽度 = 0.64
  Y: -0.00 ~ 0.16  深度 = 0.16
  Z: -1.60 ~ 1.80  高度/长度 = 3.41
  半尺寸 (MuJoCo size 常用): 0.32 0.08 1.70
```

## 调试技巧

1. 先创建简单的矩形房间测试
2. 逐步添加物体
3. 使用 MuJoCo 可视化工具查看生成的场景
4. 检查墙壁是否正确闭合
5. 确保物体不与墙壁重叠
6. 模型姿态不对时，尝试不同的 euler 旋转组合
