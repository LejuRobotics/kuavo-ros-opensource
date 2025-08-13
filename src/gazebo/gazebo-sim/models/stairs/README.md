# 参数化楼梯模型

这是一个用于Gazebo仿真的参数化楼梯模型。可以通过修改参数来自定义楼梯的尺寸、台阶数量、材质、位置和角度等特性。现在还支持创建双段楼梯，形成上下楼梯的完整结构。

## 使用方法

使用楼梯模型分为两个步骤：
1. 手动生成楼梯模型SDF文件
2. 使用launch文件加载生成的模型

### 第一步：手动生成楼梯模型

首先需要使用提供的Python脚本生成楼梯模型SDF文件：

```bash
# 基本用法
rosrun gazebo_sim generate_stairs.py <输出文件路径> <台阶数量> <台阶宽度> <台阶高度> <台阶深度> [颜色] [材质类型]

# 示例1：生成5个台阶，宽度1米，高度0.2米，深度0.3米，灰色，默认材质
rosrun gazebo_sim generate_stairs.py $(rospack find gazebo_sim)/models/stairs/stairs_generated.sdf 5 1.0 0.2 0.3 Grey default

# 示例2：生成木质楼梯
rosrun gazebo_sim generate_stairs.py $(rospack find gazebo_sim)/models/stairs/wood_stairs.sdf 5 1.0 0.2 0.3 Brown wood
```

参数说明：
1. `<输出文件路径>`：生成的SDF文件保存路径
2. `<台阶数量>`：楼梯的台阶数量
3. `<台阶宽度>`：楼梯宽度（米）
4. `<台阶高度>`：每个台阶高度（米）
5. `<台阶深度>`：每个台阶深度（米）
6. `[颜色]`：楼梯颜色（Gazebo材质名称，可选参数，默认为Grey）
7. `[材质类型]`：材质类型，可选值为default（默认Gazebo材质）或wood（木质材质）

### 第二步：使用launch文件加载模型

生成SDF文件后，可以通过launch文件加载到Gazebo中：

```bash
# 在完整仿真环境中加载楼梯
roslaunch gazebo_sim gazebo-sim.launch use_stairs:=true

# 单独加载楼梯模型
roslaunch gazebo_sim load_stairs.launch
```

### 自定义楼梯位置和角度

可以通过参数自定义楼梯的位置和角度：

```bash
# 设置楼梯位置
roslaunch gazebo_sim gazebo-sim.launch use_stairs:=true stairs_x:=2.0 stairs_y:=0.5 stairs_z:=0.1

# 设置楼梯角度（弧度）
roslaunch gazebo_sim gazebo-sim.launch use_stairs:=true stairs_yaw:=1.57 # 绕Z轴旋转90度
roslaunch gazebo_sim gazebo-sim.launch use_stairs:=true stairs_roll:=0.3 stairs_pitch:=0.1 stairs_yaw:=1.57
```

### 创建双段楼梯（上下楼梯）

现在支持创建一个完整的上下楼梯结构，由两段楼梯拼接而成：

```bash
# 先生成两种不同材质的楼梯模型
rosrun gazebo_sim generate_stairs.py $(rospack find gazebo_sim)/models/stairs/stairs_generated.sdf 5 1.0 0.2 0.3 Grey default
rosrun gazebo_sim generate_stairs.py $(rospack find gazebo_sim)/models/stairs/wood_stairs.sdf 5 1.0 0.2 0.3 Brown wood

# 加载双段楼梯（默认第一段使用普通材质，第二段使用木质材质）
roslaunch gazebo_sim gazebo-sim.launch use_stairs:=true use_stairs2:=true

# 自定义双段楼梯参数
roslaunch gazebo_sim gazebo-sim.launch use_stairs:=true use_stairs2:=true num_steps:=7 step_height:=0.15 step_depth:=0.25 stairs_x:=1.0
```

系统会自动计算第二段楼梯的位置，使其与第一段楼梯完美拼接，形成一个上下楼梯的整体结构。第一段为上楼梯，第二段为下楼梯。

### 使用木质材质的楼梯

```bash
# 先生成木质楼梯模型
rosrun gazebo_sim generate_stairs.py $(rospack find gazebo_sim)/models/stairs/wood_stairs.sdf 5 1.0 0.2 0.3 Brown wood

# 然后加载木质楼梯模型
roslaunch gazebo_sim gazebo-sim.launch use_stairs:=true stairs_model_path:=$(rospack find gazebo_sim)/models/stairs/wood_stairs.sdf
```

### 使用自定义SDF模型文件

如果您有自己生成或修改的楼梯模型文件，可以通过以下方式加载：

```bash
roslaunch gazebo_sim gazebo-sim.launch use_stairs:=true stairs_model_path:=<您的SDF文件路径>
```

## 参数说明

launch文件支持以下参数：

### 第一段楼梯（上楼梯）参数
- `use_stairs`（默认：false在gazebo-sim.launch中，true在load_stairs.launch中）：是否加载楼梯模型
- `stairs_model_path`（默认：$(find gazebo_sim)/models/stairs/stairs_generated.sdf）：楼梯模型SDF文件路径
- `stairs_x`（默认：1.5）：楼梯位置X坐标
- `stairs_y`（默认：0.0）：楼梯位置Y坐标
- `stairs_z`（默认：0.0）：楼梯位置Z坐标
- `stairs_roll`（默认：0.0）：楼梯绕X轴旋转角度（弧度）
- `stairs_pitch`（默认：0.0）：楼梯绕Y轴旋转角度（弧度）
- `stairs_yaw`（默认：0.0）：楼梯绕Z轴旋转角度（弧度）

### 第二段楼梯（下楼梯）参数
- `use_stairs2`（默认：false在gazebo-sim.launch中，true在load_stairs.launch中）：是否加载第二段楼梯模型
- `stairs2_model_path`（默认：$(find gazebo_sim)/models/stairs/wood_stairs.sdf）：第二段楼梯模型SDF文件路径
- `num_steps`（默认：5）：楼梯的台阶数量（用于计算拼接位置）
- `step_depth`（默认：0.3）：楼梯台阶深度（用于计算拼接位置）
- `step_height`（默认：0.2）：楼梯台阶高度（用于计算拼接位置）

第二段楼梯的位置和方向会根据第一段楼梯的参数自动计算，确保两段楼梯能够完美拼接成一个整体。

## 文件结构

- `load_stairs.launch`：专门用于加载楼梯模型的launch文件
- `gazebo-sim.launch`：加载完整仿真环境的主launch文件，可选择性地包含楼梯
- `models/stairs/`：包含楼梯模型的目录
- `scripts/generate_stairs.py`：生成楼梯模型的Python脚本
- `models/stairs/stairs_generated.sdf`：默认生成的楼梯模型
- `models/stairs/wood_stairs.sdf`：木质材质楼梯模型示例
