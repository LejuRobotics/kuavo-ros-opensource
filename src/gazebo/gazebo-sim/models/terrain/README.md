
## 1. 生成方块拼接的不平地面 (`generate_uneven_terrain.py`)

这个工具使用随机高度的小方块拼接成不平整的地形。

### 基本用法

```bash
# 基本语法
rosrun gazebo_sim generate_uneven_terrain.py <输出文件路径> <长度> <宽度> <分辨率> <最小高度> <最大高度> [随机种子] [颜色] [材质类型]

# 示例1：生成10x10米的灰色不平地面，方块大小0.5米，高度范围0-0.3米
rosrun gazebo_sim generate_uneven_terrain.py $(rospack find gazebo_sim)/models/terrain/terrain_generated.sdf 10 10 0.5 0.0 0.3 42 Grey default

# 示例2：生成草地材质的不平地面
rosrun gazebo_sim generate_uneven_terrain.py $(rospack find gazebo_sim)/models/terrain/grass_terrain_uneven.sdf 8 8 0.2 0.0 0.2 42 Grass grass
```

### 参数说明

1. `<输出文件路径>`：生成的SDF文件保存路径
2. `<长度>`：地面长度（米）
3. `<宽度>`：地面宽度（米）
4. `<分辨率>`：网格分辨率（米），即每个方块的大小
5. `<最小高度>`：最小高度（米）
6. `<最大高度>`：最大高度（米）
7. `[随机种子]`：随机数种子，用于生成可重复的随机地形（可选参数）
8. `[颜色]`：地面颜色（Gazebo材质名称，可选参数，默认为Grey）
9. `[材质类型]`：材质类型，可选值为default（默认Gazebo材质）或grass（草地材质）

### 注意事项

- 分辨率越小，方块数量越多，生成的模型文件越大，加载时间也越长
- 对于大面积的地形，建议使用较大的分辨率（如0.5-1.0米）
- 随机种子可以确保每次生成相同的地形，便于实验的重复性
