# 围墙贴图目录

将围墙图片放在此目录，支持两种用法。

**注意**：MuJoCo 2D 纹理仅支持 **PNG** 格式。若为 JPG，请先转换：`python3 -c "from PIL import Image; Image.open('x.jpg').save('x.png')"`

## 方式 1：每面墙单独贴图（一墙一图，等比例铺满）

提供与墙壁 1:1 尺寸比例的图片，每面墙贴一张，不重复不拉伸：

```yaml
# 多面墙：texture_files 与 points 边顺序对应
outer_room:
  points: [[6.25,4.5], [-6.25,4.5], [-6.25,-4.5], [6.25,-4.5]]
  texture_files:
    - "models/wall_textures/wall_top.png"    # 上墙
    - "models/wall_textures/wall_left.png"   # 左墙
    - "models/wall_textures/wall_bottom.png" # 下墙
    - "models/wall_textures/wall_right.png"  # 右墙

# 单面墙
inner_top_wall:
  points: [[-3.25, 1.5], [3.25, 1.5]]
  texture_file: "models/wall_textures/inner_top.png"
```

图片尺寸建议与墙尺寸等比例（如墙 6.5m×1m → 图片 650×100 像素）。

## 方式 2：材质级贴图（所有墙共用，可平铺）

在 `materials.walls_mat` 中配置，所有使用该材质的墙共用同一贴图，可设置 texrepeat 平铺。
