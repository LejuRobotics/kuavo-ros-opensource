#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import math
import random

def generate_uneven_terrain_sdf(length, width, resolution, min_height, max_height, seed=None, terrain_color="Grey", material_type="default"):
    """
    生成不平地面的SDF模型内容
    
    参数:
        length (float): 地面长度（米）
        width (float): 地面宽度（米）
        resolution (float): 网格分辨率（米），表示每个方块的大小
        min_height (float): 最小高度（米）
        max_height (float): 最大高度（米）
        seed (int): 随机数种子，用于生成可重复的随机地形
        terrain_color (str): 地面颜色（Gazebo材质名称）
        material_type (str): 材质类型，可选值为 "default"（使用Gazebo内置材质）或 "grass"（草地材质）
    
    返回:
        str: SDF模型内容
    """
    
    if seed is not None:
        random.seed(seed)
    
    sdf_header = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="uneven_terrain">
    <static>true</static>
"""
    
    sdf_footer = """  </model>
</sdf>
"""
    
    sdf_content = sdf_header
    
    # 计算网格数量
    cols = int(length / resolution)
    rows = int(width / resolution)
    
    # 为每个网格创建一个链接
    for row in range(rows):
        for col in range(cols):
            # 计算当前网格的位置
            x_pos = col * resolution + resolution/2
            y_pos = row * resolution - width/2 + resolution/2
            
            # 生成随机高度
            # 使用柏林噪声或其他平滑函数会更好，但为简单起见，这里使用简单随机
            height = random.uniform(min_height, max_height)
            z_pos = height / 2  # 中心位置是高度的一半
            
            link_name = f"block_{row}_{col}"
            
            # 创建每个网格块的SDF链接
            block_link = f"""    <link name="{link_name}">
      <pose>{x_pos} {y_pos} {z_pos} 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>{resolution} {resolution} {height}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{resolution} {resolution} {height}</size>
          </box>
        </geometry>
"""

            # 根据材质类型选择不同的材质定义
            if material_type.lower() == "grass":
                # 使用草地材质
                block_link += f"""        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <n>Gazebo/Grass</n>
          </script>
        </material>
"""
            else:
                # 使用默认Gazebo材质
                block_link += f"""        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <n>Gazebo/{terrain_color}</n>
          </script>
        </material>
"""
                
            block_link += """      </visual>
    </link>
"""
            sdf_content += block_link
    
    sdf_content += sdf_footer
    return sdf_content

def main():
    if len(sys.argv) < 6:
        print("用法: %s <输出文件> <长度> <宽度> <分辨率> <最小高度> <最大高度> [随机种子] [颜色] [材质类型]" % sys.argv[0])
        print("材质类型可以是 'default' 或 'grass'")
        sys.exit(1)
    
    output_file = sys.argv[1]
    length = float(sys.argv[2])
    width = float(sys.argv[3])
    resolution = float(sys.argv[4])
    min_height = float(sys.argv[5])
    max_height = float(sys.argv[6])
    
    seed = None
    if len(sys.argv) > 7:
        seed = int(sys.argv[7])
    
    terrain_color = "Grey"
    if len(sys.argv) > 8:
        terrain_color = sys.argv[8]
    
    material_type = "default"
    if len(sys.argv) > 9:
        material_type = sys.argv[9]
    
    sdf_content = generate_uneven_terrain_sdf(length, width, resolution, min_height, max_height, seed, terrain_color, material_type)
    
    with open(output_file, 'w') as f:
        f.write(sdf_content)
    
    print("已生成不平地面SDF模型: %s" % output_file)
    print("地面参数：长度=%.2f, 宽度=%.2f, 分辨率=%.2f, 最小高度=%.2f, 最大高度=%.2f, 随机种子=%s, 颜色=%s, 材质类型=%s" 
          % (length, width, resolution, min_height, max_height, seed, terrain_color, material_type))

if __name__ == "__main__":
    main() 
