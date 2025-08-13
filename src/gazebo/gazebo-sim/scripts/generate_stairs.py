#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import math

def generate_stairs_sdf(num_steps, step_width, step_height, step_depth, step_color="Grey", material_type="default"):
    """
    生成楼梯的SDF模型内容
    
    参数:
        num_steps (int): 楼梯台阶数量
        step_width (float): 楼梯宽度（米）
        step_height (float): 每个台阶高度（米）
        step_depth (float): 每个台阶深度（米）
        step_color (str): 楼梯颜色（Gazebo材质名称）
        material_type (str): 材质类型，可选值为 "default"（使用Gazebo内置材质）或 "wood"（木质材质）
    
    返回:
        str: SDF模型内容
    """
    
    sdf_header = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="parametric_stairs">
    <static>true</static>
"""
    
    sdf_footer = """  </model>
</sdf>
"""
    
    sdf_content = sdf_header
    
    # 为每个台阶创建一个链接
    for step_idx in range(num_steps):
        step_z_pos = step_height * (step_idx + 0.5)  # 台阶中心的z坐标
        step_x_pos = step_depth * (step_idx + 0.5)   # 台阶中心的x坐标
        
        link_name = f"step_{step_idx}"
        
        # 创建每个台阶的SDF链接
        step_link = f"""    <link name="{link_name}">
      <pose>{step_x_pos} 0 {step_z_pos} 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>{step_depth} {step_width} {step_height}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{step_depth} {step_width} {step_height}</size>
          </box>
        </geometry>
"""

        # 根据材质类型选择不同的材质定义
        if material_type.lower() == "wood":
            # 使用木质材质
            step_link += f"""        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Wood</name>
          </script>
          <ambient>0.8 0.6 0.4 1</ambient>
          <diffuse>0.8 0.6 0.4 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
"""
        else:
            # 使用默认Gazebo材质
            step_link += f"""        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/{step_color}</name>
          </script>
        </material>
"""
            
        step_link += """      </visual>
    </link>
"""
        sdf_content += step_link
    
    sdf_content += sdf_footer
    return sdf_content

def main():
    if len(sys.argv) < 6:
        print("用法: %s <输出文件> <台阶数量> <台阶宽度> <台阶高度> <台阶深度> [颜色] [材质类型]" % sys.argv[0])
        print("材质类型可以是 'default' 或 'wood'")
        sys.exit(1)
    
    output_file = sys.argv[1]
    num_steps = int(sys.argv[2])
    step_width = float(sys.argv[3])
    step_height = float(sys.argv[4])
    step_depth = float(sys.argv[5])
    
    step_color = "Grey"
    if len(sys.argv) > 6:
        step_color = sys.argv[6]
    
    material_type = "default"
    if len(sys.argv) > 7:
        material_type = sys.argv[7]
    
    sdf_content = generate_stairs_sdf(num_steps, step_width, step_height, step_depth, step_color, material_type)
    
    with open(output_file, 'w') as f:
        f.write(sdf_content)
    
    print("已生成楼梯SDF模型: %s" % output_file)
    print("楼梯参数：台阶数量=%d, 宽度=%.2f, 高度=%.2f, 深度=%.2f, 颜色=%s, 材质类型=%s" % (num_steps, step_width, step_height, step_depth, step_color, material_type))

if __name__ == "__main__":
    main() 
