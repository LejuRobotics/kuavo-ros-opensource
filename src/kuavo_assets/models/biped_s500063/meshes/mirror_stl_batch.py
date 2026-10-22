#!/usr/bin/env python3
"""
Blender脚本：批量镜像STL文件
功能：扫描当前目录及meshes子目录下的STL文件，根据文件名首字母
      (l_* → r_*, r_* → l_*) 创建镜像副本
需要先安装 blender: sudo apt install blender
使用方法: blender --background --python mirror_stl_batch.py
"""

import bpy
import os

def mirror_stl(input_path, output_path):
    """将单个STL文件进行Y轴镜像并保存"""
    # 清除场景中的所有对象
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    
    # 导入STL文件
    print(f"导入文件: {input_path}")
    bpy.ops.import_mesh.stl(filepath=input_path)
    
    # 获取导入的对象
    obj = bpy.context.selected_objects[0]
    bpy.context.view_layer.objects.active = obj
    
    # 应用Y轴镜像（缩放方法）
    print("应用Y轴镜像...")
    obj.scale[1] = -1  # Y轴镜像
    
    # 应用缩放变换
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    
    # 翻转法线（镜像后法线会反向）
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.flip_normals()
    bpy.ops.object.mode_set(mode='OBJECT')
    
    # 导出为STL文件
    print(f"导出文件: {output_path}")
    bpy.ops.export_mesh.stl(filepath=output_path)
    print(f"✓ 完成: {os.path.basename(output_path)}")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 需要搜索的目录列表
    search_dirs = [
        script_dir,  # 当前目录
        os.path.join(script_dir, "meshes")  # meshes子目录
    ]
    
    processed_count = 0
    
    for directory in search_dirs:
        if not os.path.exists(directory):
            continue
            
        print(f"\n扫描目录: {directory}")
        
        # 遍历所有STL文件
        for filename in os.listdir(directory):
            if not filename.lower().endswith('.stl'):
                continue
            
            input_path = os.path.join(directory, filename)
            
            # 根据首字母确定输出文件名
            if filename.startswith('l_'):
                output_filename = 'r_' + filename[2:]  # 去掉l_前缀，加上r_
                output_path = os.path.join(directory, output_filename)
                
                # 检查是否已存在，避免重复处理
                if os.path.exists(output_path):
                    print(f"跳过: {output_filename} 已存在")
                    continue
                    
                print(f"\n处理: {filename} → {output_filename}")
                mirror_stl(input_path, output_path)
                processed_count += 1
                
            elif filename.startswith('r_'):
                output_filename = 'l_' + filename[2:]  # 去掉r_前缀，加上l_
                output_path = os.path.join(directory, output_filename)
                
                # 检查是否已存在，避免重复处理
                if os.path.exists(output_path):
                    print(f"跳过: {output_filename} 已存在")
                    continue
                    
                print(f"\n处理: {filename} → {output_filename}")
                mirror_stl(input_path, output_path)
                processed_count += 1
    
    print(f"\n✓ 处理完成，共生成 {processed_count} 个镜像文件")

if __name__ == "__main__":
    main()