#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# 这个脚本用于将biped_s的urdf文件中的mesh文件路径修改为相对路径，并删除不需要的link和joint
# 输入：biped_s的urdf文件
# 输出：biped_s的urdf文件，其中mesh文件路径为相对路径，且只保留base和zarm两个link和joint

import xml.etree.ElementTree as ET
import os
import re

def process_urdf(input_file, output_file, version):
    # 解析XML文件
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    # 需要保留的link和joint前缀
    keep_prefixes = ['base', 'zarm']
    
    # 找出所有需要删除的elements
    to_remove = []
    for elem in root:
        if elem.tag in ['link', 'joint']:
            name = elem.get('name', '')
            if not any(name.startswith(prefix) for prefix in keep_prefixes):
                to_remove.append(elem)
    
    # 删除不需要的elements
    for elem in to_remove:
        root.remove(elem)
    
    # 修改mesh路径
    for mesh in root.findall(".//mesh"):
        filename = mesh.get('filename', '')
        if 'package://kuavo_assets/models/biped_s'+str(version) in filename:
            new_filename = filename.replace('package://kuavo_assets/models/biped_s'+str(version), '../..')
            mesh.set('filename', new_filename)
    
    # 写入新文件
    tree.write(output_file, encoding='utf-8', xml_declaration=True)

# 使用脚本
current_dir = os.path.dirname(os.path.abspath(__file__))
path = current_dir + '/../models/'

# 自动扫描models目录获取所有biped_s版本
biped_versions = []
try:
    # 列出models目录下的所有文件和文件夹
    models_dirs = os.listdir(path)
    # 筛选出biped_s开头的文件夹
    for dir_name in models_dirs:
        if dir_name.startswith('biped_s'):
            # 提取版本号
            version_match = re.search(r'biped_s(\d+)', dir_name)
            if version_match:
                version_num = int(version_match.group(1))
                biped_versions.append(version_num)
    
    # 按数字排序版本号
    biped_versions.sort()
    
except Exception as e:
    print(f"扫描目录时发生错误: {e}")
    # 如果自动扫描失败，使用默认版本列表
    biped_versions = [30, 31, 32, 40, 41, 42, 43, 45, 46, 47, 48, 49]

print(f"处理以下版本: {biped_versions}")

# 处理每个版本
for version in biped_versions:
    robot_version = 'biped_s' + str(version)
    input_file = path + robot_version + '/urdf/' + robot_version + '.urdf'
    output_file = path + robot_version + '/urdf/drake/biped_v3_arm.urdf'
    
    # 检查输入文件是否存在
    if os.path.exists(input_file):
        print(f"处理 {robot_version}...")
        process_urdf(input_file, output_file, version)
    else:
        print(f"跳过 {robot_version}: 找不到文件 {input_file}")
