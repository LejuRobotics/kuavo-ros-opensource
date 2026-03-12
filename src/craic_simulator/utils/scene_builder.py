#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
优雅的 MuJoCo 场景构建器
支持从配置文件读取场景信息，自动生成房间、墙壁等
"""

import os
import random
import yaml
import json
import struct
import xml.etree.ElementTree as ET
from xml.dom import minidom
import numpy as np
from pathlib import Path


class SceneBuilder:
    """场景构建器"""
    
    def __init__(self, robot_xml=None, robot_version=None):
        if robot_version is None:
            robot_version = os.environ.get('ROBOT_VERSION', '47')
        self.robot_version = str(robot_version)
        if robot_xml is None:
            robot_xml = f"biped_s{self.robot_version}.xml"
        self.robot_xml = robot_xml
        self.robot_model_dir = f"biped_s{self.robot_version}"
        self.root = None
        self.worldbody = None
        self.asset = None
        self.actuator = None
        self.config = {}
        
    def load_config(self, config_file):
        """
        加载配置文件
        Args:
            config_file: YAML 或 JSON 配置文件路径
        """
        config_path = Path(config_file)
        
        if config_path.suffix in ['.yaml', '.yml']:
            with open(config_file, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f)
        elif config_path.suffix == '.json':
            with open(config_file, 'r', encoding='utf-8') as f:
                self.config = json.load(f)
        else:
            raise ValueError(f"Unsupported config format: {config_path.suffix}")
        
        return self
    
    def _init_xml(self):
        """初始化 XML 结构"""
        model_name = self.config.get('scene_name', 'kuavo scene')
        self.root = ET.Element('mujoco', model=model_name)
        
        # 包含机器人
        ET.SubElement(self.root, 'include', file=self.robot_xml)
        
        # 编译器选项：自动加载 home keyframe
        compiler = ET.SubElement(self.root, 'compiler')
        compiler.set('angle', 'radian')
        compiler.set('autolimits', 'true')
        
        # 统计和视觉（控制相机视角）
        camera_cfg = self.config.get('camera', {})
        ET.SubElement(self.root, 'statistic',
                     center=camera_cfg.get('center', '0 0 1'),
                     extent=camera_cfg.get('extent', '1.8'))
        self._setup_visual()
        
        # 资产和世界体
        self.asset = ET.SubElement(self.root, 'asset')
        self.worldbody = ET.SubElement(self.root, 'worldbody')
    
    def _setup_visual(self):
        """设置视觉参数"""
        visual_cfg = self.config.get('visual', {})
        visual = ET.SubElement(self.root, 'visual')
        
        headlight = visual_cfg.get('headlight', {})
        ET.SubElement(visual, 'headlight',
                     diffuse=headlight.get('diffuse', '0.6 0.6 0.6'),
                     ambient=headlight.get('ambient', '0.3 0.3 0.3'),
                     specular=headlight.get('specular', '0 0 0'))
        
        ET.SubElement(visual, 'rgba', haze=visual_cfg.get('haze', '0.15 0.25 0.35 1'))
        ET.SubElement(visual, 'global',
                     azimuth=str(visual_cfg.get('azimuth', 160)),
                     elevation=str(visual_cfg.get('elevation', -20)))
    
    def _add_materials(self):
        """添加材质。支持 texture_file：围墙贴图时创建 texture 并引用"""
        materials = self.config.get('materials', {})
        script_dir = Path(__file__).resolve().parent
        base_dir = script_dir.parent  # craic_simulator
        output_dir = base_dir / 'models' / self.robot_model_dir / 'xml'

        for name, props in materials.items():
            props = dict(props)
            texture_file = props.pop('texture_file', None)
            texrepeat = props.pop('texrepeat', '1 1')

            if texture_file:
                # 解析纹理路径：先在 output_dir（xml 目录）查找，再到 base_dir 查找
                tex_path = (output_dir / texture_file).resolve()
                if not tex_path.exists():
                    tex_path = (base_dir / texture_file).resolve()
                if tex_path.exists():
                    rel_path = os.path.relpath(tex_path, output_dir) if output_dir.exists() else texture_file
                    tex_name = f"{name}_tex"
                    ET.SubElement(self.asset, 'texture', type='2d', name=tex_name, file=str(rel_path))
                    attrib = {'name': name, 'texture': tex_name, 'texrepeat': texrepeat}
                    for key, value in props.items():
                        attrib[key] = str(value)
                    ET.SubElement(self.asset, 'material', **attrib)
                    print(f"✓ Material '{name}' with texture: {texture_file}")
                else:
                    print(f"Warning: texture file not found: {tex_path}, using rgba only")
                    attrib = {'name': name}
                    for key, value in props.items():
                        attrib[key] = str(value)
                    ET.SubElement(self.asset, 'material', **attrib)
            else:
                attrib = {'name': name}
                for key, value in props.items():
                    attrib[key] = str(value)
                ET.SubElement(self.asset, 'material', **attrib)
    
    def _add_ground(self):
        """添加地面"""
        ground = self.config.get('ground', {})
        if ground.get('enabled', True):
            attrib = {
                'name': 'floor',
                'type': 'plane',
                'size': ground.get('size', '0 0 0.05')
            }
            
            # 如果指定了 rgba，直接使用（优先级高于 material）
            if 'rgba' in ground:
                attrib['rgba'] = ground['rgba']
            
            # 如果指定了 material，使用材质
            if 'material' in ground:
                attrib['material'] = ground['material']
            
            ET.SubElement(self.worldbody, 'geom', **attrib)
    
    def _add_lights(self):
        """添加光源"""
        lights = self.config.get('lights', [])
        for i, light in enumerate(lights):
            attrib = {
                'name': light.get('name', f'light_{i}'),
                'pos': light.get('pos', '0 0 3.5'),
                'dir': light.get('dir', '0 0 -1'),
                'directional': light.get('directional', 'true')
            }
            
            # 可选参数
            if 'diffuse' in light:
                attrib['diffuse'] = light['diffuse']
            if 'specular' in light:
                attrib['specular'] = light['specular']
            if 'active' in light:
                attrib['active'] = light['active']
            if 'castshadow' in light:
                attrib['castshadow'] = light['castshadow']
            
            ET.SubElement(self.worldbody, 'light', **attrib)
    
    def _add_cameras(self):
        """添加固定相机"""
        cameras = self.config.get('cameras', [])
        for i, camera in enumerate(cameras):
            attrib = {
                'name': camera.get('name', f'camera_{i}'),
                'pos': camera.get('pos', '0 0 3'),
                'mode': camera.get('mode', 'fixed')
            }
            
            # 可选参数
            if 'xyaxes' in camera:
                attrib['xyaxes'] = camera['xyaxes']
            if 'quat' in camera:
                attrib['quat'] = camera['quat']
            if 'euler' in camera:
                attrib['euler'] = camera['euler']
            if 'fovy' in camera:
                attrib['fovy'] = str(camera['fovy'])
            
            ET.SubElement(self.worldbody, 'camera', **attrib)
    
    @staticmethod
    def _get_image_size(image_path):
        """
        获取图片的宽高（像素），返回 (width, height)。
        优先使用 PIL，不可用时通过读取 PNG/JPEG 文件头获取。
        """
        try:
            from PIL import Image
            with Image.open(image_path) as img:
                return img.size  # (width, height)
        except ImportError:
            pass
        # Fallback: 直接读取文件头
        with open(image_path, 'rb') as f:
            header = f.read(32)
            # PNG
            if header[:8] == b'\x89PNG\r\n\x1a\n':
                w, h = struct.unpack('>II', header[16:24])
                return (w, h)
            # JPEG
            if header[:2] == b'\xff\xd8':
                f.seek(0)
                f.read(2)
                while True:
                    marker, size = struct.unpack('>HH', f.read(4))
                    if 0xFFC0 <= marker <= 0xFFC3:
                        f.read(1)  # precision
                        h, w = struct.unpack('>HH', f.read(4))
                        return (w, h)
                    f.read(size - 2)
        return None

    def _resolve_wall_texture(self, texture_file, mat_name_base, fallback_mat,
                              wall_length=None, wall_height=None):
        """
        为单面墙创建纹理+材质。
        一面墙一张图，texrepeat="1 1" 拉伸铺满整面墙。
        返回要使用的材质名；若纹理不存在则返回 fallback_mat。
        """
        script_dir = Path(__file__).resolve().parent
        base_dir = script_dir.parent
        output_dir = base_dir / 'models' / self.robot_model_dir / 'xml'
        tex_path = (base_dir / texture_file).resolve()
        if not tex_path.exists():
            print(f"Warning: wall texture not found: {tex_path}, using material '{fallback_mat}'")
            return fallback_mat
        rel_path = os.path.relpath(tex_path, output_dir) if output_dir.exists() else texture_file
        tex_name = f"{mat_name_base}_tex"
        mat_name = f"{mat_name_base}_mat"
        ET.SubElement(self.asset, 'texture', type='2d', name=tex_name, file=str(rel_path))
        mat_props = self.config.get('materials', {}).get(fallback_mat, {})
        rgba = mat_props.get('rgba', '0.9 0.9 0.9 1')
        refl = str(mat_props.get('reflectance', 0.05))
        shine = str(mat_props.get('shininess', 0.05))

        if wall_length and wall_height:
            print(f"  Wall '{mat_name_base}': {wall_length:.2f}m × {wall_height:.2f}m, "
                  f"texture={Path(texture_file).name} → texrepeat=1 1 (拉伸铺满)")

        ET.SubElement(self.asset, 'material', name=mat_name, texture=tex_name,
                      texrepeat='1 1', texuniform='false',
                      rgba=rgba, reflectance=refl, shininess=shine)
        return mat_name

    def _create_wall_from_points(self, p1, p2, height, thickness, material, name_prefix, texture_file=None):
        """
        从两个点创建墙壁
        Args:
            p1, p2: 墙壁两端点 [x, y]
            height: 墙高
            thickness: 墙厚度
            material: 材质名称（无 texture_file 时使用）
            name_prefix: 名称前缀
            texture_file: 可选，该墙专属贴图路径；一墙一图，等比例铺满
        """
        p1 = np.array(p1)
        p2 = np.array(p2)
        
        # 计算墙壁中心点
        center = (p1 + p2) / 2
        
        # 计算墙壁长度
        length = np.linalg.norm(p2 - p1)
        
        # 计算墙壁方向（单位向量）
        direction = p2 - p1
        dir_norm = direction / length  # 单位方向向量
        angle = np.arctan2(direction[1], direction[0])

        # 墙壁位置: x, y, z(高度的一半)
        pos = f"{center[0]} {center[1]} {height/2}"

        if texture_file:
            tex_material = self._resolve_wall_texture(
                texture_file, name_prefix, material,
                wall_length=length, wall_height=height)
            # xyaxes: X=墙长方向, Y=世界Z(向上), Z=墙法线(外侧)
            xyaxes = f"{dir_norm[0]} {dir_norm[1]} 0 0 0 1"
            # 法线方向（指向外侧）
            normal = np.array([-dir_norm[1], dir_norm[0]])

            # 1) 主体墙壁：用原始材质（无贴图），负责碰撞和内侧视觉
            size = f"{length/2} {height/2} {thickness/2}"
            ET.SubElement(self.worldbody, 'geom',
                         name=name_prefix,
                         type='box',
                         size=size,
                         pos=pos,
                         xyaxes=xyaxes,
                         material=material,
                         contype='1',
                         conaffinity='1')

            # 2) 内侧贴图层：极薄的装饰 box，贴在内表面，仅视觉无碰撞
            decal_thickness = 0.001
            outer_offset = -(thickness / 2 + decal_thickness / 2) * normal
            decal_pos = f"{center[0] + outer_offset[0]} {center[1] + outer_offset[1]} {height/2}"
            decal_size = f"{length/2} {height/2} {decal_thickness/2}"
            ET.SubElement(self.worldbody, 'geom',
                         name=f"{name_prefix}_decal",
                         type='box',
                         size=decal_size,
                         pos=decal_pos,
                         xyaxes=xyaxes,
                         material=tex_material,
                         contype='0',
                         conaffinity='0')
        else:
            # 无贴图的墙壁保持原来的方式
            size = f"{length/2} {thickness/2} {height/2}"
            ET.SubElement(self.worldbody, 'geom',
                         name=name_prefix,
                         type='box',
                         size=size,
                         pos=pos,
                         euler=f"0 0 {angle}",
                         material=material,
                         contype='1',
                         conaffinity='1')
    
    def _add_room(self, room_config, room_name):
        """
        添加房间（从点列表自动生成墙壁）
        支持每面墙单独贴图：texture_file（单墙）或 texture_files（多墙，与边顺序对应）
        贴图按墙面 1:1 等比例铺满，不重复。
        """
        points = room_config.get('points', [])
        if len(points) < 2:
            print(f"Warning: Room '{room_name}' needs at least 2 points")
            return
        
        height = room_config.get('height', 3.0)
        thickness = room_config.get('wall_thickness', 0.05)
        material = room_config.get('material', 'walls_mat')
        # 每面墙单独贴图：texture_files 为列表，与墙顺序对应
        texture_files = room_config.get('texture_files', [])
        texture_file_single = room_config.get('texture_file', None)
        
        # 如果只有2个点，创建单面墙
        if len(points) == 2:
            wall_name = f"{room_name}_wall"
            tex = texture_file_single or (texture_files[0] if texture_files else None)
            self._create_wall_from_points(points[0], points[1], height, thickness, material, wall_name, texture_file=tex)
            return
        
        # 3个或更多点：闭合多边形（最后一个点连回第一个点）
        points_closed = points + [points[0]]
        n_walls = len(points_closed) - 1
        
        for i in range(n_walls):
            p1 = points_closed[i]
            p2 = points_closed[i + 1]
            wall_name = f"{room_name}_wall_{i}"
            tex = texture_files[i] if i < len(texture_files) and texture_files[i] else None
            self._create_wall_from_points(p1, p2, height, thickness, material, wall_name, texture_file=tex)
    
    def _add_rooms(self):
        """添加所有房间"""
        rooms = self.config.get('rooms', {})
        for room_name, room_config in rooms.items():
            self._add_room(room_config, room_name)
    
    def _add_objects(self):
        """添加物体"""
        objects = self.config.get('objects', [])
        
        for obj in objects:
            obj_type = obj.get('type')
            name = obj.get('name', 'object')
            pos = obj.get('pos', '0 0 0')
            
            if obj_type == 'box':
                self._add_box_object(obj, name, pos)
            elif obj_type == 'cylinder':
                self._add_cylinder_object(obj, name, pos)
            elif obj_type == 'sphere':
                self._add_sphere_object(obj, name, pos)
            elif obj_type == 'table':
                self._add_table_object(obj, name, pos)
            elif obj_type == 'conveyor_belt':
                self._add_conveyor_belt_object(obj, name, pos)
            elif obj_type == 'floor_marker':
                self._add_floor_marker_object(obj, name)
            elif obj_type == 'include':
                self._add_include_object(obj, name, pos)
            elif obj_type == 'slope':
                self._add_slope_object(obj, name, pos)
            elif obj_type == 'stairs':
                self._add_stairs_object(obj, name, pos)
            elif obj_type == 'qrcode':
                self._add_qrcode_object(obj, name, pos)
            elif obj_type == 'hfield' or obj_type == 'gravel_road':
                self._add_hfield_object(obj, name, pos)
            elif obj_type == 'speed_bump':
                self._add_speed_bump_object(obj, name, pos)
            elif obj_type == 'traffic_cone':
                self._add_traffic_cone_object(obj, name, pos)
            elif obj_type == 'pillar' or obj_type == 'column':
                self._add_pillar_object(obj, name, pos)
            elif obj_type == 'button_sphere' or obj_type == 'hemisphere':
                self._add_button_sphere_object(obj, name, pos)
    
    def _add_box_object(self, obj, name, pos):
        """添加盒子物体"""
        body = ET.SubElement(self.worldbody, 'body', name=name, pos=pos)
        
        if obj.get('movable', False):
            ET.SubElement(body, 'freejoint', name=name)
        
        ET.SubElement(body, 'geom',
                     name=f"{name}_geom",
                     type='box',
                     size=obj.get('size', '0.1 0.1 0.1'),
                     rgba=obj.get('rgba', '0.5 0.5 0.5 1'),
                     mass=str(obj.get('mass', 0.1)),
                     friction=obj.get('friction', '1 0.005 0.0001'))
    
    def _add_cylinder_object(self, obj, name, pos):
        """添加圆柱物体"""
        body = ET.SubElement(self.worldbody, 'body', name=name, pos=pos)
        
        if obj.get('movable', False):
            ET.SubElement(body, 'freejoint', name=name)
        
        ET.SubElement(body, 'geom',
                     name=f"{name}_geom",
                     type='cylinder',
                     size=obj.get('size', '0.05 0.1'),
                     rgba=obj.get('rgba', '0.5 0.5 0.5 1'),
                     mass=str(obj.get('mass', 0.1)),
                     friction=obj.get('friction', '1 0.005 0.0001'))
    
    def _add_sphere_object(self, obj, name, pos):
        """添加球体物体"""
        body = ET.SubElement(self.worldbody, 'body', name=name, pos=pos)
        
        if obj.get('movable', False):
            ET.SubElement(body, 'freejoint', name=name)
        
        ET.SubElement(body, 'geom',
                     name=f"{name}_geom",
                     type='sphere',
                     size=str(obj.get('radius', 0.05)),
                     rgba=obj.get('rgba', '0.5 0.5 0.5 1'),
                     mass=str(obj.get('mass', 0.1)),
                     friction=obj.get('friction', '1 0.005 0.0001'))
    
    def _add_table_object(self, obj, name, pos):
        """添加桌子"""
        body = ET.SubElement(self.worldbody, 'body', name=name, pos=pos)
        
        # 桌面
        top_size = obj.get('top_size', [0.4, 0.8, 0.02])
        ET.SubElement(body, 'geom',
                     name=f"{name}_top",
                     type='box',
                     size=f"{top_size[0]} {top_size[1]} {top_size[2]}",
                     material=obj.get('top_material', 'table_ceramic'),
                     friction='1 0.005 0.0001')
        
        # 桌腿
        leg_radius = obj.get('leg_radius', 0.03)
        leg_height = obj.get('leg_height', 0.4)
        leg_material = obj.get('leg_material', 'table_legs_metal')
        
        x_offset, y_offset = top_size[0], top_size[1]
        z_offset = -leg_height
        
        legs = [
            (f"{x_offset}  {y_offset} {z_offset}"),
            (f"{x_offset} -{y_offset} {z_offset}"),
            (f"-{x_offset}  {y_offset} {z_offset}"),
            (f"-{x_offset} -{y_offset} {z_offset}")
        ]
        
        for i, leg_pos in enumerate(legs):
            ET.SubElement(body, 'geom',
                         name=f"{name}_leg_{i}",
                         type='cylinder',
                         size=f"{leg_radius} {leg_height}",
                         pos=leg_pos,
                         material=leg_material,
                         contype='0',
                         conaffinity='0')
    
    def _add_conveyor_belt_object(self, obj, name, pos):
        """添加传送带"""
        body = ET.SubElement(self.worldbody, 'body', name=name, pos=pos)
        
        # 惯性
        ET.SubElement(body, 'inertial',
                     pos='0 0 0',
                     mass='1e-1',
                     diaginertia='1e-6 1e-6 1e-6')
        
        # 滑动关节
        ET.SubElement(body, 'joint',
                     name=f"{name}_slide",
                     type='slide',
                     axis='1 0 0',
                     range='-20 20',
                     damping='0.001')
        
        # 传送带几何体
        size = obj.get('size', [100, 0.3, 0.01])
        ET.SubElement(body, 'geom',
                     name=f"{name}_geom",
                     type='box',
                     size=f"{size[0]} {size[1]} {size[2]}",
                     material=obj.get('material', 'belt_mat'),
                     friction='1.0 0.005 0.0001')
        
        # 添加执行器
        if self.actuator is None:
            self.actuator = ET.SubElement(self.root, 'actuator')
        
        ET.SubElement(self.actuator, 'velocity',
                     name=f"{name}_speed",
                     joint=f"{name}_slide",
                     kv='1',
                     ctrlrange='-0.1 0.1')
    
    def _add_floor_marker_object(self, obj, name):
        """
        添加地面标记（彩色区域）
        Args:
            obj: 物体配置
            name: 标记名称
        """
        points = obj.get('points', [])
        if len(points) != 4:
            print(f"Warning: Floor marker '{name}' needs exactly 4 points")
            return
        
        # 计算矩形中心点
        import numpy as np
        points_array = np.array(points)
        center_x = np.mean(points_array[:, 0])
        center_y = np.mean(points_array[:, 1])
        
        # 计算矩形的长宽
        # 假设点按顺序给出：右上、左上、左下、右下
        width = abs(points[0][0] - points[1][0])  # x方向
        length = abs(points[1][1] - points[2][1])  # y方向
        
        # 地面标记高度（非常薄，略高于地面避免z-fighting）
        height = obj.get('height', 0.001)
        z_pos = obj.get('z_offset', 0.051)  # 略高于地面
        
        # 创建地面标记
        ET.SubElement(self.worldbody, 'geom',
                     name=name,
                     type='box',
                     size=f"{width/2} {length/2} {height}",
                     pos=f"{center_x} {center_y} {z_pos}",
                     rgba=obj.get('rgba', '1 0 0 0.5'),
                     contype='0',
                     conaffinity='0')

        # 边线
        border_rgba = obj.get('border_rgba')
        if border_rgba:
            bw = obj.get('border_width', 0.03)
            bz = z_pos + 0.001
            half_w = width / 2
            half_l = length / 2
            borders = [
                (f"{name}_border_top",    center_x, center_y + half_l, half_w, bw / 2),
                (f"{name}_border_bottom", center_x, center_y - half_l, half_w, bw / 2),
                (f"{name}_border_left",   center_x - half_w, center_y, bw / 2, half_l),
                (f"{name}_border_right",  center_x + half_w, center_y, bw / 2, half_l),
            ]
            for bname, bx, by, sx, sy in borders:
                ET.SubElement(self.worldbody, 'geom',
                             name=bname, type='box',
                             size=f"{sx} {sy} {height}",
                             pos=f"{bx} {by} {bz}",
                             rgba=border_rgba,
                             contype='0', conaffinity='0')
    
    def _add_slope_object(self, obj, name, pos):
        """添加斜坡（使用 mesh）"""
        from pathlib import Path
        
        # 默认使用 slope 目录下的 xiepo.stl
        mesh_file = obj.get('mesh_file', 'models/slope/meshes/xiepo.stl')
        script_dir = Path(__file__).resolve().parent
        mesh_path = (script_dir.parent / mesh_file).resolve()
        
        # 添加 mesh 到 asset
        mesh_name = f"{name}_mesh"
        ET.SubElement(self.asset, 'mesh',
                     name=mesh_name,
                     file=str(mesh_path))
        
        # 添加 geom
        ET.SubElement(self.worldbody, 'geom',
                     name=name,
                     type='mesh',
                     mesh=mesh_name,
                     pos=pos,
                     euler=obj.get('euler', '0 0 0'),
                     contype='1',
                     conaffinity='1',
                     rgba=obj.get('rgba', '0.6 0.4 0.2 1'))

        # 顶部平台标记（不同颜色区分平台和斜面）
        platform_rgba = obj.get('platform_rgba')
        if platform_rgba:
            # 斜坡 mesh 平台位于局部坐标 X=2.0~3.0, Y=-0.75~0.75, Z=0.52
            pos_parts = pos.split()
            px, py, pz = float(pos_parts[0]), float(pos_parts[1]), float(pos_parts[2])
            plat_cx = px + 2.5  # 平台中心 X
            plat_cy = py        # 平台中心 Y
            plat_cz = pz + 0.521  # 略高于平台顶面
            ET.SubElement(self.worldbody, 'geom',
                         name=f"{name}_platform",
                         type='box',
                         size='0.5 0.75 0.001',
                         pos=f"{plat_cx} {plat_cy} {plat_cz}",
                         rgba=platform_rgba,
                         contype='0',
                         conaffinity='0')

        print(f"✓ Added slope: {name}")
    
    def _add_stairs_object(self, obj, name, pos):
        """
        添加楼梯（box 模式，仿 kuavo scene）
        每级台阶一个 box，踏面 0.28m×1.5m，台阶高 0.13m，4级上+平台1m+4级下
        """
        tread_depth = float(obj.get('tread_depth', 0.28))
        tread_width = float(obj.get('tread_width', 1.5))
        rise = float(obj.get('rise', 0.13))
        platform_length = float(obj.get('platform_length', 1.0))
        num_up = int(obj.get('num_steps_up', 4))
        num_down = int(obj.get('num_steps_down', 4))
        euler = obj.get('euler', '0 0 0')
        rgba = obj.get('rgba', '0.76 0.60 0.42 1')

        mat_name = f"{name}_mat"
        ET.SubElement(self.asset, 'material', name=mat_name, rgba=rgba, reflectance="0.05", shininess="0.1")

        half_tread = tread_depth / 2
        half_width = tread_width / 2
        half_rise = rise / 2
        platform_half_x = platform_length / 2
        platform_half_z = half_rise  # 平台厚度与台阶一致

        body = ET.SubElement(self.worldbody, 'body', name=f"{name}_body", pos=pos, euler=euler)

        geom_attrs = {
            'contype': '1', 'conaffinity': '1', 'condim': '3',
            'solref': '0.02 1', 'solimp': '0.9 0.95 0.001',
            'friction': '3 0.5 0.01', 'material': mat_name, 'rgba': rgba,
        }

        def add_step(b, gname, cx, cz):
            ET.SubElement(b, 'geom', name=gname, type='box',
                         size=f"{half_tread} {half_width} {half_rise}",
                         pos=f"{cx} 0 {cz}", quat="1 0 0 0", **geom_attrs)

        # 4 级上
        for i in range(num_up):
            cx = (i + 0.5) * tread_depth
            cz = (i + 0.5) * rise
            add_step(body, f"{name}_{i+1}", cx, cz)

        # 平台：与第4级齐平（顶面 0.52），厚度与台阶一致
        platform_cx = num_up * tread_depth + platform_length / 2
        platform_cz = num_up * rise - platform_half_z
        platform_rgba = obj.get('platform_rgba', '0.70 0.55 0.38 1')
        platform_attrs = {**geom_attrs, 'rgba': platform_rgba}
        ET.SubElement(body, 'geom', name=f"{name}_platform", type='box',
                     size=f"{platform_half_x} {half_width} {platform_half_z}",
                     pos=f"{platform_cx} 0 {platform_cz}", quat="1 0 0 0", **platform_attrs)

        # 4 级下：紧接平台后，第一级与平台等高(0.52)，逐级下降
        platform_end_x = platform_cx + platform_half_x
        for i in range(num_down):
            cx = platform_end_x + (i + 0.5) * tread_depth
            cz = (num_up - i - 0.5) * rise
            add_step(body, f"{name}_{num_up+i+1}", cx, cz)

        print(f"✓ Added stairs (box): {name} ({num_up}↑ + 平台{platform_length}m + {num_down}↓)")
    
    def _add_qrcode_object(self, obj, name, pos):
        """添加二维码/图片显示板"""
        from pathlib import Path
        
        # 获取图片文件路径
        image_file = obj.get('image_file')
        if not image_file:
            print(f"Warning: QRCode object '{name}' missing 'image_file' parameter")
            return
        
        script_dir = Path(__file__).resolve().parent
        image_path = (script_dir.parent / image_file).resolve()
        
        # 添加纹理和材质到 asset
        tex_name = f"{name}_tex"
        mat_name = f"{name}_mat"
        
        ET.SubElement(self.asset, 'texture',
                     name=tex_name,
                     type='2d',
                     file=str(image_path))
        
        ET.SubElement(self.asset, 'material',
                     name=mat_name,
                     texture=tex_name,
                     texrepeat='1 1',
                     texuniform='false')
        
        # 获取尺寸（默认 7cm x 7cm，厚度 0.5cm）
        size = obj.get('size', '0.07 0.07 0.005')
        
        # 添加 geom（无碰撞的薄板）
        geom_attrib = {
            'name': name,
            'type': 'box',
            'size': size,
            'pos': pos,
            'material': mat_name,
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 1 1 1'
        }
        if 'quat' in obj:
            geom_attrib['quat'] = obj['quat']
        elif 'euler' in obj:
            geom_attrib['euler'] = obj['euler']
        
        geom = ET.SubElement(self.worldbody, 'geom')
        for k, v in geom_attrib.items():
            geom.attrib[k] = v
        
        print(f"✓ Added QRCode: {name}")
    
    def _add_hfield_object(self, obj, name, pos):
        """添加高度场地形（如石子路）"""
        from pathlib import Path
        
        # 获取高度场图片文件路径
        hfield_file = obj.get('hfield_file', 'models/gravel_road/mujoco_assets/road_heightfield.png')
        script_dir = Path(__file__).resolve().parent
        hfield_path = (script_dir.parent / hfield_file).resolve()
        
        # 获取尺寸参数
        # size: "半宽 半长 最大高度 基础高度"
        size = obj.get('size', '1.25 1.05 0.05 0.1')
        geom_size = obj.get('geom_size', '1.25 1.05 1')
        
        # 添加材质
        mat_name = f"{name}_mat"
        ET.SubElement(self.asset, 'material',
                     name=mat_name,
                     rgba=obj.get('rgba', '0.6 0.55 0.5 1'),
                     reflectance='0.3',
                     shininess='0.1',
                     emission='0.2')
        
        # 添加 hfield 到 asset
        hfield_name = f"{name}_hf"
        ET.SubElement(self.asset, 'hfield',
                     name=hfield_name,
                     file=str(hfield_path),
                     size=size)
        
        # 添加 geom
        geom_attrib = {
            'name': name,
            'type': 'hfield',
            'hfield': hfield_name,
            'size': geom_size,
            'pos': pos,
            'material': mat_name,
            'condim': '6',
            'conaffinity': '15',
            'solref': '0.02 1',
            'solimp': '0.9 0.95 0.001',
            'friction': obj.get('friction', '1 0.1 0.1'),
            'density': '1000'
        }
        
        if 'euler' in obj:
            geom_attrib['euler'] = obj['euler']
        if 'quat' in obj:
            geom_attrib['quat'] = obj['quat']
        
        geom = ET.SubElement(self.worldbody, 'geom')
        for k, v in geom_attrib.items():
            geom.attrib[k] = v
        
        print(f"✓ Added heightfield (gravel road): {name}")
    
    def _add_speed_bump_object(self, obj, name, pos):
        """添加减速带（从 OBJ/STL mesh 加载）"""
        from pathlib import Path
        
        # 获取 mesh 文件路径
        mesh_file = obj.get('mesh_file', 'models/speed_bump/uploads_files_2559840_Lezachiq.obj')
        script_dir = Path(__file__).resolve().parent
        mesh_path = (script_dir.parent / mesh_file).resolve()
        
        # 目标尺寸（用户指定）和原始尺寸
        # 原始 OBJ 尺寸: X=0.64, Y=0.16, Z=3.41
        target_size = obj.get('target_size', '0.35 2.0 0.05')  # 长(X) 宽(Z) 高(Y)
        parts = target_size.split()
        target_length = float(parts[0]) if len(parts) > 0 else 0.35  # 沿道路方向
        target_width = float(parts[1]) if len(parts) > 1 else 2.0   # 横跨道路
        target_height = float(parts[2]) if len(parts) > 2 else 0.05  # 凸起高度
        
        orig_x, orig_y, orig_z = 0.64, 0.16, 3.41
        scale_x = target_length / orig_x
        scale_y = target_height / orig_y
        scale_z = target_width / orig_z
        scale_str = f"{scale_x:.4f} {scale_y:.4f} {scale_z:.4f}"
        
        # 添加 mesh 到 asset
        mesh_name = f"{name}_mesh"
        ET.SubElement(self.asset, 'mesh',
                     name=mesh_name,
                     file=str(mesh_path),
                     scale=scale_str)
        
        # 添加材质
        mat_name = f"{name}_mat"
        ET.SubElement(self.asset, 'material',
                     name=mat_name,
                     rgba=obj.get('rgba', '0.2 0.2 0.2 1'),
                     reflectance='0.1',
                     shininess='0.05')
        
        # 添加 geom
        geom_attrib = {
            'name': name,
            'type': 'mesh',
            'mesh': mesh_name,
            'pos': pos,
            'material': mat_name,
            'contype': '1',
            'conaffinity': '1',
            'friction': obj.get('friction', '1 0.5 0.01'),
            'condim': '4'
        }
        
        if 'euler' in obj:
            geom_attrib['euler'] = obj['euler']
        if 'quat' in obj:
            geom_attrib['quat'] = obj['quat']
        
        geom = ET.SubElement(self.worldbody, 'geom')
        for k, v in geom_attrib.items():
            geom.attrib[k] = v
        
        print(f"✓ Added speed bump: {name} (scale: {scale_str})")
    
    def _add_traffic_cone_object(self, obj, name, pos):
        """添加锥形路障"""
        from pathlib import Path
        
        # 获取 mesh 文件路径
        mesh_file = obj.get('mesh_file', 'models/traffic_cone/uploads_files_3223633_traffic+cone.obj')
        script_dir = Path(__file__).resolve().parent
        mesh_path = (script_dir.parent / mesh_file).resolve()
        
        # 目标尺寸（用户指定）和原始尺寸
        # 原始 OBJ 尺寸: X=0.34, Y=0.75, Z=0.34
        target_size = obj.get('target_size', '0.33 0.6')  # 直径 高度
        parts = target_size.split()
        target_diameter = float(parts[0]) if len(parts) > 0 else 0.33
        target_height = float(parts[1]) if len(parts) > 1 else 0.6
        
        orig_diameter, orig_height = 0.34, 0.75
        scale_xz = target_diameter / orig_diameter
        scale_y = target_height / orig_height
        scale_str = f"{scale_xz:.4f} {scale_y:.4f} {scale_xz:.4f}"
        
        # 添加 mesh 到 asset
        mesh_name = f"{name}_mesh"
        ET.SubElement(self.asset, 'mesh',
                     name=mesh_name,
                     file=str(mesh_path),
                     scale=scale_str)
        
        # 添加材质（橙色路障）
        mat_name = f"{name}_mat"
        ET.SubElement(self.asset, 'material',
                     name=mat_name,
                     rgba=obj.get('rgba', '1.0 0.4 0.0 1'),
                     reflectance='0.3',
                     shininess='0.1')
        
        # 添加 body + freejoint + geom（支持运行时移动位置）
        # body 和 joint 使用相同名称，以便 set_object_position 服务能正确查找
        body_attrib = {'name': name, 'pos': pos}
        if 'euler' in obj:
            body_attrib['euler'] = obj['euler']
        if 'quat' in obj:
            body_attrib['quat'] = obj['quat']

        body = ET.SubElement(self.worldbody, 'body', **body_attrib)
        ET.SubElement(body, 'freejoint', name=name)

        geom_attrib = {
            'name': name,
            'type': 'mesh',
            'mesh': mesh_name,
            'material': mat_name,
            'contype': '1',
            'conaffinity': '1',
            'friction': obj.get('friction', '1 0.5 0.01'),
            'condim': '4',
            'mass': obj.get('mass', '2.0'),
        }

        geom = ET.SubElement(body, 'geom')
        for k, v in geom_attrib.items():
            geom.attrib[k] = v

        print(f"✓ Added traffic cone: {name} (diameter={target_diameter}m, height={target_height}m)")
    
    def _add_pillar_object(self, obj, name, pos):
        """添加立柱（box 几何体，长×宽×高）"""
        # target_size: "长 宽 高"（米），MuJoCo box 的 size 为半尺寸
        target_size = obj.get('target_size', '0.4 0.4 1.0')
        parts = target_size.split()
        length = float(parts[0]) if len(parts) > 0 else 0.4
        width = float(parts[1]) if len(parts) > 1 else 0.4
        height = float(parts[2]) if len(parts) > 2 else 1.0
        half = f"{length/2:.4f} {width/2:.4f} {height/2:.4f}"
        
        # pos 表示底面中心，box 中心 z = 底面 z + height/2
        pos_parts = pos.split()
        if len(pos_parts) >= 3:
            z_bottom = float(pos_parts[2])
            z_center = z_bottom + height / 2
            pos = f"{pos_parts[0]} {pos_parts[1]} {z_center}"
        
        # 纹理支持
        texture_file = obj.get('texture_file')
        mat_name = None
        if texture_file:
            script_dir = Path(__file__).resolve().parent
            base_dir = script_dir.parent
            output_dir = base_dir / 'models' / self.robot_model_dir / 'xml'
            tex_path = (base_dir / texture_file).resolve()
            if tex_path.exists():
                rel_path = os.path.relpath(tex_path, output_dir) if output_dir.exists() else texture_file
                tex_name = f"{name}_tex"
                mat_name = f"{name}_mat"
                ET.SubElement(self.asset, 'texture', type='2d', name=tex_name, file=str(rel_path))
                mat_attrib = {
                    'name': mat_name, 'texture': tex_name,
                    'texrepeat': obj.get('texrepeat', '1 1'), 'texuniform': 'false',
                    'emission': str(obj.get('emission', '0.15')),
                    'specular': str(obj.get('specular', '0.4')),
                    'shininess': str(obj.get('shininess', '0.6')),
                }
                ET.SubElement(self.asset, 'material', **mat_attrib)
            else:
                print(f"Warning: pillar texture not found: {tex_path}")

        geom_attrib = {
            'name': name,
            'type': 'box',
            'size': half,
            'pos': pos,
            'contype': '1',
            'conaffinity': '1',
            'friction': obj.get('friction', '1 0.5 0.01'),
            'condim': '4'
        }
        if mat_name:
            geom_attrib['material'] = mat_name
        else:
            geom_attrib['rgba'] = obj.get('rgba', '0.5 0.5 0.5 1')
        if 'euler' in obj:
            geom_attrib['euler'] = obj['euler']
        if 'quat' in obj:
            geom_attrib['quat'] = obj['quat']

        geom = ET.SubElement(self.worldbody, 'geom')
        for k, v in geom_attrib.items():
            geom.attrib[k] = v

        print(f"✓ Added pillar: {name} ({length}×{width}×{height}m)")
    
    def _add_button_sphere_object(self, obj, name, pos):
        """添加按钮半球（用球体嵌入表面模拟半球效果）
        
        半球会嵌入下方物体一半，视觉上呈现半球。
        支持 bright/dim 状态，用于交互检测。
        """
        diameter = float(obj.get('diameter', '0.1'))
        radius = diameter / 2
        
        # 材质：支持亮/暗两种状态（初始用 bright 或 dim）
        is_bright = obj.get('bright', True)
        base_rgba = obj.get('rgba', '0 1 0 1')  # 基础颜色
        rgba_parts = [float(x) for x in base_rgba.split()]
        
        if is_bright:
            # 亮态：高亮 + 强 emission
            emission = '0.8'
            final_rgba = f"{min(rgba_parts[0]*1.2, 1):.2f} {min(rgba_parts[1]*1.2, 1):.2f} {min(rgba_parts[2]*1.2, 1):.2f} {rgba_parts[3]}"
        else:
            # 暗态：非常暗，接近黑色
            emission = '0.0'
            final_rgba = f"{rgba_parts[0]*0.15:.2f} {rgba_parts[1]*0.15:.2f} {rgba_parts[2]*0.15:.2f} {rgba_parts[3]}"
        
        # 添加材质
        mat_name = f"{name}_mat"
        ET.SubElement(self.asset, 'material',
                     name=mat_name,
                     rgba=final_rgba,
                     emission=emission,
                     reflectance='0.3',
                     shininess='0.5')
        
        # pos 是球心位置（半球显示在上半部分）
        geom_attrib = {
            'name': name,
            'type': 'sphere',
            'size': str(radius),
            'pos': pos,
            'material': mat_name,
            'contype': '1',
            'conaffinity': '1',
            'friction': obj.get('friction', '1 0.5 0.01'),
            'condim': '4'
        }
        
        geom = ET.SubElement(self.worldbody, 'geom')
        for k, v in geom_attrib.items():
            geom.attrib[k] = v
        
        state = "亮" if is_bright else "暗"
        print(f"✓ Added button sphere: {name} (d={diameter}m, {state})")
    
    def _add_include_object(self, obj, name, pos):
        """从外部XML文件加载物体"""
        xml_file = obj.get('file')
        if not xml_file:
            print(f"Warning: Include object '{name}' missing 'file' parameter")
            return
        
        try:
            from pathlib import Path
            import os
            xml_path = Path(xml_file)
            if not xml_path.is_absolute():
                script_dir = Path(__file__).resolve().parent
                xml_path = (script_dir.parent / xml_file).resolve()
            else:
                xml_path = xml_path.resolve()
            
            external_tree = ET.parse(str(xml_path))
            external_root = external_tree.getroot()
            
            # 处理根级别的 include 标签，计算相对路径
            script_dir = Path(__file__).resolve().parent
            output_dir = (script_dir.parent / 'models' / self.robot_model_dir / 'xml').resolve()
            
            # 保存原始 include 文件路径，用于后续 assets 加载
            original_include_files = {}
            for include_elem in external_root.findall('include'):
                if 'file' in include_elem.attrib:
                    include_file = include_elem.attrib['file']
                    original_include_files[id(include_elem)] = include_file
                    # 跳过 common.xml（通用配置文件，可能不存在）
                    if 'common.xml' not in include_file:
                        include_abs_path = xml_path.parent / include_file
                        if include_abs_path.exists():
                            rel_path = os.path.relpath(str(include_abs_path), str(output_dir))
                            include_elem.attrib['file'] = rel_path
                        else:
                            # 如果文件不存在，尝试相对于上级目录
                            alt_path = xml_path.parent.parent / include_file
                            if alt_path.exists():
                                rel_path = os.path.relpath(str(alt_path), str(output_dir))
                                include_elem.attrib['file'] = rel_path
            
            # 计算缩放比例
            scale_str = None
            if 'target_size' in obj:
                # 如果指定了 target_size，计算缩放比例
                # target_size 格式: "宽度 高度" 或 "直径 高度" 例如 "0.2 0.03"
                target_parts = obj['target_size'].split()
                if len(target_parts) >= 2:
                    target_width = float(target_parts[0])
                    target_height = float(target_parts[1])
                    
                    # 根据模型文件路径判断原始尺寸
                    # 这些值通过分析 OBJ 文件得到
                    if 'tray' in xml_file.lower():
                        # 托盘（圆形）：原始直径约 0.22m，高度约 0.028m
                        original_diameter = 0.22
                        original_height = 0.028
                        target_diameter = target_width  # target_size: "直径 高度"
                        scale_x = target_diameter / original_diameter
                        scale_y = target_diameter / original_diameter
                        scale_z = target_height / original_height
                    elif 'basket' in xml_file.lower():
                        # 篮子：原始宽 0.24m (X)，深 0.16m (Y)，高 0.15m (Z)
                        original_x = 0.24  # 宽
                        original_y = 0.16  # 深/长
                        original_z = 0.15  # 高
                        if len(target_parts) >= 3:
                            # target_size: "宽 深 高" 或 "X Y Z"
                            target_x = float(target_parts[0])
                            target_y = float(target_parts[1])
                            target_z = float(target_parts[2])
                        else:
                            # 只有2个参数：假设 "宽 高"，深度按比例
                            target_x = target_width
                            target_z = target_height
                            target_y = target_x * (original_y / original_x)
                        scale_x = target_x / original_x
                        scale_y = target_y / original_y
                        scale_z = target_z / original_z
                    else:
                        # 默认值
                        original_width = 0.2
                        original_height = 0.1
                        scale_x = target_width / original_width
                        scale_y = target_width / original_width
                        scale_z = target_height / original_height
                    
                    scale_str = f"{scale_x} {scale_y} {scale_z}"
                    print(f"✓ Calculated scale for {name}: {scale_str} (target: {obj['target_size']})")
            elif 'scale' in obj:
                scale_str = obj['scale']
            
            # 提取asset，并应用 scale 参数，同时重命名避免冲突
            external_assets = external_root.find('asset')
            if external_assets is not None:
                for asset_elem in external_assets:
                    # 给 mesh、texture、material 等资源添加唯一前缀
                    if 'name' in asset_elem.attrib:
                        original_name = asset_elem.attrib['name']
                        asset_elem.attrib['name'] = f"{name}_{original_name}"
                    
                    # 更新 material 中的 texture 引用
                    if asset_elem.tag == 'material' and 'texture' in asset_elem.attrib:
                        original_texture = asset_elem.attrib['texture']
                        asset_elem.attrib['texture'] = f"{name}_{original_texture}"
                    
                    if asset_elem.tag == 'mesh' and 'file' in asset_elem.attrib:
                        mesh_file = asset_elem.attrib['file']
                        mesh_path = xml_path.parent / mesh_file
                        asset_elem.attrib['file'] = str(mesh_path)
                        
                        # 应用 scale 参数到 mesh（在 asset 中）
                        if scale_str:
                            asset_elem.attrib['scale'] = scale_str
                    
                    elif asset_elem.tag == 'texture' and 'file' in asset_elem.attrib:
                        texture_file = asset_elem.attrib['file']
                        texture_path = xml_path.parent / texture_file
                        asset_elem.attrib['file'] = str(texture_path)
                    
                    self.asset.append(asset_elem)
            
            # 若主模型无 asset 节点，尝试从同目录的 include（如 assets.xml）加载
            if external_root.find('asset') is None:
                for inc in external_root.findall('include'):
                    if 'file' not in inc.attrib:
                        continue
                    # 使用保存的原始文件路径，而不是被修改后的路径
                    inc_file = original_include_files.get(id(inc), inc.attrib['file'])
                    # 跳过 common.xml
                    if 'common.xml' in inc_file:
                        continue
                    inc_path = (xml_path.parent / inc_file).resolve()
                    if not inc_path.exists():
                        print(f"  Warning: include file not found: {inc_path}")
                        continue
                    try:
                        inc_tree = ET.parse(str(inc_path))
                        inc_root = inc_tree.getroot()
                        inc_asset = inc_root.find('asset')
                        
                        # 处理 <mujocoinclude><asset>...</asset></mujocoinclude> 结构
                        if inc_asset is not None:
                            for elem in inc_asset:
                                if elem.tag in ('mesh', 'material', 'texture'):
                                    new_asset = ET.Element(elem.tag)
                                    new_asset.attrib.update(elem.attrib)
                                    if 'name' in new_asset.attrib:
                                        new_asset.attrib['name'] = f"{name}_{new_asset.attrib['name']}"
                                    if elem.tag == 'mesh' and 'file' in new_asset.attrib:
                                        new_asset.attrib['file'] = str((inc_path.parent / new_asset.attrib['file']).resolve())
                                    if elem.tag == 'material' and 'texture' in new_asset.attrib:
                                        new_asset.attrib['texture'] = f"{name}_{new_asset.attrib['texture']}"
                                    self.asset.append(new_asset)
                            print(f"  ✓ Loaded assets from: {inc_file}")
                        # 处理根元素直接是 mesh/material/texture 的情况
                        elif inc_root.tag in ('mesh', 'material', 'texture'):
                            new_asset = ET.Element(inc_root.tag)
                            new_asset.attrib.update(inc_root.attrib)
                            if 'name' in new_asset.attrib:
                                new_asset.attrib['name'] = f"{name}_{new_asset.attrib['name']}"
                            if inc_root.tag == 'mesh' and 'file' in new_asset.attrib:
                                new_asset.attrib['file'] = str((inc_path.parent / new_asset.attrib['file']).resolve())
                            self.asset.append(new_asset)
                    except Exception as e:
                        print(f"  Warning: failed to load assets from {inc_file}: {e}")
            
            # 提取worldbody中的物体
            external_worldbody = external_root.find('worldbody')
            if external_worldbody is not None:
                for body_elem in external_worldbody.findall('body'):
                    new_body = ET.Element('body')
                    new_body.attrib.update(body_elem.attrib)
                    
                    # 去掉 childclass 属性，因为 default class（如 grab）可能未在场景中定义
                    if 'childclass' in new_body.attrib:
                        del new_body.attrib['childclass']
                    
                    # body 和 joint 使用相同名称，以便 set_object_position 服务能正确查找
                    new_body.attrib['name'] = name

                    new_body.attrib['pos'] = pos

                    if 'euler' in obj:
                        new_body.attrib['euler'] = obj['euler']
                    elif 'quat' in obj:
                        new_body.attrib['quat'] = obj['quat']

                    # 添加可移动选项
                    if obj.get('movable', False):
                        ET.SubElement(new_body, 'freejoint', name=name)
                    
                    # 复制子元素，并更新 mesh/material 引用
                    for child in body_elem:
                        # 处理 include 标签：MuJoCo 不允许同一文件被 include 多次，故将 body 内的 include 内联
                        if child.tag == 'include' and 'file' in child.attrib:
                            include_file = child.attrib['file']
                            include_path = (xml_path.parent / include_file).resolve()
                            if include_path.exists():
                                try:
                                    include_tree = ET.parse(str(include_path))
                                    include_root = include_tree.getroot()
                                    inline_idx = 0
                                    for incl_child in include_root:
                                        if incl_child.tag in ('geom', 'joint', 'freejoint', 'inertial'):
                                            new_el = ET.Element(incl_child.tag)
                                            new_el.attrib.update(incl_child.attrib)
                                            if 'name' in new_el.attrib:
                                                new_el.attrib['name'] = f"{name}_{new_el.attrib['name']}"
                                            else:
                                                new_el.attrib['name'] = f"{name}_inline_{inline_idx}"
                                                inline_idx += 1
                                            if 'mesh' in new_el.attrib:
                                                new_el.attrib['mesh'] = f"{name}_{new_el.attrib['mesh']}"
                                                # 有 mesh 但没有 type，需要添加 type="mesh"
                                                if 'type' not in new_el.attrib:
                                                    new_el.attrib['type'] = 'mesh'
                                            if 'material' in new_el.attrib:
                                                new_el.attrib['material'] = f"{name}_{new_el.attrib['material']}"
                                            # 若 config 指定了 rgba，覆盖视觉 geom 的颜色
                                            if new_el.tag == 'geom' and obj.get('rgba'):
                                                cls = new_el.attrib.get('class', '')
                                                if cls != 'object_col' and new_el.attrib.get('group', '0') != '3':
                                                    new_el.attrib['rgba'] = obj['rgba']
                                                    if 'material' in new_el.attrib:
                                                        del new_el.attrib['material']
                                            # 处理 class 属性：object_col 是碰撞体，需要添加 group="3" 使其不可见
                                            if 'class' in new_el.attrib:
                                                if new_el.attrib['class'] == 'object_col':
                                                    # 添加 object_col 的默认属性（来自 common.xml）
                                                    new_el.attrib['group'] = '3'  # 不可见
                                                    new_el.attrib['contype'] = '1'
                                                    new_el.attrib['conaffinity'] = '1'
                                                    new_el.attrib['friction'] = obj.get('friction', '1 0.5 0.01')
                                                    new_el.attrib['condim'] = str(obj.get('condim', '6'))
                                                    # 接触求解器参数，提高抓取稳定性
                                                    new_el.attrib['solref'] = obj.get('solref', '0.02 1')
                                                    new_el.attrib['solimp'] = obj.get('solimp', '0.9 0.95 0.001')
                                                del new_el.attrib['class']
                                            # 覆盖 inertial 的质量（若 config 指定了 mass）
                                            elif new_el.tag == 'inertial' and obj.get('mass') is not None:
                                                new_el.attrib['mass'] = str(obj['mass'])
                                                if obj.get('diaginertia'):
                                                    new_el.attrib['diaginertia'] = obj['diaginertia']
                                                else:
                                                    # 默认惯量，随质量缩放
                                                    m = float(obj['mass'])
                                                    d = max(0.001, m * 0.02)
                                                    new_el.attrib['diaginertia'] = f"{d} {d} {d}"
                                            new_body.append(new_el)
                                except Exception as e:
                                    print(f"Warning: inline include '{include_file}' failed: {e}, keeping as include")
                                    new_child = ET.Element('include')
                                    new_child.attrib['file'] = os.path.relpath(str(include_path), str((Path(__file__).resolve().parent.parent / 'models' / self.robot_model_dir / 'xml').resolve()))
                                    new_body.append(new_child)
                            else:
                                new_child = ET.Element('include')
                                new_child.attrib.update(child.attrib)
                                include_abs_path = (xml_path.parent / include_file).resolve()
                                script_dir = Path(__file__).resolve().parent
                                output_dir = (script_dir.parent / 'models' / self.robot_model_dir / 'xml').resolve()
                                import os
                                new_child.attrib['file'] = os.path.relpath(str(include_abs_path), str(output_dir))
                                new_body.append(new_child)
                            continue
                        elif child.tag == 'include':
                            new_child = ET.Element('include')
                            new_child.attrib.update(child.attrib)
                            if 'file' in new_child.attrib:
                                include_file = new_child.attrib['file']
                                include_abs_path = (xml_path.parent / include_file).resolve()
                                script_dir = Path(__file__).resolve().parent
                                output_dir = (script_dir.parent / 'models' / self.robot_model_dir / 'xml').resolve()
                                import os
                                new_child.attrib['file'] = os.path.relpath(str(include_abs_path), str(output_dir))
                            new_body.append(new_child)
                            continue
                        
                        new_child = ET.Element(child.tag)
                        new_child.attrib.update(child.attrib)
                        new_child.text = child.text
                        new_child.tail = child.tail
                        
                        # 更新 geom 中的 name, mesh 和 material 引用
                        if child.tag == 'geom':
                            if 'name' in new_child.attrib:
                                original_name = new_child.attrib['name']
                                new_child.attrib['name'] = f"{name}_{original_name}"
                            if 'mesh' in new_child.attrib:
                                original_mesh = new_child.attrib['mesh']
                                new_child.attrib['mesh'] = f"{name}_{original_mesh}"
                            if 'material' in new_child.attrib:
                                original_material = new_child.attrib['material']
                                new_child.attrib['material'] = f"{name}_{original_material}"
                            # 若 config 指定了 rgba，覆盖视觉 geom 的颜色
                            # 需移除 material 引用，否则 MuJoCo 中 material 优先于 rgba
                            if obj.get('rgba'):
                                g = new_child.attrib.get('group', '0')
                                if g != '3':  # 只改视觉 geom，不改碰撞体
                                    new_child.attrib['rgba'] = obj['rgba']
                                    if 'material' in new_child.attrib:
                                        del new_child.attrib['material']
                            # 碰撞 geom（group 3）：应用 config 中的 friction 与接触求解器参数
                            if obj.get('movable'):
                                g = new_child.attrib.get('group', '0')
                                if g == '3':
                                    if obj.get('friction'):
                                        new_child.attrib['friction'] = obj['friction']
                                    if obj.get('condim') is not None:
                                        new_child.attrib['condim'] = str(obj['condim'])
                                    if 'contype' not in new_child.attrib:
                                        new_child.attrib['contype'] = '1'
                                    if 'conaffinity' not in new_child.attrib:
                                        new_child.attrib['conaffinity'] = '1'
                                    # solref/solimp 提高抓取接触稳定性
                                    new_child.attrib['solref'] = obj.get('solref', '0.02 1')
                                    new_child.attrib['solimp'] = obj.get('solimp', '0.9 0.95 0.001')
                        
                        # 递归复制子元素
                        for subchild in child:
                            new_child.append(subchild)
                        
                        new_body.append(new_child)
                    
                    # 可移动物体：若 config 指定 mass，确保有 inertial
                    if obj.get('movable') and obj.get('mass') is not None:
                        existing = new_body.find('inertial')
                        if existing is not None:
                            existing.attrib['mass'] = str(obj['mass'])
                            if obj.get('diaginertia'):
                                existing.attrib['diaginertia'] = obj['diaginertia']
                            else:
                                m = float(obj['mass'])
                                d = max(0.001, m * 0.02)
                                existing.attrib['diaginertia'] = f"{d} {d} {d}"
                        else:
                            m = float(obj['mass'])
                            d = max(0.001, m * 0.02)
                            inert = ET.Element('inertial', pos='0 0 0', mass=str(m), diaginertia=f"{d} {d} {d}")
                            new_body.insert(0, inert)
                    
                    self.worldbody.append(new_body)
            
            print(f"✓ Loaded external model: {xml_file}")
            
        except Exception as e:
            print(f"✗ Failed to load external model '{xml_file}': {e}")
    
    def _add_keyframe(self):
        """添加关键帧"""
        keyframe_cfg = self.config.get('keyframe')
        if keyframe_cfg:
            keyframe = ET.SubElement(self.root, 'keyframe')
            ET.SubElement(keyframe, 'key',
                         name=keyframe_cfg.get('name', 'home'),
                         qpos=keyframe_cfg.get('qpos', '0 0 0.98 1 0 0 0 0'))
    
    def build(self, seed=None):
        """构建场景

        Args:
            seed: 随机种子。若提供，则根据 randomization 配置：
                  1. seed % N 选择一套位置预设
                  2. shuffleable_parts 中的零件位置随机打乱
        """
        if seed is not None:
            self._apply_randomization(seed)
        self._init_xml()
        self._add_materials()
        self._add_ground()
        self._add_lights()
        self._add_rooms()
        self._add_objects()
        self._add_keyframe()
        return self

    def _apply_randomization(self, seed):
        """根据种子应用位置随机化"""
        rand_cfg = self.config.get('randomization')
        if not rand_cfg:
            return

        presets = rand_cfg.get('presets', [])
        if not presets:
            return

        # 选择预设
        preset_idx = seed % len(presets)
        preset = presets[preset_idx]
        print(f"[Randomization] seed={seed}, preset={preset_idx}/{len(presets)}")

        # 构建 name -> object 的索引
        objects = self.config.get('objects', [])
        obj_by_name = {}
        for obj in objects:
            name = obj.get('name')
            if name:
                obj_by_name[name] = obj

        # 应用预设位置覆盖
        for obj_name, overrides in preset.items():
            if obj_name in obj_by_name:
                for key, value in overrides.items():
                    obj_by_name[obj_name][key] = value
                print(f"  preset override: {obj_name} -> pos={overrides.get('pos', '?')}")

        # 零件位置随机交换
        shuffle_names = rand_cfg.get('shuffleable_parts', [])
        if len(shuffle_names) >= 2:
            rng = random.Random(seed)
            # 收集当前位置（pos + euler）
            positions = []
            for name in shuffle_names:
                obj = obj_by_name.get(name)
                if obj:
                    positions.append({
                        'pos': obj.get('pos', '0 0 0'),
                        'euler': obj.get('euler'),
                    })

            # 打乱位置
            rng.shuffle(positions)

            # 重新分配
            for i, name in enumerate(shuffle_names):
                obj = obj_by_name.get(name)
                if obj and i < len(positions):
                    obj['pos'] = positions[i]['pos']
                    if positions[i]['euler'] is not None:
                        obj['euler'] = positions[i]['euler']
                    print(f"  shuffle: {name} -> pos={positions[i]['pos']}")
    
    def save(self, output_file):
        """保存场景到文件"""
        if self.root is None:
            raise RuntimeError("Please call build() before save()")
        
        # 美化 XML
        rough_string = ET.tostring(self.root, encoding='utf-8')
        reparsed = minidom.parseString(rough_string)
        xml_str = reparsed.toprettyxml(indent="  ", encoding='utf-8').decode('utf-8')
        
        # 移除多余空行
        lines = [line for line in xml_str.split('\n') if line.strip()]
        xml_str = '\n'.join(lines)
        
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(xml_str)
        
        print(f"✓ Scene saved to: {output_file}")
        return self


def main():
    """主函数：支持多种使用方式"""
    import sys
    import argparse

    parser = argparse.ArgumentParser(description="MuJoCo 场景生成器")
    parser.add_argument('config', nargs='?', default=None,
                        help='配置文件路径（默认: ../config/craic_room.yaml）')
    parser.add_argument('-o', '--output', default=None,
                        help='输出文件路径')
    parser.add_argument('--seed', type=int, default=None,
                        help='随机种子（控制物体位置随机化）')
    parser.add_argument('--robot-version', default=None,
                        help='机器人版本号（默认从环境变量 ROBOT_VERSION 读取）')
    args = parser.parse_args()

    # 配置文件
    if args.config is None:
        script_dir = Path(__file__).parent
        config_file = script_dir.parent / 'config' / 'craic_room.yaml'
        if not config_file.exists():
            print(f"默认配置文件不存在: {config_file}")
            return
        config_file = str(config_file)
    else:
        config_file = args.config

    try:
        builder = SceneBuilder(robot_version=args.robot_version)

        # 自动推断输出文件名
        if args.output:
            output_file = args.output
        else:
            output_name = "my_scene.xml"
            script_dir = Path(__file__).parent
            output_dir = script_dir.parent / 'models' / builder.robot_model_dir / 'xml'
            output_dir.mkdir(parents=True, exist_ok=True)
            output_file = output_dir / output_name

        print(f"机器人版本: {builder.robot_version}")
        print(f"配置文件: {config_file}")
        print(f"输出文件: {output_file}")
        if args.seed is not None:
            print(f"随机种子: {args.seed}")

        builder.load_config(config_file)
        builder.build(seed=args.seed)
        builder.save(str(output_file))
        print("\n场景生成成功！")
    except Exception as e:
        print(f"\n生成失败: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
