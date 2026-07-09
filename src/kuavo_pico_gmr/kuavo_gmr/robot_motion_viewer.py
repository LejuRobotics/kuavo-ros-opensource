import os
import time
import mujoco as mj
import mujoco.viewer as mjv
import imageio
from scipy.spatial.transform import Rotation as R
from kuavo_gmr import ROBOT_XML_DICT, ROBOT_BASE_DICT, VIEWER_CAM_DISTANCE_DICT
from loop_rate_limiters import RateLimiter
import numpy as np
from rich import print


# 定义骨骼连接关系（用于绘制骨架）
SKELETON_CONNECTIONS = [
    # 躯干
    ("pelvis", "3_link"),     # Pelvis -> SPINE1
    ("3_link", "6_link"),     # SPINE1 -> SPINE2
    ("6_link", "9_link"),     # SPINE2 -> SPINE3
    ("9_link", "12_link"),    # SPINE3 -> NECK
    ("12_link", "15_link"),   # NECK -> HEAD
    
    # 左臂
    ("9_link", "13_link"),    # SPINE3 -> LEFT_COLLAR
    ("13_link", "left_shoulder"),
    ("left_shoulder", "left_elbow"),
    ("left_elbow", "left_wrist"),
    ("left_wrist", "22_link"),  # LEFT_HAND
    
    # 右臂
    ("9_link", "14_link"),    # SPINE3 -> RIGHT_COLLAR
    ("14_link", "right_shoulder"),
    ("right_shoulder", "right_elbow"),
    ("right_elbow", "right_wrist"),
    ("right_wrist", "23_link"),  # RIGHT_HAND
    
    # 左腿
    ("pelvis", "left_hip"),
    ("left_hip", "left_knee"),
    ("left_knee", "7_link"),  # LEFT_ANKLE
    ("7_link", "left_foot"),
    
    # 右腿
    ("pelvis", "right_hip"),
    ("right_hip", "right_knee"),
    ("right_knee", "8_link"),  # RIGHT_ANKLE
    ("8_link", "right_foot"),
]


def draw_frame(
    pos,
    mat,
    v,
    size,
    joint_name=None,
    orientation_correction=R.from_euler("xyz", [0, 0, 0]),
    pos_offset=np.array([0, 0, 0]),
):
    """绘制坐标轴（xyz轴）"""
    rgba_list = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]
    for i in range(3):
        geom = v.user_scn.geoms[v.user_scn.ngeom]
        mj.mjv_initGeom(
            geom,
            type=mj.mjtGeom.mjGEOM_ARROW,
            size=[0.01, 0.01, 0.01],
            pos=pos + pos_offset,
            mat=mat.flatten(),
            rgba=rgba_list[i],
        )
        if joint_name is not None:
            geom.label = joint_name  # 这里赋名字
        fix = orientation_correction.as_matrix()
        mj.mjv_connector(
            v.user_scn.geoms[v.user_scn.ngeom],
            type=mj.mjtGeom.mjGEOM_ARROW,
            width=0.005,
            from_=pos + pos_offset,
            to=pos + pos_offset + size * (mat @ fix)[:, i],
        )
        v.user_scn.ngeom += 1


def draw_sphere(pos, v, size=0.03, rgba=None, label=None, pos_offset=np.array([0, 0, 0])):
    """绘制球体（用于骨骼关节点）"""
    if rgba is None:
        rgba = [0.2, 0.6, 1.0, 0.8]  # 默认蓝色半透明
    
    geom = v.user_scn.geoms[v.user_scn.ngeom]
    mj.mjv_initGeom(
        geom,
        type=mj.mjtGeom.mjGEOM_SPHERE,
        size=[size, size, size],
        pos=pos + pos_offset,
        mat=np.eye(3).flatten(),
        rgba=rgba,
    )
    if label is not None:
        geom.label = label
    v.user_scn.ngeom += 1


def draw_line(from_pos, to_pos, v, width=0.01, rgba=None, pos_offset=np.array([0, 0, 0])):
    """绘制线段（用于骨骼连接）"""
    if rgba is None:
        rgba = [0.8, 0.8, 0.8, 0.6]  # 默认灰色半透明
    
    geom = v.user_scn.geoms[v.user_scn.ngeom]
    mj.mjv_connector(
        geom,
        type=mj.mjtGeom.mjGEOM_CAPSULE,
        width=width,
        from_=from_pos + pos_offset,
        to=to_pos + pos_offset,
    )
    geom.rgba[:] = rgba
    v.user_scn.ngeom += 1


def draw_pico_skeleton(human_motion_data, v, 
                       pos_offset=np.array([0, 0, 0]),
                       draw_joints=True,
                       draw_bones=True,
                       draw_axes=False,
                       joint_size=0.025,
                       bone_width=0.008,
                       show_labels=False):
    """
    绘制 PICO 骨架
    
    Args:
        human_motion_data: dict of {"bone_name": (pos, quat)}
        v: MuJoCo viewer
        pos_offset: 位置偏移
        draw_joints: 是否绘制关节球体
        draw_bones: 是否绘制骨骼连接线
        draw_axes: 是否绘制坐标轴
        joint_size: 关节球体大小
        bone_width: 骨骼线条宽度
        show_labels: 是否显示标签
    """
    if human_motion_data is None:
        return
    
    # 1. 绘制关节球体
    if draw_joints:
        for bone_name, (pos, rot) in human_motion_data.items():
            # 根据骨骼类型设置不同颜色和大小
            if "hand" in bone_name.lower() or "22_link" in bone_name or "23_link" in bone_name:
                rgba = [1.0, 0.5, 0.0, 0.8]  # 橙色 - 手
                size = joint_size * 1.2
            elif "foot" in bone_name.lower() or "10_link" in bone_name or "11_link" in bone_name:
                rgba = [1.0, 0.8, 0.0, 0.8]  # 黄色 - 脚
                size = joint_size * 1.2
            elif "head" in bone_name.lower() or "15_link" in bone_name:
                rgba = [1.0, 0.2, 0.2, 0.8]  # 红色 - 头
                size = joint_size * 2.0
            elif "pelvis" in bone_name.lower() or "_link" in bone_name and any(x in bone_name for x in ["3_", "6_", "9_", "12_"]):
                rgba = [0.2, 1.0, 0.2, 0.8]  # 绿色 - 躯干
                size = joint_size * 1.5
            else:
                rgba = [0.2, 0.6, 1.0, 0.8]  # 蓝色 - 其他关节
                size = joint_size
            
            draw_sphere(
                pos=pos,
                v=v,
                size=size,
                rgba=rgba,
                label=bone_name if show_labels else None,
                pos_offset=pos_offset
            )
    
    # 2. 绘制骨骼连接线
    if draw_bones:
        for from_bone, to_bone in SKELETON_CONNECTIONS:
            if from_bone in human_motion_data and to_bone in human_motion_data:
                from_pos = human_motion_data[from_bone][0]
                to_pos = human_motion_data[to_bone][0]
                
                # 根据骨骼部位设置颜色
                if "hand" in from_bone.lower() or "hand" in to_bone.lower():
                    rgba = [1.0, 0.5, 0.0, 0.6]  # 橙色
                elif "foot" in from_bone.lower() or "foot" in to_bone.lower():
                    rgba = [1.0, 0.8, 0.0, 0.6]  # 黄色
                elif any(x in from_bone for x in ["3_", "6_", "9_", "12_"]) or \
                     any(x in to_bone for x in ["3_", "6_", "9_", "12_"]):
                    rgba = [0.2, 1.0, 0.2, 0.6]  # 绿色 - 躯干
                else:
                    rgba = [0.8, 0.8, 0.8, 0.6]  # 灰色
                
                draw_line(
                    from_pos=from_pos,
                    to_pos=to_pos,
                    v=v,
                    width=bone_width,
                    rgba=rgba,
                    pos_offset=pos_offset
                )
    
    # 3. 可选：绘制坐标轴（原有功能）
    if draw_axes:
        for bone_name, (pos, rot) in human_motion_data.items():
            draw_frame(
                pos=pos,
                mat=R.from_quat(rot, scalar_first=True).as_matrix(),
                v=v,
                size=0.05,
                joint_name=bone_name if show_labels else None,
                pos_offset=pos_offset
            )

class RobotMotionViewer:
    def __init__(self,
                robot_type,
                camera_follow=True,
                motion_fps=30,
                transparent_robot=0,
                # video recording
                record_video=False,
                video_path=None,
                video_width=640,
                video_height=480,
                keyboard_callback=None,
                ):
        
        self.robot_type = robot_type
        self.xml_path = ROBOT_XML_DICT[robot_type]
        self.model = mj.MjModel.from_xml_path(str(self.xml_path))
        self.data = mj.MjData(self.model)
        self.robot_base = ROBOT_BASE_DICT[robot_type]
        self.viewer_cam_distance = VIEWER_CAM_DISTANCE_DICT[robot_type]
        mj.mj_step(self.model, self.data)
        
        self.motion_fps = motion_fps
        self.rate_limiter = RateLimiter(frequency=self.motion_fps, warn=False)
        self.camera_follow = camera_follow
        self.record_video = record_video


        self.viewer = mjv.launch_passive(
            model=self.model,
            data=self.data,
            show_left_ui=False,
            show_right_ui=False, 
            key_callback=keyboard_callback
            )      

        self.viewer.opt.flags[mj.mjtVisFlag.mjVIS_TRANSPARENT] = transparent_robot
        
        if self.record_video:
            assert video_path is not None, "Please provide video path for recording"
            self.video_path = video_path
            video_dir = os.path.dirname(self.video_path)
            
            if not os.path.exists(video_dir):
                os.makedirs(video_dir)
            self.mp4_writer = imageio.get_writer(self.video_path, fps=self.motion_fps)
            print(f"Recording video to {self.video_path}")
            
            # Initialize renderer for video recording
            self.renderer = mj.Renderer(self.model, height=video_height, width=video_width)
        
    def step(self, 
            # robot data
            root_pos, root_rot, dof_pos, 
            # human data
            human_motion_data=None, 
            show_human_body_name=False,
            # PICO skeleton visualization options
            use_pico_skeleton=True,         # 是否使用新的 PICO 骨架绘制模式
            draw_pico_joints=True,          # 是否绘制关节球体
            draw_pico_bones=True,           # 是否绘制骨骼连接线
            draw_pico_axes=False,           # 是否绘制坐标轴（原模式）
            pico_joint_size=0.025,          # 关节球体大小
            pico_bone_width=0.008,          # 骨骼线条宽度
            # legacy options (for backward compatibility)
            human_point_scale=0.1,          # 坐标轴模式下的缩放（已废弃，使用 pico_joint_size）
            # human pos offset add for visualization    
            human_pos_offset=np.array([0.0, 0.0, 0]),
            # rate limit
            rate_limit=True, 
            follow_camera=True,
            ):
        """
        可视化机器人运动和人体骨架
        
        Args:
            root_pos: 机器人根位置
            root_rot: 机器人根旋转（四元数，scalar-first）
            dof_pos: 机器人关节角度
            human_motion_data: dict of {"bone_name": (pos, quat)} - PICO 骨骼数据
            show_human_body_name: 是否显示骨骼名称标签
            use_pico_skeleton: 是否使用新的 PICO 骨架绘制模式（球体+连线）
            draw_pico_joints: 是否绘制关节球体
            draw_pico_bones: 是否绘制骨骼连接线
            draw_pico_axes: 是否绘制坐标轴（旧模式）
            pico_joint_size: 关节球体大小
            pico_bone_width: 骨骼线条宽度
            human_point_scale: 坐标轴缩放（旧模式）
            human_pos_offset: 人体骨架位置偏移
            rate_limit: 是否限制帧率
            follow_camera: 相机是否跟随机器人
        """
        
        # 更新机器人状态
        self.data.qpos[:3] = root_pos
        self.data.qpos[3:7] = root_rot # quat need to be scalar first! for mujoco
        self.data.qpos[7:] = dof_pos
        
        mj.mj_forward(self.model, self.data)
        
        # 相机跟随
        if follow_camera:
            self.viewer.cam.lookat = self.data.xpos[self.model.body(self.robot_base).id]
            self.viewer.cam.distance = self.viewer_cam_distance
            self.viewer.cam.elevation = -10  # 正面视角，轻微向下看
            # self.viewer.cam.azimuth = 180    # 正面朝向机器人
        
        # 绘制人体骨架
        if human_motion_data is not None:
            # 清空之前的自定义几何体
            self.viewer.user_scn.ngeom = 0
            
            if use_pico_skeleton:
                # 新模式：使用球体和连线绘制完整骨架
                draw_pico_skeleton(
                    human_motion_data=human_motion_data,
                    v=self.viewer,
                    pos_offset=human_pos_offset,
                    draw_joints=draw_pico_joints,
                    draw_bones=draw_pico_bones,
                    draw_axes=draw_pico_axes,
                    joint_size=pico_joint_size,
                    bone_width=pico_bone_width,
                    show_labels=show_human_body_name
                )
            else:
                # 旧模式：只绘制坐标轴（向后兼容）
                for human_body_name, (pos, rot) in human_motion_data.items():
                    draw_frame(
                        pos,
                        R.from_quat(rot, scalar_first=True).as_matrix(),
                        self.viewer,
                        human_point_scale,
                        pos_offset=human_pos_offset,
                        joint_name=human_body_name if show_human_body_name else None
                    )

        # 同步viewer
        self.viewer.sync()
        
        # 帧率限制
        if rate_limit is True:
            self.rate_limiter.sleep()

        # 录制视频
        if self.record_video:
            # Use renderer for proper offscreen rendering
            self.renderer.update_scene(self.data, camera=self.viewer.cam)
            img = self.renderer.render()
            self.mp4_writer.append_data(img)
    
    def close(self):
        self.viewer.close()
        time.sleep(0.5)
        if self.record_video:
            self.mp4_writer.close()
            print(f"Video saved to {self.video_path}")
