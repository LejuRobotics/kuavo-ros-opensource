#!/usr/bin/env python3
"""
Matplotlib visualization of Kuavo robot body links' positions and orientations.
Shows:
- 3D points for each body link
- Parent-child connections using MuJoCo model hierarchy
- Small orientation axes (triads) for each body from its quaternion
"""

import argparse
import time
from typing import Dict, Optional

import numpy as np
import mujoco as mj
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (needed for 3D projection)

from kuavo_gmr.params import ROBOT_XML_DICT


def draw_axes_triad(ax, origin: np.ndarray, rot_mat: np.ndarray, axis_len: float = 0.05):
    """
    Draw a small RGB triad at origin with orientation rot_mat.
    X=red, Y=green, Z=blue.
    """
    x_axis = rot_mat[:, 0] * axis_len
    y_axis = rot_mat[:, 1] * axis_len
    z_axis = rot_mat[:, 2] * axis_len
    ax.plot([origin[0], origin[0] + x_axis[0]],
            [origin[1], origin[1] + x_axis[1]],
            [origin[2], origin[2] + x_axis[2]], color='r', linewidth=1.5)
    ax.plot([origin[0], origin[0] + y_axis[0]],
            [origin[1], origin[1] + y_axis[1]],
            [origin[2], origin[2] + y_axis[2]], color='g', linewidth=1.5)
    ax.plot([origin[0], origin[0] + z_axis[0]],
            [origin[1], origin[1] + z_axis[1]],
            [origin[2], origin[2] + z_axis[2]], color='b', linewidth=1.5)


def get_body_hierarchy(model):
    """
    Build a dictionary mapping body names to their parent body names.
    Returns: Dict[body_name, parent_body_name or None]
    """
    hierarchy = {}
    for i in range(model.nbody):
        body_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i)
        parent_id = model.body_parentid[i]
        if parent_id >= 0:
            parent_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, parent_id)
        else:
            parent_name = None
        hierarchy[body_name] = parent_name
    return hierarchy


def get_body_poses(model, data):
    """
    Extract body positions and orientations from MuJoCo data.
    Returns: Dict[body_name, {'position': np.array, 'quat': np.array}]
    """
    bodies = {}
    for i in range(model.nbody):
        body_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i)
        if body_name:
            # Get position (xpos is 3D position)
            position = data.xpos[i].copy()
            # Get orientation (xquat is wxyz format)
            quat = data.xquat[i].copy()
            bodies[body_name] = {
                'position': position,
                'quat': quat,
            }
    return bodies


def visualize_single_frame(ax, model, data, axis_len: float, show_labels: bool, hierarchy: Dict):
    """
    Visualize a single frame of the robot.
    """
    bodies = get_body_poses(model, data)
    
    if not bodies:
        return
    
    # Plot body points
    pts = np.array([bodies[name]['position'] for name in bodies.keys()])
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c='k', s=10, alpha=0.8, depthshade=True)
    
    # Compute key scales
    shoulder_width = np.linalg.norm(bodies['zarm_r2_link']['position'] - bodies['zarm_l2_link']['position'])  # 0.585
    hip_width = np.linalg.norm(bodies['leg_r2_link']['position'] - bodies['leg_l2_link']['position'])         # 0.173
    up_arm_length = np.linalg.norm(bodies['zarm_r4_link']['position'] - bodies['zarm_r2_link']['position'])   # 0.284
    down_arm_length = np.linalg.norm(bodies['zarm_r4_link']['position'] - bodies['zarm_r7_link']['position']) # 0.255
    up_leg_length = np.linalg.norm(bodies['leg_r4_link']['position'] - bodies['leg_r3_link']['position'])     # 0.284
    # up_leg_length = np.linalg.norm(bodies['leg_r4_link']['position'] - bodies['leg_r2_link']['position'])     # 0.413
    down_leg_length = np.linalg.norm(bodies['leg_r4_link']['position'] - bodies['leg_r6_link']['position'])   # 0.346
    print(shoulder_width, hip_width, up_arm_length, down_arm_length, up_leg_length, down_leg_length)
    # height
    total_height = (bodies['zhead_1_link']['position'] - bodies['leg_l5_link']['position'])[2] # 1.42
    base_height = (bodies['base_link']['position'])[2] # 0.85
    print(base_height)

    # Labels
    if show_labels:
        for name, data_dict in bodies.items():
            p = data_dict['position']
            ax.text(p[0], p[1], p[2], name, fontsize=6)
    
    # Connections based on hierarchy
    for child_name, parent_name in hierarchy.items():
        if parent_name and child_name in bodies and parent_name in bodies:
            p = bodies[parent_name]['position']
            c = bodies[child_name]['position']
            # Color coding: left=blue, right=orange, else=green
            color = '#6495ED' if ('left' in child_name.lower() or '_l' in child_name.lower()) else \
                   ('#FFA07A' if ('right' in child_name.lower() or '_r' in child_name.lower()) else '#90EE90')
            ax.plot([p[0], c[0]], [p[1], c[1]], [p[2], c[2]], color=color, linewidth=1.5)
    
    # Orientation axes for each body
    for name, data_dict in bodies.items():
        # xquat is in wxyz format, convert to xyzw for scipy
        quat_wxyz = data_dict['quat']
        quat_xyzw = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]
        rot = R.from_quat(quat_xyzw).as_matrix()
        draw_axes_triad(ax, data_dict['position'], rot, axis_len=axis_len)


def setup_axes(ax):
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_box_aspect([1, 1, 1])


def auto_limits_from_points(ax, pts: np.ndarray, pad: float = 0.1):
    """
    Automatically set axis limits to fit the points with padding.
    """
    if pts.size == 0:
        return
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    ranges = maxs - mins
    max_range = max(ranges.max(), 1e-3)
    center = (mins + maxs) / 2.0
    half = max_range / 2.0 + pad
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)


def main():
    parser = argparse.ArgumentParser(description='Matplotlib visualization of Kuavo robot body links')
    parser.add_argument('--robot', type=str, default='kuavo_s45',
                        choices=list(ROBOT_XML_DICT.keys()),
                        help='Robot type to visualize')
    parser.add_argument('--qpos', type=str, default=None,
                        help='Path to motion file (.pkl) or qpos array file. If None, uses default pose.')
    parser.add_argument('--frame_index', type=int, default=None,
                        help='Specific frame index to visualize (0-based). If omitted, animates.')
    parser.add_argument('--max_frames', type=int, default=300,
                        help='Max frames to animate (ignored if frame_index is set)')
    parser.add_argument('--stride', type=int, default=1,
                        help='Frame stride for animation sampling')
    parser.add_argument('--axes_length', type=float, default=0.05,
                        help='Length of per-body orientation axes')
    parser.add_argument('--labels', action='store_true',
                        help='Show body name labels')
    parser.add_argument('--interval_ms', type=int, default=30,
                        help='Animation frame interval in milliseconds')
    args = parser.parse_args()
    
    # Load robot model
    xml_file = str(ROBOT_XML_DICT[args.robot])
    print(f"Loading robot model: {xml_file}")
    model = mj.MjModel.from_xml_path(xml_file)
    data = mj.MjData(model)
    
    # Get body hierarchy
    hierarchy = get_body_hierarchy(model)
    
    # Load motion data if provided
    qpos_list = None
    if args.qpos:
        import pickle
        with open(args.qpos, 'rb') as f:
            motion_data = pickle.load(f)
        if isinstance(motion_data, dict):
            # Assume it's a motion data dict with 'dof_pos', 'root_pos', 'root_rot'
            if 'root_pos' in motion_data and 'root_rot' in motion_data and 'dof_pos' in motion_data:
                root_pos = motion_data['root_pos']
                root_rot = motion_data['root_rot']  # xyzw format
                dof_pos = motion_data['dof_pos']
                # Convert root_rot from xyzw to wxyz and combine with root_pos
                num_frames = len(root_pos)
                qpos_list = []
                for i in range(num_frames):
                    # root_rot is xyzw, convert to wxyz
                    rot_xyzw = root_rot[i]
                    rot_wxyz = np.array([rot_xyzw[3], rot_xyzw[0], rot_xyzw[1], rot_xyzw[2]])
                    qpos = np.concatenate([root_pos[i], rot_wxyz, dof_pos[i]])
                    qpos_list.append(qpos)
            else:
                # Try to load as list of qpos arrays
                qpos_list = motion_data
        else:
            qpos_list = motion_data
    
    # If no motion data, use default pose
    if qpos_list is None:
        mj.mj_forward(model, data)
        qpos_list = [data.qpos.copy()]
    
    print(f"Frames detected: {len(qpos_list)}")
    
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    setup_axes(ax)
    
    if args.frame_index is not None:
        frame_index = max(0, min(args.frame_index, len(qpos_list) - 1))
        data.qpos[:] = qpos_list[frame_index]
        mj.mj_forward(model, data)
        # visualize_single_frame(ax, model, data, axis_len=args.axes_length, show_labels=args.labels, hierarchy=hierarchy)
        visualize_single_frame(ax, model, data, axis_len=args.axes_length, show_labels=True, hierarchy=hierarchy)
        # Auto-scale limits
        bodies = get_body_poses(model, data)
        if bodies:
            pts = np.array([b['position'] for b in bodies.values()])
            auto_limits_from_points(ax, pts)
        ax.set_title(f"Frame {frame_index} - {args.robot}")
        plt.show()
        return
    
    # Animation
    sample_indices = list(range(0, len(qpos_list), args.stride))
    if args.max_frames is not None:
        sample_indices = sample_indices[:args.max_frames]
    print(f"Animating {len(sample_indices)} frames (stride={args.stride})")
    
    for idx, frame_idx in enumerate(sample_indices):
        ax.cla()
        setup_axes(ax)
        data.qpos[:] = qpos_list[frame_idx]
        mj.mj_forward(model, data)
        # visualize_single_frame(ax, model, data, axis_len=args.axes_length, show_labels=args.labels, hierarchy=hierarchy)
        visualize_single_frame(ax, model, data, axis_len=args.axes_length, show_labels=True, hierarchy=hierarchy)
        # Auto-scale limits
        bodies = get_body_poses(model, data)
        if bodies:
            pts = np.array([b['position'] for b in bodies.values()])
            auto_limits_from_points(ax, pts)
        ax.set_title(f"Frame {idx+1}/{len(sample_indices)} ({frame_idx}/{len(qpos_list)}) - {args.robot}")
        plt.pause(max(args.interval_ms / 1000.0, 1e-3))
    
    print("Animation complete.")
    plt.show()


if __name__ == '__main__':
    main()

