#!/usr/bin/env python3
"""
Matplotlib visualization of Pico bone poses from CSV.
Shows:
- 3D points for each bone/link
- Parent-child connections using a URDF-aware hierarchy
- Small orientation axes (triads) for each bone from its quaternion
"""

import argparse
import time
from typing import Dict, Optional, List

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (needed for 3D projection)

# URDF-based hierarchy for kuavo_s45/biped_s45.urdf subset (25 links used in extraction)
URDF_BONE_HIERARCHY: Dict[str, Optional[str]] = {
    # Root
    'pelvis':           None,       # 0
    # Lower body
    'left_hip':         'pelvis',
    'right_hip':        'pelvis',
    'spine1':           'pelvis',
    'left_knee':        'left_hip',
    'right_knee':       'right_hip',
    'left_ankle':       'left_knee',
    'right_ankle':      'right_knee',
    'left_foot':        'left_ankle',
    'right_foot':       'right_ankle',

    # Upper spine
    'spine2':           'spine1',
    'spine3':           'spine2',
    # Neck & head
    'neck':             'spine3',
    'head':             'neck',

    # Shoulders / collar
    'left_collar':      'spine3',
    'right_collar':     'spine3',

    # Left arm
    'left_shoulder':    'left_collar',
    'left_elbow':       'left_shoulder',
    'left_wrist':       'left_elbow',
    'left_hand':        'left_wrist',

    # Right arm
    'right_shoulder':   'right_collar',
    'right_elbow':      'right_shoulder',
    'right_wrist':      'right_elbow',
    'right_hand':       'right_wrist',
}


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


def visualize_single_frame(ax, frame_df: pd.DataFrame, axis_len: float, show_labels: bool):
    # Build dict of bones
    bones = {}
    for _, row in frame_df.iterrows():
        bones[row['bone_name']] = {
            'position': np.array([row['position_x'], row['position_y'], row['position_z']], dtype=float),
            'quat': np.array([row['orientation_x'], row['orientation_y'], row['orientation_z'], row['orientation_w']], dtype=float),
        }
    if not bones:
        return
    
    if "24_link" in bones:
        del bones["24_link"]
    
    # Choose hierarchy based on present names
    hierarchy = URDF_BONE_HIERARCHY
    # Plot bone points
    pts = np.array([bones[name]['position'] for name in bones.keys()])
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c='k', s=10, alpha=0.8, depthshade=True)

    # Labels
    if show_labels:
        for name, data in bones.items():
            p = data['position']
            ax.text(p[0], p[1], p[2], name, fontsize=6)

    # Connections
    for child_name, parent_name in hierarchy.items():
        if parent_name and child_name in bones and parent_name in bones:
            p = bones[parent_name]['position']
            c = bones[child_name]['position']
            color = '#6495ED' if ('Left' in child_name or '_l' in child_name) else ('#FFA07A' if ('Right' in child_name or '_r' in child_name) else '#90EE90')
            ax.plot([p[0], c[0]], [p[1], c[1]], [p[2], c[2]], color=color, linewidth=1.5)

    # Orientation axes for each bone
    for name, data in bones.items():
        rot = R.from_quat(data['quat']).as_matrix()
        draw_axes_triad(ax, data['position'], rot, axis_len=axis_len)


def setup_axes(ax):
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_box_aspect([1, 1, 1])
    # Right-handed with Z up is the data; matplotlib default is fine. Just keep equal-ish limits later.


def auto_limits_from_points(ax, pts: np.ndarray, pad: float = 0.1):
    # This function, auto_limits_from_points, automatically sets the x, y, and z axis limits for a 3D matplotlib axes object (ax)
    # so that the visible plot region fits the given set of points (pts), with some optional padding.
    # It computes the minimum and maximum for each coordinate, finds the largest range among axes,
    # centers the axes on the midpoint of the data, and applies equal scaling for all axes, ensuring the 3D object isn't distorted.
    # 'pad' specifies extra margin (in world units) added around the point cloud.
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
    parser = argparse.ArgumentParser(description='Matplotlib visualization of Pico bone poses from CSV')
    # parser.add_argument('--csv_file', type=str, default='pico_data/leju_pico_bone_poses_with_names.csv',
    # parser.add_argument('--csv_file', type=str, default='pico_data/pico_full_data.csv',
    # parser.add_argument('--csv_file', type=str, default='pico_data/pico_data_ori.csv',
    parser.add_argument('--csv_file', type=str, default='pico_data/time_stamp.csv',
                        help='Path to CSV file (output of extract_bone_poses_with_names.py)')
    parser.add_argument('--frame_index', type=int, default=100,
                        help='Specific frame index to visualize (0-based). If omitted, animates.')
    parser.add_argument('--max_frames', type=int, default=4000,
                        help='Max frames to animate (ignored if frame_index is set)')
    parser.add_argument('--stride', type=int, default=1,
                        help='Frame stride for animation sampling')
    parser.add_argument('--axes_length', type=float, default=0.05,
                        help='Length of per-bone orientation axes')
    parser.add_argument('--labels', type=bool, default=True,
                        help='Show bone name labels')
    parser.add_argument('--interval_ms', type=int, default=2,
                        help='Animation frame interval in milliseconds')
    args = parser.parse_args()

    print(f"Loading {args.csv_file} ...")
    df = pd.read_csv(args.csv_file)
    timestamps = sorted(df['Time'].unique())
    print(f"Frames detected: {len(timestamps)}")

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    setup_axes(ax)

    if args.frame_index is not None:
        frame_index = max(0, min(args.frame_index, len(timestamps) - 1))
        frame_time = timestamps[frame_index]
        frame_df = df[df['Time'] == frame_time]
        visualize_single_frame(ax, frame_df, axis_len=args.axes_length, show_labels=args.labels)
        # Auto-scale limits based on this frame
        pts = frame_df[['position_x', 'position_y', 'position_z']].to_numpy()
        auto_limits_from_points(ax, pts)
        ax.set_title(f"Frame {frame_index} at t={frame_time:.3f}s")
        plt.show()
        return

    # Animation
    sample_times = timestamps[::args.stride]
    if args.max_frames is not None:
        sample_times = sample_times[:args.max_frames]
    print(f"Animating {len(sample_times)} frames (stride={args.stride})")

    for idx, t in enumerate(sample_times):
        ax.cla()
        setup_axes(ax)
        frame_df = df[df['Time'] == t]
        visualize_single_frame(ax, frame_df, axis_len=args.axes_length, show_labels=args.labels)
        pts = frame_df[['position_x', 'position_y', 'position_z']].to_numpy()
        pts = pts[:-1]
        auto_limits_from_points(ax, pts)
        ax.set_title(f"Frame {idx+1}/{len(sample_times)}  t={t:.3f}s")
        plt.pause(max(args.interval_ms / 1000.0, 1e-3))

    print("Animation complete.")
    plt.show()


if __name__ == '__main__':
    main()


