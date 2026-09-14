"""
This file is used to load the Pico motion data.
"""
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R


def get_bone_name(pico_bone_name):
    BONE_NAME_MAP = {
        "0_link": "pelvis",

        "1_link": "left_hip",
        "2_link": "right_hip",
        "4_link": "left_knee",
        "5_link": "right_knee",
        "7_link": "left_foot",
        "8_link": "right_foot",
        
        "16_link": "left_shoulder",
        "17_link": "right_shoulder",
        "18_link": "left_elbow",
        "19_link": "right_elbow",
        "20_link": "left_wrist",
        "21_link": "right_wrist",
    }
    if pico_bone_name in BONE_NAME_MAP:
        return BONE_NAME_MAP[pico_bone_name]
    else:
        return pico_bone_name 


def load_pico_file(
    pico_file = "pico_data/leju_pico_bone_poses_with_names.csv",
    hip_orientation_link = "spine3"
    ):
    """
    Must return a list of dictionary with the following structure:
        [{
            "body_link_name_0": (position, orientation),
            "body_link_name_1": (position, orientation),
            ...
        }]
    """
    # represented in metre and quaternion
    frames = []
    
    df = pd.read_csv(pico_file)
    # Use the same frame key as the visualizer
    timestamps = sorted(df["Time"].unique())
    
    # Build frames
    frame_timestamps = []
    for t in timestamps:
        frame_df = df[df["Time"] == t]
        # Optional: drop 24_link to match visualizer behavior
        frame_df = frame_df[frame_df["bone_name"] != "24_link"]
        
        result = {}
        for _, row in frame_df.iterrows():
            pos = np.array([row["position_x"], row["position_y"], row["position_z"]], dtype=float)
            # CSV is xyzw; convert to scalar-first wxyz for consistency with other loaders
            quat_xyzw = np.array(
                [row["orientation_x"], row["orientation_y"], row["orientation_z"], row["orientation_w"]],
                dtype=float,
            )
            # Normalize and reorder to wxyz
            norm = np.linalg.norm(quat_xyzw)
            if norm > 0:
                quat_xyzw = quat_xyzw / norm
            quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]], dtype=float)
            result[get_bone_name(str(row["bone_name"]))] = [pos, quat_wxyz]

        # NOTE the pelvis orientation keeps static during the motion, use spine3 instead
        result['pelvis'][1] = result[hip_orientation_link][1]   # replace the orientation of pelvis with 3_link's orientation
        # TODO the foot orientation keeps static during the motion, we may integrate ankle and foot to guess one
        
        frames.append(result)
        frame_timestamps.append(int(t*1000))
    
    # Estimate human height from first frame as Z span of all points
    # if len(frames) > 0 and len(frames[0]) > 0:
    #     first_positions = np.array([v[0] for v in frames[0].values()], dtype=float)
    #     human_height = float(first_positions[:, 2].max() - first_positions[:, 2].min())
    #     # Fallback if degenerate
    #     if not np.isfinite(human_height) or human_height <= 0:
    #         human_height = 1.4
    # else:
    #     human_height = 1.4
    # NOTE the human height is estimated from the first frame, it shall be adjusted if the PICO user is changed
    human_height = 1.54 # 1.42

    return frames, human_height, frame_timestamps


if __name__ == "__main__":
    pico_file = "pico_data/pico_data_ori.csv"
    frames, human_height, frame_timestamps = load_pico_file(pico_file)
    frame_0 = frames[10]

    # compute the key distance of kuavo in pico
    print("human_height: ", human_height)
    # shoulder_width = np.linalg.norm(frame_0['left_shoulder'][0] - frame_0['right_shoulder'][0]) # 0.366 0.585
    # up_arm_length = np.linalg.norm(frame_0['left_shoulder'][0] - frame_0['left_elbow'][0])      # 0.233 0.284
    # down_arm_length = np.linalg.norm(frame_0['left_elbow'][0] - frame_0['left_wrist'][0])       # 0.290 0.255
    # hip_width = np.linalg.norm(frame_0['left_hip'][0] - frame_0['right_hip'][0])                # 0.115 0.173
    # up_leg_length = np.linalg.norm(frame_0['left_hip'][0] - frame_0['left_knee'][0])            # 0.360 0.284 0.413
    # down_leg_length = np.linalg.norm(frame_0['left_knee'][0] - frame_0['left_foot'][0])         # 0.391 0.346
    # print(shoulder_width, hip_width, up_arm_length, down_arm_length, up_leg_length, down_leg_length)
    # base_height = frame_0['pelvis'][0][2] # 0.80 0.85
    # print(base_height)
    # shoulder_width = np.linalg.norm(bodies['zarm_r2_link']['position'] - bodies['zarm_l2_link']['position'])  # 0.585
    # hip_width = np.linalg.norm(bodies['leg_r2_link']['position'] - bodies['leg_l2_link']['position'])         # 0.173
    # up_arm_length = np.linalg.norm(bodies['zarm_r4_link']['position'] - bodies['zarm_r2_link']['position'])   # 0.284
    # down_arm_length = np.linalg.norm(bodies['zarm_r4_link']['position'] - bodies['zarm_r7_link']['position']) # 0.255
    # up_leg_length = np.linalg.norm(bodies['leg_r4_link']['position'] - bodies['leg_r3_link']['position'])     # 0.284
    # down_leg_length = np.linalg.norm(bodies['leg_r4_link']['position'] - bodies['leg_r6_link']['position'])   # 0.346

    URDF_BONE_HIERARCHY = {
        'pelvis': None,
        # arm
        'left_shoulder': 'pelvis',
        'left_elbow': 'left_shoulder',
        'left_wrist': 'left_elbow',
        'right_shoulder': 'pelvis',
        'right_elbow': 'right_shoulder',
        'right_wrist': 'right_elbow',
        # leg
        'left_hip': 'pelvis',
        'left_knee': 'left_hip',
        'left_ankle': 'left_knee',
        'left_foot': 'left_ankle',
        'right_hip': 'pelvis',
        'right_knee': 'right_hip',
        'right_ankle': 'right_knee',
        'right_foot': 'right_ankle',
    }

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    import numpy as np

    def draw_axes(ax, origin, quat, axis_len=0.05):
        # NOTE: quat is wxyz
        from scipy.spatial.transform import Rotation as R
        quat_xyzw = [quat[1], quat[2], quat[3], quat[0]]
        r = R.from_quat(quat_xyzw)
        rot_mat = r.as_matrix()
        for i, c in enumerate(['r', 'g', 'b']):
            vec = rot_mat[:, i]
            ax.plot(
                [origin[0], origin[0] + axis_len * vec[0]],
                [origin[1], origin[1] + axis_len * vec[1]],
                [origin[2], origin[2] + axis_len * vec[2]],
                color=c,
                linewidth=2,
            )

    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Pico bones (positions and orientations), frame 0")

    # plot points and axes
    for bone, (pos, quat) in frame_0.items():
        ax.scatter(pos[0], pos[1], pos[2], marker='o', s=30, c='k')
        ax.text(pos[0], pos[1], pos[2], bone, fontsize=8)
        draw_axes(ax, pos, quat, axis_len=0.06)

    # plot skeleton lines
    for bone, parent in URDF_BONE_HIERARCHY.items():
        if parent is not None and bone in frame_0 and parent in frame_0:
            p_bone = frame_0[bone][0]
            p_parent = frame_0[parent][0]
            ax.plot([p_bone[0], p_parent[0]],
                    [p_bone[1], p_parent[1]],
                    [p_bone[2], p_parent[2]], color='blue', linewidth=1)

    # Make equal aspect ratio
    def set_axes_equal(ax):
        xs, ys, zs = [], [], []
        for pos, quat in frame_0.values():
            xs.append(pos[0])
            ys.append(pos[1])
            zs.append(pos[2])
        max_range = np.ptp([xs, ys, zs])
        mean_x = np.mean(xs)
        mean_y = np.mean(ys)
        mean_z = np.mean(zs)
        ax.set_xlim(mean_x - max_range/2, mean_x + max_range/2)
        ax.set_ylim(mean_y - max_range/2, mean_y + max_range/2)
        ax.set_zlim(mean_z - max_range/2, mean_z + max_range/2)

    set_axes_equal(ax)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.tight_layout()
    plt.show()
