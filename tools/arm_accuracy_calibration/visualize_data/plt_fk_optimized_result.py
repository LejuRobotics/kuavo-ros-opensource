#!/usr/bin/env python3
import argparse
import os
import csv
import yaml
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np
from typing import List, Optional, Tuple
import rospy
from kuavo_msgs.srv import fkSrvWithReferFrame, fkSrvWithReferFrameRequest

DEFAULT_CUSTOM_URDF = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "config", "biped_v3_arm_custom.urdf"
)

# URDF frame → mocap rigid body name（raw_poses.csv 中的前缀）
# 注意：映射关系依赖 config.yaml 中的 mapping_version。
# 这里预先定义两套，具体在 compute_mocap_point_in_base 中根据 mapping_version 选择。

# mapping_version = 1:
#   joint_1 -> zarm_l1_link
#   joint_2 -> zarm_l2_link
#   ...
#   joint_6 -> zarm_l6_link
#   joint_7 -> zarm_l7_link
FRAME_TO_BODY_V1 = {
    "zarm_l1_ref_link": "joint_1",
    "zarm_l1_link": "joint_1",
    "zarm_l2_link": "joint_2",
    "zarm_l3_link": "joint_3",
    "zarm_l4_link": "joint_4",
    "zarm_l5_link": "joint_5",
    "zarm_l6_link": "joint_6",
    "zarm_l7_link": "joint_7",
}

# mapping_version = 2:
#   joint_1 -> waist_yaw_link
#   joint_2 -> zarm_l1_link
#   joint_3 -> zarm_l2_link
#   ...
#   joint_7 -> zarm_l6_link
FRAME_TO_BODY_V2 = {
    "zarm_l1_ref_link": "joint_2",
    "zarm_l1_link": "joint_2",
    "zarm_l2_link": "joint_3",
    "zarm_l3_link": "joint_4",
    "zarm_l4_link": "joint_5",
    "zarm_l5_link": "joint_6",
    "zarm_l6_link": "joint_7",
}

def draw_coordinate_frame(ax, origin, scale=0.05, label=""):
    """Draw a coordinate frame at the given origin with unit matrix orientation."""
    # Unit vectors for x, y, z axes (identity matrix)
    x_axis = np.array([1, 0, 0]) * scale
    y_axis = np.array([0, 1, 0]) * scale
    z_axis = np.array([0, 0, 1]) * scale

    # Draw x-axis (red)
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        x_axis[0],
        x_axis[1],
        x_axis[2],
        color="red",
        arrow_length_ratio=0.3,
        linewidth=2.0,
    )
    # Draw y-axis (green)
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        y_axis[0],
        y_axis[1],
        y_axis[2],
        color="green",
        arrow_length_ratio=0.3,
        linewidth=2.0,
    )
    # Draw z-axis (blue)
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        z_axis[0],
        z_axis[1],
        z_axis[2],
        color="blue",
        arrow_length_ratio=0.3,
        linewidth=2.0,
    )

    if label:
        ax.text(origin[0], origin[1], origin[2], label, fontsize=8)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize mocap and FK comparison, optionally save figure to PNG."
    )
    parser.add_argument(
        "--save-png",
        nargs="?",
        const="__PROMPT__",
        default=None,
        help="Save plot to png. Example: --save-png result.png ; use --save-png for interactive input.",
    )
    _default_data_dir = os.path.join(
        os.path.dirname(__file__), "..", "calibration_output", "raw_data_motion", "joint_0"
    )
    parser.add_argument(
        "--poses-csv",
        type=str,
        default=os.path.join(_default_data_dir, "raw_poses.csv"),
        help="Path to raw_poses.csv (mocap rigid body poses).",
    )
    parser.add_argument(
        "--sensor-csv",
        type=str,
        default=os.path.join(_default_data_dir, "raw_sensor.csv"),
        help="Path to raw_sensor.csv (left arm joint angles).",
    )
    parser.add_argument(
        "--base-frame",
        type=str,
        default="zarm_l1_ref_link",
        help="Base reference frame for FK. Default: zarm_l1_ref_link.",
    )
    parser.add_argument(
        "--end-frame",
        type=str,
        default="zarm_l5_link",
        help="End-effector frame for FK. Default: zarm_l6_link.",
    )
    parser.add_argument(
        "--hand-side",
        type=int,
        default=0,
        choices=[0, 1],
        help="Hand side: 0=left, 1=right. Default: 0.",
    )
    parser.add_argument(
        "--urdf-path",
        type=str,
        default=DEFAULT_CUSTOM_URDF,
        help=f"Custom URDF path used for fk_after. Default: {DEFAULT_CUSTOM_URDF}",
    )
    return parser.parse_args()


def read_average_sensor_q(csv_path: str) -> List[float]:
    """Read raw_sensor.csv and return the mean of 7 left-arm joint angles (radians)."""
    import csv
    if not os.path.isfile(csv_path):
        data_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "calibration_output", "raw_data_motion", "joint_0",
        )
        raise FileNotFoundError(
            f"找不到 sensor CSV 文件: {csv_path}\n"
            f"请将 raw_sensor.csv 放入目录: {os.path.normpath(data_dir)}"
        )
    cols = ["left_arm_1", "left_arm_2", "left_arm_3", "left_arm_4",
            "left_arm_5", "left_arm_6", "left_arm_7"]
    sums = [0.0] * 7
    count = 0
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                vals = [float(row[c]) for c in cols]
            except (KeyError, ValueError):
                continue
            for i, v in enumerate(vals):
                sums[i] += v
            count += 1
    if count == 0:
        raise RuntimeError(f"No valid rows found in {csv_path}")
    return [s / count for s in sums]


def call_fk_service(
    q: List[float],
    base_frame: str,
    end_frame: str,
    hand_side: int,
    urdf_path: str = "",
) -> List[float]:
    """Call /ik/fk_srv_with_refer_frame and return position [x, y, z] in meters."""


    rospy.wait_for_service("/ik/fk_srv_with_refer_frame", timeout=5.0)
    fk_client = rospy.ServiceProxy("/ik/fk_srv_with_refer_frame", fkSrvWithReferFrame)

    req = fkSrvWithReferFrameRequest()
    req.q = [float(v) for v in q]
    req.base_frame = base_frame
    req.end_effector_frame = end_frame
    req.hand_side = hand_side
    req.urdf_path = urdf_path

    res = fk_client(req)
    if not res.success:
        raise RuntimeError(f"FK service call failed: {res.error_message}")
    pose = res.hand_poses.left_pose if hand_side == 0 else res.hand_poses.right_pose
    return list(pose.pos_xyz)


def resolve_png_path(save_png_arg: Optional[str]) -> Optional[str]:
    if save_png_arg is None:
        return None

    if save_png_arg == "__PROMPT__":
        save_png_arg = input("请输入要保存的 PNG 文件名（例如 result.png）: ").strip()

    if not save_png_arg:
        save_png_arg = "fk_mocap_comparison.png"

    if not save_png_arg.lower().endswith(".png"):
        save_png_arg += ".png"

    return save_png_arg


def _build_T_from_row(row: dict, body_name: str) -> Optional[np.ndarray]:
    """
    从 raw_poses.csv 的一行中构建刚体在动捕系下的 4x4 齐次矩阵 T_body_in_mocap。
    返回 None 表示该刚体在这一行数据无效。
    """
    prefix = f"{body_name}_"
    keys = [prefix + k for k in ("px", "py", "pz", "qx", "qy", "qz", "qw")]
    try:
        vals = [row[k] for k in keys]
    except KeyError:
        return None

    try:
        px, py, pz = (float(vals[0]), float(vals[1]), float(vals[2]))
        qx, qy, qz, qw = (float(vals[3]), float(vals[4]), float(vals[5]), float(vals[6]))
    except (TypeError, ValueError):
        return None

    if any(abs(v) > 9999.0 for v in (px, py, pz)):
        return None

    # 位置单位: mm；四元数: (qx, qy, qz, qw)
    from scipy.spatial.transform import Rotation as R  # lazy import

    T = np.eye(4, dtype=float)
    T[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
    T[:3, 3] = np.array([px, py, pz], dtype=float)
    return T


def _load_array_transforms() -> Tuple[dict, int]:
    """
    从 config.yaml 加载刚体到 joint 的标定变换 T_rigid_body_to_joint (4x4, 单位 mm)。
    返回: { "joint_1": np.ndarray(4,4), ... }
    """
    config_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config",
        "config.yaml",
    )
    if not os.path.isfile(config_path):
        raise FileNotFoundError(f"找不到标定配置文件: {config_path}")

    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    mapping_version = int(data.get("mapping_version", 1))
    arrays = data.get("arrays", {})
    out = {}
    for name, entry in arrays.items():
        T_list = entry.get("T_rigid_body_to_joint")
        if T_list is None:
            continue
        T = np.array(T_list, dtype=float)
        if T.shape == (4, 4):
            out[name] = T
    return out, mapping_version


def _compute_T_joint_to_mocap(T_rigid_to_mocap: np.ndarray, T_rigid_body_to_joint: np.ndarray) -> np.ndarray:
    """
    与 frame_solver / test_link 逻辑一致:
        T_mocap_to_joint = T_rigid_body_to_joint @ inv(T_rigid_to_mocap)
        T_joint_to_mocap = inv(T_mocap_to_joint)
    """
    T_mocap_to_joint = T_rigid_body_to_joint @ np.linalg.inv(T_rigid_to_mocap)
    return np.linalg.inv(T_mocap_to_joint)


def compute_mocap_point_in_base(
    poses_csv: str,
    base_frame: str,
    end_frame: str,
) -> np.ndarray:
    """
    从 raw_poses.csv 中读取刚体位姿，取与 --base-frame / --end-frame 对应的两个刚体，
    在动捕世界系下计算 T_end_in_base，并对所有有效帧做平均。

    返回:
        mocap_pos_base: np.ndarray, shape (3,), 单位 m，表达为 base_frame (例如 zarm_l1_ref_link) 坐标系下的末端位置。
    """
    # 根据 config.yaml 中的 mapping_version 选择正确的 frame→body 映射
    array_T, mapping_version = _load_array_transforms()
    if mapping_version == 1:
        frame_to_body = FRAME_TO_BODY_V1
    elif mapping_version == 2:
        frame_to_body = FRAME_TO_BODY_V2
    else:
        raise ValueError(f"未知 mapping_version={mapping_version}，当前仅支持 1 或 2。")

    if base_frame not in frame_to_body:
        raise ValueError(
            f"当前脚本在 mapping_version={mapping_version} 下仅支持以下 base-frame: "
            f"{list(frame_to_body.keys())}，收到: {base_frame}"
        )
    if end_frame not in frame_to_body:
        raise ValueError(
            f"当前脚本在 mapping_version={mapping_version} 下仅支持以下 end-frame (左臂链路): "
            f"{list(frame_to_body.keys())}，收到: {end_frame}"
        )

    base_body = frame_to_body[base_frame]
    end_body = frame_to_body[end_frame]

    # 使用 config.yaml 中的 T_rigid_body_to_joint，把刚体坐标系转换到对应的 joint/link 坐标系
    if base_body not in array_T or end_body not in array_T:
        raise KeyError(
            f"config.yaml 中缺少 {base_body} 或 {end_body} 的 T_rigid_body_to_joint，"
            f"当前可用键: {list(array_T.keys())}"
        )
    T_rigid_base_to_joint = array_T[base_body]
    T_rigid_end_to_joint = array_T[end_body]

    if not os.path.isfile(poses_csv):
        raise FileNotFoundError(f"找不到 mocap CSV 文件: {poses_csv}")

    sum_pos = np.zeros(3, dtype=float)
    count = 0

    with open(poses_csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            T_rigid_base_in_mocap = _build_T_from_row(row, base_body)
            T_rigid_end_in_mocap = _build_T_from_row(row, end_body)
            if T_rigid_base_in_mocap is None or T_rigid_end_in_mocap is None:
                continue

            # 刚体 -> joint/link: 使用 config.yaml 提供的 T_rigid_body_to_joint
            T_joint_base_in_mocap = _compute_T_joint_to_mocap(
                T_rigid_base_in_mocap, T_rigid_base_to_joint
            )
            T_joint_end_in_mocap = _compute_T_joint_to_mocap(
                T_rigid_end_in_mocap, T_rigid_end_to_joint
            )

            # T_end_in_base = inv(T_base_in_mocap_joint) @ T_end_in_mocap_joint
            T_end_in_base = np.linalg.inv(T_joint_base_in_mocap) @ T_joint_end_in_mocap
            pos_mm = T_end_in_base[:3, 3]

            if np.any(~np.isfinite(pos_mm)):
                continue

            sum_pos += pos_mm
            count += 1

    if count == 0:
        raise RuntimeError(
            f"在 {poses_csv} 中未找到有效的 mocap 数据，请检查 raw_poses.csv 内容。"
        )

    avg_pos_mm = sum_pos / float(count)
    # 转成米
    return avg_pos_mm / 1000.0


def main() -> None:
    args = parse_args()
    png_path = resolve_png_path(args.save_png)


    rospy.init_node("plt_T21", anonymous=True)

    # fk_before: actual sensor angles (left arm avg) + right arm zeros, default URDF
    left_q_before = read_average_sensor_q(args.sensor_csv)
    q_before = left_q_before + [0.0] * 7   # pad right arm with zeros → 14 total
    print(f"Average sensor q (left arm): {[f'{v:.6f}' for v in left_q_before]}")

    # fk_after: all-zero joint angles, custom calibrated URDF
    q_after = [0.0] * 14

    print(f"Calling FK service (before): base={args.base_frame}, end={args.end_frame}, "
          f"hand_side={args.hand_side}, urdf_path='' (default)")
    fk_before = call_fk_service(q_before, args.base_frame, args.end_frame, args.hand_side, "")
    print(f"  fk_before = {fk_before}")

    print(f"Calling FK service (after):  base={args.base_frame}, end={args.end_frame}, "
          f"hand_side={args.hand_side}, urdf_path={args.urdf_path!r}")
    fk_after = call_fk_service(q_after, args.base_frame, args.end_frame, args.hand_side, args.urdf_path)
    print(f"  fk_after  = {fk_after}")

    # mocap: 从 raw_poses.csv 中读取与 --base-frame / --end-frame 对应的刚体，
    # 计算末端在 base_frame (默认 zarm_l1_ref_link) 下的平均位置，作为动捕参考点。
    mocap = compute_mocap_point_in_base(args.poses_csv, args.base_frame, args.end_frame).tolist()
    print(f"  mocap (in {args.base_frame}) = {mocap}")

    # 为保持之前的显示逻辑：在 3D 图中仍然使用 base_link 作为原点，
    # zarm_l1_ref_link 作为 "Shoulder" 点，三者都整体平移 shoulder_pos。
    # base_link → zarm_l1_ref_link 的平移 (m)
    shoulder_pos = [-0.003, 0.1737, 0.3945]
    fk_before = (np.array(fk_before) + np.array(shoulder_pos)).tolist()
    fk_after = (np.array(fk_after) + np.array(shoulder_pos)).tolist()
    mocap = (np.array(mocap) + np.array(shoulder_pos)).tolist()
    print(f"  mocap (for plot) = {mocap}")

    fig = plt.figure(figsize=(20, 6))
    # 3D subplot 1
    ax = fig.add_subplot(131, projection="3d")

    # 1. Draw coordinate frame at origin: base_link
    origin = [0.0, 0.0, 0.0]
    draw_coordinate_frame(ax, origin, scale=0.05, label="Base Link")

    # 2. Draw coordinate frame at shoulder position: zarm_l1_ref_link
    draw_coordinate_frame(ax, shoulder_pos, scale=0.05, label="Shoulder")

    # Plot points
    ax.scatter(*mocap, color="black", s=60, label=f"{args.end_frame} mocap")
    ax.scatter(*fk_before, color="tab:red", s=50, label="FK before")
    ax.scatter(*fk_after, color="tab:green", s=50, label="FK after")

    # 3. Arrow 1: mocap -> FK before
    ax.quiver(
        mocap[0],
        mocap[1],
        mocap[2],
        fk_before[0] - mocap[0],
        fk_before[1] - mocap[1],
        fk_before[2] - mocap[2],
        color="tab:red",
        linewidth=2.0,
        arrow_length_ratio=0.2,
        label="Mocap -> FK before",
    )

    # 4. Arrow 2: mocap -> FK after
    ax.quiver(
        mocap[0],
        mocap[1],
        mocap[2],
        fk_after[0] - mocap[0],
        fk_after[1] - mocap[1],
        fk_after[2] - mocap[2],
        color="tab:green",
        linewidth=2.0,
        arrow_length_ratio=0.2,
        label="Mocap -> FK after",
    )

    # 5. Dashed lines
    # Dashed line 1: base -> shoulder
    ax.plot(
        [origin[0], shoulder_pos[0]],
        [origin[1], shoulder_pos[1]],
        [origin[2], shoulder_pos[2]],
        "k--",
        linewidth=1.5,
        alpha=0.6,
        label="Base -> Shoulder",
    )

    # Dashed line 2: shoulder -> FK before
    ax.plot(
        [shoulder_pos[0], fk_before[0]],
        [shoulder_pos[1], fk_before[1]],
        [shoulder_pos[2], fk_before[2]],
        "r--",
        linewidth=1.5,
        alpha=0.6,
        label="Shoulder -> FK before",
    )

    # Dashed line 3: shoulder -> FK after
    ax.plot(
        [shoulder_pos[0], fk_after[0]],
        [shoulder_pos[1], fk_after[1]],
        [shoulder_pos[2], fk_after[2]],
        "g--",
        linewidth=1.5,
        alpha=0.6,
        label="Shoulder -> FK after",
    )

    # Dashed line 4: shoulder -> mocap
    ax.plot(
        [shoulder_pos[0], mocap[0]],
        [shoulder_pos[1], mocap[1]],
        [shoulder_pos[2], mocap[2]],
        "b--",
        linewidth=1.5,
        alpha=0.6,
        label="Shoulder -> Mocap",
    )

    ax.set_title(f"{args.end_frame}: Mocap to FK arrows with coordinate frames")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend(loc="best")

    # Keep axis scales visually balanced around all points
    all_points = [origin, shoulder_pos, mocap, fk_before, fk_after]
    xs = [p[0] for p in all_points]
    ys = [p[1] for p in all_points]
    zs = [p[2] for p in all_points]
    x_mid = (max(xs) + min(xs)) / 2.0
    y_mid = (max(ys) + min(ys)) / 2.0
    z_mid = (max(zs) + min(zs)) / 2.0
    radius = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)) / 2.0
    radius = max(radius, 0.05)  # Ensure minimum radius for visibility

    ax.set_xlim(x_mid - radius, x_mid + radius)
    ax.set_ylim(y_mid - radius, y_mid + radius)
    ax.set_zlim(z_mid - radius, z_mid + radius)

    # Calculate errors
    error_before = np.array(fk_before) - np.array(mocap)
    error_after = np.array(fk_after) - np.array(mocap)
    magnitude_error_before = np.linalg.norm(error_before)
    magnitude_error_after = np.linalg.norm(error_after)

    # Second subplot: arrows from mocap to before and after
    ax2 = fig.add_subplot(132, projection="3d")

    # Plot mocap point
    ax2.scatter(*mocap, color="black", s=100, label="Mocap", zorder=5)

    # Arrow 1: mocap -> FK before
    ax2.quiver(
        mocap[0],
        mocap[1],
        mocap[2],
        fk_before[0] - mocap[0],
        fk_before[1] - mocap[1],
        fk_before[2] - mocap[2],
        color="tab:red",
        linewidth=3.0,
        arrow_length_ratio=0.25,
        label="Mocap -> Before",
    )

    # Arrow 2: mocap -> FK after
    ax2.quiver(
        mocap[0],
        mocap[1],
        mocap[2],
        fk_after[0] - mocap[0],
        fk_after[1] - mocap[1],
        fk_after[2] - mocap[2],
        color="tab:green",
        linewidth=3.0,
        arrow_length_ratio=0.25,
        label="Mocap -> After",
    )

    # Plot target points
    ax2.scatter(*fk_before, color="tab:red", s=80, label="FK Before", marker="s")
    ax2.scatter(*fk_after, color="tab:green", s=80, label="FK After", marker="s")

    ax2.set_title("Error Vectors: Mocap to FK", fontsize=14, fontweight="bold")
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.set_zlabel("Z (m)")
    ax2.legend(loc="best")

    # Set axis limits to show arrows clearly with better padding
    arrow_points = [mocap, fk_before, fk_after]
    xs = [p[0] for p in arrow_points]
    ys = [p[1] for p in arrow_points]
    zs = [p[2] for p in arrow_points]
    x_mid = (max(xs) + min(xs)) / 2.0
    y_mid = (max(ys) + min(ys)) / 2.0
    z_mid = (max(zs) + min(zs)) / 2.0
    radius = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)) / 2.0
    radius = max(radius, 0.015) * 1.3  # Add 30% padding for clearer view

    ax2.set_xlim(x_mid - radius, x_mid + radius)
    ax2.set_ylim(y_mid - radius, y_mid + radius)
    ax2.set_zlim(z_mid - radius, z_mid + radius)
    
    # Note: 3D subplots don't support set_box_aspect, but we've set equal ranges for better visualization

    # Third subplot: Histogram for error comparison
    ax3 = fig.add_subplot(133)

    # Prepare data for grouped bar chart
    categories = [
        "before.x - mocap.x\nvs\nafter.x - mocap.x",
        "before.y - mocap.y\nvs\nafter.y - mocap.y",
        "before.z - mocap.z\nvs\nafter.z - mocap.z",
        "(before - mocap).norm2\nvs\n(after - mocap).norm2",
    ]

    # Convert errors to mm
    errors_before = [
        error_before[0] * 1000,  # X error
        error_before[1] * 1000,  # Y error
        error_before[2] * 1000,  # Z error
        magnitude_error_before * 1000,  # Norm2 error
    ]
    errors_after = [
        error_after[0] * 1000,  # X error
        error_after[1] * 1000,  # Y error
        error_after[2] * 1000,  # Z error
        magnitude_error_after * 1000,  # Norm2 error
    ]

    x = np.arange(len(categories))
    width = 0.35

    # Create grouped bar plot
    bars1 = ax3.bar(
        x - width / 2,
        errors_before,
        width,
        label="Before",
        color="tab:red",
        alpha=0.7,
        edgecolor="black",
        linewidth=1.5,
    )
    bars2 = ax3.bar(
        x + width / 2,
        errors_after,
        width,
        label="After",
        color="tab:green",
        alpha=0.7,
        edgecolor="black",
        linewidth=1.5,
    )

    # Add value labels on bars
    all_errors = errors_before + errors_after
    max_error = max([abs(e) for e in all_errors])
    for bars, errors in [(bars1, errors_before), (bars2, errors_after)]:
        for bar, error in zip(bars, errors):
            height = bar.get_height()
            ax3.text(
                bar.get_x() + bar.get_width() / 2.0,
                height + (0.02 * max_error if height >= 0 else -0.02 * max_error),
                f"{error:.3f}",
                ha="center",
                va="bottom" if height >= 0 else "top",
                fontsize=9,
                fontweight="bold",
            )

    ax3.axhline(y=0, color="black", linestyle="-", linewidth=0.5)
    ax3.set_ylabel("Error (mm)", fontsize=12)
    ax3.set_title("FK Error Comparison: Before vs After", fontsize=14, fontweight="bold")
    ax3.set_xticks(x)
    ax3.set_xticklabels(categories, fontsize=10)
    ax3.legend(loc="best")
    ax3.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    if png_path is not None:
        plt.savefig(png_path, dpi=300, bbox_inches="tight")
        print(f"PNG 已保存: {png_path}")
    plt.show()


if __name__ == "__main__":
    main()