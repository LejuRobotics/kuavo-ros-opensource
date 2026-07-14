#!/usr/bin/env python3
import csv
import os
import re
import argparse
from typing import List, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

try:
    import yaml
except ImportError:
    yaml = None

# ============================================================================
# 配置区域：在此处设置标定结果 CSV 文件的绝对路径
# ============================================================================
# 标定结果的基础目录路径（fitted_results 目录的绝对路径）
# 如果设置为空字符串 ""，将使用默认路径
CALIBRATION_BASE_DIR = "/root/kuavo_ws/tools/arm_accuracy_calibration/calibration_output/fitted_results"
# ============================================================================

REQUIRED_ORIGIN_JOINTS = [
    "zarm_l2_joint",
    "zarm_l3_joint",
    "zarm_l4_joint",
    "zarm_l5_joint",
    "zarm_l6_joint",
]
REQUIRED_AXIS_JOINTS = [
    "zarm_l1_joint",
    "zarm_l2_joint",
    "zarm_l3_joint",
    "zarm_l4_joint",
    "zarm_l5_joint",
]


def load_matrix(csv_path: str) -> np.ndarray:
    """从 CSV 文件加载 4x4 齐次变换矩阵（首行为表头）。"""
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"文件不存在: {csv_path}")

    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)
    flat = data.ravel()
    if flat.size != 16:
        raise ValueError(f"文件 {csv_path} 中的数据长度为 {flat.size}，无法组成 4x4 矩阵")
    return flat.reshape(4, 4)


def resolve_joint0_dir(base_dir: str) -> str:
    """兼容不同导出版本的目录命名：joint_-1_1 或 joint_0。"""
    candidates = ["joint_-1_1", "joint_0"]
    for dirname in candidates:
        path = os.path.join(base_dir, dirname)
        if os.path.isdir(path):
            return path
    raise FileNotFoundError(
        f"未找到 joint_0 数据目录，期望其一: "
        f"{', '.join(os.path.join(base_dir, d) for d in candidates)}"
    )


def rotmat_to_euler_xyz(R: np.ndarray) -> np.ndarray:
    """从 3x3 旋转矩阵提取 scipy xyz 欧拉角 (roll, pitch, yaw)，返回弧度。"""
    if R.shape != (3, 3):
        raise ValueError(f"旋转矩阵尺寸错误: {R.shape}，应为 3x3")
    return Rotation.from_matrix(R).as_euler("xyz", degrees=False)


def print_matrix(name: str, mat: np.ndarray) -> None:
    print(f"{name}:")
    for row in mat:
        print("  " + "  ".join(f"{v:10.6f}" for v in row))
    print()


def print_euler_xyz_from_T(name: str, T: np.ndarray) -> None:
    euler_deg = np.degrees(rotmat_to_euler_xyz(T[:3, :3]))
    print(
        f"Euler_XYZ_{name} (deg): "
        f"roll={euler_deg[0]:.6f}, pitch={euler_deg[1]:.6f}, yaw={euler_deg[2]:.6f}"
    )
    print()


def print_summary_table(T_dict: dict) -> None:
    name_col = "T name"
    trans_col = "translation (x, y, z)"
    rpy_col = "RPY (deg, XYZ)"
    w_name = 24
    w_trans = 38
    w_rpy = 38
    header = f"{name_col:<{w_name}} | {trans_col:<{w_trans}} | {rpy_col:<{w_rpy}}"
    sep = "-" * len(header)
    print(sep)
    print(header)
    print(sep)

    for name, T in T_dict.items():
        t = T[:3, 3]
        euler_xyz = np.degrees(rotmat_to_euler_xyz(T[:3, :3]))
        roll, pitch, yaw = euler_xyz[0], euler_xyz[1], euler_xyz[2]
        trans_str = f"[{t[0]: .3f}, {t[1]: .3f}, {t[2]: .3f}]"
        rpy_str = f"[R={roll: .3f}, P={pitch: .3f}, Y={yaw: .3f}]"
        print(f"{name:<{w_name}} | {trans_str:<{w_trans}} | {rpy_str:<{w_rpy}}")

    print(sep)
    print()


def print_urdf_origins_from_Ts(joint_T_map: dict) -> None:
    print("URDF origin updates (xyz in m, rpy in rad):")
    for joint_name, T in joint_T_map.items():
        # joint_0 的平移标定数据单位是 mm，URDF 需要 m。
        t = T[:3, 3] / 1000.0
        euler_xyz = rotmat_to_euler_xyz(T[:3, :3])
        roll, pitch, yaw = euler_xyz[0], euler_xyz[1], euler_xyz[2]
        print(
            f'{joint_name}: <origin xyz="{t[0]:.6f} {t[1]:.6f} {t[2]:.6f}" '
            f'rpy="{roll:.18f} {pitch:.18f} {yaw:.18f}" />'
        )
    print()


def load_first_row(csv_path: str) -> dict:
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"文件不存在: {csv_path}")
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        row = next(reader, None)
        if row is None:
            raise ValueError(f"CSV 中没有数据行: {csv_path}")
        return row


def parse_axis_from_row(row: dict, csv_path: str) -> tuple:
    try:
        axis_x = float(row["axis_x"])
        axis_y = float(row["axis_y"])
        axis_z = float(row["axis_z"])
    except KeyError as exc:
        raise KeyError(f"CSV 缺少列: {exc} ({csv_path})") from exc
    except ValueError as exc:
        raise ValueError(f"CSV axis 数值解析失败: {csv_path}") from exc

    norm = (axis_x * axis_x + axis_y * axis_y + axis_z * axis_z) ** 0.5
    if norm <= 1e-12:
        raise ValueError(f"CSV axis 范数过小，无法归一化: {csv_path}")

    ax, ay, az = axis_x / norm, axis_y / norm, axis_z / norm
    dominant = max((abs(ax), "x"), (abs(ay), "y"), (abs(az), "z"))[1]
    should_flip = (
        (dominant == "x" and ax < 0.0)
        or (dominant == "y" and ay < 0.0)
        or (dominant == "z" and az < 0.0)
    )
    if should_flip:
        ax, ay, az = -ax, -ay, -az
    return ax, ay, az


def print_urdf_axis_updates(joint_axis_map: dict) -> None:
    print("URDF axis updates:")
    for joint_name, axis in joint_axis_map.items():
        ax, ay, az = axis
        print(f'{joint_name}: <axis xyz="{ax:.18f} {ay:.18f} {az:.18f}" />')
    print()


def format_origin_line_from_T(T: np.ndarray) -> str:
    t = T[:3, 3] / 1000.0
    euler_xyz = rotmat_to_euler_xyz(T[:3, :3])
    roll, pitch, yaw = euler_xyz[0], euler_xyz[1], euler_xyz[2]
    return (
        f'<origin xyz="{t[0]:.6f} {t[1]:.6f} {t[2]:.6f}" '
        f'rpy="{roll:.18f} {pitch:.18f} {yaw:.18f}" />'
    )


def format_axis_line(axis: tuple) -> str:
    ax, ay, az = axis
    return f'<axis xyz="{ax:.18f} {ay:.18f} {az:.18f}" />'


def replace_joint_tag_line(joint_block: str, tag_name: str, new_tag_line: str, joint_name: str) -> str:
    tag_pattern = rf"^(\s*)<{tag_name}\b[^>]*/>\s*$"
    match = re.search(tag_pattern, joint_block, flags=re.M)
    if match is None:
        raise ValueError(f"joint {joint_name} 中未找到 <{tag_name} ... /> 标签")
    indent = match.group(1)
    return re.sub(tag_pattern, f"{indent}{new_tag_line}", joint_block, count=1, flags=re.M)


def replace_joint_tag_in_urdf(
    urdf_text: str,
    joint_name: str,
    tag_name: str,
    new_tag_line: str,
) -> str:
    joint_pattern = rf'(<joint name="{re.escape(joint_name)}"[^>]*>)(.*?)(</joint>)'
    joint_match = re.search(joint_pattern, urdf_text, flags=re.S)
    if joint_match is None:
        raise ValueError(f"URDF 中未找到 joint: {joint_name}")

    joint_header = joint_match.group(1)
    joint_body = joint_match.group(2)
    joint_footer = joint_match.group(3)

    joint_body = replace_joint_tag_line(joint_body, tag_name, new_tag_line, joint_name)

    new_joint_block = f"{joint_header}{joint_body}{joint_footer}"
    return urdf_text[:joint_match.start()] + new_joint_block + urdf_text[joint_match.end():]


def update_urdf_file(
    urdf_path: str,
    joint_T_map: dict,
    joint_axis_map: dict,
    required_origin_joints: Tuple[str, ...] = tuple(REQUIRED_ORIGIN_JOINTS),
    required_axis_joints: Tuple[str, ...] = tuple(REQUIRED_AXIS_JOINTS),
) -> None:
    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(f"URDF 文件不存在: {urdf_path}")

    with open(urdf_path, "r", encoding="utf-8") as f:
        urdf_text = f.read()

    for joint_name in required_origin_joints:
        if joint_name not in joint_T_map:
            raise KeyError(f"缺少 joint 的 origin 数据: {joint_name}")
    for joint_name in required_axis_joints:
        if joint_name not in joint_axis_map:
            raise KeyError(f"缺少 joint 的 axis 数据: {joint_name}")

    for joint_name in required_origin_joints:
        origin_line = format_origin_line_from_T(joint_T_map[joint_name])
        urdf_text = replace_joint_tag_in_urdf(urdf_text, joint_name, "origin", origin_line)

    for joint_name in required_axis_joints:
        axis_line = format_axis_line(joint_axis_map[joint_name])
        urdf_text = replace_joint_tag_in_urdf(urdf_text, joint_name, "axis", axis_line)

    with open(urdf_path, "w", encoding="utf-8") as f:
        f.write(urdf_text)


def validate_directory_structure(base_dir: str) -> Tuple[bool, List[str]]:
    """
    验证 fitted_results 目录结构是否符合预设规则。
    
    返回: (is_valid, error_messages)
    """
    errors = []
    
    if not os.path.isdir(base_dir):
        return False, [f"基础目录不存在: {base_dir}"]
    
    # 1. 检查 joint_0 目录（兼容 joint_-1_1 或 joint_0）
    joint0_dir = None
    try:
        joint0_dir = resolve_joint0_dir(base_dir)
    except FileNotFoundError as e:
        errors.append(str(e))
        return False, errors
    
    # 2. 检查 joint_0 目录下的变换矩阵文件
    path_T21 = os.path.join(joint0_dir, "T_joint2_in_joint1_fitted.csv")
    if not os.path.isfile(path_T21):
        errors.append(f"缺少必需文件: {path_T21}")
    
    # 检查新格式或旧格式文件
    path_T32 = os.path.join(joint0_dir, "T_joint3_in_joint2_fitted.csv")
    path_T43 = os.path.join(joint0_dir, "T_joint4_in_joint3_fitted.csv")
    path_T31 = os.path.join(joint0_dir, "T_joint3_in_joint1_fitted.csv")
    path_T41 = os.path.join(joint0_dir, "T_joint4_in_joint1_fitted.csv")
    
    has_new_format = os.path.isfile(path_T32) and os.path.isfile(path_T43)
    has_old_format = os.path.isfile(path_T31) and os.path.isfile(path_T41)
    
    if not has_new_format and not has_old_format:
        errors.append(
            f"joint_0 目录缺少变换矩阵文件。需要以下之一：\n"
            f"  新格式: {path_T32} 和 {path_T43}\n"
            f"  旧格式: {path_T31} 和 {path_T41}"
        )

    # 新格式下，joint_5/joint_6 的标定结果为必需。
    path_T54 = os.path.join(joint0_dir, "T_joint5_in_joint4_fitted.csv")
    path_T65 = os.path.join(joint0_dir, "T_joint6_in_joint5_fitted.csv")
    if not os.path.isfile(path_T54):
        errors.append(f"缺少必需文件: {path_T54}")
    if not os.path.isfile(path_T65):
        errors.append(f"缺少必需文件: {path_T65}")
    
    # 3. 检查轴向结果目录：joint_1~joint_5 的 circle_fit_result.csv 均为必需。
    required_joints = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
    for joint_dir_name in required_joints:
        joint_dir = os.path.join(base_dir, joint_dir_name)
        if not os.path.isdir(joint_dir):
            errors.append(f"缺少目录: {joint_dir}")
            continue
        
        csv_path = os.path.join(joint_dir, "circle_fit_result.csv")
        if not os.path.isfile(csv_path):
            errors.append(f"缺少文件: {csv_path}")
            continue
        
        # 验证 CSV 文件包含必需的列
        try:
            row = load_first_row(csv_path)
            required_cols = ["axis_x", "axis_y", "axis_z"]
            missing_cols = [col for col in required_cols if col not in row]
            if missing_cols:
                errors.append(
                    f"{csv_path} 缺少必需的列: {', '.join(missing_cols)}"
                )
        except Exception as e:
            errors.append(f"无法读取或验证 {csv_path}: {e}")

    is_valid = len(errors) == 0
    return is_valid, errors


def print_validation_result(is_valid: bool, errors: List[str]) -> None:
    """打印验证结果。"""
    if is_valid:
        print("=" * 60)
        print("✓ 目录结构验证通过：符合预设规则")
        print("=" * 60)
        print()
    else:
        print("=" * 60)
        print("✗ 目录结构验证失败：不符合预设规则")
        print("=" * 60)
        for i, error in enumerate(errors, 1):
            print(f"{i}. {error}")
        print()
        print("请参考 README.md 了解正确的目录结构要求。")
        print("=" * 60)
        print()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="读取标定 CSV 并打印 T/URDF origin/URDF axis；可选更新 URDF 文件。"
    )
    parser.add_argument(
        "--data-dir",
        dest="data_dir",
        default="",
        help=(
            "标定结果基础目录（fitted_results 目录绝对路径）。"
            "不传则使用脚本内 CALIBRATION_BASE_DIR 或默认路径。"
        ),
    )
    parser.add_argument(
        "--update-urdf",
        dest="update_urdf_path",
        default="",
        help=(
            "若提供该参数，则将左臂关节标定结果写回 URDF。"
            "会更新 zarm_l2/l3/l4/l5/l6 的 origin 与 zarm_l1/l2/l3/l4/l5 的 axis。"
        ),
    )
    parser.add_argument(
        "--validate",
        action="store_true",
        help="仅验证目录结构是否符合预设规则，不执行其他操作。",
    )
    parser.add_argument(
        "--auto-update",
        action="store_true",
        help="自动验证目录结构，验证通过后自动更新 URDF（需要同时提供 --update-urdf）。",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    # 优先使用命令行传入路径；未传时使用脚本开头配置；再回退到默认路径。
    if args.data_dir:
        base_dir = args.data_dir
    elif CALIBRATION_BASE_DIR:
        base_dir = CALIBRATION_BASE_DIR
    else:
        base_dir = "/root/kuavo_ws/tools/arm_accuracy_calibration/calibration_output/fitted_results"

    # 验证目录结构
    is_valid, errors = validate_directory_structure(base_dir)
    print_validation_result(is_valid, errors)
    
    # 如果只是验证模式，验证后直接返回
    if args.validate:
        return
    
    # 如果启用自动更新模式，验证失败则退出
    if args.auto_update:
        if not is_valid:
            print("目录结构验证失败，无法自动更新 URDF。")
            return
    
    # 如果验证失败且不是自动更新模式，继续执行（保持向后兼容）
    if not is_valid and not args.auto_update:
        print("警告: 目录结构不符合预设规则，但将继续执行...")
        print()

    # ---- 1) 读取并打印 joint_0 的 T 矩阵相关输出 ----
    joint0_dir = resolve_joint0_dir(base_dir)
    path_T21 = os.path.join(joint0_dir, "T_joint2_in_joint1_fitted.csv")
    path_T32 = os.path.join(joint0_dir, "T_joint3_in_joint2_fitted.csv")
    path_T43 = os.path.join(joint0_dir, "T_joint4_in_joint3_fitted.csv")
    path_T31 = os.path.join(joint0_dir, "T_joint3_in_joint1_fitted.csv")
    path_T41 = os.path.join(joint0_dir, "T_joint4_in_joint1_fitted.csv")
    path_T54 = os.path.join(joint0_dir, "T_joint5_in_joint4_fitted.csv")
    path_T65 = os.path.join(joint0_dir, "T_joint6_in_joint5_fitted.csv")

    try:
        T21 = load_matrix(path_T21)
        # 兼容两种导出格式：
        # 1) 直接给出相邻关节变换 T32/T43；
        # 2) 给出在 joint1 下的 T31/T41（旧格式）。
        if os.path.isfile(path_T32) and os.path.isfile(path_T43):
            T32 = load_matrix(path_T32)
            T43 = load_matrix(path_T43)
            T31_in_1 = T21 @ T32
            T41_in_1 = T31_in_1 @ T43
        else:
            T31_in_1 = load_matrix(path_T31)
            T41_in_1 = load_matrix(path_T41)
            # T_joint3_in_joint2 = T_joint2_in_joint1^(-1) @ T_joint3_in_joint1
            T21_inv = np.linalg.inv(T21)
            T32 = T21_inv @ T31_in_1
            # T_joint4_in_joint3 = T_joint3_in_joint1^(-1) @ T_joint4_in_joint1
            T31_inv = np.linalg.inv(T31_in_1)
            T43 = T31_inv @ T41_in_1

        T54 = load_matrix(path_T54)
        T65 = load_matrix(path_T65)
    except Exception as exc:
        print(f"加载 joint_0 T 矩阵失败: {exc}")
        return

    T_all = {}
    print_matrix("T_joint2_in_joint1", T21)
    print_euler_xyz_from_T("T_joint2_in_joint1", T21)
    T_all["T_joint2_in_joint1"] = T21

    print_matrix("T_joint3_in_joint2", T32)
    print_euler_xyz_from_T("T_joint3_in_joint2", T32)
    T_all["T_joint3_in_joint2"] = T32

    print_matrix("T_joint4_in_joint3", T43)
    print_euler_xyz_from_T("T_joint4_in_joint3", T43)
    T_all["T_joint4_in_joint3"] = T43

    print_matrix("T_joint5_in_joint4", T54)
    print_euler_xyz_from_T("T_joint5_in_joint4", T54)
    T_all["T_joint5_in_joint4"] = T54

    print_matrix("T_joint6_in_joint5", T65)
    print_euler_xyz_from_T("T_joint6_in_joint5", T65)
    T_all["T_joint6_in_joint5"] = T65

    print_matrix("T_joint3_in_joint1", T31_in_1)
    print_euler_xyz_from_T("T_joint3_in_joint1", T31_in_1)
    T_all["T_joint3_in_joint1"] = T31_in_1

    print_matrix("T_joint4_in_joint1", T41_in_1)
    print_euler_xyz_from_T("T_joint4_in_joint1", T41_in_1)
    T_all["T_joint4_in_joint1"] = T41_in_1

    print_summary_table(T_all)

    # 根据 config.yaml 中的 mapping_version 选择 T -> URDF 关节的映射关系
    mapping_version = 1
    config_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "config.yaml")
    if yaml is not None and os.path.isfile(config_path):
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                cfg_data = yaml.safe_load(f) or {}
            mapping_version = int(cfg_data.get("mapping_version", 1))
        except Exception:
            mapping_version = 1

    if mapping_version == 1:
        # 旧版：joint_1 -> zarm_l1_link, joint_2 -> zarm_l2_link, ...
        # T21: zarm_l2 in zarm_l1, T32: zarm_l3 in zarm_l2, ...
        joint_T_map = {
            "zarm_l2_joint": T21,
            "zarm_l3_joint": T32,
            "zarm_l4_joint": T43,
            "zarm_l5_joint": T54,
            "zarm_l6_joint": T65,
        }
        required_origin_joints = ("zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint", "zarm_l5_joint", "zarm_l6_joint")
    elif mapping_version == 2:
        # 新版：joint_1 -> waist_yaw_link, joint_2 -> zarm_l1_link, joint_3 -> zarm_l2_link, ...
        # 从 joint_2 开始修正手臂内部：T32 -> zarm_l2_joint, T43 -> zarm_l3_joint, ...
        joint_T_map = {
            "zarm_l2_joint": T32,
            "zarm_l3_joint": T43,
            "zarm_l4_joint": T54,
            "zarm_l5_joint": T65,
        }
        required_origin_joints = tuple(joint_T_map.keys())
    else:
        print(f"未知 mapping_version={mapping_version}，当前仅支持 1 或 2。")
        return
    print_urdf_origins_from_Ts(joint_T_map)

    # ---- 2) 读取并打印 joint_1~joint_5 的轴向输出（4/5 可选） ----
    joint_dir_to_urdf_joint = {
        "joint_1": "zarm_l1_joint",
        "joint_2": "zarm_l2_joint",
        "joint_3": "zarm_l3_joint",
        "joint_4": "zarm_l4_joint",
        "joint_5": "zarm_l5_joint",
    }
    joint_axis_map = {}
    for joint_dir, urdf_joint_name in joint_dir_to_urdf_joint.items():
        csv_path = os.path.join(base_dir, joint_dir, "circle_fit_result.csv")
        try:
            row = load_first_row(csv_path)
            joint_axis_map[urdf_joint_name] = parse_axis_from_row(row, csv_path)
        except Exception as exc:
            print(f"读取 axis 失败: {csv_path}")
            print(f"  错误: {exc}")
            return

    print_urdf_axis_updates(joint_axis_map)

    # 默认不修改 URDF，确保原有命令行打印保持不变；仅在传参时写回文件。
    # 如果启用自动更新模式，验证通过后自动更新
    if args.update_urdf_path or (args.auto_update and is_valid):
        urdf_path = args.update_urdf_path
        if args.auto_update and not urdf_path:
            # --auto-update 未指定 --update-urdf 时，回退到硬编码默认路径
            urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "biped_v3_arm_custom.urdf")
            print(f"[auto-update] 未指定 --update-urdf，使用默认路径: {urdf_path}")
        
        if urdf_path:
            try:
                update_urdf_file(
                    urdf_path,
                    joint_T_map,
                    joint_axis_map,
                    required_origin_joints=required_origin_joints,
                    required_axis_joints=tuple(joint_axis_map.keys()),
                )
                print(f"✓ URDF 已更新: {urdf_path}")
            except Exception as exc:
                print(f"✗ 更新 URDF 失败: {exc}")
                return


if __name__ == "__main__":
    main()
