#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
批量拟合齐次变换矩阵：读取matrix_data文件夹中的所有CSV文件，拟合出三个齐次变换矩阵。
然后从matrix_data_motion中读取旋转轴数据并拟合。
"""

import os
import sys
import glob
import numpy as np
import yaml

# 添加当前目录到路径
_current_dir = os.path.dirname(os.path.abspath(__file__))
if _current_dir not in sys.path:
    sys.path.insert(0, _current_dir)

from fit_matrix_ls import fit_matrix_from_csv, save_matrix_to_csv
from fit_rotation_axis_ls import fit_rotation_axis_from_csv
from dh_calibration import initialize_dh_from_transform


def load_dh_reference_frames(config_path=None):
    """
    加载DH参考坐标系变换矩阵。
    
    Args:
        config_path: 配置文件路径，默认为 config/dh_reference_frames.yaml
        
    Returns:
        dict: 包含T_dh_i_to_J_i矩阵的字典，键为 'T_dh_1_to_J_1', 'T_dh_2_to_J_2', 等
    """
    if config_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        calibration_dir = os.path.dirname(script_dir)
        config_path = os.path.join(calibration_dir, "config", "dh_reference_frames.yaml")
    
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"DH参考坐标系配置文件不存在: {config_path}")
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # 提取所有 T_dh_i_to_J_i 矩阵（dh 与 joint 一一对应：dh1<->J1, dh2<->J2, dh3<->J3, dh4<->J4）
    dh_frames = {}
    required_keys = [
        'T_dh_1_to_J_1',
        'T_dh_2_to_J_2',
        'T_dh_3_to_J_3',
        'T_dh_4_to_J_4',
    ]
    for key in required_keys:
        if key in config:
            matrix_list = config[key]
            T = np.array(matrix_list, dtype=float)
            dh_frames[key] = T
        else:
            raise ValueError(f"配置文件中缺少 {key}")
    
    return dh_frames


def apply_dh_reference_frame_transform(T_joint, file_key, dh_frames):
    """
    应用DH参考坐标系转换：将关节系下的 T 转为 DH 系下的相邻连杆变换。
    
    转换公式（dh 与 joint 一一对应，结果为 T_dh_i_to_dh_{i+1}）：
    - T_joint2_in_joint1 -> T_dh1_to_dh2 = inv(T_dh_2_to_J_2) @ inv(T_joint) @ T_dh_1_to_J_1
    - T_joint3_in_joint2 -> T_dh2_to_dh3 = inv(T_dh_3_to_J_3) @ inv(T_joint) @ T_dh_2_to_J_2
    - T_joint4_in_joint3 -> T_dh3_to_dh4 = inv(T_dh_4_to_J_4) @ inv(T_joint) @ T_dh_3_to_J_3
    
    Args:
        T_joint: 原始关节坐标系下的变换矩阵（T_joint_{i+1}_in_joint_i）
        file_key: 文件名（不含扩展名），如 'T_joint2_in_joint1'
        dh_frames: load_dh_reference_frames返回的字典
        
    Returns:
        np.ndarray: DH 系下的相邻连杆变换（T_dh_i_to_dh_{i+1}）
    """
    if file_key == 'T_joint2_in_joint1':
        T_dh_1_to_J_1 = dh_frames['T_dh_1_to_J_1']
        T_dh_2_to_J_2 = dh_frames['T_dh_2_to_J_2']
        T_dh_2_to_J_2_inv = np.linalg.inv(T_dh_2_to_J_2)
        T_joint_inv = np.linalg.inv(T_joint)
        T_dh = T_dh_2_to_J_2_inv @ T_joint_inv @ T_dh_1_to_J_1
    elif file_key == 'T_joint3_in_joint2':
        T_dh_2_to_J_2 = dh_frames['T_dh_2_to_J_2']
        T_dh_3_to_J_3 = dh_frames['T_dh_3_to_J_3']
        T_dh_3_to_J_3_inv = np.linalg.inv(T_dh_3_to_J_3)
        T_joint_inv = np.linalg.inv(T_joint)
        T_dh = T_dh_3_to_J_3_inv @ T_joint_inv @ T_dh_2_to_J_2
    elif file_key == 'T_joint4_in_joint3':
        T_dh_3_to_J_3 = dh_frames['T_dh_3_to_J_3']
        T_dh_4_to_J_4 = dh_frames['T_dh_4_to_J_4']
        T_dh_4_to_J_4_inv = np.linalg.inv(T_dh_4_to_J_4)
        T_joint_inv = np.linalg.inv(T_joint)
        T_dh = T_dh_4_to_J_4_inv @ T_joint_inv @ T_dh_3_to_J_3
    else:
        return T_joint
    
    return T_dh


def batch_fit_matrices_from_dir(data_dir="matrix_data", x_threshold=999.0, 
                                 output_dir=None, verbose=True):
    """
    批量读取matrix_data文件夹中的CSV文件并拟合齐次变换矩阵。
    然后从matrix_data_motion中读取旋转轴数据并拟合。
    
    Args:
        data_dir: 数据文件夹路径（默认: "matrix_data"）
        x_threshold: x位置阈值，大于此值的矩阵将被筛掉（默认: 999.0）
        output_dir: 输出文件夹路径（默认: None，不保存）
        verbose: 是否打印详细信息（默认: True）
        
    Returns:
        dict: 包含拟合结果的字典，键为文件名（不含扩展名），值为拟合结果字典
    """
    # 获取脚本所在目录的父目录（calibration目录）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calibration_dir = os.path.dirname(script_dir)
    
    # 构建完整的数据目录路径
    if os.path.isabs(data_dir):
        full_data_dir = data_dir
    else:
        full_data_dir = os.path.join(calibration_dir, data_dir)
    
    if not os.path.exists(full_data_dir):
        raise ValueError(f"数据文件夹不存在: {full_data_dir}")
    
    # 查找所有CSV文件
    csv_pattern = os.path.join(full_data_dir, "*.csv")
    csv_files = sorted(glob.glob(csv_pattern))
    
    if len(csv_files) == 0:
        raise ValueError(f"在 {full_data_dir} 中未找到CSV文件")
    
    if verbose:
        print("=" * 70)
        print("第一步：批量拟合齐次变换矩阵")
        print("=" * 70)
        print(f"数据文件夹: {full_data_dir}")
        print(f"找到 {len(csv_files)} 个CSV文件")
        print("说明：齐次变换矩阵拟合不需要参考轴")
        print("=" * 70)
    
    # 加载DH参考坐标系矩阵
    try:
        dh_frames = load_dh_reference_frames()
        if verbose:
            print("✓ 已加载DH参考坐标系矩阵")
    except Exception as e:
        if verbose:
            print(f"\n警告: 加载DH参考坐标系矩阵失败: {e}")
        dh_frames = None
    
    # 存储所有拟合结果
    results = {}
    
    # 遍历每个CSV文件进行拟合
    for csv_file in csv_files:
        filename = os.path.basename(csv_file)
        file_key = os.path.splitext(filename)[0]  # 去掉扩展名
        
        if verbose:
            print(f"处理文件: {filename}...", end=" ", flush=True)
        
        try:
            # 调用拟合函数（静默模式，不打印详细信息）
            result = fit_matrix_from_csv(
                csv_path=csv_file,
                x_threshold=x_threshold,
                verbose=False  # 不打印详细信息
            )
            
            # 应用DH参考坐标系转换
            if dh_frames is not None:
                T_original = result['T_fit']
                T_dh = apply_dh_reference_frame_transform(T_original, file_key, dh_frames)
                result['T_fit'] = T_dh
                result['T_fit_original'] = T_original  # 保存原始矩阵
            
            if verbose:
                print("✓")
            
            results[file_key] = result
            
            # 如果指定了输出目录，保存拟合结果
            if output_dir is not None:
                # 构建输出目录路径
                if os.path.isabs(output_dir):
                    full_output_dir = output_dir
                else:
                    full_output_dir = os.path.join(calibration_dir, output_dir)
                
                os.makedirs(full_output_dir, exist_ok=True)
                
                # 保存拟合矩阵（转换后的DH坐标系矩阵）
                output_file = os.path.join(full_output_dir, f"{file_key}_fitted.csv")
                save_matrix_to_csv(result['T_fit'], output_file, verbose=verbose)
        
        except Exception as e:
            if verbose:
                print(f"\n✗ 处理文件 {filename} 时出错: {e}")
            import traceback
            traceback.print_exc()
            continue
    
    
    # ========================================================================
    # 第二步：拟合旋转轴（独立于矩阵拟合）
    # ========================================================================
    axis_results = {}
    
    # 检查是否有matrix_data_motion文件夹
    motion_data_base = os.path.join(calibration_dir, "matrix_data_motion")
    
    if os.path.exists(motion_data_base):
        if verbose:
            print("\n" + "=" * 70)
            print("第二步：拟合旋转轴（圆拟合）...")
            print("说明：参考轴仅用于偏差评估和可视化，不参与圆拟合过程")
        
        # 文件映射：joint_X -> 对应的CSV文件名 -> 参考轴
        joint_config = {
            1: {
                'csv_file': 'T_joint2_in_joint1.csv',
                'reference_axis': 'y'  # joint_1 使用 Y 轴
            },
            2: {
                'csv_file': 'T_joint3_in_joint2.csv',
                'reference_axis': 'x'  # joint_2 使用 X 轴
            },
            3: {
                'csv_file': 'T_joint4_in_joint3.csv',
                'reference_axis': 'z'  # joint_3 使用 Z 轴
            }
        }
        
        # 遍历关节1、2、3
        for joint_idx in [1, 2, 3]:
            joint_dir = os.path.join(motion_data_base, f"joint_{joint_idx}")
            csv_filename = joint_config[joint_idx]['csv_file']
            csv_path = os.path.join(joint_dir, csv_filename)
            reference_axis = joint_config[joint_idx]['reference_axis']
            
            if not os.path.exists(csv_path):
                if verbose:
                    print(f"跳过 joint_{joint_idx}: 文件不存在 ({csv_path})")
                continue
            
            if verbose:
                print(f"处理 joint_{joint_idx} ({csv_filename}, 参考轴: {reference_axis.upper()})...", end=" ", flush=True)
            
            try:
                # 生成3D可视化图片路径
                image_dir = os.path.join(calibration_dir, "image")
                os.makedirs(image_dir, exist_ok=True)
                image_filename = f"rotation_axis_joint_{joint_idx}.png"
                image_path = os.path.join(image_dir, image_filename)
                
                # 调用旋转轴拟合函数（包含图片生成）
                axis_result = fit_rotation_axis_from_csv(
                    csv_path=csv_path,
                    reference_axis_name=reference_axis,
                    x_filter_threshold=900.0,
                    verbose=False,  # 不打印详细信息
                    unit='mm',  # 与矩阵单位保持一致
                    output_image_path=image_path  # 指定图片保存路径
                )
                
                axis_results[f"joint_{joint_idx}"] = {
                    'file': csv_filename,
                    'csv_path': csv_path,
                    'result': axis_result,
                    'image_path': image_path
                }
                
                if verbose:
                    print(f"✓ (图片: {image_filename})")
            
            except Exception as e:
                if verbose:
                    print(f"\n✗ 处理 joint_{joint_idx} 时出错: {e}")
                import traceback
                traceback.print_exc()
                continue
        
        if verbose and len(axis_results) > 0:
            print(f"✓ 成功拟合 {len(axis_results)} 个关节的旋转轴")
    
    # 将旋转轴结果添加到返回字典中
    results['axis_results'] = axis_results
    
    # ========================================================================
    # 第三步：直接从转换后的T矩阵计算DH参数
    # ========================================================================
    dh_results = {}
    
    # T矩阵文件名列表
    matrix_keys = ['T_joint2_in_joint1', 'T_joint3_in_joint2', 'T_joint4_in_joint3']
    
    if verbose and len(results) > 0:
        print("\n" + "=" * 70)
        print("第三步：从T矩阵直接计算DH参数")
        print("=" * 70)
    
    for matrix_key in matrix_keys:
        # 检查T矩阵是否存在
        if matrix_key not in results or 'T_fit' not in results[matrix_key]:
            if verbose:
                print(f"跳过 {matrix_key}: T矩阵不存在")
            continue
        
        if verbose:
            print(f"计算 {matrix_key} 的DH参数...", end=" ", flush=True)
        
        try:
            # 获取转换后的T矩阵（DH坐标系下）
            T_meas = results[matrix_key]['T_fit']
            
            # 直接从T矩阵提取DH参数（理论上应该精确，没有误差）
            dh_params = initialize_dh_from_transform(T_meas)
            
            dh_results[matrix_key] = {
                'dh_params': dh_params,  # [a, alpha, d, theta]
            }
            
            if verbose:
                print("✓")
        
        except Exception as e:
            if verbose:
                print(f"\n✗ 计算 {matrix_key} 的DH参数时出错: {e}")
            import traceback
            traceback.print_exc()
            continue
    
    # 将DH结果添加到返回字典中
    results['dh_results'] = dh_results
    
    if verbose and len(dh_results) > 0:
        print(f"✓ 成功计算 {len(dh_results)} 组DH参数")
    
    return results


def print_fitted_matrices_summary(results):
    """
    打印拟合矩阵和旋转轴的摘要信息。
    
    Args:
        results: batch_fit_matrices_from_dir返回的结果字典
    """
    print("\n" + "=" * 70)
    print("拟合结果摘要")
    print("=" * 70)
    
    # 打印三个矩阵
    matrix_keys = ['T_joint2_in_joint1', 'T_joint3_in_joint2', 'T_joint4_in_joint3']
    print("\n【三个齐次变换矩阵】")
    print("-" * 70)
    for key in matrix_keys:
        if key in results and 'T_fit' in results[key]:
            # 打印转换后的DH坐标系矩阵
            T_dh = results[key]['T_fit']
            print(f"\n{key} (DH坐标系):")
            for i in range(4):
                print(f"  [{T_dh[i,0]:12.6f}  {T_dh[i,1]:12.6f}  {T_dh[i,2]:12.6f}  {T_dh[i,3]:12.6f}]")
            
            # 如果有原始矩阵，也打印出来
            if 'T_fit_original' in results[key]:
                T_original = results[key]['T_fit_original']
                print(f"\n{key} (转换前，关节坐标系):")
                for i in range(4):
                    print(f"  [{T_original[i,0]:12.6f}  {T_original[i,1]:12.6f}  {T_original[i,2]:12.6f}  {T_original[i,3]:12.6f}]")
    
    # 打印三个旋转轴
    if 'axis_results' in results and len(results['axis_results']) > 0:
        print("\n【三个旋转轴】")
        print("-" * 70)
        joint_keys = ['joint_1', 'joint_2', 'joint_3']
        for joint_key in joint_keys:
            if joint_key in results['axis_results']:
                axis_data = results['axis_results'][joint_key]
                result = axis_data['result']
                print(f"\n{joint_key} ({axis_data['file']}):")
                print(f"  圆心: ({result['center'][0]:12.6f}, {result['center'][1]:12.6f}, {result['center'][2]:12.6f}) mm")
                print(f"  半径: {result['radius']:12.6f} mm")
                print(f"  转轴方向: ({result['axis'][0]:8.6f}, {result['axis'][1]:8.6f}, {result['axis'][2]:8.6f})")
                print(f"  偏差角度: {result['deviation']['total_angle_deg']:8.4f}°")
            else:
                print(f"\n{joint_key}: (未找到数据)")
    
    # 打印DH参数
    if 'dh_results' in results and len(results['dh_results']) > 0:
        print("\n【DH参数】")
        print("-" * 70)
        matrix_keys = ['T_joint2_in_joint1', 'T_joint3_in_joint2', 'T_joint4_in_joint3']
        for key in matrix_keys:
            if key in results['dh_results']:
                dh_data = results['dh_results'][key]
                dh = dh_data['dh_params']
                print(f"\n{key}:")
                print(f"  a (连杆长度):     {dh[0]:12.6f} mm")
                print(f"  alpha (扭转角):   {dh[1]:12.6f} rad = {np.degrees(dh[1]):8.4f}°")
                print(f"  d (连杆偏距):     {dh[2]:12.6f} mm")
                print(f"  theta (关节角):   {dh[3]:12.6f} rad = {np.degrees(dh[3]):8.4f}°")
            else:
                print(f"\n{key}: (未找到数据)")
    
    print("\n" + "=" * 70)


def main():
    """命令行入口"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="批量拟合matrix_data文件夹中的齐次变换矩阵",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python batch_fit_matrices.py
  python batch_fit_matrices.py --data_dir matrix_data_motion/joint_1
  python batch_fit_matrices.py --output_dir fitted_matrices --x_threshold 500
        """
    )
    
    parser.add_argument("--data_dir", type=str, default="matrix_data",
                        help="数据文件夹路径（默认: matrix_data）")
    parser.add_argument("--x_threshold", type=float, default=999.0,
                        help="x位置阈值，大于此值的矩阵将被筛掉（默认: 999.0）")
    parser.add_argument("--output_dir", "-o", type=str, default=None,
                        help="输出文件夹路径（可选，默认不保存）")
    parser.add_argument("--quiet", "-q", action="store_true",
                        help="静默模式，不打印详细信息")
    
    args = parser.parse_args()
    
    # 调用批量拟合函数
    results = batch_fit_matrices_from_dir(
        data_dir=args.data_dir,
        x_threshold=args.x_threshold,
        output_dir=args.output_dir,
        verbose=not args.quiet
    )
    
    # 打印矩阵摘要
    if not args.quiet:
        print_fitted_matrices_summary(results)
    
    print("\n完成！")


if __name__ == "__main__":
    main()
