#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取变换矩阵CSV文件并输出两种格式：
1. 4x4矩阵样式
2. [x, y, z]平移向量样式
"""

import csv
import sys
import numpy as np
import argparse


def read_transform_matrix_from_csv(csv_path):
    """
    从CSV文件读取4x4变换矩阵
    
    Args:
        csv_path: CSV文件路径
        
    Returns:
        numpy.ndarray: 4x4变换矩阵
    """
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        # 跳过标题行
        header = next(reader)
        # 读取数据行
        data_row = next(reader)
        
        # 将字符串转换为浮点数
        values = [float(x) for x in data_row]
        
        # 重塑为4x4矩阵
        matrix = np.array(values).reshape(4, 4)
        
    return matrix


def print_matrix_format(matrix):
    """
    以4x4矩阵样式打印
    
    Args:
        matrix: 4x4变换矩阵
    """
    print("\n=== 4x4矩阵格式 ===")
    print(f"[{matrix[0,0]:12.6f}, {matrix[0,1]:12.6f}, {matrix[0,2]:12.6f}, {matrix[0,3]:12.6f}]")
    print(f"[{matrix[1,0]:12.6f}, {matrix[1,1]:12.6f}, {matrix[1,2]:12.6f}, {matrix[1,3]:12.6f}]")
    print(f"[{matrix[2,0]:12.6f}, {matrix[2,1]:12.6f}, {matrix[2,2]:12.6f}, {matrix[2,3]:12.6f}]")
    print(f"[{matrix[3,0]:12.6f}, {matrix[3,1]:12.6f}, {matrix[3,2]:12.6f}, {matrix[3,3]:12.6f}]")


def print_translation_vector(matrix):
    """
    以[x, y, z]样式打印平移向量
    
    Args:
        matrix: 4x4变换矩阵
    """
    translation = matrix[:3, 3]  # 提取平移部分（最后一列的前三个元素）
    print("\n=== 平移向量格式 [x, y, z] ===")
    print(f"[{translation[0]:.6f}, {translation[1]:.6f}, {translation[2]:.6f}]")
    print(f"\n详细值:")
    print(f"  x: {translation[0]}")
    print(f"  y: {translation[1]}")
    print(f"  z: {translation[2]}")


def main():
    parser = argparse.ArgumentParser(description='读取变换矩阵CSV文件并输出两种格式')
    parser.add_argument('csv_path', type=str, help='CSV文件路径')
    
    args = parser.parse_args()
    
    try:
        # 读取矩阵
        matrix = read_transform_matrix_from_csv(args.csv_path)
        
        # 输出两种格式
        print(f"\n文件: {args.csv_path}")
        print_matrix_format(matrix)
        print_translation_vector(matrix)
        
    except FileNotFoundError:
        print(f"错误: 文件 '{args.csv_path}' 不存在", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"错误: {str(e)}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
