#!/bin/bash
# 下采样并绘制位姿数据脚本
# 用法: ./run_downsample_and_plot.sh [--block-size N]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FUNCTIONS_DIR="$SCRIPT_DIR/functions"
POSE_DATA_DIR="$SCRIPT_DIR/pose_data"
CSV_FILE="$POSE_DATA_DIR/calimark_pose.csv"

echo "=========================================="
echo "  下采样并绘制位姿数据"
echo "=========================================="

# 检查CSV文件是否存在
if [ ! -f "$CSV_FILE" ]; then
    echo "[ERROR] CSV文件不存在: $CSV_FILE"
    exit 1
fi

# 检查Python脚本是否存在
PYTHON_SCRIPT="$FUNCTIONS_DIR/downsample_and_plot_poses.py"
if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "[ERROR] Python脚本不存在: $PYTHON_SCRIPT"
    exit 1
fi

# 执行Python脚本，传递所有参数和CSV文件路径
echo "[INFO] 调用 downsample_and_plot_poses.py..."
echo "[INFO] CSV文件: $CSV_FILE"
cd "$FUNCTIONS_DIR"
python3 downsample_and_plot_poses.py "$@" "$CSV_FILE" || {
    echo "[ERROR] downsample_and_plot_poses.py 执行失败"
    exit 1
}

echo ""
echo "=========================================="
echo "  下采样并绘制完成"
echo "=========================================="
