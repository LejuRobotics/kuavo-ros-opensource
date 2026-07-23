#!/bin/bash
# 离线标定流程脚本
# 流程：1. 保存位姿数据到CSV 2. 筛选并拟合中心（自动更新YAML）3. 运行离线标定
# 用法: ./run_offline_calibration_pipeline.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
FUNCTIONS_DIR="$SCRIPT_DIR/functions"

echo "=========================================="
echo "  离线标定流程"
echo "=========================================="

# 1. Source ROS 环境
echo "[1/4] 加载 ROS 环境..."
cd "$WORKSPACE_ROOT"
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
elif [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "[ERROR] 未找到 devel/setup.bash 或 install/setup.bash"
    exit 1
fi

# 2. 保存位姿数据到 CSV
echo "[2/4] 保存位姿数据到 CSV..."
echo "请确保 ROS master 和 motioncapture 正在运行"
read -p "按 Enter 继续，或 Ctrl+C 退出: "

cd "$FUNCTIONS_DIR"
python3 save_pose_topics_to_csv.py || {
    echo "[ERROR] save_pose_topics_to_csv.py 执行失败"
    exit 1
}

# 3. 筛选并拟合中心（自动更新 fitting_data.yaml）
echo "[3/4] 筛选并拟合中心..."
python3 fit_center_by_pose.py || {
    echo "[ERROR] fit_center_by_pose.py 执行失败"
    exit 1
}

# 4. 运行离线标定
echo "[4/4] 运行离线标定..."
cd "$SCRIPT_DIR"
python3 run_offline_calibration.py fitting_data.yaml || {
    echo "[ERROR] run_offline_calibration.py 执行失败"
    exit 1
}

echo ""
echo "=========================================="
echo "  离线标定流程完成"
echo "=========================================="

