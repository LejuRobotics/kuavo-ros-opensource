#!/bin/bash
# 一键采集：输出文件保存在本脚本同目录下（需 Motive 已开流且 roslaunch 已运行）
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUT_CSV="$SCRIPT_DIR/mocap_poses_${TIMESTAMP}.csv"
OUT_JSON="$SCRIPT_DIR/checkerboard_relative_poses_${TIMESTAMP}.json"

source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null || true
if [ -f "$WS_ROOT/devel/setup.bash" ]; then
  source "$WS_ROOT/devel/setup.bash"
fi

echo "=========================================="
echo "  Motive 棋盘位姿采集"
echo "=========================================="
echo "  目录: $SCRIPT_DIR"
echo "  CSV : $OUT_CSV"
echo "  时长: 10s + 预热 2s"
echo "------------------------------------------"
echo "请确认：Motive 开流、checkerboard/torso/l_shoulder 全绿、roslaunch 已运行、现场静止"
echo ""

python3 "$SCRIPT_DIR/record_mocap_poses.py" \
  --duration 10 \
  --warmup 2 \
  --wait_timeout 30 \
  --output "$OUT_CSV"

echo ""
echo "离线处理 JSON："
python3 "$SCRIPT_DIR/process_mocap_poses.py" \
  --input "$OUT_CSV" \
  --output "$OUT_JSON"
