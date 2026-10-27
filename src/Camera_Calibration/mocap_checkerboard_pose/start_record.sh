#!/bin/bash
# 一键采集：支持 Motive/青瞳，输出文件保存在本脚本同目录下
set -e

MOCAP_SOURCE="${1:-}"
ROBOT_VERSION_SELECTED="${2:-${ROBOT_VERSION:-}}"
if [ "$#" -gt 2 ] || { [ -n "$MOCAP_SOURCE" ] && [ "$MOCAP_SOURCE" != "motive" ] && [ "$MOCAP_SOURCE" != "qingtong" ]; }; then
  echo "用法: bash $0 [motive|qingtong] [45|52|56|62|63]" >&2
  exit 2
fi
case "$ROBOT_VERSION_SELECTED" in
  45|52|56|62|63) ;;
  *) echo "请通过第二个参数或 ROBOT_VERSION 指定机器人版本" >&2; exit 2 ;;
esac

SOURCE_ARGS=()
if [ -n "$MOCAP_SOURCE" ]; then
  SOURCE_ARGS=(--mocap-source "$MOCAP_SOURCE")
fi
SOURCE_DISPLAY="${MOCAP_SOURCE:-bodies.yaml 配置}"

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
echo "  棋盘位姿采集"
echo "=========================================="
echo "  动捕源: $SOURCE_DISPLAY"
echo "  机器人: $ROBOT_VERSION_SELECTED"
echo "  目录: $SCRIPT_DIR"
echo "  CSV : $OUT_CSV"
echo "  时长: 10s + 预热 2s"
echo "------------------------------------------"
echo "请确认：动捕已开流、checkerboard/torso/l_shoulder 有效、ROS 接收节点已运行、现场静止"
echo ""

python3 "$SCRIPT_DIR/record_mocap_poses.py" \
  "${SOURCE_ARGS[@]}" \
  --duration 10 \
  --warmup 2 \
  --wait_timeout 30 \
  --output "$OUT_CSV"

echo ""
echo "离线处理 JSON："
python3 "$SCRIPT_DIR/process_mocap_poses.py" \
  --robot-version "$ROBOT_VERSION_SELECTED" \
  --input "$OUT_CSV" \
  --output "$OUT_JSON"
