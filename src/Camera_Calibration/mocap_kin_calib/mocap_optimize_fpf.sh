#!/usr/bin/env bash
# 采集拟合点、检查数据质量、采集验证点、拟合并验证候选 URDF。
# OptiTrack / 青瞳 VRPN 由测试人员手动启动，本脚本只使用已发布的话题。
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBOT_VERSION="${ROBOT_VERSION:-62}"
exec python3 -u "${SCRIPT_DIR}/mocap_optimize_fpf.py" "$@"
