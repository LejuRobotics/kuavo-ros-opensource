#!/usr/bin/env bash
# S53/S56/S62/S63：25 点拟合、5 点留出验证、通过阈值后 30 点最终拟合。
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
case "${ROBOT_VERSION:-}" in
  53|56) DEFAULT_CONFIG="${SCRIPT_DIR}/config/calib_s56.yaml" ;;
  62|63) DEFAULT_CONFIG="${SCRIPT_DIR}/config/calib_s63.yaml" ;;
  *)
    echo "[ERROR] 当前支持 ROBOT_VERSION=53/56/62/63（当前=${ROBOT_VERSION:-未设置}）" >&2
    exit 2
    ;;
esac
[[ $# -ge 1 ]] || {
  echo "用法: ROBOT_VERSION=63 bash ${BASH_SOURCE[0]} <capture.json> [--urdf PATH] [--output-urdf [--force-output]]" >&2
  echo "验证: ROBOT_VERSION=63 bash ${BASH_SOURCE[0]} <capture_vali.json> --validate-urdf PATH" >&2
  echo "--output-urdf 成功后询问 yes/no；yes 备份并同步 S63 urdf 目录全部模型。验证与仅输出 JSON 均不替换模型。" >&2
  exit 2
}
# 后面的显式 --config 可覆盖默认配置；Python 会校验 capture 版本。
exec python3 "${SCRIPT_DIR}/mocap_optimize_1010.py" --config "${DEFAULT_CONFIG}" "$@"
