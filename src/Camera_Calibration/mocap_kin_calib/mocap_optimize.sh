#!/usr/bin/env bash
# S53/S56 与 S62/S63 小版本：25 点拟合、5 点验证、30 点最终拟合。
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_NAME="$(python3 "${SCRIPT_DIR}/robot_variants.py" "${ROBOT_VERSION:-}")" || exit 2
DEFAULT_CONFIG="${SCRIPT_DIR}/config/${CONFIG_NAME}"
[[ $# -ge 1 ]] || {
  echo "用法: ROBOT_VERSION=63 bash ${BASH_SOURCE[0]} <capture.json> [--urdf PATH] [--output-urdf [--force-output]]" >&2
  echo "验证: ROBOT_VERSION=63 bash ${BASH_SOURCE[0]} <capture_vali.json> --validate-urdf PATH" >&2
  echo "--output-urdf 成功后询问 yes/no；yes 备份并同步实际 62/63 小版本的 urdf 目录。验证与仅输出 JSON 均不替换模型。" >&2
  exit 2
}
# 后面的显式 --config 可覆盖默认配置；Python 会校验 capture 版本。
exec python3 "${SCRIPT_DIR}/mocap_optimize_1010.py" --config "${DEFAULT_CONFIG}" "$@"
