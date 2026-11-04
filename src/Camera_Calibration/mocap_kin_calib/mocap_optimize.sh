#!/usr/bin/env bash
# S53/S55/S56 与 S62/S63 小版本：25 点拟合、5 点留出验证、通过阈值后 30 点最终拟合。
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION="${ROBOT_VERSION:-}"
if [[ ! "$VERSION" =~ ^[0-9]+$ ]]; then
  echo "[ERROR] ROBOT_VERSION 必须是数字（当前=${VERSION:-未设置}）" >&2
  exit 2
fi
# 路由用 %100 折回基础型号（200053/400055 -> 53/55，500062/400063 -> 62/63），
# 布局与标定输入按统一机型表（Camera_Calibration/robot_versions.py）判定；
# mocap_kin_calib 内部其余脚本仍走 robot_variants.py 的严格编码校验，两者对
# 合法基础型号族（53/55/56 双足、62/63 轮臂）的判定一致。
case "$((10#$VERSION % 100))" in
  53|55|56) DEFAULT_CONFIG="${SCRIPT_DIR}/config/calib_s56.yaml" ;;
  62|63) DEFAULT_CONFIG="${SCRIPT_DIR}/config/calib_s63.yaml" ;;
  *)
    echo "[ERROR] 运动学标定当前支持基础版本 53/55/56/62/63 及对应小版本（当前=${VERSION}，折回 $((10#$VERSION % 100))）" >&2
    exit 2
    ;;
esac
[[ $# -ge 1 ]] || {
  echo "用法: ROBOT_VERSION=63 bash ${BASH_SOURCE[0]} <capture.json> [--urdf PATH] [--output-urdf [--force-output]]" >&2
  echo "验证: ROBOT_VERSION=63 bash ${BASH_SOURCE[0]} <capture_vali.json> --validate-urdf PATH" >&2
  echo "--output-urdf 成功后询问 yes/no；yes 备份并同步实际 62/63 小版本的 urdf 目录。验证与仅输出 JSON 均不替换模型。" >&2
  exit 2
}
# 后面的显式 --config 可覆盖默认配置；Python 会按统一机型表校验 capture 版本，
# 并按完整版本号使用 kuavo_assets 中该版本自身的主 URDF 拟合。
exec python3 "${SCRIPT_DIR}/mocap_optimize_1010.py" --config "${DEFAULT_CONFIG}" "$@"
