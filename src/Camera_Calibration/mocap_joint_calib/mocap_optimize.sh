#!/usr/bin/env bash
# 纯动捕关节零位标定（方案 2a）——一键优化
#
# 自动完成：定位 workspace → source devel/setup.bash → 起 roscore（如未起）→
#           自动找最新 capture_*.json → 跑 optimize_mocap（Ceres）。
# 机型自动适配：45/52/56/62/63 → 对应标定 URDF；可显式 --layout 覆盖。
#
# 用法：
#   bash mocap_optimize.sh                     # 自动用最新 capture + 按 ROBOT_VERSION 选机型
#   bash mocap_optimize.sh <capture.json>      # 指定 capture 文件
#   bash mocap_optimize.sh <capture.json> --layout biped56
#   bash mocap_optimize.sh --layout wheel62
set -Ee -o pipefail

# 定位 workspace：脚本在 src/Camera_Calibration/mocap_joint_calib/ 下，上溯三级
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

die() { echo "[ERROR] $1" >&2; exit 1; }

OUTPUT_DIR="${SCRIPT_DIR}/output"
FINAL_CALIBRATION="${OUTPUT_DIR}/calibration.yaml"
TMP_OUTPUT_DIR=""

cleanup() {
  if [[ -n "${TMP_OUTPUT_DIR}" ]]; then
    rm -f -- "${TMP_OUTPUT_DIR}/calibration.yaml"
    rmdir -- "${TMP_OUTPUT_DIR}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

# 解析参数：位置参数 1 为 capture；--layout 指定机型。
CAPTURE=""
LAYOUT=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --layout)
      [[ $# -ge 2 ]] || die "--layout 缺少参数（仅 biped45|biped52|biped56|wheel62）"
      LAYOUT="${2:-}"; shift 2 ;;
    -h|--help)
      echo "用法: bash mocap_optimize.sh [capture.json] [--layout biped45|biped52|biped56|wheel62]"; exit 0 ;;
    *)
      if [[ -z "${CAPTURE}" ]]; then CAPTURE="$1"; else echo "未知参数: $1" >&2; exit 1; fi
      shift ;;
  esac
done

# 1) 指定或自动找最新 capture
if [[ -z "${CAPTURE}" ]]; then
  CAPTURE="$(find "${SCRIPT_DIR}" -maxdepth 1 -type f -name 'capture_*.json' \
    -printf '%T@ %p\n' | sort -nr | awk 'NR == 1 { sub(/^[^ ]+ /, ""); print }')"
fi
[[ -n "${CAPTURE}" ]] || die "未找到 capture_*.json，请先运行 record_joint_poses.py 采集，或指定文件参数"
[[ -f "${CAPTURE}" ]] || die "capture 文件不存在: ${CAPTURE}"
# 转为绝对路径：optimize_mocap 节点的 cwd 不是 workspace 根，相对路径会读不到
CAPTURE="$(readlink -f -- "${CAPTURE}")"
echo "[INFO] capture: ${CAPTURE}"

# 2) 从 capture metadata 推断机型。路径通过 argv 传入，避免拼进 Python 代码。
if ! META_TEXT="$(python3 - "${CAPTURE}" <<'PY'
import json
import sys

try:
    with open(sys.argv[1], "r", encoding="utf-8") as stream:
        root = json.load(stream)
    meta = root.get("meta")
    if not isinstance(meta, dict):
        raise ValueError("缺少对象类型的 meta")
    print(str(meta.get("fk_root", "")))
    print(str(meta.get("urdf", "")))
    print(str(meta.get("robot_version", "")))
    print(str(meta.get("robot_layout", "")))
except Exception as exc:
    print(f"capture JSON 无效: {exc}", file=sys.stderr)
    raise SystemExit(1)
PY
)"; then
  die "无法读取 capture metadata: ${CAPTURE}"
fi
mapfile -t META_FIELDS <<< "${META_TEXT}"
META_FK="${META_FIELDS[0]:-}"
META_URDF="${META_FIELDS[1]:-}"
META_VERSION="${META_FIELDS[2]:-}"
META_LAYOUT="${META_FIELDS[3]:-}"

INFERRED_LAYOUT=""
set_inferred_layout() {
  local candidate="$1"
  local source="$2"
  [[ -n "${candidate}" ]] || return 0
  if [[ -n "${INFERRED_LAYOUT}" && "${INFERRED_LAYOUT}" != "${candidate}" ]]; then
    die "capture metadata 自相矛盾: 已推断 ${INFERRED_LAYOUT}，但 ${source} 推断 ${candidate}"
  fi
  INFERRED_LAYOUT="${candidate}"
}

case "${META_LAYOUT}" in
  biped45|biped52|biped56|wheel62) set_inferred_layout "${META_LAYOUT}" "robot_layout" ;;
  "") ;;
  *) die "capture meta.robot_layout 不支持: ${META_LAYOUT}" ;;
esac

case "${META_VERSION}" in
  45) set_inferred_layout "biped45" "robot_version=${META_VERSION}" ;;
  52) set_inferred_layout "biped52" "robot_version=${META_VERSION}" ;;
  56) set_inferred_layout "biped56" "robot_version=${META_VERSION}" ;;
  62|63) set_inferred_layout "wheel62" "robot_version=${META_VERSION}" ;;
  ""|None|null) ;;
  *) die "capture meta.robot_version=${META_VERSION} 尚未适配" ;;
esac

URDF_LAYOUT=""
case "${META_URDF##*/}" in
  biped_v3_arm_s45.urdf) URDF_LAYOUT="biped45" ;;
  biped_v3_arm.urdf) URDF_LAYOUT="biped52" ;;
  biped_v3_arm_s56.urdf) URDF_LAYOUT="biped56" ;;
  biped_v3_arm_s62.urdf) URDF_LAYOUT="wheel62" ;;
esac
set_inferred_layout "${URDF_LAYOUT}" "urdf=${META_URDF}"

if [[ -n "${INFERRED_LAYOUT}" ]]; then
  EXPECTED_FK="waist_yaw_link"
  [[ "${INFERRED_LAYOUT}" == "biped45" ]] && EXPECTED_FK="base_link"
  if [[ -n "${META_FK}" && "${META_FK}" != "${EXPECTED_FK}" ]]; then
    die "capture metadata 自相矛盾: layout=${INFERRED_LAYOUT} 期望 fk_root=${EXPECTED_FK}，实际 ${META_FK}"
  fi
elif [[ -n "${META_FK}" && "${META_FK}" != "base_link" && "${META_FK}" != "waist_yaw_link" ]]; then
  die "capture meta.fk_root 不支持: ${META_FK}"
fi

if [[ -n "${LAYOUT}" && -n "${INFERRED_LAYOUT}" && "${LAYOUT}" != "${INFERRED_LAYOUT}" ]]; then
  die "--layout=${LAYOUT} 与 capture 推断机型 ${INFERRED_LAYOUT} 不一致"
fi
[[ -n "${LAYOUT}" ]] || LAYOUT="${INFERRED_LAYOUT}"

# metadata 不足时才回退 ROBOT_VERSION；未知机型不再静默选择 wheel62。
if [[ -z "${LAYOUT}" ]]; then
  case "${ROBOT_VERSION:-}" in
    45) LAYOUT="biped45" ;;
    52) LAYOUT="biped52" ;;
    56) LAYOUT="biped56" ;;
    62|63) LAYOUT="wheel62" ;;
    "") die "capture 无法推断机型，且未设置 ROBOT_VERSION；请显式传 --layout" ;;
    *) die "不支持的 ROBOT_VERSION=${ROBOT_VERSION}；请显式传 --layout" ;;
  esac
fi
case "${LAYOUT}" in
  biped45|biped52|biped56|wheel62) ;;
  *) die "无效 --layout: ${LAYOUT}（仅 biped45|biped52|biped56|wheel62）" ;;
esac
echo "[INFO] robot_layout: ${LAYOUT}"

# 同一输出目录只允许一个优化任务；结果先写临时目录，通过校验后再替换正式文件。
mkdir -p -- "${OUTPUT_DIR}"
command -v flock >/dev/null 2>&1 || die "缺少 flock 命令（util-linux），无法安全锁定输出目录"
exec 9>"${OUTPUT_DIR}/.mocap_optimize.lock"
flock -n 9 || die "已有 mocap 优化任务正在写 ${OUTPUT_DIR}"
TMP_OUTPUT_DIR="$(mktemp -d "${OUTPUT_DIR}/.mocap_optimize.XXXXXX")"

# 3) source ROS 环境
[[ -f /opt/ros/noetic/setup.bash ]] || die "未找到 ROS Noetic: /opt/ros/noetic/setup.bash"
source /opt/ros/noetic/setup.bash
[[ -f "${WS_DIR}/devel/setup.bash" ]] || die "工作空间尚未编译: ${WS_DIR}/devel/setup.bash 不存在"
source "${WS_DIR}/devel/setup.bash"

# 4) 起 roscore（如未运行）
if ! timeout 3 rostopic list >/dev/null 2>&1; then
  echo "[INFO] 未检测到 roscore，自动启动..."
  roscore >/dev/null 2>&1 &
  sleep 4
  timeout 3 rostopic list >/dev/null 2>&1 || die "roscore 启动失败"
fi

# 5) 跑优化
echo "[INFO] 运行 optimize_mocap ..."
if ! roslaunch "${SCRIPT_DIR}/mocap_optimize.launch" \
    "capture:=${CAPTURE}" \
    "robot_layout:=${LAYOUT}" \
    "output_dir:=${TMP_OUTPUT_DIR}"; then
  die "optimize_mocap 失败。请检查上方 FATAL/ERROR 日志"
fi

TMP_CALIBRATION="${TMP_OUTPUT_DIR}/calibration.yaml"
[[ -f "${TMP_CALIBRATION}" ]] || die "优化进程成功退出，但未生成 calibration.yaml"

# 写零前的最低限度格式检查：双臂 14 关节必须齐全，且全部为有限数值。
python3 - "${TMP_CALIBRATION}" <<'PY' || die "calibration.yaml 内容校验失败"
import math
import sys
import yaml

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream)
if not isinstance(data, dict):
    raise SystemExit("calibration.yaml 顶层不是字典")
expected = {
    *(f"zarm_l{i}_joint" for i in range(1, 8)),
    *(f"zarm_r{i}_joint" for i in range(1, 8)),
}
missing = sorted(expected - set(data))
if missing:
    raise SystemExit(f"缺少关节: {missing}")
for joint in sorted(expected):
    try:
        value = float(data[joint])
    except (TypeError, ValueError):
        raise SystemExit(f"{joint} 不是数值: {data[joint]!r}")
    if not math.isfinite(value):
        raise SystemExit(f"{joint} 不是有限数值: {value}")
PY

if [[ -f "${FINAL_CALIBRATION}" ]]; then
  BACKUP="${FINAL_CALIBRATION}.bak.$(date +%Y%m%d-%H%M%S-%N)"
  cp -p -- "${FINAL_CALIBRATION}" "${BACKUP}"
  echo "[INFO] 旧结果备份: ${BACKUP}"
fi
mv -f -- "${TMP_CALIBRATION}" "${FINAL_CALIBRATION}"
echo "[INFO] 优化完成，输出在 ${FINAL_CALIBRATION}"
