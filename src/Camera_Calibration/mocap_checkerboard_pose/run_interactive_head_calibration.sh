#!/usr/bin/env bash
# 头部联合标定：动捕棋盘位姿 -> 更新机型 URDF -> 头部棋盘采集 -> 优化/画图 -> 结果验证 -> 写头部零点。
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAMERA_CAL_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${CAMERA_CAL_DIR}/../.." && pwd)"
CHESSBOARD_RUNNER="${CAMERA_CAL_DIR}/run_chessboard_calibration.sh"
WRITE_ZERO="${CAMERA_CAL_DIR}/mocap_joint_calib/write_zero.py"
MOCAP_CONFIG="${SCRIPT_DIR}/config/bodies.yaml"
BOARD_OUTPUT_ROOT="${SCRIPT_DIR}/output"
HEAD_CSV_ROOT="${CAMERA_CAL_DIR}/output_csv/kuavo_head_sessions"
HEAD_RESULT_DIR="${CAMERA_CAL_DIR}/output/kuavo_head"
HEAD_CALIBRATION_YAML="${HEAD_RESULT_DIR}/calibration.yaml"
HEAD_OPTIMIZATION_MARKER="${HEAD_RESULT_DIR}/.head_calibration_session.json"
HEAD_TEST_TEACH_JSON="${CAMERA_CAL_DIR}/teach_capture_output/teach_head_joint_test.json"
HEAD_TEST_RESULT_DIR="${CAMERA_CAL_DIR}/output/kuavo_head_test"
HEAD_TEST_CSV_DIR="${CAMERA_CAL_DIR}/output_csv/kuavo_head_test"

ROBOT_VERSION_ARG="auto"
MOCAP_SOURCE_ARG=""
MODE=""
HEAD_CSV_DIR_ARG=""
HEAD_WRITE_DONE=false
TEST_CONFIRMED=false
WRITE_ALREADY_PREVIEWED=false

die() { echo "[ERROR] $*" >&2; exit 1; }
warn() { echo "[WARN] $*" >&2; }
info() { echo "[INFO] $*"; }

run_cmd() {
  printf '[RUN]'
  printf ' %q' "$@"
  printf '\n'
  "$@"
}

confirm() {
  local answer=""
  read -r -p "$1 [y/N]: " answer
  [[ "${answer}" =~ ^[Yy]$ ]]
}

usage() {
  cat <<'EOF'
用法: run_interactive_head_calibration.sh [选项]

选项:
  --robot-version auto|45|52|56|62|63
  --mocap-source motive|qingtong
  --mode full|board|capture|optimize|verify|dry-run|write|test
  --head-csv-dir DIR   capture 的输出目录，或 optimize 使用的 CSV 目录
  --test-confirmed     仅配合 --mode test；表示上层流程已完成实机安全确认
  --already-previewed  仅配合 --mode write；表示本轮已经完成并检查过 dry-run

不传 --mode 时进入交互菜单。full 的顺序为：
  动捕棋盘 -> 写 checkerboard_joint -> 头部相机采集 -> 优化并画图 -> 结果验证 -> 写零 dry-run
  -> 可选正式写零 -> 重启运控 -> 独立测试姿态采集并画验证图
verify 只打印头部标定结果验证（bias + 标定前后误差 + PASS/FAIL 判定）。
test 在确认头部零点已通过重启生效后，使用独立测试姿态采集并输出验证图。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-version) ROBOT_VERSION_ARG="${2:-}"; shift 2 ;;
    --mocap-source) MOCAP_SOURCE_ARG="${2:-}"; shift 2 ;;
    --mode) MODE="${2:-}"; shift 2 ;;
    --head-csv-dir) HEAD_CSV_DIR_ARG="${2:-}"; shift 2 ;;
    --test-confirmed) TEST_CONFIRMED=true; shift ;;
    --already-previewed) WRITE_ALREADY_PREVIEWED=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "未知参数: $1" ;;
  esac
done

case "${ROBOT_VERSION_ARG}" in auto|45|52|56|62|63) ;; *) die "不支持的机器人版本: ${ROBOT_VERSION_ARG}" ;; esac
case "${MOCAP_SOURCE_ARG}" in ""|motive|qingtong) ;; *) die "动捕源仅支持 motive|qingtong" ;; esac
case "${MODE}" in ""|full|board|capture|optimize|verify|dry-run|write|test) ;; *) die "未知 mode: ${MODE}" ;; esac
if [[ "${TEST_CONFIRMED}" == true && "${MODE}" != "test" ]]; then
  die "--test-confirmed 只能与 --mode test 一起使用"
fi
if [[ "${WRITE_ALREADY_PREVIEWED}" == true && "${MODE}" != "write" ]]; then
  die "--already-previewed 只能与 --mode write 一起使用"
fi

source_ros_environment() {
  [[ -f /opt/ros/noetic/setup.bash ]] || die "未找到 /opt/ros/noetic/setup.bash"
  [[ -f "${WS_DIR}/devel/setup.bash" ]] || die "未找到 ${WS_DIR}/devel/setup.bash，请先编译工作空间"
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  # shellcheck disable=SC1091
  source "${WS_DIR}/devel/setup.bash"
  set -u
}

resolve_robot_version() {
  local selected=""
  local ros_version=""
  local env_version="${ROBOT_VERSION:-}"
  ros_version="$(timeout 2 rosparam get /robot_version 2>/dev/null | tr -d '[:space:]' | tr -d "'\"" || true)"
  case "${ros_version}" in 45|52|56|62|63) ;; *) ros_version="" ;; esac
  case "${env_version}" in 45|52|56|62|63) ;; *) env_version="" ;; esac
  if [[ "${ROBOT_VERSION_ARG}" != "auto" ]]; then
    selected="${ROBOT_VERSION_ARG}"
    if [[ -n "${ros_version}" && "${ros_version}" != "${selected}" ]]; then
      die "指定机型 ${selected} 与运控 /robot_version=${ros_version} 冲突"
    fi
    if [[ -n "${env_version}" && "${env_version}" != "${selected}" ]]; then
      die "指定机型 ${selected} 与环境变量 ROBOT_VERSION=${env_version} 冲突"
    fi
  else
    if [[ -n "${ros_version}" && -n "${env_version}" && "${ros_version}" != "${env_version}" ]]; then
      die "机型信息冲突：ROBOT_VERSION=${env_version}，运控 /robot_version=${ros_version}"
    fi
    selected="${ros_version:-${env_version}}"
    if [[ -z "${selected}" ]]; then
      read -r -p "机器人版本 [45/52/56/62/63]: " selected
    fi
  fi
  case "${selected}" in 45|52|56|62|63) ;; *) die "无法确定受支持的机器人版本: ${selected}" ;; esac
  ROBOT_VERSION="${selected}"
  export ROBOT_VERSION
}

resolve_mocap_source() {
  if [[ -n "${MOCAP_SOURCE_ARG}" ]]; then
    MOCAP_SOURCE="${MOCAP_SOURCE_ARG}"
    return
  fi
  local selected=""
  echo "动捕系统："
  echo "  1) Motive / OptiTrack"
  echo "  2) 青瞳 / VRPN"
  read -r -p "选择 [1/2，默认 1]: " selected
  case "${selected:-1}" in
    1) MOCAP_SOURCE="motive" ;;
    2) MOCAP_SOURCE="qingtong" ;;
    *) die "无效动捕选择: ${selected}" ;;
  esac
}

resolve_robot_paths() {
  case "${ROBOT_VERSION}" in
    45) LAYOUT="biped45"; URDF_PATH="${CAMERA_CAL_DIR}/biped_v3_arm_s45.urdf" ;;
    52) LAYOUT="biped52"; URDF_PATH="${CAMERA_CAL_DIR}/biped_v3_arm.urdf" ;;
    56) LAYOUT="biped56"; URDF_PATH="${CAMERA_CAL_DIR}/biped_v3_arm_s56.urdf" ;;
    62|63) LAYOUT="wheel62"; URDF_PATH="${CAMERA_CAL_DIR}/biped_v3_arm_s62.urdf" ;;
  esac
  [[ -f "${URDF_PATH}" ]] || die "机型 URDF 不存在: ${URDF_PATH}"
  [[ -f "${MOCAP_CONFIG}" ]] || die "动捕棋盘配置不存在: ${MOCAP_CONFIG}"
  [[ -f "${CHESSBOARD_RUNNER}" ]] || die "棋盘标定入口不存在: ${CHESSBOARD_RUNNER}"
}

print_context() {
  echo
  echo "=========================================="
  echo "  头部联合标定"
  echo "=========================================="
  echo "  workspace : ${WS_DIR}"
  echo "  version   : ${ROBOT_VERSION}"
  echo "  layout    : ${LAYOUT}"
  echo "  mocap     : ${MOCAP_SOURCE}"
  echo "  board FK  : zarm_l1_ref_link -> checkerboard_link"
  echo "  urdf      : ${URDF_PATH}"
  echo "------------------------------------------"
}

check_checkerboard_topics() {
  timeout 3 rostopic list >/dev/null 2>&1 || die "无法连接 ROS master，请先启动 ROS 和动捕接收节点"
  local topics=()
  mapfile -t topics < <(python3 - "${MOCAP_CONFIG}" "${MOCAP_SOURCE}" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
source = sys.argv[2]
for body in cfg.get("bodies", []):
    name = body.get("name")
    topic = body.get("topic")
    if not topic:
        topic = f"/vrpn_client_node/{name}/pose" if source == "qingtong" else f"/{name}_pose"
    print(topic)
PY
  )
  local topic=""
  for topic in "${topics[@]}"; do
    if ! timeout 5 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
      die "动捕刚体话题无有效消息: ${topic}"
    fi
    info "动捕话题有效: ${topic}"
  done
}

validate_board_config() {
  python3 - "${MOCAP_CONFIG}" "${URDF_PATH}" "${ROBOT_VERSION}" <<'PY'
import sys, xml.etree.ElementTree as ET, yaml
cfg_path, urdf_path, robot_version = sys.argv[1:]
cfg = yaml.safe_load(open(cfg_path, encoding="utf-8"))
shoulder_profile = "s45" if robot_version == "45" else "default"
shoulder_offset = (cfg.get("left_shoulder_fixtures") or {}).get(shoulder_profile)
if not isinstance(shoulder_offset, list) or len(shoulder_offset) != 3:
    raise SystemExit(f"bodies.yaml 缺少 left_shoulder_fixtures.{shoulder_profile}")
bodies = {b.get("name"): b for b in cfg.get("bodies", [])}
for name in ("checkerboard", "l_shoulder", "torso"):
    if name not in bodies:
        raise SystemExit(f"bodies.yaml 缺少 {name}")
    offset = bodies[name].get("link_offset_mm")
    if not isinstance(offset, list) or len(offset) != 3:
        raise SystemExit(f"{name}.link_offset_mm 必须包含3个数")
root = ET.parse(urdf_path).getroot()
links = {node.get("name") for node in root.findall("link")}
if not {"zarm_l1_ref_link", "checkerboard_link"} <= links:
    raise SystemExit("URDF 缺少 zarm_l1_ref_link/checkerboard_link")
joint = next((j for j in root.findall("joint") if j.get("name") == "checkerboard_joint"), None)
if joint is None:
    raise SystemExit("URDF 缺少 checkerboard_joint")
print("[CHECK] 棋盘位姿将写为 zarm_l1_ref_link -> checkerboard_link")
print(f"[CHECK] left_shoulder profile={shoulder_profile}, offset_mm={shoulder_offset}")
for name in ("checkerboard", "l_shoulder", "torso"):
    print(f"[CHECK] {name}.link_offset_mm={bodies[name]['link_offset_mm']}")
PY
}

validate_board_result() {
  python3 - "$1" "${ROBOT_VERSION}" <<'PY'
import json, sys
data = json.load(open(sys.argv[1], encoding="utf-8"))
expected_version = sys.argv[2]
if str(data.get("robot_version", "")) != expected_version:
    raise SystemExit(
        f"动捕结果机型不匹配: JSON={data.get('robot_version')}, 当前={expected_version}"
    )
shoulder = data.get("left_shoulder_fixture") or {}
expected_profile = "s45" if expected_version == "45" else "default"
if shoulder.get("profile") != expected_profile:
    raise SystemExit(
        f"左肩工装不匹配: JSON={shoulder.get('profile')}, 期望={expected_profile}"
    )
pose = data.get("checkerboard_in_l_shoulder")
if not pose:
    raise SystemExit("动捕结果缺少 checkerboard_in_l_shoulder")
statistics = data.get("statistics") or {}
missing = sorted({"l_shoulder", "torso"} - set(statistics))
if missing:
    raise SystemExit(f"动捕结果缺少静止性统计: {missing}")
bad = []
for parent, stats in statistics.items():
    pos = float(stats.get("pos_std_mm", 1e9))
    rot = float(stats.get("rot_std_deg", 1e9))
    print(f"[CHECK] {parent}: pos_std={pos:.3f} mm, rot_std={rot:.4f} deg")
    if pos >= 1.0 or rot >= 0.1:
        bad.append(parent)
if bad:
    raise SystemExit(f"动捕静止性不达标: {bad}（要求 pos_std<1mm 且 rot_std<0.1deg）")
print(f"[CHECK] checkerboard_in_l_shoulder xyz(m)={pose['xyz']} rpy(rad)={pose['rpy']}")
print(f"[CHECK] left_shoulder offset_mm={shoulder.get('link_offset_mm')}")
PY
}

capture_and_apply_board() {
  validate_board_config
  check_checkerboard_topics
  echo
  warn "采集期间机器人、棋盘和三个动捕刚体必须完全静止。"
  warn "请确认刚体坐标轴已分别对齐 checkerboard_link、torso link、zarm_l1_ref_link。"
  confirm "开始采集棋盘位姿并更新当前机型 URDF" || return 1

  local stamp=""
  stamp="$(date +%Y%m%d_%H%M%S)"
  BOARD_SESSION_DIR="${BOARD_OUTPUT_ROOT}/${ROBOT_VERSION}_${stamp}"
  BOARD_CSV="${BOARD_SESSION_DIR}/mocap_poses.csv"
  BOARD_JSON="${BOARD_SESSION_DIR}/checkerboard_relative_poses.json"
  mkdir -p "${BOARD_SESSION_DIR}"

  run_cmd python3 "${SCRIPT_DIR}/record_mocap_poses.py" \
    --config "${MOCAP_CONFIG}" --mocap-source "${MOCAP_SOURCE}" \
    --duration 10 --warmup 2 --wait_timeout 30 --output "${BOARD_CSV}"
  run_cmd python3 "${SCRIPT_DIR}/process_mocap_poses.py" \
    --config "${MOCAP_CONFIG}" --robot-version "${ROBOT_VERSION}" \
    --input "${BOARD_CSV}" --output "${BOARD_JSON}"
  validate_board_result "${BOARD_JSON}"
  run_cmd python3 "${SCRIPT_DIR}/apply_checkerboard_to_urdf.py" \
    --json "${BOARD_JSON}" --urdf "${URDF_PATH}" --mode left_shoulder
  info "棋盘 URDF 已更新；从现在到头部采集结束，不要移动棋盘或机器人底座。"
}

new_head_csv_dir() {
  local stamp="$(date +%Y%m%d_%H%M%S)"
  echo "${HEAD_CSV_ROOT}/${ROBOT_VERSION}/session_${stamp}"
}

latest_head_csv_dir() {
  python3 - "${HEAD_CSV_ROOT}/${ROBOT_VERSION}" "${ROBOT_VERSION}" "${LAYOUT}" <<'PY'
import json, pathlib, sys
root, version, layout = pathlib.Path(sys.argv[1]), sys.argv[2], sys.argv[3]
matches = []
for marker in root.glob("session_*/.head_capture_session.json"):
    try:
        data = json.loads(marker.read_text(encoding="utf-8"))
        if str(data.get("robot_version")) != version or data.get("layout") != layout:
            continue
        if list(marker.parent.glob("*.csv")):
            matches.append(marker.parent)
    except Exception:
        pass
if matches:
    print(max(matches, key=lambda p: p.stat().st_mtime))
PY
}

record_head_capture_marker() {
  python3 - "${HEAD_CSV_DIR}" "${ROBOT_VERSION}" "${LAYOUT}" "${URDF_PATH}" "${BOARD_JSON:-}" <<'PY'
import datetime, hashlib, json, pathlib, sys
csv_dir, version, layout, urdf, board_json = sys.argv[1:]
urdf_path = pathlib.Path(urdf).resolve()
payload = {
    "created_at": datetime.datetime.now().isoformat(),
    "robot_version": version,
    "layout": layout,
    "csv_dir": str(pathlib.Path(csv_dir).resolve()),
    "urdf": str(urdf_path),
    "urdf_sha256": hashlib.sha256(urdf_path.read_bytes()).hexdigest(),
    "checkerboard_json": str(pathlib.Path(board_json).resolve()) if board_json else None,
    "checkerboard_parent": "zarm_l1_ref_link",
}
path = pathlib.Path(csv_dir) / ".head_capture_session.json"
path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(f"[INFO] 已记录头部采集来源: {path}")
PY
}

capture_head_data() {
  HEAD_CSV_DIR="${HEAD_CSV_DIR_ARG:-$(new_head_csv_dir)}"
  mkdir -p "${HEAD_CSV_DIR}"
  echo
  warn "即将驱动头部逐姿态采集棋盘图像；人员远离机构，确认急停可用。"
  warn "棋盘和机器人底座在整轮采集期间不能移动。"
  confirm "开始头部棋盘采集" || return 1
  run_cmd bash "${CHESSBOARD_RUNNER}" capture --demo head \
    --robot_layout "${LAYOUT}" --out_dir "${HEAD_CSV_DIR}"
  record_head_capture_marker
}

require_head_csv_dir() {
  if [[ -n "${HEAD_CSV_DIR_ARG}" ]]; then
    HEAD_CSV_DIR="${HEAD_CSV_DIR_ARG}"
  elif [[ -z "${HEAD_CSV_DIR:-}" ]]; then
    HEAD_CSV_DIR="$(latest_head_csv_dir)"
  fi
  [[ -n "${HEAD_CSV_DIR:-}" && -d "${HEAD_CSV_DIR}" ]] || die "未找到当前机型的头部采集目录"
  compgen -G "${HEAD_CSV_DIR}/*.csv" >/dev/null || die "头部采集目录没有 CSV: ${HEAD_CSV_DIR}"
  python3 - "${HEAD_CSV_DIR}/.head_capture_session.json" "${ROBOT_VERSION}" \
    "${LAYOUT}" "${URDF_PATH}" <<'PY'
import hashlib, json, pathlib, sys
marker, version, layout, urdf = pathlib.Path(sys.argv[1]), sys.argv[2], sys.argv[3], pathlib.Path(sys.argv[4]).resolve()
if not marker.is_file():
    raise SystemExit(f"缺少头部采集来源标记: {marker}")
data = json.loads(marker.read_text(encoding="utf-8"))
if str(data.get("robot_version")) != version or data.get("layout") != layout:
    raise SystemExit("头部 CSV 与当前机型不匹配")
if pathlib.Path(data.get("urdf", "")).resolve() != urdf:
    raise SystemExit("头部 CSV 记录的 URDF 路径与当前机型不匹配")
actual = hashlib.sha256(urdf.read_bytes()).hexdigest()
if actual != data.get("urdf_sha256"):
    raise SystemExit("头部采集后机型 URDF 已变化；禁止用不同棋盘位姿优化，请重新采集")
if data.get("checkerboard_parent") != "zarm_l1_ref_link":
    raise SystemExit("头部 CSV 的棋盘父坐标系不是 zarm_l1_ref_link")
print(f"[CHECK] 头部采集来源有效: version={version}, layout={layout}, urdf={urdf}")
PY
  info "使用头部 CSV: ${HEAD_CSV_DIR}"
}

record_head_optimization_marker() {
  python3 - "${HEAD_OPTIMIZATION_MARKER}" "${HEAD_CALIBRATION_YAML}" "${HEAD_CSV_DIR}" \
    "${ROBOT_VERSION}" "${LAYOUT}" <<'PY'
import datetime, hashlib, json, math, pathlib, sys, yaml
marker, calibration, csv_dir, version, layout = sys.argv[1:]
calibration = pathlib.Path(calibration).resolve()
if not calibration.is_file():
    raise SystemExit(f"头部优化结果不存在: {calibration}")
result = yaml.safe_load(calibration.read_text(encoding="utf-8")) or {}
for joint in ("zhead_1_joint", "zhead_2_joint"):
    try:
        value = float(result[joint])
    except (KeyError, TypeError, ValueError):
        raise SystemExit(f"头部优化结果缺少有效 {joint}")
    if not math.isfinite(value):
        raise SystemExit(f"头部优化结果 {joint} 不是有限数值")
payload = {
    "created_at": datetime.datetime.now().isoformat(),
    "robot_version": version,
    "layout": layout,
    "csv_dir": str(pathlib.Path(csv_dir).resolve()),
    "calibration": str(calibration),
    "calibration_sha256": hashlib.sha256(calibration.read_bytes()).hexdigest(),
}
path = pathlib.Path(marker)
path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(f"[INFO] 已记录头部优化来源: {path}")
PY
}

verify_head_optimization() {
  python3 - "${HEAD_OPTIMIZATION_MARKER}" "${HEAD_CALIBRATION_YAML}" \
    "${ROBOT_VERSION}" "${LAYOUT}" <<'PY'
import hashlib, json, pathlib, sys
marker, calibration, version, layout = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2]).resolve(), sys.argv[3], sys.argv[4]
if not marker.is_file():
    raise SystemExit("缺少本次头部优化来源标记，请先执行头部 optimize")
data = json.loads(marker.read_text(encoding="utf-8"))
if str(data.get("robot_version")) != version or data.get("layout") != layout:
    raise SystemExit("头部优化结果与当前机型不匹配")
if pathlib.Path(data.get("calibration", "")).resolve() != calibration or not calibration.is_file():
    raise SystemExit("头部 calibration.yaml 路径无效")
digest = hashlib.sha256(calibration.read_bytes()).hexdigest()
if digest != data.get("calibration_sha256"):
    raise SystemExit("头部 calibration.yaml 在优化后已被替换或修改，请重新优化")
csv_dir = pathlib.Path(data.get("csv_dir", ""))
if not csv_dir.is_dir() or not list(csv_dir.glob("*.csv")):
    raise SystemExit(f"头部优化对应 CSV 已不存在: {csv_dir}")
print(f"[CHECK] 头部优化来源有效: version={version}, layout={layout}, csv={csv_dir}")
PY
}

# 防重复写零：检查当前头部 calibration 是否已写入过零点。
# write_head_zero 写零成功后会 mv marker 成 .head_calibration_session.json.written.*，
# 其中含 calibration_sha256；同一结果（同 sha）再次写零会从当前零点再扣一次 bias，禁止。
check_head_calibration_not_written() {
  [[ -f "${HEAD_CALIBRATION_YAML}" ]] || die "头部优化结果不存在: ${HEAD_CALIBRATION_YAML}"
  python3 - "${HEAD_OPTIMIZATION_MARKER}" "${HEAD_CALIBRATION_YAML}" "${ROBOT_VERSION}" "${LAYOUT}" <<'PY'
import glob
import hashlib
import json
import pathlib
import sys

marker_base, calibration, version, layout = sys.argv[1:]
cal_path = pathlib.Path(calibration)
digest = hashlib.sha256(cal_path.read_bytes()).hexdigest()
for written in sorted(glob.glob(marker_base + ".written.*")):
    try:
        data = json.loads(pathlib.Path(written).read_text(encoding="utf-8"))
    except Exception:
        continue
    if (str(data.get("robot_version")) == version and data.get("layout") == layout
            and str(data.get("calibration_sha256")) == digest):
        raise SystemExit(
            f"该头部 calibration 已写零过（记录: {pathlib.Path(written).name}），禁止重复写入。\n"
            f"同一结果重复写零会让头部零点偏 2×bias；如需重新标定，请重新采集后再优化。"
        )
PY
}

print_head_result_verification() {
  [[ -f "${HEAD_CALIBRATION_YAML}" ]] || die "头部优化结果不存在: ${HEAD_CALIBRATION_YAML}"
  verify_head_optimization
  python3 - "${HEAD_CALIBRATION_YAML}" \
    "${HEAD_RESULT_DIR}/optimization_metrics.md" \
    "${HEAD_BIAS_MAX_RAD:-0.20}" "${HEAD_BIAS_HARD_MAX_RAD:-0.785}" \
    "${HEAD_POS_ERR_OK_M:-0.05}" "${HEAD_POS_ERR_WARN_M:-0.10}" \
    "${HEAD_ROT_ERR_OK_DEG:-3.0}" "${HEAD_ROT_ERR_WARN_DEG:-10.0}" <<'PY'
import math
import pathlib
import re
import sys

import yaml

cal_path, metrics_path = sys.argv[1], sys.argv[2]
bias_max = float(sys.argv[3])
bias_hard = float(sys.argv[4])
pos_ok = float(sys.argv[5])
pos_warn = float(sys.argv[6])
rot_ok = float(sys.argv[7])
rot_warn = float(sys.argv[8])

result = yaml.safe_load(open(cal_path, encoding="utf-8")) or {}
fails = 0
warns = 0

print()
print("==================================================")
print("  头部标定结果验证")
print("==================================================")

# --- 1) 头部关节 bias ---
print("-- 头部零点 bias（calibration.yaml）--")
for joint in ("zhead_1_joint", "zhead_2_joint"):
    raw = result.get(joint)
    if raw is None:
        print(f"  {joint}: 缺失 (FAIL)")
        fails += 1
        continue
    try:
        value = float(raw)
    except (TypeError, ValueError):
        print(f"  {joint}: 非数值 {raw!r} (FAIL)")
        fails += 1
        continue
    if not math.isfinite(value):
        print(f"  {joint}: 非有限 (FAIL)")
        fails += 1
        continue
    if abs(value) > bias_hard:
        tag = "FAIL"
        fails += 1
    elif abs(value) > bias_max:
        tag = "WARN"
        warns += 1
    else:
        tag = "OK"
    print(f"  {joint} = {value:+.6f} rad ({math.degrees(value):+.3f} deg)  [{tag}]")

# --- 2) 相机外参（若该轮优化了 camera_base）---
camera = {
    key: result[key]
    for key in (
        "camera_base_x", "camera_base_y", "camera_base_z",
        "camera_base_a", "camera_base_b", "camera_base_c",
    )
    if key in result
}
if camera:
    print("-- 相机外参 camera_base --")
    print("  " + ", ".join(f"{key}={value:g}" for key, value in camera.items()))

# --- 3) 标定前后误差摘要（解析 optimization_metrics.md [summary]）---
print("-- 标定前后误差（optimization_metrics.md [summary]）--")
summary = re.compile(
    r"\[summary\].*?position: mean ([0-9.eE+-]+) -> ([0-9.eE+-]+) m, "
    r"drop [0-9.eE+-]+ m \(([0-9.eE+-]+)%\).*?"
    r"rotation: mean ([0-9.eE+-]+) -> ([0-9.eE+-]+) deg, "
    r"drop [0-9.eE+-]+ deg \(([0-9.eE+-]+)%\)",
    re.S,
)
if not pathlib.Path(metrics_path).is_file():
    print(f"  {metrics_path} 不存在，无法读取误差摘要")
    print("  整体判定: FAIL（缺少验证依据，请检查优化/画图是否成功）")
else:
    text = pathlib.Path(metrics_path).read_text(encoding="utf-8")
    match = None
    for found in summary.finditer(text):
        match = found
    if match is None:
        print("  未找到 [summary] 误差摘要（可能画图失败），请检查优化报告")
        print("  整体判定: FAIL（缺少验证依据）")
    else:
        pos_pre, pos_post, pos_drop = (float(match.group(i)) for i in (1, 2, 3))
        rot_pre, rot_post, rot_drop = (float(match.group(i)) for i in (4, 5, 6))
        print(f"  position: mean {pos_pre:.4f} -> {pos_post:.4f} m  (drop {pos_drop:.2f}%)")
        print(f"  rotation: mean {rot_pre:.4f} -> {rot_post:.4f} deg  (drop {rot_drop:.2f}%)")

        def grade(value, ok, warn_thr):
            if value <= ok:
                return "OK"
            if value <= warn_thr:
                return "WARN"
            return "FAIL"

        pos_grade = grade(pos_post, pos_ok, pos_warn)
        rot_grade = grade(rot_post, rot_ok, rot_warn)
        print(f"  [{pos_grade}] 位置误差(标定后) = {pos_post:.4f} m  (OK<={pos_ok:g}, WARN<={pos_warn:g})")
        print(f"  [{rot_grade}] 旋转误差(标定后) = {rot_post:.4f} deg  (OK<={rot_ok:g}, WARN<={rot_warn:g})")
        if pos_grade == "FAIL":
            fails += 1
        elif pos_grade == "WARN":
            warns += 1
        if rot_grade == "FAIL":
            fails += 1
        elif rot_grade == "WARN":
            warns += 1
        if pos_drop <= 0:
            print(f"  [FAIL] 位置误差未下降 (drop={pos_drop:+.2f}%)")
            fails += 1
        if rot_drop <= 0:
            print(f"  [FAIL] 旋转误差未下降 (drop={rot_drop:+.2f}%)")
            fails += 1

print("-" * 46)
if fails > 0:
    print(f"  整体判定: FAIL（{fails} 项不达标，{warns} 项警告）")
elif warns > 0:
    print(f"  整体判定: PASS（{warns} 项警告，建议人工复核误差图）")
else:
    print("  整体判定: PASS")
print()
PY
}

optimize_head() {
  require_head_csv_dir
  run_cmd bash "${CHESSBOARD_RUNNER}" optimize --demo head \
    --robot_layout "${LAYOUT}" --out_dir "${HEAD_CSV_DIR}"
  record_head_optimization_marker
  print_head_result_verification
  info "头部优化报告与图片: ${HEAD_RESULT_DIR}"
}

dry_run_head_zero() {
  verify_head_optimization
  run_cmd python3 "${WRITE_ZERO}" --robot-version "${ROBOT_VERSION}" \
    --parts head --calibration-yaml "${HEAD_CALIBRATION_YAML}" --dry-run
}

write_head_zero() {
  HEAD_WRITE_DONE=false
  # 已写零过或用户取消时保持 HEAD_WRITE_DONE=false 并正常返回；
  # 真正的写入命令失败则由 set -e 终止，不能被当成“取消”吞掉。
  if ! check_head_calibration_not_written; then
    info "已取消正式写零（该头部 calibration 已写零过，禁止重复写入）"
    return 0
  fi
  if [[ "${1:-}" != "--already-previewed" ]]; then
    dry_run_head_zero
  fi
  echo
  warn "正式写入会修改当前机器人的头部零点文件；同一结果禁止重复写入。"
  local token=""
  read -r -p "确认 dry-run 无误后输入 WRITE 继续: " token
  [[ "${token}" == "WRITE" ]] || { info "已取消正式写入"; return 0; }
  run_cmd python3 "${WRITE_ZERO}" --robot-version "${ROBOT_VERSION}" \
    --parts head --calibration-yaml "${HEAD_CALIBRATION_YAML}"
  mv -- "${HEAD_OPTIMIZATION_MARKER}" \
    "${HEAD_OPTIMIZATION_MARKER}.written.$(date +%Y%m%d-%H%M%S)"
  HEAD_WRITE_DONE=true
  warn "头部零点已写入；停止并重新启动机器人控制程序后才会生效。"
}

run_head_validation() {
  [[ -f "${HEAD_TEST_TEACH_JSON}" ]] || die \
    "缺少头部独立测试姿态: ${HEAD_TEST_TEACH_JSON}；请先用 teach_joint_capture.py 生成用于测试的姿态"
  warn "头部独立验证会驱动头部。请确认新零点已通过重启运控生效，且棋盘和人员处于安全位置。"
  if [[ "${1:-}" != "--confirmed" && "${TEST_CONFIRMED}" != true ]] \
      && ! confirm "开始头部写零后独立验证"; then
    info "已取消头部独立验证；不会读取或绘制目录中的旧测试数据"
    return 0
  fi
  run_cmd bash "${CHESSBOARD_RUNNER}" test --demo head \
    --robot_layout "${LAYOUT}"
  info "头部独立验证 CSV: ${HEAD_TEST_CSV_DIR}"
  info "头部独立验证指标与图片: ${HEAD_TEST_RESULT_DIR}"
}

full_workflow() {
  capture_and_apply_board || { info "已取消或棋盘动捕未通过，流程停止"; return; }
  capture_head_data || { info "已取消头部采集，流程停止"; return; }
  optimize_head
  dry_run_head_zero
  echo
  info "请检查 ${HEAD_RESULT_DIR} 下的 optimization_metrics.md 和两张误差图。"
  if confirm "优化结果合理，进入正式写头部零点确认"; then
    write_head_zero --already-previewed
    if [[ "${HEAD_WRITE_DONE}" != true ]]; then
      info "未完成正式写零，完整流程停止；不会执行重启确认和独立验证"
      return 0
    fi
    read -r -p "请停止并重新启动机器人控制程序，确认头部零点生效后按 Enter 继续..." _
    run_head_validation
  else
    info "已停在安全阶段：没有写入头部零点"
  fi
}

show_menu() {
  echo
  echo "选择操作："
  echo "  1) 头部完整流程（推荐）"
  echo "  2) 动捕棋盘并更新 URDF"
  echo "  3) 头部棋盘数据采集"
  echo "  4) 优化当前/最新头部 CSV 并画图（含结果验证）"
  echo "  5) 头部写零 dry-run"
  echo "  6) 正式写头部零点"
  echo "  7) 打印头部标定结果验证"
  echo "  8) 头部写零后独立验证（测试姿态采集并画图）"
  echo "  0) 返回"
}

run_selected_mode() {
  case "$1" in
    full) full_workflow ;;
    # 非交互调用必须把“取消/失败”传给上层流程，不能伪装成成功。
    board) capture_and_apply_board ;;
    capture) capture_head_data ;;
    optimize) optimize_head ;;
    verify) print_head_result_verification ;;
    dry-run) dry_run_head_zero ;;
    write)
      if [[ "${WRITE_ALREADY_PREVIEWED}" == true ]]; then
        # 上层联合流程已完成 dry-run，但新进程仍需重新核验优化来源。
        verify_head_optimization
        write_head_zero --already-previewed
      else
        write_head_zero
      fi
      ;;
    test) run_head_validation ;;
  esac
}

main() {
  source_ros_environment
  resolve_robot_version
  resolve_mocap_source
  resolve_robot_paths
  print_context

  if [[ -n "${MODE}" ]]; then
    run_selected_mode "${MODE}"
    return
  fi

  while true; do
    show_menu
    local choice=""
    read -r -p "输入选项 [0-8，默认 1]: " choice
    case "${choice:-1}" in
      1) full_workflow ;;
      2) capture_and_apply_board || true ;;
      3) capture_head_data || true ;;
      4) optimize_head ;;
      5) dry_run_head_zero ;;
      6) write_head_zero ;;
      7) print_head_result_verification ;;
      8) run_head_validation ;;
      0) break ;;
      *) warn "无效选项: ${choice}" ;;
    esac
  done
}

main
