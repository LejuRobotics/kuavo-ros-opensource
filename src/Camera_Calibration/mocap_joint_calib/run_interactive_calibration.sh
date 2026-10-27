#!/usr/bin/env bash
# 机器人联合标定交互入口：纯动捕双臂零点标定，并可进入动捕棋盘 + 头部相机标定。
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAMERA_CAL_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
HAND_TEST_DIR="${CAMERA_CAL_DIR}/hand_accuracy_test"
OUTPUT_DIR="${SCRIPT_DIR}/output"
CALIBRATION_YAML="${OUTPUT_DIR}/calibration.yaml"
OPTIMIZATION_MARKER="${OUTPUT_DIR}/.interactive_last_optimization.json"
HEAD_CALIBRATION_RUNNER="${CAMERA_CAL_DIR}/mocap_checkerboard_pose/run_interactive_head_calibration.sh"
HEAD_CSV_ROOT="${CAMERA_CAL_DIR}/output_csv/kuavo_head_sessions"
HEAD_OPTIMIZATION_MARKER="${CAMERA_CAL_DIR}/output/kuavo_head/.head_calibration_session.json"
HEAD_TEST_TEACH_JSON="${CAMERA_CAL_DIR}/teach_capture_output/teach_head_joint_test.json"
HEAD_TEST_RESULT_DIR="${CAMERA_CAL_DIR}/output/kuavo_head_test"
HEAD_TEST_METRICS="${HEAD_TEST_RESULT_DIR}/test_metrics.txt"
PARALLEL_HAND_TEST_PID=""
PARALLEL_HEAD_TEST_PID=""
# 棋盘位姿写在 URDF 里，是头部标定与头部验证共同的比较基准，只能靠动捕实测。
# 只要流程涉及头部（1/2/3/4），进入时一律重新测量并写回 —— 不再用"本轮是否已写入过"
# 这种进程内状态来决定跳过：写零后要重启运控，重启后底座姿态/位置可能变化；操作者
# 也可能手动重启运控或挪动机器人，脚本无从感知，复用旧基准会让头部结果整体偏移。
# board 步只做"动捕测量 + 写 URDF"，不驱动任何机构，重复测量的代价只有几十秒。
DISPLAY_ZERO_ISSUED=false

die() { echo "[ERROR] $*" >&2; exit 1; }
warn() { echo "[WARN] $*" >&2; }
info() { echo "[INFO] $*"; }

# 退出/中断时收尾：并行精度验证（单步工具 11）会后台起两个测试进程，
# 这里负责把它们停掉，避免留下孤儿节点。
cleanup_parallel_tests() {
  local test_pid=""
  for test_pid in "${PARALLEL_HAND_TEST_PID:-}" "${PARALLEL_HEAD_TEST_PID:-}"; do
    if [[ -n "${test_pid}" ]] && kill -0 "${test_pid}" 2>/dev/null; then
      warn "停止本流程启动的精度测试进程 (pid=${test_pid})"
      kill "${test_pid}" 2>/dev/null || true
      wait "${test_pid}" 2>/dev/null || true
    fi
  done
  PARALLEL_HAND_TEST_PID=""
  PARALLEL_HEAD_TEST_PID=""
}

trap cleanup_parallel_tests EXIT
trap 'cleanup_parallel_tests; exit 130' INT TERM

run_cmd() {
  printf '[RUN]'
  printf ' %q' "$@"
  printf '\n'
  "$@"
}

confirm() {
  local prompt="$1"
  local answer=""
  read -r -p "${prompt} [y/N]: " answer
  [[ "${answer}" =~ ^[Yy]$ ]]
}

# 在独立子 shell 中执行一个步骤，退出码写入 ISOLATE_RC。调用方必须按
#   isolate <step>; if [[ ${ISOLATE_RC} -ne 0 ]]; then ...; fi
# 使用，并且**禁止**把它写进 if/&&/||/! 的条件位置。
#
# 原因：bash 会在条件上下文中关闭整条命令（含被调函数体、以及它内部的子 shell）的
# errexit。若写成 `if isolate xxx; then`，某个步骤中途失败（例如 mocap_optimize.sh
# 失败）后仍会继续往下执行，甚至为一份陈旧的 calibration.yaml 记录优化来源标记，
# 最终让 write_zero 把旧 bias 再扣一次。已实测确认该行为。
ISOLATE_RC=0
isolate() {
  local rc=0
  set +e
  ( set -Eeuo pipefail; "$@" )
  rc=$?
  set -e
  ISOLATE_RC="${rc}"
}

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

select_robot_version() {
  local env_version="${ROBOT_VERSION:-}"
  local ros_version=""
  local ros_value=""
  local selected=""

  # 运控由 robot_version_manager.launch 在 ROS 参数服务器写入 /robot_version。
  # 标定终端与运控终端相互独立，因此不能只依赖当前 shell 的环境变量。
  if ros_value="$(timeout 2 rosparam get /robot_version 2>/dev/null)"; then
    ros_version="$(printf '%s' "${ros_value}" | tr -d '[:space:]' | tr -d "'\"")"
  fi

  case "${env_version}" in
    45|52|56|62|63) ;;
    "") env_version="" ;;
    *) warn "忽略不支持的环境变量 ROBOT_VERSION=${env_version}"; env_version="" ;;
  esac
  case "${ros_version}" in
    45|52|56|62|63) ;;
    "") ros_version="" ;;
    *) warn "忽略不支持的 ROS 参数 /robot_version=${ros_version}"; ros_version="" ;;
  esac

  if [[ -n "${env_version}" && -n "${ros_version}" && "${env_version}" != "${ros_version}" ]]; then
    die "机型信息冲突：当前终端 ROBOT_VERSION=${env_version}，运控 /robot_version=${ros_version}。"\
"请核对实机并修正后重试，禁止继续标定。"
  elif [[ -n "${ros_version}" ]]; then
    selected="${ros_version}"
    info "自动检测机器人版本: ${selected}（来源：ROS /robot_version）"
  elif [[ -n "${env_version}" ]]; then
    selected="${env_version}"
    info "自动检测机器人版本: ${selected}（来源：环境变量 ROBOT_VERSION）"
  else
    warn "未从环境变量或 ROS 参数检测到受支持的机器人版本"
    read -r -p "机器人版本 [45/52/56/62/63]: " selected
  fi

  case "${selected}" in
    45|52|56|62|63) ;;
    *) die "一键实机流程当前仅支持 45/52/56/62/63，实际输入 ${selected}" ;;
  esac
  ROBOT_VERSION="${selected}"
  export ROBOT_VERSION
}

select_mocap_source() {
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

select_fixture() {
  local selected="${MOCAP_FIXTURE:-}"
  case "${selected}" in
    legacy|new) ;;
    "")
      echo "手臂末端工装："
      echo "  1) 旧工装 (legacy)"
      echo "  2) 新工装 (new)"
      read -r -p "选择 [1/2，默认 1]: " selected
      case "${selected:-1}" in
        1) selected="legacy" ;;
        2) selected="new" ;;
        *) die "无效工装选择: ${selected}" ;;
      esac
      ;;
    *) die "无效 MOCAP_FIXTURE=${selected}（仅 legacy|new）" ;;
  esac
  FIXTURE_ID="${selected}"
  export MOCAP_FIXTURE="${FIXTURE_ID}"
}

resolve_paths() {
  case "${ROBOT_VERSION}" in
    45)
      LAYOUT="biped45"
      EXPECTED_FK_ROOT="base_link"
      URDF_PATH="${CAMERA_CAL_DIR}/biped_v3_arm_s45.urdf"
      HAND_CONFIG="${HAND_TEST_DIR}/config/hand_accuracy_s45.yaml"
      ;;
    52)
      LAYOUT="biped52"
      EXPECTED_FK_ROOT="waist_yaw_link"
      URDF_PATH="${CAMERA_CAL_DIR}/biped_v3_arm.urdf"
      HAND_CONFIG="${HAND_TEST_DIR}/config/hand_accuracy.yaml"
      ;;
    56)
      LAYOUT="biped56"
      EXPECTED_FK_ROOT="waist_yaw_link"
      URDF_PATH="${CAMERA_CAL_DIR}/biped_v3_arm_s56.urdf"
      HAND_CONFIG="${HAND_TEST_DIR}/config/hand_accuracy.yaml"
      ;;
    62|63)
      LAYOUT="wheel62"
      EXPECTED_FK_ROOT="waist_yaw_link"
      URDF_PATH="${CAMERA_CAL_DIR}/biped_v3_arm_s62.urdf"
      HAND_CONFIG="${HAND_TEST_DIR}/config/hand_accuracy.yaml"
      ;;
  esac
  if [[ "${MOCAP_SOURCE}" == "motive" ]]; then
    CAPTURE_CONFIG="${SCRIPT_DIR}/config/$([[ "${ROBOT_VERSION}" == "45" ]] && echo calib_s45.yaml || echo calib.yaml)"
  else
    CAPTURE_CONFIG="${SCRIPT_DIR}/config/$([[ "${ROBOT_VERSION}" == "45" ]] && echo calib_qingtong_s45.yaml || echo calib_qingtong.yaml)"
  fi
  [[ -f "${CAPTURE_CONFIG}" ]] || die "采集配置不存在: ${CAPTURE_CONFIG}"
  [[ -f "${HAND_CONFIG}" ]] || die "精度验证配置不存在: ${HAND_CONFIG}"
  [[ -f "${URDF_PATH}" ]] || die "机型 URDF 不存在: ${URDF_PATH}"
}

print_context() {
  echo
  echo "=========================================="
  echo "  纯动捕双臂零点标定"
  echo "=========================================="
  echo "  workspace : ${WS_DIR}"
  echo "  version   : ${ROBOT_VERSION}"
  echo "  layout    : ${LAYOUT}"
  echo "  mocap     : ${MOCAP_SOURCE}"
  echo "  fixture   : ${FIXTURE_ID}"
  echo "  fk_root   : ${EXPECTED_FK_ROOT}"
  echo "  urdf      : ${URDF_PATH}"
  echo "  config    : ${CAPTURE_CONFIG}"
  echo "------------------------------------------"
}

validate_capture_config() {
  python3 - "${CAPTURE_CONFIG}" "${LAYOUT}" "${EXPECTED_FK_ROOT}" "${URDF_PATH}" \
    "${FIXTURE_ID}" "${MOCAP_SOURCE}" <<'PY'
import math
import sys
import xml.etree.ElementTree as ET
import yaml

path, expected_layout, expected_root, urdf_path, fixture_id, mocap_source = sys.argv[1:]
with open(path, "r", encoding="utf-8") as stream:
    cfg = yaml.safe_load(stream)
fixture_offsets = (cfg.get("fixtures") or {}).get(fixture_id)
if not isinstance(fixture_offsets, dict) or not {"l_hand", "r_hand"} <= set(fixture_offsets):
    raise SystemExit(f"配置缺少 fixtures.{fixture_id}.l_hand/r_hand")
bodies_for_fixture = {body.get("name"): body for body in cfg.get("mocap_bodies", [])}
for name in ("l_hand", "r_hand"):
    if name not in bodies_for_fixture:
        raise SystemExit(f"mocap_bodies 缺少 {name}")
    bodies_for_fixture[name]["link_offset_mm"] = list(fixture_offsets[name])

template_layout = str(cfg.get("robot", {}).get("robot_layout", ""))
urdf = ET.parse(urdf_path).getroot()
links = {node.get("name") for node in urdf.findall("link")}
required_links = {expected_root, "zarm_l7_link", "zarm_r7_link"}
missing_links = sorted(required_links - links)
if missing_links:
    raise SystemExit(f"机型 URDF 缺少标定链 link: {missing_links}")

align = cfg.get("mocap_frame_align") or {}
if align.get("enabled", True) and "axis_flip" in align:
    flip = align["axis_flip"]
    if not isinstance(flip, list) or len(flip) != 3:
        raise SystemExit("mocap_frame_align.axis_flip 必须包含3个数")
    determinant = math.prod(float(v) for v in flip)
    if determinant <= 0.0:
        raise SystemExit(
            f"axis_flip={flip} 是镜像(det={determinant:g})，不能用于6DOF优化；"
            "请重对齐刚体轴，或使用合法 rpy_rad/单位旋转"
        )

bodies = {body.get("name"): body for body in cfg.get("mocap_bodies", [])}
for name in ("l_hand", "r_hand", "torso", "l_shoulder"):
    if name not in bodies:
        raise SystemExit(f"mocap_bodies 缺少 {name}")
    offset = bodies[name].get("link_offset_mm")
    if not isinstance(offset, list) or len(offset) != 3:
        raise SystemExit(f"{name}.link_offset_mm 必须包含3个数")

print(f"[CHECK] config OK: template_layout={template_layout}, effective_layout={expected_layout}, fixture={fixture_id}, fk_root={expected_root}")
print(f"[CHECK] urdf={urdf_path}")
for name in ("l_hand", "r_hand", "torso", "l_shoulder"):
    print(f"[CHECK] {name}: topic={bodies[name].get('topic')}, offset_mm={bodies[name]['link_offset_mm']}")
PY
}

check_live_topics() {
  local config_path="${1:-${CAPTURE_CONFIG}}"
  local source_name="${2:-${MOCAP_SOURCE}}"
  timeout 3 rostopic list >/dev/null 2>&1 || die "无法连接 ROS master，请先启动机器人控制程序和动捕接收节点"
  local topics=()
  mapfile -t topics < <(python3 - "${config_path}" "${source_name}" <<'PY'
import sys
import yaml
with open(sys.argv[1], "r", encoding="utf-8") as stream:
    cfg = yaml.safe_load(stream)
print(cfg.get("robot", {}).get("sensor_topic", "/sensors_data_raw"))
body_key = "mocap_bodies_alt" if sys.argv[2] == "qingtong" and cfg.get("mocap_bodies_alt") else "mocap_bodies"
for body in cfg.get(body_key, []):
    if body.get("name") in {"l_hand", "r_hand", "torso", "l_shoulder"}:
        print(body.get("topic", ""))
PY
  )
  local topic=""
  for topic in "${topics[@]}"; do
    [[ -n "${topic}" ]] || continue
    if ! timeout 5 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
      die "话题无有效消息: ${topic}"
    fi
    info "话题有效: ${topic}"
  done
}

check_head_camera_topics() {
  local image_topic="/head_camera/color/image_raw"
  local info_topic="/head_camera/color/camera_info"
  # doc/README 的规则是按机型：S45 为倒装相机，需要先启动 180° 旋转节点。
  if [[ "${ROBOT_VERSION}" == "45" ]]; then
    image_topic="/head_camera/color/image_raw_rotate_180"
    info_topic="/head_camera/color/camera_info_rotate_180"
  fi
  local topic=""
  for topic in "${image_topic}" "${info_topic}"; do
    if ! timeout 5 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
      die "头部相机话题无有效消息: ${topic}"
    fi
    info "头部相机话题有效: ${topic}"
  done
}

latest_capture() {
  python3 - "${SCRIPT_DIR}" "${EXPECTED_FK_ROOT}" "${ROBOT_VERSION}" "${LAYOUT}" \
    "${FIXTURE_ID}" "${MOCAP_SOURCE}" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
expected_fk = sys.argv[2]
expected_version = sys.argv[3]
expected_layout = sys.argv[4]
expected_fixture = sys.argv[5]
expected_source = sys.argv[6]
version_layout = {
    "45": "biped45", "52": "biped52", "56": "biped56",
    "62": "wheel62", "63": "wheel62",
}
urdf_layout = {
    "biped_v3_arm_s45.urdf": "biped45",
    "biped_v3_arm.urdf": "biped52",
    "biped_v3_arm_s56.urdf": "biped56",
    "biped_v3_arm_s62.urdf": "wheel62",
}
matches = []
for path in root.glob("capture_*.json"):
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        meta = data.get("meta") or {}
        if str(meta.get("fk_root", "")) != expected_fk:
            continue
        inferred = set()
        layout = str(meta.get("robot_layout") or "")
        if layout:
            inferred.add(layout)
        version = str(meta.get("robot_version") or "")
        if version:
            inferred.add(version_layout.get(version, f"unsupported:{version}"))
        urdf_name = pathlib.Path(str(meta.get("urdf") or "")).name
        if urdf_name in urdf_layout:
            inferred.add(urdf_layout[urdf_name])
        # waist_yaw_link 被 S52/S56/S62 共用，不能仅凭 fk_root 猜机型。
        if inferred != {expected_layout}:
            continue
        fixture = meta.get("fixture") or {}
        actual_fixture = str(fixture.get("id") or "")
        if actual_fixture != expected_fixture:
            continue
        if fixture.get("primary_source") != expected_source:
            continue
        matches.append(path)
    except Exception:
        continue
if matches:
    print(max(matches, key=lambda p: p.stat().st_mtime))
PY
}

latest_accuracy_report() {
  python3 - "${HAND_TEST_DIR}" "${LAYOUT}" "${FIXTURE_ID}" "${MOCAP_SOURCE}" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
expected_layout = sys.argv[2]
expected_fixture = sys.argv[3]
expected_source = sys.argv[4]
matches = []
for path in root.glob("hand_accuracy_report_*.json"):
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        layout = (
            data.get("meta", {})
            .get("config", {})
            .get("robot", {})
            .get("robot_layout")
        )
        fixture = (
            data.get("meta", {})
            .get("config", {})
            .get("fixture", {})
            .get("id", "")
        )
        source = (
            data.get("meta", {})
            .get("config", {})
            .get("fixture", {})
            .get("selected_mocap_mode", "")
        )
        if layout == expected_layout and fixture == expected_fixture and source == expected_source:
            matches.append(path)
    except Exception:
        continue
if matches:
    print(max(matches, key=lambda p: p.stat().st_mtime))
PY
}

require_capture() {
  CAPTURE_PATH="$(latest_capture)"
  [[ -n "${CAPTURE_PATH}" && -f "${CAPTURE_PATH}" ]] || \
    die "未找到与 layout=${LAYOUT}, fixture=${FIXTURE_ID}, mocap=${MOCAP_SOURCE} 匹配的 capture，请先采集"
  info "使用 capture: ${CAPTURE_PATH}"
}

motion_preview() {
  echo
  warn "即将驱动双臂。人员退出运动范围，确认急停可用。"
  if [[ "${1:-}" != "--confirmed" ]]; then
    confirm "开始只运动检查" || return 0
  fi
  run_cmd python3 "${SCRIPT_DIR}/record_joint_poses.py" \
    --robot-version "${ROBOT_VERSION}" --config "${CAPTURE_CONFIG}" \
    --robot-layout "${LAYOUT}" --urdf "${URDF_PATH}" --fk-root "${EXPECTED_FK_ROOT}" \
    --fixture "${FIXTURE_ID}" --mocap-source "${MOCAP_SOURCE}" \
    --no-capture-head --motion-only
}


capture_data() {
  CAPTURE_PATH=""
  validate_capture_config
  check_live_topics "${CAPTURE_CONFIG}" "${MOCAP_SOURCE}"
  echo
  warn "即将驱动双臂并采集数据。人员退出运动范围，确认急停可用。"
  confirm "开始采集" || return 0
  run_cmd python3 "${SCRIPT_DIR}/record_joint_poses.py" \
    --robot-version "${ROBOT_VERSION}" --config "${CAPTURE_CONFIG}" \
    --robot-layout "${LAYOUT}" --urdf "${URDF_PATH}" --fk-root "${EXPECTED_FK_ROOT}" \
    --fixture "${FIXTURE_ID}" --mocap-source "${MOCAP_SOURCE}" \
    --no-capture-head
  require_capture
}

optimize_capture() {
  validate_capture_config
  if [[ -z "${CAPTURE_PATH:-}" ]]; then
    require_capture
  else
    [[ -f "${CAPTURE_PATH}" ]] || die "指定 capture 不存在: ${CAPTURE_PATH}"
    info "使用本轮 capture: ${CAPTURE_PATH}"
  fi
  run_cmd bash "${SCRIPT_DIR}/mocap_optimize.sh" "${CAPTURE_PATH}" --layout "${LAYOUT}"
  python3 - "${OPTIMIZATION_MARKER}" "${CALIBRATION_YAML}" "${CAPTURE_PATH}" \
    "${ROBOT_VERSION}" "${LAYOUT}" "${FIXTURE_ID}" "${MOCAP_SOURCE}" <<'PY'
import datetime
import hashlib
import json
import pathlib
import sys

marker = pathlib.Path(sys.argv[1])
calibration = pathlib.Path(sys.argv[2]).resolve()
capture = pathlib.Path(sys.argv[3]).resolve()
version = sys.argv[4]
layout = sys.argv[5]
fixture_id = sys.argv[6]
mocap_source = sys.argv[7]
if not calibration.is_file():
    raise SystemExit(f"优化结果不存在: {calibration}")
digest = hashlib.sha256(calibration.read_bytes()).hexdigest()
payload = {
    "created_at": datetime.datetime.now().isoformat(),
    "robot_version": version,
    "layout": layout,
    "fixture_id": fixture_id,
    "mocap_source": mocap_source,
    "capture": str(capture),
    "calibration": str(calibration),
    "calibration_sha256": digest,
}
marker.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(f"[INFO] 已记录优化来源: {marker}")
PY
}

verify_calibration_provenance() {
  python3 - "${OPTIMIZATION_MARKER}" "${CALIBRATION_YAML}" \
    "${ROBOT_VERSION}" "${LAYOUT}" "${FIXTURE_ID}" "${MOCAP_SOURCE}" "${CAPTURE_PATH:-}" <<'PY'
import hashlib
import json
import pathlib
import sys

marker = pathlib.Path(sys.argv[1])
calibration = pathlib.Path(sys.argv[2]).resolve()
version = sys.argv[3]
layout = sys.argv[4]
fixture_id = sys.argv[5]
mocap_source = sys.argv[6]
expected_capture = sys.argv[7]
if not marker.is_file():
    raise SystemExit("缺少本次优化来源标记；请先在菜单中执行优化，禁止直接写入未知 calibration.yaml")
try:
    data = json.loads(marker.read_text(encoding="utf-8"))
except Exception as exc:
    raise SystemExit(f"优化来源标记无效: {exc}")
if str(data.get("robot_version")) != version or data.get("layout") != layout:
    raise SystemExit(
        f"优化结果属于 version={data.get('robot_version')}, layout={data.get('layout')}，"
        f"当前选择 version={version}, layout={layout}"
    )
if data.get("fixture_id") != fixture_id:
    raise SystemExit(
        f"优化结果属于 fixture={data.get('fixture_id')}，当前选择 fixture={fixture_id}"
    )
if data.get("mocap_source") != mocap_source:
    raise SystemExit(
        f"优化结果属于 mocap={data.get('mocap_source')}，当前选择 mocap={mocap_source}"
    )
if pathlib.Path(data.get("calibration", "")).resolve() != calibration:
    raise SystemExit("优化来源标记中的 calibration 路径不匹配")
if not calibration.is_file():
    raise SystemExit(f"优化结果不存在: {calibration}")
actual = hashlib.sha256(calibration.read_bytes()).hexdigest()
if actual != data.get("calibration_sha256"):
    raise SystemExit("calibration.yaml 在优化后已被替换或修改，请重新优化")
capture = pathlib.Path(data.get("capture", ""))
if not capture.is_file():
    raise SystemExit(f"对应 capture 已不存在: {capture}")
if expected_capture and capture.resolve() != pathlib.Path(expected_capture).resolve():
    raise SystemExit(
        f"当前 capture={pathlib.Path(expected_capture).resolve()} 与优化使用的 capture={capture.resolve()} 不一致"
    )
print(f"[CHECK] calibration 来源有效: version={version}, layout={layout}, fixture={fixture_id}, mocap={mocap_source}, capture={capture}")
PY
}

# 防重复写零：检查当前 calibration 是否已写入过零点。
# write_zero 写零成功后会 mv marker 成 .interactive_last_optimization.json.written.*，
# 其中含 calibration_sha256；同一结果（同 sha）再次写零会从当前零点再扣一次 bias，禁止。
check_calibration_not_written() {
  [[ -f "${CALIBRATION_YAML}" ]] || die "优化结果不存在: ${CALIBRATION_YAML}"
  python3 - "${OPTIMIZATION_MARKER}" "${CALIBRATION_YAML}" "${ROBOT_VERSION}" "${LAYOUT}" <<'PY'
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
            f"该 calibration 已写零过（记录: {pathlib.Path(written).name}），禁止重复写入。\n"
            f"同一结果重复写零会让零点偏 2×bias；如需重新标定，请重新采集后再优化。"
        )
PY
}

plot_calibration() {
  if [[ -z "${CAPTURE_PATH:-}" ]]; then
    require_capture
  else
    [[ -f "${CAPTURE_PATH}" ]] || die "指定 capture 不存在: ${CAPTURE_PATH}"
  fi
  [[ -f "${CALIBRATION_YAML}" ]] || die "优化结果不存在: ${CALIBRATION_YAML}"
  verify_calibration_provenance
  run_cmd python3 "${SCRIPT_DIR}/scripts/plot_mocap_error.py" \
    --capture "${CAPTURE_PATH}" --layout "${LAYOUT}" \
    --calibration_yaml "${CALIBRATION_YAML}"
}

dry_run_zero() {
  [[ -f "${CALIBRATION_YAML}" ]] || die "优化结果不存在: ${CALIBRATION_YAML}"
  verify_calibration_provenance
  run_cmd python3 "${SCRIPT_DIR}/write_zero.py" \
    --robot-version "${ROBOT_VERSION}" --parts arm \
    --calibration-yaml "${CALIBRATION_YAML}" --dry-run
}

write_zero() {
  WRITE_ZERO_DONE=false
  # 已写零过 -> 优雅取消（返回 0，避免 set -e 把整个交互会话踢掉）；错误信息已由 check 打到 stderr
  if ! check_calibration_not_written; then
    info "已取消正式写零（该 calibration 已写零过，禁止重复写入）"
    return 0
  fi
  if [[ "${1:-}" != "--already-previewed" ]]; then
    dry_run_zero
  fi
  echo
  warn "正式写入会修改当前机器人的零点文件；同一结果禁止重复写入。"
  local token=""
  read -r -p "确认 dry-run 无误后输入 WRITE 继续: " token
  [[ "${token}" == "WRITE" ]] || { info "已取消正式写入"; return 0; }
  run_cmd python3 "${SCRIPT_DIR}/write_zero.py" \
    --robot-version "${ROBOT_VERSION}" --parts arm \
    --calibration-yaml "${CALIBRATION_YAML}"
  mv -- "${OPTIMIZATION_MARKER}" \
    "${OPTIMIZATION_MARKER}.written.$(date +%Y%m%d-%H%M%S)"
  WRITE_ZERO_DONE=true
  # 单步写零同样要求重启运控：置位后菜单会持续提示，且头部流程会重测棋盘位姿。
  DISPLAY_ZERO_ISSUED=true
  warn "零点已写入。必须停止并重新启动机器人控制程序后才会生效。"
}

run_accuracy_test() {
  ACCURACY_TEST_DONE=false
  check_live_topics "${HAND_CONFIG}" "${MOCAP_SOURCE}"
  warn "末端精度验证也会驱动双臂。请先确认新零点已通过重启运控生效。"
  if [[ "${1:-}" != "--confirmed" ]] && ! confirm "开始末端精度验证"; then
    return 0
  fi
  run_cmd python3 "${HAND_TEST_DIR}/run_hand_accuracy_test.py" \
    --config "${HAND_CONFIG}" --mocap-mode "${MOCAP_SOURCE}" --fixture "${FIXTURE_ID}"
  ACCURACY_TEST_DONE=true
}

plot_accuracy() {
  # 允许调用方传入本轮刚生成的报告，避免 latest_accuracy_report 在旧报告
  # mtime 更大时挑中上一轮的报告。
  local report="${1:-}"
  if [[ -z "${report}" ]]; then
    report="$(latest_accuracy_report)"
  fi
  [[ -n "${report}" && -f "${report}" ]] || die "未找到末端精度 JSON 报告"
  run_cmd python3 "${HAND_TEST_DIR}/plot_hand_accuracy_report.py" --json "${report}"
}

run_head_calibration() {
  [[ -f "${HEAD_CALIBRATION_RUNNER}" ]] || die "头部联合标定入口不存在: ${HEAD_CALIBRATION_RUNNER}"
  run_cmd bash "${HEAD_CALIBRATION_RUNNER}" \
    --robot-version "${ROBOT_VERSION}" --mocap-source "${MOCAP_SOURCE}" --mode full
}

run_head_validation() {
  [[ -f "${HEAD_CALIBRATION_RUNNER}" ]] || die "头部联合标定入口不存在: ${HEAD_CALIBRATION_RUNNER}"
  local extra_args=()
  if [[ "${1:-}" == "--confirmed" ]]; then
    extra_args+=(--test-confirmed)
  fi
  run_cmd bash "${HEAD_CALIBRATION_RUNNER}" \
    --robot-version "${ROBOT_VERSION}" --mocap-source "${MOCAP_SOURCE}" \
    --mode test "${extra_args[@]}"
}

run_parallel_accuracy_validation() {
  local hand_pre_rc=0
  local head_pre_rc=0
  local hand_ready=false
  local head_ready=false

  # 两路预检相互隔离：某一路缺数据时只跳过该路，另一套测试仍可运行。
  set +e
  (set -Eeuo pipefail; check_live_topics "${HAND_CONFIG}" "${MOCAP_SOURCE}")
  hand_pre_rc=$?
  (
    set -Eeuo pipefail
    [[ -f "${HEAD_CALIBRATION_RUNNER}" ]] || die \
      "头部联合标定入口不存在: ${HEAD_CALIBRATION_RUNNER}"
    [[ -f "${CAMERA_CAL_DIR}/teach_capture_output/teach_head_joint_test.json" ]] || die \
      "缺少头部测试姿态: ${CAMERA_CAL_DIR}/teach_capture_output/teach_head_joint_test.json"
    check_head_camera_topics
  )
  head_pre_rc=$?
  set -e
  if [[ "${hand_pre_rc}" -eq 0 ]]; then
    hand_ready=true
  else
    warn "手腕动捕预检失败，本轮不启动手腕测试；头部测试仍可继续。"
  fi
  if [[ "${head_pre_rc}" -eq 0 ]]; then
    head_ready=true
  else
    warn "头部棋盘预检失败，本轮不启动头部测试；手腕测试仍可继续。"
  fi
  if [[ "${hand_ready}" != true && "${head_ready}" != true ]]; then
    warn "两路精度验证预检均失败，没有启动任何运动。"
    return 0
  fi

  echo
  warn "联合精度验证：手腕使用动捕，头部使用棋盘和头部相机；预检通过的两路将同时驱动。"
  warn "两路会同时驱动头部和双臂，一个操作者无法同时盯住两路运动；"
  warn "若需要逐个确认每一路的运动，请改用主菜单 2（完整精度验证）或 6（末端精度验证）。"
  warn "请确认双臂与头部零点均已正式写入，并已重启运控使其生效。"
  confirm "开始并行联合精度验证" || { info "已取消联合精度验证"; return 0; }

  if [[ "${hand_ready}" == true ]]; then
    (
      info "[手腕测试] 启动动捕末端精度验证；完成后自动画图"
      run_accuracy_test --confirmed
      if [[ "${ACCURACY_TEST_DONE}" != true ]]; then
        echo "[ERROR] [手腕测试] 未生成本轮精度报告" >&2
        exit 1
      fi
      plot_accuracy
      info "[手腕测试] 动捕验证与画图完成"
    ) &
    PARALLEL_HAND_TEST_PID=$!
  fi

  if [[ "${head_ready}" == true ]]; then
    (
      info "[头部测试] 启动棋盘/头部相机验证；完成后自动画图"
      run_head_validation --confirmed
      info "[头部测试] 棋盘验证与画图完成"
    ) &
    PARALLEL_HEAD_TEST_PID=$!
  fi

  local hand_rc="${hand_pre_rc}"
  local head_rc="${head_pre_rc}"
  set +e
  if [[ -n "${PARALLEL_HAND_TEST_PID}" ]]; then
    wait "${PARALLEL_HAND_TEST_PID}"
    hand_rc=$?
  fi
  if [[ -n "${PARALLEL_HEAD_TEST_PID}" ]]; then
    wait "${PARALLEL_HEAD_TEST_PID}"
    head_rc=$?
  fi
  set -e
  PARALLEL_HAND_TEST_PID=""
  PARALLEL_HEAD_TEST_PID=""

  echo
  info "联合精度验证结束：手腕退出码=${hand_rc}，头部退出码=${head_rc}"
  if [[ "${hand_rc}" -ne 0 ]]; then
    warn "手腕动捕验证失败；头部测试已独立运行，不受该失败影响。"
  fi
  if [[ "${head_rc}" -ne 0 ]]; then
    warn "头部棋盘验证失败；手腕测试已独立运行，不受该失败影响。"
  fi
  if [[ "${hand_rc}" -eq 0 && "${head_rc}" -eq 0 ]]; then
    info "手腕动捕报告/图和头部棋盘测试图均已生成。"
  fi
}




# 纯头部采集（流程 1 头部段、流程 3、单步工具）：只驱动头部走完整条 teach 轨迹并逐点触发棋盘采样，
# 双臂全程保持不动 —— 头部链的采集入口本身就是"只动头部"的，不向手臂发任何指令。
# 本函数不启动手臂采集、不产出也不校验手臂 capture，只需要 HEAD_CSV_DIR，
# 后续 head_zero_pipeline 的 --head-csv-dir 直接可用。
# 棋盘位姿的前提校验由子脚本 require_head_csv_dir 在 optimize 阶段完成，不在这里重复。
capture_head_only() {
  HEAD_CAPTURE_DONE=false
  check_head_camera_topics

  local stamp=""
  stamp="$(date +%Y%m%d_%H%M%S)"
  HEAD_CSV_DIR="${HEAD_CSV_ROOT}/${ROBOT_VERSION}/session_${stamp}"
  mkdir -p "${HEAD_CSV_DIR}"

  echo
  warn "即将驱动头部走完整条 teach 轨迹逐点采集棋盘图像；双臂全程保持不动。"
  warn "棋盘和机器人底座在整轮采集期间不能移动。"
  run_cmd bash "${HEAD_CALIBRATION_RUNNER}" \
    --robot-version "${ROBOT_VERSION}" --mocap-source "${MOCAP_SOURCE}" \
    --mode capture --head-csv-dir "${HEAD_CSV_DIR}" || {
    HEAD_CSV_DIR=""
    warn "头部采集已取消或未通过，流程停止；不会使用目录中的旧头部 CSV。"
    return 0
  }
  if ! compgen -G "${HEAD_CSV_DIR}/*.csv" >/dev/null; then
    warn "头部采集目录没有 CSV: ${HEAD_CSV_DIR}；流程停止"
    HEAD_CSV_DIR=""
    return 0
  fi
  HEAD_CAPTURE_DONE=true
  info "头部棋盘数据: ${HEAD_CSV_DIR}"
}

ensure_head_board() {
  [[ -f "${HEAD_CALIBRATION_RUNNER}" ]] || die "头部联合标定入口不存在: ${HEAD_CALIBRATION_RUNNER}"
  echo
  info "第 1 步：动捕测量棋盘位姿并写入当前机型 URDF（每次进入头部流程都重测）。"
  run_cmd bash "${HEAD_CALIBRATION_RUNNER}" \
    --robot-version "${ROBOT_VERSION}" --mocap-source "${MOCAP_SOURCE}" --mode board || {
    info "已取消或棋盘动捕未通过，头部流程停止"
    return 1
  }
}

# 头部写零管线。pretend 只做到 dry-run，apply 才会进入正式写入确认。
# 每一步都在 isolate() 子 shell 中执行，并用文件副作用判断是否真的成功：
#   - 优化/画图成功 -> 本轮 HEAD_OPTIMIZATION_MARKER 被重新写入
#   - 正式写入成功 -> 该 marker 被归档成 .written.*，即 marker 不再存在
head_zero_pipeline() {
  local mode="$1"
  HEAD_ZERO_OPTIMIZE_DONE=false
  HEAD_ZERO_DRY_RUN_DONE=false
  HEAD_ZERO_WRITTEN=false
  [[ -n "${HEAD_CSV_DIR:-}" && -d "${HEAD_CSV_DIR}" ]] || { warn "本轮头部 CSV 无效，跳过头部标定"; return 0; }

  # 先删掉旧 marker：子 shell 里的赋值传不回来，用"重新出现"判断本步真的跑成功。
  rm -f -- "${HEAD_OPTIMIZATION_MARKER}"
  isolate run_cmd bash "${HEAD_CALIBRATION_RUNNER}" \
    --robot-version "${ROBOT_VERSION}" --mocap-source "${MOCAP_SOURCE}" \
    --mode optimize --head-csv-dir "${HEAD_CSV_DIR}"
  if [[ "${ISOLATE_RC}" -ne 0 ]]; then
    warn "头部优化/画图失败（exit=${ISOLATE_RC}）；跳过头部 dry-run 与写零。"
    return 0
  fi
  if [[ ! -f "${HEAD_OPTIMIZATION_MARKER}" ]]; then
    warn "头部优化未生成本轮来源标记；跳过头部 dry-run 与写零。"
    return 0
  fi
  HEAD_ZERO_OPTIMIZE_DONE=true

  isolate run_cmd bash "${HEAD_CALIBRATION_RUNNER}" \
    --robot-version "${ROBOT_VERSION}" --mocap-source "${MOCAP_SOURCE}" \
    --mode dry-run --head-csv-dir "${HEAD_CSV_DIR}"
  if [[ "${ISOLATE_RC}" -ne 0 ]]; then
    warn "头部写零 dry-run 失败（exit=${ISOLATE_RC}）；跳过头部写零。"
    return 0
  fi
  HEAD_ZERO_DRY_RUN_DONE=true
  if [[ "${mode}" != "apply" ]]; then
    return 0
  fi

  echo
  if ! confirm "头部优化结果和 dry-run 是否合理，是否写入头部零点"; then
    info "已跳过头部正式写零。"
    return 0
  fi
  isolate run_cmd bash "${HEAD_CALIBRATION_RUNNER}" \
    --robot-version "${ROBOT_VERSION}" --mocap-source "${MOCAP_SOURCE}" \
    --mode write --head-csv-dir "${HEAD_CSV_DIR}" --already-previewed
  if [[ "${ISOLATE_RC}" -ne 0 ]]; then
    warn "头部写零步骤失败（exit=${ISOLATE_RC}）；禁止直接重试，请先检查上方日志和零点文件。"
    return 0
  fi
  if [[ ! -f "${HEAD_OPTIMIZATION_MARKER}" ]]; then
    HEAD_ZERO_WRITTEN=true
    DISPLAY_ZERO_ISSUED=true
    info "头部零点写入完成。"
  else
    info "头部正式写零已取消。"
  fi
}

# dry-run 之后、正式写入之前再次核验来源标记，防止确认期间 calibration.yaml
# 被替换成另一份结果。必须与 write_zero 放在同一个子 shell 里连续执行，
# 否则中间又会出现一个"已确认但未复核"的窗口。
arm_write_zero_step() {
  verify_calibration_provenance
  write_zero --already-previewed
}

# 手臂写零管线，语义与 head_zero_pipeline 对称，但全部走本目录的既有入口
# （optimize_capture 负责 mv 归档 marker、write_zero 负责 mv 归档自身 marker）。
arm_zero_pipeline() {
  local mode="$1"
  ARM_ZERO_OPTIMIZE_DONE=false
  ARM_ZERO_DRY_RUN_DONE=false
  ARM_ZERO_WRITTEN=false
  [[ -n "${CAPTURE_PATH:-}" && -f "${CAPTURE_PATH}" ]] || { warn "本轮手臂 capture 无效，跳过手臂标定"; return 0; }

  rm -f -- "${OPTIMIZATION_MARKER}"
  isolate optimize_capture
  if [[ "${ISOLATE_RC}" -ne 0 ]]; then
    warn "手臂优化失败（exit=${ISOLATE_RC}）；跳过手臂画图、dry-run 与写零。"
    return 0
  fi
  if [[ ! -f "${OPTIMIZATION_MARKER}" ]]; then
    warn "手臂优化未生成本轮来源标记；跳过手臂画图、dry-run 与写零。"
    return 0
  fi
  ARM_ZERO_OPTIMIZE_DONE=true

  # 手臂误差图不再是可选步骤：优化成功后一律画，失败只告警不阻断 dry-run。
  isolate plot_calibration
  if [[ "${ISOLATE_RC}" -ne 0 ]]; then
    warn "手臂误差图绘制失败（exit=${ISOLATE_RC}）；优化结果仍然保留，继续 dry-run。"
  fi

  isolate dry_run_zero
  if [[ "${ISOLATE_RC}" -ne 0 ]]; then
    warn "手臂写零 dry-run 失败（exit=${ISOLATE_RC}）；跳过手臂写零。"
    return 0
  fi
  ARM_ZERO_DRY_RUN_DONE=true
  if [[ "${mode}" != "apply" ]]; then
    return 0
  fi

  echo
  if ! confirm "双臂优化结果和 dry-run 是否合理，是否写入双臂零点"; then
    info "已跳过双臂正式写零。"
    return 0
  fi
  isolate arm_write_zero_step
  if [[ "${ISOLATE_RC}" -ne 0 ]]; then
    warn "双臂写零步骤失败（exit=${ISOLATE_RC}）；禁止直接重试，请先检查上方日志和零点文件。"
    return 0
  fi
  if [[ ! -f "${OPTIMIZATION_MARKER}" ]]; then
    ARM_ZERO_WRITTEN=true
    DISPLAY_ZERO_ISSUED=true
    info "双臂零点写入完成。"
  else
    info "双臂正式写零已取消。"
  fi
}

remind_restart_after_zero() {
  echo
  if [[ "${DISPLAY_ZERO_ISSUED}" == true ]]; then
    warn "本轮已写入零点：必须停止并重新启动机器人控制程序后才会生效，然后才能做精度验证。"
  else
    info "本轮没有正式写入任何零点，无需因本轮操作重启运控。"
  fi
}

# 头部零点标定（流程 3）：棋盘位姿 -> 纯头部采集(只动头部) -> 头部优化/画图/dry-run -> 可选写零。
# 流程 3 只标定头部，因此用纯头部采集（只动头部、不产手臂 capture）。
# 安全检查已去掉：本流程只有头部运动，采集前由子脚本自己的提示与确认把关。
head_calibration_workflow() {
  ensure_head_board || return 0
  capture_head_only
  if [[ "${HEAD_CAPTURE_DONE}" != true ]]; then
    info "已取消头部采集，流程停止"
    return 0
  fi
  echo
  # 采集成功后直接优化并出图，不再询问：底层 optimize 与画图是同一个步骤；
  # 采集已经让操作者确认过一次运动，再问一次只是多余的停顿。
  # 本轮流程只保留一个询问点 —— 正式写零，因为那是唯一真正改硬件的关口。
  info "开始优化头部采集数据并绘制误差报告。"
  head_zero_pipeline apply
  if [[ "${HEAD_ZERO_OPTIMIZE_DONE}" != true ]]; then
    return 0
  fi
  info "头部优化报告与图片: ${CAMERA_CAL_DIR}/output/kuavo_head"
  remind_restart_after_zero
}

arm_calibration_workflow() {
  if confirm "是否先执行只运动安全检查"; then
    motion_preview --confirmed
  fi
  capture_data
  if [[ -z "${CAPTURE_PATH:-}" ]]; then
    info "已取消采集，流程停止；不会使用目录中的旧 capture"
    return 0
  fi
  echo
  # 同流程 3：采集成功后直接优化并绘制优化前后误差报告，不再询问；
  # 本轮流程只保留"是否正式写零"这一个询问点。
  info "开始优化手臂采集数据并绘制优化前后误差报告。"
  arm_zero_pipeline apply
  if [[ "${ARM_ZERO_OPTIMIZE_DONE}" != true ]]; then
    return 0
  fi
  remind_restart_after_zero
}

# 流程 1：先头部、后手臂，两段各自独立采集，全程不存在两个机构同时被下发的时刻。
# 头部段走 capture_head_only（头部子脚本自己的采集链，只驱动头部，不发任何手臂指令），
# 手臂段走 capture_data（record_joint_poses.py --no-capture-head，只驱动双臂）。
# 头部整条轨迹走完、会话结束后才开始手臂段，头部此后不再被下发。
# 随后先头部、后手臂分别优化、画图、写零；两路写零必须严格串行：
# Ruiwo 电机构型下两路写的是同一个 arms_zero.yaml，只是槽位不同，并行会互相覆盖。
full_calibration_workflow() {
  ensure_head_board || return 0

  echo
  info "[1/4 头部采集] 只驱动头部走完整条 teach 轨迹，逐点触发棋盘采样"
  capture_head_only
  if [[ "${HEAD_CAPTURE_DONE}" != true ]]; then
    info "已取消头部采集，流程停止；不会使用目录中的旧头部 CSV"
    return 0
  fi

  echo
  info "[2/4 手臂采集] 头部已采集完毕，只驱动双臂走完整条 teach 轨迹，逐点采集手腕动捕"
  # 只运动检查同样只针对双臂：头部已经采完，这里不再驱动头部。
  if confirm "是否先执行只运动安全检查（驱动双臂，不采集）"; then
    motion_preview --confirmed
  fi
  capture_data
  if [[ -z "${CAPTURE_PATH:-}" ]]; then
    info "已取消手臂采集，流程停止；不会使用目录中的旧 capture"
    return 0
  fi

  echo
  # 采集成功后直接优化并出图（底层 optimize 与画图不可拆），只在正式写零前询问。
  info "[3/4 头部写零] 优化、画图并按确认写入头部零点"
  head_zero_pipeline apply

  echo
  # 同上：优化与出图不再询问，只保留正式写零的确认。
  info "[4/4 手臂写零] 优化、画图并按确认写入手臂零点"
  arm_zero_pipeline apply

  remind_restart_after_zero
  if [[ "${DISPLAY_ZERO_ISSUED}" == true ]]; then
    info "两路零点写入并重启运控后，可选择主菜单 2 做完整精度验证（先头部，后末端）。"
  fi
}

# 头部精度验证管线（流程 4）：头部棋盘测试，出报告与出图都不再询问，并始终打印精度结果。
# 只保留"开始头部精度验证"一个确认 —— 它后面就是头部运动。
head_validation_pipeline() {
  HEAD_VALIDATION_TEST_DONE=false
  [[ -f "${HEAD_CALIBRATION_RUNNER}" ]] || die "头部联合标定入口不存在: ${HEAD_CALIBRATION_RUNNER}"
  [[ -f "${HEAD_TEST_TEACH_JSON}" ]] || die \
    "缺少头部测试姿态: ${HEAD_TEST_TEACH_JSON}；请先用 teach_joint_capture.py 生成测试姿态"
  check_head_camera_topics
  echo
  warn "头部精度验证：按测试轨迹逐点采集棋盘图像并与当前 URDF 比较。"
  if ! confirm "开始头部精度验证"; then
    info "已取消头部精度验证。"
    return 0
  fi

  # 测试指标覆盖前先快照目录状态，避免把上一轮的旧报告当成本轮结果。
  local before="missing"
  if [[ -d "${HEAD_TEST_RESULT_DIR}" ]]; then
    before="$(stat -c '%Y:%s' "${HEAD_TEST_RESULT_DIR}" 2>/dev/null || echo unknown)"
  fi

  isolate run_head_validation --confirmed
  if [[ "${ISOLATE_RC}" -ne 0 ]]; then
    warn "头部精度验证失败（exit=${ISOLATE_RC}）；不再展示或绘制目录中的旧结果。"
    return 0
  fi

  local after="missing"
  if [[ -d "${HEAD_TEST_RESULT_DIR}" ]]; then
    after="$(stat -c '%Y:%s' "${HEAD_TEST_RESULT_DIR}" 2>/dev/null || echo unknown)"
  fi
  if [[ "${before}" == "${after}" ]]; then
    warn "头部测试目录未更新（${HEAD_TEST_RESULT_DIR}），无法确认本轮生成了新结果；不展示旧报告。"
    return 0
  fi
  HEAD_VALIDATION_TEST_DONE=true

  echo
  info "===== 头部精度结果 ====="
  if [[ -f "${HEAD_TEST_METRICS}" ]]; then
    cat -- "${HEAD_TEST_METRICS}"
  else
    warn "未找到头部测试指标文件: ${HEAD_TEST_METRICS}（请查看上方命令输出的判定结果）"
  fi
  info "头部测试指标与图片: ${HEAD_TEST_RESULT_DIR}"
  # 底部 test 阶段已随测试一起生成精度报告图，这里只做结果告知，不再问一次。
  echo
  info "头部精度报告图已随本轮测试生成于: ${HEAD_TEST_RESULT_DIR}"
}

# 手臂精度验证管线（流程 6）：手腕动捕测试，报告与图一律产出，并始终打印精度结果。
# 确认只在 run_accuracy_test 内部（开始末端精度验证）保留一次。
arm_validation_pipeline() {
  ARM_VALIDATION_TEST_DONE=false
  local report=""
  # 话题预检不在这里做：下面的 run_accuracy_test 会自己 check_live_topics，
  # 这里再查一遍只会把同一组话题有效信息重复打印一次。放在 isolate 里也更好——
  # 预检失败只让本路返回失败，不会 die 掉整个交互会话。

  # plot_accuracy 默认取 latest_accuracy_report()，而该函数按 mtime 排序，
  # 本轮报告若比上一轮旧就会被取错；这里先记下当前最新报告的指纹，
  # 测试后重新取一次并要求指纹变化，既避免误报旧报告，也给画图传准确路径。
  report="$(latest_accuracy_report)"
  local before=""
  if [[ -n "${report}" && -f "${report}" ]]; then
    before="$(stat -c '%Y:%s' "${report}" 2>/dev/null || echo unknown)"
  fi

  isolate run_accuracy_test
  if [[ "${ISOLATE_RC}" -ne 0 ]]; then
    warn "末端精度验证失败（exit=${ISOLATE_RC}）；不再展示或绘制目录中的旧报告。"
    return 0
  fi

  report="$(latest_accuracy_report)"
  if [[ -z "${report}" || ! -f "${report}" ]]; then
    warn "未生成本轮末端精度报告；不绘制目录中的旧报告。"
    return 0
  fi
  local after=""
  after="$(stat -c '%Y:%s' "${report}" 2>/dev/null || echo unknown)"
  if [[ -n "${before}" && "${before}" == "${after}" ]]; then
    warn "末端精度报告未更新（${report}）：本轮测试被取消或未落盘新结果；不展示旧报告。"
    return 0
  fi
  ARM_VALIDATION_TEST_DONE=true

  echo
  info "===== 末端精度结果 ====="
  info "精度报告: ${report}"
  # 有本轮报告就一律画图，不再询问；画图失败只告警，不影响已落盘的报告。
  echo
  isolate plot_accuracy "${report}"
  if [[ "${ISOLATE_RC}" -ne 0 ]]; then
    warn "末端精度报告图绘制失败（exit=${ISOLATE_RC}）。"
  else
    info "末端精度报告图已绘制。"
  fi
}

# 流程 2：先头部验证、后末端验证，严格串行（不沿用并行实现的 &/wait）。
# 两路各自预检、各自记录失败，一路失败仍继续另一路。
full_validation_workflow() {
  echo
  warn "完整精度验证将依次驱动头部、再驱动双臂；两路顺序执行，便于逐路确认运动。"
  warn "请确认头部与双臂零点均已正式写入并已重启运控。"

  # 棋盘位姿写在 URDF 里，是头部验证的比较基准，只能靠动捕重测，无法从精度报告反推。
  # 与流程 4 一样作为第一步无条件重测（ensure_head_board 不再跳过）；
  # board 步只做"动捕测量 + 写 URDF"，不驱动任何机构；它取消或失败只跳过头部段，末端段照走。
  info "[1/2 头部] 棋盘位姿与头部棋盘精度验证"
  if ensure_head_board; then
    head_validation_pipeline
  else
    HEAD_VALIDATION_TEST_DONE=false
    warn "棋盘位姿未写入（取消或棋盘动捕未通过）；跳过头部精度验证。"
  fi
  if [[ "${HEAD_VALIDATION_TEST_DONE}" != true ]]; then
    warn "头部精度验证未完成（取消或失败）；仍继续末端精度验证。"
  fi

  echo
  info "[2/2 末端] 手腕动捕精度验证"
  arm_validation_pipeline
  if [[ "${ARM_VALIDATION_TEST_DONE}" != true ]]; then
    warn "末端精度验证未完成（取消或失败）。"
  fi

  echo
  info "完整精度验证结束：头部=${HEAD_VALIDATION_TEST_DONE}，末端=${ARM_VALIDATION_TEST_DONE}。"
}

# 流程 4：头部精度验证。棋盘位姿是头部验证的比较基准，作为第一步无条件重测后写入 URDF。
head_validation_workflow() {
  ensure_head_board || return 0
  head_validation_pipeline
}

# 流程 6：末端精度验证。
arm_validation_workflow() {
  arm_validation_pipeline
}

single_step_tools_menu() {
  echo
  echo "单步工具（每一步都不会自动衔接下一步）："
  echo "  1) 只运动检查（驱动双臂，不采集）"
  echo "  2) 仅采集双臂数据"
  echo "  3) 优化最新且与当前机型匹配的 capture（含误差图）"
  echo "  4) 绘制优化前后误差图（不重新优化）"
  echo "  5) 写零 dry-run"
  echo "  6) 正式写零（含 dry-run 和二次确认）"
  echo "  7) 末端精度验证（含精度报告图）"
  echo "  8) 绘制最新精度报告"
  echo "  9) 头部联合标定（棋盘位姿 -> 相机采集 -> 优化/写零）"
  echo " 10) 头部写零后独立验证（含测试图片）"
  echo " 11) 头部+手腕并行精度验证（两路同时驱动）"
  echo "  0) 返回主菜单"
  local choice=""
  while true; do
    read -r -p "输入单步选项 [0-11，默认 0]: " choice
    case "${choice:-0}" in
      1) motion_preview ;;
      2) capture_data ;;
      3) optimize_capture; if [[ -f "${CALIBRATION_YAML}" && -n "${CAPTURE_PATH:-}" ]]; then plot_calibration; fi ;;
      4) plot_calibration ;;
      5) dry_run_zero ;;
      6) write_zero ;;
      7) run_accuracy_test; if [[ "${ACCURACY_TEST_DONE:-false}" == true ]]; then plot_accuracy; fi ;;
      8) plot_accuracy ;;
      9) run_head_calibration ;;
      10) run_head_validation ;;
      11) run_parallel_accuracy_validation ;;
      0) return 0 ;;
      *) warn "无效单步选项: ${choice}" ;;
    esac
  done
}
show_menu() {
  echo
  echo "选择操作："
  echo "  1) 完整零点标定（先头部，后末端）：头部采集 + 手臂采集（两段独立）+ 优化 + 误差报告 + 写零"
  echo "  2) 完整精度验证（先头部，后末端）：头部棋盘位姿 + 末端动捕 + 精度报告 + 结果"
  echo "  3) 头部零点标定：棋盘位姿 + 头部采集 + 优化 + 误差报告 + 写零"
  echo "  4) 头部精度验证：棋盘位姿 + 头部棋盘验证 + 精度报告 + 结果"
  echo "  5) 手臂零点标定：采集 + 优化 + 误差报告 + 写零"
  echo "  6) 末端精度验证：手腕动捕验证 + 精度报告 + 结果"
  echo "  7) 单步工具（只运动检查 / 单独优化 / 单独画图 / 单独写零 / 单独验证）"
  echo "  0) 退出"
  echo "提示：流程 1/3/5 都会重新采集；如需复用已有结果或只补做某一步，请用 7。"
  if [[ "${DISPLAY_ZERO_ISSUED}" == true ]]; then
    echo "[提示] 本轮已写入零点，运控尚未重启：新零点未生效，请先停止并重新启动运控再做精度验证。"
  fi
}

main() {
  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    echo "用法: bash ${BASH_SOURCE[0]}"
    echo "交互完成头部/双臂纯动捕零点标定与精度验证；当前支持 ROBOT_VERSION 45/52/56/62/63。"
    echo "可设置 MOCAP_FIXTURE=legacy|new，未设置时启动后交互选择。"
    return 0
  fi
  [[ $# -eq 0 ]] || die "未知参数: $*"

  source_ros_environment
  select_robot_version
  select_fixture
  select_mocap_source
  resolve_paths
  print_context

  while true; do
    show_menu
    local choice=""
    read -r -p "输入选项 [0-7，默认 1]: " choice
    case "${choice:-1}" in
      1) full_calibration_workflow ;;
      2) full_validation_workflow ;;
      3) head_calibration_workflow ;;
      4) head_validation_workflow ;;
      5) arm_calibration_workflow ;;
      6) arm_validation_workflow ;;
      7) single_step_tools_menu ;;
      0) break ;;
      *) warn "无效选项: ${choice}" ;;
    esac
  done
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
