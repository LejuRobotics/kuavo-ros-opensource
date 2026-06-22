#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
用法:
  交互式（推荐）:
    bash start_capture.sh

  命令行（可选，兼容自动化）:
    bash start_capture.sh --mode {pre|post} [--out_dir DIR] [--intrinsics_yaml_head PATH] [--intrinsics_yaml_left_wrist PATH] [--intrinsics_yaml_right_wrist PATH] [--write_every_n_frames N] [--display] [--with_motion] [--no_motion] [--loops N] [--auto_exit] [--no_auto_exit]

说明:
  - 本脚本会启动 record_chessboard_pose_to_csv.py
  - --mode pre/post 用于区分“标定前/后”，并写入不同 CSV 文件名
  - 内参来源:
      - 默认从对应相机的 /camera_info 订阅（--intrinsics_yaml 不传）
      - 若传 --intrinsics_yaml，则使用该 YAML 覆盖内参（用于“使用你校准后的内参”场景）
  - 可选同时启动 demo 关节运动（默认开启）：
      - 头部：demos/kuavo_head_demo/head_table_publisher.py（teach_head_joint.json）
      - 左右手：demos/kuavo_both_arms/both_arms_table_publisher.py（teach_left_joint.json + teach_right_joint.json）
    之所以左右手用“合并下发”，是为了避免两个进程同时往 /kuavo_arm_traj 发导致打架。

输出:
  - 默认输出到: src/Camera_Calibration/output_csv/chessboard_pose/<camera>/
  - 文件名: board_pose_<camera>_<mode>_<timestamp>.csv
EOF
}

MODE=""
OUT_DIR=""
INTRINSICS_YAML_HEAD=""
INTRINSICS_YAML_LEFT_WRIST=""
INTRINSICS_YAML_RIGHT_WRIST=""
WRITE_EVERY=1
DISPLAY=false
WITH_MOTION=true
LOOPS=1
AUTO_EXIT=true

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="${2:-}"; shift 2 ;;
    --out_dir) OUT_DIR="${2:-}"; shift 2 ;;
    --intrinsics_yaml_head) INTRINSICS_YAML_HEAD="${2:-}"; shift 2 ;;
    --intrinsics_yaml_left_wrist) INTRINSICS_YAML_LEFT_WRIST="${2:-}"; shift 2 ;;
    --intrinsics_yaml_right_wrist) INTRINSICS_YAML_RIGHT_WRIST="${2:-}"; shift 2 ;;
    --write_every_n_frames) WRITE_EVERY="${2:-}"; shift 2 ;;
    --display) DISPLAY=true; shift ;;
    --with_motion) WITH_MOTION=true; shift ;;
    --no_motion) WITH_MOTION=false; shift ;;
    --loops) LOOPS="${2:-}"; shift 2 ;;
    --auto_exit) AUTO_EXIT=true; shift ;;
    --no_auto_exit) AUTO_EXIT=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "${MODE}" ]]; then
  echo ""
  echo "请选择内参模式："
  echo "  1) 标定前（pre）：使用在线 camera_info（未校准内参）"
  echo "  2) 标定后（post）：使用你校准后的内参 YAML 覆盖"
  read -r -p "输入 1/2 并回车: " MODE_CHOICE
  case "${MODE_CHOICE}" in
    1) MODE="pre" ;;
    2) MODE="post" ;;
    *) echo "[ERROR] 仅支持输入 1/2" >&2; exit 1 ;;
  esac
fi

[[ "$MODE" == "pre" || "$MODE" == "post" ]] || { echo "--mode 仅支持 pre/post" >&2; exit 1; }

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
CC_DIR="${WS_DIR}/src/Camera_Calibration"
SCRIPT_DIR="${CC_DIR}/chessboard_pose_logger"

if [[ -z "${OUT_DIR}" ]]; then
  OUT_DIR="${CC_DIR}/output_csv/chessboard_pose"
fi
mkdir -p "${OUT_DIR}"

if [[ "${MODE}" == "post" ]]; then
  # post 模式仅用于区分输出文件名（pre/post）；内参默认仍来自 /camera_info。
  # 只有显式传入 --intrinsics_yaml_* 时才会使用 YAML 覆盖内参。
  :
fi

echo "[INFO] workspace: ${WS_DIR}"
echo "[INFO] mode: ${MODE}"
echo "[INFO] out_dir: ${OUT_DIR}"
echo "[INFO] intrinsics_yaml_head: ${INTRINSICS_YAML_HEAD:-<camera_info>}"
echo "[INFO] intrinsics_yaml_left_wrist: ${INTRINSICS_YAML_LEFT_WRIST:-<camera_info>}"
echo "[INFO] intrinsics_yaml_right_wrist: ${INTRINSICS_YAML_RIGHT_WRIST:-<camera_info>}"

source "${WS_DIR}/devel/setup.bash" >/dev/null 2>&1 || true

PIDS=()
cleanup() {
  # 结束所有后台进程（采集 + 运动）
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
}
trap cleanup INT TERM EXIT

start_one() {
  local cam="$1"
  local image_topic="$2"
  local info_topic="$3"
  local intr_yaml="$4"
  local keyframe_flag_topic="$5"

  mkdir -p "${OUT_DIR}/${cam}"
  local out_csv="${OUT_DIR}/${cam}/board_pose_${cam}_${MODE}.csv"
  # 固定文件名模式下，为避免把历史实时采集数据追加进来，这里每次启动都清空旧文件。
  rm -f "${out_csv}"

  echo "[INFO] ---- start ${cam} ----"
  echo "[INFO] image_topic: ${image_topic}"
  echo "[INFO] camera_info_topic: ${info_topic}"
  echo "[INFO] out_csv: ${out_csv}"
  echo "[INFO] intrinsics_yaml: ${intr_yaml:-<camera_info>}"

  local -a args=(
    --image_topic "${image_topic}"
    --camera_info_topic "${info_topic}"
    --out_csv "${out_csv}"
    --points_x 11
    --points_y 8
    --square_size 0.03
    --write_every_n_frames "${WRITE_EVERY}"
    --keyframe_flag_topic "${keyframe_flag_topic}"
  )

  if [[ -n "${intr_yaml}" ]]; then
    args+=( --intrinsics_source yaml --intrinsics_yaml "${intr_yaml}" )
  else
    args+=( --intrinsics_source camera_info )
  fi

  if $DISPLAY; then
    args+=( --display )
  fi

  python3 "${SCRIPT_DIR}/record_chessboard_pose_to_csv.py" "${args[@]}" &
  PIDS+=( "$!" )
}

start_one "head" "/head_camera/color/image_raw" "/head_camera/color/camera_info" "${INTRINSICS_YAML_HEAD}" "/head_keyframe_flag"
start_one "left_wrist" "/left_wrist_camera/color/image_raw" "/left_wrist_camera/color/camera_info" "${INTRINSICS_YAML_LEFT_WRIST}" "/left_wrist_keyframe_flag"
start_one "right_wrist" "/right_wrist_camera/color/image_raw" "/right_wrist_camera/color/camera_info" "${INTRINSICS_YAML_RIGHT_WRIST}" "/right_wrist_keyframe_flag"

if ${WITH_MOTION}; then
  echo ""
  echo "[INFO] ---- start demo joint motion (loops=${LOOPS}) ----"
  TEACH_DIR="${CC_DIR}/teach_capture_output"
  PID_HEAD_MOTION=""
  PID_ARMS_MOTION=""

  # head motion
  python3 "${CC_DIR}/demos/kuavo_head_demo/head_table_publisher.py" \
    _play_loop_count:="${LOOPS}" \
    _teach_json_path:="${TEACH_DIR}/teach_head_joint.json" &
  PID_HEAD_MOTION="$!"
  PIDS+=( "${PID_HEAD_MOTION}" )

  # both arms motion (merged publisher to avoid /kuavo_arm_traj conflict)
  python3 "${CC_DIR}/demos/kuavo_both_arms/both_arms_table_publisher.py" \
    _play_loop_count:="${LOOPS}" \
    _set_external_control_mode:=true \
    _enable_wbc_arm_trajectory_control:=true \
    _hold_sec:=5.0 \
    _teach_left_json:="${TEACH_DIR}/teach_left_joint.json" \
    _teach_right_json:="${TEACH_DIR}/teach_right_joint.json" &
  PID_ARMS_MOTION="$!"
  PIDS+=( "${PID_ARMS_MOTION}" )
else
  echo ""
  echo "[INFO] demo joint motion disabled (--no_motion)."
fi

echo ""
echo "[INFO] 三路相机采集已启动。"

if ${WITH_MOTION} && ${AUTO_EXIT}; then
  echo "[INFO] 等待 demo 运动进程结束（完成后将自动退出并结束采集进程）..."
  # 比等待 done 话题更可靠：done 通常只发一次且非 latched，可能被 rostopic echo 漏掉。
  if [[ -n "${PID_HEAD_MOTION:-}" ]]; then
    wait "${PID_HEAD_MOTION}" || true
  fi
  if [[ -n "${PID_ARMS_MOTION:-}" ]]; then
    wait "${PID_ARMS_MOTION}" || true
  fi

  # 给采集脚本一点时间把最后几帧写入 CSV
  sleep 1.0
  echo "[INFO] 运动已结束，自动退出。"
  exit 0
else
  echo "[INFO] 按 Ctrl-C 结束（会同时结束采集/运动进程）。"
  wait
fi

