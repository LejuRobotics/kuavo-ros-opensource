#!/usr/bin/env bash
set -e

chmod +x "$0" 2>/dev/null || true

usage() {
  cat <<'EOF'
用法: run_chessboard_calibration.sh [capture|optimize|test|move] [--build] [--loops N] [--out_dir DIR] [--robot_layout biped52|biped56|wheel62]

说明:
  - 运行后可选择：头部标定 / 右手标定 / 左手标定 / 全部串行
  - capture: 启动对应 demo 的 capture_to_csv（手动触发采样）
  - optimize: 启动对应 demo 的 optimize_from_csv（从 CSV 读取）
  - test: 读取 teach_*_joint_test.json 下发测试姿态并采数到带 _test 后缀目录，采数完自动画图输出测试图片
  - move: 仅下发 teach JSON 中的多姿态关节轨迹（不采数、不写 CSV），适用于“内参标定只需运动覆盖”的场景
  - 机型：默认读 ROBOT_VERSION（52→biped52，56→biped56，62/63→wheel62）；可用 --robot_layout 覆盖

选项:
  --build       先编译 robot_calibration、robot_calibration_msgs 和 kuavo_msgs
  --loops N     仅头部有效：关键帧轨迹循环次数（默认 1）
  --out_dir DIR 覆盖 CSV 输出目录（不填则使用默认输出目录）
  --robot_layout biped52|biped56|wheel62  覆盖 ROBOT_VERSION 自动识别
EOF
}

MODE=""
BUILD=false
LOOPS=1
OUT_DIR=""
ROBOT_LAYOUT_ARG=""
CONTROL_TOPIC="/rgb_calib/control"

if [[ $# -gt 0 ]]; then
  case "$1" in
    capture|optimize|test|move)
      MODE="$1"
      shift
      ;;
  esac
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) BUILD=true; shift ;;
    --loops) LOOPS="${2:-}"; shift 2 ;;
    --out_dir) OUT_DIR="${2:-}"; shift 2 ;;
    --robot_layout) ROBOT_LAYOUT_ARG="${2:-}"; shift 2 ;;
    --control_topic) CONTROL_TOPIC="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage; exit 1 ;;
  esac
done

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CC_DIR="${WS_DIR}/src/Camera_Calibration"
TEACH_DIR="${CC_DIR}/teach_capture_output"
# launch 内 camera_calib_root 默认读此环境变量（不注册 ROS 包）
export CAMERA_CALIB_ROOT="${CC_DIR}"

die() {
  echo "[ERROR] $1" >&2
  exit 1
}

# 机型布局：ROBOT_VERSION 自动识别，或 --robot_layout 覆盖
resolve_robot_layout() {
  if [[ -n "${ROBOT_LAYOUT_ARG}" ]]; then
    case "${ROBOT_LAYOUT_ARG}" in
      biped52|biped56|wheel62) echo "${ROBOT_LAYOUT_ARG}"; return 0 ;;
      *) die "无效的 --robot_layout: ${ROBOT_LAYOUT_ARG}（仅 biped52|biped56|wheel62）" ;;
    esac
  fi
  local rv="${ROBOT_VERSION:-}"
  case "${rv}" in
    52) echo "biped52" ;;
    56) echo "biped56" ;;
    62|63) echo "wheel62" ;;
    "")
      echo "biped52"
      ;;
    *)
      echo "[WARN] 未知 ROBOT_VERSION=${rv}，回退 biped52" >&2
      echo "biped52"
      ;;
  esac
}

ROBOT_LAYOUT="$(resolve_robot_layout)"
if [[ "${ROBOT_LAYOUT}" == "wheel62" ]]; then
  NOMINAL_URDF="${CC_DIR}/biped_v3_arm_s62.urdf"
  CALIBRATED_URDF="${CC_DIR}/biped_v3_arm_s62_calibrated.urdf"
elif [[ "${ROBOT_LAYOUT}" == "biped56" ]]; then
  NOMINAL_URDF="${CC_DIR}/biped_v3_arm_s56.urdf"
  CALIBRATED_URDF="${CC_DIR}/biped_v3_arm_s56_calibrated.urdf"
else
  NOMINAL_URDF="${CC_DIR}/biped_v3_arm.urdf"
  CALIBRATED_URDF="${CC_DIR}/biped_v3_arm_calibrated.urdf"
fi

banner() {
  echo ""
  echo "============================================================"
  echo "$1"
  echo "============================================================"
}

pub_control() {
  local msg="$1"
  # rostopic pub 需要 ROS 环境已 source
  rostopic pub -1 "${CONTROL_TOPIC}" std_msgs/String "data: \"${msg}\"" >/dev/null 2>&1 || true
}

select_cam_and_wait() {
  local cam_name="$1"
  echo "[INFO] 标定控制：SELECT|${cam_name} -> ${CONTROL_TOPIC}"
  pub_control "SELECT|${cam_name}"
  # 给上位机一点时间切换订阅并等首帧
  sleep 0.5
}

select_cams_and_wait() {
  # 一次性声明本轮会用到的相机（连续发 SELECT），最后统一等待一次，避免逐个等待导致时序被拉长。
  # 用法：select_cams_and_wait head_camera right_wrist_camera left_wrist_camera
  local cam_name
  for cam_name in "$@"; do
    echo "[INFO] 标定控制：SELECT|${cam_name} -> ${CONTROL_TOPIC}"
    pub_control "SELECT|${cam_name}"
  done
  # 给上位机一点时间切换订阅并等首帧
  sleep 0.5
}

motion_done_cam() {
  local cam_name="$1"
  echo "[INFO] 标定控制：MOTION_DONE|${cam_name} -> ${CONTROL_TOPIC}"
  pub_control "MOTION_DONE|${cam_name}"
  sleep 0.2
}

session_done() {
  echo "[INFO] 标定控制：SESSION_DONE -> ${CONTROL_TOPIC}"
  pub_control "SESSION_DONE"
}

read_launch_param_default() {
  local launch_file="$1"
  local param_name="$2"
  local default_value="$3"
  python3 - "$launch_file" "$param_name" "$default_value" <<'PY'
import sys
import xml.etree.ElementTree as ET

launch_file, param_name, default_value = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    root = ET.parse(launch_file).getroot()
except Exception:
    print(default_value)
    raise SystemExit(0)

for elem in root.iter("param"):
    if elem.get("name") == param_name and elem.get("value") is not None:
        print(elem.get("value"))
        raise SystemExit(0)

print(default_value)
PY
}

count_csv_files() {
  local d="$1"
  # 只统计常见 csv（忽略隐藏文件）
  if [[ ! -d "$d" ]]; then
    echo 0
    return 0
  fi
  ls -1 "$d"/*.csv 2>/dev/null | wc -l | tr -d ' '
}

# 检查话题已注册且存在发布者。不用 rostopic echo 收图：在部分环境（如 root + 大图像）会段错误。
check_topic_once() {
  local topic="$1"
  local timeout_sec="${2:-5}"
  if ! timeout "${timeout_sec}" rostopic list 2>/dev/null | grep -Fxq "${topic}"; then
    return 1
  fi
  local info
  info="$(timeout "${timeout_sec}" rostopic info "${topic}" 2>/dev/null)" || return 1
  echo "${info}" | sed -n '/^Publishers:/,/^Subscribers:/p' | grep -q '\* /' && return 0
  return 1
}

precheck_ros_and_topics() {
  local demo_choice="$1"
  local stage_mode="${2:-}"
  local ros_timeout_sec=5
  local topic_timeout_sec=5
  local image_topic=""
  local info_topic=""

  echo "[INFO] 预检 ROS master 与关键话题..."
  if ! timeout "${ros_timeout_sec}" rostopic list >/dev/null 2>&1; then
    echo "[ERROR] 无法连接 ROS master。请确认 roscore/网络/ROS_MASTER_URI 正常。" >&2
    return 1
  fi

  # optimize_from_csv 仅读 CSV + URDF，不订阅图像；无需相机与关节话题在线。
  if [[ "${stage_mode}" == "optimize" ]]; then
    echo "[INFO] 预检通过：optimize 仅校验 ROS master（离线优化，不依赖相机与 /sensors_data_raw）。"
    return 0
  fi

  if ! check_topic_once "/sensors_data_raw" "${topic_timeout_sec}"; then
    echo "[ERROR] 预检失败：/sensors_data_raw 在 ${topic_timeout_sec}s 内无消息。" >&2
    return 1
  fi

  case "${demo_choice}" in
    1)
      image_topic="/head_camera/color/image_raw"
      info_topic="/head_camera/color/camera_info"
      ;;
    2)
      image_topic="/right_wrist_camera/color/image_raw"
      info_topic="/right_wrist_camera/color/camera_info"
      ;;
    3)
      image_topic="/left_wrist_camera/color/image_raw"
      info_topic="/left_wrist_camera/color/camera_info"
      ;;
    4)
      # 全部：逐个检查三套相机话题
      if ! check_topic_once "/head_camera/color/image_raw" "${topic_timeout_sec}"; then
        echo "[ERROR] 预检失败：/head_camera/color/image_raw 在 ${topic_timeout_sec}s 内无消息。" >&2
        return 1
      fi
      if ! check_topic_once "/head_camera/color/camera_info" "${topic_timeout_sec}"; then
        echo "[ERROR] 预检失败：/head_camera/color/camera_info 在 ${topic_timeout_sec}s 内无消息。" >&2
        return 1
      fi
      if ! check_topic_once "/right_wrist_camera/color/image_raw" "${topic_timeout_sec}"; then
        echo "[ERROR] 预检失败：/right_wrist_camera/color/image_raw 在 ${topic_timeout_sec}s 内无消息。" >&2
        return 1
      fi
      if ! check_topic_once "/right_wrist_camera/color/camera_info" "${topic_timeout_sec}"; then
        echo "[ERROR] 预检失败：/right_wrist_camera/color/camera_info 在 ${topic_timeout_sec}s 内无消息。" >&2
        return 1
      fi
      if ! check_topic_once "/left_wrist_camera/color/image_raw" "${topic_timeout_sec}"; then
        echo "[ERROR] 预检失败：/left_wrist_camera/color/image_raw 在 ${topic_timeout_sec}s 内无消息。" >&2
        return 1
      fi
      if ! check_topic_once "/left_wrist_camera/color/camera_info" "${topic_timeout_sec}"; then
        echo "[ERROR] 预检失败：/left_wrist_camera/color/camera_info 在 ${topic_timeout_sec}s 内无消息。" >&2
        return 1
      fi
      echo "[INFO] 预检通过：三套相机话题可读。"
      return 0
      ;;
  esac

  if [[ -n "${image_topic}" ]] && ! check_topic_once "${image_topic}" "${topic_timeout_sec}"; then
    echo "[ERROR] 预检失败：${image_topic} 在 ${topic_timeout_sec}s 内无消息。" >&2
    return 1
  fi
  if [[ -n "${info_topic}" ]] && ! check_topic_once "${info_topic}" "${topic_timeout_sec}"; then
    echo "[ERROR] 预检失败：${info_topic} 在 ${topic_timeout_sec}s 内无消息。" >&2
    return 1
  fi

  echo "[INFO] 预检通过：ROS 连接正常，关键话题可读。"
}

echo ""
echo "请选择标定 demo："
echo "  1) 头部（kuavo_head_demo）"
echo "  2) 右手（kuavo_right_wrist）"
echo "  3) 左手（kuavo_left_wrist）"
echo "  4) 全部同时（头部 + 左右手并行；左右手合并下发避免 /kuavo_arm_traj 打架）"
read -r -p "输入 1/2/3/4 并回车: " CHOICE

case "${CHOICE}" in
  1)
    DEMO_NAME="kuavo_head"
    DEMO_LAUNCH="${CC_DIR}/demos/kuavo_head_demo/kuavo_head_demo.launch"
    DEFAULT_CSV_DIR="${CC_DIR}/output_csv/kuavo_head"
    DEFAULT_TEST_CSV_DIR="${CC_DIR}/output_csv/kuavo_head_test"
    PLOT_OUT_DIR="${CC_DIR}/output/kuavo_head"
    CAMERA_TIP_LINK="head_camera_color_optical_frame"
    SENSOR_NAME="camera_to_base"
    FK_ROOT="zarm_l1_ref_link"
    REMAP_TO_CENTER="--remap_to_center"
    TEST_TEACH_JSON="${CC_DIR}/teach_capture_output/teach_head_joint_test.json"
    ;;
  2)
    DEMO_NAME="kuavo_right_wrist"
    DEMO_LAUNCH="${CC_DIR}/demos/kuavo_right_wrist/kuavo_right_wrist_demo.launch"
    DEFAULT_CSV_DIR="${CC_DIR}/output_csv/kuavo_right_wrist"
    DEFAULT_TEST_CSV_DIR="${CC_DIR}/output_csv/kuavo_right_wrist_test"
    PLOT_OUT_DIR="${CC_DIR}/output/kuavo_right_wrist"
    CAMERA_TIP_LINK="right_wrist_camera_color_optical_frame"
    SENSOR_NAME="right_wrist_camera_to_base"
    FK_ROOT="zarm_l1_ref_link"
    REMAP_TO_CENTER="--remap_to_center"
    TEST_TEACH_JSON="${CC_DIR}/teach_capture_output/teach_right_joint_test.json"
    ;;
  3)
    DEMO_NAME="kuavo_left_wrist"
    DEMO_LAUNCH="${CC_DIR}/demos/kuavo_left_wrist/kuavo_left_wrist_demo.launch"
    DEFAULT_CSV_DIR="${CC_DIR}/output_csv/kuavo_left_wrist"
    DEFAULT_TEST_CSV_DIR="${CC_DIR}/output_csv/kuavo_left_wrist_test"
    PLOT_OUT_DIR="${CC_DIR}/output/kuavo_left_wrist"
    CAMERA_TIP_LINK="left_wrist_camera_color_optical_frame"
    SENSOR_NAME="left_wrist_camera_to_base"
    FK_ROOT="zarm_l1_ref_link"
    REMAP_TO_CENTER="--remap_to_center"
    TEST_TEACH_JSON="${CC_DIR}/teach_capture_output/teach_left_joint_test.json"
    ;;
  4)
    DEMO_NAME="all_parallel"
    ;;
  *)
    echo "[ERROR] 仅支持输入 1/2/3/4" >&2
    exit 1
    ;;
esac

if [[ -z "${MODE}" ]]; then
  echo ""
  echo "请选择运行阶段："
  echo "  1) 采数（capture -> 写 CSV）"
  echo "  2) 优化（optimize -> 读 CSV 并输出结果）"
  echo "  3) 测试（test -> 读 teach_*_joint_test.json 下发并写 CSV + 自动画图）"
  echo "  4) 仅运动（move -> 只下发多姿态轨迹，不采数）"
  read -r -p "输入 1/2/3/4 并回车: " STAGE_CHOICE
  case "${STAGE_CHOICE}" in
    1) MODE="capture" ;;
    2) MODE="optimize" ;;
    3) MODE="test" ;;
    4) MODE="move" ;;
    *) die "仅支持输入 1/2/3/4" ;;
  esac
fi

#
# CHOICE=4（全部并行）现在支持 capture/optimize/test/move：
# - capture/test/move：头部与左右手并行（左右手合并轨迹避免控制冲突）。
# - optimize：三路 roslaunch 顺序执行（head→right→left），避免 /robot_description 竞态与终端刷屏。
#

if [[ -z "${OUT_DIR}" ]]; then
  if [[ "${MODE}" == "test" ]]; then
    OUT_DIR="${DEFAULT_TEST_CSV_DIR}"
  elif [[ "${MODE}" == "move" ]]; then
    OUT_DIR=""
  else
    OUT_DIR="${DEFAULT_CSV_DIR}"
  fi
fi

echo "[INFO] 工作空间: ${WS_DIR}"
echo "[INFO] robot_layout: ${ROBOT_LAYOUT} (ROBOT_VERSION=${ROBOT_VERSION:-未设置})"
echo "[INFO] nominal URDF: ${NOMINAL_URDF}"
echo "[INFO] calibrated URDF: ${CALIBRATED_URDF}"
echo "[INFO] demo: ${DEMO_NAME}"
if [[ -n "${OUT_DIR}" ]]; then
  echo "[INFO] CSV 输出目录: ${OUT_DIR}"
fi
if [[ -n "${DEMO_LAUNCH:-}" ]]; then
  echo "[INFO] demo launch: ${DEMO_LAUNCH}"
fi

if [[ "${CHOICE}" != "4" && ! -f "${DEMO_LAUNCH}" ]]; then
  echo "[ERROR] 找不到 demo launch 文件: ${DEMO_LAUNCH}" >&2
  exit 1
fi

if $BUILD; then
  echo "[INFO] 编译 robot_calibration, robot_calibration_msgs, kuavo_msgs ..."
  cd "${WS_DIR}"
  if command -v catkin >/dev/null 2>&1; then
    catkin build robot_calibration robot_calibration_msgs kuavo_msgs
  else
    catkin_make --pkg robot_calibration robot_calibration_msgs kuavo_msgs
  fi
  echo "[INFO] 编译完成"
fi

echo "[INFO] source devel/setup.bash"
source "${WS_DIR}/devel/setup.bash"

if [[ "${CHOICE}" == "4" ]]; then
  banner "全部并行：头部 + 左右手（stage=${MODE}）"
  precheck_ros_and_topics "${CHOICE}" "${MODE}" || exit 1
  if [[ "${MODE}" == "optimize" ]]; then
    echo "[INFO] optimize：三路将顺序执行 roslaunch（head → right_wrist → left_wrist），避免"
    echo "       并行时抢占 /robot_description 导致离群点/FK 异常，并避免终端日志交错刷屏。"
    echo "[INFO] optimize：三路将顺序执行 roslaunch（head → right_wrist → left_wrist），避免"
    echo "       并行时抢占 /robot_description 导致离群点/FK 异常。"
    echo "[INFO] 不再落盘 _parallel_optimize_logs；roslaunch 输出直接打印到终端。"
    echo "[INFO] 每路 optimize 成功后立即 plot_board_error_from_csv（与选项 1/2/3 单路 optimize 一致），"
    echo "       PNG 写入各 output/kuavo_*/ ，以免共享 URDF 被下一路覆盖后无法正确出图。"
  fi

  # 统一：若用户传了 --out_dir，则按子目录分流，避免互相覆盖
  out_dir_for() {
    local cam="$1"
    local default_dir="$2"
    if [[ -n "${OUT_DIR}" ]]; then
      echo "${OUT_DIR}/${cam}"
    else
      echo "${default_dir}"
    fi
  }

  HEAD_LAUNCH="${CC_DIR}/demos/kuavo_head_demo/kuavo_head_demo.launch"
  RIGHT_LAUNCH="${CC_DIR}/demos/kuavo_right_wrist/kuavo_right_wrist_demo.launch"
  LEFT_LAUNCH="${CC_DIR}/demos/kuavo_left_wrist/kuavo_left_wrist_demo.launch"

  # 并行：统一后台进程管理（参考 chessboard_pose_logger/start_capture.sh）
  PIDS=()
  cleanup_bg() {
    for pid in "${PIDS[@]:-}"; do
      kill "${pid}" 2>/dev/null || true
    done
  }
  trap cleanup_bg INT TERM EXIT

  ensure_running_or_die() {
    local tag="$1"
    sleep 5
    for pid in "${PIDS[@]:-}"; do
      if ! kill -0 "${pid}" 2>/dev/null; then
        die "(${tag}) 后台进程提前退出（PID=${pid}）。请查看 ~/.ros/log/*/roslaunch-*.log。"
      fi
    done
  }

  # 旧版「并行 optimize」已弃用：多 roslaunch 同时写参数服务器会导致 /robot_description 与优化结果互相干扰。

  start_motion_head_then_arms() {
    local head_json="$1"
    local left_json="$2"
    local right_json="$3"

    echo "[INFO] (all) 顺序执行：先头部，再左右手（合并下发）"
    python3 "${CC_DIR}/demos/kuavo_head_demo/head_table_publisher.py" \
      _play_loop_count:="${LOOPS}" \
      _robot_layout:="${ROBOT_LAYOUT}" \
      _teach_json_path:="${head_json}"

    python3 "${CC_DIR}/demos/kuavo_both_arms/both_arms_table_publisher.py" \
      _play_loop_count:="${LOOPS}" \
      _set_external_control_mode:=true \
      _enable_arm_quick_mode:=true \
      _robot_layout:="${ROBOT_LAYOUT}" \
      _hold_sec:=5.0 \
      _teach_left_json:="${left_json}" \
      _teach_right_json:="${right_json}"
  }

  start_capture_launches_parallel() {
    local out_head="$1"
    local out_right="$2"
    local out_left="$3"

    mkdir -p "${out_head}" "${out_right}" "${out_left}"
    echo "[INFO] (head) capture 输出: ${out_head}"
    echo "[INFO] (right_wrist) capture 输出: ${out_right}"
    echo "[INFO] (left_wrist) capture 输出: ${out_left}"

    # 三路同时 launch 须区分 rsp_name / capture_name，否则 ROS 报「同名节点注册」并互相踢掉
    roslaunch "${HEAD_LAUNCH}" "csv_dir:=${out_head}" "do_capture_to_csv:=true" "do_optimize_from_csv:=false" "do_calibrate_manual:=false" \
      "robot_layout:=${ROBOT_LAYOUT}" \
      "rsp_name:=rsp_head" "capture_name:=cap_capture_head" &
    PIDS+=( "$!" )
    roslaunch "${RIGHT_LAUNCH}" "csv_dir:=${out_right}" "do_capture_to_csv:=true" "do_optimize_from_csv:=false" "do_calibrate_manual:=false" \
      "robot_layout:=${ROBOT_LAYOUT}" \
      "rsp_name:=rsp_right" "capture_name:=cap_capture_right" &
    PIDS+=( "$!" )
    roslaunch "${LEFT_LAUNCH}" "csv_dir:=${out_left}" "do_capture_to_csv:=true" "do_optimize_from_csv:=false" "do_calibrate_manual:=false" \
      "robot_layout:=${ROBOT_LAYOUT}" \
      "rsp_name:=rsp_left" "capture_name:=cap_capture_left" &
    PIDS+=( "$!" )
  }

  check_csv_or_die() {
    local cam="$1"
    local dir="$2"
    local tag="$3"
    local csv_count
    csv_count="$(count_csv_files "${dir}")"
    if [[ "${csv_count}" -le 0 ]]; then
      die "(${cam}) ${tag} 未生成 CSV：${dir}"
    fi
    echo "[INFO] (${cam}) ${tag} 完成：CSV=${csv_count}"
  }

  case "${MODE}" in
    move)
      echo "[INFO] 全部同时：头部 + 左右手（并行；左右手使用「合并下发」避免在 /kuavo_arm_traj 上打架；全部结束后再发 SESSION_DONE）"

      # 先一次性声明三路相机，再执行运动；结束后统一 SESSION_DONE
      select_cams_and_wait "head_camera" "right_wrist_camera" "left_wrist_camera"
      start_motion_head_then_arms \
        "${TEACH_DIR}/teach_head_joint.json" \
        "${TEACH_DIR}/teach_left_joint.json" \
        "${TEACH_DIR}/teach_right_joint.json"
      motion_done_cam "head_camera"
      motion_done_cam "right_wrist_camera"
      motion_done_cam "left_wrist_camera"
      session_done
      ;;
    capture)
      out_head="$(out_dir_for "head" "${CC_DIR}/output_csv/kuavo_head")"
      out_right="$(out_dir_for "right_wrist" "${CC_DIR}/output_csv/kuavo_right_wrist")"
      out_left="$(out_dir_for "left_wrist" "${CC_DIR}/output_csv/kuavo_left_wrist")"

      start_capture_launches_parallel "${out_head}" "${out_right}" "${out_left}"
      ensure_running_or_die "capture"

      # 顺序触发采样：先头部，再合并左右臂（避免一起动导致不好观察）
      select_cam_and_wait "head_camera"
      start_motion_head_then_arms \
        "${TEACH_DIR}/teach_head_joint.json" \
        "${TEACH_DIR}/teach_left_joint.json" \
        "${TEACH_DIR}/teach_right_joint.json"
      motion_done_cam "head_camera"

      select_cam_and_wait "right_wrist_camera"
      select_cam_and_wait "left_wrist_camera"
      motion_done_cam "right_wrist_camera"
      motion_done_cam "left_wrist_camera"

      # 给 capture_to_csv 落盘一点时间
      sleep 2
      cleanup_bg
      PIDS=()

      check_csv_or_die "head" "${out_head}" "capture"
      check_csv_or_die "right_wrist" "${out_right}" "capture"
      check_csv_or_die "left_wrist" "${out_left}" "capture"
      ;;
    test)
      out_head="$(out_dir_for "head" "${CC_DIR}/output_csv/kuavo_head_test")"
      out_right="$(out_dir_for "right_wrist" "${CC_DIR}/output_csv/kuavo_right_wrist_test")"
      out_left="$(out_dir_for "left_wrist" "${CC_DIR}/output_csv/kuavo_left_wrist_test")"

      # 先校验测试 JSON 存在（沿用原来的严格检查语义）
      [[ -f "${TEACH_DIR}/teach_head_joint_test.json" ]] || die "(head) 找不到测试关节角 JSON：${TEACH_DIR}/teach_head_joint_test.json"
      [[ -f "${TEACH_DIR}/teach_right_joint_test.json" ]] || die "(right_wrist) 找不到测试关节角 JSON：${TEACH_DIR}/teach_right_joint_test.json"
      [[ -f "${TEACH_DIR}/teach_left_joint_test.json" ]] || die "(left_wrist) 找不到测试关节角 JSON：${TEACH_DIR}/teach_left_joint_test.json"

      start_capture_launches_parallel "${out_head}" "${out_right}" "${out_left}"
      ensure_running_or_die "test"

      select_cam_and_wait "head_camera"
      start_motion_head_then_arms \
        "${TEACH_DIR}/teach_head_joint_test.json" \
        "${TEACH_DIR}/teach_left_joint_test.json" \
        "${TEACH_DIR}/teach_right_joint_test.json"
      motion_done_cam "head_camera"

      select_cam_and_wait "right_wrist_camera"
      select_cam_and_wait "left_wrist_camera"
      motion_done_cam "right_wrist_camera"
      motion_done_cam "left_wrist_camera"

      sleep 2
      cleanup_bg
      PIDS=()

      check_csv_or_die "head" "${out_head}" "test"
      check_csv_or_die "right_wrist" "${out_right}" "test"
      check_csv_or_die "left_wrist" "${out_left}" "test"

      # 自动画图：全部并行 test 也输出测试图（仅用原始 URDF 做 FK，不依赖 optimize 产物）
      plot_test_nominal_only() {
        local tag="$1"
        local csv_d="$2"
        local plot_dir="$3"
        local cam_tip="$4"
        local sensor="$5"
        local pos_m rot_deg metrics prc

        # test 阶段不做二次离群点剔除；阈值留空/0
        pos_m="0"
        rot_deg="0"
        metrics="${plot_dir}/test_metrics.txt"
        mkdir -p "${plot_dir}"

        echo ""
        echo "[INFO] ${tag} test plot_board_error_from_csv（nominal only）→ ${plot_dir}"
        set +e
        {
          echo "============================================================"
          echo "相机标定测试指标报告"
          echo "  demo: ${tag}"
          echo "  时间: $(date -Iseconds)"
          echo "  测试 CSV 目录: ${csv_d}"
          echo "  测试结果目录: ${plot_dir}"
          echo "  FK 使用 URDF: ${NOMINAL_URDF}（nominal only）"
          echo "============================================================"
          echo ""
          python3 "${CC_DIR}/plot_board_error_from_csv.py" \
            --csv_dir "${csv_d}" \
            --nominal_urdf "${NOMINAL_URDF}" \
            --output_dir "${plot_dir}" \
            --use_nominal_only \
            --camera_tip_link "${cam_tip}" \
            --sensor_name "${sensor}" \
            --fk_root "zarm_l1_ref_link" \
            --points_x 11 --points_y 8 --square_size 0.03 \
            --ignore_optimization_used_sample_ids \
            --out_err_png "${plot_dir}/board_pose_error_pre_post_vs_urdf_test.png" \
            --out_abs_png "${plot_dir}/board_pose_bars_vs_urdf_test.png" \
            --remap_to_center \
            2>&1
          echo ""
          echo "---------- 结束 ----------"
        } > "${metrics}" 2>&1
        prc=$?
        set -e
        if [[ "${prc}" -ne 0 ]]; then
          echo "[WARN] ${tag} 测试自动画图失败（不影响测试 CSV 输出），请查看 ${metrics}" >&2
        else
          echo "[INFO] ${tag} 测试图与指标已输出: ${plot_dir}"
        fi
      }

      plot_test_nominal_only "head" "${out_head}" "${CC_DIR}/output/kuavo_head_test" \
        "head_camera_color_optical_frame" "camera_to_base"
      plot_test_nominal_only "right_wrist" "${out_right}" "${CC_DIR}/output/kuavo_right_wrist_test" \
        "right_wrist_camera_color_optical_frame" "right_wrist_camera_to_base"
      plot_test_nominal_only "left_wrist" "${out_left}" "${CC_DIR}/output/kuavo_left_wrist_test" \
        "left_wrist_camera_color_optical_frame" "left_wrist_camera_to_base"
      ;;
    optimize)
      out_head="$(out_dir_for "head" "${CC_DIR}/output_csv/kuavo_head")"
      out_right="$(out_dir_for "right_wrist" "${CC_DIR}/output_csv/kuavo_right_wrist")"
      out_left="$(out_dir_for "left_wrist" "${CC_DIR}/output_csv/kuavo_left_wrist")"

      csv_count="$(count_csv_files "${out_head}")"
      [[ "${csv_count}" -gt 0 ]] || die "(head) optimize 找不到可用 CSV：${out_head}"
      csv_count="$(count_csv_files "${out_right}")"
      [[ "${csv_count}" -gt 0 ]] || die "(right_wrist) optimize 找不到可用 CSV：${out_right}"
      csv_count="$(count_csv_files "${out_left}")"
      [[ "${csv_count}" -gt 0 ]] || die "(left_wrist) optimize 找不到可用 CSV：${out_left}"

      # 顺序执行：不落盘日志，避免生成 output/_parallel_optimize_logs
      run_sequential_optimize() {
        local tag="$1"
        local launch_f="$2"
        local csv_d="$3"
        local rsp="$4"
        local optn="$5"
        echo ""
        echo "[INFO] >>> ${tag} optimize 启动（csv: ${csv_d}）"
        set +e
        roslaunch "${launch_f}" \
          "csv_dir:=${csv_d}" \
          "do_capture_to_csv:=false" \
          "do_optimize_from_csv:=true" \
          "do_calibrate_manual:=false" \
          "robot_layout:=${ROBOT_LAYOUT}" \
          "rsp_name:=${rsp}" \
          "optimize_name:=${optn}"
        local rc=$?
        set -e
        if [[ "${rc}" -ne 0 ]]; then
          echo "[ERROR] ${tag} optimize 失败 (exit=${rc})" >&2
          exit "${rc}"
        fi
        echo "[INFO] <<< ${tag} optimize 完成"
      }

      # 必须在「该路 optimize 刚写完共享 URDF」后立即画图：下一路会覆盖 ${CALIBRATED_URDF}。
      # 行为与单路 MODE=optimize 中 plot_board_error_from_csv 一致；指标追加写入各 demo 的 optimization_metrics.md。
      plot_after_sequential_optimize() {
        local tag="$1"
        local launch_f="$2"
        local csv_d="$3"
        local plot_dir="$4"
        local cam_tip="$5"
        local sensor="$6"
        local pos_m rot_deg metrics prc
        pos_m="$(read_launch_param_default "${launch_f}" "outlier_reject_pos_m" "0.1")"
        rot_deg="$(read_launch_param_default "${launch_f}" "outlier_reject_rot_deg" "10")"
        metrics="${plot_dir}/optimization_metrics.md"
        mkdir -p "${plot_dir}"
        # 兼容旧版本输出：避免后续又生成/追加同名 txt
        rm -f "${plot_dir}/optimization_metrics.txt" 2>/dev/null || true
        {
          echo ""
          echo "============================================================"
          echo "相机标定优化指标报告"
          echo "  demo: ${tag}"
          echo "  时间: $(date -Iseconds)"
          echo "  CSV 目录: ${csv_d}"
          echo "  结果目录: ${plot_dir}"
          echo "============================================================"
          echo ""
          echo "---------- optimize_from_csv ----------"
          echo "（本脚本不再落盘 roslaunch 完整日志；请直接看终端输出或 /root/.ros/log）"
          echo ""
        } > "${metrics}"

        echo ""
        echo "[INFO] ${tag} plot_board_error_from_csv（与单路 optimize 相同）→ PNG: ${plot_dir}/board_pose_*_vs_urdf.png"
        set +e
        {
          echo ""
          echo "---------- plot_board_error_from_csv (${tag}, $(date -Iseconds)) ----------"
          echo "离群点阈值（与 launch 一致）: pos=${pos_m} m, rot=${rot_deg} deg"
          python3 "${CC_DIR}/plot_board_error_from_csv.py" \
            --csv_dir "${csv_d}" \
            --nominal_urdf "${NOMINAL_URDF}" \
            --output_dir "${plot_dir}" \
            --calibrated_urdf "${CALIBRATED_URDF}" \
            --calibration_yaml "${plot_dir}/calibration.yaml" \
            --camera_tip_link "${cam_tip}" \
            --sensor_name "${sensor}" \
            --fk_root "zarm_l1_ref_link" \
            --points_x 11 --points_y 8 --square_size 0.03 \
            --reject_outliers_pos_m "${pos_m}" \
            --reject_outliers_rot_deg "${rot_deg}" \
            --remap_to_center \
            --out_err_png "${plot_dir}/board_pose_error_pre_post_vs_urdf.png" \
            --out_abs_png "${plot_dir}/board_pose_bars_vs_urdf.png" \
            2>&1
        } 2>&1 | tee -a "${metrics}"
        prc=${PIPESTATUS[0]}
        set -e
        if [[ "${prc}" -ne 0 ]]; then
          echo "[WARN] ${tag} plot_board_error_from_csv 失败 (exit=${prc})，标定 YAML 仍可能有效" >&2
        fi
      }

      run_sequential_optimize "head" "${HEAD_LAUNCH}" "${out_head}" "rsp_head" "cap_optimize_head"
      plot_after_sequential_optimize "head" "${HEAD_LAUNCH}" "${out_head}" "${CC_DIR}/output/kuavo_head" \
        "head_camera_color_optical_frame" "camera_to_base"

      run_sequential_optimize "right_wrist" "${RIGHT_LAUNCH}" "${out_right}" "rsp_right" "cap_optimize_right"
      plot_after_sequential_optimize "right_wrist" "${RIGHT_LAUNCH}" "${out_right}" "${CC_DIR}/output/kuavo_right_wrist" \
        "right_wrist_camera_color_optical_frame" "right_wrist_camera_to_base"

      run_sequential_optimize "left_wrist" "${LEFT_LAUNCH}" "${out_left}" "rsp_left" "cap_optimize_left"
      plot_after_sequential_optimize "left_wrist" "${LEFT_LAUNCH}" "${out_left}" "${CC_DIR}/output/kuavo_left_wrist" \
        "left_wrist_camera_color_optical_frame" "left_wrist_camera_to_base"

      echo ""
      echo "[INFO] 三路顺序 optimize 已全部完成。"
      echo "[INFO] 共享 URDF 最后一次写入来自: left_wrist（顺序: head → right_wrist → left_wrist）。"
      echo "[INFO] 各 demo 标定 YAML: ${CC_DIR}/output/kuavo_head|kuavo_right_wrist|kuavo_left_wrist/calibration.yaml"
      echo "[INFO] 各 demo 误差图: 上述目录下 board_pose_error_pre_post_vs_urdf.png / board_pose_bars_vs_urdf.png"
      ;;
    *)
      die "未知 MODE: ${MODE}"
      ;;
  esac

  echo "[INFO] 全部并行结束"
  exit 0
fi

if [[ "${MODE}" == "move" ]]; then
  banner "阶段：仅运动（按 teach JSON 下发多姿态轨迹，不采数）"
  precheck_ros_and_topics "${CHOICE}" "${MODE}" || exit 1
  if [[ "${CHOICE}" == "1" ]]; then
    select_cam_and_wait "head_camera"
    echo "[INFO] 头部：启动 head_table_publisher.py，循环次数: ${LOOPS}"
    python3 "${CC_DIR}/demos/kuavo_head_demo/head_table_publisher.py" \
      _play_loop_count:="${LOOPS}" \
      _robot_layout:="${ROBOT_LAYOUT}" \
      _teach_json_path:="${TEACH_DIR}/teach_head_joint.json"
    motion_done_cam "head_camera"
  elif [[ "${CHOICE}" == "2" ]]; then
    select_cam_and_wait "right_wrist_camera"
    echo "[INFO] 右手：启动 right_wrist_table_publisher.py（按 teach_right_joint.json 下发）"
    python3 "${CC_DIR}/demos/kuavo_right_wrist/right_wrist_table_publisher.py" \
      _play_loop_count:="${LOOPS}" \
      _set_external_control_mode:=true \
      _enable_arm_quick_mode:=true \
      _robot_layout:="${ROBOT_LAYOUT}" \
      _teach_json_path:="${TEACH_DIR}/teach_right_joint.json"
    motion_done_cam "right_wrist_camera"
  elif [[ "${CHOICE}" == "3" ]]; then
    select_cam_and_wait "left_wrist_camera"
    echo "[INFO] 左手：启动 left_wrist_table_publisher.py（按 teach_left_joint.json 下发）"
    python3 "${CC_DIR}/demos/kuavo_left_wrist/left_wrist_table_publisher.py" \
      _play_loop_count:="${LOOPS}" \
      _set_external_control_mode:=true \
      _enable_arm_quick_mode:=true \
      _robot_layout:="${ROBOT_LAYOUT}" \
      _teach_json_path:="${TEACH_DIR}/teach_left_joint.json"
    motion_done_cam "left_wrist_camera"
  else
    echo "[INFO] 全部同时：头部 + 左右手（同时执行；左右手使用「合并下发」避免在 /kuavo_arm_traj 上打架；全部结束后再发 SESSION_DONE）"

    # 先一次性声明三路相机，再执行运动；结束后统一 SESSION_DONE
    select_cams_and_wait "head_camera" "right_wrist_camera" "left_wrist_camera"
    echo "[INFO] 全部同时：顺序执行，先头部..."
    python3 "${CC_DIR}/demos/kuavo_head_demo/head_table_publisher.py" \
      _play_loop_count:="${LOOPS}" \
      _robot_layout:="${ROBOT_LAYOUT}" \
      _teach_json_path:="${TEACH_DIR}/teach_head_joint.json"
    echo "[INFO] 全部同时：再左右手（合并下发）..."
    python3 "${CC_DIR}/demos/kuavo_both_arms/both_arms_table_publisher.py" \
      _play_loop_count:="${LOOPS}" \
      _set_external_control_mode:=true \
      _enable_arm_quick_mode:=true \
      _robot_layout:="${ROBOT_LAYOUT}" \
      _hold_sec:=5.0 \
      _teach_left_json:="${TEACH_DIR}/teach_left_joint.json" \
      _teach_right_json:="${TEACH_DIR}/teach_right_joint.json"
    motion_done_cam "head_camera"
    motion_done_cam "right_wrist_camera"
    motion_done_cam "left_wrist_camera"

    session_done
  fi
  echo "[INFO] move 阶段结束"
  exit 0
fi

if [[ "${MODE}" == "capture" ]]; then
  banner "阶段 1/2：采数（capture_to_csv -> 写入 CSV）"
  precheck_ros_and_topics "${CHOICE}" "${MODE}" || exit 1
  roslaunch "${DEMO_LAUNCH}" "csv_dir:=${OUT_DIR}" "do_capture_to_csv:=true" "do_optimize_from_csv:=false" "do_calibrate_manual:=false" \
    "robot_layout:=${ROBOT_LAYOUT}" &
  LAUNCH_PID=$!
  echo "[INFO] roslaunch PID = ${LAUNCH_PID}"
  sleep 5

  # 如果 roslaunch 很快退出，说明 launch 内部节点启动失败（比如缺少 features/chains 参数）
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    die "采数阶段 roslaunch 已提前退出（PID=${LAUNCH_PID}）。请查看 ~/.ros/log/*/roslaunch-*.log，常见原因：capture_to_csv 报 features/chains 参数缺失。"
  fi

  if [[ "${CHOICE}" == "1" ]]; then
    echo "[INFO] 头部：启动 head_table_publisher.py，循环次数: ${LOOPS}"
    python3 "${CC_DIR}/demos/kuavo_head_demo/head_table_publisher.py" \
      _play_loop_count:="${LOOPS}" \
      _robot_layout:="${ROBOT_LAYOUT}" \
      _teach_json_path:="${TEACH_DIR}/teach_head_joint.json"
    sleep 2
  elif [[ "${CHOICE}" == "2" ]]; then
    echo "[INFO] 右手：启动 right_wrist_table_publisher.py（按 teach_right_joint.json 下发并触发采样）"
    python3 "${CC_DIR}/demos/kuavo_right_wrist/right_wrist_table_publisher.py" \
      _play_loop_count:="${LOOPS}" \
      _set_external_control_mode:=true \
      _enable_arm_quick_mode:=true \
      _robot_layout:="${ROBOT_LAYOUT}" \
      _teach_json_path:="${TEACH_DIR}/teach_right_joint.json"
  else
    echo "[INFO] 左手：启动 left_wrist_table_publisher.py（按 teach_left_joint.json 下发并触发采样）"
    python3 "${CC_DIR}/demos/kuavo_left_wrist/left_wrist_table_publisher.py" \
      _play_loop_count:="${LOOPS}" \
      _set_external_control_mode:=true \
      _enable_arm_quick_mode:=true \
      _robot_layout:="${ROBOT_LAYOUT}" \
      _teach_json_path:="${TEACH_DIR}/teach_left_joint.json"
  fi

  # 优先等待 capture_to_csv 自然退出并完成 CSV 落盘；超时后再强制停 roslaunch。
  WAIT_LAUNCH_EXIT_SEC=0
  WAIT_LAUNCH_EXIT_MAX=30
  while kill -0 "${LAUNCH_PID}" 2>/dev/null && [[ "${WAIT_LAUNCH_EXIT_SEC}" -lt "${WAIT_LAUNCH_EXIT_MAX}" ]]; do
    if [[ "${WAIT_LAUNCH_EXIT_SEC}" -eq 0 ]]; then
      echo "[INFO] 等待 capture_to_csv 自然退出并写盘（最多 ${WAIT_LAUNCH_EXIT_MAX}s）..."
    fi
    sleep 1
    WAIT_LAUNCH_EXIT_SEC=$((WAIT_LAUNCH_EXIT_SEC + 1))
  done

  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[WARN] 等待超时，停止采数阶段 roslaunch (pid=${LAUNCH_PID})"
    kill "${LAUNCH_PID}" 2>/dev/null || true
  else
    echo "[INFO] 采数 roslaunch 已自然退出（CSV 写盘应已完成）"
  fi

  # 采数完检查 CSV 是否真的生成
  CSV_COUNT="$(count_csv_files "${OUT_DIR}")"
  if [[ "${CSV_COUNT}" -le 0 ]]; then
    die "采数阶段未生成任何 CSV：${OUT_DIR}。上面如果看到 capture_to_csv 的 FATAL（features/chains 未设置），需要先修 launch/yaml 参数后再采数。"
  fi
  echo "[INFO] 采数完成：发现 ${CSV_COUNT} 个 CSV 文件：${OUT_DIR}"
fi

if [[ "${MODE}" == "test" ]]; then
  banner "阶段：测试（按 teach_*_joint_test.json 下发 -> 写入带 _test 后缀 CSV -> 自动画图）"
  precheck_ros_and_topics "${CHOICE}" "${MODE}" || exit 1

  if [[ ! -f "${TEST_TEACH_JSON}" ]]; then
    die "找不到测试关节角 JSON：${TEST_TEACH_JSON}。请先运行 teach_joint_capture.py 选择“用于测试”生成 *_test.json"
  fi

  roslaunch "${DEMO_LAUNCH}" "csv_dir:=${OUT_DIR}" "do_capture_to_csv:=true" "do_optimize_from_csv:=false" "do_calibrate_manual:=false" \
    "robot_layout:=${ROBOT_LAYOUT}" &
  LAUNCH_PID=$!
  echo "[INFO] roslaunch PID = ${LAUNCH_PID}"
  sleep 5

  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    die "测试阶段 roslaunch 已提前退出（PID=${LAUNCH_PID}）。请查看 ~/.ros/log/*/roslaunch-*.log。"
  fi

  if [[ "${CHOICE}" == "1" ]]; then
    echo "[INFO] 头部测试：启动 head_table_publisher.py（读取 teach_head_joint_test.json），循环次数: ${LOOPS}"
    python3 "${CC_DIR}/demos/kuavo_head_demo/head_table_publisher.py" \
      _play_loop_count:="${LOOPS}" \
      _robot_layout:="${ROBOT_LAYOUT}" \
      _teach_json_path:="${TEACH_DIR}/teach_head_joint_test.json"
    sleep 2
  elif [[ "${CHOICE}" == "2" ]]; then
    echo "[INFO] 右手测试：启动 right_wrist_table_publisher.py（读取 teach_right_joint_test.json 下发并触发采样）"
    python3 "${CC_DIR}/demos/kuavo_right_wrist/right_wrist_table_publisher.py" \
      _play_loop_count:="${LOOPS}" \
      _set_external_control_mode:=true \
      _enable_arm_quick_mode:=true \
      _robot_layout:="${ROBOT_LAYOUT}" \
      _teach_json_path:="${TEACH_DIR}/teach_right_joint_test.json"
  else
    echo "[INFO] 左手测试：启动 left_wrist_table_publisher.py（读取 teach_left_joint_test.json 下发并触发采样）"
    python3 "${CC_DIR}/demos/kuavo_left_wrist/left_wrist_table_publisher.py" \
      _play_loop_count:="${LOOPS}" \
      _set_external_control_mode:=true \
      _enable_arm_quick_mode:=true \
      _robot_layout:="${ROBOT_LAYOUT}" \
      _teach_json_path:="${TEACH_DIR}/teach_left_joint_test.json"
  fi

  WAIT_LAUNCH_EXIT_SEC=0
  WAIT_LAUNCH_EXIT_MAX=30
  while kill -0 "${LAUNCH_PID}" 2>/dev/null && [[ "${WAIT_LAUNCH_EXIT_SEC}" -lt "${WAIT_LAUNCH_EXIT_MAX}" ]]; do
    if [[ "${WAIT_LAUNCH_EXIT_SEC}" -eq 0 ]]; then
      echo "[INFO] 等待 capture_to_csv 自然退出并写盘（最多 ${WAIT_LAUNCH_EXIT_MAX}s）..."
    fi
    sleep 1
    WAIT_LAUNCH_EXIT_SEC=$((WAIT_LAUNCH_EXIT_SEC + 1))
  done

  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[WARN] 等待超时，停止测试阶段 roslaunch (pid=${LAUNCH_PID})"
    kill "${LAUNCH_PID}" 2>/dev/null || true
  else
    echo "[INFO] 测试 roslaunch 已自然退出（CSV 写盘应已完成）"
  fi

  CSV_COUNT="$(count_csv_files "${OUT_DIR}")"
  if [[ "${CSV_COUNT}" -le 0 ]]; then
    die "测试阶段未生成任何 CSV：${OUT_DIR}"
  fi
  echo "[INFO] 测试采数完成：发现 ${CSV_COUNT} 个 CSV 文件：${OUT_DIR}"

  # 自动画图：仅使用原始 URDF 做 FK（不依赖 optimize 产物）
  TEST_PLOT_OUT_DIR="${PLOT_OUT_DIR}_test"
  mkdir -p "${TEST_PLOT_OUT_DIR}"
  TEST_METRICS_FILE="${TEST_PLOT_OUT_DIR}/test_metrics.txt"
  echo "[INFO] 自动画图与指标输出到: ${TEST_METRICS_FILE}"

  {
    echo "============================================================"
    echo "相机标定测试指标报告"
    echo "  demo: ${DEMO_NAME}"
    echo "  时间: $(date -Iseconds)"
    echo "  测试 CSV 目录: ${OUT_DIR}"
    echo "  测试结果目录: ${TEST_PLOT_OUT_DIR}"
    echo "  FK 使用 URDF: ${NOMINAL_URDF}（nominal only）"
    echo "============================================================"
    echo ""
    python3 "${CC_DIR}/plot_board_error_from_csv.py" \
      --csv_dir "${OUT_DIR}" \
      --nominal_urdf "${NOMINAL_URDF}" \
      --output_dir "${TEST_PLOT_OUT_DIR}" \
      --use_nominal_only \
      --camera_tip_link "${CAMERA_TIP_LINK}" \
      --sensor_name "${SENSOR_NAME}" \
      --fk_root "${FK_ROOT}" \
      --points_x 11 --points_y 8 --square_size 0.03 \
      --ignore_optimization_used_sample_ids \
      --out_err_png "${TEST_PLOT_OUT_DIR}/board_pose_error_pre_post_vs_urdf_test.png" \
      --out_abs_png "${TEST_PLOT_OUT_DIR}/board_pose_bars_vs_urdf_test.png" \
      ${REMAP_TO_CENTER}
    echo ""
    echo "---------- 结束 ----------"
  } > "${TEST_METRICS_FILE}" 2>&1 || echo "[WARN] 测试自动画图失败（不影响测试 CSV 输出），请查看 ${TEST_METRICS_FILE}"

  echo "[INFO] 测试阶段结束；测试指标与图已输出到: ${TEST_PLOT_OUT_DIR}"
fi

if [[ "${MODE}" == "optimize" ]]; then
  banner "阶段：优化（optimize_from_csv -> 读取 CSV 并输出 URDF/YAML）"
  CSV_COUNT="$(count_csv_files "${OUT_DIR}")"
  if [[ "${CSV_COUNT}" -le 0 ]]; then
    die "优化阶段找不到可用 CSV（目录为空或不存在）：${OUT_DIR}。请先运行 capture 阶段生成 CSV。"
  fi
  PLOT_REJECT_POS_M="$(read_launch_param_default "${DEMO_LAUNCH}" "outlier_reject_pos_m" "0.1")"
  PLOT_REJECT_ROT_DEG="$(read_launch_param_default "${DEMO_LAUNCH}" "outlier_reject_rot_deg" "10")"
  echo "[INFO] 离群点阈值（来自 launch）: pos=${PLOT_REJECT_POS_M} m, rot=${PLOT_REJECT_ROT_DEG} deg"

  mkdir -p "${PLOT_OUT_DIR}"
  OPTIMIZE_METRICS_FILE="${PLOT_OUT_DIR}/optimization_metrics.md"
  # 兼容旧版本输出：避免后续又生成同名 txt
  rm -f "${PLOT_OUT_DIR}/optimization_metrics.txt" 2>/dev/null || true
  echo "[INFO] 优化过程与指标性评价（终端少刷屏）写入: ${OPTIMIZE_METRICS_FILE}"

  {
    echo "============================================================"
    echo "相机标定优化指标报告"
    echo "  demo: ${DEMO_NAME}"
    echo "  时间: $(date -Iseconds)"
    echo "  CSV 目录: ${OUT_DIR}"
    echo "  结果目录: ${PLOT_OUT_DIR}"
    echo "============================================================"
    echo ""
    echo "---------- optimize_from_csv / roslaunch ----------"
    roslaunch "${DEMO_LAUNCH}" "csv_dir:=${OUT_DIR}" "do_capture_to_csv:=false" "do_optimize_from_csv:=true" "do_calibrate_manual:=false" \
      "robot_layout:=${ROBOT_LAYOUT}"
    echo ""
    echo "---------- plot_board_error_from_csv（标定前后 FK 误差表与 summary）----------"
    python3 "${CC_DIR}/plot_board_error_from_csv.py" \
      --csv_dir "${OUT_DIR}" \
      --nominal_urdf "${NOMINAL_URDF}" \
      --output_dir "${PLOT_OUT_DIR}" \
      --calibrated_urdf "${CALIBRATED_URDF}" \
      --calibration_yaml "${PLOT_OUT_DIR}/calibration.yaml" \
      --camera_tip_link "${CAMERA_TIP_LINK}" \
      --sensor_name "${SENSOR_NAME}" \
      --fk_root "${FK_ROOT}" \
      --points_x 11 --points_y 8 --square_size 0.03 \
      --reject_outliers_pos_m "${PLOT_REJECT_POS_M}" \
      --reject_outliers_rot_deg "${PLOT_REJECT_ROT_DEG}" \
      ${REMAP_TO_CENTER} || echo "[WARN] 画图失败（不影响优化结果）"
    echo ""
    echo "---------- 结束 ----------"
  } > "${OPTIMIZE_METRICS_FILE}" 2>&1

  echo "[INFO] 优化阶段结束；完整指标已保存: ${OPTIMIZE_METRICS_FILE}"
fi
