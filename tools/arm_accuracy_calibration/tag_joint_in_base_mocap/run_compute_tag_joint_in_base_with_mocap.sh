#!/usr/bin/env bash
set -e

usage() {
  cat <<'EOF'
用法:
  run_compute_tag_joint_in_base_with_mocap.sh [--build] [--mocap-sdk=optitrack|nokov] [--server-ip=IP] [--] [compute_tag_joint_in_base.py args...]

功能:
  1) 可选编译动捕相关包
  2) source ROS 环境
  3) 启动动捕 SDK（返回 pid）
  4) 运行 compute_tag_joint_in_base.py（默认订阅 tag_pose / joint_1，外参读 optitrack_poses.yaml）
  5) 自动停止动捕进程

示例:
  bash run_compute_tag_joint_in_base_with_mocap.sh --build --mocap-sdk=optitrack
  bash run_compute_tag_joint_in_base_with_mocap.sh -- --offline-from-yaml --poses /path/to/optitrack_poses.yaml
EOF
}

start_mocap_sdk() {
  local mocap_sdk="$1"
  local server_ip="$2"

  echo "[mocap] starting mocap: ${mocap_sdk}, server: ${server_ip}" >&2

  local mocap_pid=""
  local log_dir="/tmp/kuavo_mocap_logs"
  mkdir -p "${log_dir}"
  local ts
  ts="$(date +%Y%m%d_%H%M%S)"
  local mocap_log="${log_dir}/mocap_${mocap_sdk}_${ts}.log"

  if [[ "$mocap_sdk" == "optitrack" ]]; then
    export MOCAP_SERVER_IP="${server_ip}"
    # 重要：在命令替换 $(...) 场景下，后台进程若继承 stdout，会导致外层一直等待管道关闭（表现为“卡住”）
    rosrun optitrack_data_receive OptiTrack_Data_Receive.py "${server_ip}" >"${mocap_log}" 2>&1 </dev/null &
  else
    rosrun motioncapture SampleClient "${server_ip}" >"${mocap_log}" 2>&1 </dev/null &
  fi

  mocap_pid=$!
  sleep 2

  if ! kill -0 "${mocap_pid}" 2>/dev/null; then
    echo "[mocap] Error: mocap process died immediately (pid=${mocap_pid})" >&2
    return 1
  fi

  echo "${mocap_pid}"
}

MOCAP_SERVER_IP="10.10.30.191"
MOCAP_SDK="optitrack"
BUILD_FIRST=0
PY_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build)
      BUILD_FIRST=1
      shift
      ;;
    --mocap-sdk=*)
      MOCAP_SDK="${1#*=}"
      shift
      ;;
    --server-ip=*)
      MOCAP_SERVER_IP="${1#*=}"
      shift
      ;;
    --)
      shift
      PY_ARGS+=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      PY_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ "$MOCAP_SDK" != "optitrack" && "$MOCAP_SDK" != "nokov" ]]; then
  echo "Error: MOCAP_SDK must be optitrack or nokov, got: $MOCAP_SDK" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARM_ACCURACY_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

COMPUTE_PY="${SCRIPT_DIR}/compute_tag_joint_in_base.py"
if [[ ! -f "${COMPUTE_PY}" ]]; then
  echo "Error: missing compute script: ${COMPUTE_PY}" >&2
  exit 1
fi

MOCAP_PID=""
cleanup() {
  echo ""
  echo "[cleanup] stopping mocap pid: ${MOCAP_PID}"
  if [[ -n "${MOCAP_PID}" ]]; then
    kill "${MOCAP_PID}" 2>/dev/null || true
  fi
}
trap cleanup INT TERM

echo "=========================================="
echo " compute_tag_joint_in_base (with mocap) "
echo "  mocap sdk: ${MOCAP_SDK}"
echo "  server ip: ${MOCAP_SERVER_IP}"
echo "=========================================="

if [[ "$BUILD_FIRST" -eq 1 ]]; then
  echo "[build] catkin build mocap deps..."
  cd "${WORKSPACE_ROOT}"
  if command -v catkin >/dev/null 2>&1; then
    catkin build optitrack_data_receive kuavo_msgs motioncapture -DCMAKE_BUILD_TYPE=Release || true
  else
    catkin_make || true
  fi
  echo "[build] done"
fi

echo "[source] loading ROS env..."
cd "${WORKSPACE_ROOT}"
if [[ -f "devel/setup.bash" ]]; then
  source devel/setup.bash
elif [[ -f "install/setup.bash" ]]; then
  source install/setup.bash
else
  echo "Error: cannot find devel/setup.bash or install/setup.bash" >&2
  exit 1
fi

# 如果传了 --offline-from-yaml，则不需要动捕；这里自动跳过启动 SDK
SKIP_MOCAP=0
for arg in "${PY_ARGS[@]}"; do
  if [[ "$arg" == "--offline-from-yaml" ]]; then
    SKIP_MOCAP=1
  fi
done

if [[ "$SKIP_MOCAP" -eq 0 ]]; then
  echo "[mocap] start mocap sdk..."
  MOCAP_PID="$(start_mocap_sdk "${MOCAP_SDK}" "${MOCAP_SERVER_IP}")"
  echo "[mocap] pid=${MOCAP_PID}"
else
  echo "[mocap] --offline-from-yaml detected, skip mocap sdk."
fi

echo "[run] python compute_tag_joint_in_base.py ..."
echo "       args: ${PY_ARGS[*]}"

python3 "${COMPUTE_PY}" "${PY_ARGS[@]}"

cleanup

