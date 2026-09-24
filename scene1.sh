#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(realpath "${SCRIPT_DIR}")"
IMAGE_NAME="${SCENE1_IMAGE:-kuavo_opensource_mpc_wbc_img:0.6.1-gpu}"
CONTAINER_NAME="${SCENE1_CONTAINER:-kuavo-mpc-wbc}"
CONTAINER_WORKSPACE="/root/kuavo_ws"

usage() {
  cat <<'EOF'
Scene 1 delivery entry point (run every command on the Docker host)

One-time setup:
  ./scene1.sh setup
  ./scene1.sh build

Run and collect:
  ./scene1.sh task [SEED]                  Run once with a MuJoCo window, no bag
  ./scene1.sh task-headless [SEED]         Run once without a window, no bag
  ./scene1.sh collect ROUNDS [START_SEED]  Record successful rounds headlessly

Bag handling:
  ./scene1.sh verify BAG
  ./scene1.sh replay BAG [SEED]
  ./scene1.sh replay-headless BAG [SEED]

Operations:
  ./scene1.sh status
  ./scene1.sh stop                          Stop Scene 1 processes only
  ./scene1.sh container-stop                Stop the delivery container
  ./scene1.sh shell

Environment overrides used by setup:
  SCENE1_IMAGE=registry.example.com/team/scene1-runtime:1.0
  SCENE1_CONTAINER=kuavo-mpc-wbc
  SCENE1_GPU=1                              1 = request all NVIDIA GPUs (default)
EOF
}

die() {
  echo "[ERROR] $*" >&2
  exit 1
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"
}

require_uint() {
  [[ "$1" =~ ^[0-9]+$ ]] || die "$2 must be a non-negative integer"
}

require_positive_int() {
  [[ "$1" =~ ^[1-9][0-9]*$ ]] || die "$2 must be a positive integer"
}

container_exists() {
  docker inspect "${CONTAINER_NAME}" >/dev/null 2>&1
}

validate_container() {
  local configured_image mounted_repo network_mode
  configured_image="$(docker inspect -f '{{.Config.Image}}' "${CONTAINER_NAME}")"
  mounted_repo="$(docker inspect -f '{{range .Mounts}}{{if eq .Destination "/root/kuavo_ws"}}{{.Source}}{{end}}{{end}}' "${CONTAINER_NAME}")"
  network_mode="$(docker inspect -f '{{.HostConfig.NetworkMode}}' "${CONTAINER_NAME}")"

  [[ "${configured_image}" == "${IMAGE_NAME}" ]] || die \
    "Container ${CONTAINER_NAME} uses ${configured_image}, expected ${IMAGE_NAME}. Use another SCENE1_CONTAINER name or explicitly recreate it."
  [[ -n "${mounted_repo}" ]] || die \
    "Container ${CONTAINER_NAME} does not mount ${CONTAINER_WORKSPACE}."
  [[ "$(realpath "${mounted_repo}")" == "${REPO_DIR}" ]] || die \
    "Container ${CONTAINER_NAME} mounts ${mounted_repo}, not this checkout ${REPO_DIR}. Use another SCENE1_CONTAINER name or explicitly recreate it."
  [[ "${network_mode}" == "host" ]] || die \
    "Container ${CONTAINER_NAME} uses network mode ${network_mode}, expected host. Use another SCENE1_CONTAINER name or explicitly recreate it."
}

setup_container() {
  require_command docker
  docker info >/dev/null 2>&1 || die \
    "Docker daemon is unavailable or the current user lacks Docker permission."
  docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1 || die \
    "Image ${IMAGE_NAME} is not installed. Pull it or load the delivered image archive first."

  if container_exists; then
    validate_container
    echo "[INFO] Reusing validated container ${CONTAINER_NAME}."
  else
    local -a gpu_args=()
    case "${SCENE1_GPU:-1}" in
      1) gpu_args=(--gpus all) ;;
      0) ;;
      *) die "SCENE1_GPU must be 0 or 1" ;;
    esac
    docker run -d \
      --name "${CONTAINER_NAME}" \
      --network host \
      --workdir "${CONTAINER_WORKSPACE}" \
      "${gpu_args[@]}" \
      -e ROBOT_VERSION=400062 \
      -e QT_X11_NO_MITSHM=1 \
      -v "${REPO_DIR}:${CONTAINER_WORKSPACE}" \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      "${IMAGE_NAME}" \
      bash -lc 'trap : TERM INT; sleep infinity & wait'
    echo "[INFO] Created container ${CONTAINER_NAME}."
  fi
  ensure_container_running
}

ensure_container_running() {
  require_command docker
  container_exists || die "Container ${CONTAINER_NAME} does not exist. Run ./scene1.sh setup first."
  validate_container
  if [[ "$(docker inspect -f '{{.State.Status}}' "${CONTAINER_NAME}")" != "running" ]]; then
    docker start "${CONTAINER_NAME}" >/dev/null
  fi
}

allow_container_gui() {
  export DISPLAY="${DISPLAY:-:0}"
  command -v xhost >/dev/null 2>&1 || die "xhost is required for visual replay. Install x11-xserver-utils or use a headless command."
  xhost +SI:localuser:root >/dev/null
}

build_workspace() {
  ensure_container_running
  stop_task
  docker exec -i \
    -e ROBOT_VERSION=400062 \
    "${CONTAINER_NAME}" \
    bash -lc '
      set -e
      source /opt/ros/noetic/setup.bash
      cd /root/kuavo_ws
      catkin build kuavo_msgs mujoco_cpp humanoid_controllers \
        data_challenge_simulator --no-status -j2 \
        --cmake-args -Dpybind11_DIR=/opt/drake/lib/cmake/pybind11
    '
}

require_built_workspace() {
  ensure_container_running
  docker exec "${CONTAINER_NAME}" test -f "${CONTAINER_WORKSPACE}/devel/setup.bash" || die \
    "Workspace is not built. Run ./scene1.sh build first."
}

run_task() {
  local headless="$1" seed="$2"
  require_uint "${seed}" "SEED"
  require_built_workspace
  local -a headless_arg=()
  local -a tty_arg=(-i)
  local keep_arg=""
  if [[ "${headless}" == "1" ]]; then
    headless_arg=(--headless)
  else
    allow_container_gui
    [[ -t 0 && -t 1 ]] && tty_arg=(-it) && keep_arg="--keep-sim-on-failure"
  fi

  docker exec "${tty_arg[@]}" \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e ROBOT_VERSION=400062 \
    "${CONTAINER_NAME}" \
    bash -lc '
      set -e
      source /opt/ros/noetic/setup.bash
      source /root/kuavo_ws/devel/setup.bash
      cd /root/kuavo_ws/src/data_challenge_simulator/examples
      exec python3 helperfunc.py "$@"
    ' bash --task-id 1 --record 0 --repeat 1 --start-seed "${seed}" "${headless_arg[@]}" ${keep_arg:+"${keep_arg}"}
}

collect_bags() {
  local rounds="$1" start_seed="$2"
  require_positive_int "${rounds}" "ROUNDS"
  require_uint "${start_seed}" "START_SEED"
  require_built_workspace
  docker exec -i \
    -e ROBOT_VERSION=400062 \
    "${CONTAINER_NAME}" \
    bash -lc '
      set -e
      source /opt/ros/noetic/setup.bash
      source /root/kuavo_ws/devel/setup.bash
      cd /root/kuavo_ws/src/data_challenge_simulator/examples
      exec python3 helperfunc.py "$@"
    ' bash --task-id 1 --record 1 --repeat "${rounds}" --start-seed "${start_seed}" --headless
}

container_path_for_repo_file() {
  local input_path="$1" absolute_path
  absolute_path="$(realpath "${input_path}")"
  [[ -f "${absolute_path}" ]] || die "File does not exist: ${input_path}"
  case "${absolute_path}" in
    "${REPO_DIR}"/*)
      printf '%s/%s\n' "${CONTAINER_WORKSPACE}" "${absolute_path#"${REPO_DIR}/"}"
      ;;
    *)
      die "BAG must be inside this checkout (${REPO_DIR}) so the container can read it."
      ;;
  esac
}

run_bag_tool() {
  local command="$1" bag_path="$2" seed="$3" headless="$4" container_bag
  local -a extra_args=()
  container_bag="$(container_path_for_repo_file "${bag_path}")"
  require_built_workspace
  if [[ -n "${seed}" ]]; then
    require_uint "${seed}" "SEED"
    extra_args+=(--seed "${seed}")
  fi
  if [[ "${headless}" == "1" ]]; then
    extra_args+=(--headless)
  elif [[ "${command}" == "replay" ]]; then
    allow_container_gui
  fi

  docker exec -i \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e ROBOT_VERSION=400062 \
    "${CONTAINER_NAME}" \
    bash -lc '
      set -e
      source /opt/ros/noetic/setup.bash
      source /root/kuavo_ws/devel/setup.bash
      cd /root/kuavo_ws/src/data_challenge_simulator/examples
      exec python3 scene1_bag.py "$@"
    ' bash "${command}" "${container_bag}" "${extra_args[@]}"
}

# Patterns for every process a Scene 1 round can leave behind.  Kept in one
# place so `stop` and `status` cannot drift apart again -- they used to
# disagree, and `status` showed processes `stop` could not kill.  Scene 1 is
# the only task this entry point runs, so the task id is fixed at 1.
# The leading character is bracketed so a pattern cannot match the pgrep/pkill
# command line that is looking for it.
scene1_process_patterns() {
  local patterns=(
    "[t]ask1[a-z0-9_]*\.py"
    "[h]elperfunc\.py"
    "[m]odel_entry\.py"
    "[t]ask_scorer\.py"
    "[s]cene1_bag\.py"
    "[r]osbag (record|play)"
    "[r]oslaunch data_challenge_simulator"
    "[s]ource_bin_latch_v2\.py"
    "[r]osbag_compat_publisher\.py"
    "[t]opic_repeater\.py"
    "[n]odelet_manager"
    "[n]odelet_mujoco"
    "[n]odelet_controller"
  )
  local joined
  joined="$(IFS='|'; echo "${patterns[*]}")"
  printf '%s' "${joined}"
}

stop_task() {
  require_command docker
  container_exists || return 0
  [[ "$(docker inspect -f '{{.State.Status}}' "${CONTAINER_NAME}")" == "running" ]] || return 0
  local pattern
  pattern="$(scene1_process_patterns)"
  # rosbag takes SIGINT first: SIGTERM does not give it time to finish writing
  # the bag.  A signalled process can take a few seconds to actually exit, so
  # sweep twice with a pause, then SIGKILL whatever is still holding on.
  docker exec "${CONTAINER_NAME}" bash -lc "
    pkill -INT -f '[r]osbag (record|play)' || true
    pkill -TERM -f '${pattern}' || true
    sleep 2
    pkill -TERM -f '${pattern}' || true
    sleep 2
    pkill -KILL -f '${pattern}' || true
    sleep 1
    pgrep -af '${pattern}' || true
  "
}

show_status() {
  require_command docker
  if ! container_exists; then
    echo "[INFO] Container ${CONTAINER_NAME}: absent"
    return
  fi
  local state
  state="$(docker inspect -f '{{.State.Status}}' "${CONTAINER_NAME}")"
  echo "[INFO] Container ${CONTAINER_NAME}: ${state}"
  echo "[INFO] Image: $(docker inspect -f '{{.Config.Image}}' "${CONTAINER_NAME}")"
  if [[ "${state}" == "running" ]]; then
    # Same pattern list as stop_task, so status cannot show something stop
    # is unable to kill.
    docker exec "${CONTAINER_NAME}" bash -lc \
      "pgrep -af '$(scene1_process_patterns)' || true"
  fi
}

mode="${1:-}"
case "${mode}" in
  setup)
    setup_container
    ;;
  build)
    build_workspace
    ;;
  task)
    run_task 0 "${2:-1}"
    ;;
  task-headless)
    run_task 1 "${2:-1}"
    ;;
  collect)
    [[ -n "${2:-}" ]] || die "Usage: ./scene1.sh collect ROUNDS [START_SEED]"
    collect_bags "$2" "${3:-1}"
    ;;
  verify)
    [[ -n "${2:-}" ]] || die "Usage: ./scene1.sh verify BAG"
    run_bag_tool verify "$2" "" 0
    ;;
  replay)
    [[ -n "${2:-}" ]] || die "Usage: ./scene1.sh replay BAG [SEED]"
    run_bag_tool replay "$2" "${3:-}" 0
    ;;
  replay-headless)
    [[ -n "${2:-}" ]] || die "Usage: ./scene1.sh replay-headless BAG [SEED]"
    run_bag_tool replay "$2" "${3:-}" 1
    ;;
  stop)
    stop_task
    ;;
  container-stop)
    require_command docker
    container_exists && docker stop "${CONTAINER_NAME}" >/dev/null || true
    ;;
  status)
    show_status
    ;;
  shell)
    require_built_workspace
    exec docker exec -it \
      -e DISPLAY="${DISPLAY:-:0}" \
      -e ROBOT_VERSION=400062 \
      "${CONTAINER_NAME}" bash
    ;;
  -h|--help|help|"")
    usage
    ;;
  *)
    usage
    die "Unknown command: ${mode}"
    ;;
esac
