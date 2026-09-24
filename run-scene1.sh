#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_ID="1"
if [[ "${1:-}" == "--internal-task-id" ]]; then
  TASK_ID="${2:-}"
  shift 2
fi
[[ "${TASK_ID}" =~ ^[123]$ ]] || { echo "[ERROR] task id must be 1, 2 or 3" >&2; exit 1; }

CONTAINER_NAME="${SCENE1_CONTAINER:-kuavo-mpc-wbc}"
CONTAINER_WORKSPACE="/root/kuavo_ws"
EXAMPLES_DIR="${CONTAINER_WORKSPACE}/src/data_challenge_simulator/examples"
PACKAGE_DIR="${CONTAINER_WORKSPACE}/src/data_challenge_simulator"
COLLECT_LAYOUT_HOST_FILE=""
COLLECT_LAYOUT_CONTAINER_FILE=""
COLLECT_LAYOUT_ENV_NAME=""
ENTRY_LOCK_FD=""
ENTRY_PREPARED="0"

die() { echo "[ERROR] $*" >&2; exit 1; }

usage() {
  cat <<EOF
Usage:
  ./run-scene${TASK_ID}.sh setup                    Create or validate the delivery container
  ./run-scene${TASK_ID}.sh build                    Build the shared ROS workspace
  ./run-scene${TASK_ID}.sh task [SEED]              Run one task episode without rosbag
  ./run-scene${TASK_ID}.sh collect ROUNDS           Generate fresh layouts, then run and record episodes
  ./run-scene${TASK_ID}.sh model [MODEL_NAME]       Start simulator for external model inference
  ./run-scene${TASK_ID}.sh stop                     Stop this task's simulator processes
  ./run-scene${TASK_ID}.sh status                   Show this task's process status

Task 1 also supports waypoint, verify, replay and replay-headless through scene1.sh.
EOF
}

container_exists() { docker inspect "${CONTAINER_NAME}" >/dev/null 2>&1; }

ensure_container_running() {
  command -v docker >/dev/null 2>&1 || die "docker is not installed"
  container_exists || die "container ${CONTAINER_NAME} does not exist; run ./run-scene${TASK_ID}.sh setup first"
  local mounted_repo
  mounted_repo="$(docker inspect -f '{{range .Mounts}}{{if eq .Destination "/root/kuavo_ws"}}{{.Source}}{{end}}{{end}}' "${CONTAINER_NAME}")"
  [[ -n "${mounted_repo}" ]] || die "container ${CONTAINER_NAME} does not mount ${CONTAINER_WORKSPACE}"
  [[ "$(realpath "${mounted_repo}")" == "$(realpath "${SCRIPT_DIR}")" ]] || die \
    "container ${CONTAINER_NAME} mounts ${mounted_repo}, not this checkout ${SCRIPT_DIR}"
  if [[ "$(docker inspect -f '{{.State.Status}}' "${CONTAINER_NAME}")" != "running" ]]; then
    docker start "${CONTAINER_NAME}" >/dev/null
  fi
  docker exec "${CONTAINER_NAME}" test -f "${CONTAINER_WORKSPACE}/devel/setup.bash" || die \
    "workspace is not built; run ./run-scene${TASK_ID}.sh build first"
}

allow_gui() {
  export DISPLAY="${DISPLAY:-:0}"
  command -v xhost >/dev/null 2>&1 || die "xhost is required for the visual simulator"
  xhost +SI:localuser:root >/dev/null
}

generate_collect_layouts() {
  local rounds="$1"
  [[ "${rounds}" =~ ^[1-9][0-9]*$ ]] || die "ROUNDS must be a positive integer"
  ensure_container_running

  local generator_name config_name layout_env_name
  case "${TASK_ID}" in
    1)
      generator_name="generate_task1_v2_layout_seeds.py"
      config_name="task1_v2_layout_seeds.json"
      layout_env_name="TASK1_V2_LAYOUT_FILE"
      ;;
    2)
      generator_name="generate_task2_layout_seeds.py"
      config_name="task2_layout_seeds.json"
      layout_env_name="TASK2_LAYOUT_FILE"
      ;;
    3)
      generator_name="generate_task3_layout_seeds.py"
      config_name="task3_layout_seeds.json"
      layout_env_name="TASK3_LAYOUT_FILE"
      ;;
  esac

  local host_config host_generated host_install
  local container_generated container_install
  host_config="${SCRIPT_DIR}/src/data_challenge_simulator/config/${config_name}"
  host_generated="${host_config}.collect.$$.tmp"
  container_generated="${PACKAGE_DIR}/config/${config_name}.collect.$$.tmp"
  host_install="${host_config}.collect.$$.json"
  container_install="${PACKAGE_DIR}/config/${config_name}.collect.$$.json"

  echo "[INFO] Generating ${rounds} fresh independently seeded Task ${TASK_ID} layouts"
  local generator_status container_state
  if docker exec -i -e ROBOT_VERSION=400062 "${CONTAINER_NAME}" \
      bash -lc '
        set -e
        source /opt/ros/noetic/setup.bash
        source /root/kuavo_ws/devel/setup.bash
        cd /root/kuavo_ws/src/data_challenge_simulator
        exec python3 "tools/$1" \
          --output "$2" --count "$3" --random-seeds
      ' bash "${generator_name}" "${container_generated}" "${rounds}"; then
    generator_status=0
  else
    generator_status=$?
    rm -f -- "${host_generated}"
    container_state="$(docker inspect -f \
      'status={{.State.Status}}, exit={{.State.ExitCode}}, oom={{.State.OOMKilled}}, error={{json .State.Error}}' \
      "${CONTAINER_NAME}" 2>/dev/null || echo 'unavailable')"
    die "Task ${TASK_ID} layout generator command exited with status ${generator_status}; container ${CONTAINER_NAME}: ${container_state}"
  fi

  if ! mv -- "${host_generated}" "${host_install}"; then
    rm -f -- "${host_generated}" "${host_install}"
    die "failed to install generated Task ${TASK_ID} collect layouts"
  fi
  COLLECT_LAYOUT_HOST_FILE="${host_install}"
  COLLECT_LAYOUT_CONTAINER_FILE="${container_install}"
  COLLECT_LAYOUT_ENV_NAME="${layout_env_name}"
  echo "[INFO] Installed ${rounds} fresh collect-only layouts in ${host_install}"
}

cleanup_collect_layout() {
  if [[ -n "${COLLECT_LAYOUT_HOST_FILE}" ]]; then
    rm -f -- "${COLLECT_LAYOUT_HOST_FILE}"
    COLLECT_LAYOUT_HOST_FILE=""
    COLLECT_LAYOUT_CONTAINER_FILE=""
    COLLECT_LAYOUT_ENV_NAME=""
  fi
}

run_task() {
  local record="$1" rounds="$2" seed="$3" headless="$4"
  [[ "${rounds}" =~ ^[1-9][0-9]*$ ]] || die "ROUNDS must be a positive integer"
  [[ "${seed}" =~ ^[0-9]+$ ]] || die "SEED must be a non-negative integer"
  prepare_entry_start
  local -a display_args=()
  local -a exec_args=(-i)
  local -a layout_args=()
  local -a helper_args=(--task-id "${TASK_ID}" --record "${record}" --repeat "${rounds}" --start-seed "${seed}")
  if [[ -n "${COLLECT_LAYOUT_CONTAINER_FILE}" ]]; then
    layout_args=(-e "${COLLECT_LAYOUT_ENV_NAME}=${COLLECT_LAYOUT_CONTAINER_FILE}")
  fi
  # With a terminal, allocate a container PTY so Ctrl+C reaches helperfunc.py
  # and its ShutdownGuard.  Without a terminal (CI/redirected execution), the
  # host-side traps below run the same stop sweep if docker exec is signalled.
  [[ -t 0 && -t 1 ]] && exec_args=(-it)
  if [[ "${headless}" == "1" ]]; then
    helper_args+=(--headless)
  else
    allow_gui
    display_args=(-e "DISPLAY=${DISPLAY}")
    [[ -t 0 && -t 1 ]] && helper_args+=(--keep-sim-on-failure)
  fi

  trap 'trap "" INT TERM; stop_task; exit 130' INT
  trap 'trap "" INT TERM; stop_task; exit 143' TERM
  local docker_status
  if docker exec "${exec_args[@]}" "${display_args[@]}" "${layout_args[@]}" \
      -e ROBOT_VERSION=400062 "${CONTAINER_NAME}" \
      bash -lc '
        set -e
        source /opt/ros/noetic/setup.bash
        source /root/kuavo_ws/devel/setup.bash
        cd /root/kuavo_ws/src/data_challenge_simulator/examples
        exec python3 helperfunc.py "$@"
      ' bash "${helper_args[@]}"; then
    docker_status=0
  else
    docker_status=$?
  fi
  trap - INT TERM
  return "${docker_status}"
}

run_model() {
  local model_name="${1:-anonymous}"
  prepare_entry_start
  allow_gui
  docker exec -it -e "DISPLAY=${DISPLAY}" -e ROBOT_VERSION=400062 "${CONTAINER_NAME}" \
    bash -lc '
      set -e
      source /opt/ros/noetic/setup.bash
      source /root/kuavo_ws/devel/setup.bash
      cd /root/kuavo_ws/src/data_challenge_simulator/examples
      exec rosrun data_challenge_simulator model_entry.py --task-id "$1" --model-name "$2"
    ' bash "${TASK_ID}" "${model_name}"
}

# Patterns for every process one task round can leave behind.  Kept in one
# place so `stop` and `status` cannot drift apart again -- they used to
# disagree, and `status` showed processes `stop` could not kill.
# The leading character is bracketed so a pattern cannot match the pgrep/pkill
# command line that is looking for it.
task_process_patterns() {
  local patterns=(
    "[t]ask${TASK_ID}[a-z0-9_]*\.py"
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
  command -v docker >/dev/null 2>&1 || die "docker is not installed"
  container_exists || return 0
  [[ "$(docker inspect -f '{{.State.Status}}' "${CONTAINER_NAME}")" == "running" ]] || return 0
  local pattern
  pattern="$(task_process_patterns)"
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

prepare_entry_start() {
  [[ "${ENTRY_PREPARED}" == "0" ]] || return 0
  ensure_container_running

  # Stop the old supervisor before the new one exists.  The previous order
  # let a retiring model's cleanup discover and kill the newly started helper.
  stop_task

  command -v flock >/dev/null 2>&1 || die "flock is required to serialize simulator entry points"
  exec {ENTRY_LOCK_FD}>"/tmp/${CONTAINER_NAME}.simulator-entry.lock"
  flock -w 15 "${ENTRY_LOCK_FD}" || die \
    "another simulator entry is still active for container ${CONTAINER_NAME}"

  # Two launchers can both reach the first stop before either acquires the
  # lock.  Sweep once more after winning the lock, then launch exactly one.
  stop_task
  ENTRY_PREPARED="1"
}

show_status() {
  command -v docker >/dev/null 2>&1 || die "docker is not installed"
  if ! container_exists; then echo "[INFO] container ${CONTAINER_NAME}: absent"; return; fi
  local state
  state="$(docker inspect -f '{{.State.Status}}' "${CONTAINER_NAME}")"
  echo "[INFO] container ${CONTAINER_NAME}: ${state}"
  [[ "${state}" == "running" ]] || return
  # Same pattern list as stop_task, so status cannot show something stop
  # is unable to kill.
  docker exec "${CONTAINER_NAME}" bash -lc \
    "pgrep -af '$(task_process_patterns)' || true"
}

mode="${1:-}"
case "${mode}" in
  setup|build)
    exec "${SCRIPT_DIR}/scene1.sh" "${mode}"
    ;;
  task)
    run_task 0 1 "${2:-1}" 0
    ;;
  collect)
    [[ -n "${2:-}" ]] || die "collect requires ROUNDS"
    [[ "$#" == "2" ]] || die "collect accepts only ROUNDS; layout seeds are generated automatically"
    prepare_entry_start
    generate_collect_layouts "$2"
    trap cleanup_collect_layout EXIT
    run_task 1 "$2" 1 1
    ;;
  model)
    [[ "$#" -le 2 ]] || die "model accepts at most one MODEL_NAME"
    run_model "${2:-anonymous}"
    ;;
  waypoint|verify|replay|replay-headless)
    [[ "${TASK_ID}" == "1" ]] || die "${mode} is only available for Task 1"
    exec "${SCRIPT_DIR}/scene1.sh" "$@"
    ;;
  stop)
    stop_task
    ;;
  status)
    show_status
    ;;
  -h|--help|help|"")
    usage
    ;;
  *)
    usage
    die "unknown command: ${mode}"
    ;;
esac
