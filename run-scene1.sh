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
ENTRY_LOCK_FD=""
ENTRY_PREPARED="0"

die() { echo "[ERROR] $*" >&2; exit 1; }

usage() {
  cat <<EOF
Usage:
  ./run-scene${TASK_ID}.sh setup                    Create or validate the delivery container
  ./run-scene${TASK_ID}.sh build [-j N]             Build the shared ROS workspace (default: 8 jobs)
  ./run-scene${TASK_ID}.sh task [ROUND] [--keep-sim-on-failure]  Run one task episode without rosbag
  ./run-scene${TASK_ID}.sh collect ROUNDS           Generate seeded scenes, then run and record episodes
  ./run-scene${TASK_ID}.sh model [MODEL_NAME]       Start simulator for external model inference
  ./run-scene${TASK_ID}.sh stop                     Stop this task's simulator processes
  ./run-scene${TASK_ID}.sh status                   Show this task's process status

All tasks: task [ROUND] defaults to 1; seed = config/task2_randomization.json seed_base + ROUND.
All tasks generate the same per-round scenes in task, collect and model.
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

run_task() {
  local record="$1" rounds="$2" seed="$3" headless="$4" keep="${5:-0}"
  [[ "${rounds}" =~ ^[1-9][0-9]*$ ]] || die "ROUNDS must be a positive integer"
  [[ "${seed}" =~ ^[0-9]+$ ]] || die "SEED must be a non-negative integer"
  prepare_entry_start
  local -a display_args=()
  local -a exec_args=(-i)
  local -a helper_args=(--task-id "${TASK_ID}" --record "${record}" --repeat "${rounds}" --start-seed "${seed}")
  # With a terminal, allocate a container PTY so Ctrl+C reaches helperfunc.py
  # and its ShutdownGuard.  Without a terminal (CI/redirected execution), the
  # host-side traps below run the same stop sweep if docker exec is signalled.
  [[ -t 0 && -t 1 ]] && exec_args=(-it)
  if [[ "${headless}" == "1" ]]; then
    helper_args+=(--headless)
  else
    allow_gui
    display_args=(-e "DISPLAY=${DISPLAY}")
    if [[ "$keep" == "1" ]]; then
      [[ -t 0 && -t 1 ]] || die "--keep-sim-on-failure requires an interactive terminal"
      helper_args+=(--keep-sim-on-failure)
    fi
  fi

  run_container_entry "${exec_args[@]}" "${display_args[@]}" \
      -e ROBOT_VERSION=400062 "$CONTAINER_NAME" \
      bash -lc '
        set -e
        source /opt/ros/noetic/setup.bash
        source /root/kuavo_ws/devel/setup.bash
        cd /root/kuavo_ws/src/data_challenge_simulator/examples
        exec python3 helperfunc.py "$@"
      ' bash "${helper_args[@]}"
}

run_model() {
  local model_name="${1:-anonymous}"
  prepare_entry_start
  allow_gui
  local -a exec_args=(-i)
  [[ -t 0 && -t 1 ]] && exec_args=(-it)
  run_container_entry "${exec_args[@]}" -e "DISPLAY=$DISPLAY" -e ROBOT_VERSION=400062 "$CONTAINER_NAME" \
    bash -lc '
      set -e
      source /opt/ros/noetic/setup.bash
      source /root/kuavo_ws/devel/setup.bash
      cd /root/kuavo_ws/src/data_challenge_simulator/examples
      exec rosrun data_challenge_simulator model_entry.py --task-id "$1" --model-name "$2"
    ' bash "${TASK_ID}" "${model_name}"
}

# Waiting on an asynchronous client lets bash handle disconnect/stop signals
# immediately. Keep stdin connected for explicit interactive failure inspection.
run_container_entry() {
  local client status
  trap 'trap "" INT TERM HUP; stop_task; exit 130' INT
  trap 'trap "" INT TERM HUP; stop_task; exit 143' TERM
  trap 'trap "" INT TERM HUP; stop_task; exit 129' HUP
  docker exec "$@" <&0 &
  client=$!
  if wait "$client"; then status=0; else status=$?; fi
  if (( status != 0 )); then
    trap "" INT TERM HUP
    stop_task || status=1
  fi
  trap - INT TERM HUP
  return "$status"
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
  # Let the supervisor finalize scores, flush rosbag and reap its owned
  # tree. Name-based sweeping is only a fallback for abandoned children.
  docker exec -i "$CONTAINER_NAME" bash -s -- "$pattern" <<'CLEANUP'
set -eu
pattern="$1"
supervisors='[h]elperfunc\.py|[m]odel_entry\.py'
pkill -TERM -f "$supervisors" || true
for ((i=0; i<30; i++)); do
  pgrep -f "$pattern" >/dev/null || exit 0
  pgrep -f "$supervisors" >/dev/null || break
  sleep 1
done
pkill -INT -f '[r]osbag (record|play)' || true
for ((i=0; i<10; i++)); do
  pgrep -f '[r]osbag (record|play)' >/dev/null || break
  sleep 1
done
pkill -TERM -f "$pattern" || true
for ((i=0; i<5; i++)); do
  pgrep -f "$pattern" >/dev/null || exit 0
  sleep 1
done
pkill -KILL -f "$pattern" || true
sleep 1
if pgrep -af "$pattern"; then
  echo "[ERROR] Simulator cleanup left running processes" >&2
  exit 1
fi
CLEANUP
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
    exec "${SCRIPT_DIR}/scene1.sh" "$@"
    ;;
  task)
    shift
    round=1
    if [[ "${1:-}" =~ ^[0-9]+$ ]]; then round="$1"; shift; fi
    keep=0
    if [[ "${1:-}" == "--keep-sim-on-failure" ]]; then keep=1; shift; fi
    [[ "$#" == "0" ]] || die "task accepts ROUND and --keep-sim-on-failure"
    run_task 0 1 "$round" 0 "$keep"
    ;;
  collect)
    [[ -n "${2:-}" ]] || die "collect requires ROUNDS"
    [[ "$#" == "2" ]] || die "collect accepts only ROUNDS; layout seeds are generated automatically"
    prepare_entry_start
    # Collection uses the same GLFW/GLX display setup as task mode.
    run_task 1 "$2" 1 0
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
