#!/usr/bin/env bash
# Ubuntu 22.04 aarch64：挂载代码仓库 + 宿主机 build/devel/.ros/lejuconfig；ROS/Drake 在镜像内。
# 用法: bash docker/run_jammy_aarch64.sh
# 重建: FORCE_RECREATE=1 bash docker/run_jammy_aarch64.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
IMAGE_TAG="${IMAGE_TAG:-kuavo-jammy-aarch64:wheel-mpc-run-real-0712-1244}"
# IMAGE_TAG="${IMAGE_TAG:-kuavo-jammy-aarch64:wheel-mpc-clean}"
USE_GPU="${USE_GPU:-1}"
FORCE_RECREATE="${FORCE_RECREATE:-0}"

export DISPLAY="${DISPLAY:-:1.0}"

KUAVO_WS_HOST="${KUAVO_WS_HOST:-$REPO_ROOT}"
DIR_HASH="$(echo "$REPO_ROOT" | md5sum | cut -c1-8)"
CONTAINER_NAME="${CONTAINER_NAME:-kuavo_jammy_${DIR_HASH}}"

# catkin build/devel 使用仓库目录，显式挂载避免 docker commit 时打进镜像
CATKIN_BUILD="${CATKIN_BUILD:-${KUAVO_WS_HOST}/build}"
CATKIN_DEVEL="${CATKIN_DEVEL:-${KUAVO_WS_HOST}/devel}"
CCACHE_DIR="${HOME}/.ccache"
mkdir -p "$CATKIN_BUILD" "$CATKIN_DEVEL" "$CCACHE_DIR" "${HOME}/.ros" "${HOME}/.config/lejuconfig"

_image_has_entrypoint() {
  docker run --rm --entrypoint test "$IMAGE_TAG" -f /usr/local/bin/jammy_entrypoint.sh 2>/dev/null
}

_exec_shell() {
  docker exec -it "$CONTAINER_NAME" bash -lc '
    source /root/kuavo_ws/docker/aarch64/setup_jammy_container_env.sh 2>/dev/null \
      || source /usr/local/bin/setup_jammy_container_env.sh 2>/dev/null || true
    exec bash -l
  '
}

_show_container_info() {
  local div="━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  echo -e "\n${div}"
  echo -e "📌 Container: ${CONTAINER_NAME}"
  echo -e "📂 Repo:      ${KUAVO_WS_HOST} -> /root/kuavo_ws"
  echo -e "🔨 Build:     ${CATKIN_BUILD} -> /root/kuavo_ws/build"
  echo -e "📦 Devel:     ${CATKIN_DEVEL} -> /root/kuavo_ws/devel"
  echo -e "🐳 Image:     ${IMAGE_TAG}"
  if docker inspect "$CONTAINER_NAME" &>/dev/null; then
    echo -e "🔗 Mounts:"
    docker inspect -f '{{range .Mounts}}   {{.Source}} → {{.Destination}}{{println}}{{end}}' "$CONTAINER_NAME"
  fi
  echo -e "${div}\n"
}

_build_docker_run_args() {
  DOCKER_RUN_ARGS=(
    --net=host
    --privileged
    --ulimit rtprio=99
    --cap-add=sys_nice
    --group-add=dialout
    --name "$CONTAINER_NAME"
    --add-host kuavo_master:192.168.26.1
    -e KUAVO_WS=/root/kuavo_ws
    -e ROS_INSTALL=/opt/ros/noetic
    -e DRAKE_PREFIX=/opt/drake
    -e TMPDIR=/tmp
    -e DISPLAY="${DISPLAY}"
    -e "ROBOT_VERSION=${ROBOT_VERSION:-62}"
    -e DISABLE_ROS1_EOL_WARNINGS=1
    -v /dev:/dev
    -v "${KUAVO_WS_HOST}:/root/kuavo_ws:rw"
    -v "${CATKIN_BUILD}:/root/kuavo_ws/build:rw"
    -v "${CATKIN_DEVEL}:/root/kuavo_ws/devel:rw"
    -v "${HOME}/.ros:/root/.ros:rw"
    -v "${CCACHE_DIR}:/root/.ccache:rw"
    -v "${HOME}/.config/lejuconfig:/root/.config/lejuconfig:rw"
  )

  if [[ "$USE_GPU" == "1" ]] && command -v nvidia-smi >/dev/null 2>&1; then
    xhost +local:root 2>/dev/null || true
    DOCKER_RUN_ARGS+=(--runtime nvidia --gpus all)
    DOCKER_RUN_ARGS+=(-e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all,display)
  fi

  if [[ -d /tmp/.X11-unix ]]; then
    DOCKER_RUN_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
  fi
}

_create_container() {
  echo "[run_jammy_aarch64] Creating container ${CONTAINER_NAME} ..."
  local inner='source /root/kuavo_ws/docker/aarch64/setup_jammy_container_env.sh 2>/dev/null || source /usr/local/bin/setup_jammy_container_env.sh 2>/dev/null || true; exec bash -l'
  if _image_has_entrypoint; then
    docker run -it "${DOCKER_RUN_ARGS[@]}" "$IMAGE_TAG" bash -lc "${inner}"
  else
    docker run -it "${DOCKER_RUN_ARGS[@]}" "$IMAGE_TAG" bash -lc "${inner}"
  fi
}

_attach_or_create() {
  if [[ "$FORCE_RECREATE" == "1" ]] && docker ps -aq -f "name=^${CONTAINER_NAME}$" | grep -q .; then
    echo "[run_jammy_aarch64] FORCE_RECREATE=1: removing ${CONTAINER_NAME} ..."
    docker rm -f "$CONTAINER_NAME" >/dev/null
  fi

  if docker ps -aq -f "name=^${CONTAINER_NAME}$" | grep -q .; then
    if docker ps -q -f "name=^${CONTAINER_NAME}$" | grep -q .; then
      echo "[run_jammy_aarch64] Container ${CONTAINER_NAME} is running — attaching ..."
    else
      echo "[run_jammy_aarch64] Starting stopped container ${CONTAINER_NAME} ..."
      docker start "$CONTAINER_NAME" >/dev/null
    fi
    _show_container_info
    _exec_shell
    return 0
  fi

  _show_container_info
  _create_container
}

echo "[run_jammy_aarch64] Image: ${IMAGE_TAG}"
echo "[run_jammy_aarch64] Container: ${CONTAINER_NAME}  (hash=${DIR_HASH})"
echo "[run_jammy_aarch64] 挂载: 仓库 + build/devel/.ros/lejuconfig；ROS/Drake: /opt/ros/noetic /opt/drake"
echo "[run_jammy_aarch64] DISPLAY=${DISPLAY}"

if ! docker image inspect "$IMAGE_TAG" &>/dev/null; then
  echo "[run_jammy_aarch64] ERROR: image ${IMAGE_TAG} not found." >&2
  exit 1
fi

if ! docker run --rm --entrypoint test "$IMAGE_TAG" -f /opt/ros/noetic/setup.bash 2>/dev/null; then
  echo "[run_jammy_aarch64] ERROR: 镜像内缺少 /opt/ros/noetic（需使用已 commit ROS+Drake 的镜像）" >&2
  exit 1
fi

if ! docker run --rm --entrypoint test "$IMAGE_TAG" -f /opt/drake/lib/libdrake.so 2>/dev/null; then
  echo "[run_jammy_aarch64] WARN: 镜像内无 /opt/drake，Drake 相关包可能无法编译"
fi

if ! _image_has_entrypoint; then
  echo "[run_jammy_aarch64] WARN: 镜像无 jammy_entrypoint，将使用仓库内 setup 脚本"
fi

_build_docker_run_args
_attach_or_create
