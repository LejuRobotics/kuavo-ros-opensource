#!/bin/bash
# Build robot_calibration against the shared Noetic deps in
# /home/lab/ros-noetic-extra. Does not modify the kuavo git repo.
set -euo pipefail

EXTRA=/home/lab/ros-noetic-extra
EXTRA_DEVEL="${EXTRA}/devel"

if [ "$(id -u)" -eq 0 ]; then
  SUDO=()
else
  SUDO=(sudo)
fi

export PATH="/opt/lejurobot-venv/bin:${PATH}"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash

find_workspace() {
  local start="${1:-$PWD}"
  local dir
  dir="$(cd "$start" && pwd)"
  while [ "$dir" != "/" ]; do
    if [ -d "$dir/src" ] && [ -f "$dir/src/Camera_Calibration/robot_calibration-ros1/robot_calibration/package.xml" ]; then
      echo "$dir"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

if [ "${1:-}" != "" ]; then
  WS="$(cd "$1" && pwd)"
else
  WS="$(find_workspace "$PWD")" || {
    echo "在 kuavo 工作区里运行，或传入路径： $0 /path/to/kuavo-ros-control" >&2
    exit 1
  }
fi

if [ ! -f "$WS/src/Camera_Calibration/robot_calibration-ros1/robot_calibration/package.xml" ]; then
  echo "这里没有 robot_calibration：$WS" >&2
  exit 1
fi

echo "工作区：$WS"

missing=()
for pkg in libassimp-dev libfcl-dev liboctomap-dev libqhull-dev libeigen3-dev \
           libconsole-bridge-dev libceres-dev liborocos-kdl-dev libtinyxml-dev; do
  if ! dpkg -s "$pkg" >/dev/null 2>&1; then
    missing+=("$pkg")
  fi
done
if [ "${#missing[@]}" -gt 0 ]; then
  echo "安装系统包：${missing[*]}"
  "${SUDO[@]}" apt-get update
  "${SUDO[@]}" apt-get install -y "${missing[@]}"
fi

clone_pin() {
  local url="$1" dest="$2" commit="$3"
  if [ -f "$dest/package.xml" ]; then
    return 0
  fi
  rm -rf "$dest"
  git clone "$url" "$dest"
  git -C "$dest" checkout "$commit"
}

mkdir -p "$EXTRA/src"
clone_pin https://github.com/ros-planning/geometric_shapes.git "$EXTRA/src/geometric_shapes" f3ad2f3
clone_pin https://github.com/ros-planning/moveit_msgs.git "$EXTRA/src/moveit_msgs" 851fe62
clone_pin https://github.com/ros/eigen_stl_containers.git "$EXTRA/src/eigen_stl_containers" d2a41f0
clone_pin https://github.com/ros-planning/random_numbers.git "$EXTRA/src/random_numbers" b36ab4a
clone_pin https://github.com/wg-perception/object_recognition_msgs.git "$EXTRA/src/object_recognition_msgs" c6f2241
clone_pin https://github.com/OctoMap/octomap_msgs.git "$EXTRA/src/octomap_msgs" dcaaf62
if [ ! -f "$EXTRA/src/shape_msgs/package.xml" ]; then
  rm -rf /tmp/common_msgs "$EXTRA/src/shape_msgs"
  git clone --depth 1 -b noetic-devel https://github.com/ros/common_msgs.git /tmp/common_msgs
  cp -a /tmp/common_msgs/shape_msgs "$EXTRA/src/shape_msgs"
  rm -rf /tmp/common_msgs
fi

mkdir -p "$EXTRA/.catkin_tools/profiles"
if ! grep -q '^active:' "$EXTRA/.catkin_tools/profiles/profiles.yaml" 2>/dev/null; then
  printf 'active: default\n' > "$EXTRA/.catkin_tools/profiles/profiles.yaml"
fi

if [ ! -f "$EXTRA_DEVEL/share/geometric_shapes/cmake/geometric_shapesConfig.cmake" ]; then
  echo "编译独立依赖工作区"
  (
    cd "$EXTRA"
    catkin config --extend /opt/ros/noetic
    catkin build
  )
else
  echo "独立依赖工作区已编译，跳过"
fi

in_ws=0
for pkg in geometric_shapes moveit_msgs shape_msgs object_recognition_msgs octomap_msgs eigen_stl_containers random_numbers; do
  if [ -f "$WS/src/$pkg/package.xml" ]; then
    echo "工作区 src 里已有 $pkg。两处同名包会冲突，请先移出 $WS/src/$pkg" >&2
    in_ws=1
  fi
done
if [ "$in_ws" -ne 0 ]; then
  exit 1
fi

PROFILE="$(awk '/^active:/{print $2; exit}' "$WS/.catkin_tools/profiles/profiles.yaml" 2>/dev/null || true)"
if [ -z "$PROFILE" ]; then
  PROFILE=default
fi
BUILD_SPACE="$(awk '/^build_space:/{print $2; exit}' "$WS/.catkin_tools/profiles/$PROFILE/config.yaml" 2>/dev/null || true)"
if [ -z "$BUILD_SPACE" ]; then
  if [ "$PROFILE" = "default" ]; then
    BUILD_SPACE=build
  else
    BUILD_SPACE="build/$PROFILE"
  fi
fi

PREFIX="${EXTRA_DEVEL};${WS}/devel;/opt/ros/noetic"
if [ -d /opt/drake ]; then
  PREFIX="${PREFIX};/opt/drake"
fi

cache="$WS/$BUILD_SPACE/robot_calibration/CMakeCache.txt"
if [ -f "$cache" ] && ! grep -q '/home/lab/ros-noetic-extra/devel' "$cache"; then
  rm -rf "$WS/$BUILD_SPACE/robot_calibration"
fi

echo "profile=$PROFILE"
echo "编译 robot_calibration"
(
  cd "$WS"
  catkin config --profile "$PROFILE" --extend /opt/ros/noetic
  catkin build robot_calibration --profile "$PROFILE" --cmake-args "-DCMAKE_PREFIX_PATH=${PREFIX}"
)

echo "完成。使用前执行： source ${WS}/devel/setup.bash"
