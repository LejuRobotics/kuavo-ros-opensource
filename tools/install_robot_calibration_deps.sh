#!/bin/bash
# Install robot_calibration dependencies from configured apt repositories.
set -euo pipefail

find_workspace() {
  local dir
  dir="$(cd "${1:-$PWD}" && pwd)"
  while [ "$dir" != "/" ]; do
    if [ -f "$dir/src/Camera_Calibration/robot_calibration-ros1/robot_calibration/package.xml" ]; then
      echo "$dir"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

if [ -n "${1:-}" ]; then
  WS="$(cd "$1" && pwd)"
else
  WS="$(find_workspace "$PWD")" || {
    echo "在 kuavo 工作区里运行，或传入路径：$0 /path/to/kuavo-ros-control" >&2
    exit 1
  }
fi

if [ ! -f "$WS/src/Camera_Calibration/robot_calibration-ros1/robot_calibration/package.xml" ]; then
  echo "这里没有 robot_calibration：$WS" >&2
  exit 1
fi
if [ ! -f /opt/ros/noetic/setup.bash ]; then
  echo "未找到 ROS Noetic：/opt/ros/noetic/setup.bash" >&2
  exit 1
fi

missing=()
for pkg in libassimp-dev libfcl-dev liboctomap-dev libqhull-dev libeigen3-dev \
           libconsole-bridge-dev libceres-dev liborocos-kdl-dev libtinyxml-dev \
           ros-noetic-geometric-shapes ros-noetic-moveit-msgs ros-noetic-shape-msgs \
           ros-noetic-object-recognition-msgs ros-noetic-octomap-msgs \
           ros-noetic-eigen-stl-containers ros-noetic-random-numbers; do
  if [ "$(dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null || true)" != 'install ok installed' ]; then
    missing+=("$pkg")
  fi
done

if [ "${#missing[@]}" -gt 0 ]; then
  echo "安装系统包：${missing[*]}"
  if [ "$(id -u)" -eq 0 ]; then
    SUDO=()
  else
    SUDO=(sudo)
  fi
  "${SUDO[@]}" apt-get update
  "${SUDO[@]}" apt-get install -y "${missing[@]}" || {
    echo "安装失败：请检查 apt 软件源和网络连接。" >&2
    exit 1
  }
fi

# ROS setup scripts may refer to unset variables.
set +u
source /opt/ros/noetic/setup.bash
set -u

echo "编译 robot_calibration：$WS"
cd "$WS"
catkin build robot_calibration

echo "完成。使用前执行：source ${WS}/devel/setup.bash"
