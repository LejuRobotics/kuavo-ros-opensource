#!/bin/bash
# Prepare ROS 1 calibration dependencies and build robot_calibration.
set -Eeuo pipefail
trap 'echo "失败：第 ${LINENO} 行，命令: ${BASH_COMMAND}；修复后可重新运行本脚本。" >&2' ERR

find_workspace() {
  local dir
  dir="$(cd "${1:-$PWD}" && pwd)"
  while [ "$dir" != / ]; do
    if [ -f "$dir/src/Camera_Calibration/robot_calibration-ros1/robot_calibration/package.xml" ]; then
      echo "$dir"; return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

WS="$(find_workspace "${1:-$PWD}")" || { echo "找不到 kuavo 工作区；请传入工作区路径。" >&2; exit 1; }
NOETIC_SETUP="${CALIBRATION_NOETIC_SETUP:-/opt/ros/noetic/setup.bash}"
EXTRA_WS="${CALIBRATION_EXTRA_WS:-$HOME/ros-noetic-extra}"
REPO_BASE="${CALIBRATION_REPO_BASE:-}"
JOBS="${CALIBRATION_JOBS:-2}"
[ -f "$NOETIC_SETUP" ] || { echo "未找到 ROS Noetic: $NOETIC_SETUP；请先配置 Noetic。" >&2; exit 1; }
if [ -n "${ROS_DISTRO:-}" ] && [ "$ROS_DISTRO" != noetic ]; then
  echo "当前 ROS_DISTRO=$ROS_DISTRO；请在未加载其他 ROS 版本的新终端中运行。" >&2; exit 1
fi
# ROS setup scripts may refer to unset variables.
set +u
source "$NOETIC_SETUP"
set -u
[ "${ROS_DISTRO:-}" = noetic ] || { echo "Noetic 环境加载失败：$NOETIC_SETUP" >&2; exit 1; }
for cmd in rospack catkin cmake pkg-config git; do
  command -v "$cmd" >/dev/null || { echo "缺少命令 $cmd，请先安装或配置 Noetic 构建环境。" >&2; exit 1; }
done
# Reuse an existing, usable overlay if present.
if [ -f "$EXTRA_WS/devel/setup.bash" ]; then
  set +u; source "$EXTRA_WS/devel/setup.bash"; set -u
fi

ros_packages=(geometric_shapes moveit_msgs shape_msgs object_recognition_msgs octomap_msgs eigen_stl_containers random_numbers)
missing_ros=()
for pkg in "${ros_packages[@]}"; do
  rospack find "$pkg" >/dev/null 2>&1 || missing_ros+=("$pkg")
done

# Ceres is not guaranteed to provide pkg-config metadata; check its CMake package.
probe_dir="$(mktemp -d)"
trap 'rm -rf "$probe_dir"' EXIT
printf '%s\n' 'cmake_minimum_required(VERSION 3.16)' 'project(ros_probe CXX)' \
  'find_package(catkin REQUIRED COMPONENTS geometric_shapes moveit_msgs shape_msgs object_recognition_msgs octomap_msgs eigen_stl_containers random_numbers)' > "$probe_dir/CMakeLists.txt"
# rospack may see source packages from an interrupted build without usable CMake exports.
if ! cmake -S "$probe_dir" -B "$probe_dir/ros-build" >/dev/null 2>&1 && [ "${#missing_ros[@]}" -eq 0 ]; then
  missing_ros=("${ros_packages[@]}")
fi
printf '%s\n' 'cmake_minimum_required(VERSION 3.16)' 'project(calibration_probe CXX)' 'find_package(Ceres REQUIRED)' > "$probe_dir/CMakeLists.txt"
missing_system=()
if ! cmake -S "$probe_dir" -B "$probe_dir/build" >/dev/null 2>&1; then
  missing_system+=(libceres-dev)
fi
if ! pkg-config --exists fcl; then missing_system+=(libfcl-dev); fi
# These are direct robot_calibration dependencies, even if every ROS package exists.
for pkg in liborocos-kdl-dev libtinyxml-dev; do
  dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -qx 'install ok installed' || missing_system+=("$pkg")
done
# The source fallback needs the development packages of geometric_shapes.
if [ "${#missing_ros[@]}" -gt 0 ]; then
  for pkg in libassimp-dev liboctomap-dev libqhull-dev libeigen3-dev libconsole-bridge-dev; do
    dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -qx 'install ok installed' || missing_system+=("$pkg")
  done
fi
if [ "${#missing_system[@]}" -gt 0 ]; then
  command -v apt-get >/dev/null || { echo "缺少系统依赖：${missing_system[*]}；需要 apt-get。" >&2; exit 1; }
  sudo_cmd=()
  if [ "$(id -u)" -ne 0 ]; then
    command -v sudo >/dev/null || { echo "安装依赖需要 sudo：${missing_system[*]}" >&2; exit 1; }
    sudo_cmd=(sudo)
  fi
  echo "安装缺少的系统依赖：${missing_system[*]}"
  "${sudo_cmd[@]}" apt-get update
  "${sudo_cmd[@]}" apt-get install -y "${missing_system[@]}"
fi

if [ "${#missing_ros[@]}" -gt 0 ]; then
  echo "从源码补齐 ROS 依赖：${missing_ros[*]}"
  mkdir -p "$EXTRA_WS/src"
  if [ -z "$REPO_BASE" ]; then
    bundle="$WS/tools/robot_calibration_ros1_sources.tar.gz"
    [ -f "$bundle" ] || { echo "缺少离线源码包：$bundle" >&2; exit 1; }
    echo '2190afd33e8cfaa407b43aa3afc25d41820b5926e3b7bf058928608c3535d343  '"$bundle" | sha256sum -c --status || {
      echo "离线源码包校验失败：$bundle" >&2; exit 1; }
    mkdir -p "$probe_dir/sources"
    tar -xzf "$bundle" -C "$probe_dir/sources"
  fi
  # Pinned ROS 1 revisions verified on Ubuntu 24.04; existing checkouts are never reset.
  while read -r name repo revision; do
    dest="$EXTRA_WS/src/$name"
    if [ -e "$dest" ]; then
      [ -d "$dest/.git" ] && [ "$(git -C "$dest" rev-parse HEAD)" = "$revision" ] || {
        echo "已有源码 $dest 不是预期版本 $revision；请检查或使用 CALIBRATION_EXTRA_WS 指定新目录。" >&2; exit 1; }
      continue
    fi
    tmp="$(mktemp -d "$EXTRA_WS/src/.${name}.XXXXXX")"
    if [ -z "$REPO_BASE" ]; then
      if ! cp -a "$probe_dir/sources/$name/." "$tmp/" || [ "$(git -C "$tmp" rev-parse HEAD)" != "$revision" ]; then
        rm -rf "$tmp"
        echo "离线源码 $name 无效或版本不匹配。" >&2; exit 1
      fi
    elif ! git -C "$tmp" init -q || ! git -C "$tmp" fetch -q --depth 1 "${REPO_BASE%/}/$repo.git" "$revision" || ! git -C "$tmp" checkout -q --detach FETCH_HEAD; then
      rm -rf "$tmp"
      echo "获取 $repo 失败；可设置 CALIBRATION_REPO_BASE 为镜像或本地 Git 仓库根目录。" >&2; exit 1
    fi
    mv "$tmp" "$dest"
  done <<'REPOS'
geometric_shapes ros-planning/geometric_shapes f3ad2f31a4fbace9cd96b5013c80c78aebd314fe
moveit_msgs ros-planning/moveit_msgs 851fe62676ce61a0172450a5fc859536112bc55e
common_msgs ros/common_msgs 092bc230c8153dd4cfee0c84c9796cc7e6b09fdb
object_recognition_msgs wg-perception/object_recognition_msgs c6f22411f9929c37ec96d9e2fee308a8cdbbe90b
octomap_msgs OctoMap/octomap_msgs dcaaf62bd071db0fcd806a98a9101a2e470f7f6d
eigen_stl_containers ros/eigen_stl_containers d2a41f0b73fb6ede3d9d2f23dd9a3e79ffd9a89e
random_numbers ros-planning/random_numbers b36ab4a204f20db2096a30fec17422327fe0506a
REPOS
  cd "$EXTRA_WS"
  # Only configure a new workspace; never rewrite an existing catkin profile.
  if [ ! -d .catkin_tools ]; then
    catkin config --extend "$(dirname "$NOETIC_SETUP")" --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
  fi
  catkin build "${ros_packages[@]}" -j"$JOBS"
  set +u; source "$EXTRA_WS/devel/setup.bash"; set -u
  rospack profile >/dev/null
  for pkg in "${ros_packages[@]}"; do
    rospack find "$pkg" >/dev/null || { echo "构建后仍找不到 ROS 包：$pkg" >&2; exit 1; }
  done
fi

echo "编译 robot_calibration：$WS"
cd "$WS"
# Keep calibration in the normal workspace so its standard setup is sufficient.
# Preserve all other profile options; only record the actual dependency underlay.
underlay="$(dirname "$NOETIC_SETUP")"
if [ -f "$EXTRA_WS/devel/setup.bash" ]; then underlay="$EXTRA_WS/devel"; fi
catkin config --extend "$underlay"
devel_space="$(catkin locate --devel)"
# --force-cmake alone does not regenerate an existing prebuild setup. Removing
# this generated entry makes catkin rebuild it with the current underlay, while
# retaining every package's build products and the rest of the devel space.
rm -f "$devel_space/_setup_util.py"
catkin build robot_calibration -j"$JOBS" --force-cmake --no-env-cache \
  --cmake-args "-DCALIBRATION_EXTRA_PREFIX=$EXTRA_WS/devel"
echo "编译成功。使用前执行：source $devel_space/setup.bash"
