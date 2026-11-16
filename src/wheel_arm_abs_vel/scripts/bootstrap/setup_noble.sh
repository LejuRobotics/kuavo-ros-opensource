#!/usr/bin/env bash
# Ubuntu 24.04 aarch64 dependencies for the absolute-IK workspace.
set -e -o pipefail
KUAVO_ABS_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd -P)"
cd "$KUAVO_ABS_WS"
if (( EUID == 0 )); then echo 'Run as the ordinary workspace user, not root.' >&2; exit 1; fi
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 && "$(uname -m)" == aarch64 ]] || exit 1
export PATH="/usr/bin:/bin:$PATH"
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 PIP_DISABLE_PIP_VERSION_CHECK=1
unset PYTHONHOME PYTHONPATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROS_ROOT LD_LIBRARY_PATH PKG_CONFIG_PATH
mkdir -p .absolute-deps/downloads
printf '*\n' > .absolute-deps/.gitignore
exec > >(tee .absolute-deps/install.log) 2>&1
trap 'echo "Failed at line $LINENO; inspect .absolute-deps/install.log." >&2' ERR
if [[ -f .absolute-deps/platform && "$(cat .absolute-deps/platform)" != 24.04-aarch64 ]]; then
  echo 'Use a fresh workspace when changing Ubuntu or CPU architecture.' >&2; exit 1
fi
printf '24.04-aarch64\n' > .absolute-deps/platform
printf 'aarch64\n' > .absolute-deps/architecture
: > .absolute-deps/dependencies.sha256
sudo -n true || sudo -v
sudo apt-get update
sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ca-certificates curl git build-essential cmake pkg-config python3-dev python3-pip python3-venv
/usr/bin/python3 -c 'import sys; assert sys.version_info[:2] == (3, 12), sys.version'

KUAVO_DRAKE_DEB="$KUAVO_ABS_WS/.absolute-deps/downloads/drake-dev_1.55.0-1_arm64-noble.deb"
KUAVO_DRAKE_SHA=76bf666a3ccf7b439ffbf8ebc4c570c1273f5b75bacbece3468edc72c8778d40
if ! printf '%s  %s\n' "$KUAVO_DRAKE_SHA" "$KUAVO_DRAKE_DEB" | sha256sum -c -; then
  curl -fL --retry 3 --connect-timeout 20 \
    https://github.com/RobotLocomotion/drake/releases/download/v1.55.0/drake-dev_1.55.0-1_arm64-noble.deb \
    -o "$KUAVO_DRAKE_DEB"
fi
printf '%s  %s\n' "$KUAVO_DRAKE_SHA" "$KUAVO_DRAKE_DEB" | sha256sum -c -
mapfile -t KUAVO_APT_PACKAGES < src/wheel_arm_abs_vel/scripts/bootstrap/noble-apt-packages.txt
sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  "${KUAVO_APT_PACKAGES[@]}" "$KUAVO_DRAKE_DEB"

# Cmeel wheels share a prefix: install the complete locked set in one invocation.
/usr/bin/python3 -m pip install --upgrade --no-compile --no-deps \
  --target "$KUAVO_ABS_WS/.absolute-deps/python" -r src/wheel_arm_abs_vel/scripts/bootstrap/noble-python.lock

KUAVO_ROS_CANDIDATE="${KUAVO_ROS_PREFIX:-/opt/ros/noetic}"
if [[ ! -f "$KUAVO_ROS_CANDIDATE/setup.bash" && -f .absolute-deps/ros/install/setup.bash ]]; then
  KUAVO_ROS_CANDIDATE="$KUAVO_ABS_WS/.absolute-deps/ros/install"
fi
reuse_noetic() (
  [[ -f "$KUAVO_ROS_CANDIDATE/setup.bash" ]] || exit 1
  source "$KUAVO_ROS_CANDIDATE/setup.bash"
  export LD_LIBRARY_PATH="$KUAVO_ABS_WS/.absolute-deps/python/cmeel.prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  [[ "${ROS_DISTRO:-}" == noetic ]] || exit 1
  /usr/bin/python3 -c 'import rospy, rosbag, cv_bridge' || exit 1
  for package in angles catkin cmake_modules controller_manager cv_bridge diagnostic_updater \
      eigen_conversions geographic_msgs hardware_interface image_transport interactive_markers \
      kdl_parser realtime_tools robot_state_publisher roslint rostest tf tf2_geometry_msgs \
      tf2_ros transmission_interface joint_state_controller urdf xacro joy; do
    rospack find "$package" >/dev/null || exit 1
  done
  # Reject a system-urdfdom 4 underlay: its XML ABI differs from Pinocchio's 3.
  for library in liburdf.so libkdl_parser.so; do
    local linked_libraries
    linked_libraries="$(ldd "$KUAVO_ROS_CANDIDATE/lib/$library")" || exit 1
    grep -q 'liburdfdom_model.so.3.0' <<< "$linked_libraries" || exit 1
    if grep 'liburdfdom_' <<< "$linked_libraries" | grep -v '\.so\.3\.0 '; then
      exit 1
    fi
  done
  # Drake 1.55 requires C++23; older Noetic message headers use removed allocators.
  /usr/bin/c++ -std=c++23 -fsyntax-only -x c++ -I"$KUAVO_ROS_CANDIDATE/include" - <<'CPP' || exit 1
#include <ros/serialization.h>
#include <std_msgs/Header.h>
int main() {
  std_msgs::Header header;
  return ros::serialization::serializationLength(header) == 0;
}
CPP
  # A cached underlay can import successfully while its logger hierarchy is
  # freed by enumeration/shutdown. Reject it before reusing its shared libraries.
  /usr/bin/c++ -std=c++17 -x c++ -o .absolute-deps/rosconsole-lifecycle-check - -x none \
    $(pkg-config --cflags --libs rosconsole) <<'CPPLOG' || exit 1
#include <ros/console.h>
#include <map>
int main() {
  ros::console::initialize();
  std::map<std::string, ros::console::levels::Level> loggers;
  if (!ros::console::get_loggers(loggers)) return 1;
  if (!ros::console::get_loggers(loggers)) return 1;
  ros::console::shutdown();
  ROS_INFO("Logger lifecycle check after shutdown");
  return 0;
}
CPPLOG
  (ulimit -c 0; timeout 10 .absolute-deps/rosconsole-lifecycle-check) || exit 1
)
if reuse_noetic; then
  echo "Using existing Noetic: $KUAVO_ROS_CANDIDATE"
else
  echo 'Building a workspace-local Noetic underlay; existing ROS installations are preserved.'
  bash src/wheel_arm_abs_vel/scripts/bootstrap/ros_noble.sh
  KUAVO_ROS_CANDIDATE="$KUAVO_ABS_WS/.absolute-deps/ros/install"
  reuse_noetic || { echo "Built Noetic failed the Python, URDF ABI, C++ or logger lifecycle compatibility check." >&2; exit 1; }
fi
printf '%s\n' "$KUAVO_ROS_CANDIDATE" > .absolute-deps/ros-root
printf '/opt/drake\n' > .absolute-deps/drake-root
printf 'python3.12\n' > .absolute-deps/python-version
cp src/wheel_arm_abs_vel/scripts/bootstrap/deps_setup.bash .absolute-deps/setup.bash
source "$KUAVO_ROS_CANDIDATE/setup.bash"
source .absolute-deps/setup.bash
/usr/bin/python3 - <<'PY'
import numpy, scipy, quaternion, aiortc, psutil, yaml, pinocchio, numba
from pydrake.all import MultibodyPlant, MathematicalProgram
assert numpy.__version__ == '1.26.4', numpy.__version__
assert pinocchio.__version__ == '2.6.21', pinocchio.__version__
print('Runtime dependencies OK', numpy.__version__, scipy.__version__, pinocchio.__version__, numba.__version__)
PY
dpkg-query -W > .absolute-deps/installed-packages.txt
/usr/bin/python3 -m pip list --path "$KUAVO_ABS_WS/.absolute-deps/python" --format=freeze > .absolute-deps/installed-python.txt
bash src/wheel_arm_abs_vel/scripts/bootstrap/deps_fingerprint.sh > .absolute-deps/dependencies.sha256
echo 'Dependencies installed. Next: bash src/wheel_arm_abs_vel/scripts/build_workspace.sh'
