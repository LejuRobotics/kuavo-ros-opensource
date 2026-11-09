#!/usr/bin/env bash
# Build wheel_arm_abs_vel and its workspace dependency closure, including first builds.
set -eo pipefail
workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$workspace"
source /etc/os-release
if [[ "$ID-$VERSION_ID-$(uname -m)" == ubuntu-24.04-aarch64 ]]; then
  bootstrap="$workspace/src/wheel_arm_abs_vel/scripts/bootstrap"
  fingerprint="$(bash "$bootstrap/deps_fingerprint.sh")"
  if [[ ! -f .absolute-deps/setup.bash || ! -f .absolute-deps/ros-root ||
        "$(cat .absolute-deps/dependencies.sha256 2>/dev/null || true)" != "$fingerprint" ]]; then
    bash "$bootstrap/setup_noble.sh"
  fi
  # Avoid inheriting ROS or Cmeel libraries from an unrelated workspace.
  unset PYTHONHOME PYTHONPATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROS_ROOT LD_LIBRARY_PATH PKG_CONFIG_PATH
  export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
  ROS_PREFIX="$(cat .absolute-deps/ros-root)"
  DRAKE_PREFIX="$(cat .absolute-deps/drake-root)"
  source "$ROS_PREFIX/setup.bash"
  source .absolute-deps/setup.bash
else
  source "${ROS_PREFIX:-/opt/ros/noetic}/setup.bash"
fi
cmake_extra=()
if [[ -n "${KUAVO_ABS_CMEEL_ROOT:-}" ]]; then
  cmake_extra+=("-DCMAKE_PREFIX_PATH=$KUAVO_ABS_CMEEL_ROOT;$DRAKE_PREFIX;$workspace/devel;$ROS_PREFIX")
fi
export ROBOT_VERSION="${ROBOT_VERSION:-62}"
export ROS_PACKAGE_PATH="$workspace/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
jobs="${JOBS:-$(nproc)}"
package_jobs="${PACKAGE_JOBS:-$(nproc)}"

catkin config --profile velocity-ik --extend "${ROS_PREFIX:-/opt/ros/noetic}" \
  --no-install --link-devel --build-space build/velocity-ik --devel-space devel \
  --log-space logs/velocity-ik -j"$jobs" -p"$package_jobs" --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON \
  -Ddrake_DIR="${DRAKE_PREFIX:-/opt/drake}/lib/cmake/drake" \
  -DCMAKE_ASM_COMPILER=/usr/bin/as -DKUAVO_SOLVER_BUILD_PYTHON=OFF "${cmake_extra[@]}"
catkin profile set velocity-ik

# Initialize Catkin/message generation before selecting an explicit make target.
catkin build kuavo_msgs --no-deps -j"$jobs" -p"$package_jobs" --mem-limit 80% --no-status
# The assets ALL target installs packages and rewrites hardware configuration.
# Only its C++ library is required for compilation; dependencies must be provisioned first.
catkin build kuavo_assets --no-deps -j"$jobs" -p"$package_jobs" --mem-limit 80% --no-status \
  --make-args kuavo_assets

# Resolve from package.xml rather than retaining an outdated manual package list.
# --no-deps is safe here because every workspace dependency is explicitly selected,
# except kuavo_assets, whose required library was built above.
package_list=$(catkin list --rdeps wheel_arm_abs_vel --unformatted | python3 -c '
import sys
import yaml
package = yaml.safe_load(sys.stdin)["wheel_arm_abs_vel"]
names = set(package["build_depend"]) | set(package["run_depend"])
names.add("wheel_arm_abs_vel")
names.remove("kuavo_assets")
print("\n".join(sorted(names)))
')
mapfile -t packages <<< "$package_list"
catkin build "${packages[@]}" --no-deps -j"$jobs" -p"$package_jobs" --mem-limit 80% --no-status
