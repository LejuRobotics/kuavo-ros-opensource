#!/usr/bin/env bash
# Build wheel_arm_abs_vel and its workspace dependency closure, including first builds.
set -eo pipefail
workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$workspace"
source "${ROS_PREFIX:-/opt/ros/noetic}/setup.bash"
export ROBOT_VERSION="${ROBOT_VERSION:-62}"
export ROS_PACKAGE_PATH="$workspace/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
jobs="${JOBS:-16}"
package_jobs="${PACKAGE_JOBS:-6}"

catkin config --profile velocity-ik --extend "${ROS_PREFIX:-/opt/ros/noetic}" \
  --no-install --link-devel --build-space build/velocity-ik --devel-space devel \
  --log-space logs/velocity-ik -j"$jobs" -p"$package_jobs" --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON \
  -Ddrake_DIR="${DRAKE_PREFIX:-/opt/drake}/lib/cmake/drake" \
  -DCMAKE_ASM_COMPILER=/usr/bin/as -DKUAVO_SOLVER_BUILD_PYTHON=OFF
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
