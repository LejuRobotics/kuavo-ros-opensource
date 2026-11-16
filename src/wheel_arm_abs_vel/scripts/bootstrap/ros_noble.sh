#!/usr/bin/env bash
# Build the pinned Noetic source closure locally when no compatible underlay exists.
set -e -o pipefail
KUAVO_ABS_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd -P)"
cd "$KUAVO_ABS_WS"
export ROS_DISTRO=noetic ROS_VERSION=1 ROS_PYTHON_VERSION=3
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 PIP_DISABLE_PIP_VERSION_CHECK=1
export PYTHONUNBUFFERED=1
unset CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH ROS_PACKAGE_PATH ROS_ROOT PKG_CONFIG_PATH
# Anchor the underlay explicitly; otherwise catkin discovers the outer workspace profile.
mkdir -p .absolute-deps/ros/src .absolute-deps/ros/.catkin_tools/profiles/default
if [[ ! -f .absolute-deps/ros/sources-ready ]]; then
  /usr/bin/python3 -m venv --without-pip --system-site-packages .absolute-deps/ros-tools
  .absolute-deps/ros-tools/bin/python -m pip install --no-compile vcstool==0.3.0
  # vcstool's tar client does not honor --skip-existing; filter completed
  # packages before retrying so a network failure cannot erase cached sources.
  KUAVO_ROS_FETCH_OK=false
  for KUAVO_ROS_FETCH_ATTEMPT in 1 2 3; do
    /usr/bin/python3 - <<'PYFETCH'
from pathlib import Path
import yaml
manifest = yaml.safe_load(Path('src/wheel_arm_abs_vel/scripts/bootstrap/noetic-noble.rosinstall').read_text())
pending = [entry for entry in manifest if not
    (Path('.absolute-deps/ros/src') / entry['tar']['local-name'] / 'package.xml').is_file()]
Path('.absolute-deps/ros/pending.rosinstall').write_text(yaml.safe_dump(pending))
print('Noetic packages remaining:', len(pending))
PYFETCH
    if .absolute-deps/ros-tools/bin/vcs import --workers 4 \
        --input .absolute-deps/ros/pending.rosinstall .absolute-deps/ros/src; then
      KUAVO_ROS_FETCH_OK=true
      break
    fi
    echo "Noetic source download attempt $KUAVO_ROS_FETCH_ATTEMPT failed; retrying incomplete packages."
  done
  [[ "$KUAVO_ROS_FETCH_OK" == true ]] || { echo 'Noetic source download failed; rerun this build after restoring network.' >&2; exit 1; }
  touch .absolute-deps/ros/sources-ready
fi
if ! printf '%s\n' '18bd4f15fcbbd356e61c12c2e65969686b1ee6c9f9e882a98d5721713eb44ac8  .absolute-deps/ros/src/rosconsole/src/rosconsole/impl/rosconsole_log4cxx.cpp' | sha256sum -c -; then
if ! printf '%s\n' 'e07d22add4f20ee6412e3c08f550913b7e33674c6ae1b4bcfa60c82631051274  .absolute-deps/ros/src/rosconsole/src/rosconsole/impl/rosconsole_log4cxx.cpp' | sha256sum -c -; then
  curl -fL --retry 3 --connect-timeout 20 \
    https://raw.githubusercontent.com/ros/rosconsole/9f930c007dd40aa7ede771b8859b529e024d7bfb/src/rosconsole/impl/rosconsole_log4cxx.cpp \
    -o .absolute-deps/ros/src/rosconsole/src/rosconsole/impl/rosconsole_log4cxx.cpp
printf '%s\n' '7b9e445570c288c19b8fb05f1b008cd386badf3f2ccc80516194b31e5a897e9f  .absolute-deps/ros/src/rosconsole/src/rosconsole/impl/rosconsole_log4cxx.cpp' | sha256sum -c -
# Noble log4cxx 1.1 adds the short-file-name argument to LocationInfo.
sed -i 's/LocationInfo(file, function, line)/LocationInfo(file, log4cxx::spi::LocationInfo::calcShortFileName(file), function, line)/' \
  .absolute-deps/ros/src/rosconsole/src/rosconsole/impl/rosconsole_log4cxx.cpp
printf '%s\n' 'e07d22add4f20ee6412e3c08f550913b7e33674c6ae1b4bcfa60c82631051274  .absolute-deps/ros/src/rosconsole/src/rosconsole/impl/rosconsole_log4cxx.cpp' | sha256sum -c -
fi
# log4cxx 1.1 returns a borrowed repository pointer. Creating an owning shared
# pointer here deletes the global hierarchy during enumeration or shutdown.
/usr/bin/python3 - <<'PYLOG'
from pathlib import Path
p = Path('.absolute-deps/ros/src/rosconsole/src/rosconsole/impl/rosconsole_log4cxx.cpp')
s = p.read_text()
s = s.replace('auto repo = log4cxx::spi::LoggerRepositoryPtr(log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME)->getLoggerRepository());', 'auto* repo = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME)->getLoggerRepository();')
s = s.replace('static_cast<log4cxx::spi::LoggerRepositoryPtr>(log4cxx::Logger::getRootLogger()->getLoggerRepository())->shutdown();', 'log4cxx::Logger::getRootLogger()->getLoggerRepository()->shutdown();')
p.write_text(s)
PYLOG
printf '%s\n' '18bd4f15fcbbd356e61c12c2e65969686b1ee6c9f9e882a98d5721713eb44ac8  .absolute-deps/ros/src/rosconsole/src/rosconsole/impl/rosconsole_log4cxx.cpp' | sha256sum -c -
fi
# These two packages force an older language level instead of honoring the workspace.
sed -i 's/CMAKE_CXX_STANDARD 11/CMAKE_CXX_STANDARD 17/' .absolute-deps/ros/src/joystick_drivers/joy/CMakeLists.txt
sed -i 's/CMAKE_CXX_STANDARD 14/CMAKE_CXX_STANDARD 17/' .absolute-deps/ros/src/kdl_parser/kdl_parser/CMakeLists.txt
# Pinocchio 2.6 uses urdfdom 3/TinyXML. Noble's urdfdom 4/TinyXML2 exports
# the same C++ symbol names with incompatible return types; never mix both.
KUAVO_NOETIC_CMEEL="$KUAVO_ABS_WS/.absolute-deps/python/cmeel.prefix"
export LD_LIBRARY_PATH="$KUAVO_NOETIC_CMEEL/lib"
read -r KUAVO_BUILD_JOBS KUAVO_PACKAGE_JOBS KUAVO_RAM_MB < <(/usr/bin/python3 src/wheel_arm_abs_vel/scripts/bootstrap/jobs.py 1024 8)
catkin config --workspace "$KUAVO_ABS_WS/.absolute-deps/ros" \
  --env-cache --install --install-space "$KUAVO_ABS_WS/.absolute-deps/ros/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 \
  -Durdfdom_DIR="$KUAVO_NOETIC_CMEEL/lib/urdfdom/cmake" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_ENABLE_TESTING=OFF -DBUILD_TESTING=OFF
catkin build --workspace "$KUAVO_ABS_WS/.absolute-deps/ros" \
  -j"$KUAVO_BUILD_JOBS" -p"$KUAVO_PACKAGE_JOBS" --no-status 2>&1 | tee .absolute-deps/ros-build.log
source .absolute-deps/ros/install/setup.bash
/usr/bin/python3 -c 'import rospy, rosbag, cv_bridge; print("Noetic / Python 3.12 imports OK")'
