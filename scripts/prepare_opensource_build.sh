#!/usr/bin/env bash
# Sourced from the workspace root inside the delivery container.
# Open-source releases supply private dependencies in installed/.

# The development repository builds these dependencies from source. Keep its
# existing environment and build caches unchanged, even if installed/ exists.
if [[ -f src/humanoid-control/humanoid_interface/package.xml ]]; then
  return 0
fi

if [[ ! -f installed/setup.bash ]]; then
  echo "[ERROR] Missing installed/setup.bash; update the complete open-source checkout." >&2
  return 1
fi

if [[ ! -f /opt/drake/lib/libdrake.so ]]; then
  echo "[ERROR] Missing /opt/drake/lib/libdrake.so; use the delivered runtime image." >&2
  return 1
fi

source installed/setup.bash

# Changing --extend or forcing CMake does not discard catkin's cached package
# environments. Clean only when existing output used a different underlay.
if [[ -d build || -d devel ]]; then
  if ! python3 - <<"PY"
from pathlib import Path
import sys
import yaml

workspace = Path.cwd()
previous_build = workspace / ".catkin_tools/profiles/default/build.yaml"
try:
    previous = yaml.safe_load(previous_build.read_text()) or {}
    matches = previous.get("extend_path") == str(workspace / "installed")
except (OSError, yaml.YAMLError):
    matches = False
sys.exit(0 if matches else 1)
PY
  then
    echo "[INFO] Clearing build/devel caches to use the bundled installed dependencies."
    catkin clean --build --devel --yes
  fi
fi

catkin config --extend "${PWD}/installed"

# Prebuilt libraries depend on Drake even when the consuming target does not
# link Drake directly. Register the image's existing libraries for ld and ld.so.
printf "%s\n" /opt/drake/lib > /etc/ld.so.conf.d/kuavo-drake.conf
ldconfig
