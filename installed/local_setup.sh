#!/bin/sh
CATKIN_SETUP_UTIL_ARGS="--extend --local"
if [ -n "${_KUAVO_MULTIARCH_ROOT:-}" ]; then
  # shellcheck disable=SC1091
  . "${_KUAVO_MULTIARCH_ROOT}/setup.sh"
else
  # shellcheck disable=SC1091
  . "$(cd "$(dirname "$0")" && pwd)/setup.sh"
fi
unset CATKIN_SETUP_UTIL_ARGS
