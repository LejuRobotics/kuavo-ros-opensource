#!/bin/zsh
CATKIN_SHELL=zsh
export _KUAVO_MULTIARCH_ROOT="$(cd "$(dirname "${(%):-%x}")" && pwd)"
emulate sh
# shellcheck disable=SC1091
source "${_KUAVO_MULTIARCH_ROOT}/local_setup.sh"
emulate zsh
