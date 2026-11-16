#!/bin/bash
CATKIN_SHELL=bash
export _KUAVO_MULTIARCH_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_KUAVO_MULTIARCH_ROOT}/local_setup.sh"
