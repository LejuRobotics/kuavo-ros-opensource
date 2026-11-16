#!/bin/sh
_kuavo_m="$(uname -m)"
case "$_kuavo_m" in
  aarch64|arm64) _kuavo_arch=aarch64 ;;
  x86_64|amd64) _kuavo_arch=x86_64 ;;
  *) _kuavo_arch="$_kuavo_m" ;;
esac
if [ -n "${_KUAVO_MULTIARCH_ROOT:-}" ]; then
  _kuavo_script_dir="${_KUAVO_MULTIARCH_ROOT}"
  unset _KUAVO_MULTIARCH_ROOT
elif [ -n "${BASH_VERSION:-}" ] && [ -n "${BASH_SOURCE:-}" ]; then
  _kuavo_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
elif [ -n "${ZSH_VERSION:-}" ]; then
  eval '_kuavo_script_dir="$(cd "$(dirname "${(%):-%x}")" && pwd)"'
else
  case "$0" in
    */*) _kuavo_script_dir="$(cd "$(dirname "$0")" && pwd)" ;;
    *) _kuavo_script_dir="$(pwd)" ;;
  esac
fi
_kuavo_arch_dir="${_kuavo_script_dir}/${_kuavo_arch}"
if [ ! -d "$_kuavo_arch_dir" ]; then
  echo "ERROR: missing install space for arch ${_kuavo_arch}: ${_kuavo_arch_dir}" >&2
  (return 1 2>/dev/null) || exit 1
fi
export LD_LIBRARY_PATH="${_kuavo_arch_dir}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PATH="${_kuavo_arch_dir}/bin${PATH:+:${PATH}}"
export CMAKE_PREFIX_PATH="${_kuavo_arch_dir}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export KUAVO_ASSETS_PATH="${_kuavo_arch_dir}/share/kuavo_assets"
export KUAVO_COMMON_PATH="${_kuavo_arch_dir}/share/kuavo_common"
export KUAVO_SOLVER_PATH="${_kuavo_arch_dir}/share/kuavo_solver"
export HARDWARE_PLANT_PATH="${_kuavo_arch_dir}/share/hardware_plant"
