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
else
  case "$0" in
    */*) _kuavo_script_dir="$(cd "$(dirname "$0")" && pwd)" ;;
    *) _kuavo_script_dir="$(pwd)" ;;
  esac
fi
_kuavo_arch_dir="${_kuavo_script_dir}/${_kuavo_arch}"
_kuavo_target="${_kuavo_arch_dir}/setup.sh"
if [ ! -f "$_kuavo_target" ]; then
  echo "ERROR: missing install space for arch ${_kuavo_arch}: ${_kuavo_target}" >&2
  (return 1 2>/dev/null) || exit 1
fi
for _kuavo_sub in bin lib include share tests; do
  _kuavo_top="${_kuavo_script_dir}/${_kuavo_sub}"
  if [ ! -e "$_kuavo_top" ] || [ -L "$_kuavo_top" ]; then
    ln -sfn "${_kuavo_arch}/${_kuavo_sub}" "$_kuavo_top"
  fi
done
export _CATKIN_SETUP_DIR="$_kuavo_arch_dir"
# shellcheck disable=SC1090
. "$_kuavo_target"
