# Copied to .absolute-deps/setup.bash by bootstrap/setup_noble.sh; source after ROS/devel.
_kuavo_deps_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export KUAVO_ABS_DRAKE_ROOT="$(cat "$_kuavo_deps_dir/drake-root")"
export KUAVO_ABS_ROS_ROOT="$(cat "$_kuavo_deps_dir/ros-root")"
_kuavo_python="$(cat "$_kuavo_deps_dir/python-version")"
export PATH="$KUAVO_ABS_DRAKE_ROOT/bin:$PATH"
export CMAKE_PREFIX_PATH="$KUAVO_ABS_DRAKE_ROOT${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$KUAVO_ABS_DRAKE_ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$_kuavo_deps_dir/python:$KUAVO_ABS_ROS_ROOT/lib/$_kuavo_python/site-packages:$KUAVO_ABS_DRAKE_ROOT/lib/$_kuavo_python/site-packages${PYTHONPATH:+:$PYTHONPATH}"
unset KUAVO_ABS_CMEEL_ROOT
if [[ -d "$_kuavo_deps_dir/python/cmeel.prefix" ]]; then
  export KUAVO_ABS_CMEEL_ROOT="$_kuavo_deps_dir/python/cmeel.prefix"
  export CMAKE_PREFIX_PATH="$KUAVO_ABS_CMEEL_ROOT:$CMAKE_PREFIX_PATH"
  export LD_LIBRARY_PATH="$KUAVO_ABS_CMEEL_ROOT/lib:$LD_LIBRARY_PATH"
  export PKG_CONFIG_PATH="$KUAVO_ABS_CMEEL_ROOT/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
  export PYTHONPATH="$KUAVO_ABS_CMEEL_ROOT/lib/$_kuavo_python/site-packages:$PYTHONPATH"
fi
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
unset _kuavo_deps_dir _kuavo_python
