#!/bin/bash
set -e
package_dir="$(rospack find wheel_arm_abs_vel)"
# Source the same pinned Noble dependencies used at build time.
workspace_dir="$(cd "$package_dir/../.." && pwd)"
if [[ -f "$workspace_dir/.absolute-deps/setup.bash" ]]; then
  source "$workspace_dir/.absolute-deps/setup.bash"
fi
mode="${1:?Expected runtime role}"
shift
if [[ "${KUAVO_SCHED_APPLIED:-}" != 1 && "${KUAVO_RUNTIME_PARTITION:-1}" != 0 ]]; then
  exec /usr/bin/python3 "$package_dir/scripts/runtime_partition.py" "$0" "$mode" "$@"
fi
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$package_dir/scripts:${PYTHONPATH:-}"
case "$mode" in
  controller)
    # EtherCAT loads its link driver by name with dlopen().
    case "$(uname -m)" in
      x86_64) ethercat_lib_dir="$package_dir/../kuavo-ros-control-lejulib/hardware_plant/lib/EC_Master/lib" ;;
      # Use the link driver shipped with the ARM64 SDK linked by hardware CMake.
      aarch64) ethercat_lib_dir="$package_dir/../kuavo-ros-control-lejulib/hardware_plant/lib/EC_Master_arm64/Bin/Linux/aarch64" ;;
      *) ethercat_lib_dir="" ;;
    esac
    if [[ -n "$ethercat_lib_dir" && -d "$ethercat_lib_dir" ]]; then
      export LD_LIBRARY_PATH="$ethercat_lib_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
    exec "$@"
    ;;
  background) exec "$@" ;;
  ik) shift; exec rosrun wheel_arm_abs_vel pinocchio_velocity_ik_node "$@" ;;
  reference_services) shift; exec rosrun wheel_arm_abs_vel fixed_ik_reference_services_node "$@" ;;
  mapper) shift; exec /usr/bin/python3 "$package_dir/scripts/ik_ros_velocity.py" "$@" ;;
  quest) shift; exec /usr/bin/python3 "$package_dir/scripts/monitor_quest3_velocity.py" "$@" ;;
  *) echo "Unknown runtime role: $mode" >&2; exit 2 ;;
esac
