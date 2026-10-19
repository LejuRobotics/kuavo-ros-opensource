#!/bin/bash
set -e
# Never apply the absolute runtime policy to legacy or unrelated launches.
absolute_entry=false
if [[ "$1" == wheel_arm_abs_vel ]]; then
  case "$2" in
    load_kuavo_real_velocity_ik.launch|load_kuavo_mujoco_velocity_ik.launch|load_kuavo_absolute_sim_base.launch|quest_velocity_ik.launch|pico_velocity_ik.launch)
      absolute_entry=true ;;
    control.launch)
      selector=legacy
      for arg in "$@"; do
        case "$arg" in controller_type:=*) selector="${arg#*:=}" ;; esac
      done
      [[ "$selector" == wheel_arm_abs_vel ]] && absolute_entry=true ;;
  esac
fi
if [[ "$absolute_entry" != true ]]; then
  exec roslaunch "$@"
fi
package_dir="$(rospack find wheel_arm_abs_vel)"
resolved_version="$(/usr/bin/python3 "$package_dir/scripts/absolute_robot_version.py" "$@")"
export ROBOT_VERSION="$resolved_version"
for arg in "$@"; do
  case "$arg" in
    require_ik_hard_isolation:=*) export KUAVO_REQUIRE_IK_HARD_ISOLATION="${arg#*:=}" ;;
    arm_driver_reserved_cpus:=*) export KUAVO_ARM_DRIVER_RESERVED_CPUS="${arg#*:=}" ;;
  esac
done
if [[ "${KUAVO_RUNTIME_PARTITION:-1}" == 0 ]]; then
  exec roslaunch "$@"
fi
exec /usr/bin/python3 "$package_dir/scripts/runtime_partition.py" --launch /usr/bin/env roslaunch "$@"
