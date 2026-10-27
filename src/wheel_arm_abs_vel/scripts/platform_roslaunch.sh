#!/bin/bash
set -e
# Never apply the absolute runtime policy to legacy or unrelated launches.
absolute_entry=false
fence_entry=false
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
elif [[ "$1" == noitom_hi5_hand_udp_python ]]; then
  # Quest3 遥操作入口：把 IK 线程放进分区策略算出的预留核，避让相机。
  #
  # 这里刻意不收 humanoid_controllers load_kuavo_real_wheel.launch：VR 与运控是
  # 两棵独立的 roslaunch 树（load_kuavo_real_wheel.launch 里 include VR 的那段是
  # 注释掉的），Quest3 这棵自己就能算出一致的 profile；而给运控树套围栏只会把
  # nodelet 进程的核池从 10 个收到 8 个，对 IK 延迟没有收益，却是安全关键路径。
  case "$2" in
    launch_quest3_ik.launch|launch_quest3_ik_videostream_orbbec.launch|launch_dual_quest3_ik.launch)
      fence_entry=true ;;
  esac
fi
if [[ "$absolute_entry" != true && "$fence_entry" != true ]]; then
  exec roslaunch "$@"
fi
# 包目录优先用 rospack（安装在 devel/install 空间时的标准路径）；本机只构建
# 运行需要的包，wheel_arm_abs_vel 不在构建列表里，rospack 会返回非零。脚本
# 自己就在这个包内，此时按脚本位置定位同目录的 .py 脚本。
package_dir="$(rospack find wheel_arm_abs_vel 2>/dev/null || true)"
scripts_dir=""
if [[ -n "$package_dir" && -d "$package_dir/scripts" ]]; then
  scripts_dir="$package_dir/scripts"
else
  here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  for candidate in "$here" "$here/.." "$here/../scripts"; do
    if [[ -f "$candidate/runtime_partition.py" ]]; then
      scripts_dir="$candidate"
      break
    fi
  done
fi
if [[ -z "$scripts_dir" ]]; then
  echo "platform_roslaunch: cannot locate runtime_partition.py next to $0" >&2
  exit 1
fi
if [[ "$absolute_entry" == true ]]; then
  # The project installer places Drake here. Keep the fallback inside the
  # absolute entry only; legacy launch paths above retain their environment.
  if [[ -d /opt/drake/lib ]]; then
    export LD_LIBRARY_PATH="/opt/drake/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi
  if ! /usr/bin/python3 -c 'import pydrake' >/dev/null 2>&1; then
    if [[ -d /opt/drake/lib/python3.8/site-packages ]]; then
      export PYTHONPATH="/opt/drake/lib/python3.8/site-packages${PYTHONPATH:+:$PYTHONPATH}"
    fi
  fi
  resolved_version="$(/usr/bin/python3 "$scripts_dir/absolute_robot_version.py" "$@")"
  export ROBOT_VERSION="$resolved_version"
fi
# Scope built-in scheduler defaults to the invoking control pipeline.
if [[ "$absolute_entry" == true ]]; then
  export KUAVO_RUNTIME_ENTRY=absolute
else
  export KUAVO_RUNTIME_ENTRY=quest3
fi
for arg in "$@"; do
  case "$arg" in
    require_ik_hard_isolation:=*) export KUAVO_REQUIRE_IK_HARD_ISOLATION="${arg#*:=}" ;;
    arm_driver_reserved_cpus:=*) export KUAVO_ARM_DRIVER_RESERVED_CPUS="${arg#*:=}" ;;
  esac
done
if [[ "${KUAVO_RUNTIME_PARTITION:-1}" == 0 ]]; then
  exec roslaunch "$@"
fi
exec /usr/bin/python3 "$scripts_dir/runtime_partition.py" --launch /usr/bin/env roslaunch "$@"
