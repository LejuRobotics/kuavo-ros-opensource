#!/bin/bash
# 依次（串行）运行 joint=1..6 的标定流程：
#   ./run_arm_calibration.sh -- --joint N
#
# 用法：
#   ./run_arm_calibration_joints_1_6.sh [--build] [--mocap-sdk=optitrack|nokov] [--continue-on-fail] [-- <额外透传给 run_data_collection.py 的参数...>]
#
# 说明：
# - 每个 joint 会等待上一轮流程完全退出后才开始下一轮
# - 默认任意一轮失败即退出（返回该轮 exit code）；加 --continue-on-fail 可继续跑完 1..6

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTINUE_ON_FAIL=0
CALIB_ARGS=()
PY_ARGS=()

# 解析参数：把 --build/--mocap-sdk= 传给 run_arm_calibration.sh，其余在 -- 后透传到 python
while [[ $# -gt 0 ]]; do
  case "$1" in
    --continue-on-fail)
      CONTINUE_ON_FAIL=1
      shift
      ;;
    --build|--mocap-sdk=*)
      CALIB_ARGS+=("$1")
      shift
      ;;
    --)
      shift
      PY_ARGS+=("$@")
      break
      ;;
    *)
      # 默认把未知参数也交给 python（与 run_arm_calibration.sh 行为一致）
      PY_ARGS+=("$1")
      shift
      ;;
  esac
done

overall_rc=0

for joint in  2 3 4 5 6; do
  echo "=========================================="
  echo "开始 joint=${joint} 标定：./run_arm_calibration.sh ${CALIB_ARGS[*]:-} -- --joint ${joint} ${PY_ARGS[*]:-}"
  echo "------------------------------------------"

  set +e
  "$SCRIPT_DIR/run_arm_calibration.sh" "${CALIB_ARGS[@]}" -- --joint "$joint" "${PY_ARGS[@]}"
  rc=$?
  set -e

  if [[ $rc -ne 0 ]]; then
    echo "joint=${joint} 失败，exit code=${rc}"
    overall_rc=$rc
    if [[ $CONTINUE_ON_FAIL -ne 1 ]]; then
      echo "未指定 --continue-on-fail，终止后续 joint。"
      exit "$rc"
    fi
  else
    echo "joint=${joint} 完成。"
  fi
done

exit "$overall_rc"

