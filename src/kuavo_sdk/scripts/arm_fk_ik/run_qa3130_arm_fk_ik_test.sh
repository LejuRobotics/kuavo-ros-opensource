#!/usr/bin/env bash
# QA#3130 kuavo5 手臂正逆解回归测试启动脚本
#
# 完整流程：
#   1. export ROBOT_VERSION=52
#   2. roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch
#   3. 运行本脚本（或其中的测试步骤）
#
# 用法：
#   ./run_qa3130_arm_fk_ik_test.sh              # 启动仿真并测试
#   ./run_qa3130_arm_fk_ik_test.sh --no-launch  # 仿真已启动，仅跑测试
#   ./run_qa3130_arm_fk_ik_test.sh --skip-motion

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
ROBOT_VERSION="${ROBOT_VERSION:-52}"
NO_LAUNCH=0
SKIP_MOTION=0
EXTRA_TEST_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-launch) NO_LAUNCH=1; shift ;;
        --skip-motion) SKIP_MOTION=1; shift ;;
        *) EXTRA_TEST_ARGS+=("$1"); shift ;;
    esac
done

source /opt/ros/noetic/setup.bash
cd "${PROJECT_DIR}"
source devel/setup.bash

export ROBOT_VERSION
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export LD_LIBRARY_PATH="/opt/drake/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

cleanup() {
    if [[ "${NO_LAUNCH}" -eq 0 ]]; then
        if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
            kill "${LAUNCH_PID}" 2>/dev/null || true
            sleep 2
            kill -9 "${LAUNCH_PID}" 2>/dev/null || true
        fi
        if [[ -n "${ROSCORE_PID:-}" ]] && kill -0 "${ROSCORE_PID}" 2>/dev/null; then
            kill "${ROSCORE_PID}" 2>/dev/null || true
        fi
    fi
}
trap cleanup EXIT INT TERM

launch_sim_background() {
    if ! pgrep -x rosmaster >/dev/null 2>&1; then
        roscore >/tmp/qa3130_roscore.log 2>&1 &
        ROSCORE_PID=$!
        sleep 3
    fi
    roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch joystick_type:=sim \
        >/tmp/qa3130_mujoco_sim.log 2>&1 &
    LAUNCH_PID=$!
}

wait_for_robot() {
    echo "等待仿真与 IK 服务就绪 (ROBOT_VERSION=${ROBOT_VERSION})..."
    local i
    for i in $(seq 1 90); do
        if rosservice list 2>/dev/null | grep -q "/ik/fk_srv" \
            && rosservice list 2>/dev/null | grep -q "/ik/two_arm_hand_pose_cmd_srv" \
            && rosnode list 2>/dev/null | grep -q "arms_ik_node" \
            && rosnode list 2>/dev/null | grep -q "humanoid_sqp_mpc"; then
            echo "服务已就绪 (等待 ${i} 轮)"
            sleep 10
            return 0
        fi
        sleep 5
    done
    echo "错误: 等待仿真/IK 服务超时"
    return 1
}

if [[ "${NO_LAUNCH}" -eq 0 ]]; then
    echo "启动 mujoco 仿真: ROBOT_VERSION=${ROBOT_VERSION}"
    launch_sim_background
    sleep 25
fi

wait_for_robot

TEST_CMD=(python3 "${SCRIPT_DIR}/test_qa3130_arm_fk_ik.py")
if [[ "${SKIP_MOTION}" -eq 1 ]]; then
    TEST_CMD+=(--skip-motion)
fi
if [[ ${#EXTRA_TEST_ARGS[@]} -gt 0 ]]; then
    TEST_CMD+=("${EXTRA_TEST_ARGS[@]}")
fi

echo "运行测试: ${TEST_CMD[*]}"
"${TEST_CMD[@]}"
echo "QA#3130 手臂正逆解测试通过"
