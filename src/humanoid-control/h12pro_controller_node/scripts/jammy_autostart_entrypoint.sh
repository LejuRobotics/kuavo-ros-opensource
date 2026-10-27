#!/usr/bin/env bash
# 开机自启动容器入口：自动拉起 roscore + H12 遥控器监控
# 由 systemd 管理的 docker 容器调用，保持容器前台运行不死
set -eo pipefail

log() { echo "[kuavo-autostart $(date +%T)] $*"; }

# ---------- 环境初始化 ----------
source /root/kuavo_ws/docker/aarch64/setup_jammy_container_env.sh 2>/dev/null \
  || source /usr/local/bin/setup_jammy_container_env.sh 2>/dev/null \
  || { log "FATAL: 无法 source 环境脚本"; exit 1; }

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://kuavo_master:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-kuavo_master}"
export ROS_IP="${ROS_IP:-kuavo_master}"
export KUAVO_CONTROL_SCHEME="${KUAVO_CONTROL_SCHEME:-multi}"
export KUAVO_ROS_CONTROL_WS_PATH="${KUAVO_ROS_CONTROL_WS_PATH:-/root/kuavo_ws}"
export ROBOT_VERSION="${ROBOT_VERSION:-62}"
export STAIR_DETECTION_CAMERA="${STAIR_DETECTION_CAMERA:-orbbec}"

H12PRO_SCRIPTS="${KUAVO_ROS_CONTROL_WS_PATH}/src/humanoid-control/h12pro_controller_node/scripts"
MONITOR_SCRIPT="${H12PRO_SCRIPTS}/monitor_ocs2_h12pro.sh"
START_SCRIPT="${H12PRO_SCRIPTS}/start_ocs2_h12pro_node.sh"

export NODE_SCRIPT="${START_SCRIPT}"

if [ ! -f "${MONITOR_SCRIPT}" ]; then
    log "FATAL: monitor_ocs2_h12pro.sh 不存在: ${MONITOR_SCRIPT}"
    exit 1
fi

# ---------- 启动 roscore ----------
log "启动 roscore ..."
roscore &
ROSCORE_PID=$!

for i in $(seq 1 30); do
    if rosnode list >/dev/null 2>&1; then
        log "roscore 就绪 (PID ${ROSCORE_PID})"
        break
    fi
    sleep 1
done

if ! kill -0 "${ROSCORE_PID}" 2>/dev/null; then
    log "FATAL: roscore 启动失败"
    exit 1
fi

# ---------- 启动 H12 监控（前台运行，保持容器存活）----------
log "启动 H12PRO monitor (ROBOT_VERSION=${ROBOT_VERSION}, SCHEME=${KUAVO_CONTROL_SCHEME})"
exec bash "${MONITOR_SCRIPT}"
