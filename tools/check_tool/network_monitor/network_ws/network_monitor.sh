#!/usr/bin/env bash

set -euo pipefail

log() {
    echo "[network_monitor] $*"
}

err() {
    echo "[network_monitor][ERROR] $*" >&2
}

# 1) 解析目录
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

# 解析/定位 network_ws 目录（脚本位于 network_monitor/ 下）
resolve_ws_dir() {
    # 优先使用环境变量
    if [[ -n "${NETWORK_MONITOR_WS:-}" && -d "${NETWORK_MONITOR_WS}" ]]; then
        echo "${NETWORK_MONITOR_WS}"
        return 0
    fi

    # 候选目录（按优先级从高到低）
    local candidates=(
        "${SCRIPT_DIR}/network_ws"                    # newcheck_tool/network_monitor/network_ws
        "${PROJECT_ROOT}/network_ws"                  # newcheck_tool/network_ws
        "${PROJECT_ROOT}/network_monitor/network_ws"  # 兼容冗余
        "${PROJECT_ROOT}/../network_monitor/network_ws"
    )
    for d in "${candidates[@]}"; do
        if [[ -d "${d}" ]]; then
            echo "${d}"
            return 0
        fi
    done
    return 1
}

WS_DIR="$(resolve_ws_dir || true)"
if [[ -z "${WS_DIR}" ]]; then
    err "未能自动定位 network_ws 工作空间目录。可通过设置环境变量 NETWORK_MONITOR_WS 指定。"
    err "尝试的位置包括:"
    err "  ${SCRIPT_DIR}/network_ws"
    err "  ${PROJECT_ROOT}/network_ws"
    err "  ${PROJECT_ROOT}/network_monitor/network_ws"
    err "  ${PROJECT_ROOT}/../network_monitor/network_ws"
    exit 1
fi
log "已定位工作空间: ${WS_DIR}"

PKG_DIR="${WS_DIR}/src/network_monitor"

# 2) 检查包是否存在
if [[ ! -d "${PKG_DIR}" ]]; then
    err "未找到包目录: ${PKG_DIR}"
    err "请确认 network_monitor 包位于: ${SCRIPT_DIR}/network_ws/src/network_monitor"
    exit 1
fi

# 3) 尝试加载 ROS1 环境（如未加载）
ensure_ros_env() {
    if command -v rosversion >/dev/null 2>&1; then
        return 0
    fi
    # 依次尝试常见 ROS1 发行版
    for distro in noetic melodic kinetic; do
        if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
            log "检测到 ROS: ${distro}，自动 source /opt/ros/${distro}/setup.bash"
            # shellcheck disable=SC1090
            source "/opt/ros/${distro}/setup.bash"
            break
        fi
    done
    if ! command -v rosversion >/dev/null 2>&1; then
        err "未检测到 ROS 环境，请确认已安装并可用（ROS1: noetic/melodic/kinetic）。"
        exit 1
    fi
}

# 4) 编译（如有必要）
compile_if_needed() {
    local setup_file="${WS_DIR}/devel/setup.bash"
    local need_build=0

    if [[ ! -f "${setup_file}" ]]; then
        need_build=1
    fi
    if [[ ! -d "${WS_DIR}/build" || ! -d "${WS_DIR}/devel" ]]; then
        need_build=1
    fi

    if [[ ${need_build} -eq 1 ]]; then
        log "未发现已编译产物，开始编译: ${WS_DIR}"
        pushd "${WS_DIR}" >/dev/null
        catkin_make
        popd >/dev/null
        log "编译完成"
    else
        log "检测到已编译环境，跳过编译"
    fi
}

# 5) 加载工作空间环境
source_ws() {
    local setup_file="${WS_DIR}/devel/setup.bash"
    if [[ ! -f "${setup_file}" ]]; then
        err "缺少 ${setup_file}，请先编译。"
        exit 1
    fi
    # shellcheck disable=SC1090
    source "${setup_file}"
}

# 5.1) 赋权可执行脚本
ensure_scripts_executable() {
    local scripts_dir="${WS_DIR}/src/network_monitor/scripts"
    if [[ -d "${scripts_dir}" ]]; then
        chmod +x "${scripts_dir}"/*.py 2>/dev/null || true
    fi
}

# 5.2) 校验包是否可见
verify_package_visible() {
    if ! rospack find network_monitor >/dev/null 2>&1; then
        log "rospack 未找到 network_monitor，尝试强制重编译后重载环境"
        pushd "${WS_DIR}" >/dev/null
        catkin_make --force-cmake
        popd >/dev/null
        source_ws
        if ! rospack find network_monitor >/dev/null 2>&1; then
            err "仍未找到 package 'network_monitor'"
            echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}"
            echo "请确认存在目录: ${WS_DIR}/src/network_monitor"
            exit 1
        fi
    fi
}

# 6) 启动函数
start_server() {
    log "启动上位机监测（服务端）..."
    exec rosrun network_monitor shangweiji_server.py
}

start_client() {
    log "启动下位机监测（客户端）..."
    exec rosrun network_monitor xiaweiji_client.py
}

main_menu() {
    echo ""
    echo "==== 网络连接监控 启动器 ===="
    echo "1) 下位机监测启动（client）"
    echo "2) 上位机监测启动（server）"
    echo "q) 退出"
    echo -n "请选择: "
    read -r choice
    case "${choice}" in
        1)
            start_client
            ;;
        2)
            start_server
            ;;
        q|Q)
            log "已退出"
            exit 0
            ;;
        *)
            err "无效选项: ${choice}"
            exit 2
            ;;
    esac
}

# --- 执行顺序 ---
ensure_ros_env
compile_if_needed
source_ws
ensure_scripts_executable
verify_package_visible
main_menu


