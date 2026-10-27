#!/usr/bin/env bash
# 一键部署 Kuavo 开机自启动（仅需在宿主机执行一次）
# 功能:
#   1. 创建 systemd 服务，开机自动启动 Docker 容器 + ROS + H12 监控
#   2. 启用并启动服务
# 用法: sudo bash tools/setup_kuavo_autostart.sh
set -eo pipefail

if [ "$(id -u)" != "0" ]; then
    echo "请用 sudo 运行: sudo bash $0"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# ---------- 架构检测 ----------
ARCH=$(uname -m)
# 工作空间路径：取脚本所在仓库根目录
WS_HOST="${REPO_ROOT}"

# ROBOT_VERSION：优先当前环境变量，其次架构默认值
if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
    DEFAULT_ROBOT_VERSION="63"
else
    DEFAULT_ROBOT_VERSION="40"
fi
ROBOT_VERSION="${ROBOT_VERSION:-$DEFAULT_ROBOT_VERSION}"

# 镜像 TAG：优先当前环境变量，其次从 docker/run_jammy_aarch64.sh 解析默认值
if [ -z "${IMAGE_TAG:-}" ] && [ -f "${REPO_ROOT}/docker/run_jammy_aarch64.sh" ]; then
    IMAGE_TAG=$(grep -oP 'IMAGE_TAG="?\$\{IMAGE_TAG:-[^}]*\}"?' "${REPO_ROOT}/docker/run_jammy_aarch64.sh" | head -1 | sed 's/.*:-\([^}]*\).*/\1/')
fi
IMAGE_TAG="${IMAGE_TAG:-kuavo-jammy-aarch64:wheel-mpc-clean}"
CONTAINER_NAME="${CONTAINER_NAME:-kuavo_autostart}"

echo "=========================================="
echo " Kuavo 开机自启动部署"
echo "=========================================="
echo " 架构:     ${ARCH}"
echo " 工作空间: ${WS_HOST}"
echo " 版本:     ${ROBOT_VERSION}"
echo " 镜像:     ${IMAGE_TAG}"
echo "=========================================="

# ---------- Step 1: 创建 systemd 服务 ----------
echo ""
echo "[1/2] 创建 systemd 服务 /etc/systemd/system/kuavo-autostart.service ..."

# 用户家目录用于挂载 .ros .ccache .config/lejuconfig
REAL_HOME=$(eval echo ~${SUDO_USER:-root})
CCACHE_DIR="${REAL_HOME}/.ccache"
mkdir -p "${REAL_HOME}/.ros" "${CCACHE_DIR}" "${REAL_HOME}/.config/lejuconfig"

cat > /etc/systemd/system/kuavo-autostart.service << SYSTEMDEOF
[Unit]
Description=Kuavo Robot Docker Container (Autostart)
After=docker.service network.target
Requires=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash -c '\\
    if docker ps -aq -f name=^${CONTAINER_NAME}\$ | grep -q . ; then \\
        docker start ${CONTAINER_NAME} ; \\
    else \\
        docker run -d \\
            --restart=always \\
            --name ${CONTAINER_NAME} \\
            --net=host \\
            --privileged \\
            --ulimit rtprio=99 \\
            --cap-add=sys_nice \\
            --group-add=dialout \\
            --add-host kuavo_master:192.168.26.1 \\
            -e KUAVO_WS=/root/kuavo_ws \\
            -e ROS_INSTALL=/opt/ros/noetic \\
            -e DRAKE_PREFIX=/opt/drake \\
            -e DISPLAY=${DISPLAY:-:1.0} \\
            -e ROBOT_VERSION=${ROBOT_VERSION} \\
            -e ROS_MASTER_URI=http://kuavo_master:11311 \\
            -e ROS_HOSTNAME=kuavo_master \\
            -e KUAVO_CONTROL_SCHEME=multi \\
            -e STAIR_DETECTION_CAMERA=orbbec \\
            -e DISABLE_ROS1_EOL_WARNINGS=1 \\
            -v /dev:/dev \\
            -v ${WS_HOST}:/root/kuavo_ws:rw \\
            -v ${WS_HOST}/build:/root/kuavo_ws/build:rw \\
            -v ${WS_HOST}/devel:/root/kuavo_ws/devel:rw \\
            -v ${REAL_HOME}/.ros:/root/.ros:rw \\
            -v ${CCACHE_DIR}:/root/.ccache:rw \\
            -v ${REAL_HOME}/.config/lejuconfig:/root/.config/lejuconfig:rw \\
            ${IMAGE_TAG} \\
            /root/kuavo_ws/src/humanoid-control/h12pro_controller_node/scripts/jammy_autostart_entrypoint.sh ; \\
    fi'
ExecStop=/usr/bin/docker stop -t 10 ${CONTAINER_NAME}
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
SYSTEMDEOF

# ---------- Step 2: 启用并启动服务 ----------
echo "[2/2] 启用并启动 kuavo-autostart 服务 ..."
systemctl daemon-reload
systemctl enable kuavo-autostart.service
systemctl start kuavo-autostart.service

sleep 3

if systemctl is-active --quiet kuavo-autostart.service; then
    echo ""
    echo "=========================================="
    echo "  部署成功！"
    echo "  systemd 服务状态:"
    systemctl status kuavo-autostart.service --no-pager -l 2>/dev/null || true
    echo ""
    echo "  查看容器日志:  docker logs -f ${CONTAINER_NAME}"
    echo "  停用自启动:    sudo systemctl disable kuavo-autostart"
    echo "  停止容器:      sudo systemctl stop kuavo-autostart"
    echo "  删除容器:      docker rm -f ${CONTAINER_NAME}"
    echo "=========================================="
else
    echo ""
    echo "=========================================="
    echo "  部署失败！请检查日志:"
    echo "    journalctl -u kuavo-autostart -n 50 --no-pager"
    echo "    docker logs ${CONTAINER_NAME}"
    echo "=========================================="
    exit 1
fi
