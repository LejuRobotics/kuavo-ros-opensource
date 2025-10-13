#!/bin/bash

# 蓝牙自动连接服务卸载脚本

set -e

# 配置变量
INSTALL_DIR="/opt/lejurobot/kuavo-bt-service"
SERVICE_NAME="kuavo-bt-service"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 日志函数
log() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查root权限
check_root() {
    if [[ $EUID -ne 0 ]]; then
        error "此脚本需要root权限运行"
        error "请使用: sudo $0"
        exit 1
    fi
}

# 主卸载函数
uninstall() {
    log "开始卸载蓝牙自动连接服务"
    
    # 停止服务
    if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
        log "停止服务"
        systemctl stop "$SERVICE_NAME" || warn "停止服务失败"
    fi
    
    # 禁用服务
    if systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
        log "禁用服务"
        systemctl disable "$SERVICE_NAME" || warn "禁用服务失败"
    fi
    
    # 删除服务文件
    if [[ -f "$SERVICE_FILE" ]]; then
        log "删除服务文件"
        rm -f "$SERVICE_FILE"
    fi
    
    # 删除安装目录
    if [[ -d "$INSTALL_DIR" ]]; then
        log "删除安装目录"
        rm -rf "$INSTALL_DIR"
    fi
    
    # 清理systemd
    log "清理systemd配置"
    systemctl daemon-reload
    systemctl reset-failed "$SERVICE_NAME" 2>/dev/null || true
    
    log "卸载完成"
}

# 主函数
main() {
    check_root
    uninstall
}

# 运行主函数
main "$@"