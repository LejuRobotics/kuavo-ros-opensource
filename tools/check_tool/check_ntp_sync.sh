#!/bin/bash

# 检查是否以 root 用户执行
if [ "$(id -u)" -ne 0 ]; then
    echo "错误: 该脚本需要使用 sudo 执行"
    echo "请使用: sudo $0"
    exit 1
fi

# 根据 ROBOT_VERSION 环境变量选择配置
if [ -z "$ROBOT_VERSION" ]; then
    echo -e "${RED}错误: 未设置 ROBOT_VERSION 环境变量${RESET}"
    echo "请设置环境变量后再运行脚本，例如:"
    echo "  export ROBOT_VERSION=62"
    echo "  或"
    echo "  export ROBOT_VERSION=63"
    exit 1
elif [ "$ROBOT_VERSION" = "62" ]; then
    SSH_USER="ucore"
    SSH_PASS="133233"
    # ROBOT_VERSION=62 需要额外的SSH加密算法配置
    SSH_COMMON_OPTS="-o StrictHostKeyChecking=no \
        -oKexAlgorithms=+diffie-hellman-group14-sha1 \
        -oHostKeyAlgorithms=+ssh-rsa \
        -oCiphers=+aes128-cbc,3des-cbc"
elif [ "$ROBOT_VERSION" = "63" ]; then
    SSH_USER="jt"
    SSH_PASS="lab123"
    SSH_COMMON_OPTS="-o StrictHostKeyChecking=no"
else
    echo -e "${RED}错误: 未知的 ROBOT_VERSION: $ROBOT_VERSION${RESET}"
    echo "支持的版本: 62, 63"
    exit 1
fi

echo "========================================"
echo -e "当前 ROBOT_VERSION: ${YELLOW}$ROBOT_VERSION${RESET}"
echo -e "使用配置: SSH_USER=${SSH_USER}, SSH_PASS=******"
echo "========================================"

# SSH连接信息
SSH_HOST="192.168.26.22"
TIMESYNCD_CONF="/etc/systemd/timesyncd.conf"

# 颜色定义
RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
BLUE='\033[34m'
RESET='\033[0m'

# 检测本机架构，按架构选择 NTP 服务器地址
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    TARGET_NTP="192.168.26.1"
elif [ "$ARCH" = "x86_64" ] || [ "$ARCH" = "amd64" ]; then
    TARGET_NTP="192.168.26.12"
else
    echo -e "${RED}错误: 不支持的本机架构: $ARCH${RESET}"
    echo "支持的架构: x86_64/amd64, aarch64/arm64"
    exit 1
fi

echo ""
echo -e "${BLUE}开始检查底盘时间同步状态...${RESET}"
echo "========================================"

echo ""
echo -e "${BLUE}本机架构: ${YELLOW}$ARCH${RESET}"
echo -e "${BLUE}目标 NTP 服务器: ${YELLOW}$TARGET_NTP${RESET}"

# ========================================
# 第一部分：检查本机（下位机）时间同步状态
# ========================================
echo ""
echo -e "${BLUE}=== 检查本机（下位机）时间同步状态 ===${RESET}"

# 检查本机timedatectl状态
echo ""
echo -e "${BLUE}[本机步骤1] 检查 timedatectl status${RESET}"
LOCAL_STATUS=$(timedatectl status)
echo "$LOCAL_STATUS"

LOCAL_SYNC_STATUS=$(echo "$LOCAL_STATUS" | grep "System clock synchronized" | awk '{print $4}')
echo ""
echo -e "本机 System clock synchronized状态: ${YELLOW}$LOCAL_SYNC_STATUS${RESET}"

if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    # arm64: Ubuntu 22.04 AGX Orin, 检查 chrony allow 配置
    echo ""
    echo -e "${BLUE}[本机步骤2] 检查 chrony allow 配置 (arm64)${RESET}"

    if [ -f /etc/chrony/chrony.conf ]; then
        ALLOW_CONF=$(grep -E "^allow" /etc/chrony/chrony.conf)
        echo "chrony allow 配置："
        echo "$ALLOW_CONF"
        if echo "$ALLOW_CONF" | grep -q "allow"; then
            echo -e "${GREEN}✓ chrony.conf 已配置 allow 网段${RESET}"
        else
            echo -e "${YELLOW}⚠ chrony.conf 未配置 allow，底盘可能无法同步${RESET}"
        fi
    else
        echo -e "${RED}✗ /etc/chrony/chrony.conf 不存在${RESET}"
    fi
else
    # x86_64: 原方案, 使用 chrony
    echo ""
    echo -e "${BLUE}[本机步骤2] 检查 /etc/chrony/chrony.conf 配置 (x86_64)${RESET}"
    if [ -f /etc/chrony/chrony.conf ]; then
        CHRONY_CONF=$(cat /etc/chrony/chrony.conf | grep -E "^server.*$TARGET_NTP" | head -1)
        echo "当前配置: $CHRONY_CONF"

        if echo "$CHRONY_CONF" | grep -q "server.*$TARGET_NTP"; then
            echo -e "${GREEN}✓ 本机 chrony.conf 配置正确 (server $TARGET_NTP)${RESET}"
        else
            echo -e "${RED}✗ 本机 chrony.conf 配置不正确，需要修复${RESET}"
        fi
    else
        echo -e "${RED}✗ /etc/chrony/chrony.conf 文件不存在${RESET}"
    fi
fi

echo ""
echo -e "${YELLOW}本机检查完成！${RESET}"

# ========================================
# 第二部分：检查底盘时间同步状态
# ========================================
echo ""
echo -e "${BLUE}=== 检查底盘（192.168.26.22）时间同步状态 ===${RESET}"

# 清理旧的 SSH host key，避免因主机密钥变更导致连接失败
echo ""
echo "[底盘步骤0] 清理旧的 SSH host key..."
ssh-keygen -R 192.168.26.22 2>/dev/null
ssh-keygen -f '/root/.ssh/known_hosts' -R '192.168.26.22' 2>/dev/null
echo -e "${GREEN}✓ SSH host key 已清理${RESET}"

# 检查网络连通性（ping测试）
echo ""
echo "[底盘步骤1] 检查底盘网络连通性..."
if ! ping -c 1 -W 5 "$SSH_HOST" &> /dev/null; then
    echo -e "${RED}✗ 无法ping通底盘 $SSH_HOST，网络不通${RESET}"
    echo -e "${RED}错误: 底盘连接失败，请检查网络连接${RESET}"
    exit 1
fi
echo -e "${GREEN}✓ 底盘 $SSH_HOST 网络连通正常${RESET}"

# 检查SSH是否可用
echo ""
echo "[底盘步骤2] 检查SSH连接..."
if ! command -v sshpass &> /dev/null; then
    echo -e "${YELLOW}警告: 未找到sshpass命令，正在自动安装...${RESET}"
    sudo apt-get update && sudo apt-get install -y sshpass
    if [ $? -ne 0 ]; then
        echo -e "${RED}错误: sshpass安装失败${RESET}"
        exit 1
    fi
    echo -e "${GREEN}✓ sshpass安装成功${RESET}"
fi

# 执行timedatectl status检查
echo ""
echo "[底盘步骤2] 执行timedatectl status检查..."
STATUS_OUTPUT=$(sshpass -p "$SSH_PASS" ssh $SSH_COMMON_OPTS "$SSH_USER@$SSH_HOST" "timedatectl status")
echo "$STATUS_OUTPUT"

# 检查System clock synchronized状态
SYNC_STATUS=$(echo "$STATUS_OUTPUT" | grep "System clock synchronized" | awk '{print $4}')
echo ""
echo -e "底盘 System clock synchronized状态: ${YELLOW}$SYNC_STATUS${RESET}"

if [ "$SYNC_STATUS" = "yes" ]; then
    echo ""
    echo -e "${GREEN}✓ 底盘时间同步状态已为yes，检查配置文件...${RESET}"

    # 检查配置文件
    CONF_CONTENT=$(sshpass -p "$SSH_PASS" ssh $SSH_COMMON_OPTS "$SSH_USER@$SSH_HOST" "cat $TIMESYNCD_CONF")
    echo ""
    echo "当前配置文件内容:"
    echo "$CONF_CONTENT"

    # 检查NTP配置
    NTP_CONFIG=$(echo "$CONF_CONTENT" | grep -E "^NTP=" | cut -d'=' -f2)

    if [ "$NTP_CONFIG" = "$TARGET_NTP" ]; then
        echo ""
        echo -e "${GREEN}✓ 配置文件NTP=$TARGET_NTP正确${RESET}"
        echo -e "${GREEN}✓ 底盘时间同步状态已为yes${RESET}"
        exit 0
    else
        echo ""
        echo -e "${YELLOW}⚠ 配置文件NTP配置不正确，需要修复${RESET}"
    fi
else
    echo ""
    echo -e "${RED}✗ 底盘时间同步状态为no，需要修复${RESET}"
fi

# 创建临时配置文件
TMP_CONF=$(mktemp)
cat > "$TMP_CONF" << EOF
[Time]
NTP=$TARGET_NTP
#FallbackNTP=ntp.ubuntu.com
#RootDistanceMaxSec=5
#PollIntervalMinSec=32
#PollIntervalMaxSec=2048
EOF

# 需要修复配置
echo ""
echo "[底盘步骤3] 修复timesyncd配置..."

# 备份原始配置文件（带时间戳）
echo -e "${YELLOW}正在备份原始配置文件...${RESET}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
sshpass -p "$SSH_PASS" ssh $SSH_COMMON_OPTS -tt "$SSH_USER@$SSH_HOST" "echo '$SSH_PASS' | sudo -S cp $TIMESYNCD_CONF ${TIMESYNCD_CONF}.bak.${TIMESTAMP}; exit"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 配置文件备份成功${RESET}"
else
    echo -e "${RED}✗ 配置文件备份失败${RESET}"
    rm -f "$TMP_CONF"
    exit 1
fi

# 使用sshpass和scp上传配置文件到远程临时目录
sshpass -p "$SSH_PASS" scp $SSH_COMMON_OPTS "$TMP_CONF" "$SSH_USER@$SSH_HOST:/tmp/timesyncd.conf.tmp"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 配置文件上传成功${RESET}"
else
    echo -e "${RED}✗ 配置文件上传失败${RESET}"
    rm -f "$TMP_CONF"
    exit 1
fi

# 使用ssh执行sudo命令复制文件
sshpass -p "$SSH_PASS" ssh $SSH_COMMON_OPTS -tt "$SSH_USER@$SSH_HOST" "echo '$SSH_PASS' | sudo -S cp /tmp/timesyncd.conf.tmp $TIMESYNCD_CONF; exit"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 配置文件写入成功${RESET}"
else
    echo -e "${RED}✗ 配置文件写入失败${RESET}"
    rm -f "$TMP_CONF"
    exit 1
fi

# 清理临时文件
rm -f "$TMP_CONF"

# 重启时间同步服务
echo ""
echo "[底盘步骤4] 重启systemd-timesyncd服务..."
sshpass -p "$SSH_PASS" ssh $SSH_COMMON_OPTS -tt "$SSH_USER@$SSH_HOST" "echo '$SSH_PASS' | sudo -S systemctl restart systemd-timesyncd; exit"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 服务重启成功${RESET}"
else
    echo -e "${RED}✗ 服务重启失败${RESET}"
    exit 1
fi

# 延时1秒
echo ""
echo "[底盘步骤5] 等待1秒..."
sleep 1

# 再次检查状态
echo ""
echo "[底盘步骤6] 再次验证时间同步状态..."
STATUS_OUTPUT=$(sshpass -p "$SSH_PASS" ssh $SSH_COMMON_OPTS "$SSH_USER@$SSH_HOST" "timedatectl status")
echo "$STATUS_OUTPUT"

SYNC_STATUS=$(echo "$STATUS_OUTPUT" | grep "System clock synchronized" | awk '{print $4}')

echo ""
echo "========================================"
if [ "$SYNC_STATUS" = "yes" ]; then
    echo -e "${GREEN}✓ 成功！底盘 System clock synchronized状态已变为yes${RESET}"
    echo "========================================"
    exit 0
else
    echo -e "${RED}✗ 失败！底盘 System clock synchronized状态仍然为$SYNC_STATUS${RESET}"
    echo "========================================"
    exit 1
fi
