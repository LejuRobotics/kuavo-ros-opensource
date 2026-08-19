#!/bin/bash

# 颜色定义
RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
BLUE='\033[34m'
RESET='\033[0m'

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

# 等待底盘能连通 NTP 服务器（AGX 上的 chrony）的 UDP 123 端口。
# 探测的是「底盘 → NTP 服务器」的连通性，而非底盘自身 timesyncd 的监听状态。
# 注意：nc -z -u 的 UDP 探测依赖「未收到 ICMP port unreachable」这个负信号，
# 在「服务器整机未开机/网络断」时会误报成功，因此以 ping（主机在线）为主判断，
# nc 端口探测仅作补充、不参与成败判定（用 || true 兜底）。
wait_ntp_server_ready() {
    echo ""
    echo "[底盘步骤3.5] 等待底盘能连通 NTP 服务器 $TARGET_NTP 的 123 端口 ..."
    local attempt=1
    local max_attempts=30
    local probe="ping -c 1 -W 2 $TARGET_NTP >/dev/null 2>&1 && { command -v nc >/dev/null 2>&1 && nc -z -u -w 2 $TARGET_NTP 123 || true; }"
    while [ $attempt -le $max_attempts ]; do
        if sshpass -p "$SSH_PASS" ssh $SSH_COMMON_OPTS "$SSH_USER@$SSH_HOST" "$probe >/dev/null 2>&1"; then
            echo -e "${GREEN}✓ 底盘已能连通 $TARGET_NTP 的 123 端口（第 ${attempt} 次尝试）${RESET}"
            return 0
        fi
        echo -e "${YELLOW}  第 ${attempt} 次尝试探测 $TARGET_NTP:123 失败，2 秒后重试...${RESET}"
        sleep 2
        attempt=$((attempt + 1))
    done
    echo -e "${RED}✗ 底盘在约 $((max_attempts * 4)) 秒内仍无法连通 $TARGET_NTP 的 123 端口${RESET}"
    return 1
}

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
    echo -e "${GREEN}✓ 底盘时间同步状态已为yes${RESET}"
else
    echo ""
    echo -e "${RED}✗ 底盘时间同步状态为no，需要修复${RESET}"
fi

# 读取底盘当前配置文件，判断 NTP 配置是否已正确
CONF_CONTENT=$(sshpass -p "$SSH_PASS" ssh $SSH_COMMON_OPTS "$SSH_USER@$SSH_HOST" "cat $TIMESYNCD_CONF")
echo ""
echo "当前配置文件内容:"
echo "$CONF_CONTENT"

NTP_CONFIG=$(echo "$CONF_CONTENT" | grep -E "^NTP=" | cut -d'=' -f2)

if [ "$NTP_CONFIG" = "$TARGET_NTP" ]; then
    echo ""
    echo -e "${GREEN}✓ 配置文件NTP=$TARGET_NTP正确，跳过重写${RESET}"
    NEED_CONFIG_FIX=0
else
    echo ""
    echo -e "${YELLOW}⚠ 配置文件NTP配置不正确（当前: $NTP_CONFIG），需要修复${RESET}"
    NEED_CONFIG_FIX=1
fi

# 仅在配置确实不正确时才重写，避免覆盖 NTP= 之外的手工设置并堆积 .bak 备份文件
if [ "$NEED_CONFIG_FIX" -eq 1 ]; then
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
fi

# 等待底盘能连通 NTP 服务器（AGX 已就绪）再触发同步
wait_ntp_server_ready || exit 1

# 启用并重启时间同步服务：enable 设置开机自启，restart 确保立即重新发起同步。
# （enable --now 对已在运行的 active 单元是 no-op，无法修复「启动过早」导致的同步失败）
echo ""
echo "[底盘步骤4] 启用并重启systemd-timesyncd服务（含开机自启）..."
sshpass -p "$SSH_PASS" ssh $SSH_COMMON_OPTS -tt "$SSH_USER@$SSH_HOST" "echo '$SSH_PASS' | sudo -S sh -c 'systemctl enable systemd-timesyncd && systemctl restart systemd-timesyncd'; exit"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 服务已启用并重启（含开机自启）${RESET}"
else
    echo -e "${RED}✗ 服务启用/重启失败${RESET}"
    exit 1
fi

# 延时3秒，等待首次时间同步完成
echo ""
echo "[底盘步骤5] 等待3秒..."
sleep 3

# 再次检查状态（带重试：timesyncd 重启后首次查询存在网络往返延迟，单次判定易误报失败）
echo ""
echo "[底盘步骤6] 再次验证时间同步状态..."
SYNC_STATUS=""
VERIFY_ATTEMPTS=3
VERIFY_INTERVAL=3
for ((i=1; i<=VERIFY_ATTEMPTS; i++)); do
    STATUS_OUTPUT=$(sshpass -p "$SSH_PASS" ssh $SSH_COMMON_OPTS "$SSH_USER@$SSH_HOST" "timedatectl status")
    echo "$STATUS_OUTPUT"
    SYNC_STATUS=$(echo "$STATUS_OUTPUT" | grep "System clock synchronized" | awk '{print $4}')
    if [ "$SYNC_STATUS" = "yes" ]; then
        break
    fi
    if [ "$i" -lt "$VERIFY_ATTEMPTS" ]; then
        echo -e "${YELLOW}  第 ${i} 次校验仍为 $SYNC_STATUS，${VERIFY_INTERVAL} 秒后重试...${RESET}"
        sleep $VERIFY_INTERVAL
    fi
done

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
