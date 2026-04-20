#!/bin/bash
# 通过 SSH 连接 ucore 并重启 urobot.service
# 注意：密码以明文存储，生产环境建议使用 SSH 密钥认证
# 依赖：需安装 sshpass (apt install sshpass)
set -e

SSH_HOST="ucore@192.168.26.22"
SSH_PASS="133233"
SSH_OPTS="-oKexAlgorithms=+diffie-hellman-group14-sha1 -oHostKeyAlgorithms=+ssh-rsa -oCiphers=+aes128-cbc,3des-cbc -oStrictHostKeyChecking=no -oConnectTimeout=5"

if ! command -v sshpass &>/dev/null; then
    echo "[restart_urobot_service] 错误: 未安装 sshpass，请执行: sudo apt install sshpass"
    exit 1
fi

echo "[restart_urobot_service] 正在连接 ${SSH_HOST} 并重启 urobot.service ..."
sshpass -p "${SSH_PASS}" ssh ${SSH_OPTS} ${SSH_HOST} "echo ${SSH_PASS} | sudo -S systemctl restart urobot.service"

if [ $? -eq 0 ]; then
    echo "[restart_urobot_service] urobot.service 重启成功"
else
    echo "[restart_urobot_service] 重启失败，请检查网络连接或手动执行"
    exit 1
fi
