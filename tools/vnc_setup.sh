#!/bin/bash

# 确保脚本以root权限运行
if [ "$(id -u)" -ne 0 ]; then
    echo "请使用sudo或root权限运行此脚本" >&2
    exit 1
fi

# 定义变量
USER="leju_kuavo"
GROUP="leju_kuavo"
DISPLAY_NUM="2"
RESOLUTION="1920x1080"
SERVICE_FILE="/etc/systemd/system/vncserver@.service"
VNC_PASSWORD="leju_kuavo"  # 固定VNC密码

# 更新软件源
echo "=== 更新软件源 ==="
apt update -y

# 安装VNC相关软件
echo "=== 安装VNC服务器 ==="
apt install -y tigervnc-common tigervnc-standalone-server tightvncserver

# 安装桌面环境
echo "=== 安装XFCE4桌面环境 ==="
apt install -y xfce4 xfce4-goodies

# 配置VNC密码（非交互式）
echo "=== 配置VNC密码 ==="
su - $USER -c "echo -e '$VNC_PASSWORD\n$VNC_PASSWORD\nn' | vncpasswd"

# 配置xstartup文件
echo "=== 配置xstartup文件 ==="
VNC_DIR="/home/$USER/.vnc"
XSTARTUP_FILE="$VNC_DIR/xstartup"

# 创建.vnc目录（如果不存在）
su - $USER -c "mkdir -p $VNC_DIR"

# 写入xstartup配置
su - $USER -c "cat > $XSTARTUP_FILE << EOF
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
startxfce4 &

[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup
[ -r \$HOME/.Xresources ] && xrdb \$HOME/.Xresources
xsetroot -solid grey
EOF"

# 设置xstartup权限
su - $USER -c "chmod +x $XSTARTUP_FILE"

# 创建systemd服务文件
echo "=== 创建系统服务 ==="
cat > $SERVICE_FILE << EOF
[Unit]
Description=Remote desktop (VNC)
After=syslog.target network.target

[Service]
Type=forking
User=$USER
Group=$GROUP

PIDFile=/home/$USER/.vnc/ubuntu:$DISPLAY_NUM.pid
ExecStart=/usr/bin/vncserver -geometry $RESOLUTION :$DISPLAY_NUM -localhost no
ExecStop=/usr/bin/vncserver -kill :$DISPLAY_NUM

[Install]
WantedBy=multi-user.target
EOF

# 重新加载systemd配置
echo "=== 配置服务自动启动 ==="
systemctl daemon-reload
systemctl enable vncserver@$DISPLAY_NUM.service

# 清理可能的残留进程
su - $USER -c "vncserver -kill :$DISPLAY_NUM > /dev/null 2>&1"

# 启动服务
echo "=== 启动VNC服务 ==="
systemctl start vncserver@$DISPLAY_NUM.service

# 显示状态信息
echo "=== 配置完成 ==="
systemctl status vncserver@$DISPLAY_NUM.service --no-pager

echo "VNC服务已配置，端口号: $((5900 + DISPLAY_NUM))"
echo "连接信息："
echo "  用户名: $USER"
echo "  VNC密码: $VNC_PASSWORD"
echo "  VNC端口: $((5900 + DISPLAY_NUM))"
echo "  分辨率: $RESOLUTION"