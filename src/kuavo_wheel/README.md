# Kuavo Wheel Bridge

这个包提供了kuavo系统和轮式小车系统之间的ROS话题桥接功能。

## 话题映射

### Kuavo系统 → Wheel系统
- `/cmd_vel` (Twist) → `/base_cmd_vel` (Twist)

### Wheel系统 → Kuavo系统  
- `/odom` (Odometry) → `/odom` (Odometry)

## 使用方法

### 基本使用（使用默认参数）
```bash
cd <kuavo-ros-control>
python3 src/kuavo_wheel/scripts/wheel_bridge.py
```

### 使用自定义参数
```bash
cd <kuavo-ros-control>
python3 src/kuavo_wheel/scripts/wheel_bridge.py \
    --kuavo-master http://169.254.128.130:11311 \
    --wheel-master http://169.254.128.2:11311 \
    --kuavo-slave-ip 169.254.128.130 \
    --wheel-slave-ip 169.254.128.130
```

### 查看帮助信息
```bash
python3 src/kuavo_wheel/scripts/wheel_bridge.py --help
```

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--kuavo-master` | `http://169.254.128.130:11311` | Kuavo系统的ROS Master URI |
| `--wheel-master` | `http://169.254.128.2:11311` | Wheel系统的ROS Master URI |
| `--kuavo-slave-ip` | `169.254.128.130` | Kuavo从机的IP地址 |
| `--wheel-slave-ip` | `169.254.128.130` | Wheel从机的IP地址 |

## 环境变量配置

服务启动时会自动从环境变量获取kuavo相关配置：

### 环境变量优先级
1. **KUAVO_MASTER_URI** - kuavo master URI（最高优先级）
2. **KUAVO_SLAVE_IP** - kuavo slave IP地址
3. **ROS_IP** - 如果KUAVO_SLAVE_IP未设置，使用ROS_IP
4. **默认值** - 如果以上都未设置，使用硬编码默认值

### 环境变量设置示例
```bash
# 设置kuavo master URI
export KUAVO_MASTER_URI="http://169.254.128.130:11311"

# 设置kuavo slave IP
export KUAVO_SLAVE_IP="169.254.128.130"

# 或者使用ROS_IP（会被用作kuavo slave IP的备选）
export ROS_IP="169.254.128.130"
```

### 服务中的环境变量
服务启动时会自动设置以下环境变量：
- `KUAVO_MASTER_URI` - kuavo master URI
- `KUAVO_SLAVE_IP` - kuavo slave IP
- `ROS_MASTER_URI` - ROS master URI
- `ROS_IP` - ROS IP地址

## 系统服务安装

### 安装为系统服务（开机自启）
```bash
cd <kuavo-ros-application>
sudo src/kuavo_wheel/scripts/deploy_autostart_vehicle_bridge.sh
```

### 使用自定义参数安装服务
```bash
cd <kuavo-ros-application>
sudo src/kuavo_wheel/scripts/deploy_autostart_vehicle_bridge.sh \
    --kuavo-master=http://192.168.26.1:11311 \
    --wheel-master=http://169.254.128.2:11311 \
    --kuavo-slave-ip=192.168.26.12 \
    --wheel-slave-ip=169.254.128.109
```

### 非交互式安装（CI模式）
```bash
cd <kuavo-ros-application>
sudo src/kuavo_wheel/scripts/deploy_autostart_vehicle_bridge.sh -ci
```

### 服务管理命令
```bash
# 启动服务
sudo systemctl start kuavo_wheel_bridge

# 停止服务
sudo systemctl stop kuavo_wheel_bridge

# 重启服务
sudo systemctl restart kuavo_wheel_bridge

# 查看服务状态
sudo systemctl status kuavo_wheel_bridge

# 查看服务日志
sudo journalctl -u kuavo_wheel_bridge -f

# 禁用开机自启
sudo systemctl disable kuavo_wheel_bridge
```

### 卸载服务
```bash
# 停止并禁用服务
sudo systemctl stop kuavo_wheel_bridge
sudo systemctl disable kuavo_wheel_bridge

# 删除服务文件
sudo rm /etc/systemd/system/kuavo_wheel_bridge.service

# 重新加载systemd配置
sudo systemctl daemon-reload
```

## 自动网络检测

服务启动时会自动检测wheel网络配置：

1. **自动检测wheel网段**：扫描所有网络接口，查找169.254.x.x网段的IP地址
2. **自动设置wheel slave IP**：使用检测到的wheel网段IP作为当前机器的wheel slave IP

### 检测逻辑
- 扫描所有网络接口（除lo外）
- 查找169.254.x.x网段的IP地址
- 自动设置wheel slave IP为检测到的IP

### 手动覆盖
如果自动检测不符合需求，可以通过参数手动指定：
```bash
sudo src/kuavo_wheel/scripts/deploy_autostart_vehicle_bridge.sh \
    --wheel-master=http://169.254.100.2:11311 \
    --wheel-slave-ip=169.254.100.109
```

## 文件结构

```
src/kuavo_wheel/
├── scripts/
│   ├── wheel_bridge.py                     # 主程序
│   ├── start_wheel_bridge.sh               # 启动脚本（自动检测网络和环境变量）
│   └── deploy_autostart_vehicle_bridge.sh  # 部署脚本（支持参数传递）
├── service/
│   └── kuavo_wheel_bridge.service          # 服务配置文件
└── README.md
```
