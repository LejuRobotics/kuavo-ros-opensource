# 蓝牙自动连接服务

本服务用于自动连接已配对的蓝牙设备，确保机器人启动时蓝牙设备能够自动重新连接。

## 安装步骤

### 1. 运行安装脚本

> **⚠️ 重要提示：在运行安装脚本之前，请确保已手动连接并配对您的蓝牙设备！**  
> **⚠️ 重要提示：在运行安装脚本之前，请确保已手动连接并配对您的蓝牙设备！**  
> **⚠️ 重要提示：在运行安装脚本之前，请确保已手动连接并配对您的蓝牙设备！**  
> **安装脚本会扫描当前已连接的设备并自动配置到服务中。**

```bash
sudo ./tools/bluetooth_tool/kuavo-bt-service/install.sh

### 示例输出
[INFO] 开始安装蓝牙自动连接服务
[INFO] 停止现有服务
[INFO] 禁用现有服务
[INFO] 清理旧文件
[INFO] 备份配置文件
[INFO] 检查蓝牙控制器

=== 蓝牙控制器信息 ===
--------------------------------
hci1:   Type: Primary  Bus: USB
        BD Address: 64:79:F0:22:7D:DC  ACL MTU: 1021:4  SCO MTU: 96:6
        UP RUNNING PSCAN 
        RX bytes:19286 acl:0 sco:0 events:3054 errors:0
        TX bytes:746264 acl:0 sco:0 commands:3034 errors:0
        Features: 0xbf 0xfe 0x0f 0xfe 0xdb 0xff 0x7b 0x87
        Packet type: DM1 DM3 DM5 DH1 DH3 DH5 HV1 HV2 HV3 
        Link policy: RSWITCH SNIFF 
        Link mode: SLAVE ACCEPT 
        Name: 'P4-185-NUC-BT'
        Class: 0x0c0104
        Service Classes: Rendering, Capturing
        Device Class: Computer, Desktop workstation
        HCI Version:  (0xb)  Revision: 0x3362
        LMP Version:  (0xb)  Subversion: 0x3362
        Manufacturer: Intel Corp. (2)

hci0:   Type: Primary  Bus: USB
        BD Address: 80:C8:AC:00:03:23  ACL MTU: 310:10  SCO MTU: 64:8
        UP RUNNING 
        RX bytes:9489 acl:34 sco:0 events:752 errors:0
        TX bytes:361789 acl:1625 sco:0 commands:74 errors:0
        Features: 0xff 0xff 0x8f 0xfe 0xdb 0xff 0x5b 0x87
        Packet type: DM1 DM3 DM5 DH1 DH3 DH5 HV1 HV2 HV3 
        Link policy: RSWITCH HOLD SNIFF PARK 
        Link mode: SLAVE ACCEPT 
        Name: 'NUC11TNKi7'
        Class: 0x0c0104
        Service Classes: Rendering, Capturing
        Device Class: Computer, Desktop workstation
        HCI Version: 4.0 (0x6)  Revision: 0x22bb
        LMP Version: 4.0 (0x6)  Subversion: 0x22bb
        Manufacturer: Cambridge Silicon Radio (10)


=== 当前默认控制器 ===
--------------------------------
Controller 80:C8:AC:00:03:23 NUC11TNKi7   [default]
Controller 64:79:F0:22:7D:DC P4-185-NUC-BT

默认控制器是否正确? [Y/n]: y
✓ 使用当前默认控制器
[INFO] 检查已连接的蓝牙设备

=== 找到已连接的设备 ===
  • Shinco (41:42:52:94:E8:5F)
    状态: 配对=yes, 信任=yes, 连接=yes

注意: 服务将自动连接这些设备
```
### 2. 验证安装

```bash
# 检查服务状态
systemctl status kuavo-bt-service

# 查看服务日志
journalctl -u kuavo-bt-service -f
```

### 查看服务状态

```bash
# 查看服务状态
sudo systemctl status kuavo-bt-service

# 查看实时日志
sudo journalctl -u kuavo-bt-service -f

# 查看最近的日志
sudo journalctl -u kuavo-bt-service --since "1 hour ago"
```

## 手动测试蓝牙连接

### 使用设备扫描脚本

```bash
# 扫描已连接设备
python3 /opt/lejurobot/kuavo-bt-service/bluetooth_device_scanner.py
```

> **🔧 如果服务没有检测到您的设备，请先手动连接并配对设备，然后重新运行安装脚本。**