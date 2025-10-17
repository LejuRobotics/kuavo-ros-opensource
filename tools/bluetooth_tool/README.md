# 蓝牙模块安装脚本

本目录包含用于构建、配置和管理蓝牙模块的脚本工具。

## 📋 脚本说明

- **build_btusb.sh**: 自动化编译、安装及配置蓝牙核心模块，包含环境准备、模块编译、系统安装、开机配置等完整流程
- **enable_vnc_bluetooth_config.sh**: 配置用户蓝牙权限和服务重启，添加用户到bluetooth组并设置Polkit策略
- **kuavo-bt-service/**: 蓝牙自动连接服务，确保机器人启动时已配对的蓝牙设备能够自动重新连接

## 🚀 使用流程

### 1. 构建蓝牙模块

```bash
# 1. 构建并安装蓝牙模块
sudo ./build_btusb.sh

# 2. 配置用户权限
sudo ./enable_vnc_bluetooth_config.sh

# 3. [可选]安装蓝牙管理工具
sudo apt update
sudo apt install blueman

```

> **💡 blueman 工具说明**:
- blueman 是一个功能完整的蓝牙管理器，提供图形化界面
- 支持设备扫描、配对、连接、文件传输等功能
- 提供系统托盘图标，方便快速管理蓝牙设备
- 支持音频设备、输入设备、网络等多种蓝牙协议

> **⚠️ 蓝牙自动连接服务重要提示**:
- 在运行安装脚本之前，请确保已手动连接并配对您的蓝牙设备
- 安装脚本会扫描当前已连接的设备并自动配置到服务中
- 服务将在系统启动时自动连接已配对的设备

### 2. 验证蓝牙功能

```bash
# 检查蓝牙服务状态
systemctl status bluetooth

# 列出蓝牙适配器
bluetoothctl list

# 检查蓝牙控制器
hciconfig -a

## 示例输出
hciconfig -a
hci1:   Type: Primary  Bus: USB
        BD Address: 80:C8:AC:00:02:AC  ACL MTU: 310:10  SCO MTU: 64:8
        UP RUNNING PSCAN 
        RX bytes:79048 acl:88 sco:0 events:6372 errors:0
        TX bytes:2780032 acl:12598 sco:0 commands:150 errors:0
        Features: 0xff 0xff 0x8f 0xfe 0xdb 0xff 0x5b 0x87
        Packet type: DM1 DM3 DM5 DH1 DH3 DH5 HV1 HV2 HV3 
        Link policy: RSWITCH HOLD SNIFF PARK 
        Link mode: SLAVE ACCEPT 
        Name: 'P4-185-NUC'
        Class: 0x0c0104
        Service Classes: Rendering, Capturing
        Device Class: Computer, Desktop workstation
        HCI Version: 4.0 (0x6)  Revision: 0x22bb
        LMP Version: 4.0 (0x6)  Subversion: 0x22bb
        Manufacturer: Cambridge Silicon Radio (10)

hci0:   Type: Primary  Bus: USB
        BD Address: 64:79:F0:22:7D:DC  ACL MTU: 1021:4  SCO MTU: 96:6
        UP RUNNING PSCAN 
        RX bytes:131235 acl:41 sco:0 events:5487 errors:0
        TX bytes:820166 acl:148 sco:0 commands:3319 errors:0
        Features: 0xbf 0xfe 0x0f 0xfe 0xdb 0xff 0x7b 0x87
        Packet type: DM1 DM3 DM5 DH1 DH3 DH5 HV1 HV2 HV3 
        Link policy: RSWITCH SNIFF 
        Link mode: SLAVE ACCEPT 
        Name: 'P4-185'
        Class: 0x0c0104
        Service Classes: Rendering, Capturing
        Device Class: Computer, Desktop workstation
        HCI Version:  (0xb)  Revision: 0x3362
        LMP Version:  (0xb)  Subversion: 0x3362
        Manufacturer: Intel Corp. (2)
```

**可以看到`hci0`是NUC自带的Intel蓝牙模块，`hci1`是我们外接的USB蓝牙模块！可以通过此来区分他们的地址和名称！**

## 🔗 如何连接蓝牙设备

连接指南请参考：**[CONNECTION_GUIDE.md](./CONNECTION_GUIDE.md)**

## 🤖 蓝牙自动连接服务管理

详细文档请参考：**[BLUETOOTH_SERVICE.md](./kuavo-bt-service/BLUETOOTH_SERVICE.md)**

## 🔧 故障排除

### 常见问题

1. **权限错误**: 确保使用sudo运行脚本
2. **内核源码缺失**: 检查`/home/lab/kenel/linux-5.15.158`目录是否存在
3. **模块加载失败**: 使用`dmesg | grep bluetooth`查看内核日志
4. **蓝牙服务连接超时**: 出现 "connect error: Connection timed out" 或 "a2dp-sink profile connect failed" 错误

### 蓝牙服务连接问题处理

当出现以下错误时，通常表示蓝牙服务出现了连接问题：

**蓝牙服务状态示例**:
```
systemctl status bluetooth.service
● bluetooth.service - Bluetooth service
     Loaded: loaded (/lib/systemd/system/bluetooth.service; enabled; vendor preset: enabled)
     Active: active (running) since Sat 2025-09-06 17:39:40 CST; 1min 5s ago
       Docs: man:bluetoothd(8)
   Main PID: 772 (bluetoothd)
     Status: "Running"
      Tasks: 1 (limit: 76813)
     CGroup: /system.slice/bluetooth.service
             └─772 /usr/lib/bluetooth/bluetoothd

Sep 06 17:40:13 NUC11TNKi7 bluetoothd[772]: Endpoint registered: sender=:1.168 path=/MediaEndpoint/A2DPSource/sbc
Sep 06 17:40:22 NUC11TNKi7 bluetoothd[772]: Endpoint unregistered: sender=:1.168 path=/MediaEndpoint/A2DPSink/sbc
Sep 06 17:40:22 NUC11TNKi7 bluetoothd[772]: Endpoint unregistered: sender=:1.168 path=/MediaEndpoint/A2DPSource/sbc
Sep 06 17:40:22 NUC11TNKi7 bluetoothd[772]: Endpoint unregistered: sender=:1.168 path=/MediaEndpoint/A2DPSink/sbc
Sep 06 17:40:22 NUC11TNKi7 bluetoothd[772]: Endpoint unregistered: sender=:1.168 path=/MediaEndpoint/A2DPSource/sbc
Sep 06 17:40:34 NUC11TNKi7 bluetoothd[772]: connect error: Connection timed out (110)
Sep 06 17:40:34 NUC11TNKi7 bluetoothd[772]: a2dp-sink profile connect failed for 41:42:52:94:E8:5F: Protocol not available
Sep 06 17:40:36 NUC11TNKi7 bluetoothd[772]: a2dp-sink profile connect failed for 41:42:52:94:E8:5F: Protocol not available
Sep 06 17:40:38 NUC11TNKi7 bluetoothd[772]: a2dp-sink profile connect failed for 41:42:52:94:E8:5F: Protocol not available
Sep 06 17:40:42 NUC11TNKi7 bluetoothd[772]: a2dp-sink profile connect failed for 41:42:52:94:E8:5F: Protocol not available
```

**关键错误信息**:
```
connect error: Connection timed out (110)
a2dp-sink profile connect failed for 41:42:52:94:E8:5F: Protocol not available
```

**解决方案**:

```bash
# 1. 检查蓝牙服务状态
systemctl status bluetooth.service

# 2. 重启蓝牙服务即可解决问题
sudo systemctl restart bluetooth

# 3. 重新加载PulseAudio蓝牙发现模块
pactl load-module module-bluetooth-discover

# 4. 在重启服务之前，可以尝试重启PulseAudio服务
pulseaudio -k
# 等一会儿
systemctl --user restart pulseaudio.service
pactl load-module module-bluetooth-discover

# 5. 验证服务已恢复正常
systemctl status bluetooth
```

**预防措施**:
- 定期检查蓝牙服务状态
- 避免频繁插拔蓝牙适配器
- 确保系统资源充足，避免内存不足导致服务异常

### 日志查看

```bash
# 查看蓝牙服务日志
journalctl -u bluetooth -f

# 查看内核蓝牙相关日志
dmesg | grep -i bluetooth

# 查看已加载的蓝牙模块
lsmod | grep -E 'bluetooth|btusb|rfcomm'

# 检查USB音频模块
lsmod | grep snd_usb_audio

## ⚠️ 注意事项

- 构建脚本需要较长时间，请耐心等待
- 建议在运行前备份重要数据
- 如需修改内核源码路径，请编辑`build_btusb.sh`中的`KERNEL_DIR`变量