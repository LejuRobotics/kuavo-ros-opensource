# VR有线连接配置说明

## **特别说明**
目前上下位机之间ROS主从通信使用的USB转网口设备是realtak的，如果使用该教程配置VR有线连接方案请购买使用芯片厂商为ASIX的USB转网口设备(且产品id为772b，即idProduct == 772b)！！！否则该教程不会生效！！！


## 概述
本文档介绍如何完成VR有线配置


## 脚本功能说明
1. **setup_udev_rules.sh**  
   负责复制并配置udev规则文件，实现ASIX和Realtek网卡的重命名，并通过重启系统使规则生效。

2. **configure_dhcp.sh**  
   在udev规则生效后，根据重命名后的网卡名称配置DHCP服务，分别为ASIX网卡分配192.168.28.x网段，为Realtek网卡分配192.168.26.x网段。


## 前置条件
1. 确保两个udev规则文件存在于指定路径：  
   - ASIX规则：`/home/lab/kuavo-ros-opensource/tools/adapt_vr_wire/99-usb-net-asix.rules`  
   - Realtek规则：`/home/lab/kuavo-ros-opensource/tools/adapt_vr_wire/99-usb-net-realtek.rules`  
2. 确保已插入两个USB网卡（ASIX和Realtek各一个）。


## 操作步骤

### 步骤1：执行udev规则配置脚本
1. 第一个脚本`setup_udev_rules.sh`赋予执行权限：  
   chmod +x setup_udev_rules.sh

2. 以root权限运行脚本：  
   sudo ./setup_udev_rules.sh

### 步骤2：重启机器人

### 步骤3：执行DHCP配置脚本
1. 第二个脚本`configure_dhcp.sh`赋予执行权限：  
   chmod +x configure_dhcp.sh

2. 以root权限运行脚本：  
   sudo ./configure_dhcp.sh


## 验证配置
执行完成后，可通过以下命令验证配置是否成功：

1. 检查网卡名称是否已按udev规则重命名：  
   ip link show | grep -E 'enxasix|enx00e04c68345c'  # 名称需与udev规则一致

2. 检查网卡IP配置：  
   ip addr show enxasix    # 应显示192.168.28.1/24
   ip addr show enx00e04c68345c # 应显示192.168.26.1/24

3. 检查DHCP服务状态：  
   sudo systemctl status isc-dhcp-server  # 应显示“active (running)”


## 注意事项
1. **两个脚本必须按顺序执行，且第一个脚本执行后需等待系统重启完成再执行第二个脚本。** 
