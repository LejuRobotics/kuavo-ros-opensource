# H12日志串口功能使用说明

## 概述

H12日志串口功能允许用户通过遥控器查看机器人的实时日志输出。该功能通过 USB 转串口设备连接到 Multi link V1.0 模块，实现日志数据的无线传输。

## 硬件连接

### 所需硬件
- **Multi link V1.0模块**：2.4GHz无线通信模块
- **USB转串口转换器**：用于连接下位机和 Multi link 模块
- **连接线**：用于连接两个模块

### 硬件连接示意图

![H12日志串口硬件连接图](./images/H12_log_tty.jpg)

## 部署配置

### 特别注意
- 如果机器人有改装，即 sbus 和 log 的接口都连接，选择第一种方法：自动部署脚本。
- 如果机器人没有接线，选择第二种方法：手动加载串口规则。
  - 需要使用两个 H12，原本的 H12 用于启动机器人，外接的 H12 才能查看 log。
  - 第二个 H12 的接收机不可以连接 sbus 针脚，接好线后连到机器人的下位机。
  - 如果操作失误使用了方案一，需要拔掉第二个 H12 接收机，重新运行 H12 部署脚本，选择跳过 log 串口的规则，然后重启机器，再将第二个 H12 接收机连接到机器人上使用。

### 1. 自动部署脚本

使用 `deploy_autostart.sh` 脚本进行一键部署：

```bash
cd src/humanoid-control/h12pro_controller_node/scripts
./deploy_autostart.sh
```

#### 关键配置选项

**串口规则加载：**
- 脚本会询问是否加载遥控器 log 串口udev规则
- 选择'y'：自动加载H12_log_serial.rules规则
- 选择'n'：跳过规则加载


### 2. 手动加载串口规则

如果需要单独加载串口规则，可以使用：

```bash
cd src/humanoid-control/h12pro_controller_node/scripts
sudo ./load_h12_log_serial_rule.sh
```

#### 规则文件说明
udev规则文件 `H12_log_serial.rules` 配置：
- 设备识别：USB 厂商ID 10c4，产品ID ea60 （**后续如果更换串口，请联系开发人员修改**）
- 权限设置：组为 dialout，模式为 0777
- 符号链接：创建 `/dev/H12_log_channel` 设备文件
- 串口配置：波特率 57600，8数据位，无停止位，无奇偶校验


### 3. H12 遥控器 LOG 软件安装

![H12日志软件](./images/H12_log_app.jpg)

#### 下载链接

https://kuavo.lejurobot.com/H12SerialLogApks/kuavo_h12_controller.apk

#### 安装方式
- 用数据线将 H12 遥控器连接到电脑
- 将下载的软件放到 H12 的文件夹
- 在 H12 遥控器的文件管理器找到并安装
- 打开软件，选择 `终端`
- 右上角有 `清除` 和 `暂停`

![H12日志软件](./images/H12_log_use.jpg)