# 修改 DHCP 主机服务

本说明文档介绍如何使用 `update_dhcp.sh` 脚本在 Linux 系统上自动配置 USB 声卡、5G 模块、DHCP 服务及 NAT 转发，实现内网设备通过指定网卡共享 5G 网络上网。

## 文档说明

- 本脚本适用于基于 Debian/Ubuntu 的系统。
- 脚本自动完成以下操作：
  1. 配置 USB 声卡（通过 `roban_sound_card.sh` 独立脚本）。
  2. 配置 5G 模块的 udev 规则，将其命名为 `wwan0`。
  3. 自动查找以 `enx0` 开头的 USB 网卡，并将其作为 DHCP 服务的内网网卡。
  4. 安装并配置 `isc-dhcp-server`，为内网分配固定 IP 地址。
  5. 使用 NetworkManager 设置内网网卡静态地址。
  6. 配置 iptables，实现内网到 5G 网口（`wwan0`）的 NAT 转发。
  7. 检查并为其他活动网络接口（如 WiFi）自动配置 NAT 转发规则。

## 前提条件

- 需要以 root 权限运行脚本。
- 系统已联网，能够访问软件源安装必要软件包。
- 目标 USB 网卡名称以 `enx0` 开头，且仅有一个此类网卡。
- 5G 模块为 Quectel RM530N-GL，VendorID 为 `2c7c`，ProductID 为 `0801`。
- USB 声卡为 C-Media CM108 Audio Controller，VendorID 为 `0d8c`，ProductID 为 `013c`。

## 使用方法

1. 进入脚本所在目录：

   ```bash
   cd ~/kuavo-ros-control/tools/change_dhcp_host
   ```

2. 赋予脚本执行权限：

   ```bash
   chmod +x update_dhcp.sh roban_sound_card.sh
   ```

3. 以 root 权限运行脚本：

   ```bash
   sudo ./update_dhcp.sh
   ```
   ***中间会有一次弹框选择，按回车键选择 'yes' 即可***

4. 按照脚本提示操作，等待脚本自动完成所有配置。

5. **重启机器使声卡配置生效**：

   ```bash
   sudo reboot
   ```

## 最终结果

- USB 声卡成为系统唯一声卡（card 0），所有音频程序默认输出指向它。
- 5G 模块插入后自动命名为 `wwan0`，作为外网出口。
- 内网 USB 网卡（如 `enx0xxxx`）被配置为静态 IP `192.168.26.1`，并作为 DHCP 服务的分配网卡。
- 内网设备通过 DHCP 获取 `192.168.26.12` 地址，并通过 5G 网络上网。
- 系统已自动配置 NAT 转发和必要的 iptables 规则，支持多种外部网络接口（如 5G、WiFi）。
- 所有配置在重启后依然生效。

如需自定义网段、IP 分配范围或其他参数，请根据实际需求修改 `update_dhcp.sh` 脚本中的相关变量。

## 结果验证

1. 验证声卡，重启后执行：
   ```bash
   aplay -l
   ```
   应只显示 USB 声卡：
   ```
   card 0: Device_1 [USB PnP Sound Device], device 0: USB Audio [USB Audio]
   ```

2. 播放测试音频：
   ```bash
   sudo su
   cd /home/lab/.config/lejuconfig/music/
   play 1_挥手.wav
   ```

3. 验证 DHCP 服务是否正常：
   - 接入上位机，检查上位机的 IP 是否被分配为 192.168.26.12。

---

# USB 声卡配置脚本 (roban_sound_card.sh)

`roban_sound_card.sh` 是从 `update_dhcp.sh` 中剥离出来的声卡专用配置脚本，也可以独立运行。

## 背景问题

在旧版本配置流程中，声卡配置依赖 `/root/.asoundrc` 文件手动指定 card 编号：

```
pcm.!default {
  type hw
  card 1    ← 需要手动填写 USB 声卡的 card 编号
}
```

这种方式存在以下问题：

| 问题 | 说明 |
|---|---|
| **card 编号不固定** | USB 声卡在不同机器上可能是 card 0、card 1 或 card 2，取决于设备枚举顺序 |
| **需要手动查看** | 每台机器都需要先运行 `aplay -l` 查看 USB 声卡编号，再修改 `.asoundrc` |
| **重启后可能变化** | USB 插拔顺序变化可能导致编号改变，之前配好的突然没声音 |
| **容易配错** | 如果填了 Intel HDA 的编号（假声卡），服务调用成功但没有声音输出 |

## 新方案原理

新脚本不再依赖 card 编号，而是从根源上解决问题：

```
1. blacklist 禁用 Intel HDA 声卡的所有内核模块
2. udev 规则确保 USB 声卡插入时加载驱动
3. 系统只剩一个声卡 → 它就是默认设备 → 不需要 .asoundrc
```

## 脚本执行内容

| 步骤 | 操作 | 配置文件 |
|---|---|---|
| 1 | 配置 udev 规则，USB 声卡插入时加载 snd-usb-audio 驱动 | `/etc/udev/rules.d/90-usb-audio.rules` |
| 2 | blacklist Intel HDA 相关模块，禁止加载 | `/etc/modprobe.d/alsa-card-order.conf` |
| 3 | 删除旧的 `.asoundrc` 文件（不再需要） | `/root/.asoundrc`, `/home/lab/.asoundrc` |
| 4 | 允许 root 用户使用 PulseAudio | `/lib/systemd/user/pulseaudio.service` |
| 5 | 将 root 和 lab 用户加入 audio 组 | - |
| 6 | 安装 alsa-utils、sox 依赖 | - |
| 7 | 拷贝预置音频文件到音乐目录 | `/home/lab/.config/lejuconfig/music/` |
| 8 | 更新 initramfs | `/boot/initrd.img-*` |

## 独立使用

```bash
cd ~/kuavo-ros-control/tools/change_dhcp_host
sudo ./roban_sound_card.sh
sudo reboot
```

## 对已部署机器的处理

如果机器之前使用旧版 `update_dhcp.sh` 配置过声卡（存在 `/root/.asoundrc` 手动指定 card 编号），有两种处理方式：

### 方式一：运行新脚本（推荐）

直接运行新脚本即可覆盖旧配置：

```bash
sudo ./roban_sound_card.sh
sudo reboot
```

脚本会自动：
- 删除旧的 `/root/.asoundrc` 和 `/home/lab/.asoundrc`
- 写入新的 blacklist 和 udev 配置
- 更新 initramfs

无需手动 `aplay -l` 查看编号，无需手动编辑任何文件。

### 方式二：不运行脚本，手动修改 card 编号

如果不想运行新脚本，可以手动修正 `/root/.asoundrc` 中的编号：

1. 查看 USB 声卡的实际 card 编号：

   ```bash
   aplay -l
   ```

   输出示例（USB 声卡为 card 2）：
   ```
   card 0: PCH [HDA Intel PCH], device 0: ALC897 Analog [ALC897 Analog]
   card 2: Device_1 [USB PnP Sound Device], device 0: USB Audio [USB Audio]
   ```

2. 找到带 `USB PnP Sound Device` 或 `USB Audio` 的那行，记下 card 编号（上例中为 `2`）。

3. 修改 `/root/.asoundrc`，将 card 编号改为实际值：

   ```bash
   sudo vim /root/.asoundrc
   ```

   ```
   pcm.!default {
     type hw
     card 2    ← 改为 aplay -l 看到的 USB 声卡编号
   }

   ctl.!default {
     type hw
     card 2    ← 同上
   }
   ```

4. 验证：

   ```bash
   play /home/lab/.config/lejuconfig/music/1_挥手.wav
   ```

> **注意**：手动方式的 card 编号在不同机器上可能不同（card 0、1 或 2），且重启或 USB 插拔后可能变化。如果遇到播放无声音的问题，需要重新执行上述步骤确认编号。推荐使用方式一彻底解决。
