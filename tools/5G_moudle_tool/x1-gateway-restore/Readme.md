# X1 网络环境一键恢复

X1 原型机网络架构中，**移远板**（QCS6490 / SG560D）作为机器人头部控制器（网关），
**Thor**（上位机）与 **RK3588**（下位机）通过交换机接入。本目录提供三块板子各自的
重刷镜像后一键恢复脚本。

| 脚本 | 运行位置 | 作用 |
| --- | --- | --- |
| [`setup-x1-network.sh`](#移远板脚本setup-x1-networksh) | 移远板（网关） | 用户改名、eth0/eth1 双网段、DHCP、NAT、端口转发、VNC、蜂窝射频与 DNS 自启 |
| [`setup-3588-network.sh`](#下位机脚本setup-3588-networksh) | RK3588（下位机） | 26/28 双网段静态 IP、路由 metric 固定 |
| [`setup-thor-network.sh`](#上位机脚本setup-thor-networksh) | Thor（上位机） | 同上（IP 为 26.12 / 28.12） |

> **三块板子都要配。** 移远板是网关，Thor/RK3588 是接入端。只配移远板、下位机 IP 落错口，
> 一样上不了网（见文末踩坑记录）。

## 网络约定

| 设备 | IP | SSH | VNC |
| --- | --- | --- | --- |
| 移远板（网关） | 192.168.26.1 | 自身 22 | 自身 5901 |
| Thor（上位机） | 192.168.26.12 | 移远板 23 转发 → :22 | 移远板 5902 转发 → :5902 |
| RK3588（下位机） | 192.168.26.13 | 移远板 24 转发 → :22 | 移远板 5903 转发 → :5903 |

> IP（12/13）与 VNC 端口（5902/5903）为两端约定的静态值，Thor / RK3588 真机必须按此配置，否则转发失败。外部经移远板端口 23/24/5902/5903 远程访问：**移远板按目标 IP 转发——配 26.12 的板子就是 Thor（走 23/5902），配 26.13 的板子就是 RK3588（走 24/5903）**。

**28 网段（DDS 专用）：**
| 设备 | IP |
| --- | --- |
| 移远板 eth1 | 192.168.28.1 |
| Thor | 192.168.28.12 |
| RK3588 | 192.168.28.13 |

- 28 网段为 DDS 数据专用链路，**不提供 DHCP**，需两端静态配置。
- 28 网段流量**只在内部转发**（移远板 eth1→eth1 放行），**不转发到 26 网段 / 外网**（FORWARD DROP 兜底）。
- 下位机配置自身静态 IP 即可 ping 通 26.1 / 28.1 网关。

### 物理接线（关键）

```
                        ┌── eth0 ── 交换机 ──┬── Thor 接交换机的口
   移远板 ──┤           │                    └── RK3588 接交换机的口
                        └── eth1 ── 直连 ────┬── Thor 直连口
                                             └── RK3588 直连口
```

- **接交换机的那条链路 → 配 26 网段的 IP**（这是唯一上网路径）
- **直连移远板 eth1 的那条链路 → 配 28 网段的 IP**（DDS，不配网关）

> ⚠️ **不要按网口名字（eth0/eth1）判断哪条线接哪里**。USB 网卡会抢占 `eth0`
> 这个名字（见文末踩坑记录），IP 会落到错误的物理口上。
> 用 `driver` 名或 DHCP 探测判断，或用脚本的 `--lan-iface/--dds-iface` 显式指定。

## 移远板脚本（`setup-x1-network.sh`）

X1 原型机网络架构中，移远板是机器人唯一网络出口，负责：
- 局域网 DHCP 分配（192.168.26.x）
- NAT 路由（下位机/上位机经移远板访问外网）
- SSH / VNC 端口转发（外部访问 Thor / RK3588）
- 28 网段 DDS 数据链路（内部隔离，不外发）

> 出口优先级：**默认走 WiFi（wlan0），WiFi 不可用时才走 5G（rmnet_data0）**。
> 5G 需模块射频处于 `AT+CFUN=1`；模块重启后会停在 `CFUN=0`（射频关），
> 脚本已内置开机自启服务自动打开（见下「蜂窝射频」）。

本脚本将工单 #2169（DHCP）、#2171（iptables NAT）、#2173（SSH/VNC 端口转发）、#2172（联调验证）中完成的全部配置固化为**一键恢复**，解决"重刷镜像后手工重配"的问题。

### 用法

在**移远板**（头部控制器）上执行：

```bash
sudo bash setup-x1-network.sh            # 全量恢复（含建用户 + VNC + 射频自启）
sudo bash setup-x1-network.sh --no-vnc   # 跳过 VNC 配置
sudo bash setup-x1-network.sh --no-rf    # 跳过蜂窝射频开机自启
```

> 用户名 / 密码写死为 `leju_x1` / `leju_x1`（交付约定），无需交互输入。

### 脚本功能

| 模块 | 恢复内容 |
| --- | --- |
| 用户 | 出厂用户 `quectel` **改名为** `leju_x1`（uid / 家目录内容 / 附加组原样保留），并设置密码；系统中不再存在 `quectel`，VNC 复用该用户 |
| NetworkManager | eth0 固定 `192.168.26.1/24`（robot-lan，metric 600，never-default，不抢外网默认路由） |
| dhcpcd | `denyinterfaces eth0/eth1`，防止抢地址 / 抢默认路由（曾导致外网全断） |
| DHCP | isc-dhcp-server 监听 eth0，地址池 `.100 ~ .200`，下发网关/DNS；28 网段空 subnet 声明（不提供 DHCP） |
| 28 网段 | eth1 固定 `192.168.28.1/24`（dds-lan，metric 600，never-default），FORWARD 放行 eth1→eth1 内部转发 |
| IP 转发 | `net.ipv4.ip_forward=1`（持久化） |
| iptables | NAT + DNAT 端口转发（23/24/5902/5903）+ FORWARD DROP，**同时覆盖 wlan0 与 rmnet_data0 两个出口**，28 网段内部隔离，自动清理旧网络残留（含 bridge0/ppp0），持久化并启用开机恢复 |
| VNC | 移远板自身桌面（5901，xfce4，systemd 自启，用户/密码 leju_x1） |
| 蜂窝射频 | 装 `x1-cellular-rf.service`（oneshot），开机经 port_bridge 的 AT 通道下发 `AT+CFUN=1` 打开射频；已是 1 则跳过，失败不阻塞开机 |
| 蜂窝 DNS | 装 `x1-cellular-dns.service`（oneshot），开机等 `rmnet_data0` 拿到 IP 后为它注册 DNS（优先运营商下发，兜底 `223.5.5.5 / 114.114.114.114`），保证 WiFi 掉线时域名可解析 |

### 出口冗余：WiFi ⇄ 5G 自动切换

iptables 的 NAT/DNAT/FORWARD 规则**同时覆盖 `wlan0` 和 `rmnet_data0`**，
实际走哪条路由内核选路决定 —— **WiFi 掉线自动走 5G，WiFi 恢复自动切回，无需重跑脚本**。

| 场景 | 默认路由 | 26 网段出网 |
| --- | --- | --- |
| WiFi 正常 | `wlan0`（metric 334） | ✅ 走 WiFi |
| WiFi 掉线 | `rmnet_data0`（metric 500） | ✅ 自动走 5G |
| WiFi 恢复 | `wlan0`（metric 334） | ✅ 自动切回 WiFi |

> **5G 出口必须单独注册 DNS**：蜂窝口在 NetworkManager 里是 `unmanaged`，
> NM 不会把它的 DNS 交给 `systemd-resolved`。WiFi 正常时看不出问题（resolv.conf 里有
> WiFi 下发的 DNS），WiFi 一掉线 NM 撤走那些 DNS，`resolved` 对 `rmnet_data0` 又是
> `Current Scopes: none` → **能 ping 通 IP 但域名报 `unknown host`**。
> 这正是 `x1-cellular-dns.service` 要解决的问题。

### 蜂窝射频开机自启（`x1-cellular-rf.service`）

模块重启后 `AT+CFUN` 停在 `0`（最小功能，射频关），不会自动注网/拨号，5G 兜底出口形同虚设。
脚本安装 `/usr/local/sbin/x1-cellular-rf-on.sh` 并注册 oneshot 服务：

```
等 port_bridge 的 AT 通道（127.0.0.1:9083）就绪（最多 60s）
  → AT+CFUN? 已是 1 则退出
  → 否则 AT+CFUN=1（最多重试 3 次）→ 复查确认
```

- 依赖 `port_bridge.service`（提供 AT 通道 ↔ `/dev/at_mdm0`），用 `Wants=` 而非 `Requires=`
- `SuccessExitStatus=0 1`：射频没开起来也不阻塞开机
- 手动验证：`systemctl start x1-cellular-rf.service` + `journalctl -u x1-cellular-rf.service`
- 无蜂窝模块的板子（无 `/dev/at_mdm0`）自动跳过

## 验证

脚本运行后自动输出验证信息：
- eth0 IP / 默认路由 / ip_forward
- NAT / DNAT 规则
- VNC 端口监听状态
- 外网连通性自检

手动检查：

```bash
ip -o -4 addr show eth0          # 应显示 192.168.26.1/24
ip route show default            # 应经 wlan0（WiFi 出口）
iptables -t nat -L PREROUTING    # 应有 8 条 DNAT（4 端口 × 2 出口）
iptables -t nat -S POSTROUTING   # 应有 2 条 MASQUERADE（wlan0 + rmnet_data0）
systemctl status vncserver@:2    # VNC 服务状态
systemctl status x1-cellular-rf  # 蜂窝射频自启服务
systemctl status x1-cellular-dns # 5G 出口 DNS 注册服务
```

**验证出口冗余**（关掉 WiFi 看 5G 是否接管）：

```bash
sudo nmcli con down Lejurobot
ip route show default            # 应只剩 rmnet_data0
ping -c3 baidu.com               # 应通（走 5G）
sudo nmcli con up Lejurobot      # 恢复
```

## 重启后自动生效（已实测）

脚本配置的东西**重启后全部自动恢复**，不需要重跑。实测（真机重启，`uptime` 归零后验证）：

| 项 | 依赖 | 重启后 |
| --- | --- | --- |
| 用户 `leju_x1` | passwd 文件 | ✅ |
| eth0 / eth1 的 IP | NetworkManager | ✅ |
| ip_forward | `/etc/sysctl.conf` | ✅ |
| **iptables（NAT/DNAT/FORWARD）** | **`iptables.service`** | ✅ 规则自动恢复 |
| DHCP 服务 | `dhcpd` + drop-in 等待 eth0 | ✅ |
| VNC | `vncserver@:2` | ✅ |
| 蜂窝射频 | `x1-cellular-rf.service` | ✅ CFUN 自动置 1 |
| 5G DNS | `x1-cellular-dns.service` | ✅ |
| SSH | `ssh.socket` | ✅ |

> ⚠️ **踩过的坑**：出厂镜像自带的 `iptables.service` 读的是 `/etc/iptables/iptables.rules`
> （**不是** Debian 惯用的 `rules.v4`），且默认 `disabled`。早期版本只写了 `rules.v4` 又没启用
> 任何服务 → **重启后规则全丢**，表现为端口转发（:23/:24/:5902/:5903）连不上、
> 26 网段上不了外网。现在两个路径都写，并 `enable iptables.service`（不依赖联网装包）。
>
> 另一个坑：`dhcpd.service` 只有 `After=network.target`，但 eth0 的 IP 由 NetworkManager
> 在更晚阶段配置 —— 开机瞬间 eth0 没地址，dhcpd 直接退出（`status=1/FAILURE`），
> 26 网段拿不到 DHCP。已用 drop-in 补上等待条件。

## 注意事项

1. **首次运行前请先确认 WiFi（wlan0）已联网**，否则出口检测可能失败。
2. **出口优先级**：脚本按 `WAN_CANDIDATES=("wlan0" "rmnet_data0")` 检测——**WiFi 优先，5G 兜底**。
   只认真正拿到 IPv4 的口（避免把 NAT/DNAT 挂到未拨号的空壳口上）。
   实测 WiFi metric 334、5G metric 500，默认路由本来就走 WiFi，与该优先级一致。
   **NAT/DNAT 规则同时覆盖两个出口**，所以 WiFi ⇄ 5G 切换不需要重跑脚本。
3. **5G 驻网制式**：实测 `AT+QENG="servingcell"` 报 `"LTE"`（B3，earfcn 1300），
   即**驻留在 4G LTE 而非 5G NR**；`AT+QNWPREFCFG` 等 5G 相关 AT 命令返回 `ERROR`。
   若需确认能否真正承载 5G，需向移远确认。
4. **VNC 密码**：写死为 `leju_x1`（与登录密码一致）。VNC 协议密码最长 8 字符，`leju_x1` 为 7 字符，正常生效。
5. **脚本会清空并重建 iptables 规则**（执行前自动备份到 `/tmp/iptables-rules.backup.*`），请勿在已配置好的环境中随意执行。
6. 脚本幂等，可重复执行，不会叠加规则。

## 下位机脚本（`setup-3588-network.sh`）

在 **RK3588** 上执行，恢复与移远板网关对接的网络配置。

```bash
sudo bash setup-3588-network.sh                                      # 自动识别网口
sudo bash setup-3588-network.sh --lan-iface eth1 --dds-iface eth0    # 显式指定
```

| 模块 | 恢复内容 |
| --- | --- |
| 接交换机链路 | `192.168.26.13/24` + 网关 `192.168.26.1`，metric **100**（保证回包走这条） |
| 直连链路 | `192.168.28.13/24`，无网关，metric 600（DDS 专用，不参与外网路由） |
| 无线连接 | 所有 wlan 连接 metric 降到 **30000**，只作兜底不抢 26 段 |
| DNS | `223.5.5.5, 114.114.114.114`（26.1 本身不提供 DNS） |

**网口自动识别**（按可靠性排序）：
1. `--lan-iface` / `--dds-iface` 显式指定
2. **DHCP 探测**：只有交换机侧有 DHCP 服务（移远板 dhcpd 只监听 eth0），
   直连链路（28 段）不提供 DHCP —— 能拿到 `192.168.26.x` 租约的就是接交换机的口
3. **驱动名兜底**：USB 网卡 = 直连，非 USB（板载）= 接交换机

**为什么要固定 metric**：NetworkManager 会给「非默认连接」的默认路由加 20000 惩罚，
加给谁取决于**激活顺序** —— 同一台机器两次开机 metric 会翻转（实测
`20100/20600` ↔ `40100/600`），26 段时通时不通。把两条路 metric 拉开 >20000
（100 vs 30000），惩罚加给谁都改变不了优先级。

## 上位机脚本（`setup-thor-network.sh`）

在 **Thor** 上执行，配置与 RK3588 完全同源，仅 IP 不同：

```bash
sudo bash setup-thor-network.sh                                      # 自动识别网口
sudo bash setup-thor-network.sh --lan-iface eth0 --dds-iface eth1
```

| 项 | Thor | RK3588 |
| --- | --- | --- |
| 接交换机（26 段） | `192.168.26.12/24` + 网关 | `192.168.26.13/24` + 网关 |
| 直连（28 段） | `192.168.28.12/24` | `192.168.28.13/24` |

> Thor 是 Jetson 平台，出厂镜像可能用 **systemd-networkd** 而非 NetworkManager。
> 脚本会先检测：若 NetworkManager 没装，会提示改用 netplan 或先安装 network-manager，
> **不会擅自切换网络栈**。
>
> ⚠️ Thor 真机尚未接入联调，脚本**未在实机验证过**，首次使用请先 `--help` 看清参数
> 并留意输出。RK3588 版本已在实机验证。

## 踩坑记录：下位机（RK3588）侧「26 网段不能上网 + 端口转发连不上」

> 2026-09-16 / 09-18 两轮联调排查。**结论：问题 100% 在下位机侧，移远板（本脚本）配置正确、无需改动。**
>
> ⚠️ 第一轮排查曾误判为「网线接反」，实为下位机 **USB 网卡抢占 `eth0` 命名** 导致 IP 落在了错误的物理口上。
> 现场人员接线自始至终是对的。

### 现象

- 下位机 `ping 192.168.26.1`（网关）**通**
- 下位机 `ping 223.5.5.5` / DNS 解析 → **100% 丢包 / 失败**
- 外部经移远板 `:24`（SSH）/`:5903`（VNC）访问下位机 → **连接超时**

### 根因：下位机 `eth0` 是 USB 网卡，IP 落在了错误的物理链路上

RK3588 上的网口与内核命名**不对应**（`net.ifnames=0`，名字按枚举顺序给）：

```
eth0  mac=00:e0:3a:46:0e:c5  driver=r8152  ← USB 网卡（/sys/devices/.../usb1/1-2/1-2.1）
eth1  mac=3c:6d:66:16:3b:ce  driver=r8168  ← 板载 PCIe 网卡（/sys/devices/.../pcie/...）
```

NetworkManager 的 profile 是**按内核口名绑定**的，于是 IP 落反：

| 内核名 | 物理口 | 实际接线 | 原 IP（错） | 修正后 |
| --- | --- | --- | --- | --- |
| `eth1` | **板载 RJ45** | 接交换机 | `192.168.28.13/24` ❌ | `192.168.26.13/24` + 网关 26.1 |
| `eth0` | **USB 网卡** | 直连移远板 eth1 | `192.168.26.13/24` + 网关 ❌ | `192.168.28.13/24`（无网关） |

**核心要求不是「口名必须是 eth0/eth1」，而是「哪条物理链路接交换机，哪条链路就要配 26 网段的 IP」。**
移远板 DNAT 规则只认目标 IP（`--to-destination 192.168.26.13:22`），不关心下位机口名。

于是下位机的默认路由变成 `via 192.168.26.1 dev eth0`——**上网出口挂在了直连的 28 段链路上**。
数据包从 `eth0`（USB 口）发到移远板 `eth1`，而移远板 FORWARD 链只放行 `-i eth0 -o wlan0`，
其余的命中 `policy DROP`，所以 26 段上不了外网。

### 定位方法：抓帧的源/目的 MAC（与「线怎么插的」无关，客观可靠）

在两个口同时抓包，看每一帧的二层地址：

```
移远板 eth1 抓到:  00:e0:3a:46:0e:c5 > 00:55:18:38:16:33   192.168.26.13 > 223.5.5.5
                   ↑ 下位机 eth0(USB)     ↑ 移远板 eth1
```

即「下位机 `eth0` 发出的、源 IP 为 `26.13` 的包，出现在移远板 `eth1`（28 段口）上」。
再对照下位机 `ip neigh` / `ip -o addr`，即可确认 IP 与物理口的错配。

辅助判据（同样有效）：

```bash
# 在移远板看每个口各自的 L2 邻居 MAC —— 每个口只应看到一个对端 MAC
tcpdump -i eth0 -e -n arp | grep -oE '([0-9a-f]{2}:){5}[0-9a-f]{2} >' | sort -u
tcpdump -i eth1 -e -n arp | grep -oE '([0-9a-f]{2}:){5}[0-9a-f]{2} >' | sort -u

# 在下位机看网口到硬件的映射（driver 名比口名可靠）
for i in eth0 eth1; do
  echo "$i $(cat /sys/class/net/$i/address) $(basename $(readlink -f /sys/class/net/$i/device/driver))"
done
```

### 修复（下位机侧操作）

**按下位机脚本一键恢复（推荐）**，或手工按下表操作：

```bash
sudo bash setup-3588-network.sh          # 自动识别网口并配好 26/28 两段
```

手工方式（`Wired connection N` 与 `ethN` 的对应关系各机器可能不同，
动手前先 `nmcli -t -f NAME,DEVICE con show` 确认）：

```bash
# 板载口（接交换机）→ 26 段：静态 IP + 网关，metric 100
nmcli con mod "Wired connection 1" \
    ipv4.method manual \
    ipv4.addresses 192.168.26.13/24 \
    ipv4.gateway 192.168.26.1 \
    ipv4.route-metric 100 \
    ipv4.never-default no \
    ipv4.dns "223.5.5.5,114.114.114.114" ipv4.ignore-auto-dns yes

# USB 网卡（直连移远板）→ 28 段：静态 IP，不配网关
nmcli con mod "Wired connection 3" \
    ipv4.method manual \
    ipv4.addresses 192.168.28.13/24 \
    ipv4.gateway "" \
    ipv4.route-metric 600 \
    ipv4.never-default yes

# 无线连接降到最低优先级，不抢 26 段
nmcli con mod "Lejurobot" ipv4.route-metric 30000

nmcli con up "Wired connection 3"; nmcli con up "Wired connection 1"
```

> **metric 必须拉开 >20000 的差距**（100 vs 30000）。NM 给「非默认连接」的默认路由
> 加 20000 惩罚，加给谁取决于激活顺序 —— 只设 `20100`/`20600` 这种接近的值，
> 两次开机就可能翻转成 `40100`/`600`，26 段时通时不通。见上文「下位机脚本」。

### 验证结果（修复后）

| 测试项 | 结果 |
| --- | --- |
| 下位机 → 外网 `ping 223.5.5.5`（走 26 段 NAT） | ✅ 0% 丢包，17~29ms |
| 下位机 → DNS 解析 `baidu.com` | ✅ 正常 |
| 下位机 → 网关 `ping 192.168.26.1` | ✅ 0% 丢包 |
| 端口转发 `:24` → 下位机 SSH | ✅ OPEN |
| 端口转发 `:5903` → 下位机 VNC | ✅ OPEN |
| 移远板 eth0 上看到的邻居 | ✅ 只有板载口 MAC `3c:6d:66:16:3b:ce` |
| 端口转发 `:23` / `:5902` → Thor | ⏸ Thor 未接入，待验 |

### 其他两个坑（同一现象的不同成因，一并记录）

**① ARP flux 掩盖故障**

移远板 Linux 默认 `arp_ignore=0`，即**任一网口都会替本机任意地址应答 ARP**。
下位机在错误链路上 ARP 询问 `192.168.26.1` 时，移远板会用**自己的 MAC** 代答
（`26.1` 其实只配在 `eth0` 上）。结果是「`ping 26.1` 居然能通」，把配置错误掩盖成了
「网关正常但上不了网」。

**判据**：把移远板某口 `arp_ignore` 临时设为 `1`（禁止代答非本口地址）并清空下位机 ARP 缓存，
若 `26.1 dev <该口>` 立刻变成 `INCOMPLETE`，即证明该口不在 26 段链路上。

**② 下位机自带 wlan0 外网默认路由 → 端口转发回包走错路**

下位机同时有 `default via <wifi网关> dev wlan0`（测试台特有的 WiFi 外网）。
即使 IP 已配对，若 26 段默认路由的 metric **高于** wlan0，回包仍会走 wlan0：

```
外部 → 移远板 :5903 --DNAT--> 192.168.26.13:5903
下位机回包时选路: ip route get <外部IP> → via wlan0     ← 源地址变成 wifi 的，对不上
→ 外部收到来源不符的 SYN-ACK，连接被丢弃 → VNC/SSH 转发超时
```

**判据**：在下位机执行 `ip route get <客户端IP>` 看走哪个口。
修法是让 26 段默认路由的 metric 优于 wlan0 —— 但**必须拉开 >20000 的差距**
（实测用 `100` vs `30000`），否则 NM 的 20000 惩罚会随激活顺序翻转优先级，问题复发。

### 教训

1. **`ping` 通网关 ≠ 能上网**。网关地址是本地投递（INPUT 链），而转发流量走 FORWARD 链；
   `arp_ignore=0` 会让网关在错误网段上"看起来可达"，掩盖真实的链路错配。
2. **别用口名判断接线，用 MAC 和驱动名**。USB 网卡会抢占 `eth0`，`net.ifnames=0` 下名字不稳定；
   `readlink -f /sys/class/net/<if>/device/driver` 才是可信的物理口标识。
3. **排查第一步**：在网关侧抓帧看源/目的 MAC，或清空 ARP 表重新学习，
   客观还原「哪台设备的哪个物理口在哪条链路上」，比看线怎么插的可靠。
4. **下位机若有独立外网出口**（如测试台 WiFi），必须保证业务网段的路由 metric 更优，
   否则回包走错口会导致 DNAT 端口转发静默失败。
5. **metric 要拉开足够大的差距**。NM 对「非默认连接」加 20000 惩罚，加给谁看激活顺序 ——
   `20100` / `20600` 这种接近的值两次开机就可能翻转，必须用 `100` / `30000` 这种量级差。
6. **USB 网卡命名不稳定**是长期隐患：拔插或开机顺序变化，`eth0`/`eth1` 可能互换，网段再次串位。
   根治需用 udev 按 MAC 固定网卡名（或不用 USB 网卡）——**待设计确认**。

## 相关文档

- [5G 模块接入](./../Readme.md) — 5G 模块配置、DHCP 主机、拨号守护说明
- [移远板网络拓扑](../../../../docs/) — X1 网络拓扑与端口约定
- [SG560D_Ubuntu_QLRIL_蜂窝网络上网指南](./SG560D_Ubuntu_QLRIL_蜂窝网络上网指南.md) — 移远官方拨号文档
