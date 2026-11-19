# 关闭轮臂机器人 NTP 服务

轮臂控制器启动时会检查**控制器所在主机**的 NTP 状态。NTP 开着时日志里会出现：

```text
[ntp] 控制器所在主机的 NTP 服务已开启（timedatectl NTP=yes，正在运行: systemd-timesyncd.service）。...
```

未开启时是一条 INFO：`timedatectl NTP=no`。查不到 `timedatectl` 时也会打警告。这条日志只提示，不阻止控制器启动。

## 风险

轮臂底盘的时间跟着机器人走。机器人联网后，NTP 如果发现本地时钟和标准时间差得很多，会把系统时间一次性拨过去。

底盘时钟跟着跳。`robot_localization` 用时间间隔积分里程计，这一跳会让位姿发散：上报速度仍然很小，位置却跳出几米到上千米。发散后的 `/odom` 进入 MPC 后，基座跟踪代价淹没手臂跟踪，MPC 失控。

控制器里的里程计保护会在这之后停掉 MPC 并保持关节指令，错误码锁存在 `/humanoid_wheel/error_code`：

| 错误码 | 含义 |
| --- | --- |
| 31001 | 里程计断流 |
| 31002 | 位姿或速度突变 |
| 31003 | 里程计含 NaN/Inf |
| 31004 | 消息时间戳跳变 |

保护锁存后不会自动恢复，需要重启控制器。关闭 NTP 是为了不让这次阶跃发生。

## 先确认是谁在对时

在**跑轮臂控制器的那台机器人主机**上执行：

```bash
timedatectl
```

关注这一行：

```text
NTP service: active
```

`active` 表示 NTP 开着。再看具体服务：

```bash
systemctl is-active systemd-timesyncd.service chrony.service chronyd.service ntp.service ntpd.service
```

输出为 `active` 的那一项就是要关的服务。Ubuntu 上常见的是 `systemd-timesyncd.service`。

底盘如果自己也跑了 NTP，同样会改底盘时钟。控制器只能看到自己所在的主机，底盘上的 NTP 需要单独到轮臂底盘系统里检查、关闭。

## 关闭

下面的命令都在机器人主机上执行，需要 sudo。

先关掉 systemd 管理的网络对时：

```bash
sudo timedatectl set-ntp false
```

再按上面查到的服务名禁用，避免下次开机又拉起来。只执行正在运行的那一个：

```bash
# systemd-timesyncd（timedatectl 默认使用的服务）
sudo systemctl disable --now systemd-timesyncd.service

# chrony
sudo systemctl disable --now chrony.service

# ntpd
sudo systemctl disable --now ntp.service
```

服务名是 `chronyd.service` 或 `ntpd.service` 时，把上面的单元名换成实际名称。

## 确认已经关掉

```bash
timedatectl show -p NTP --value
```

输出应为 `no`。`timedatectl` 里对应 `NTP service: inactive`。

重启控制器后，启动日志应为：

```text
[ntp] 控制器所在主机的 NTP 服务未开启（timedatectl NTP=no）
```

不要在运行中用 `ntpdate`、`chronyc makestep` 这类会立刻改系统时间的命令。时钟可以慢慢漂移；不能在底盘正在跟随时把时间阶跃拨过去。
