# 个人手势阈值标定（全身 IK）

适用本地仓库 `/home/chenzibo/kuavo_ws/kuavo-ros-control`、分支 `czb/beta/glove-vel-ik`。本功能按操作者、左右手和当前穿戴状态分别标定三种捏合距离与握拳距离，合格结果默认供下次遥操启动自动加载。

这是**手势识别阈值标定**，与 Quest 手背安装的姿态、15 cm 距离补偿无关。不会修改末端位置映射、手指关节映射或 X+Y 的时间保护。识别距离来自手套传感模块位置，不等于人体实际指尖间距；重新穿戴、换操作者或调整模块后建议重做。

## 1. 编译与环境

```bash
cd /home/chenzibo/kuavo_ws/kuavo-ros-control
source /opt/ros/noetic/setup.bash
catkin build sg100_glove_bridge
source devel/setup.bash
```

已有全身 IK 工作区只更新本功能时编译上述包即可。首次构建全身遥操仍按主说明编译 `wheel_arm_abs_vel` 及其依赖。

## 2. Windows UDP 标定

先正常退出运控、遥操、手套接收节点及 MuJoCo；手套和 Windows 发送软件保持运行。**不需要启动机器人运控或 Quest 应用。** Windows 目标 IP 填这台接收电脑的地址，默认端口：

| 手 | IKHand UDP | SendDof UDP |
| --- | --- | --- |
| 左手 | 18003 | 18003 |
| 右手 | 18002 | 18002 |

同一只手的两种协议共用端口，标定必须收到左右 **SendDof**；只有 IKHand 关节数据不能标定。五指模块顺序默认 `A B C D E`（拇指、食指、中指、无名指、小指）。必须与日常遥操配置一致。

```bash
rosrun sg100_glove_bridge calibrate_glove_gestures.py --profile 张三
```

脚本自己启动独立、仅监听本机的 ROS master（默认 11339）与数据接收节点，无需另开 `roscore`。不会启动手势按键、Quest、运控或手指执行器；后端的关节输出被导向无订阅者的丢弃话题。若检测到已有遥操接收进程、端口占用或非采集节点，会报错退出，不会结束其他程序。

端口、模块映射不同时可指定：

```bash
rosrun sg100_glove_bridge calibrate_glove_gestures.py --profile 张三 \
  --left-port 18003 --right-port 18002 \
  --left-module-map A B C D E --right-module-map A B C D E
```

按终端提示做 **3 轮**；前两轮拟合，第三轮独立验证。每个动作前先双手张开、按回车，保持到“采集质量”提示；有效释放数据至少 1 秒。再摆好提示的动作、按回车，默认采集至少 4 秒有效保持数据。出现间隔超限时自动清空双手当前窗口并重新计时，整个步骤最多额外等待 10 秒，动作始终保持到完成提示。每轮包含：

1. 张手、自然放松、四指伸直并拢（拇指分开）。
2. 双手分别捏食指、捏中指、捏小指。
3. 双手握拳、仅左手握拳、仅右手握拳。
4. X+B：左拇指捏食指、右拇指捏中指。
5. X+Y：左拇指同时捏食指和中指，左无名指/小指伸开，右手张开。

非目标动作也必须认真采集，用来检查误触，特别是单手握拳误识别为 X+Y。短暂间隔超限会自动丢弃当前双手窗口并重新采集，保持当前动作。达到总时限后可以手动重试，目标重试不用重复已通过的释放步骤。输入 `q` 或 Ctrl+C 保存进度并退出。需要更长保持可加 `--hold-seconds 6`（范围 3–15 秒）。

### 提示标定 master 端口占用时

默认使用本机 11339。新版探测允许上次正常退出后的 TCP TIME_WAIT，不会因此误认为已有 master；真正有进程占用时仍拒绝连接。检查 `ss -ltnp 'sport = :11339'`，先正常退出已有标定任务。若该端口由其他需要保留的服务使用，可加 `--master-port 11340` 指定另一个空闲端口；不要使用日常运控的 11311。该参数只改变标定 ROS master，不改变左右手套 UDP 端口。

### 一直提示重录时

新版标定入口保留 UDP 批次内所有通过顺序校验的真实 DoF 帧，不复制或插值补帧；普通遥操仍只取每批最新帧。采集结束显示左右实际帧数、有效时间跨度、平均 Hz、最大 ROS 到达间隔、末帧距今时间。

要求双手平均至少 20 Hz、到达间隔及末帧年龄不超过 0.3 秒，并检查左右记录的首尾对齐。有效时长从实际第一帧到最后一帧计算，最多额外等待 10 秒（重置不延长总时限）。未满足时拒绝该段，不会靠无限等待跳过质量检查。

- 偶发约 0.2 秒成批到达：新版可以正常采集，保持到完成提示即可。
- 到达间隔超过 0.3 秒：检查 Windows SendDof 发送、无线网络和本机负载。日志说明接收间隔过大，并不单凭它断言发送端断流。
- 平均频率不足 20 Hz 或仅一侧末帧很旧：核对该侧 SendDof、端口和网络；IKHand 不能代替 DoF。
- 若使用旧进程，退出后用原命令重启标定；代码修改不会更新正在运行的 Python 进程。

## 3. Linux SDK / 单板双手

同一接收板同时读取左右手套，默认左 ID 12、右 ID 2：

```bash
rosrun sg100_glove_bridge calibrate_glove_gestures.py --profile 张三 \
  --backend linux_sdk --serial-port auto --left-id 12 --right-id 2
```

此时通过原有 SDK 自动搜索串口，不需要 Windows UDP。可用 `--serial-port /dev/ttyUSB0` 或 `--sdk-python /绝对路径/python3` 指定环境。当前标定流程要求**双手**，单手日常启动并不等于可做单手标定。

## 4. 自动生效及日常启动

**是的：标定通过且未加 `--no-activate` 时，下次启动默认自动应用，无需重新编译或手动填写阈值。** 已运行的遥操不会热更新；重新启动遥操后生效。

通过后自动写入：

```text
src/manipulation_nodes/sg100_glove_bridge/config/glove_gesture_calibration/
  active.yaml              下次启动默认使用
  profiles/张三.yaml         具名配置
  recordings/*.json         原始采集数据
  reports/*.json            质量结果及失败原因
```

本机完整目录为：

```text
/home/chenzibo/kuavo_ws/kuavo-ros-control/src/manipulation_nodes/sg100_glove_bridge/config/glove_gesture_calibration/
```

目录中的 `active.yaml` 用于启动自动读取，`profiles/` 保存具名配置，`recordings/` 保存原始数据和断点，`reports/` 保存质量报告。以下涉及 `src/...` 的文件命令在仓库根目录执行。默认目录由包位置计算，无需每次传 `--directory`；文件夹内的 README 说明参与代码管理，当前工作区的个人生成数据由本地忽略规则排除，不纳入代码提交。

旧 `~/.ros/glove_gesture_calibration/` 不再自动读取。本次已复制旧的三份原始记录到新目录，原文件保留作备份；最新的 `张三_20261123_154123_481071.json` 包含 9/33 段，可用新路径断点继续。还没有通过完整标定生成的 `active.yaml`，它会在完整验证通过后自动创建。Quest 安装偏置的 bag 路径仍为 `~/.ros`，本次不改变。

合格配置包括左右各四组按下/释放阈值。两组距离分布至少分离 4 mm，并用第三轮数据回放真实手势识别逻辑，检查误触、交互脉冲、握拳脉冲和释放/断流。数据重叠、动作不正确或验证失败时**不会替换 active**，应根据报告重新调整穿戴/模块映射并采集，不能靠放大阈值强行通过。

之后使用原来的全身启动流程即可，例如在已准备好的遥操环境中：

```bash
roslaunch wheel_arm_abs_vel quest_windows_udp.launch ip_address:=<Quest头显IP>
```

`quest_windows_udp.launch`、`quest_velocity_ik.launch` 默认启用加载；Linux 独立 `sg100_glove_bridge.launch` 也接入了相同默认文件。启动时打印配置名字，运行后可以查看：

```bash
rostopic echo -n 1 /glove_gesture/calibration_status
```

- 默认 active 不存在：沿用原来的 launch 阈值。
- active 存在但损坏，或 UDP/SDK 后端、模块映射、板卡 ID 不匹配：手势节点报错，所属 launch 退出，需要选择相符配置或重新标定。
- 配置只在启动时加载；采集或切换配置不会热修改正在运行的遥操。
- 标定与遥操应 source **同一份仓库**。默认路径与 HOME、当前终端目录无关；切换账号不改变目录，但必须保证文件读写权限。

状态中 `profile_loaded: true` 表示确实已加载，`profile_name` 为操作者，`source` 为实际文件路径，`thresholds` 为左右手生效值。

本地与机器人使用不同文件系统。个人配置位于各自仓库的上述目录，当前工作区通过本机 `.git/info/exclude` 忽略生成数据，不会随代码提交或拉取同步；机器人需先更新本功能代码，再在接收机标定，或复制输入配置一致的合格文件并指定实际路径。

全身入口可显式覆盖或关闭：

```text
glove_gesture_calibration_file:=/绝对路径/active.yaml
glove_gesture_calibration_enable:=false
```

独立 `sg100_glove_bridge.launch` 的对应参数不带 `glove_` 前缀：`gesture_calibration_file`、`gesture_calibration_enable`。关闭会恢复原 launch 距离阈值。标定配置存在时，它优先于 launch 的距离参数，保持时间、超时和其他模式逻辑仍由原参数控制。

## 5. 多人切换、离线重算与回退

```bash
# 列出已保存配置
rosrun sg100_glove_bridge calibrate_glove_gestures.py --list
# 切换下一次启动使用的人
rosrun sg100_glove_bridge calibrate_glove_gestures.py --activate 张三
# 只保存合格结果，暂不设为启动默认
rosrun sg100_glove_bridge calibrate_glove_gestures.py --profile 李四 --no-activate
# 用本工具的原始记录离线重算；通过后仍默认激活
rosrun sg100_glove_bridge calibrate_glove_gestures.py \
  --from-recording src/manipulation_nodes/sg100_glove_bridge/config/glove_gesture_calibration/recordings/张三_实际时间.json
```

每次成功替换 YAML 前保留上一版本为同目录 `原文件名.before-时间戳` 文件。`--directory /自定义目录` 可改变标定输出目录，但日常 launch 仍默认读取 `src/manipulation_nodes/sg100_glove_bridge/config/glove_gesture_calibration/active.yaml`；自定义目录需显式设置 `glove_gesture_calibration_file`。

本工具使用带动作标签的现场采集 JSON，不从普通遥操 bag 猜测动作。之前的 Quest 安装偏置 bag 标定仍按原流程使用。


### 标定中断后继续（断点恢复）

每完成一个目标动作，立即保存原始采集 JSON，终端显示已保存的 `N/33` 和文件路径；手动退出也保留已完成动作。网络抖动时，脚本只丢弃当前不连续窗口，不删除已保存阶段；并且总等待时间固定，不会无限自动重来。目标动作默认有效保持 4 秒、总等待上限约 14 秒，释放段约 11 秒。

继续时使用终端打印的实际 JSON 文件路径：

```bash
rosrun sg100_glove_bridge calibrate_glove_gestures.py --profile 张三 \
  --resume-recording src/manipulation_nodes/sg100_glove_bridge/config/glove_gesture_calibration/recordings/张三_实际时间.json
```

恢复前严格检查操作者、后端/映射/ID、已有数据质量及连续阶段顺序，从下一未完成动作继续；保留原记录，写入新的记录文件。Linux SDK 恢复需带上原来的 `--backend linux_sdk` 和对应板卡参数。恢复时保持同一操作者、手套穿戴和模块安装；若重新穿戴或调整模块，应重新标定，不能拼接旧数据。所有 33 段完整验证通过后才自动更新 active。`--from-recording` 用于完整数据离线重算，不能代替断点继续。

本机实测 Windows Wi-Fi 发送存在源帧连续但到达批量延迟的情况。建议发送端改用有线与接收端做对照；自动寻找连续窗口只改善标定流程，不会消除实际遥操链路的网络延迟。
