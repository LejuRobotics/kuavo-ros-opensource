# 绝对速度 IK 使用指南：研扬 / AGX，62 / 63 / 400062

本指南适用于 `wheel_arm_abs_vel`。常规启动自动识别 CPU 平台、在线核与已保存的机器人版本；不需要每次填写平台、核数、`robot_version` 或手型参数。两端均使用 `platform_roslaunch.sh`，以使本启动链的后台子进程继承正确核池。

源码与调度改动限于本包。增量 IK、人形控制、公共 WBC 和硬件驱动沿用固定 dev 基线；本指南中的命令启动的是绝对式链路，不要同时运行旧链路的命令发布端。

## 1. 5.19 当前部署与快速启动

- 机器：AAEON `192.168.5.19`，Core Ultra 7 255H，16 核在线。
- 工作区：`/home/lab/lhm/kuavo-ros-control-absolute-cpu5-20261019`。
- 已保存机型：`62`，root 和 lab 的 `.bashrc` 均已配置，不需要再修改。
- Quest IP：示例沿用 2026-10-19 启动记录的 `192.168.5.21`；若头显换了地址，只替换此参数。平台与机型识别不会推测网络地址。
- 以下命令使用 `base_locked:=false` 开启摇杆底盘；仅测手臂时改回true，省略该参数也默认锁定。

**在 5.19 的两个终端各执行一次准备命令：**

```bash
cd /home/lab/lhm/kuavo-ros-control-absolute-cpu5-20261019
sudo bash --noprofile --norc
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rospack find wheel_arm_abs_vel
```

最后一条应指向上述新工作区中的 `src/wheel_arm_abs_vel`。不要同时 source 旧工作区。即使该干净 root shell 没有继承 `ROBOT_VERSION`，入口也会读取 root 已保存的字面量配置。

**终端 1：启动实机控制器。**

```bash
bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel load_kuavo_real_velocity_ik.launch \
  dynamics_backend:=rnea vr_remote_control:=true base_locked:=false
```

**终端 2：控制器正常就绪后，启动 Quest 输入、Mapper 与速度 IK。**

```bash
bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel quest_velocity_ik.launch \
  ip_address:=192.168.5.21
```

两个终端使用同一个工作区、同一个已保存机型。无需每次 `export ROBOT_VERSION`，也无需再传 `robot_version:=62`、`ee_type:=qiangnao` 或平台核数。

## 底盘操作（实机绝对式）

终端1只需使用`base_locked:=false`；终端2保持原命令，不增加额外启动命令。参数在控制器启动时读取，运行中修改rosparam不会解除锁定。

1. 长按A至少0.35秒请求初始化，等待就绪，再按原流程用X+A两次从复位进入跟随；只有ACTIVE且模式2就绪后摇杆才允许驱动底盘。
2. 左摇杆前后控制前进／后退，左右控制横移；右摇杆左右控制原地转向。前后、横移各轴上限0.40 m/s，转向上限0.45 rad/s。沿用原摇杆死区、平滑和底盘加减速限制，不修改手臂、腰部滤波或增益。
3. 右摇杆纵向幅度超过0.8执行原躯干操作时，底盘输入清零。松开摇杆目标回零，经原加减速限制减速；X+B暂停／固定、退出跟随、异常、急停或参考失效时禁止底盘指令。底盘通道超过0.20秒未收到新指令即清零，不重放旧摇杆值。

62与400062沿用已有UCore服务重启步骤，400062的启动条件已补齐。ROS发布地址默认为底盘网段192.168.26.1，覆盖ROS_HOSTNAME以避免底盘无法解析研扬主机名；这是控制器地址，Quest的ip_address仍填头显地址。其他机器若采用不同底盘网段，仅在终端1设置ros_ip:=该控制器可达地址。63共用速度门控，但未据此改变其现有底盘服务启动方式，仍需按实物验证通信与执行。

## 2. 其他机器：机型只配置一次

研扬与 AGX 使用同一套入口；将上面的 `cd` 换成该机器实际已构建的工作区。ROS 与依赖必须已经部署，本入口不安装依赖。

机型读取优先级：

1. 显式 `robot_version:=62/63/400062`，用于有意覆盖当前机器配置。
2. 当前进程的 `ROBOT_VERSION` 环境变量。
3. 当前运行用户的 `~/.bashrc` 与 `/etc/environment` 中唯一一致的字面量赋值。

没有前两项时，解析器只读取配置文本，不执行 shell 文件。多个不同的赋值、非法值或 `ROBOT_VERSION=$OTHER` 等非字面量写法会报错；不会默认为 62。仅当机器尚未配置或机型确实变更时，在**实际启动用户**的 `.bashrc` 中保留一条正确赋值，例如：

```bash
export ROBOT_VERSION=63
```

这是一条要保存到配置文件中的示例，不是每次启动都要输入的命令。实机使用 root 时，配置的是 `/root/.bashrc`；普通用户的配置不会自动替代 root 的配置。检查已有环境与持久配置是否一致；旧 shell 中的环境变量优先于文件，修改文件后可重新打开终端，或在确认后 `unset ROBOT_VERSION` 让入口重新读取文件。

CPU 型号无法区分 62、63 与 400062。不要依据遗留的 TotalMass 文件、安装了哪些 URDF、主机 IP 或核数猜测机型。

临时覆盖示例：在**两个终端**的对应启动命令末尾都增加 `robot_version:=400062`。只覆盖一端会造成模型不一致。400062 默认 `ee_type=heiman`；62、63 默认 `qiangnao`，实物末端不同才显式指定 `ee_type`。

## 3. 启动前检查与日志判断

在已准备好环境、位于工作区根目录的终端执行：

```bash
python3 src/wheel_arm_abs_vel/scripts/absolute_robot_version.py
python3 src/wheel_arm_abs_vel/scripts/runtime_partition.py --check
```

第一条输出机型与配置来源；第二条只读检查在线核、隔离核、cpuset、核分配与 `valid`，不改变线程亲和度或调度。应确认 `valid: true`。

仅验证 launch 解析、不启动节点，可在实际启动命令末尾加 `--nodes`：

```bash
bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel load_kuavo_real_velocity_ik.launch \
  dynamics_backend:=rnea vr_remote_control:=true base_locked:=true --nodes
```

此方式会给检查进程本身设置后台亲和度，但不启动机器人节点。解析通过不代表硬件已就绪。

5.19 正常日志应包含以下信息（实际路径可能不同）：

```text
[absolute startup] robot_version=62 source=/root/.bashrc
profile=.../runtime_partition_aaeon_255h.json IK isolation=soft sidecar=[14, 15]
Soft-Isolation Fallback ... IK=[5] ... background=[0, 1, 8, 9, 10, 11, 12, 13]
```

若从环境继承机型，来源显示 `environment`；显式参数显示 `launch argument`。`Soft-Isolation Fallback` 是现有内核隔离配置的兼容提示，默认不会因此拒绝启动。

## 4. 平台与核分配自动选择

研扬 255H 的逐核核查结果为：CPU 0–5 是 P 核，6–13 是 E 核，14–15 是无 L3 的 LP-E 核，没有超线程。程序通过 `/proc/cpuinfo` 的型号选择已核查配置，不依赖旧内核不可靠的频率或 socket 字段猜核型。ARM 路径读取实际拓扑；下表 AGX 指 12 核 Cortex-A78AE 的已适配布局。

Linux CPU 编号从 0 开始，口头第 15、16 核对应 CPU 14、15。屏蔽末两核后在线集合为 `0-13`；代码检查实际编号与 cpuset，不只检查核总数。

|角色|研扬 255H，16 核在线|研扬 255H，末两核屏蔽|AGX，12 核在线|
|---|---|---|---|
|WBC|2、3|2、3|2|
|IK 求解线程|5，FIFO 40|5，FIFO 40|3，FIFO 40|
|Mapper|4，nice=-5|4，nice=-5|4，nice=-5|
|驱动保留池|6、7，主核7|6、7，主核7|7|
|旁路异步发布|14、15|可用后台 E 核8–13|优先10、11|
|通用后台池|0、1、8–13|0、1、8–13|0、1、5、6、8–11|

该表适用于三种机型，且假设表内关键 CPU 都在线并在 cgroup 允许范围内。研扬仅剩一个 LP-E 可用时，旁路使用该核；两个都不可用时使用后台 E 核子集，与后台共享，不落到 P 核。BIOS 若改变了核型编号布局而不只是关闭末两核，应重新核查映射。

|平台|内核隔离示例|IK 模式|
|---|---|---|
|研扬16核|`isolcpus=2,3,7,14,15`|CPU5 软隔离，WARN，正常启动|
|研扬14核|`isolcpus=2,3,7`|CPU5 软隔离，WARN，正常启动|
|研扬14/16核|`isolcpus=2,3,5,7`|CPU5 硬隔离|
|AGX|`isolcpus=2,7`|CPU3 软隔离，WARN，正常启动|
|AGX|`isolcpus=2,3,7`|CPU3 硬隔离|

无需为了普通软隔离兼容启动改内核参数。只有主动传 `require_ik_hard_isolation:=true` 才要求 IK 必须已隔离。

父入口将新建后台子进程迁出控制、IK、Mapper 和驱动保留核；旁路使用 sidecar 池。该围栏仅覆盖本启动链，**不迁移已经运行的相机、感知服务、外部 ROS master 或 IRQ**。绑核不是物理核心独占，也不等于已达到 500 Hz 实机实时性。

其他 x86 型号仍使用既有 `runtime_partition.json`，不能把任意 x86 当作 255H。`KUAVO_RUNTIME_PROFILE` 可显式指定经过核查的配置；普通部署无需设置。`arm_driver_reserved_cpus` 只作用于 ARM，默认7。`KUAVO_RUNTIME_PARTITION=0` 只用于隔离的离线测试，不是实机排错时绕过调度约束的默认办法。

## 5. 模型对应关系

|机型|私有配置|完整 URDF，相对 kuavo_assets/models|task，相对 humanoid_wheel_interface/config|
|---|---|---|---|
|62|config/model_s62.json|biped_s62/urdf/biped_s62.urdf|kuavo_s62/task.info|
|63|config/model_s63.json|biped_s63/urdf/biped_s63.urdf|kuavo_s63/task.info|
|400062|config/model_s400062.json|biped_s400062/urdf/biped_s400062.urdf|kuavo_s400062/task.info|

400062 有独立模型、手和相机结构，不映射为62。三个版本均在各自完整模型上保留躯干4轴与双臂14轴。400062 另外固定手指关节；手指由对应手部链路控制。

IK、Mapper 和私有控制器拒绝缺失的模型契约、其他版本 URDF 或旧 Drake arm-only URDF，不静默回退。S63 自动使用 `with_ik=false`，不要强制打开旧 `arms_ik_node`。400062 的 S62/S63 J3 增益覆盖仍关闭，沿用其自身设置。

## 6. Pico 与可见仿真

**Pico 替换终端2的 Quest 入口，不同时运行两套输入。** 下列变量按现场网络填写后再执行：

```bash
PICO_IP=192.168.5.240
ROBOT_IP=192.168.5.242
BROADCAST_IP=192.168.5.255
bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel pico_velocity_ik.launch \
  ip_address:="$PICO_IP" robot_ip:="$ROBOT_IP" broadcast_ip:="$BROADCAST_IP"
```

地址只是已有 Pico 默认网段示例，不代表5.19当前网络。机型和手型仍自动读取。

**仿真替换终端1的实机入口**，保留一个 Quest/Pico 输入端，确保两端机型一致：

```bash
bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel load_kuavo_mujoco_velocity_ik.launch
```

仿真默认 WBC，实机默认 RNEA；不要把实机的 `dynamics_backend:=rnea` 参数照搬到既有力矩型 MuJoCo 执行器。仿真应在具备图形会话的机器上可见运行，确认窗口实际打开；仅 SSH 无图形显示时不要将无窗口状态当作正常仿真。不要让仿真与真实控制共享同一套运行中的命令链路。

## 7. 常见问题

|现象|处理|
|---|---|
|`robot model is not configured` / `UNCONFIGURED_ROBOT_VERSION`|检查实际启动用户的配置；使用专属 wrapper。直接 roslaunch 只读取环境，不能替代 wrapper 的持久配置回退和整链围栏。|
|来源为 environment，但机型不是预期值|当前 shell 环境优先；核对保存值与实物，重开终端或清除旧环境后再启动。|
|`conflicting saved ROBOT_VERSION`|检查当前用户 .bashrc 与 /etc/environment，保留一致的唯一机型，勿反复追加不同版本。|
|`nonliteral ROBOT_VERSION`|配置使用62/63/400062字面量；入口不会执行变量展开、命令替换或脚本。|
|`Soft-Isolation Fallback`|上表软隔离配置的正常提示，查看实际IK核与后台池是否符合预期。|
|关键CPU不可用、cpuset不包含关键核|读取 --check 输出，核对online和cgroup限制；不要用关闭分区的方式直接启动实机。|
|FIFO/亲和度权限错误|按上述实机root终端步骤启动，核对实际权限；解析成功不代表FIFO已申请成功。|
|模型或版本不匹配|确认两个终端同一工作区、同一机型；不要使用S62模型代替400062或63。|
|Quest无输入|核对实际Quest IP与输入节点状态；CPU/模型自动识别不会修正网络地址。|
|rospack 指向旧目录|重新打开干净终端，只source正确ROS与目标工作区。|

运行后可查看线程，并按实际 TID 查询允许核与调度策略：

```bash
ps -eLo pid,tid,psr,cls,rtprio,comm
```

`PSR` 只是最近运行的 CPU，不是允许核集合。对已找到的线程使用 `taskset -pc TID` 和 `chrt -p TID` 查询，命令中的 TID 替换为实际线程号；不要增加设置参数。IK应为CPU5（研扬）或CPU3（AGX）及FIFO40。

## 8. 停止与回退

按已有实机停止流程结束动作并保持现场安全状态，再用 Ctrl+C 结束终端2输入链，等待退出后结束终端1控制器。不要同时运行新旧控制器，也不要对所有ROS进程执行无差别kill。

5.19 原有 `/home/lab/lhm/kuavo-ros-control-yyb-test` 和 `/home/lab/lhm/kuavo-ros-control-merge-review` 保留原样。需要回退时，在当前两端完全退出后，重新打开终端，进入此前实际使用的旧目录，source它自己的环境并使用原来的启动命令；不要在新旧devel之间混用。

## 9. 构建、测试与验收边界

2026-10-19已完成：5.19新工作区62包原生构建；自动识别更新后本包增量构建及全部42/42 CTest通过。拓扑测试54项、模型平台矩阵21项、启动识别6项、递归launch7项通过。矩阵包括三机型 × 研扬16核软/硬、研扬14核软/硬、AGX12核软/硬18组组合，另3项模型检查。实际5.19清空环境后的实机/Quest `--nodes` 检查成功读取root配置并选择CPU5，无机器人运行。

以下只用于已经构建好的5.19工作区复核；不是新机器安装步骤：

```bash
cd /home/lab/lhm/kuavo-ros-control-absolute-cpu5-20261019
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
cd build/velocity-ik/wheel_arm_abs_vel
ctest --output-on-failure --timeout 180
```

测试包含隔离ROS master/FSM检查，应在非实机运行期间执行。

新机器应复用已审计的依赖环境与构建流程；不要直接运行整个仓库 ALL 目标，因为共享 kuavo_assets 的 ALL 含安装/配置钩子，需按已有流程显式构建其库目标。不自动安装依赖、不修改公共构建钩子。

2026-10-19 发布候选已合入 dev 基线 02aba90958101c97808f24d6d370e8f0fc8d096a；在研扬 5.19 的独立验证目录原生编译通过，全部 45 项 CTest 通过（包含底盘门控、末端配置及 Quest/Pico 启动解析）。包外文件与该 dev 基线完全一致。编译、单元测试及解析通过不等于实机闭环验收；AGX 原生编译/运行、研扬 14 核实体机器、三机型完整 500Hz 性能与底盘执行仍需对应硬件验证。

末端类型默认自动读取当前工作区 `kuavo_assets/config/kuavo_v<robot_version>/kuavo.json` 的 `EndEffectorType: [左, 右]`，无需填写 `ee_type`。与 dev 控制器使用同一配置；不再按机型硬编码强脑或黑漫。Quest/Pico 均支持强脑 `qiangnao`、黑漫 `heiman`、夹爪 `lejuclaw`；仅黑漫启动 SG100 转发。可选 `ee_type:=...` 显式覆盖输入映射，不改写硬件配置。缺失、未知或左右不同类型会明确报错，不能静默按强脑启动。
