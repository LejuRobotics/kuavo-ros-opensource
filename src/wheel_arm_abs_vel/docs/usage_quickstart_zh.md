# 绝对式轮臂 IK：拉取、编译、启动与仿真

> 2026-10-22 更新：适用于 Draft MR !4139 候选分支；源码基线为 `b26d5d18e`（包含躯干启动授权撤销修复）。当前未合入 dev。本文与[参数和控制参考](usage_reference_zh.md)配套使用；验收边界见参考文档第15节。

本文按当前草稿分支的实际行为编写。所有操作命令均在正文展开，不依赖额外的文档辅助脚本。完整复制每个代码块，包括 Bash 执行块的结束标记；使用 Bash 交互终端。首次部署按第1—4节执行，已完成部署和编译则从第5或第6节启动。参数、依赖解释及排错见第二份文档。

## 1. 必要条件

- 真机：Ubuntu 20.04 x86_64、sudo 权限、该机器已配置的底层驱动、底盘/CAN通信、急停、关节零位和末端类型。直接在宿主机安装、编译和启动，不用容器。
- 电脑仿真：Ubuntu 22.04/24.04 x86_64、已解锁的图形桌面；建议32 GB内存及70 GB可用磁盘。首次按第1.1—1.3节创建并进入容器，后续源码、依赖和运行都在同一容器内完成。真机跳过这些小节。
- 拥有 LejuHub 仓库及子模块读取权限，能访问 APT、ROS、Drake 和 Python 软件源。配置和编译期间，现场安全停止机器人控制。
- 机型身份使用启动Shell已有的ROBOT_VERSION环境变量；启动时读取对应kuavo.json并检测CPU。已有机器人无需重新配置；仅缺失时按第二份第13.5节设置一次，日常无需手填机型、末端和CPU参数。
- Quest采用手动填写头显IP直连，头显与电脑网络互通；不使用Quest自动广播发现。
- Pico 需配套App、全身模式、脚踝追踪器及完成校准，与电脑处在允许UDP广播的同一局域网；Quest/Pico接收器只启动一个。

已有候选曾完成干净容器62个目标包构建、协议测试和可视仿真。冲突合并与本次权限修复后的源码通过 Release 构建及62/62 CTest；头显实测记录、合成输入仿真和完整版本验收分开记录，不能把旧版首装或部分实测当成本版全部实机通过。音频、压力继电器、灯带不纳入本次配置验收。

### 1.1 仿真电脑宿主：Docker准备

仅电脑仿真执行。已有Docker时保留现有安装；冲突包需人工处理。

```bash
bash --noprofile --norc -e <<'ABS_DOCKER_INSTALL'
source /etc/os-release
test "$ID" = ubuntu
case "$VERSION_ID" in 22.04|24.04) ;; *) echo '本宿主安装步骤要求 Ubuntu 22.04/24.04'; exit 1;; esac
test "$(uname -m)" = x86_64
sudo apt-get update
sudo apt-get install -y ca-certificates curl xauth
if ! command -v docker >/dev/null 2>&1; then
  for package in docker.io docker-compose docker-compose-v2 docker-doc podman-docker containerd runc; do
    if dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -qx 'install ok installed'; then
      echo "发现 $package，请先按 Docker 官方迁移说明处理后重试"; exit 1
    fi
  done
  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
  sudo chmod 644 /etc/apt/keyrings/docker.asc
  printf 'deb [arch=%s signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu %s stable\n' "$(dpkg --print-architecture)" "$VERSION_CODENAME" | sudo tee /etc/apt/sources.list.d/docker-absolute.list
  sudo apt-get update
  sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
  sudo systemctl enable --now docker
fi
sudo docker info >/dev/null
sudo docker version
ABS_DOCKER_INSTALL
```


### 1.2 仿真电脑宿主：创建容器

首次执行一次。Dockerfile内容已完整列出；无需另取Dockerfile。

```bash
bash --noprofile --norc -e <<'SIM_CREATE'
test "$(id -u)" -ne 0 || { echo '请以普通桌面用户执行'; exit 1; }
test -n "${DISPLAY:-}" || { echo '请在已解锁的图形桌面终端执行'; exit 1; }
test -d /tmp/.X11-unix
if sudo docker container inspect kuavo-absolute-sim >/dev/null 2>&1; then
  echo '容器已存在，执行第1.3节进入，勿重复创建'; exit 1
fi
sim_build_dir=$(mktemp -d /tmp/kuavo-sim-image.XXXXXX)
cat > "$sim_build_dir/Dockerfile" <<'SIM_DOCKERFILE'
FROM public.ecr.aws/docker/library/ubuntu:20.04
ARG SIM_UID=1000
ARG SIM_GID=1000
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends sudo ca-certificates git curl wget gnupg lsb-release iproute2 tzdata xauth x11-utils mesa-utils libgl1-mesa-dri libgl1-mesa-glx && rm -rf /var/lib/apt/lists/*
RUN test "$SIM_UID" -ne 0 && (getent group "$SIM_GID" || groupadd --gid "$SIM_GID" sim) && useradd --uid "$SIM_UID" --gid "$SIM_GID" --create-home --shell /bin/bash sim && printf 'sim ALL=(ALL) NOPASSWD: ALL\n' > /etc/sudoers.d/kuavo-sim && chmod 0440 /etc/sudoers.d/kuavo-sim
USER sim
WORKDIR /home/sim
CMD ["sleep", "infinity"]
SIM_DOCKERFILE
sudo docker build --build-arg SIM_UID="$(id -u)" --build-arg SIM_GID="$(id -g)" \
  -t kuavo-absolute-sim-base:20.04 "$sim_build_dir"
install -d -m 700 "$HOME/.local/share/kuavo-absolute-sim"
xauth_dir="$HOME/.local/share/kuavo-absolute-sim"
xauth nlist "$DISPLAY" > "$xauth_dir/current-cookie"
test -s "$xauth_dir/current-cookie" || { echo '未找到 DISPLAY 的 X11 cookie，请检查当前桌面 XAUTHORITY'; exit 1; }
touch "$xauth_dir/Xauthority"
chmod 600 "$xauth_dir/Xauthority"
sed 's/^..../ffff/' "$xauth_dir/current-cookie" | xauth -f "$xauth_dir/Xauthority" nmerge -
sudo docker run -d --name kuavo-absolute-sim --init   --network host --ipc private --cap-add SYS_NICE --ulimit rtprio=99 --shm-size 1g --cpus 8 --memory 24g   -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/kuavo-xauth/Xauthority   -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1   --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly   --mount "type=bind,src=$xauth_dir,dst=/tmp/kuavo-xauth,readonly"   --mount type=volume,src=kuavo-absolute-sim-home,dst=/home/sim   kuavo-absolute-sim-base:20.04 sleep infinity
sudo docker exec kuavo-absolute-sim xdpyinfo >/dev/null
sudo docker exec kuavo-absolute-sim glxinfo -B
SIM_CREATE
```


### 1.3 仿真电脑宿主：进入已有容器

每个仿真终端都先执行本块；会刷新当前桌面图形授权。进入后再执行后续步骤。

```bash
bash --noprofile --norc -e <<'SIM_DISPLAY'
test -n "${DISPLAY:-}"
xauth_dir="$HOME/.local/share/kuavo-absolute-sim"
test -d "$xauth_dir"
xauth nlist "$DISPLAY" > "$xauth_dir/current-cookie"
test -s "$xauth_dir/current-cookie"
sed 's/^..../ffff/' "$xauth_dir/current-cookie" | xauth -f "$xauth_dir/Xauthority" nmerge -
sudo docker start kuavo-absolute-sim >/dev/null
sudo docker exec -e DISPLAY="$DISPLAY" kuavo-absolute-sim xdpyinfo >/dev/null
SIM_DISPLAY
sudo docker exec -it -e DISPLAY="$DISPLAY" --user sim --workdir /home/sim \
  kuavo-absolute-sim bash -i
```


## 2. 拉取草稿候选分支

真机在宿主机、电脑仿真在容器内执行。已有本版代码不要重复clone。

```bash
bash --noprofile --norc -e <<'CLONE_ABSOLUTE'
set -e
test "$(id -u)" -ne 0 || { echo "请用普通用户执行，脚本内部会调用 sudo。"; exit 1; }
source /etc/os-release
[ "$ID:$VERSION_ID:$(uname -m)" = "ubuntu:20.04:x86_64" ] || { echo "要求 Ubuntu 20.04 x86_64；电脑仿真请先进入容器。"; exit 1; }
if ! command -v git >/dev/null; then
  sudo apt-get update
  sudo apt-get install -y git ca-certificates
fi
test ! -e "$HOME/kuavo-ros-control-absolute" || { echo "目录已存在，不覆盖。已有工作区先按第二份更新说明核对，勿覆盖。"; exit 1; }
cd "$HOME"
git clone --branch codex/abs-pico-startup-20261021 --single-branch https://www.lejuhub.com/highlydynamic/kuavo-ros-control.git kuavo-ros-control-absolute
cd kuavo-ros-control-absolute
git submodule update --init --recursive
git rev-parse HEAD
CLONE_ABSOLUTE
```


## 3. 首次安装依赖

全新系统执行；已有部署只有在第二份文档的依赖和运行前检查通过后才能跳过安装。APT如提示Drake签名过期，先执行第二份的密钥修复命令。

```bash
bash --noprofile --norc -e <<'ABS_BASIC_TOOLS'
sudo env DEBIAN_FRONTEND=noninteractive apt-get update
sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y \
  sudo git ca-certificates curl wget gnupg lsb-release iproute2 tzdata
ABS_BASIC_TOOLS
```


### 3.1 安装核心依赖

调用已拉取仓库内的实际安装函数，跳过终端美化和完整main；此步骤会配置系统软件源。

```bash
bash --noprofile --norc -e <<'ABS_STEP_04'
  set -e
  cd ~/kuavo-ros-control-absolute/scripts
  export DEBIAN_FRONTEND=noninteractive
  sudo() { command sudo --preserve-env=DEBIAN_FRONTEND "$@"; }
  # 只加载函数定义，不调用脚本末尾的 main。
  source <(sed '/^main "\$@"$/d' kuavo_environment_setup.sh)
  check_system
  update_system
  configure_sources
  install_ros
  install_kuavo_libs
  install_drake
  install_ocs2_deps
ABS_STEP_04
```


### 3.2 补齐编译和Python依赖

保留完整依赖，等待pip check通过。

```bash
bash --noprofile --norc -e <<'ABS_STEP_05'
  set -e
  cd ~/kuavo-ros-control-absolute
  sudo apt-get update
  sudo apt-get install -y \
    build-essential cmake pkg-config git python3-pip python3-catkin-tools \
    python3-rosdep python3-testresources python3-numpy python3-scipy python3-yaml \
    python3-netifaces python3-psutil python3-zmq \
    libudev-dev libusb-1.0-0-dev libmodbus-dev \
    libprotobuf-dev protobuf-compiler libprotobuf-c-dev \
    libgeographic-dev libglfw3-dev libglew-dev libjsoncpp-dev \
    nlohmann-json3-dev libyaml-cpp-dev liblcm-dev liblcm-bin libgl1-mesa-dri libgl1-mesa-glx
  sudo -v
  bash src/kuavo_assets/scripts/install_deps_packages.sh
  sudo ldconfig
  sudo /usr/bin/python3 -m pip install \
    --index-url https://pypi.tuna.tsinghua.edu.cn/simple pip==24.3.1
  sudo /usr/bin/python3 -m pip install \
    --index-url https://pypi.tuna.tsinghua.edu.cn/simple --ignore-installed \
    -r src/manipulation_nodes/noitom_hi5_hand_udp_python/requirements.txt \
    numpy-quaternion==2023.0.4 Pillow==9.5.0
  sudo /usr/bin/python3 -m pip check
ABS_STEP_05
```


已有机器人不需要重新配置机型。只有新机器或全新仿真容器缺少ROBOT_VERSION时，才执行第二份第13.5节的一次性配置；已配置环境直接继续编译。

## 4. 编译

先消息、再资产、最后完整运行依赖。等待BUILD_PASS；随后执行第二份第12.2节的完整运行前检查，确认PREFLIGHT_PASS后启动。

```bash
bash --noprofile --norc -e <<'BUILD_ABSOLUTE'
set -e
test "$(id -u)" -ne 0 || { echo "请用普通用户执行，脚本内部会调用 sudo。"; exit 1; }
source /etc/os-release
[ "$ID:$VERSION_ID:$(uname -m)" = "ubuntu:20.04:x86_64" ] || { echo "要求 Ubuntu 20.04 x86_64；电脑仿真请先进入容器。"; exit 1; }
python3 - <<'NO_ACTIVE_MASTER'
import socket
with socket.socket() as s:
    s.settimeout(1)
    if s.connect_ex(("127.0.0.1",11311)) == 0:
        raise SystemExit("有活动 ROS master；请现场安全停止控制后再配置或编译。")
NO_ACTIVE_MASTER

cd ~/kuavo-ros-control-absolute
source /opt/ros/noetic/setup.bash
robot_model=$(/usr/bin/python3 src/wheel_arm_abs_vel/scripts/absolute_robot_version.py)
export ROBOT_VERSION="$robot_model"
export ROS_PACKAGE_PATH="$PWD/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"

catkin config --profile velocity-ik --extend /opt/ros/noetic \
  --build-space build/velocity-ik --devel-space devel --log-space logs/velocity-ik \
  -j4 -p2 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_ENABLE_TESTING=ON \
  -Ddrake_DIR=/opt/drake/lib/cmake/drake \
  -DCMAKE_ASM_COMPILER=/usr/bin/as \
  -DKUAVO_SOLVER_BUILD_PYTHON=OFF
catkin profile set velocity-ik

  # 首次先初始化 Catkin 并生成消息，不能从带自定义 make 目标的资产包开始。
  catkin build kuavo_msgs --no-deps -j4 -p2 --no-status

  catkin build kuavo_assets --no-deps -j4 -p2 --no-status \
    --make-args kuavo_assets

  catkin build --no-deps -j4 -p2 --no-status \
    blasfeo_catkin g11_controller_node h12pro_controller_node \
    handcontrollerdemorosnode hardware_node hpipm_catkin \
    humanoid_common humanoid_controllers humanoid_estimation \
    humanoid_interface humanoid_interface_drake humanoid_interface_ros \
    humanoid_plan_arm_trajectory humanoid_wbc humanoid_wheel_interface \
    humanoid_wheel_interface_ros humanoid_wheel_wbc image_processing joy \
    kuavo_audio_player kuavo_audio_receiver kuavo_common kuavo_estimation \
    kuavo_led_controller kuavo_mocap_gmr kuavo_monitor kuavo_msgs \
    kuavo_pico_gmr kuavo_ros_interfaces kuavo_sdk kuavo_solver lcm_msg \
    leju_mobile_base_msgs leju_utils mobile_manipulator_controllers \
    motion_capture_ik mujoco_cpp noitom_hi5_hand_udp_python \
    ocs2_centroidal_model ocs2_core ocs2_ddp ocs2_ipm \
    ocs2_mobile_manipulator ocs2_mobile_manipulator_ros ocs2_mpc \
    ocs2_msgs ocs2_oc ocs2_pinocchio_interface ocs2_qp_solver \
    ocs2_robotic_assets ocs2_robotic_tools ocs2_ros_interfaces \
    ocs2_self_collision ocs2_self_collision_visualization ocs2_sqp \
    ocs2_thirdparty qpoases_catkin realsense_sim_metadata \
    robot_localization ruckig_catkin teach_pendant wheel_arm_abs_vel

git rev-parse HEAD
echo BUILD_PASS
BUILD_ABSOLUTE
```


## 5. 真机双终端启动

以下统一使用第2节创建的 `~/kuavo-ros-control-absolute`，已有测试目录只替换 `cd` 路径，不另建或覆盖工作区。仅在已完成部署和编译的真机宿主执行；已有环境直接从本节启动，不重复安装或配置机型。每个终端按顺序执行对应代码块；sudo行进入root交互Shell后，余下命令在该Shell中执行，默认参数省略；不用再手动读取JSON、选择Pico网卡或填写CPU编号。控制器默认底盘通信地址为192.168.26.1，非标准网络按第二份第14.3节覆盖ros_ip。

### 5.1 终端A：控制器

```bash
cd ~/kuavo-ros-control-absolute
sudo --preserve-env=ROBOT_VERSION bash --noprofile --norc
source /opt/ros/noetic/setup.bash
source devel/setup.bash

bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel load_kuavo_real_velocity_ik.launch
```

需要开启底盘时，在控制器启动命令末尾追加 `base_locked:=false`；默认锁定底盘。

### 5.2 终端B：Pico输入

先启动控制器，再执行此块。程序自动在启用的IPv4广播网卡上发布机器人地址，Pico端搜索并连接即可，无需填写头显IP。

```bash
cd ~/kuavo-ros-control-absolute
sudo --preserve-env=ROBOT_VERSION bash --noprofile --norc
source /opt/ros/noetic/setup.bash
source devel/setup.bash

bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel pico_velocity_ik.launch
```

### 5.3 终端B：Quest输入

使用Quest时执行此块，不同时启动Pico。本次 Quest 头显 IP 为 `192.168.5.24`（不是机器人 IP）；采用指定IP直连，不启动自动广播发现。

```bash
cd ~/kuavo-ros-control-absolute
sudo --preserve-env=ROBOT_VERSION bash --noprofile --norc
source /opt/ros/noetic/setup.bash
source devel/setup.bash

bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel quest_velocity_ik.launch \
  ip_address:=192.168.5.24
```

### 5.4 初始化与按键

按控制器提示在终端 A（第一个控制器终端）按 `o` 完成初始化。终端 B 的 `enable_terminal_start` 默认 `false`，无需再次确认。初始化请求受理与准备动作完成是两个阶段，按日志确认完成后再遥操作。

| 操作 | 当前行为 |
| --- | --- |
| 首次双前扳机解锁 | 两侧超过 0.5，可错开按下；等待初始化、输入和执行握手满足后跟随 |
| 默认躯干 | 正常启动后首次有效跟随开启一次；启动等待本身不取消默认开启 |
| Quest 触摸 X+Y，再新按 B | 躯干开关；触摸 X/Y，不按下 X/Y；持续按 B 不重复切换 |
| Pico RT+Y | 躯干开关，先 RT 或先 Y 都可；组合消费末端操作，松开后可再次切换 |
| X+A | 复位/跟随切换；复位中再次按下可中断复位并恢复跟随 |
| X+B | 固定/恢复双臂关节；保留躯干会话，持续按住也不锁底盘；复位中不响应 |
| X+Y | 原有软件停止；不替代硬件急停 |
| Pico 单独长按 X/A | 初始化后分别切换左/右拇指；至少 0.5 秒，左右扳机均小于 0.5 |

主动关闭躯干后不会自动重开。复位、停止、故障、无效输入或已建立输入流过期会撤销权限并取消尚未消费的默认开启，即使发生在首次跟随前也一样；恢复后需显式使用躯干组合键。正常首次握手等待与真正撤销权限分开处理。

底盘默认 `base_locked:=true`。需要驾驶时，仅在终端 A 的真机启动命令末尾加 `base_locked:=false`；初始化完成、底盘本地 ON/使能满足、手柄新鲜且摇杆回中后可独立驾驶，无需先解锁双臂。急停、失联、反馈过期仍会停止底盘，恢复后先回中。仿真底盘指令通道通过不等于真实底盘位移验收。

头控默认 `head_control_mode:=vr_follow`，由控制器侧独立节点处理；不默认跟手。需要跟手才显式使用 `head_control_mode:=track_hand`。Chest→Head 输入受身体追踪影响的问题仍在分析，不列为已修复。

## 6. 电脑容器仿真

仅在第1节创建的仿真容器内执行；两个终端均通过第1.3节进入同一个容器。不要在真机宿主复制本节命令。容器保留图形环境，MuJoCo窗口必须在桌面可见。仿真关闭平台CPU分区，不改变真机默认策略。

仿真与实机使用同一机型读取规则：默认读取当前启动环境的 `ROBOT_VERSION`（62、63、400062），自动选择对应 `kuavo.json`、URDF、控制配置和 MuJoCo 场景。仅临时切换机型时传 `robot_version:=`；控制器与输入终端应使用相同机型。未配置或不支持的机型明确报错，不回退到 S62。S63 保持关闭旧版 `with_ik`，使用独立绝对式 IK。

### 6.1 终端A：MuJoCo控制器

```bash
cd ~/kuavo-ros-control-absolute
sudo --preserve-env=DISPLAY,XAUTHORITY,ROBOT_VERSION bash --noprofile --norc
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export KUAVO_RUNTIME_PARTITION=0

bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel load_kuavo_mujoco_velocity_ik.launch visualize_humanoid:=true
```

### 6.2 终端B：Pico输入

```bash
cd ~/kuavo-ros-control-absolute
sudo --preserve-env=DISPLAY,XAUTHORITY,ROBOT_VERSION bash --noprofile --norc
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export KUAVO_RUNTIME_PARTITION=0

bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel pico_velocity_ik.launch
```

### 6.3 终端B：Quest输入

与Pico二选一。本次 Quest 头显 IP 为 `192.168.5.24`；采用指定IP直连，不启动自动广播发现。

```bash
cd ~/kuavo-ros-control-absolute
sudo --preserve-env=DISPLAY,XAUTHORITY,ROBOT_VERSION bash --noprofile --norc
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export KUAVO_RUNTIME_PARTITION=0

bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel quest_velocity_ik.launch \
  ip_address:=192.168.5.24
```

## 7. 退出

先在输入终端按Ctrl+C，等待退出，再在控制器终端按Ctrl+C。真机退出前按现场流程支撑机器人。绝对式专用仿真退出回收已修复并有历史复验；确认出现 `[AbsoluteSimulation] clean exit` 并检查进程退出，异常时保存日志。

电脑仿真两个终端先输入exit退出root子Shell，再输入exit退出容器返回宿主机，确认容器内ROS进程退出后停止容器，保留环境与编译产物：

```bash
if sudo docker top kuavo-absolute-sim -eo args | grep -E "[r]oslaunch|[r]osmaster" >/dev/null; then
  echo "ROS仍在运行，请先正常退出输入与控制器。"
else
  sudo docker stop kuavo-absolute-sim
fi
```
