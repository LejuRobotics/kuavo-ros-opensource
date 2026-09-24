# Scene 1 交付、安装、采集与回放

## 交付结论

Scene 1 应拆成三类产物，不要把本机完整目录压成一个包，也不要让接收方复现本机 Conda：

| 产物 | 推荐载体 | 内容 |
|---|---|---|
| 源码 | Git 仓库的固定 tag | Scene 1 源码、模型、launch、脚本和本文档 |
| 运行环境 | Docker registry 镜像或 `.tar.zst` 镜像归档 | Ubuntu 20.04、ROS Noetic、MuJoCo、MPC/WBC 依赖 |
| 数据 | 对象存储/网盘/移动硬盘；必要时 Git LFS | 已验收的 `.bag` 和 SHA-256 清单 |

正式运行链只有“宿主机 + Docker”。宿主机 Conda 仅用于开发期离线查看，不是交付依赖。

接收方运行 `scene1.sh` 时，当前 Git checkout 会挂载到容器 `/root/kuavo_ws`。因此源码只保留一份；容器使用的正是接收方 checkout 的代码。

## 一、交付方准备产物

### 1. 发布源码

当前开发仓库有较多未提交和无关改动时，不要执行 `git add -A`。先在开发机仓库根目录检查：

```bash
cd /home/zjy/codex/mujoko/kuavo-ros-control
git status --short
git diff --stat
```

应建立专用 release branch，只提交 Scene 1 实际依赖的源码、模型、ROS service、`scene1.sh` 和本文档。不得提交 `build/`、`devel/`、`logs/`、本机 Conda、运行日志或普通 bag。

发布到接收方能访问的 Git 服务。下面的 URL、分支和版本号由交付人选择，命令不会自动替你做这个选择：

```bash
git remote add delivery <DELIVERY_GIT_URL>
git push delivery <LOCAL_RELEASE_BRANCH>:scene1-release
git tag -a scene1-v1.0.0 <LOCAL_RELEASE_BRANCH> -m "Scene 1 delivery v1.0.0"
git push delivery scene1-v1.0.0
```

交付时给接收方 Git URL 和固定 tag，不能只说“拉最新分支”。固定 tag 才能把代码、镜像和 bag 的版本对应起来。

### 2. 发布 Docker 镜像

当前已验证的镜像名是：

```text
kuavo_opensource_mpc_wbc_img:0.6.1-gpu
```

当前本机对应 image ID 是
`sha256:295544461cc4e1e3ea465708475ba567a056aab2a0e80533fb9c29384db49d5a`。
交付前应再次用 `docker image inspect` 记录实际 ID；如果推到 registry，还要记录不可变的 repo digest。

不要直接 `docker commit kuavo-mpc-wbc` 作为交付环境。该开发容器的可写层包含大量 ROS 临时 bag、日志、缓存和 NVIDIA 运行时注入文件；`docker save` 下方指定的干净版本镜像即可，源码由 Git 单独提供。

二选一：

- 有团队 Docker registry：给镜像增加正式版本 tag 后 `docker push`。适合多人长期使用。
- 没有 registry：导出压缩镜像和校验文件。适合一次性交付。

在安装了 `zstd` 的交付机执行：

```bash
docker save kuavo_opensource_mpc_wbc_img:0.6.1-gpu \
  | zstd -T0 -10 -o scene1-runtime-0.6.1-gpu.tar.zst
sha256sum scene1-runtime-0.6.1-gpu.tar.zst \
  > scene1-runtime-0.6.1-gpu.tar.zst.sha256
```

镜像展开后约 20 GB，再加源码构建产物和 bag，接收机建议至少预留 50 GB 可用空间。

### 3. 发布 bag

`.bag` 通常上百 MB，不应直接提交进普通 Git 历史。把通过验收的 bag 放到独立目录并生成清单：

```bash
sha256sum *.bag > SHA256SUMS
```

如果组织已有 Git LFS，也可以建立独立数据仓库并跟踪 `*.bag`；不要把 bag 混入源码仓库的普通 Git 对象。

最终交付清单至少包含：

```text
Scene 1 Git URL + scene1-v1.0.0
Docker image URL，或 scene1-runtime-0.6.1-gpu.tar.zst + sha256
一个已验收的样例 bag + SHA256SUMS
```

## 二、接收方首次安装

以下命令均在接收方 Ubuntu x86_64 宿主机终端执行，不进入 Conda。

### 1. 安装宿主机依赖

安装 Docker Engine、Git、`zstd`。将当前用户加入 `docker` group 后需要重新登录，使 `docker info` 不依赖 `sudo`。可视化运行还需要 X11；GPU 模式需要 NVIDIA 驱动和 NVIDIA Container Toolkit。

检查：

```bash
docker info
git --version
zstd --version
```

### 2. 拉取固定版本源码

```bash
git clone <DELIVERY_GIT_URL> kuavo-ros-control
cd kuavo-ros-control
git checkout scene1-v1.0.0
```

后续所有 `./scene1.sh ...` 命令都在这个仓库根目录执行。

### 3. 获得运行镜像

如果镜像在 registry：

```bash
docker pull <REGISTRY>/scene1-runtime:1.0.0
export SCENE1_IMAGE=<REGISTRY>/scene1-runtime:1.0.0
```

如果收到镜像归档：

```bash
sha256sum -c scene1-runtime-0.6.1-gpu.tar.zst.sha256
zstd -dc scene1-runtime-0.6.1-gpu.tar.zst | docker load
```

归档恢复出的默认镜像名是 `kuavo_opensource_mpc_wbc_img:0.6.1-gpu`，无需设置 `SCENE1_IMAGE`。

### 4. 创建容器并编译

按当前验证配置（NVIDIA GPU、host network、源码 bind mount）执行：

```bash
./scene1.sh setup
./scene1.sh build
```

`setup` 只在首次创建容器时运行；已有同名容器时会核对镜像和源码挂载，发现不一致会停止并报告，不会删除或覆盖容器。

`build` 首次必须运行。它在容器内加载 ROS Noetic，并编译 `kuavo_msgs`、`mujoco_cpp`、`humanoid_controllers` 和 `data_challenge_simulator` 及其依赖。脚本会显式复用镜像内 Drake 自带的 `pybind11`，首次构建不需要从 GitHub 下载该依赖。生成的 `build/`、`devel/`、`logs/` 位于宿主机 checkout，但不会进入 Git。

如果接收机明确只运行无窗口模式并且没有 NVIDIA Container Toolkit，可以在首次创建容器时选择不请求 GPU：

```bash
SCENE1_GPU=0 ./scene1.sh setup
./scene1.sh build
```

这是运行资源选择，不会改变源码和数据格式；可视化与性能取决于该机器的图形环境。

## 三、使用 Scene 1

### 1. 先跑一次不录包实验

有 MuJoCo 窗口：

```bash
./scene1.sh task 1
```

无窗口：

```bash
./scene1.sh task-headless 1
```

末尾的 `1` 是随机种子。相同代码和环境下用同一种子初始化同一场景。

### 2. 采集 rosbag

从 seed 1 开始采集 5 轮：

```bash
./scene1.sh collect 5 1
```

采集默认无窗口。只有任务判定成功的轮次保留 bag。输出在：

```text
src/data_challenge_simulator/examples/bags/run_<time>/
```

每个文件名包含 seed，例如 `data_round_0000001.bag`。

### 3. 检查已有 bag

先把收到的 bag 放到当前 Git checkout 内，例如：

```text
delivery_bags/data_round_0000001.bag
```

然后执行：

```bash
./scene1.sh verify delivery_bags/data_round_0000001.bag
```

`verify` 不启动仿真。它检查 Scene 1 必需的控制/结果 topics，并用 bag 最后一帧检查两个圆柱和拨杆结果。

### 4. 跑已有 bag

可视化回放：

```bash
./scene1.sh replay delivery_bags/data_round_0000001.bag
```

无窗口回放并检查最终状态：

```bash
./scene1.sh replay-headless delivery_bags/data_round_0000001.bag
```

脚本默认从 `data_round_XXXXXXX.bag` 文件名推断初始化 seed。如果 bag 被改名，显式给 seed：

```bash
./scene1.sh replay delivery_bags/example.bag 1
```

回放只重新发布三类控制 topic：双臂轨迹、左手命令、右手命令；物体 pose 等结果 topic 不会灌回仿真。回放结束后重新读取仿真最终状态并验收。

## 四、日常操作与代码更新

查看容器和 Scene 1 进程：

```bash
./scene1.sh status
```

任务异常退出后，只清理 Scene 1 相关进程：

```bash
./scene1.sh stop
```

停止容器以释放资源：

```bash
./scene1.sh container-stop
```

下次执行 `task`、`collect` 或 `replay` 时脚本会自动启动原容器。

源码更新规则：

- 只改 Python 或 XML：bind mount 会立即生效，不需要编译。
- 改 C++、ROS message/service 或 CMake：先 `./scene1.sh stop`，再 `./scene1.sh build`。
- 切换 Git tag 后：重新执行 `./scene1.sh build`，并使用与该 tag 对应的镜像和 bag。

## 五、交付边界和常见失败

- `scene1.sh` 不会自动下载未知镜像、删除容器或替换环境。
- 同名容器若指向另一份源码，脚本会拒绝运行。可通过设置新的容器名并重新 setup 隔离：

  ```bash
  SCENE1_CONTAINER=kuavo-scene1-v1 ./scene1.sh setup
  ```

  后续命令也要带相同的 `SCENE1_CONTAINER`，或在当前 shell 中 `export`。

- bag 必须位于当前 checkout 内，否则容器无法通过既定 bind mount 读取。
- `verify` 通过表示 bag 内容满足当前验收规则；`replay` 通过才表示该 bag 在当前代码、镜像和 seed 下可复现最终状态。
- 发布源码时应记录 Git tag、Docker image digest 和 bag SHA-256。只给分支名或可变镜像 tag，之后无法确定接收方实际跑的是哪一版。
