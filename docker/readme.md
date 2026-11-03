## 容器使用说明

### aarch64 / Jammy（AGX Orin 轮臂开发）

Ubuntu 22.04 容器内源码编译 ROS Noetic + Drake 1.51.1，详见 **[aarch64/README.md](aarch64/README.md)**。

```bash
# 构建镜像
bash docker/aarch64/build_jammy_aarch64.sh

# 进入容器（唯一保留在 docker/ 根目录的入口）
bash docker/run_jammy_aarch64.sh

# 容器内首次（各 1–3+ 小时）
bash docker/aarch64/scripts/build_ros_noetic_jammy_aarch64.sh all
bash docker/aarch64/scripts/install_drake_jammy_source_aarch64.sh all
bash docker/aarch64/scripts/catkin_build_jammy_aarch64.sh
```

---

### x86_64 仿真镜像（历史）

#### 1. 安装 docker

```bash
./install_docker.sh
```

#### 2. 构建容器镜像

```bash
./build.sh
```

#### 2. （或者）下载容器镜像

从[这里](https://kuavo.lejurobot.com/kuavo_research_editiion/docker_images/kuavo_opensource_mpc_wbc_img_v1.3.0.tar.gz)下载容器镜像，导入：

```bash
docker load -i kuavo_opensource_mpc_wbc_img_v1.3.0.tar.gz
```

#### 3. 运行容器

```bash
./run.sh
```

GPU 版本：

```bash
./run_with_gpu.sh
```
