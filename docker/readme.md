## 容器使用说明

#### 1. 安装docker
- 提供了一个方便的脚本来安装docker，只需要运行以下命令即可：
```bash
./install_docker.sh
```

#### 2. 构建容器镜像
- 根据Dockerfile文件构建容器镜像，运行以下命令：

```bash
./build.sh
```

#### 2. (或者)下载容器镜像
- 从[这里](https://kuavo.lejurobot.com/docker_images/kuavo_opensource_mpc_wbc_img_v0.6.1.tar.gz)下载容器镜像
- 导入镜像：
```bash
docker load -i kuavo_opensource_mpc_wbc_img_v0.6.1.tar.gz
```

#### 3. 运行容器
- 运行容器需要配置一些环境变量，挂载目录等，所以提供了一些运行脚本方便使用。
- 普通运行(cpu)，没有GPU或者没有配置好`nvidia-container-toolkit`的机器，运行以下命令：
```bash
./run.sh
```

- 运行GPU版本，需要配置好`nvidia-container-toolkit`和`nvidia-runtime`等环境变量，可以在带GPU的宿主机上mujoco、gazebo等仿真更流畅
```bash
./run_with_gpu.sh
```

