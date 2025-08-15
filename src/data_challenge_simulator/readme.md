````markdown
# Kuavo Data Challenge Simulator - 简易安装教程

> ⚠️ **通用安装教程请参考 main branch**

## 📦 1. 克隆代码仓库
```bash
git clone <repo_url>
cd <repo_name>
git checkout kuavo_data_challenge_simulator
````

## 🐳 2. 启动 Docker 环境

```bash
cd docker
./run_with_gpu.sh
```

## ⚙️ 3. 设置机器人版本环境变量

```bash
export ROBOT_VERSION=45
```

## 🔑 4. 添加密钥

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys AD19BAB3CBF125EA
```

## 📥 5. 安装缺失依赖

```bash
sudo apt-get update
sudo apt-get install ros-noetic-geographic-msgs
```

## 🛠 6. 编译工程

```bash
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release
source installed/setup.zsh
catkin build humanoid_controllers data_challenge_simulator
source devel/setup.zsh
```

## 🔧 7. 安装 SDK

```bash
cd src/kuavo_humanoid_sdk
./install.sh
```

## 🚀 8. 运行 MuJoCo 仿真

```bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch
```

等待机器人加载完成后退出。

## ▶️ 9. 运行示例

```bash
cd src/data_challenge_simulator/example
python3 example.py
```

根据提示选择任务和执行方式，若能正常运行则说明环境配置成功 ✅


