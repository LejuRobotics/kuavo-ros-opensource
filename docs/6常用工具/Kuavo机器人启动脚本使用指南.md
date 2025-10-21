# KUAVO 机器人启动脚本使用指南

本文档介绍 KUAVO 人形机器人系统的启动脚本使用方法，包括交互式启动界面、系统配置工具和硬件检测功能。

## 概述

KUAVO 机器人系统提供两种主要的部署方式：

1. **源码部署**: 直接从 Git 仓库克隆源码进行开发
2. **.deb 包部署**: 通过预编译的 Debian 包进行安装

本文档主要针对 **.deb 包部署** 的用户，介绍如何使用系统提供的 `leju` 命令来管理 KUAVO 机器人。

### .deb 包部署优势

- **简化安装**: 一键安装，无需手动配置环境
- **版本管理**: 清晰的版本控制和依赖管理
- **系统集成**: 与系统包管理器完美集成
- **自动更新**: 支持 apt 系统更新机制
- **标准化**: 符合 Debian 包管理标准
- **完整性**: 包含完整的工作空间和启动脚本

## 系统要求

- **操作系统**: Ubuntu 20.04 
- **用户权限**: 必须使用 root 用户权限运行
- **工作空间**: `/opt/ros/leju`（由 .deb 包自动安装）
- **机器人版本**: 需要设置 `ROBOT_VERSION` 环境变量

## 安装和准备

### 1. .deb 包安装

#### 方法一：快速安装最新正式版（推荐）

```bash
wget https://kuavo.lejurobot.com/packages/ros-leju-workspace_latest.deb && sudo dpkg -i ros-leju-workspace_latest.deb && sudo apt-get install -f -y
```

#### 方法二：安装指定版本

将 `${VERSION}` 替换为实际版本号（如 `v1.2.3`）：

```bash
wget https://kuavo.lejurobot.com/packages/ros-leju-workspace_${VERSION}.deb && sudo dpkg -i ros-leju-workspace_${VERSION}.deb && sudo apt-get install -f -y
```

#### 方法三：安装开发版本

将 `${VERSION}` 和 `${BRANCH}` 替换为实际值（例如：`v1.2.3` 和 `dev`）：

```bash
wget https://kuavo.lejurobot.com/packages/ros-leju-workspace_${VERSION}-${BRANCH}.deb && sudo dpkg -i ros-leju-workspace_${VERSION}-${BRANCH}.deb && sudo apt-get install -f -y
```

#### 版本命名规则

- **正式版**（master分支）：`ros-leju-workspace_${VERSION}.deb`
  - 例如：`ros-leju-workspace_v1.2.3.deb`
  - 最新版链接：`ros-leju-workspace_latest.deb`

- **开发版**（其他分支）：`ros-leju-workspace_${VERSION}-${BRANCH}.deb`
  - 例如：`ros-leju-workspace_v1.2.3-dev.deb`
  - 例如：`ros-leju-workspace_v1.2.3-opensource.deb`

安装完成后，系统会自动：
- 将工作空间安装到 `/opt/ros/leju`
- 将启动脚本复制到 `/usr/bin/leju`
- 设置必要的权限


### 2. 验证安装

验证 .deb 包安装是否成功：

```bash
# 检查工作空间
ls -la /opt/ros/leju

# 检查启动脚本
which leju

# 检查脚本权限
ls -la /usr/bin/leju
```

## 使用启动脚本

### 基本启动

使用 .deb 包安装后，可以直接通过 `leju` 命令启动：

```bash
# 以 root 权限运行
sudo su
leju 
```

### 源码部署用户

如果您使用的是源码部署，可以通过以下方式启动：

```bash
# 进入脚本目录
cd ~/kuavo-ros-opensource

# 以 root 权限运行
sudo ./scripts/leju_start.sh
```

### 主菜单选项

启动脚本后，您将看到以下主菜单：

```
==========================================
      KUAVO机器人启动系统
==========================================
当前机器人版本: 49

请选择启动方式：
1) 启动仿真机器人
2) 启动实物机器人
3) 系统配置工具
4) 硬件检测工具
5) 查看环境信息
6) 退出
```

## 功能详解

### 1. 启动仿真机器人

选择选项 1 后，可以进一步选择仿真环境：

```
==========================================
      仿真机器人选择
==========================================
请选择仿真环境：
1) Mujoco仿真 (推荐)
2) Gazebo仿真
3) 返回主菜单
```

- **Mujoco 仿真**: 高保真物理仿真，推荐用于开发和测试
- **Gazebo 仿真**: ROS 标准仿真环境，支持多种传感器模型

### 2. 启动实物机器人

选择选项 2 后，首先选择启动模式：

```
==========================================
      实物机器人启动
==========================================
请选择启动模式：
1) 正常模式
2) 校准模式 (cali:=true)
3) 返回主菜单
```

然后，系统会提示您选择操控器类型：

```
==========================================
      操控器选择
==========================================
请选择操控器类型：
1) 北通阿修罗2 (bt2)
2) H12pro 遥控器 (h12)
3) 无操控器 (none)
4) 返回上级菜单
```

#### 启动模式说明

- **正常模式**: 标准的机器人运行模式
- **校准模式**: 用于机器人零点校准和维护


#### 操控器说明

- **北通阿修罗2 (bt2)**: 
  - 字母键切换步态: A(STANCE), B(TROT), X(JUMP), Y(WALK)
  - 左摇杆: 前后左右控制
  - 右摇杆: 转向控制
  - Start键: 从悬挂准备阶段切换到站立

- **H12pro 遥控器 (h12)**:
  - 左摇杆: 前后左右控制
  - 右摇杆: 转向控制
  - Start开关: 切换到站立状态
  - 左侧开关: 终止程序

- **无操控器 (none)**: 不启动操控器节点，适用于其他控制方式

### 3. 系统配置工具

选择选项 3 后，可以访问系统配置功能：

```
==========================================
      系统配置工具
==========================================
请选择配置工具：
1) KUAVO系统自动配置脚本
2) 返回主菜单
```

系统配置脚本包括：
- PIP 镜像源配置
- 代码仓库克隆
- 机器人版本设置
- 机器人重量配置
- 驱动板类型配置
- 手臂电机配置
- 末端执行器配置
- VR 依赖安装
- 项目编译
- 网络配置

### 4. 硬件检测工具

选择选项 4 后，可以运行硬件检测：

```
==========================================
      硬件检测工具
==========================================
请选择检测工具：
1) 硬件检测和配置工具
2) 返回主菜单
```

硬件检测工具提供：
- USB 设备检测
- IMU 状态检查
- 电机通信测试
- 传感器校准
- 系统资源监控

### 5. 环境信息

选择选项 5 可以查看当前系统环境信息：
- 工作空间目录
- 机器人版本
- 用户权限
- 编译状态
- 机器人质量配置
- 校准文件状态


## 故障排除

### .deb 包安装问题

**问题**: .deb 包下载失败

**解决方案**：
```bash
# 检查网络连接
ping kuavo.lejurobot.com

# 联系技术支持获取备用下载地址
```

**问题**: .deb 包安装失败
```bash
dpkg: error processing package ros-leju-workspace (--install):
 dependency problems - leaving unconfigured
```

**解决方案**：
```bash
# 修复依赖问题
sudo apt-get install -f

# 如果仍有问题，手动安装依赖
sudo apt-get install -y ros-noetic-catkin ros-noetic-rosbash
```

### 权限错误

```
ERROR: 此脚本必须使用 root 权限运行
```

**解决方案**：
```bash
# .deb 包安装用户
sudo su

leju 

# 源码部署用户
sudo ./scripts/leju_start.sh
```

### 工作空间未找到

```
ERROR: 未找到工作空间目录: /opt/ros/leju
```

**解决方案**：
1. **.deb 包用户**: 重新安装 .deb 包
```bash
sudo dpkg -i ros-leju-workspace_${VERSION}.deb
```

2. **源码部署用户**: 运行系统配置脚本创建工作空间
```bash
wget https://kuavo.lejurobot.com/statics/setup-kuavo-ros-control.sh
sudo ./setup-kuavo-ros-control.sh
```

### 启动命令未找到

```
bash: leju: command not found
```

**解决方案**：
```bash
# 检查是否正确安装了 .deb 包
dpkg -l | grep ros-leju-workspace

# 如果未安装，重新安装 .deb 包
sudo dpkg -i ros-leju-workspace_${VERSION}.deb
```

### 机器人版本未设置

```
WARNING: 未设置ROBOT_VERSION环境变量
```

**解决方案**：
```bash
export ROBOT_VERSION=49
# 或者在 ~/.bashrc 中永久设置
echo 'export ROBOT_VERSION=49' >> ~/.bashrc
source ~/.bashrc
```

### 编译环境问题

```
WARNING: 未找到编译后的环境
```

**解决方案**：
```bash
# .deb 包应该已经包含编译好的环境
# 如果有问题，检查安装
cd /opt/ros/leju
source installed/setup.bash

# 源码部署用户需要手动编译
catkin build humanoid_controllers
```

## 系统配置说明

### 配置文件位置

- **机器人质量**: `~/.config/lejuconfig/TotalMassV${ROBOT_VERSION}`
- **腿部零位校准**: `~/.config/lejuconfig/offset.csv`
- **手臂零位校准**: `~/.config/lejuconfig/arms_zero.yaml`
- **驱动板类型**: `~/.config/lejuconfig/EcMasterType.ini`

### 网络配置

脚本会自动检查和配置：
- ROS_MASTER_URI
- ROS_HOSTNAME
- /etc/hosts 映射关系

## 开发者信息

### 源代码位置

- **启动脚本**: `scripts/leju_start.sh`
- **系统配置**: `tools/setup-kuavo-ros-control.sh`
- **硬件检测**: `tools/check_tool/Hardware_tool.py`

### .deb 包构建信息

.deb 包包含：
- 完整的工作空间 (`/opt/ros/leju`)
- 启动脚本 (`/usr/bin/leju`)
- 所有必需的配置文件和依赖
- 预编译的二进制文件

### 包信息

- **包名**: `ros-leju-workspace`
- **版本**: 由 Git 标签自动生成
- **架构**: amd64
- **维护者**: LejuRobot <LejuRobot@lejurobot.com>
- **描述**: Leju kuavo robot Workspace under /opt/ros/leju
- **下载地址**: https://kuavo.lejurobot.com/packages/
- **最新版直链**: https://kuavo.lejurobot.com/packages/ros-leju-workspace_latest.deb

### 日志和调试

- 启动脚本会输出详细的执行信息
- 各工具都有独立的错误处理和提示
- 使用 `set -euo pipefail` 确保错误及时捕获


## 技术支持

如遇到问题，请：
1. 检查本文档的故障排除部分
2. 查看脚本输出的详细错误信息
3. 运行硬件检测工具诊断系统状态
4. 联系技术支持团队

## 部署建议

### 生产环境推荐
- 使用 .deb 包部署
- 配置自动更新机制
- 定期备份配置文件
- 监控系统运行状态

### 开发环境推荐
- 使用源码部署
- 利用 Git 进行版本控制
- 定期同步最新代码
- 参与社区贡献

---

*KUAVO 机器人系统 - 为人形机器人控制提供完整的解决方案*