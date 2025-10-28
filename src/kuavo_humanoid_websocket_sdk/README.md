# Kuavo Humanoid SDK
[![Version](https://img.shields.io/pypi/v/kuavo-humanoid-sdk-ws.svg)](https://pypi.org/project/kuavo-humanoid-sdk-ws/)[![License](https://img.shields.io/pypi/l/kuavo-humanoid-sdk-ws.svg)](#)[![Supported Python Versions](https://img.shields.io/pypi/pyversions/kuavo-humanoid-sdk-ws.svg)](https://pypi.python.org/pypi/kuavo-humanoid-sdk-ws)

一个全面的 Python SDK，用于控制 Kuavo 人形机器人。该 SDK 提供了机器人状态管理、手臂和头部控制以及末端执行器操作的接口。它设计用于与 ROS（机器人操作系统）环境一起工作。

**警告**：该 SDK 目前仅支持 **ROS1**。不支持 ROS2。

PyPI 项目地址: https://pypi.org/project/kuavo-humanoid-sdk-ws/

## 安装
**提示：对于本 SDK 目前存在两个版本，正式发布版与beta内测版, 他们的区别是：**
- 正式发布版：稳定版，对应[kuavo-ros-opensource](https://gitee.com/leju-robot/kuavo-ros-opensource/)的`master` 分支提供的功能，
- beta内测版：该版本较正式版会激进一些，同时也会提供更丰富的功能，对应[kuavo-ros-opensource](https://gitee.com/leju-robot/kuavo-ros-opensource/)的`beta` 分支提供的功能。

**温馨提示：请务必明确您需要安装的版本，如果您的SDK版本与`kuavo-ros-opensource`未匹配，可能会出现某些功能不可用的错误。**

安装最新的**正式版** Kuavo Humanoid SDK，可以使用 pip：
```bash
pip install kuavo-humanoid-sdk-ws
```

安装最新的**beta版** Kuavo Humanoid SDK，可以使用 pip：
```bash
pip install --pre kuavo-humanoid-sdk-ws

```
对于本地开发安装（可编辑模式），请使用：
```bash
cd src/kuavo_humanoid_sdk_ws
chmod +x install.sh
./install.sh
```

## 升级更新

在升级更新之前，您可以先执行以下命令来查看当前安装的版本：
```bash
pip show kuavo-humanoid-sdk-ws
# Output:
Name: kuavo-humanoid-sdk-ws
Version: 0.1.2
...
```
**提示：如果您的版本号中包含字母`b`，则表示该版本为测试版, 比如`Version: 0.1.2b113`**

**当前为正式版**，升级到最新正式版:
```bash
pip install --upgrade kuavo-humanoid-sdk-ws
```
**当前为beta版**，升级到最新正式版:
```bash
pip install --upgrade --force-reinstall kuavo-humanoid-sdk-ws
# 或者
pip uninstall kuavo-humanoid-sdk-ws && pip install kuavo-humanoid-sdk-ws
```
**当前为正式版/beta版**，升级到最新beta版:
```bash
pip install --upgrade --pre kuavo-humanoid-sdk-ws
```

## 描述

有关详细的 SDK 文档和使用示例，请参阅 [sdk_description.md](sdk_description.md)。

## 文档
文档提供两种格式：
- HTML 格式：[docs/html](docs/html), **需要自己执行脚本生成**
- Markdown 格式：[docs/markdown](docs/markdown)

我们推荐您自己执行文档脚本生成文档到本地, 文档会输出到`docs/html`和`docs/markdown`文件夹:
```bash
chmod +x ./gen_docs.sh
./gen_docs.sh
```

我们推荐您使用`html`查看文档更加方便，比如:
![](docs/images/image.png)


