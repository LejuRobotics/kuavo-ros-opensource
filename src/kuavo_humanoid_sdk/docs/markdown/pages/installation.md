<a id="installation"></a>

# 安装指南

## 安装

安装 Kuavo Humanoid SDK 有两种方式:

1. 通过 PyPI 安装(推荐)

   #### NOTE
   目前 SDK 有两个版本,稳定版和测试版。它们的区别是:
   - **稳定版**: 对应 [kuavo-ros-opensource](https://gitee.com/leju-robot/kuavo-ros-opensource/) 仓库 master 分支提供的功能。
   - **测试版**: 该版本比正式版更激进,提供更丰富的功能,对应 [kuavo-ros-opensource](https://gitee.com/leju-robot/kuavo-ros-opensource/) 仓库 beta 分支提供的功能。

   #### WARNING
   请明确你需要安装的版本，如果你的 SDK 版本与 kuavo-ros-opensource 不匹配,某些功能可能无法使用。

   使用 pip 安装最新的 **稳定版** Kuavo Humanoid SDK:
   ```bash
   pip install kuavo-humanoid-sdk
   ```

   使用 pip 安装最新的 **测试版** Kuavo Humanoid SDK:
   ```bash
   pip install --pre kuavo-humanoid-sdk
   ```
2. **或者** 以开发模式本地安装

   克隆代码仓库并以可编辑模式安装:
   ```bash
   git clone https://gitee.com/leju-robot/kuavo-ros-opensource.git
   cd kuavo-ros-opensource/src/kuavo_humanoid_sdk
   chmod +x install.sh
   ./install.sh
   ```

   这种方式允许你修改源代码,修改后无需重新安装即可生效。

## 升级

> 升级前,你可以通过以下命令查看当前安装的版本:

> ```bash
> pip show kuavo-humanoid-sdk
> # 输出:
> Name: kuavo-humanoid-sdk
> Version: 0.1.2
> ...
> ```

> #### NOTE
> 如果版本号中包含字母 b,表示这是测试版,例如 Version: 0.1.2b113

> 如果版本号中包含字母 a,表示这是开发版,例如 Version: 0.1.2a113

> 从 **稳定版** 升级到最新稳定版:

> ```bash
> pip install --upgrade kuavo_humanoid_sdk
> ```

> 从 **测试版** 升级到最新稳定版:

> ```bash
> pip install --upgrade --force-reinstall kuavo_humanoid_sdk
> # 或者
> pip uninstall kuavo_humanoid_sdk && pip install kuavo_humanoid_sdk
> ```

> 从 **稳定版/测试版** 升级到最新测试版:

> ```bash
> pip install --upgrade --pre kuavo_humanoid_sdk
> ```
