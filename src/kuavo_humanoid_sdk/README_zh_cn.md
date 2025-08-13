# Kuavo Humanoid SDK

一个全面的 Python SDK，用于控制 Kuavo 人形机器人。该 SDK 提供了机器人状态管理、手臂和头部控制以及末端执行器操作的接口。它设计用于与 ROS（机器人操作系统）环境一起工作。

**警告**：该 SDK 目前仅支持 **ROS1**。不支持 ROS2。

PyPI 项目地址: https://pypi.org/project/kuavo-humanoid-sdk/

## 安装

要安装 Kuavo Humanoid SDK，可以使用 pip：
```bash
pip install kuavo-humanoid-sdk
```

对于开发安装（可编辑模式），请使用：
```bash
pip install -e .
```

## 描述

有关详细的 SDK 文档和使用示例，请参阅 [sdk_description.md](sdk_description.md)。

## 文档

文档提供两种格式：
- HTML 格式：[docs/html](docs/html)
- Markdown 格式：[docs/markdown](docs/markdown)


## 维护者人员参考

### 打包 SDK

要打包 SDK 以便分发，请按照以下步骤操作：
```bash
python3 setup.py sdist bdist_wheel

ls -lh ./dist/
总用量 76K
-rw-rw-r-- 1 lab lab 41K 2月  22 17:37 kuavo_humanoid_sdk-0.1.0-py3-none-any.whl
-rw-rw-r-- 1 lab lab 32K 2月  22 17:37 kuavo_humanoid_sdk-0.1.0.tar.gz
```

### 上传到 PyPI

首先，创建或编辑 `~/.pypirc` 并添加您的 API 令牌：
```
[testpypi]
username = __token__
password = pypi-<your-token>
```

要将软件包上传到 PyPI，请使用 twine：
```bash
pip install --upgrade requests-toolbelt
pip install "urllib3<=1.26.16" "twine<4.0.0" pyopenssl cryptography

twine upload --repository testpypi dist/* --verbose
```
