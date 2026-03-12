# 兼容旧版 pip：仅含 pyproject.toml 时部分环境无法 -e 安装，此文件令 setuptools 从 pyproject.toml 读取配置
from setuptools import setup

setup()
