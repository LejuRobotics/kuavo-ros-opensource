#!/usr/bin/env python3
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 将 scripts/g11_screen_protocol.py 安装为顶层模块 g11_screen_protocol,
# 供 h12pro_controller_node 的 ocs2_h12pro_node.py import 使用.
d = generate_distutils_setup(
    packages=[],
    py_modules=['g11_screen_protocol'],
    package_dir={'': 'scripts'},
)

setup(**d)
