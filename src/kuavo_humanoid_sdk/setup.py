from setuptools import setup
import os

def find_msg_subpackages(base_dir1):
    subpackages = []
    # Get the script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Ensure we're working with the correct base directory for message packages
    base_dir = os.path.join(script_dir, base_dir1)
    for root, dirs, _ in os.walk(base_dir):
        if "__init__.py" in os.listdir(root):
            rel_path = os.path.relpath(root, script_dir)
            pkg_name = rel_path.replace("/", ".").replace("\\", ".")
            subpackages.append(pkg_name)
    return subpackages

def find_subpackages(base_dir):
    """查找指定目录下的所有子包"""
    subpackages = []
    script_dir = os.path.dirname(os.path.abspath(__file__))
    full_base_dir = os.path.join(script_dir, base_dir)

    if not os.path.exists(full_base_dir):
        return subpackages

    for root, dirs, files in os.walk(full_base_dir):
        # 排除不需要的目录
        dirs[:] = [d for d in dirs if d not in ['__pycache__', '.git', '.pytest_cache',
                                               'node_modules', '.vscode', '.idea',
                                               'build', 'dist', 'egg-info']]

        # 将所有目录转换为包名
        rel_path = os.path.relpath(root, script_dir)
        if rel_path != base_dir:  # 不包括基础目录本身
            pkg_name = rel_path.replace("/", ".").replace("\\", ".")
            subpackages.append(pkg_name)

    return sorted(subpackages)  # 返回排序后的列表

# 自动查找子包
core_packages = find_subpackages("kuavo_humanoid_sdk/kuavo/core")
strategy_packages = find_subpackages("kuavo_humanoid_sdk/kuavo_strategy")
strategy_v2_packages = find_subpackages("kuavo_humanoid_sdk/kuavo_strategy_v2")

# Check if a version argument is provided
sdk_version = os.environ.get('KUAVO_HUMANOID_SDK_VERSION')
# If no version is provided, raise an error
if not sdk_version:
    raise ValueError("KUAVO_HUMANOID_SDK_VERSION environment variable must be set. "
                     "Please set it before running setup.py.")
print("sdk_version:", sdk_version)

with open("sdk_description.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="kuavo_humanoid_sdk",
    license="MIT",
    author=["lejurobot"],
    author_email="edu@lejurobot.com",
    version=sdk_version,
    description="A Python SDK for kuavo humanoid robot.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://gitee.com/leju-robot/kuavo-ros-opensource/",
    keywords=["kuavo", "humanoid", "robot", "robotics", "lejurobot", "ros"],
    packages=[
    'kuavo_humanoid_sdk',
    'kuavo_humanoid_sdk.common',
    'kuavo_humanoid_sdk.interfaces',
    'kuavo_humanoid_sdk.kuavo',
    ] + core_packages + strategy_packages + strategy_v2_packages + find_msg_subpackages("kuavo_humanoid_sdk/msg"),
    install_requires=[
        "numpy", 
        "transitions",
        "pyOpenSSL>=22.1.0",
        "scikit-learn",
        "funasr",
        "torchaudio",
        "websockets",
        "ultralytics"
    ],
    python_requires=">=3.8",
    classifiers=[
        "Development Status :: 4 - Beta", 
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    project_urls={
        "Documentation": "https://gitee.com/leju-robot/kuavo-ros-opensource/",
        "Source Code": "https://gitee.com/leju-robot/kuavo-ros-opensource/",
    }
)
