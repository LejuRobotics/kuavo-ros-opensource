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
    'kuavo_humanoid_sdk.kuavo.core',
    'kuavo_humanoid_sdk.kuavo.core.ros',
    'kuavo_humanoid_sdk.kuavo_strategy',
    'kuavo_humanoid_sdk.kuavo_strategy.grasp_box',
    'kuavo_humanoid_sdk.kuavo_strategy_v2',
    'kuavo_humanoid_sdk.kuavo_strategy_v2.common',
    'kuavo_humanoid_sdk.kuavo_strategy_v2.common.events',
    'kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box',
    'kuavo_humanoid_sdk.kuavo_strategy_v2.utils',
    ]+find_msg_subpackages("kuavo_humanoid_sdk/msg"),
    install_requires=[
        "numpy", 
        "transitions"
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