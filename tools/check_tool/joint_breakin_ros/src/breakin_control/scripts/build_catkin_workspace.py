#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
独立脚本：自动检测并编译当前所在的 ROS catkin 工作空间

使用方法（在任意子目录下都可以）：
    python3 build_catkin_workspace.py

脚本会：
1. 从本文件所在目录开始，一直向上查找，找到「包含 src 目录且/或 .catkin_workspace 文件」的那个目录，作为工作空间根目录
2. 检查 workspace_root 下是否已经存在：
      - build/ 目录
      - devel/setup.bash
   如果都存在，认为已经编译过，直接提示已完成并退出
3. 否则，在该 workspace_root 目录下执行一次 catkin_make
"""

import os
import sys
import subprocess
import shutil
from pathlib import Path


def detect_workspace_root(start_dir: Path, max_depth: int = 8) -> Path:
    """
    从 start_dir 向上递归查找，找到 catkin 工作空间根目录：
    - 优先：同时满足「存在 src/ 目录」并且「存在 .catkin_workspace 文件」
    - 退化：只要存在 src/ 目录也认为是工作空间根
    """
    path = start_dir.resolve()

    best_candidate = None

    for _ in range(max_depth):
        src_dir = path / "src"
        catkin_marker = path / ".catkin_workspace"

        if src_dir.is_dir() and catkin_marker.is_file():
            # 最理想的匹配，直接返回
            return path

        if src_dir.is_dir() and best_candidate is None:
            # 先记下一个仅含 src 的候选
            best_candidate = path

        if path.parent == path:
            break
        path = path.parent

    # 如果没找到带 .catkin_workspace 的，就返回第一个包含 src 的目录
    if best_candidate is not None:
        return best_candidate

    # 实在找不到就抛异常，让调用方提示用户
    raise RuntimeError("未能在上级目录中找到包含 src 的 ROS 工作空间根目录")


def build_catkin_make_command(workspace_root: Path, base_env: dict):
    """
    生成调用 catkin_make 的命令和环境：
    1) 如果 PATH 中能找到 catkin_make，直接调用绝对路径
    2) 否则尝试 source 常见的 ROS setup.bash 后再调用 catkin_make
    """
    env = base_env.copy()

    # 1) 直接在 PATH 中查找
    catkin_path = shutil.which("catkin_make", path=env.get("PATH", ""))
    if catkin_path:
        return [catkin_path], env

    # 2) 尝试常见的 setup.bash
    ros_distro = env.get("ROS_DISTRO")
    setup_candidates = []
    if ros_distro:
        setup_candidates.append(f"/opt/ros/{ros_distro}/setup.bash")
    # 再加一些常见发行版兜底
    setup_candidates.extend([
        "/opt/ros/noetic/setup.bash",
        "/opt/ros/melodic/setup.bash",
        "/opt/ros/kinetic/setup.bash",
    ])

    for setup in setup_candidates:
        setup_path = Path(setup)
        if setup_path.exists():
            # 用 bash -c 方式 source 后再调用 catkin_make，并在命令里显式 cd
            bash_cmd = f"source {setup_path} && cd {workspace_root} && catkin_make"
            return ["bash", "-c", bash_cmd], env

    # 仍未找到，抛出 FileNotFoundError 由上层处理
    raise FileNotFoundError("catkin_make not found in PATH, and no usable /opt/ros/<distro>/setup.bash")


def ensure_workspace_built(workspace_root: Path) -> int:
    """
    确保指定 workspace_root 已经 catkin_make 过。
    返回值:
        0  成功（已编译或本次编译成功）
        非0  失败
    """
    build_lib_dir = workspace_root / "build_lib" / "lib"
    build_dir = workspace_root / "build"
    devel_setup = workspace_root / "devel" / "setup.bash"

    print(f"检测到 ROS 工作空间路径: {workspace_root}")

    # 检查 build_lib 中是否存在预编译的库文件（优先检查）
    if build_lib_dir.exists():
        # 检查关键库文件是否存在
        key_files = [
            build_lib_dir / "libcanbus_sdk.so",
            build_lib_dir / "libmotorevo_controller.so",
            build_lib_dir / "arm_breakin" / "arm_breakin_node",
        ]
        found_files = [f for f in key_files if f.exists()]
        if len(found_files) > 0:
            print(f"✓ 检测到 build_lib/lib 目录及其中的编译产物（找到 {len(found_files)} 个关键文件），认为已经编译完成。")
            return 0

    # 如果 build_lib 不存在，检查编译标志
    if build_dir.is_dir() and devel_setup.exists():
        print("✓ 检测到 build/ 和 devel/setup.bash，认为已经编译完成，无需再次编译。")
        return 0

    print("⚠ 未检测到编译产物（build_lib/lib 或 build/ 目录），准备执行 catkin_make ...")
    print(f"Base path (catkin 工作空间根): {workspace_root}")

    try:
        env = os.environ.copy()
        # 确保 catkin_make 获取到正确的工作目录（catkin_make 内部依赖 PWD）
        env["PWD"] = str(workspace_root)

        cmd, cmd_env = build_catkin_make_command(workspace_root, env)

        result = subprocess.run(
            cmd,
            cwd=str(workspace_root),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=cmd_env,
            shell=False,
        )

        # 打印输出（保留最后一部分，避免刷屏过长）
        output = result.stdout or ""
        lines = output.splitlines()
        if len(lines) > 60:
            print("------ catkin_make 输出（结尾部分，前面略） ------")
            for line in lines[-60:]:
                print(line)
        else:
            print(output)

        if result.returncode == 0:
            print("✓ catkin_make 编译成功。")
            return 0
        else:
            print("✗ catkin_make 编译失败，请根据上面的日志排查问题。")
            return result.returncode

    except FileNotFoundError:
        print("错误：未找到 catkin_make 命令，请确认已安装 ROS 并正确配置环境（例如 source /opt/ros/noetic/setup.bash）")
        return 1
    except Exception as e:
        print(f"执行 catkin_make 时发生异常: {e}")
        return 1


def main() -> int:
    script_dir = Path(__file__).resolve().parent
    print(f"当前脚本所在目录: {script_dir}")

    try:
        workspace_root = detect_workspace_root(script_dir)
    except RuntimeError as e:
        print(f"错误：{e}")
        print("提示：请确认此脚本位于某个 catkin 工作空间的子目录下（该工作空间应包含 src/ 目录）。")
        return 1

    return ensure_workspace_built(workspace_root)


if __name__ == "__main__":
    sys.exit(main())
