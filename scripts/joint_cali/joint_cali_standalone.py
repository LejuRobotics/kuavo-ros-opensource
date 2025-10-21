#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import shlex
import argparse
import subprocess
import hashlib
from pathlib import Path
from typing import Optional, Dict
import json

# ===== 新增：确保交互 stdin 可用 =====
def ensure_interactive_stdin() -> None:
    """
    确保本进程的 stdin 可用于交互：
    - 若 fd0 无效或不是 TTY，则尝试绑定到 /dev/tty
    - 调用 'stty sane' 恢复终端模式（若可用）
    """
    def _fd0_is_valid():
        try:
            os.fstat(0)
            return True
        except OSError:
            return False

    def _stdin_is_tty():
        try:
            return sys.stdin is not None and sys.stdin.isatty()
        except Exception:
            return False

    if (not _fd0_is_valid()) or (not _stdin_is_tty()):
        try:
            tty = open('/dev/tty', 'r')
            os.dup2(tty.fileno(), 0)
            sys.stdin = tty
            print("[INFO] stdin 重新绑定到 /dev/tty")
        except Exception as e:
            # 在无 TTY（后台、重定向）情况下会失败，此时只能走非交互路径
            print(f"[WARN] 无法绑定 /dev/tty（{e}），禁用交互输入。")

    # 恢复 sane 终端模式（若存在 TTY）
    try:
        subprocess.run(["bash", "-lc", "stty sane < /dev/tty"], check=False)
    except Exception:
        pass

# ===== 新增：安全 input =====
def safe_input(prompt: str, default: str = "") -> str:
    """
    更稳的 input：
    - 无 TTY 或遇到 EOF 时返回 default
    - 清掉残留输入，避免上一轮运行留下的奇怪状态
    """
    try:
        import termios
        termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    except Exception:
        pass

    if not sys.stdin or not sys.stdin.isatty():
        print(f"[INFO] 无可用交互输入，返回默认值：{default!r}")
        return default
    try:
        return input(prompt)
    except EOFError:
        print(f"[INFO] 收到 EOF，返回默认值：{default!r}")
        return default

# === 需要随程序打包/运行的资源（源路径 -> 运行时目标相对路径）===
# 支持文件或目录；目录会整目录复制
EMBED_FILES: Dict[str, str] = {
    "start_host_apriltag.py": "start_host_apriltag.py",
    "head_cali.py": "head_cali.py",
    "arm_cail_noui.py": "arm_cail_noui.py",
    "arm_kinematics.py": "arm_kinematics.py",

    # 额外模块
    "identifiability_analyzer.py": "identifiability_analyzer.py",
    "apriltag_cube.py": "apriltag_cube.py",
    "ik_cmd.py": "ik_cmd.py",
    "target_tracker.py": "target_tracker.py",

    # 你把脚本打包在根目录，所以目标也放在根：cache_dir/create_venv.sh
    "create_venv.sh": "create_venv.sh",
    "requirements.txt": "requirements.txt",

    # 整个 config 目录
    "config": "config",

    # 整个 bags 目录
    "bags": "bags",

    # 构建信息
    "build_info.json": "build_info.json",
}

def run_bash(cmd: str, check: bool = True, env=None, capture_output: bool = False):
    """在 bash -lc 下执行（可 source 环境）"""
    print(f"[BASH] {cmd}")
    return subprocess.run(
        ["bash", "-lc", cmd],
        check=check,
        env=env,
        text=True,
        capture_output=capture_output,
    )

def detect_cache_dir() -> Path:
    """生成与当前可执行路径绑定的稳定缓存目录，放置解压资源。"""
    exe = Path(sys.argv[0]).resolve()
    h = hashlib.sha256(str(exe).encode()).hexdigest()[:12]
    base = Path(os.environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache"))) / "joint_cali" / h
    (base / "config").mkdir(parents=True, exist_ok=True)
    return base

def copy_file(src: Path, dst: Path):
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_bytes(src.read_bytes())

def copy_dir(src_dir: Path, dst_dir: Path):
    for p in src_dir.rglob("*"):
        rel = p.relative_to(src_dir)
        dst_p = dst_dir / rel
        if p.is_dir():
            dst_p.mkdir(parents=True, exist_ok=True)
        else:
            dst_p.parent.mkdir(parents=True, exist_ok=True)
            dst_p.write_bytes(p.read_bytes())

def extract_embeds(target_dir: Path):
    """
    把资源复制到 target_dir。
    - 打包(onefile)运行：资源在 sys._MEIPASS 下优先按精确相对路径查找，
      若不存在则回退到按 basename 搜索（兼容某些 PyInstaller 布局）
    - 开发模式：从脚本同目录复制（支持文件或目录）
    """
    meipass = getattr(sys, "_MEIPASS", None)

    if meipass is not None:
        meipass_path = Path(meipass)
        print(f"[INFO] MEIPASS = {meipass_path}")

        for src_rel, dst_rel in EMBED_FILES.items():
            dst = target_dir / dst_rel
            # 1) 精确相对路径
            exact = meipass_path / src_rel
            if exact.exists():
                if exact.is_dir():
                    copy_dir(exact, dst)
                else:
                    copy_file(exact, dst)
                continue

            # 2) 回退 basename 搜索
            name = Path(src_rel).name
            candidates = list(meipass_path.rglob(name))
            if not candidates:
                print(f"[WARN] 未在打包资源中找到：{src_rel}")
                continue

            chosen = next((c for c in candidates if c.is_dir()), candidates[0])
            if chosen.is_dir():
                copy_dir(chosen, dst)
            else:
                copy_file(chosen, dst)

    else:
        project_root = Path(__file__).resolve().parent
        print(f"[INFO] 开发模式复制资源，项目根：{project_root}")
        for src_rel, dst_rel in EMBED_FILES.items():
            src = project_root / src_rel
            dst = target_dir / dst_rel
            if not src.exists():
                print(f"[WARN] 源路径缺失(开发模式)：{src}")
                continue
            if src.is_dir():
                copy_dir(src, dst)
            else:
                copy_file(src, dst)

def build_ros_prefix(ros_setup: Optional[str], ws_setup: Optional[str]) -> str:
    """构造前缀命令：按需 source ROS 环境与工作空间"""
    parts = []
    if ros_setup:
        parts.append(f"source {shlex.quote(ros_setup)}")
    if ws_setup:
        parts.append(f"source {shlex.quote(ws_setup)}")
    else:
        # 如果未指定ws_setup，尝试两个常用默认路径
        default_ws_setups = [
            "/home/lab/kuavo-ros-control/devel/setup.bash",
            "/home/lab/kuavo-ros-opensource/devel/setup.bash"
        ]
        for ws in default_ws_setups:
            if Path(ws).is_file():
                parts.append(f"source {shlex.quote(ws)}")
                break
        else:
            print("[WARN] 未找到默认的工作空间setup.bash，后续可能找不到ROS包。")
            print("[ERROR] 请检查ROS环境是否正确配置。")
            sys.exit(1)
    return " && ".join(parts) + (" && " if parts else "")

def build_venv_prefix(venv_activate: Optional[str]) -> str:
    """构造前缀命令：按需 source 虚拟环境（python3 将来自 venv）"""
    if venv_activate:
        return f"source {shlex.quote(venv_activate)} && "
    return ""

def build_py_env_exports(cache_dir: Path, extra_ld: Optional[str]) -> str:
    """
    构造环境导出命令（在已激活 venv 之后执行）：
    - 设定 PYTHONNOUSERSITE=1，避免用户/系统 site-packages 污染
    - PYTHONPATH: cache_dir 与 cache_dir/config（以及 MEIPASS/MEIPASS/config）
    - LD_LIBRARY_PATH:
        * 自动探测 venv 中 cmeel.prefix/lib，并放到最前面（优先使用 venv 的 .so）
        * 再按需追加 extra_ld
    """
    parts = [str(cache_dir), str(cache_dir / "config")]

    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        parts.append(meipass)
        mp_cfg = Path(meipass) / "config"
        if mp_cfg.exists():
            parts.append(str(mp_cfg))

    py_path = ":".join(parts)

    # 用 venv 的 python3 动态求 cmeel.prefix/lib（eigenpy 或 pinocchio 任取其一定位）
    get_cmeel_lib = r"""python3 - <<'PY'
import pathlib, sys
def find():
    try:
        import eigenpy
        return (pathlib.Path(eigenpy.__file__).resolve().parents[2] / "cmeel.prefix" / "lib")
    except Exception:
        try:
            import pinocchio
            return (pathlib.Path(pinocchio.__file__).resolve().parents[2] / "cmeel.prefix" / "lib")
        except Exception:
            return ""
p = find()
print(str(p) if p else "")
PY"""

    cmd = (
        # 1) 关闭 user site
        f'export PYTHONNOUSERSITE=1 && '
        # 2) 设置 PYTHONPATH
        f'export PYTHONPATH={shlex.quote(py_path)}:$PYTHONPATH && '
        # 3) 探测 cmeel lib 并优先放入 LD_LIBRARY_PATH
        f'__CMEEL_LIB__=$({get_cmeel_lib}) && '
        f'if [ -n "$__CMEEL_LIB__" ] && [ -d "$__CMEEL_LIB__" ]; then '
        f'  echo "[INFO] cmeel lib: $__CMEEL_LIB__"; '
        f'  export LD_LIBRARY_PATH="$__CMEEL_LIB__:$LD_LIBRARY_PATH"; '
        f'fi && '
    )
    if extra_ld:
        cmd += f'export LD_LIBRARY_PATH={shlex.quote(extra_ld)}:$LD_LIBRARY_PATH && '
    return cmd

def ensure_venv_from_embedded(cache_dir: Path, venv_root: Path):
    """
    检查虚拟环境是否存在，不存在则用打包资源里的 create_venv.sh 创建
    - 你把 create_venv.sh 打包在根目录，所以这里直接从 cache_dir 执行
    """
    if venv_root.exists():
        print(f"[INFO] 检测到虚拟环境：{venv_root}")
        return

    print(f"[INFO] 未找到虚拟环境目录：{venv_root}")
    script_path = cache_dir / "create_venv.sh"
    if not script_path.is_file():
        print(f"[ERR ] 打包资源中缺少 create_venv.sh：{script_path}")
        sys.exit(1)

    # 确保可执行
    try:
        script_path.chmod(script_path.stat().st_mode | 0o111)
    except Exception:
        pass

    try:
        run_bash(f"bash {shlex.quote(str(script_path))}", check=True)
        print("✓ 虚拟环境创建完成")
    except subprocess.CalledProcessError as e:
        print(f"[ERR ] 虚拟环境创建失败: {e}")
        sys.exit(1)

def main():
    # 每次运行开始就修复 stdin（确保同一终端可多次运行）
    ensure_interactive_stdin()

    ap = argparse.ArgumentParser(description="关节标定一键执行（打包版，含 venv 激活与 ROS 环境加载）")
    ap.add_argument("--yes", action="store_true", help="无人值守：默认回答 yes")
    ap.add_argument("--skip-host", action="store_true", help="跳过上位机 AprilTag 启动")
    ap.add_argument("--skip-head", action="store_true", help="跳过头部标定")
    ap.add_argument("--skip-arm", action="store_true", help="跳过手臂标定")

    # ROS 环境
    ap.add_argument("--ros-setup", default="/opt/ros/noetic/setup.bash",
                    help="ROS 全局 setup.bash（默认 /opt/ros/noetic/setup.bash；留空字符串可禁用）")
    ap.add_argument("--ws-setup", default="",
                    help="工作空间 devel/setup.bash（可选）")

    # 虚拟环境（必须是 python3 的 venv）
    ap.add_argument("--venv-activate", default="/home/lab/kuavo_venv/joint_cali/bin/activate",
                    help="虚拟环境 activate 脚本路径（默认 /home/lab/kuavo_venv/joint_cali/bin/activate；留空禁用）")

    # 额外库路径
    ap.add_argument("--extra-ld", default="",
                    help="额外的 LD_LIBRARY_PATH 项，以 : 分隔（可选）")

    args = ap.parse_args()

    cache_dir = detect_cache_dir()
    extract_embeds(cache_dir)
    print(f"[INFO] 资源已准备到：{cache_dir}")
        # 检查构建信息
    build_info_path = cache_dir / "build_info.json"
    if not build_info_path.is_file():
        print(f"[ERR ] 未找到构建信息：{build_info_path}")
        sys.exit(1)
    build_info = json.loads(build_info_path.read_text(encoding="utf-8"))
    print(f"[INFO] 构建信息：{build_info}")

    # 确保虚拟环境存在（从打包的脚本创建）
    if args.venv_activate.strip():
        venv_root = Path(args.venv_activate).resolve().parent.parent
        ensure_venv_from_embedded(cache_dir, venv_root)

    ros_prefix  = build_ros_prefix(args.ros_setup.strip() or None, args.ws_setup.strip() or None)
    venv_prefix = build_venv_prefix(args.venv_activate.strip() or None)
    env_exports = build_py_env_exports(cache_dir, args.extra_ld.strip() or None)

    # ============= 步骤 3：上位机 AprilTag =============
    if not args.skip_host:
        host_py = cache_dir / "start_host_apriltag.py"
        if not host_py.is_file():
            print(f"[ERR ] 缺少 start_host_apriltag.py（打包时未包含？）")
            sys.exit(1)
        cmd = f"{ros_prefix}{venv_prefix}{env_exports}python3 {shlex.quote(str(host_py))}"
        try:
            run_bash(cmd, check=True)
            print("✓ 上位机 AprilTag 启动完成")
        except subprocess.CalledProcessError as e:
            print(f"[ERR ] 上位机启动失败: {e}")
            sys.exit(1)

    # ============= 步骤 4：头部标定 =============
    if not args.skip_head:
        cfg = cache_dir / "config" / "head_cali_config.yaml"
        if cfg.is_file():
            print("当前头部标定配置：\n----------------------------------------")
            try:
                print(cfg.read_text(encoding="utf-8"))
            except Exception as e:
                print(f"[WARN] 读取配置失败：{e}")
            print("----------------------------------------\n")

        # 改为安全交互（仍支持 --yes 直接继续）
        ans = "y" if args.yes else safe_input("是否继续头部标定？[y/N]: ", default="n")
        cont = ans.strip().lower() in ("y", "yes", "是", "s", "1", "true")

        if cont:
            head_py = cache_dir / "head_cali.py"
            if not head_py.is_file():
                print(f"[ERR ] 缺少 head_cali.py（打包时未包含？）")
                sys.exit(1)
            cmd = f"{ros_prefix}{venv_prefix}{env_exports}python3 {shlex.quote(str(head_py))} --use_cali_tool"
            try:
                run_bash(cmd, check=True)
            except subprocess.CalledProcessError as e:
                print(f"[WARN] 头部标定失败：{e}")
            else:
                bak = Path("/home/lab/.config/lejuconfig/arms_zero.yaml.head_cali.bak")
                if bak.is_file():
                    print(f"✓ 发现标定备份文件：{bak}")
                else:
                    print("⚠ 未发现标定备份文件，可能标定未完成")
        else:
            print("跳过头部标定")

    # ============= 步骤 5：手臂标定 =============
    if not args.skip_arm:
        arm_py = cache_dir / "arm_cail_noui.py"
        if not arm_py.is_file():
            print(f"[WARN] 未找到手臂标定脚本（跳过）：{arm_py}")
        else:
            # --- 计算 VENV 根与 lib 目录 ---
            venv_activate = args.venv_activate.strip() or ""
            venv_lib = ""
            if venv_activate:
                # /path/to/venv/bin/activate -> /path/to/venv/lib
                venv_bin = Path(venv_activate).resolve().parent
                venv_root = venv_bin.parent
                venv_lib = str(venv_root / "lib")

            # --- 组装 PYTHONPATH（保持你原有的 cache/config + 可选 MEIPASS）---
            meipass = getattr(sys, "_MEIPASS", None)
            py_parts = [str(cache_dir), str(cache_dir / "config")]
            if meipass:
                py_parts += [meipass, str(Path(meipass) / "config")]
            py_path = ":".join(py_parts)

            # --- 组装命令：严格按你给出的环境导入顺序 ---
            lines = []
            if venv_activate:
                lines.append(f"source {shlex.quote(venv_activate)}")
            if args.ws_setup.strip():
                lines.append(f"source {shlex.quote(args.ws_setup.strip())}")
            # 不再显式 source 全局 ROS（按你的示例）；如需保留可加：source {args.ros_setup}
            lines.append("unset LD_LIBRARY_PATH")
            if venv_lib:
                lines.append(f"export LD_LIBRARY_PATH={shlex.quote(venv_lib)}:$LD_LIBRARY_PATH")
            # 你的要求里明确加 ROS 的 lib 路径
            lines.append("export LD_LIBRARY_PATH=/opt/ros/noetic/lib:$LD_LIBRARY_PATH")
            # 保留原有的 PYTHONPATH（否则 cache 中模块找不到）
            lines.append(f"export PYTHONPATH={shlex.quote(py_path)}:$PYTHONPATH")
            # 真正执行
            lines.append(f"python3 {shlex.quote(str(arm_py))} --real")

            cmd = " && ".join(lines)

            try:
                run_bash(cmd, check=True)
                print("✓ 手臂标定完成")
            except subprocess.CalledProcessError as e:
                print(f"[WARN] 手臂标定失败：{e}")

    print("\n===========================================================")
    print("系统启动/标定流程完成。")
    print("===========================================================")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] 用户中断，退出。")
