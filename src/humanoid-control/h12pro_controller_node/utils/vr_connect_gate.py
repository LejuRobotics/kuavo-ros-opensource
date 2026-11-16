"""H12→VR 连通门禁：切态 / 静音摇杆前等待 /quest3/connected。

长按 A 拉起 tmux 后，旧逻辑只检查 session 是否存在就进入 vr_remote_control，
导致 Quest 未握手时 H12 摇杆被 mute、界面显示「未连接」。本模块提供：
- 1.4.0 对齐的默认 LAUNCH_VR 命令
- 轮询 /quest3/connected 的等待
- 超时诊断文案（区分 param 未出现 / 仍为 false，并附 tmux 片段）
"""

from __future__ import annotations

import time
from typing import Callable, Optional

# 对齐 1.4.0 / readme：未设置环境变量时的硬默认
DEFAULT_LAUNCH_VR_REMOTE_CONTROL_CMD = (
    "roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch"
)

# Quest 握手门禁超时（秒）。略大于 tmux 拉起 + monitor 冷启动，短于用户可忍受的「假死」窗口。
VR_QUEST_CONNECT_TIMEOUT_SEC = 12.0
VR_QUEST_CONNECT_POLL_INTERVAL_SEC = 0.2
QUEST3_CONNECTED_PARAM = "/quest3/connected"
VR_REMOTE_CONTROL_SESSION_NAME = "vr_remote_control"


def resolve_launch_vr_cmd(env_value: Optional[str]) -> tuple[str, bool]:
    """返回 (launch_cmd, used_default)。空 / None 回退到 1.4.0 默认。"""
    if env_value is None or not str(env_value).strip():
        return DEFAULT_LAUNCH_VR_REMOTE_CONTROL_CMD, True
    return str(env_value).strip(), False


def reset_quest3_connected_for_new_session(
    *,
    set_param_fn: Callable[[str, bool], None],
) -> None:
    """启动新 VR 实例前强制把 /quest3/connected 置 False。

    旧 monitor 异常退出可能残留 True；若不复位，本次 launch 失败时
    wait_for_quest3_connected 会立刻放行，再次出现「未连通却切态/mute H12」。
    """
    set_param_fn(QUEST3_CONNECTED_PARAM, False)


def wait_for_quest3_connected(
    *,
    timeout_sec: float = VR_QUEST_CONNECT_TIMEOUT_SEC,
    poll_interval_sec: float = VR_QUEST_CONNECT_POLL_INTERVAL_SEC,
    has_param_fn: Callable[[str], bool],
    get_param_fn: Callable[[str, bool], bool],
    sleep_fn: Callable[[float], None] = time.sleep,
    monotonic_fn: Callable[[], float] = time.monotonic,
    is_shutdown_fn: Optional[Callable[[], bool]] = None,
    require_false_before_true: bool = False,
) -> bool:
    """轮询 /quest3/connected，True 时返回 True；超时或 ROS shutdown 返回 False。

    require_false_before_true=True 时，必须先在轮询中观测到 False（或参数尚未出现），
    之后再出现的 True 才算新鲜连通，用于尚未复位、需挡住历史 True 的场景。

    启动回调若已成功调用 reset_quest3_connected_for_new_session，应传
    require_false_before_true=False：复位事实已计入，首次轮询即可接受本次 True。
    否则在「复位 → sleep(3) 启动 session → 再等待」时序下，若握手在首次轮询前
    完成，等待器会一直等不到轮询内的 False，误杀正常 VR session。
    """
    deadline = monotonic_fn() + float(timeout_sec)
    seen_false_or_missing = not require_false_before_true
    while monotonic_fn() < deadline:
        if is_shutdown_fn is not None and is_shutdown_fn():
            return False
        try:
            has = has_param_fn(QUEST3_CONNECTED_PARAM)
            if not has:
                seen_false_or_missing = True
            else:
                value = bool(get_param_fn(QUEST3_CONNECTED_PARAM, False))
                if not value:
                    seen_false_or_missing = True
                elif seen_false_or_missing:
                    return True
        except Exception:
            # param 服务器瞬时不可用时继续等，由超时兜底
            pass
        sleep_fn(poll_interval_sec)
    return False


def describe_quest_connect_timeout(
    *,
    timeout_sec: float,
    used_default_launch: bool,
    launch_cmd: str,
    has_param: Optional[bool],
    connected_value: Optional[bool],
    tmux_pane_tail: str = "",
) -> str:
    """生成超时诊断日志，便于区分 launch 未起 / 发现或握手未完成。"""
    if has_param is None:
        param_status = "读取 /quest3/connected 失败（ROS param 不可用）"
    elif not has_param:
        param_status = (
            f"{QUEST3_CONNECTED_PARAM} 尚未出现 → monitor_quest3 可能未启动，"
            "或仍卡在空 IP 发现 / 10019 握手之前"
        )
    elif connected_value:
        param_status = f"{QUEST3_CONNECTED_PARAM}=True（异常：超时分支不应见到 True）"
    else:
        param_status = (
            f"{QUEST3_CONNECTED_PARAM}=False → monitor 已起但 Quest 未连通"
            "（空 IP 广播发现失败、15170 quest_client_* 未注册、或 10019 握手失败）"
        )

    default_note = (
        "LAUNCH_VR_REMOTE_CONTROL_CMD 未设置，已使用 1.4.0 默认 launch"
        if used_default_launch
        else "LAUNCH_VR_REMOTE_CONTROL_CMD 来自环境变量"
    )
    lines = [
        f"[vr_connect_gate] 等待 Quest 连通超时（{timeout_sec:.0f}s），"
        "取消进入 vr_remote_control，保持 stance，避免 H12 摇杆被 mute。",
        f"  - {default_note}",
        f"  - launch_cmd: {launch_cmd}",
        f"  - {param_status}",
        "  - 请检查: tmux capture-pane -pt vr_remote_control；"
        "Quest 与机器人同网段；h12_vr_launch.yaml 的 ip_address；"
        "UDP 15170 上 client_id 是否为 quest_client_*；10019 握手日志。",
    ]
    pane = (tmux_pane_tail or "").strip()
    if pane:
        # 只保留末尾若干行，避免刷屏
        tail_lines = pane.splitlines()[-20:]
        lines.append("  - tmux vr_remote_control 末尾输出:")
        lines.extend(f"      | {ln}" for ln in tail_lines)
    else:
        lines.append(
            "  - tmux vr_remote_control 无输出/不存在 → 更像 launch 未真正跑起来（H3）"
        )
    return "\n".join(lines)


def capture_tmux_pane_tail(session_name: str, run_fn: Callable) -> str:
    """读取 tmux 面板文本；失败返回空串。run_fn 签名同 subprocess.run。"""
    try:
        result = run_fn(
            ["tmux", "capture-pane", "-pt", session_name],
            capture_output=True,
            text=True,
            timeout=2,
        )
        if getattr(result, "returncode", 1) == 0:
            return result.stdout or ""
    except Exception:
        pass
    return ""
