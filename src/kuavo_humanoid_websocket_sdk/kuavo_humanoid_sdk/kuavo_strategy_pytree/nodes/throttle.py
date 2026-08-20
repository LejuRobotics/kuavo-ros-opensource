"""频率控制打印工具。

提供 ``throttle_print`` 按 key 限流终端输出，避免高频循环中的 print 刷屏。
"""
import time
from typing import Optional, Dict

_throttle_last: Dict[str, float] = {}


def throttle_print(key: str, *args, interval: float = 1.0, **kwargs) -> None:
    """按 key 限流的 print，同一 key 在 interval 秒内只输出一次。

    用法::

        throttle_print("walk_vel", "vel_x:", vel_x, interval=0.5)
        throttle_print("walk_vel", "vel_y:", vel_y)  # 与上行共用 key，共享限流
        throttle_print("arm_error", "pos_err:", err, interval=2.0)

    Args:
        key: 限流标识，相同的 key 共享限流计时器
        interval: 最小输出间隔（秒），默认 1.0
    """
    now = time.time()
    last = _throttle_last.get(key, 0.0)
    if now - last >= interval:
        _throttle_last[key] = now
        print(*args, **kwargs)


def throttle_reset(key: Optional[str] = None) -> None:
    """重置限流计时器。

    Args:
        key: 要重置的 key，不传则全部重置
    """
    global _throttle_last
    if key is None:
        _throttle_last.clear()
    else:
        _throttle_last.pop(key, None)
