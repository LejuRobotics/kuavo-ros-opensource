"""Scheduling for the TCPROS input thread; no mapping or control state."""
import os
import json
import sys
import ctypes
import threading
from pathlib import Path


def parse_cpus(value):
    cpus = set()
    for item in str(value).strip().split(","):
        if not item.strip():
            continue
        parts = item.strip().split("-")
        if len(parts) == 1:
            lo = hi = int(parts[0])
        elif len(parts) == 2:
            lo, hi = map(int, parts)
        else:
            raise ValueError("invalid CPU list")
        if lo < 0 or hi < lo or hi > 65535:
            raise ValueError("invalid CPU range")
        cpus.update(range(lo, hi + 1))
    return cpus


class MapperInputScheduler:
    """Apply once per receiving thread, including TCPROS reconnect threads.

    Without a validated runtime profile, affinity only narrows the inherited mask.
    A validated profile gives this thread an dedicated Mapper core within the launched stack.
    No real-time policy is introduced. Failure is diagnostic-only.
    """
    def __init__(self, cpus="", nice=None):
        self.profile = json.loads(os.environ.get("KUAVO_RUNTIME_PARTITION_JSON", "null"))
        self.cpus = cpus if self.profile is None else ",".join(map(str, self.profile["mapper_cpus"]))
        self.nice = nice if self.profile is None else self.profile["mapper_nice"]
        self.local = threading.local()
        if self.profile is not None:
            # Limits time another Python callback can retain the GIL. No signal,
            # filter coefficient, source clock or control rate is changed.
            sys.setswitchinterval(self.profile["python_switch_interval_sec"])

    def apply_current_thread(self):
        if getattr(self.local, "applied", False):
            return None
        self.local.applied = True
        result = dict(pid=os.getpid(), tid=threading.get_native_id(),
                      requested_cpus=str(self.cpus), requested_nice=self.nice,
                      errors=[])
        try:
            inherited = set(os.sched_getaffinity(0))
            result["inherited_cpus"] = sorted(inherited)
            if str(self.cpus).strip():
                # A failed isolation read must not silently admit a reserved core.
                isolated = parse_cpus(Path("/sys/devices/system/cpu/isolated").read_text())
                result["isolated_cpus"] = sorted(isolated)
                if self.profile is None:
                    selected = (parse_cpus(self.cpus) & inherited) - isolated
                else:
                    # Explicit profile was validated before process creation.
                    # Widen only this callback thread onto its dedicated core.
                    selected = parse_cpus(self.cpus)
                    reserved = set(self.profile["wbc_cpus"] + self.profile["driver_reserved_cpus"] + self.profile["ik_cpus"])
                    if selected & (reserved | set(self.profile["background_cpus"])):
                        raise ValueError("Mapper partition collides with protected control CPUs")
                if not selected:
                    raise ValueError("no permitted Mapper CPU")
                os.sched_setaffinity(0, selected)
                if self.profile is not None:
                    ctypes.CDLL(None).prctl(15, b"kuavo_mapper", 0, 0, 0)
        except (OSError, ValueError, AttributeError) as exc:
            result["errors"].append("affinity: " + str(exc))
        try:
            if self.nice is not None:
                target = int(self.nice)
                if not -5 <= target <= 19:
                    raise ValueError("nice must be within [-5, 19]")
                if os.sched_getscheduler(0) != os.SCHED_OTHER:
                    raise ValueError("nice adjustment requires SCHED_OTHER")
                os.setpriority(os.PRIO_PROCESS, 0, target)
        except (OSError, ValueError, AttributeError) as exc:
            result["errors"].append("nice: " + str(exc))
        try:
            result.update(actual_cpus=sorted(os.sched_getaffinity(0)),
                          policy=os.sched_getscheduler(0),
                          priority=os.sched_getparam(0).sched_priority,
                          nice=os.getpriority(os.PRIO_PROCESS, 0))
        except (OSError, AttributeError) as exc:
            result["errors"].append("readback: " + str(exc))
        return result
