"""Bind newly spawned observation processes to the selected sidecar pool."""
import json
import os
import platform


def apply_background_fence():
    raw = os.environ.get("KUAVO_RUNTIME_PARTITION_JSON")
    if not raw:
        return
    profile = json.loads(raw)
    if 'sidecar_cpus' not in profile and platform.machine().lower() not in ('aarch64', 'arm64'):
        return
    cpus = set(profile.get("sidecar_cpus", profile["background_cpus"]))
    protected = set(profile["wbc_cpus"] + profile["ik_cpus"] +
                    profile["mapper_cpus"] + profile["driver_reserved_cpus"])
    if not cpus or cpus & protected:
        raise ValueError("invalid background fence")
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    os.sched_setaffinity(0, cpus)
    return True
