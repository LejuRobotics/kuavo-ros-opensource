import math
import os
import sys
from pathlib import Path
mem_mb = int(next(s.split()[1] for s in Path('/proc/meminfo').read_text().splitlines() if s.startswith('MemAvailable:'))) // 1024
cpus = len(os.sched_getaffinity(0))
for limit, usage in [('/sys/fs/cgroup/memory.max', '/sys/fs/cgroup/memory.current'), ('/sys/fs/cgroup/memory/memory.limit_in_bytes', '/sys/fs/cgroup/memory/memory.usage_in_bytes')]:
    try:
        mem_mb = min(mem_mb, max(0, int(Path(limit).read_text()) - int(Path(usage).read_text())) // 2**20)
    except (OSError, ValueError):
        pass
try:
    quota, period = Path('/sys/fs/cgroup/cpu.max').read_text().split()
    if quota != 'max':
        cpus = min(cpus, max(1, math.ceil(int(quota) / int(period))))
except (OSError, ValueError):
    try:
        quota = int(Path('/sys/fs/cgroup/cpu/cpu.cfs_quota_us').read_text())
        period = int(Path('/sys/fs/cgroup/cpu/cpu.cfs_period_us').read_text())
        if quota > 0:
            cpus = min(cpus, max(1, math.ceil(quota / period)))
    except (OSError, ValueError):
        pass
# Noetic's smaller packages may request a lower per-job budget and more packages.
job_mb = int(sys.argv[1]) if len(sys.argv) > 1 else 2048
jobs = max(1, min(cpus, (mem_mb - 2048) // job_mb))
packages = min(jobs, int(sys.argv[2])) if len(sys.argv) > 2 else max(1, min(4, jobs // 2))
print(jobs, max(1, packages), max(1024, mem_mb - 2048))
