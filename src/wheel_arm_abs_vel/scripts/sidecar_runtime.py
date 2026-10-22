#!/usr/bin/env python3
"""Launch a private observation node after applying its non-realtime CPU pool."""
import os
import sys
from tools.background_fence import apply_background_fence


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: sidecar_runtime.py executable [args...]')
    if apply_background_fence():
        os.setpriority(os.PRIO_PROCESS, 0, 10)
    os.execvp(sys.argv[1], sys.argv[1:])


if __name__ == '__main__':
    main()
