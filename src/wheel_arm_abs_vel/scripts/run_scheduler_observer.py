#!/usr/bin/env python3
import legacy_dependencies
"""ROS-launch-owned observer: starts with Quest, stops with that launch.
No robot commands, service calls, dependency provisioning, or scheduler changes
outside the observer's own threads. No time limit while waiting for FOLLOW.
"""
import argparse
import fcntl
import os
from pathlib import Path
import sys
import time
import rospy
from tools import scheduler_capture

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", default=str(Path.home()/".ros/kuavo_scheduler_flight"))
    parser.add_argument("--duration", type=float, default=180)
    parser.add_argument("--max-mib", type=int, default=256)
    args = parser.parse_args(rospy.myargv()[1:])
    if not 0 < args.duration <= 300 or not 1 <= args.max_mib <= 256:
        parser.error("duration must be in (0,300], max-mib in [1,256]")
    root = Path(args.output_root).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    lock = (root/"observer.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError("A scheduler observer already owns this output root: "+str(root))
    args.output = str(root/("capture_"+time.strftime("%Y%m%d_%H%M%S")+"_"+str(os.getpid())))
    args.wait = 0  # User can wait before enabling; acquisition starts only in FOLLOW.
    args.gil_binding = str(Path(__file__).parent/"tools/scheduler_gil_binding.json")
    try:
        return scheduler_capture.run(args)
    finally:
        lock.close()

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        sys.stderr.write("[sched-observer] ERROR: no valid scheduler capture: "+str(error)+"\n")
        raise
