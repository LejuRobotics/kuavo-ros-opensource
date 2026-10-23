"""Bound one ROS init notification without leaking blocked RPC threads."""
import subprocess
import sys
import time

# The worker does not initialize a ROS node or own simulator resources.
# Killing it closes only this outgoing RPC, including a blocked master lookup.
INIT_CALL = """
import rospy
from std_srvs.srv import Trigger
try:
    rospy.wait_for_service('/simulator/init', timeout=1.0)
    result = rospy.ServiceProxy('/simulator/init', Trigger)()
    raise SystemExit(0 if result.success else 1)
except Exception:
    raise SystemExit(1)
"""


def notify_init(cancelled, timeout=3.0):
    """Return acknowledgement; always reap a timed-out or cancelled worker."""
    if cancelled():
        return False
    process = subprocess.Popen(
        [sys.executable, "-c", INIT_CALL], start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        deadline = time.monotonic() + timeout
        while not cancelled():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            try:
                return process.wait(timeout=min(0.1, remaining)) == 0
            except subprocess.TimeoutExpired:
                pass
        return False
    finally:
        if process.poll() is None:
            process.kill()
        process.wait()
