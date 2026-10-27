#!/usr/bin/env python3
"""Input-terminal 'o': the same initialization request as the VR long-A gesture."""
import contextlib
import fcntl
import os
import select
import subprocess
import termios
import time
import yaml

SERVICE = '/humanoid_controller/real_initial_start'


@contextlib.contextmanager
def terminal_input(path=None):
    # roslaunch calls setsid() for children: /dev/tty then has no controlling
    # terminal, but the inherited interactive stdin still identifies the TTY.
    if path is None:
        path = os.ttyname(0) if os.isatty(0) else '/dev/tty'
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    previous = None
    try:
        # Only one helper may consume a given terminal.
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        previous = termios.tcgetattr(fd)
        current = termios.tcgetattr(fd)
        current[3] &= ~(termios.ICANON | termios.ECHO)
        # Keep ISIG: Ctrl+C still shuts down roslaunch normally.
        current[6][termios.VMIN] = 0
        current[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, current)
        yield fd
    finally:
        if previous is not None:
            termios.tcsetattr(fd, termios.TCSANOW, previous)
        os.close(fd)


class KeyGate:
    def __init__(self):
        self.last_o = float('-inf')

    def accept(self, data, now):
        if b'o' not in data.lower():
            return False
        accepted = now - self.last_o >= 1.0
        self.last_o = now
        return accepted


def request_initial_start():
    """Bound the RPC; do not retry or fall back to bypassing hardware readiness."""
    try:
        result = subprocess.run(['rosservice', 'call', SERVICE, '{}'],
                                stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True, timeout=3.0)
        if result.returncode:
            return False, result.stderr.strip() or 'initialization service unavailable'
        response = yaml.safe_load(result.stdout)
        if not isinstance(response, dict) or response.get('success') is not True:
            return False, str(response)
        return True, str(response.get('message', ''))
    except (OSError, subprocess.TimeoutExpired, yaml.YAMLError) as error:
        return False, str(error)


def main():
    import rospy
    rospy.init_node('absolute_terminal_start')
    try:
        with terminal_input() as fd:
            gate = KeyGate()
            rospy.loginfo("Press o in this input terminal to confirm initialization (same as long A). "
                          "X+A still controls arm following. Leave at least one second between presses.")
            while not rospy.is_shutdown():
                if not select.select([fd], [], [], 0.1)[0]:
                    continue
                data = os.read(fd, 1024)
                if not data:
                    break
                if gate.accept(data, time.monotonic()):
                    ok, message = request_initial_start()
                    (rospy.loginfo if ok else rospy.logwarn)(
                        'Initialization %s: %s', 'accepted' if ok else 'not confirmed', message)
                    # Discard repeats buffered during the RPC; never queue confirmations.
                    termios.tcflush(fd, termios.TCIFLUSH)
                    gate.last_o = time.monotonic()
    except (OSError, termios.error) as error:
        rospy.logwarn('Terminal o unavailable (%s); use an interactive input terminal or long A.', error)


if __name__ == '__main__':
    main()
