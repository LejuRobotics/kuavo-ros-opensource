#!/usr/bin/env python3
"""Run the software bridge on an isolated loopback ROS master, without hardware."""
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

with socket.socket() as s:
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
           ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1')
with tempfile.TemporaryDirectory(prefix='sg100-bridge-test-') as tmp:
    env['ROS_HOME'] = tmp
    env['ROS_LOG_DIR'] = tmp
    with open(os.path.join(tmp, 'master.txt'), 'w') as log:
        master = subprocess.Popen(['roscore', '-p', str(port)], env=env,
                                  stdout=log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        try:
            ready = False
            for _ in range(100):
                if master.poll() is not None:
                    break
                try:
                    ready = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/sg100_test')[0] == 1
                except OSError:
                    pass
                if ready:
                    break
                time.sleep(0.1)
            if not ready:
                raise RuntimeError('isolated ROS master did not start')
            result = subprocess.run([sys.argv[1]], env=env, timeout=25)
            code = result.returncode
        finally:
            if master.poll() is None:
                os.killpg(master.pid, signal.SIGINT)
                try:
                    master.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(master.pid, signal.SIGKILL)
                    master.wait()
sys.exit(code)
