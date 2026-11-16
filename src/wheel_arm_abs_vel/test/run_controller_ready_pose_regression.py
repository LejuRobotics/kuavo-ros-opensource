#!/usr/bin/env python3
"""Load launch parameters into a private ROS master and run an offline solver.

Launch files are parsed only: no input, controller, simulator or driver starts.
"""
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

import roslaunch.config
import rosparam

binary, model, kind = sys.argv[1:]
package = Path(__file__).resolve().parents[1]
with socket.socket() as sock:
    sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
os.environ['ROS_IP'] = '127.0.0.1'
os.environ.pop('ROS_HOSTNAME', None)
with tempfile.TemporaryDirectory(prefix='filter-regression-') as tmp:
    os.environ['ROS_HOME'] = tmp
    with open(Path(tmp)/'master.log', 'w') as log:
        master = subprocess.Popen(['roscore', '-p', str(port)], stdout=log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        try:
            client = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
            for _ in range(100):
                if master.poll() is not None: raise RuntimeError('private master exited')
                try:
                    if client.getPid('/filter_regression')[0] == 1: break
                except OSError: pass
                time.sleep(.1)
            else: raise RuntimeError('private master timeout')
            config = roslaunch.config.load_config_default(
                [(str(package/'launch/quest_velocity_ik.launch'),
                  (['robot_version:='+model] +
                   (['ee_type:=lejuclaw', 'unlock_transition_enabled:=true']
                    if kind == 'unlock' else [])))], None, verbose=False)
            for key, param in config.params.items(): rosparam.set_param_raw(key, param.value)
            subprocess.run([binary, kind], check=True, timeout=90)
        finally:
            if master.poll() is None: os.killpg(master.pid, signal.SIGINT)
            try: master.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(master.pid, signal.SIGTERM); master.wait(timeout=5)
