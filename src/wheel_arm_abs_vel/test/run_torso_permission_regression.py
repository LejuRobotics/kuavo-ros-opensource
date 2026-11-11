#!/usr/bin/env python3
"""Run the production solver fixture against a private localhost ROS master."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

binary, model, scenario = sys.argv[1:]
package = Path(__file__).resolve().parents[1]
with socket.socket() as sock:
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
           ROS_IP='127.0.0.1')
env.pop('ROS_HOSTNAME', None)
with tempfile.TemporaryDirectory(prefix='torso-regression-') as tmp:
    env['ROS_HOME'] = tmp
    with open(Path(tmp) / 'master.log', 'w') as log:
        master = subprocess.Popen(['roscore', '-p', str(port)], env=env,
                                  stdout=log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        try:
            client = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
            for _ in range(100):
                if master.poll() is not None:
                    raise RuntimeError('private ROS master exited')
                try:
                    if client.getPid('/torso_regression')[0] == 1:
                        break
                except OSError:
                    pass
                time.sleep(.1)
            else:
                raise RuntimeError('private ROS master timeout')
            # The node constructor requires the validated model contract that
            # production launches provide; resolve it read-only like the
            # absolute-record fixture does.
            contract = subprocess.check_output([
                sys.executable, str(package/'scripts/resolve_velocity_model.py'),
                '--robot-version', model,
                '--assets', str(package.parent/'kuavo_assets'),
                '--wheel-interface',
                str(package.parent/'humanoid-wheel-control/humanoid_wheel_interface'),
                '--overlay-dir', str(package/'config')], env=env, text=True)
            client.setParam('/torso_regression',
                            '/whole_body_velocity_ik/model_contract_json', contract)
            subprocess.run(['rosparam', 'load', str(package / 'config/fixed_base_18d.yaml'),
                            '/torso_permission_test'], env=env, check=True)
            subprocess.run([binary, model, scenario], env=env, check=True, timeout=90)
        finally:
            if master.poll() is None:
                os.killpg(master.pid, signal.SIGINT)
            try:
                master.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(master.pid, signal.SIGTERM)
                master.wait(timeout=5)
