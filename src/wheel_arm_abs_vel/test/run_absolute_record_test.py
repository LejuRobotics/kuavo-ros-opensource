#!/usr/bin/env python3
"""Exercise production record conversion/FK on four models, with an isolated ROS master."""
import os
import json
from pathlib import Path
import yaml
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
with tempfile.TemporaryDirectory(prefix='absolute-record-test-') as tmp:
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
                    ready = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/absolute_record_runner')[0] == 1
                except OSError:
                    pass
                if ready:
                    break
                time.sleep(0.1)
            if not ready:
                raise RuntimeError('isolated ROS master did not start')
            root = Path(__file__).resolve().parents[1]
            server = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
            code = 0
            for version in (62, 63, 400062, 400063):
                contract = subprocess.check_output([
                    sys.executable, str(root/'scripts/resolve_velocity_model.py'),
                    '--robot-version', str(version), '--assets', str(root.parent/'kuavo_assets'),
                    '--wheel-interface', str(root.parent/'humanoid-wheel-control/humanoid_wheel_interface'),
                    '--overlay-dir', str(root/'config')], env=env, text=True)
                assert server.setParam('/absolute_record_runner',
                    '/whole_body_velocity_ik/model_contract_json', contract)[0] == 1
                config = yaml.safe_load((root/'config/fixed_base_18d.yaml').read_text())
                assert server.setParam('/absolute_record_runner', '/absolute_record_test', config)[0] == 1
                result = subprocess.run([sys.argv[1], str(version)], env=env, timeout=30)
                if result.returncode:
                    code = result.returncode
                    break
        finally:
            if master.poll() is None:
                os.killpg(master.pid, signal.SIGINT)
                try:
                    master.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(master.pid, signal.SIGKILL)
                    master.wait()
sys.exit(code)
