#!/usr/bin/env python3
"""Run Quest/Pico MuJoCo regressions after sourcing the candidate ROS overlay.

Usage: python3 xa_torso_simulation_test.py OUTPUT_DIRECTORY [quest pico]
The hardware input receivers are replaced only by the protocol fixture.
"""
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import xmlrpc.client

package = Path(__file__).resolve().parents[1]
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=True)
tree = ET.parse(package/'launch/quest_velocity_ik.launch')
removed = []
for node in list(tree.getroot()):
    if (node.tag == 'node' and node.get('name') in ('monitor_quest3', 'monitor_quest3_cpp', 'robot_state_server')) or (node.tag == 'include' and node.get('file', '').endswith('/pico_input.launch')):
        removed.append(ET.tostring(node, encoding='unicode'))
        tree.getroot().remove(node)
assert len(removed) == 4
launch_file = out/'synthetic_input.launch'
tree.write(launch_file, encoding='unicode')
(out/'removed-network-nodes.txt').write_text('\n'.join(removed))
for source in sys.argv[2:] or ['quest', 'pico']:
    if source not in ('quest', 'pico'):
        raise ValueError(source)
    root = out/source
    if root.exists():
        raise RuntimeError('Use a fresh evidence directory: ' + str(root))
    root.mkdir()
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
               ROS_IP='127.0.0.1', KUAVO_RUNTIME_PARTITION='0', ROBOT_VERSION='62',
               ROS_HOME=str(root/'ros'))
    env.pop('ROS_HOSTNAME', None)
    with (root/'simulation.log').open('w') as log:
        proc = subprocess.Popen(['bash', str(package/'scripts/platform_roslaunch.sh'),
            str(package/'launch/load_kuavo_mujoco_velocity_ik.launch'),
            'visualize_humanoid:=false', 'reference_shm_key:=542421', 'owner_shm_key:=542422'],
            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            master = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
            for _ in range(150):
                if proc.poll() is not None:
                    raise RuntimeError('simulation exited: ' + str(root/'simulation.log'))
                try:
                    response = master.getParam('/xa_regression', '/real')
                    if response[0] == 1 and response[2] is False:
                        break
                except OSError:
                    pass
                time.sleep(.2)
            else:
                raise RuntimeError('simulation master timeout')
            time.sleep(8)
            with (root/'scenario.log').open('w') as runlog:
                subprocess.run(['python3', str(package/'test/xa_torso_simulation_scenario.py'),
                                source, str(root), str(launch_file)], env=env, stdout=runlog,
                               stderr=subprocess.STDOUT, check=True, timeout=300)
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=15)
    print(source + ' full-body reset and torso-mode simulation PASS', flush=True)
