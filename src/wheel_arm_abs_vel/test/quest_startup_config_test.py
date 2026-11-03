#!/usr/bin/env python3
"""Run only the Quest FSM against a private ROS master. Never start hardware."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

import roslaunch

PACKAGE = Path(__file__).resolve().parents[1]
BINARY = sys.argv[1]
with socket.socket() as sock:
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
env = dict(os.environ, ROS_MASTER_URI="http://127.0.0.1:%d" % port,
           ROS_HOSTNAME="127.0.0.1", ROS_IP="127.0.0.1", ROBOT_VERSION="34")
# An inherited/environment version must never override the explicit launch version.
socket.setdefaulttimeout(3)
master = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
caller = "/quest_startup_test"
children = []
results = []

def stop(p):
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait()

def setparam(key, value):
    assert master.setParam(caller, key, value)[0] == 1

def wait_until(predicate, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.05)
    raise AssertionError("timed out waiting for test condition")

with tempfile.TemporaryDirectory(prefix="quest-config-test-") as tmp:
    env.update(ROS_HOME=tmp, ROS_LOG_DIR=tmp, HOME=tmp)
    with open(Path(tmp)/"master.log", "w") as log:
        core = subprocess.Popen(["roscore", "-p", str(port)], env=env, stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        children.append(core)
        def master_ready():
            try:
                return master.getPid(caller)[0] == 1
            except OSError:
                return False
        try:
            wait_until(master_ready)
            for version in (62, 63, 400062):
                cfg = roslaunch.config.load_config_default(
                    [(str(PACKAGE/"launch/quest_velocity_ik.launch"),
                      ["robot_version:=%d" % version, "ip_address:=127.0.0.1"])],
                    None, verbose=False)
                base = {k: v.value for k, v in cfg.params.items()}
                prefix = "/humanoid_quest_control_with_arm/"
                ref = base[prefix + "referenceFile"]
                for case in ("quest_first", "controller_first", "missing_version",
                             "missing_reference", "missing_gait", "wrong_gait", "invalid_version", "wrong_reference",
                             "controller_mismatch", "late_controller_mismatch"):
                    # Fresh isolated parameter state; no node from the preceding case remains.
                    print("CASE", version, case, flush=True)
                    for key in master.getParamNames(caller)[2]:
                        master.deleteParam(caller, key)
                    setparam("/use_sim_time", False)
                    params = dict(base)
                    if case == "missing_version":
                        del params[prefix+"robot_version"]
                    if case == "missing_reference":
                        del params[prefix+"referenceFile"]
                    if case == "missing_gait":
                        del params[prefix+"gaitCommandFile"]
                    if case == "wrong_gait":
                        params[prefix+"gaitCommandFile"] = params[prefix+"gaitCommandFile"].replace(
                            "kuavo_v%d" % version, "kuavo_v%d" % ((63 if version == 62 else 62)))
                    if case == "invalid_version":
                        params[prefix+"robot_version"] = 34
                    if case == "wrong_reference":
                        params[prefix+"referenceFile"] = ref.replace(
                            "kuavo_v%d" % version, "kuavo_v%d" % ((63 if version == 62 else 62)))
                    for key, value in params.items():
                        setparam(key, value)
                    if case == "missing_version":
                        assert not master.hasParam(caller, prefix+"robot_version")[2]
                    if case == "missing_reference":
                        assert not master.hasParam(caller, prefix+"referenceFile")[2]
                    if case in ("controller_first", "controller_mismatch"):
                        setparam("/robot_version", version if case == "controller_first" else (63 if version == 62 else 62))
                        setparam("/referenceFile", ref)
                    path = Path(tmp)/("%d-%s.log" % (version, case))
                    with path.open("w") as out:
                        # Exercise the production logging pipeline's failure propagation.
                        wrapper = PACKAGE.parent/"humanoid-control/humanoid_controllers/scripts/start_node.sh"
                        proc = subprocess.Popen(["bash", "-o", "pipefail", str(wrapper), BINARY,
                            "__name:=humanoid_quest_control_with_arm"], env=env, stdout=out,
                            stderr=subprocess.STDOUT, start_new_session=True)
                        children.append(proc)
                        try:
                            if case in ("quest_first", "controller_first", "late_controller_mismatch"):
                                wait_until(lambda: proc.poll() is not None or
                                    master.hasParam(caller, prefix+"resolved_robot_version")[2])
                                assert proc.poll() is None, path.read_text()
                                assert master.getParam(caller, prefix+"resolved_robot_version")[2] == version
                                assert master.getParam(caller, prefix+"resolved_reference_file")[2] == ref
                                if case != "controller_first":
                                    assert not master.hasParam(caller, "/robot_version")[2]
                                    wait_until(lambda: "waiting for matching controller" in path.read_text())
                                    setparam("/robot_version", version if case == "quest_first" else (63 if version == 62 else 62))
                                    setparam("/referenceFile", ref)
                                if case == "late_controller_mismatch":
                                    wait_until(lambda: proc.poll() is not None)
                                    assert proc.returncode != 0
                                    assert "/robot_version mismatch" in path.read_text()
                                else:
                                    wait_until(lambda: "input callbacks enabled" in path.read_text())
                                    time.sleep(.1)
                                    assert proc.poll() is None, path.read_text()
                                    assert "kuavo_v%d/kuavo.json" % version in path.read_text()
                            else:
                                wait_until(lambda: proc.poll() is not None)
                                assert proc.returncode != 0, path.read_text()
                                assert "[QuestStartup]" in path.read_text(), path.read_text()
                                expected = {
                                    "missing_version": "explicit ~robot_version",
                                    "missing_reference": "explicit ~referenceFile",
                                    "missing_gait": "explicit ~gaitCommandFile",
                                    "wrong_gait": "gaitCommandFile must be the readable matching S",
                                    "invalid_version": "explicit ~robot_version",
                                    "wrong_reference": "readable matching S",
                                    "controller_mismatch": "/robot_version mismatch",
                                }[case]
                                assert expected in path.read_text(), path.read_text()
                            assert "kuavo_v34/kuavo.json" not in path.read_text()
                            assert "Creating new instance of HumanoidInterfaceDrake" not in path.read_text()
                            results.append({"version": version, "case": case, "result": "PASS"})
                        except Exception:
                            print(path.read_text(), file=sys.stderr)
                            raise
                        finally:
                            stop(proc)
            print(json.dumps(results, indent=2))
            print("PASS: 20 isolated FSM startup/configuration cases; no hardware or input source started")
        finally:
            for child in reversed(children):
                stop(child)
