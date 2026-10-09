"""Keep one ROS master alive across all rounds of a model session."""

import os
import signal
import subprocess
import time
from xmlrpc.client import ServerProxy, Transport

import rosgraph
from rosgraph.network import is_local_address, parse_http_host_and_port


class _ProbeTransport(Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.0
        return connection


class ModelMaster:
    """Reuse an external master or own a roscore outside the round process group."""

    def __init__(self, timeout=15.0):
        self.uri = rosgraph.get_master_uri()
        self.timeout = timeout
        self.process = None
        self.pid = None

    def _probe(self):
        try:
            with ServerProxy(self.uri, transport=_ProbeTransport()) as master:
                code, _, pid = master.getPid('/model_simulator_master')
            return pid if code == 1 else None
        except (OSError, ValueError):
            return None

    def __enter__(self):
        try:
            self.pid = self._probe()
            if self.pid is not None:
                print('[INFO] model entry: reusing ROS master {} pid {}'.format(
                    self.uri, self.pid), flush=True)
                return self
            host, port = parse_http_host_and_port(self.uri)
            if not is_local_address(host):
                raise RuntimeError('ROS master {} is unavailable; refusing to '
                                   'replace a remote master'.format(self.uri))
            self.process = subprocess.Popen(
                ['roscore', '-p', str(port)], start_new_session=True)
            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                if self.process.poll() is not None:
                    raise RuntimeError('session roscore exited with code {}'.format(
                        self.process.returncode))
                self.pid = self._probe()
                if self.pid is not None:
                    print('[INFO] model entry: session ROS master {} pid {}; '
                          'preserved across resets'.format(self.uri, self.pid),
                          flush=True)
                    return self
                time.sleep(0.1)
            raise RuntimeError('timed out starting ROS master {}'.format(self.uri))
        except BaseException:
            self.close()
            raise

    def check(self):
        if ((self.process is not None and self.process.poll() is not None)
                or self._probe() != self.pid):
            raise RuntimeError('ROS master {} stopped or changed; restart the '
                               'evaluation session'.format(self.uri))

    def close(self):
        if self.process is None:
            return
        # Only this session's roscore group is ours. Never kill by process name.
        process, self.process = self.process, None
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(process.pid, signum)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                continue
            # Also remove descendants if their parent exited before them.
        process.wait(timeout=1.0)

    def __exit__(self, *_exc):
        self.close()
