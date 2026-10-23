"""Keep one ROS master alive across all rounds of a simulator session."""

import subprocess
import time
from xmlrpc.client import ServerProxy, Transport

import rosgraph
from utils.shutdown_guard import ShutdownGuard
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
        self.guard = ShutdownGuard(0)

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
                print('[INFO] simulator session: reusing ROS master {} pid {}'.format(
                    self.uri, self.pid), flush=True)
                return self
            host, port = parse_http_host_and_port(self.uri)
            if not is_local_address(host):
                raise RuntimeError('ROS master {} is unavailable; refusing to '
                                   'replace a remote master'.format(self.uri))
            self.process = self.guard.add(subprocess.Popen(
                ['roscore', '-p', str(port)], start_new_session=True),
                graceful=True, process_group=True)
            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                if self.process.poll() is not None:
                    raise RuntimeError('session roscore exited with code {}'.format(
                        self.process.returncode))
                self.pid = self._probe()
                if self.pid is not None:
                    print('[INFO] simulator session: session ROS master {} pid {}; '
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
        # Close only the master we created, including separately grouped ROS children.
        self.guard.sweep()
        self.process = None

    def __exit__(self, *_exc):
        self.close()
