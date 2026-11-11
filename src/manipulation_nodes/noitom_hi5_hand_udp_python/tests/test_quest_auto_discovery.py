#!/usr/bin/env python3
"""Automatic discovery regression tests without ROS or network access."""
import ast
import importlib.util
import queue
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
spec = importlib.util.spec_from_file_location("quest_udp_server", SCRIPTS / "udp_server.py")
udp_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(udp_module)
UDPServer = udp_module.UDPServer
source = SCRIPTS / "monitor_quest3.py"
tree = ast.parse(source.read_text())
node_class = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                  and n.name == "Quest3BoneFramePublisher")


def method(name, scope):
    function = next(n for n in node_class.body if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), scope)
    return scope[name]


class StateClientTest(unittest.TestCase):
    def test_only_quest_clients_and_deduplicate_addresses(self):
        server = UDPServer(force_port=False)
        server.clients = {
            "quest_client_1": server.ClientInfo(("192.168.0.196", 1234), 0),
            "quest_client_2": server.ClientInfo(("192.168.0.196", 4321), 0),
            "test_client": server.ClientInfo(("192.168.0.197", 1234), 0),
        }
        self.assertEqual(server.get_quest_client_ips(), ["192.168.0.196"])
        server.clients.clear()
        self.assertEqual(server.get_quest_client_ips(), [])

    def test_ambiguous_and_dual_clients_do_not_auto_select(self):
        resolve = method("_state_server_quest_ip", {"rospy": SimpleNamespace(logwarn_throttle=Mock())})
        udp = SimpleNamespace(get_quest_client_ips=Mock(return_value=["a", "b"]))
        node = SimpleNamespace(tf_prefix="", connected_param="",
                               robot_state_server=SimpleNamespace(udp_server=udp))
        self.assertIsNone(resolve(node))
        udp.get_quest_client_ips.return_value = ["a"]
        self.assertEqual(resolve(node), "a")
        node.tf_prefix = "right"
        self.assertIsNone(resolve(node))
        node.tf_prefix = ""
        node.connected_param = "/vr_left/connected"
        self.assertIsNone(resolve(node))
        node.connected_param = ""
        node.robot_state_server = None
        self.assertIsNone(resolve(node))


class DiscoveryTest(unittest.TestCase):
    def run_discovery(self, results, broadcast_ip=None, shutdown=False, ports_busy=False):
        sockets = []
        def make_socket(*args):
            sock = Mock()
            if ports_busy:
                sock.bind.side_effect = OSError("address in use")
            sockets.append(sock)
            return sock
        candidates = Mock()
        candidates.get.side_effect = ([broadcast_ip] if broadcast_ip else []) + [queue.Empty] * 4
        ros = SimpleNamespace(is_shutdown=lambda: shutdown, loginfo=Mock(), logwarn=Mock())
        scope = {
            "socket": SimpleNamespace(socket=make_socket, AF_INET=2, SOCK_DGRAM=2),
            "queue": SimpleNamespace(Queue=lambda **kwargs: candidates, Empty=queue.Empty),
            "threading": SimpleNamespace(Thread=Mock()), "rospy": ros,
            "time": SimpleNamespace(monotonic=Mock(side_effect=[0.0, 0.0, 4.0, 8.0, 8.0])),
        }
        discover = method("broadcast_robot_info_and_wait_for_quest3", scope)
        node = SimpleNamespace(exit_listen_thread_for_quest3_broadcast=False,
                               socket_lock=threading.Lock(), sock=None, server_address=None,
                               listen_for_quest3_broadcasts=Mock(),
                               _periodic_robot_info_broadcaster=Mock(),
                               _state_server_quest_ip=Mock(return_value="192.168.0.196"),
                               send_initial_message=Mock(side_effect=results))
        def setup(ip, port):
            node.server_address = (ip, port)
            node.sock = make_socket()
        node.setup_socket = Mock(side_effect=setup)
        result = discover(node)
        return result, node, sockets, scope

    def test_registered_client_connects_without_discovery_packet(self):
        result, node, sockets, scope = self.run_discovery([True])
        self.assertTrue(result)
        node.setup_socket.assert_called_once_with("192.168.0.196", 10019)
        node.send_initial_message.assert_called_once_with(max_retries=1)
        self.assertTrue(node.exit_listen_thread_for_quest3_broadcast)
        for call in scope["threading"].Thread.call_args_list:
            self.assertIn("target", call.kwargs)
        self.assertEqual(len(sockets), 12)

    def test_failed_handshake_closes_socket_and_retries(self):
        result, node, sockets, scope = self.run_discovery([False, True])
        self.assertTrue(result)
        self.assertEqual(node.send_initial_message.call_count, 2)
        self.assertEqual(node.setup_socket.call_count, 2)
        sockets[11].close.assert_called_once()
        sockets[12].close.assert_not_called()

    def test_broadcast_packet_uses_same_connection_owner(self):
        result, node, sockets, scope = self.run_discovery([True], broadcast_ip="192.168.0.197")
        self.assertTrue(result)
        node.setup_socket.assert_called_once_with("192.168.0.197", 10019)

    def test_shutdown_does_not_attempt_handshake(self):
        result, node, sockets, scope = self.run_discovery([], shutdown=True)
        self.assertFalse(result)
        node.send_initial_message.assert_not_called()
        self.assertTrue(node.exit_listen_thread_for_quest3_broadcast)

    def test_busy_discovery_ports_still_allow_state_client_fallback(self):
        result, node, sockets, scope = self.run_discovery([True], ports_busy=True)
        self.assertTrue(result)
        node.setup_socket.assert_called_once_with("192.168.0.196", 10019)
        for sock in sockets[:11]:
            sock.close.assert_called_once()

    def test_listener_closes_socket_without_connecting(self):
        scope = {"rospy": SimpleNamespace(is_shutdown=lambda: False)}
        listen = method("listen_for_quest3_broadcasts", scope)
        sock = Mock()
        node = SimpleNamespace(exit_listen_thread_for_quest3_broadcast=True)
        listen(node, sock, 11000, Mock())
        sock.close.assert_called_once()
        sock.recvfrom.assert_not_called()


if __name__ == "__main__":
    unittest.main()
