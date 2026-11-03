import importlib.util
import json
from pathlib import Path
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("discovery", Path(__file__).resolve().parents[1] / "scripts/pico_input/discovery.py")
d = importlib.util.module_from_spec(spec)
spec.loader.exec_module(d)


def iface(ip, prefix=24, flags=None, state="UP"):
    return {"flags": flags if flags is not None else ["UP", "BROADCAST"],
            "operstate": state,
            "addr_info": [{"family": "inet", "local": ip, "prefixlen": prefix}]}


class DiscoveryTest(unittest.TestCase):
    def test_multiple_subnets_and_real_23_prefix(self):
        self.assertEqual(d.interface_pairs([iface("10.10.30.176", 23), iface("192.168.26.1")]),
                         [("10.10.30.176", "10.10.31.255"), ("192.168.26.1", "192.168.26.255")])

    def test_skip_ineligible_and_deduplicate(self):
        self.assertEqual(d.interface_pairs([
            iface("127.0.0.1", 8, ["UP", "LOOPBACK"]), iface("169.254.1.2"),
            iface("10.0.0.2", state="DOWN"), iface("10.0.0.3", 32),
            iface("192.168.1.2"), iface("192.168.1.2")]),
            [("192.168.1.2", "192.168.1.255")])

    def test_auto_reads_interfaces_without_default_route(self):
        with patch.object(d.subprocess, "check_output", return_value=json.dumps([iface("10.0.0.2")])) as read:
            self.assertEqual(d.discovery_pairs(), [("10.0.0.2", "10.0.0.255")])
            self.assertEqual(read.call_args[0][0], ["ip", "-j", "-4", "addr", "show"])

    def test_explicit_override_and_invalid_pair(self):
        with patch.object(d.subprocess, "check_output", side_effect=AssertionError):
            self.assertEqual(d.discovery_pairs("10.0.0.1", "10.0.0.255"), [("10.0.0.1", "10.0.0.255")])
        for args in [("10.0.0.1", ""), ("", "10.0.0.255"), ("bad", "bad")]:
            with self.assertRaises(ValueError):
                d.discovery_pairs(*args)

    def test_failed_interface_does_not_block_other_advertisements(self):
        class Socket:
            sent = []
            def sendto(self, payload, address):
                if address[0] == "10.0.0.255":
                    raise OSError("unreachable")
                self.sent.append((json.loads(payload), address))
        sock = Socket()
        errors = []
        d.announce(sock, [("10.0.0.1", "10.0.0.255"), ("10.1.0.1", "10.1.0.255")],
                   "lejuclaw", lambda *args: errors.append(args))
        self.assertEqual(len(errors), 1)
        self.assertEqual(sock.sent[0][0]["data"]["robot_ip"], "10.1.0.1")
        self.assertEqual(sock.sent[0][1], ("10.1.0.255", 8443))


if __name__ == "__main__":
    unittest.main()
