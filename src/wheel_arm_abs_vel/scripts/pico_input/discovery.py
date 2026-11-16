"""Pico discovery on active IPv4 broadcast interfaces, without ROS dependencies."""
import ipaddress
import json
import subprocess


def interface_pairs(interfaces):
    pairs = set()
    for interface in interfaces:
        flags = interface.get("flags", [])
        if "UP" not in flags or "BROADCAST" not in flags or "LOOPBACK" in flags:
            continue
        if interface.get("operstate") == "DOWN":
            continue
        for address in interface.get("addr_info", []):
            if address.get("family") != "inet":
                continue
            try:
                local = ipaddress.IPv4Address(address["local"])
                network = ipaddress.IPv4Network(
                    "%s/%s" % (local, address["prefixlen"]), strict=False)
                if (local.is_loopback or local.is_link_local or local.is_unspecified
                        or local.is_multicast or network.prefixlen > 30):
                    continue
                pairs.add((str(local), str(network.broadcast_address)))
            except (KeyError, ValueError):
                continue
    return sorted(pairs)


def discovery_pairs(advertise_ip="", broadcast_ip=""):
    if bool(advertise_ip) != bool(broadcast_ip):
        raise ValueError("advertise_ip and broadcast_ip must both be supplied or both empty")
    if advertise_ip:
        return [(str(ipaddress.IPv4Address(advertise_ip)),
                 str(ipaddress.IPv4Address(broadcast_ip)))]
    output = subprocess.check_output(
        ["ip", "-j", "-4", "addr", "show"], timeout=2, text=True)
    return interface_pairs(json.loads(output))


def announce(sock, pairs, eef_type, on_error):
    # Match the humanoid broadcaster: one advertisement per interface, so the
    # headset receives an address reachable on its own network.
    for local, broadcast in pairs:
        packet = {"data": {"robot_name": "KUAVO", "robot_ip": local,
                           "eef_type": eef_type}}
        try:
            sock.sendto(json.dumps(packet).encode(), (broadcast, 8443))
        except OSError as error:
            on_error(local, error)
