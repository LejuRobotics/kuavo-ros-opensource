#!/usr/bin/env python3
"""Read absolute input end-effector selection from the controller's kuavo.json."""
import argparse
import json
import os
import pwd
from pathlib import Path
import sys

SUPPORTED = frozenset(('qiangnao', 'qiangnao_touch', 'heiman', 'lejuclaw',
                       'jodell', 'revo2', 'linker_hand', 'none'))


def checked(value):
    if not isinstance(value, str) or value.strip().lower() not in SUPPORTED:
        raise ValueError('unsupported absolute EndEffectorType: %r' % (value,))
    return value.strip().lower()


def resolve(config, override='auto'):
    if override not in ('', 'auto'):
        return checked(override)
    pair = config.get('EndEffectorType')
    if not isinstance(pair, list) or len(pair) != 2:
        raise ValueError('kuavo.json EndEffectorType must contain [left, right]')
    left, right = (checked(value) for value in pair)
    if left != right:
        raise ValueError('absolute Mapper requires matching left/right EndEffectorType; got %r' % pair)
    return left


# CAN SDK classes describe installed hardware independently of ROBOT_VERSION.
CAN_TYPES = {'lejuclaw':'lejuclaw', 'revo1_hand':'qiangnao', 'revo2_hand':'revo2'}


def machine_config_directory():
    explicit = os.environ.get('KUAVO_MACHINE_CONFIG_DIR')
    if explicit:
        return Path(explicit)
    home = Path.home()
    if os.geteuid() == 0 and os.environ.get('SUDO_USER') not in (None, '', 'root'):
        home = Path(pwd.getpwnam(os.environ['SUDO_USER']).pw_dir)
    return home/'.config/lejuconfig'


def machine_pair(directory):
    """Read the SDK's device inventory; never open CAN/serial or send a command."""
    import yaml
    directory = Path(directory)
    files = [directory/'canbus_device_config.yaml', directory/'canbus_device_cofig.yaml']
    path = next((p for p in files if p.exists()), None)
    if path is None:
        return None, None
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError('CAN device inventory must be a mapping: ' + str(path))
    found, active, identities = [], [], set()
    for key, entries in data.items():
        if not key.endswith('_devices'):
            continue
        if not isinstance(entries, list):
            raise ValueError('invalid CAN device list: ' + key)
        for device in entries:
            if not isinstance(device, dict):
                raise ValueError('invalid CAN device entry: ' + key)
            kind = device.get('class')
            if kind not in CAN_TYPES:
                continue
            found.append(kind)
            ignored = device.get('ignore', False)
            if not isinstance(ignored, bool):
                raise ValueError('CAN ignore must be a boolean')
            if ignored:
                continue
            identity = (key, device.get('device_id'))
            if not isinstance(identity[1], int) or isinstance(identity[1], bool) or identity in identities:
                raise ValueError('missing or duplicate CAN end-effector identity')
            identities.add(identity)
            active.append(CAN_TYPES[kind])
    if not found:
        return None, None  # E.g. serial SG100: retain the model's explicit type.
    if not active:
        return None, None  # Ignored CAN devices do not identify serial/non-CAN hardware.
    if len(active) != 2 or active[0] != active[1]:
        raise ValueError('CAN inventory requires two matching active end effectors; got %r' % active)
    return active, str(path)


def machine_mode(value, inspect_inventory=True):
    if str(value).lower() in ('true','1'): return True
    if str(value).lower() in ('false','0'): return False
    if value != 'auto': raise ValueError('machine mode must be auto/true/false')
    # Only read the current controller's real/simulation flag, never a cached
    # end-effector type. Launch parsing without a running controller stays model-only.
    import xmlrpc.client
    class TimeoutTransport(xmlrpc.client.Transport):
        def make_connection(self, host):
            connection = super().make_connection(host); connection.timeout = 2; return connection
    try:
        server = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI','http://localhost:11311'),
                                         transport=TimeoutTransport())
        code, _, real = server.getParam('/end_effector_resolver', '/real')
        if code == 1 and isinstance(real, bool): return real
    except (OSError, xmlrpc.client.Error):
        pass
    if not inspect_inventory:
        return False  # Explicit selection does not depend on CAN inventory.
    pair, _ = machine_pair(machine_config_directory())
    if pair is not None:
        raise ValueError('controller /real is unavailable; start the controller first or explicitly select machine_end_effector:=true/false')
    return False


def select(config, override='auto', use_machine=False, directory=None):
    if override not in ('','auto'):
        kind = checked(override)
        return [kind,kind], 'explicit ee_type'
    if use_machine:
        pair, source = machine_pair(machine_config_directory() if directory is None else directory)
        if pair is not None: return pair, source
    kind = resolve(config)
    return [kind,kind], 'model kuavo.json'


def validate_contract(text, version, real, override='auto'):
    data = json.loads(text)
    if (type(data.get('robot_version')) is not int or data['robot_version'] != version
            or type(data.get('real')) is not bool or data['real'] != real
            or not isinstance(data.get('source'), str) or not data['source']):
        raise ValueError('controller end-effector contract model/mode mismatch')
    kind = resolve({'EndEffectorType': data['types']})
    if override not in ('', 'auto') and checked(override) != kind:
        raise ValueError('Mapper ee_type disagrees with the controller contract')
    return kind


def bridge_contract(override):
    import xmlrpc.client
    class TimeoutTransport(xmlrpc.client.Transport):
        def make_connection(self, host):
            conn = super().make_connection(host); conn.timeout = 2; return conn
    server = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'),
                                     transport=TimeoutTransport())
    try:
        code, _, text = server.getParam('/absolute_bridge', '/absolute_velocity_ik/end_effector_contract')
        if code != 1: return None
        vc, _, version = server.getParam('/absolute_bridge', '/robot_version')
        rc, _, real = server.getParam('/absolute_bridge', '/real')
    except (OSError, xmlrpc.client.Error):
        return None
    if vc != 1 or rc != 1: raise ValueError('controller contract lacks model/mode')
    return validate_contract(text, version, real, override)


def resolve_runtime(config, override='auto'):
    import rospy
    contract = rospy.get_param('/absolute_velocity_ik/end_effector_contract', None)
    if contract is not None:
        return validate_contract(contract, rospy.get_param('/robot_version'),
                                 rospy.get_param('/real', False), override)
    pair, source = select(config, override, use_machine=rospy.get_param('/real', False) is True)
    rospy.loginfo('[EndEffectorSelection] types=%s source=%s', pair, source)
    return pair[0]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--ee-type', default=os.environ.get('KUAVO_END_EFFECTOR_TYPE', 'auto'))
    parser.add_argument('--bridge', action='store_true')
    parser.add_argument('--machine-mode', default='false')
    parser.add_argument('--contract-version', type=int)
    args, remaps = parser.parse_known_args(argv)
    if any(':=' not in arg for arg in remaps):
        parser.error('only ROS remappings may follow resolver arguments')
    try:
        config = json.loads(Path(args.config).read_text())
        contracted = bridge_contract(args.ee_type) if args.bridge else None
        if contracted is not None:
            kind = contracted; pair = [kind, kind]; source = 'controller contract'; real = True
        else:
            real = machine_mode(args.machine_mode, inspect_inventory=args.ee_type in ('', 'auto'))
            pair, source = select(config, args.ee_type, use_machine=real)
            kind = pair[0]
    except (OSError, ValueError, TypeError, AttributeError) as exc:
        print('[absolute end effector] ERROR: %s' % exc, file=sys.stderr, flush=True)
        return 2
    print('[absolute end effector] type=%s source=%s' %
          (kind, args.config if source == 'model kuavo.json' else source),
          file=sys.stderr, flush=True)
    if args.contract_version is not None:
        if args.contract_version <= 0: raise ValueError('positive robot version required')
        sys.stdout.write(json.dumps(dict(robot_version=args.contract_version, types=pair, source=source, real=real)))
        return 0
    if args.bridge:
        if kind == 'heiman':
            os.execvp('rosrun', ['rosrun', 'wheel_arm_abs_vel', 'sg100_mapper_bridge'] + remaps)
        return 0
    # roslaunch string command parameters preserve trailing newlines.
    sys.stdout.write(kind)
    return 0


if __name__ == '__main__':
    sys.exit(main())
