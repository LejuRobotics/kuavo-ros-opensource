#!/usr/bin/env python3
"""Read absolute input end-effector selection from the controller's kuavo.json."""
import argparse
import json
import os
from pathlib import Path
import sys

SUPPORTED = frozenset(('qiangnao', 'qiangnao_touch', 'heiman', 'lejuclaw',
                       'jodell', 'revo2', 'linker_hand', 'qibeng', 'none'))


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


def machine_mode(value):
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
    return False


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
    return resolve(config, override)


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
        contracted = bridge_contract(args.ee_type) if args.bridge else None
        if contracted is not None:
            kind = contracted
            source = 'controller contract'
        else:
            config = json.loads(Path(args.config).read_text()) if args.ee_type in ('', 'auto') else {}
            kind = resolve(config, args.ee_type)
            source = 'model kuavo.json' if args.ee_type in ('', 'auto') else 'explicit ee_type'
        if args.contract_version is not None:
            real = machine_mode(args.machine_mode)
    except (OSError, ValueError, TypeError, AttributeError) as exc:
        print('[absolute end effector] ERROR: %s' % exc, file=sys.stderr, flush=True)
        return 2
    print('[absolute end effector] type=%s source=%s' %
          (kind, args.config if source == 'model kuavo.json' else source),
          file=sys.stderr, flush=True)
    if args.contract_version is not None:
        if args.contract_version <= 0: raise ValueError('positive robot version required')
        sys.stdout.write(json.dumps(dict(robot_version=args.contract_version, types=[kind, kind], source=source, real=real)))
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
