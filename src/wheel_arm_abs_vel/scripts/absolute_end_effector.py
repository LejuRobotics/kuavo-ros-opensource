#!/usr/bin/env python3
"""Read absolute input end-effector selection from the controller's kuavo.json."""
import argparse
import json
import os
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--ee-type', default='auto')
    parser.add_argument('--bridge', action='store_true')
    args, remaps = parser.parse_known_args(argv)
    if any(':=' not in arg for arg in remaps):
        parser.error('only ROS remappings may follow resolver arguments')
    try:
        config = json.loads(Path(args.config).read_text())
        kind = resolve(config, args.ee_type)
    except (OSError, ValueError, TypeError, AttributeError) as exc:
        print('[absolute end effector] ERROR: %s' % exc, file=sys.stderr, flush=True)
        return 2
    print('[absolute end effector] type=%s source=%s' %
          (kind, args.config if args.ee_type in ('', 'auto') else 'explicit ee_type'),
          file=sys.stderr, flush=True)
    if args.bridge:
        if kind == 'heiman':
            os.execvp('rosrun', ['rosrun', 'wheel_arm_abs_vel', 'sg100_mapper_bridge'] + remaps)
        return 0
    print(kind)
    return 0


if __name__ == '__main__':
    sys.exit(main())
