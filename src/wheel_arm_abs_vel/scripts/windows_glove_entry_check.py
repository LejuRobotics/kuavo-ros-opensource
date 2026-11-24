#!/usr/bin/env python3
"""Validate the dedicated Windows glove entry before ROS launches any nodes."""
import argparse
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from controller_mount_calibration import checked_calibration
import yaml


def validate_entry(robot_version, calibration_file, teleop_mode='absolute',
                   hand_control_source='qianshi', backend='windows_udp', freshness_timeout=.5):
    if (teleop_mode, hand_control_source, backend) != ('absolute', 'qianshi', 'windows_udp'):
        raise ValueError('quest_windows_udp.launch requires teleop_mode=absolute, '
                         'hand_control_source=qianshi, qianshi_backend=windows_udp')
    if not math.isfinite(freshness_timeout) or not 0 < freshness_timeout <= .5:
        raise ValueError('glove_ik_freshness_timeout must be finite and within (0, 0.5] seconds')
    document = yaml.safe_load(Path(calibration_file).expanduser().read_text())
    if not isinstance(document, dict) or not isinstance(document.get('quest3'), dict):
        raise ValueError('mount calibration YAML must contain quest3/controller_calibration')
    config = document['quest3'].get('controller_calibration')
    if not isinstance(config, dict):
        raise ValueError('mount calibration YAML must contain quest3/controller_calibration')
    try:
        checked_calibration(config, robot_version)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError('Mount calibration is incompatible with robot_version=%s: %s. '
                         'Set quest_controller_calibration_file to a matching file; '
                         'the shipped enabled calibration belongs to 400062.' % (robot_version, exc)) from exc
    return 'ready: Windows glove IK timeout %.3fs, robot %s' % (freshness_timeout, robot_version)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot-version', required=True, type=int)
    p.add_argument('--calibration-file', required=True)
    p.add_argument('--teleop-mode', default='absolute')
    p.add_argument('--hand-control-source', default='qianshi')
    p.add_argument('--backend', default='windows_udp')
    p.add_argument('--freshness-timeout', type=float, default=.5)
    args = p.parse_args()
    try:
        print(validate_entry(args.robot_version, args.calibration_file,
                             args.teleop_mode, args.hand_control_source,
                             args.backend, args.freshness_timeout))
    except (ValueError, OSError, yaml.YAMLError) as exc:
        p.exit(2, 'Windows glove startup check failed: %s\n' % exc)


if __name__ == '__main__':
    main()
