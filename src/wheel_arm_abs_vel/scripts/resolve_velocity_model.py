#!/usr/bin/env python3
"""Read-only startup model resolution: print one JSON contract without ROS."""
import argparse
import json
import sys
from pathlib import Path

# catkin's devel relay execs this file with sys.path[0] pointing at libexec,
# while __file__ is the source script. Resolve siblings from this file instead.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from tools.full_model_contract import resolve_model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-version', required=True)
    parser.add_argument('--assets', required=True)
    parser.add_argument('--wheel-interface', required=True)
    parser.add_argument('--overlay-dir', required=True)
    args = parser.parse_args()
    try:
        from tools.full_model_contract import version_string
        version = version_string(args.robot_version)
        overlay = json.loads((Path(args.overlay_dir) / ('model_s' + version + '.json')).read_text())
        result = resolve_model(version, args.assets, args.wheel_interface, overlay)
    except (ValueError, TypeError, KeyError, OSError) as exc:
        print('[velocity model] ERROR: ' + str(exc), file=sys.stderr)
        return 2
    sys.stdout.write(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
