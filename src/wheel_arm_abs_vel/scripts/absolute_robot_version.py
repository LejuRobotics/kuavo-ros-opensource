#!/usr/bin/env python3
"""Resolve an absolute robot model from explicit input or saved deployment identity.

Read only literal assignments; never execute shell startup files or infer a robot
model from CPU type, installed model directories, or calibration leftovers.
"""
import os
from pathlib import Path
import re
import sys

SUPPORTED = {'62', '63', '400062'}
ASSIGNMENT = re.compile(r'^\s*(?:export\s+)?ROBOT_VERSION\s*=\s*(?:([0-9]+)|"([0-9]+)"|\'([0-9]+)\')\s*(?:#.*)?$')


def checked(value, source):
    if value not in SUPPORTED:
        raise ValueError('unsupported ROBOT_VERSION from %s: %r; expected 62, 63 or 400062' % (source, value))
    return value, source


def resolve(args, environ, files):
    explicit = [a.split(':=', 1)[1] for a in args if a.startswith('robot_version:=')]
    if explicit:
        if len(set(explicit)) != 1:
            raise ValueError('conflicting robot_version arguments')
        return checked(explicit[0], 'launch argument')
    if 'ROBOT_VERSION' in environ:
        return checked(environ['ROBOT_VERSION'], 'environment')
    found = []
    for path in files:
        path = Path(path)
        if not path.is_file():
            continue
        for line in path.read_text().splitlines():
            match = ASSIGNMENT.fullmatch(line)
            if match:
                found.append((next(v for v in match.groups() if v is not None), str(path)))
            elif re.match(r'^\s*(?:export\s+)?ROBOT_VERSION\s*=', line):
                raise ValueError('nonliteral ROBOT_VERSION in %s; export an explicit value' % path)
    if len({value for value, _ in found}) > 1:
        raise ValueError('conflicting saved ROBOT_VERSION assignments: ' + repr(found))
    if found:
        return checked(*found[0])
    raise ValueError('robot model is not configured; set ROBOT_VERSION=62, 63 or 400062 once in ~/.bashrc, or pass robot_version:= explicitly')


def main():
    try:
        version, source = resolve(sys.argv[1:], os.environ,
                                  [Path.home()/'.bashrc', Path('/etc/environment')])
    except (ValueError, OSError) as exc:
        print('[absolute startup] ERROR: ' + str(exc), file=sys.stderr)
        return 2
    print('[absolute startup] robot_version=%s source=%s' % (version, source), file=sys.stderr, flush=True)
    print(version)
    return 0


if __name__ == '__main__':
    sys.exit(main())
