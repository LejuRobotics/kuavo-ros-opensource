#!/usr/bin/env python3
"""Use the humanoid launch contract: explicit robot_version, then ROBOT_VERSION.

The shell owns persistent environment setup. Never parse startup files here.
Asset paths and configuration loading remain in robot_version_manager.launch.
"""
import os
import sys

SUPPORTED = {'62', '63', '400062'}


def checked(value, source):
    if value not in SUPPORTED:
        raise ValueError('unsupported ROBOT_VERSION from %s: %r; expected 62, 63 or 400062' % (source, value))
    return value, source


def resolve(args, environ):
    explicit = [a.split(':=', 1)[1] for a in args if a.startswith('robot_version:=')]
    if explicit:
        if len(set(explicit)) != 1:
            raise ValueError('conflicting robot_version arguments')
        return checked(explicit[0], 'launch argument')
    if 'ROBOT_VERSION' in environ:
        return checked(environ['ROBOT_VERSION'], 'environment')
    raise ValueError('ROBOT_VERSION is not in the launch environment; load the configured shell environment and preserve ROBOT_VERSION through sudo, or pass robot_version:=')


def main():
    try:
        version, source = resolve(sys.argv[1:], os.environ)
    except ValueError as exc:
        print('[absolute startup] ERROR: ' + str(exc), file=sys.stderr)
        return 2
    print('[absolute startup] robot_version=%s source=%s' % (version, source), file=sys.stderr, flush=True)
    print(version)
    return 0


if __name__ == '__main__':
    sys.exit(main())
