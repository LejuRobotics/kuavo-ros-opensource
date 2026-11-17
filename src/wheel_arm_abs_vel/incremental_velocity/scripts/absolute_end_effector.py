"""Reuse test machine/model selection; reject unsupported incremental adapters."""
from pathlib import Path
import runpy
import rospkg
_shared = runpy.run_path(str(Path(rospkg.RosPack().get_path('wheel_arm_abs_vel')) /
                            'scripts/absolute_end_effector.py'))
SUPPORTED = _shared['SUPPORTED'] - {'jodell'}
def checked(value):
    value = _shared['checked'](value)
    if value not in SUPPORTED:
        raise ValueError('jodell has no incremental command adapter')
    return value
def resolve(config, override='auto'):
    return checked(_shared['resolve'](config, override))
def resolve_runtime(config, override='auto'):
    return checked(_shared['resolve_runtime'](config, override))
