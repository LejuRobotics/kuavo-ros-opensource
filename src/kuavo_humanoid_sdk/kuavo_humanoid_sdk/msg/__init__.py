"""Expose catkin-generated ROS packages under the SDK message namespace.

Catkin installs the generated Python packages as top-level ``kuavo_msgs`` and
``ocs2_msgs`` modules.  The SDK imports them through
``kuavo_humanoid_sdk.msg.<package>``.  Registering aliases here keeps those
imports working in a clean source checkout without relying on ignored copies
created by ``install.sh``.
"""

import importlib
import sys


for _package_name in ("kuavo_msgs", "ocs2_msgs"):
    _module = importlib.import_module(_package_name)
    globals()[_package_name] = _module
    sys.modules["{}.{}".format(__name__, _package_name)] = _module

del _module
del _package_name


