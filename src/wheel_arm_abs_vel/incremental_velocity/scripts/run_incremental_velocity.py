#!/usr/bin/env python3
"""Start the incremental Mapper from the package's source/install location."""
from pathlib import Path
import sys, runpy, rospkg
scripts = Path(rospkg.RosPack().get_path('wheel_arm_abs_vel')) / 'incremental_velocity/scripts'
sys.path.insert(0, str(scripts))
runpy.run_path(str(scripts / 'ik_ros_velocity.py'), run_name='__main__')
