"""Incremental-specific helpers; reuse current test common utilities."""
import os, rospkg
__path__.append(os.path.join(rospkg.RosPack().get_path("wheel_arm_abs_vel"), "scripts", "tools"))
__path__.append(os.path.join(rospkg.RosPack().get_path("motion_capture_ik"), "scripts", "tools"))
