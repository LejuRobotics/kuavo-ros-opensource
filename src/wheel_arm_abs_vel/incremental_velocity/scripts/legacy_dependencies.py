"""Resolve unchanged common helpers from their owning ROS packages."""
import os
import sys
import rospkg

def configure():
    own_tools = os.path.join(os.path.dirname(__file__), "tools")
    # The test runtime wrapper also adds common tools to PYTHONPATH. Private
    # bare imports must resolve before them, just like tools.* imports do.
    if own_tools in sys.path:
        sys.path.remove(own_tools)
    sys.path.insert(1, own_tools)
    packages = rospkg.RosPack()
    for name in ("wheel_arm_abs_vel", "motion_capture_ik", "noitom_hi5_hand_udp_python"):
        root = packages.get_path(name)
        for path in (os.path.join(root, "scripts"), os.path.join(root, "scripts", "tools"), root):
            if path not in sys.path:
                sys.path.append(path)
configure()
