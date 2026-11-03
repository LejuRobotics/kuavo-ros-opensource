"""Resolve unchanged common helpers from their owning ROS packages."""
import os
import sys
import rospkg

def configure():
    own_tools = os.path.join(os.path.dirname(__file__), "tools")
    if own_tools not in sys.path:
        sys.path.append(own_tools)
    packages = rospkg.RosPack()
    for name in ("motion_capture_ik", "noitom_hi5_hand_udp_python"):
        root = packages.get_path(name)
        for path in (os.path.join(root, "scripts"), os.path.join(root, "scripts", "tools"), root):
            if path not in sys.path:
                sys.path.append(path)
configure()
