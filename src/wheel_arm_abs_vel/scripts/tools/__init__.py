"""Private absolute helpers with explicit reuse of unchanged common utilities."""
import os
import rospkg
__path__.append(os.path.join(rospkg.RosPack().get_path("motion_capture_ik"), "scripts", "tools"))
