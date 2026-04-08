import os
import tempfile
import xml.etree.ElementTree as ET

try:
    import rospy
except Exception:
    rospy = None


def prepare_model_file(model_file, joints_to_force_fixed):
    """
    If rosparam 'wheel_ik' is true (or env WHEEL_IK), create a temporary URDF where
    joints in joints_to_force_fixed are set to type="fixed" and return the
    (model_to_load, temp_path). If no temp file created, temp_path is None and
    model_to_load == model_file.

    This function also converts relative <mesh filename> entries to absolute
    paths based on model_file directory to avoid parser URI resolution issues.
    """
    model_to_load = model_file
    temp_path = None

    # determine wheel_ik flag from rosparam or environment
    wheel_ik = False
    if rospy is not None:
        try:
            wheel_ik = bool(rospy.get_param('wheel_ik', False))
        except Exception:
            wheel_ik = False
    else:
        wheel_ik = os.environ.get('WHEEL_IK', 'false').lower() in ('1', 'true', 'yes')

    if not wheel_ik:
        print(f"[urdf_utils] wheel_ik param is false; loading original URDF: {model_file}")
        return model_to_load, temp_path

    print("[urdf_utils] 当前IK为轮臂模式，准备创建临时URDF")
    try:
        tree = ET.parse(model_file)
        root = tree.getroot()
        modified = False
        for joint in root.findall('joint'):
            name = joint.get('name')
            if name in joints_to_force_fixed:
                joint.set('type', 'fixed')
                for child_tag in ['axis', 'limit', 'dynamics']:
                    for c in joint.findall(child_tag):
                        joint.remove(c)
                modified = True
                print(f"[urdf_utils] Converted joint '{name}' to fixed in temporary URDF")
        if modified:
            model_dir = os.path.dirname(model_file)
            for mesh in root.findall('.//mesh'):
                fn = mesh.get('filename')
                if fn is None:
                    continue
                if fn.startswith('package:') or fn.startswith('package://') or os.path.isabs(fn):
                    continue
                abs_fn = os.path.normpath(os.path.join(model_dir, fn))
                mesh.set('filename', abs_fn)
            tf = tempfile.NamedTemporaryFile(delete=False, suffix='.urdf')
            temp_path = tf.name
            tree.write(temp_path, encoding='utf-8', xml_declaration=True)
            tf.close()
            model_to_load = temp_path
            print(f"[urdf_utils] Using temporary URDF: {temp_path}")
    except Exception as e:
        print(f"[urdf_utils] Could not create temp fixed-URDF: {e}, loading original model_file")

    return model_to_load, temp_path
