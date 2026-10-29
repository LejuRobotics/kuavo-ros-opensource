"""Versioned wheel model resolution without a model-number registry or ROS."""
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

CONTROL_JOINTS = frozenset(
    ['knee_joint', 'leg_joint', 'waist_yaw_joint', 'waist_pitch_joint'] +
    [f'zarm_{side}{i}_joint' for side in ('l', 'r') for i in range(1, 8)])


def version_string(version):
    value = str(version)
    if not re.fullmatch(r'[1-9][0-9]{0,8}', value):
        raise ValueError(f'invalid robot version: {version!r}')
    return value


def full_model_relative_path(version):
    version = version_string(version)
    if int(version) not in (62, 63, 400062):
        raise ValueError('Private absolute pipeline supports 62, 63 and 400062')
    model = 'biped_s' + version
    return f"{model}/urdf/{model}.urdf"


def require_full_model(version, assets, config):
    expected = Path(assets) / "models" / full_model_relative_path(version)
    relative = config.get("velocity_ik_urdf")
    if not isinstance(relative, str) or not relative or not assets:
        raise ValueError(f"Absolute velocity IK version {version} requires an explicit full-model contract")
    actual = Path(assets) / "models" / relative
    if actual.resolve(strict=True) != expected.resolve(strict=True):
        raise ValueError(f"Absolute velocity IK version {version} requires its own complete URDF: {expected}")
    return actual


def task_fixed_joints(path):
    # The controller reads this same Boost INFO removeJoints array.
    source = re.sub(r'(?:;|//|#)[^\n]*', '', Path(path).read_text())
    blocks = re.findall(r'\bremoveJoints\s*\{([^{}]*)\}', source)
    if len(blocks) != 1:
        raise ValueError(f'expected one removeJoints block in {path}')
    pattern = r'\[([0-9]+)\]\s*"([^"\n]+)"'
    entries = re.findall(pattern, blocks[0])
    if re.sub(pattern, '', blocks[0]).strip():
        raise ValueError(f'invalid removeJoints array in {path}')
    if [int(i) for i, _ in entries] != list(range(len(entries))):
        raise ValueError(f'non-contiguous removeJoints indices in {path}')
    names = [name for _, name in entries]
    if len(names) != len(set(names)):
        raise ValueError(f'duplicate removeJoints in {path}')
    return names


def bounds(joint):
    limit = joint.find('limit')
    if joint.get('type') not in ('revolute', 'prismatic') or limit is None:
        raise ValueError(f'control joint needs finite scalar limits: {joint.get("name")}')
    result = [float(limit.get('lower')), float(limit.get('upper'))]
    if not all(math.isfinite(v) for v in result) or result[0] >= result[1]:
        raise ValueError(f'invalid joint limits: {joint.get("name")}')
    return result


def resolve_model(version, assets, wheel_interface, overlay=None):
    version = version_string(version)
    if int(version) not in (62, 63, 400062):
        raise ValueError('Private absolute pipeline supports 62, 63 and 400062')
    assets = Path(assets)
    config_path = assets/f'config/kuavo_v{version}/kuavo.json'
    config = json.loads(config_path.read_text())
    if not isinstance(config, dict):
        raise ValueError(f'robot configuration must be an object: {config_path}')
    if overlay is not None:
        if not isinstance(overlay, dict) or any(not key.startswith('velocity_ik_') for key in overlay):
            raise ValueError('Non-absolute key in model contract')
        config.update(overlay)
    if (config.get('NUM_ARM_JOINT') != 14 or
            config.get('NUM_JOINT', 0) - config.get('NUM_HEAD_JOINT', 0) - 14 != 4):
        raise ValueError(f'robot {version}: wheel IK requires 14 arm and 4 body sensor joints; '
                         f'ROBOT_MODULE={config.get("ROBOT_MODULE")}')
    task = Path(wheel_interface)/f'config/kuavo_s{version}/task.info'
    fixed = task_fixed_joints(task)
    configured_fixed = config.get('velocity_ik_fixed_joints', fixed)
    if (not isinstance(configured_fixed, list) or
            any(not isinstance(n, str) for n in configured_fixed) or
            len(configured_fixed) != len(set(configured_fixed)) or
            set(configured_fixed) != set(fixed)):
        raise ValueError(f'robot {version}: kuavo.json fixed joints disagree with controller task.info')
    relative = config.get('velocity_ik_urdf', full_model_relative_path(version))
    model = require_full_model(version, assets, {'velocity_ik_urdf': relative})
    tree = ET.parse(model).getroot()
    joint_list = tree.findall('joint')
    joints = {j.get('name'): j for j in joint_list}
    if len(joints) != len(joint_list) or not set(fixed) <= joints.keys():
        raise ValueError(f'robot {version}: duplicate or missing model joints')
    active = {n for n, j in joints.items() if j.get('type') != 'fixed' and n not in fixed}
    if active != CONTROL_JOINTS:
        raise ValueError(f'robot {version}: incompatible wheel control layout; '
                         f'missing={sorted(CONTROL_JOINTS-active)}, extra={sorted(active-CONTROL_JOINTS)}')
    links = {link.get('name') for link in tree.findall('link')}
    frames = config.get('end_frames_name_ik', [])
    shoulders = config.get('shoulder_frame_names', [])
    if (not isinstance(frames, list) or len(frames) < 5 or
            not isinstance(shoulders, list) or len(shoulders) != 2 or
            any(not isinstance(n, str) or n not in links for n in frames + shoulders)):
        raise ValueError(f'robot {version}: IK/shoulder frames are missing from the full URDF')
    # Retain narrower limits from this version's legacy arm model; never borrow
    # another model's limits or silently widen its historical control domain.
    limits = {}
    legacy_path = config.get('arm_urdf')
    if legacy_path:
        legacy = {j.get('name'): j for j in ET.parse(assets/'models'/legacy_path).getroot().findall('joint')}
        for name in sorted(CONTROL_JOINTS):
            current = bounds(joints[name])
            if name in legacy:
                old = bounds(legacy[name])
                intersection = [max(current[0], old[0]), min(current[1], old[1])]
                if intersection[0] >= intersection[1]:
                    raise ValueError(f'robot {version}: empty legacy/full joint range for {name}')
                if intersection != current:
                    limits[name] = intersection
    explicit_limits = config.get('velocity_ik_position_limits', {})
    if not isinstance(explicit_limits, dict):
        raise ValueError('velocity_ik_position_limits must be an object')
    limits.update(explicit_limits)
    for name, interval in limits.items():
        if name not in CONTROL_JOINTS or not isinstance(interval, list) or len(interval) != 2:
            raise ValueError(f'invalid velocity IK limit: {name}')
        interval = [float(x) for x in interval]
        current = bounds(joints[name])
        if not all(math.isfinite(x) for x in interval) or max(current[0], interval[0]) >= min(current[1], interval[1]):
            raise ValueError(f'robot {version}: invalid/empty limits for {name}')
        limits[name] = interval
    for name in CONTROL_JOINTS:
        bounds(joints[name])
    # Existing launches used an uncalibrated configured forearm, not a measured
    # calibration. Prefer explicit teleop geometry; otherwise label the JSON
    # baseline clearly instead of applying a hardcoded V62/0.45 m to every model.
    forearm = float(config.get('teleop_tcp_forearm_length', config.get('lower_arm_length')))
    if not math.isfinite(forearm) or forearm <= 0:
        raise ValueError(f'robot {version}: invalid configured forearm length')
    geometry_id = config.get('teleop_tcp_geometry_id') or f'V{version}_KUAVO_JSON_FOREARM_UNCALIBRATED'
    return dict(robot_version=int(version), config_path=str(config_path.resolve()),
                task_file=str(task.resolve()), velocity_ik_urdf=relative,
                velocity_ik_fixed_joints=fixed, velocity_ik_position_limits=limits,
                teleop_tcp_forearm_length=forearm, teleop_tcp_geometry_id=geometry_id)
