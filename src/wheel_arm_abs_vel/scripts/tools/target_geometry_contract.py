"""Explicit source/model geometry contract; no ROS or reference-state ownership.

All lengths are metres, rotations are SO(3), twists use the same expressed frame.
A declared evidence string is provenance, never proof of hardware calibration.
"""
import copy
import hashlib
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

LEGACY = "LEGACY_PALM_6D"
WRIST = "WRIST_CONSISTENT_6D"

def checked_se3(value):
    X = np.asarray(value, dtype=float)
    if X.shape != (4, 4) or not np.isfinite(X).all():
        raise ValueError("SE3 must be a finite 4x4 matrix")
    R = X[:3, :3]
    if (not np.allclose(X[3], [0, 0, 0, 1], atol=1e-12, rtol=0) or
        not np.allclose(R.T @ R, np.eye(3), atol=1e-9, rtol=0) or
        abs(np.linalg.det(R)-1.) > 1e-9):
        raise ValueError("Invalid SE3 / SO3 (reflection and scaling forbidden)")
    return X.copy()

def inverse_se3(value):
    X = checked_se3(value)
    Y = np.eye(4)
    Y[:3, :3] = X[:3, :3].T
    Y[:3, 3] = -Y[:3, :3] @ X[:3, 3]
    return Y

def point_twist(v_origin, omega, expressed_offset):
    return np.asarray(v_origin) + np.cross(omega, expressed_offset)

def _positive(x, name):
    v = float(x)
    if not math.isfinite(v) or v <= 0:
        raise ValueError(name + " must be finite and positive")
    return v

def validate_wrist_contract(config, model_sha256, real_source=True):
    c = copy.deepcopy(config)
    if c.get("units") != {"length": "m", "angle": "rad"}:
        raise ValueError("Explicit m/rad units required")
    if c.get("fixed_axes") != "quest_chest_robot_axes":
        raise ValueError("Unsupported fixed axes")
    if (len(model_sha256) != 64 or c.get("model_sha256") != model_sha256 or
        any(ch not in "0123456789abcdef" for ch in model_sha256)):
        raise ValueError("IK model identity mismatch")
    if not c.get("geometry_id") or not c.get("evidence"):
        raise ValueError("BLOCKED_CALIBRATION: geometry identity/evidence absent")
    allowed = {"SOURCE_WRIST_CALIBRATION_DECLARED"}
    if not real_source:
        allowed.add("SIM_ONLY_SYNTHETIC")
    if c.get("calibration_status") not in allowed:
        raise ValueError("BLOCKED_CALIBRATION: no approved source-to-wrist contract")
    axes = np.asarray(c.get("robot_task_from_chest_axes"), dtype=float)
    if axes.shape != (3, 3):
        raise ValueError("Explicit fixed robot_task_from_chest_axes required")
    Xaxes = np.eye(4); Xaxes[:3, :3] = axes
    checked_se3(Xaxes)
    c["robot_task_from_chest_axes"] = axes.tolist()
    if not c.get("sources"):
        raise ValueError("BLOCKED_CALIBRATION: no source definition")
    for source, sides in c["sources"].items():
        if source not in {"controller", "hand_tracking"} or set(sides) != {"Left", "Right"}:
            raise ValueError("Source/side mismatch")
        for a in sides.values():
            allowed_origins = ({"CONTROLLER_GRIP", "CONTROLLER_AIM"} if source == "controller"
                               else {"HAND_PALM", "ANATOMICAL_WRIST"})
            if a.get("source_origin") not in allowed_origins:
                raise ValueError("Source origin is unknown or inconsistent")
            a["source_to_wrist"] = checked_se3(a["source_to_wrist"]).tolist()
            a["joint7_to_tcp"] = checked_se3(a["joint7_to_tcp"]).tolist()
            R = np.asarray(a["wrist_to_joint7_rotation"], dtype=float)
            X = np.eye(4)
            if R.shape != (3, 3): raise ValueError("wrist_to_joint7_rotation shape")
            X[:3, :3] = R
            checked_se3(X)
            a["forearm_to_joint7_m"] = _positive(a["forearm_to_joint7_m"], "forearm")
    return c

def map_wrist_pose(chest, shoulder, elbow, source, robot_task_from_chest_axes,
                   shoulder_anchor, upper_length, arm):
    """Retarget explicit wrist direction, then use the single robot tool SE3.

    Inputs use one fixed Cartesian frame. The fixed axis map is explicit and
    never taken from a neutral/initial chest attitude. source includes the already verified
    source-axis conversion. This function does not infer an anatomical wrist.
    Returns TCP, elbow hint and joint7 witness target (not a joint configuration).
    """
    C, S = checked_se3(chest), checked_se3(source)
    Xw = S @ checked_se3(arm["source_to_wrist"])
    Rremove = np.asarray(robot_task_from_chest_axes) @ C[:3, :3].T
    upper = Rremove @ (np.asarray(elbow)-np.asarray(shoulder))
    lower = Rremove @ (Xw[:3, 3]-np.asarray(elbow))
    un, ln = np.linalg.norm(upper), np.linalg.norm(lower)
    if not np.isfinite([un, ln]).all() or min(un, ln) < 1e-6:
        raise ValueError("Unresolved source segment direction")
    hint = np.asarray(shoulder_anchor) + _positive(upper_length, "upper") * upper / un
    X7 = np.eye(4)
    X7[:3, 3] = hint + _positive(arm["forearm_to_joint7_m"], "forearm") * lower / ln
    X7[:3, :3] = Rremove @ Xw[:3, :3] @ np.asarray(arm["wrist_to_joint7_rotation"])
    X7 = checked_se3(X7)
    return X7 @ checked_se3(arm["joint7_to_tcp"]), hint, X7

def _origin(joint):
    o = joint.find("origin")
    xyz = np.fromstring(o.get("xyz", "0 0 0") if o is not None else "0 0 0", sep=" ")
    rpy = np.fromstring(o.get("rpy", "0 0 0") if o is not None else "0 0 0", sep=" ")
    if xyz.size != 3 or rpy.size != 3: raise ValueError("Invalid URDF origin")
    r, p, y = rpy
    Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    Rz = np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
    X = np.eye(4); X[:3,:3] = Rz@Ry@Rx; X[:3,3] = xyz
    return checked_se3(X)

def model_geometry(urdf_path, end_frames, tcp_transforms):
    """Bind actual IK URDF; derive a conservative rigid-chain necessary bound.

    Prismatic chains are rejected rather than assigned a constant reach radius.
    The bound is not a sufficient reachability test. Fixed tool rotation is kept.
    """
    if len(end_frames) != 2 or set(tcp_transforms) != {"Left", "Right"}:
        raise ValueError("Two independently identified arm/tool definitions required")
    p = Path(urdf_path)
    raw = p.read_bytes()
    root = ET.fromstring(raw)
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
    result = {"model_path": str(p), "model_sha256": hashlib.sha256(raw).hexdigest(),
              "arms": {}, "qualification": "MODEL_GEOMETRY_ONLY_NOT_HARDWARE_CALIBRATION"}
    for side, prefix, end in zip(("Left", "Right"), ("l", "r"), end_frames):
        shoulder, wrist = "zarm_"+prefix+"1_link", "zarm_"+prefix+"7_link"
        cur, radius, chain, visited = wrist, 0., [], set()
        while cur != shoulder:
            if cur in visited: raise ValueError("Cyclic URDF chain")
            visited.add(cur)
            j = by_child[cur]
            if j.get("type") not in {"fixed", "revolute", "continuous"}:
                raise ValueError("Reach bound requires a rigid revolute chain")
            radius += float(np.linalg.norm(_origin(j)[:3, 3]))
            chain.append(j.get("name")); cur = j.find("parent").get("link")
        X, cur, visited = np.eye(4), end, set()
        while cur != wrist:
            if cur in visited: raise ValueError("Cyclic URDF tool chain")
            visited.add(cur)
            j = by_child[cur]
            if j.get("type") != "fixed":
                raise ValueError("joint7 to TCP must be a fixed transform")
            X = _origin(j) @ X; cur = j.find("parent").get("link")
        X = X @ checked_se3(tcp_transforms[side])
        result["arms"][side] = {"shoulder_frame": shoulder, "joint7_frame": wrist,
            "configured_tcp_parent": end, "joint7_to_tcp": X.tolist(),
            "shoulder_to_joint7_necessary_upper_m": radius, "bound_chain": chain}
    return result


def neutral_retarget_geometry(urdf_path, base_frame, shoulders, elbows):
    """Zero-joint robot geometry for the symmetric shoulder retarget convention.

    This binds robot geometry only. Human forearm/Palm calibration stays explicit.
    """
    root = ET.parse(urdf_path).getroot()
    links = {link.get("name") for link in root.findall("link")}
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
    def pose(frame):
        if frame not in links:
            raise ValueError("Missing retarget frame: " + frame)
        X, visited = np.eye(4), set()
        while frame != base_frame:
            if frame in visited or frame not in by_child:
                raise ValueError("Retarget frame is not descended from the base")
            visited.add(frame)
            joint = by_child[frame]
            if joint.get("type") not in {"fixed", "revolute", "continuous", "prismatic"}:
                raise ValueError("Unsupported neutral retarget joint")
            X = _origin(joint) @ X
            frame = joint.find("parent").get("link")
        return X[:3, 3]
    if len(shoulders) != 2 or len(elbows) != 2:
        raise ValueError("Two shoulder/elbow frames required")
    shoulder = [pose(name) for name in shoulders]
    lengths = [float(np.linalg.norm(pose(e)-s)) for e, s in zip(elbows, shoulder)]
    if (not np.allclose(shoulder[0], shoulder[1] * [1., -1., 1.], atol=1e-9, rtol=0)
            or not np.isclose(lengths[0], lengths[1], atol=1e-9, rtol=0)
            or shoulder[0][1] <= 0 or min(lengths) <= 0):
        raise ValueError("URDF does not satisfy the symmetric shoulder retarget convention")
    return {"base_chest_offset_x": float(shoulder[0][0]),
            "base_height_offset": float(shoulder[0][2]),
            "shoulder_width": float(shoulder[0][1]),
            "upper_arm_length": lengths[0],
            "left_shoulder": shoulder[0].tolist(),
            "right_shoulder": shoulder[1].tolist(),
            "joint_configuration": "URDF_ZERO"}
