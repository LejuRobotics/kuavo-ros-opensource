

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Dict, Tuple, Any, Optional

import kuavo_gmr.utils.lafan_vendor.utils as utils


# ---------------------------- name mapping ----------------------------
# Motive (your “upper” cali pose) -> LAFAN (your “lower” cali pose)
XS_NAME_MAPPING: Dict[str, str]= {
    # torso
    "Hips": "Hips", 
    "Chest": "Spine",  
    "Neck": "Neck",
    "Head": "Head",

    # arms (Motive chain: Collar -> Shoulder -> Elbow -> Wrist)
    "LeftCollar": "LeftShoulder",
    "LeftShoulder": "LeftArm",
    "LeftElbow": "LeftForeArm",
    "LeftWrist": "LeftHand",

    "RightCollar": "RightShoulder",
    "RightShoulder": "RightArm",
    "RightElbow": "RightForeArm",
    "RightWrist": "RightHand",

    # legs
    "LeftHip": "LeftUpLeg",
    "LeftKnee": "LeftLeg",
    "LeftAnkle": "LeftFoot",
    "LeftToe": "LeftToeBase",

    "RightHip": "RightUpLeg",
    "RightKnee": "RightLeg",
    "RightAnkle": "RightFoot",
    "RightToe": "RightToeBase",
}


# ---------------------------- helpers ----------------------------
def _decode_name(x: Any) -> str:
    if isinstance(x, (bytes, bytearray)):
        return x.decode("utf-8", errors="ignore")
    return str(x)


def _strip_prefix(name: str) -> str:
    # "abc_Hips" -> "Hips"
    return name.split("_", 1)[1] if "_" in name else name


def _wxyz_from_euler_deg(rx: float, ry: float, rz: float, order: str = "xyz") -> np.ndarray:
    """Return quaternion in wxyz from euler degrees."""
    q_xyzw = R.from_euler(order, [rx, ry, rz], degrees=True).as_quat()  # xyzw
    return np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]], dtype=float)  # wxyz


def remap_pose_names_xsense_to_lafan(
    pose: Dict[str, Tuple[np.ndarray, np.ndarray]],
    mapping: Dict[str, str] = XS_NAME_MAPPING,
) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    """
    Remap pose dict keys from Motive names to LAFAN names.
    Only mapped keys are kept.
    """
    out: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    src_of_dst: Dict[str, str] = {}

    for src_name, val in pose.items():
        dst_name = mapping.get(src_name)
        if dst_name is None:
            continue

        # Resolve collisions by priority
        if dst_name not in out:
            out[dst_name] = val
            src_of_dst[dst_name] = src_name
        
    return out


# ---------------------------- main API ----------------------------
def cali_pose_from_seg_desc(
    rb_desc: dict,
    rotation_matrix: np.ndarray = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]),
    unit_scale: float = 1.0,
    arm_pose_deg: float = 75.0,
    output_lafan_names: bool = True,
    verbose: bool = True,
) -> Tuple[Dict[str, Tuple[np.ndarray, np.ndarray]], Optional[float]]:
    """
    rb_desc example:
      { rid: {'name': b'abc_Hips', 'parent_id': 0, 'offset': [x,y,z]}, ... }

    Return:
      pose: dict[name] = (gpos, gquat_wxyz)
      human_height: float (rough) or None
    """

    rotation_matrix = np.asarray(rotation_matrix, dtype=float).reshape(3, 3)
    rot_quat_wxyz = R.from_matrix(rotation_matrix).as_quat(scalar_first=True)  # wxyz

    # ---- ids / indexing ----
    ids = sorted(rb_desc.keys())
    id2idx = {rid: i for i, rid in enumerate(ids)}
    N = len(ids)

    names: list[str] = []
    parents = np.full((N,), -1, dtype=int)
    lpos = np.zeros((N, 3), dtype=float)

    # ---- parse rb_desc into (names, parents, offsets) ----
    for rid in ids:
        i = id2idx[rid]
        info = rb_desc[rid]

        name = _strip_prefix(_decode_name(info["name"]))
        names.append(name)

        pid = int(info.get("parent_id", 0))
        parents[i] = id2idx[pid] if (pid != 0 and pid in id2idx) else -1

        lpos[i] = np.asarray(info.get("offset", [0, 0, 0]), dtype=float).reshape(3,)

    if verbose:
        print("matched names:", names)

    # ---- force roots offsets to 0 ----
    root_idxs = np.where(parents == -1)[0]
    for ri in root_idxs:
        lpos[ri] = 0.0

    # ---- local rotations: identity everywhere (wxyz) ----
    lrot = np.zeros((1, N, 4), dtype=float)
    lrot[..., 0] = 1.0

    # ---- optional: neutral A-pose for arms ----
    # Motive typically uses LeftShoulder/RightShoulder as upper-arm joints
    name_to_idx = {n: i for i, n in enumerate(names)}
    left_arm_joint = "LeftArm" if "LeftArm" in name_to_idx else ("LeftShoulder" if "LeftShoulder" in name_to_idx else None)
    right_arm_joint = "RightArm" if "RightArm" in name_to_idx else ("RightShoulder" if "RightShoulder" in name_to_idx else None)

    if left_arm_joint is not None:
        lrot[0, name_to_idx[left_arm_joint], :] = _wxyz_from_euler_deg(0.0, 0.0, -arm_pose_deg, "xyz")
    if right_arm_joint is not None:
        lrot[0, name_to_idx[right_arm_joint], :] = _wxyz_from_euler_deg(0.0, 0.0, +arm_pose_deg, "xyz")

    # ---- FK in lafan utils convention ----
    gquat, gpos = utils.quat_fk(lrot, lpos[None, :, :], parents)  # (1,N,4), (1,N,3)
    gquat0 = gquat[0]
    gpos0 = gpos[0]

    # ---- coordinate + unit ----
    gpos0 = (gpos0 @ rotation_matrix.T) * float(unit_scale)
    gquat0 = utils.quat_mul(rot_quat_wxyz, gquat0)

    pose_raw: Dict[str, Tuple[np.ndarray, np.ndarray]] = {
        names[i]: (gpos0[i].copy(), gquat0[i].copy()) for i in range(N)
    }

    # ---- remap Motive names -> LAFAN names (to match offline BVH loader) ----
    pose = remap_pose_names_xsense_to_lafan(pose_raw) if output_lafan_names else pose_raw

    if verbose:
        print("cali pose loaded.", pose)

    # ---- FootMod convention ----
    if "LeftFoot" in pose and "LeftToeBase" in pose:
        pose["LeftFootMod"] = (pose["LeftFoot"][0], pose["LeftToeBase"][1])
    if "RightFoot" in pose and "RightToeBase" in pose:
        pose["RightFootMod"] = (pose["RightFoot"][0], pose["RightToeBase"][1])

    # ---- rough height from cali pose ----
    human_height: Optional[float] = None
    if "Head" in pose and "LeftFootMod" in pose and "RightFootMod" in pose:
        head_z = pose["Head"][0][2]
        foot_z = min(pose["LeftFootMod"][0][2], pose["RightFootMod"][0][2])
        human_height = float(head_z - foot_z)

    return pose, human_height


__all__ = [
    "XS_NAME_MAPPING",
    "remap_pose_names_xsense_to_lafan",
    "cali_pose_from_seg_desc",
]
