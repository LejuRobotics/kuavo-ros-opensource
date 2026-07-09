import numpy as np
from scipy.spatial.transform import Rotation as R

from utils.lafan_vendor import utils
from utils.lafan_vendor.extract import read_bvh

# ---------------------------------------------------------------------------
# BVH conventions (used for both motion loading and automatic offset calibration)
# ---------------------------------------------------------------------------

LF_NAME_MAPPING = {
    "Hips": "Hips",
    "Spine": "Spine",
    "Neck": "Neck",
    "Head": "Head",
    "LeftShoulder": "LeftShoulder",
    "LeftArm": "LeftArm",
    "LeftForeArm": "LeftForeArm",
    "LeftHand": "LeftHand",
    "RightShoulder": "RightShoulder",
    "RightArm": "RightArm",
    "RightForeArm": "RightForeArm",
    "RightHand": "RightHand",
    "LeftUpLeg": "LeftUpLeg",
    "LeftLeg": "LeftLeg",
    "LeftFoot": "LeftFoot",
    "LeftToe": "LeftToeBase",
    "RightUpLeg": "RightUpLeg",
    "RightLeg": "RightLeg",
    "RightFoot": "RightFoot",
    "RightToe": "RightToeBase",
}

BJ_NAME_MAPPING = {
    "Skeleton": "Hips",
    "Chest": "Spine",
    "Neck": "Neck",
    "Head": "Head",
    "LShoulder": "LeftShoulder",
    "LUArm": "LeftArm",
    "LFArm": "LeftForeArm",
    "LHand": "LeftHand",
    "RShoulder": "RightShoulder",
    "RUArm": "RightArm",
    "RFArm": "RightForeArm",
    "RHand": "RightHand",
    "LThigh": "LeftUpLeg",
    "LShin": "LeftLeg",
    "LFoot": "LeftFoot",
    "LToe": "LeftToeBase",
    "RThigh": "RightUpLeg",
    "RShin": "RightLeg",
    "RFoot": "RightFoot",
    "RToe": "RightToeBase",
}

SZ_NAME_MAPPING = {
    "Hips": "Hips",
    "Spine": "Spine",
    "Neck": "Neck",
    "Head": "Head",
    "LeftShoulder": "LeftShoulder",
    "LeftArm": "LeftArm",
    "LeftForeArm": "LeftForeArm",
    "LeftHand": "LeftHand",
    "RightShoulder": "RightShoulder",
    "RightArm": "RightArm",
    "RightForeArm": "RightForeArm",
    "RightHand": "RightHand",
    "LeftUpLeg": "LeftUpLeg",
    "LeftLeg": "LeftLeg",
    "LeftFoot": "LeftFoot",
    "LeftToeBase": "LeftToeBase",
    "RightUpLeg": "RightUpLeg",
    "RightLeg": "RightLeg",
    "RightFoot": "RightFoot",
    "RightToeBase": "RightToeBase",
}

NK_NAME_MAPPING = {
    "Hips": "Hips",
    "Spine": "Spine",
    "Neck": "Neck",
    "Head": "Head",
    "LeftShoulder": "LeftShoulder",
    "LeftArm": "LeftArm",
    "LeftForeArm": "LeftForeArm",
    "LeftHand": "LeftHand",
    "RightShoulder": "RightShoulder",
    "RightArm": "RightArm",
    "RightForeArm": "RightForeArm",
    "RightHand": "RightHand",
    "LeftUpLeg": "LeftUpLeg",
    "LeftLeg": "LeftLeg",
    "LeftFoot": "LeftFoot",
    "LeftToeBase": "LeftToeBase",
    "RightUpLeg": "RightUpLeg",
    "RightLeg": "RightLeg",
    "RightFoot": "RightFoot",
    "RightToeBase": "RightToeBase",
}

def _find_bvh_name_for_canonical(name_mapping, canonical_name: str):
    # name_mapping: {bvh_name: canonical_name}
    for bvh_name, canon in name_mapping.items():
        if canon == canonical_name:
            return bvh_name
    return None

def _xyzw_to_wxyz(q_xyzw: np.ndarray) -> np.ndarray:
    q_xyzw = np.asarray(q_xyzw, dtype=float).reshape(4,)
    return np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]], dtype=float)

def _wxyz_from_euler_deg(rx: float, ry: float, rz: float, order) -> np.ndarray:
    # Single-axis rotations are unambiguous; for multi-axis, we follow SciPy's intrinsic 'xyz'.
    q_xyzw = R.from_euler(order, [rx, ry, rz], degrees=True).as_quat()
    return _xyzw_to_wxyz(q_xyzw)

LAFAN_APOSE_ZYX = {
    "Hips": (90, 0, 90),
    # spine/neck/head neutral
    "Spine": (0, 0, 0), "Spine1": (0, 0, 0), "Spine2": (0, 0, 0),
    "Neck": (0, 0, 0), "Head": (0, 0, 0),

    # arms 
    "LeftShoulder":  (-80, -100, -100),
    "LeftArm":       (0, 80, 0),
    "LeftForeArm":   (0, 0, 0),
    "LeftHand":      (0, 0, 0),

    "RightShoulder": (-80, 100, 100),
    "RightArm":      (0, -80, 0),
    "RightForeArm":  (0, 0, 0),
    "RightHand":     (0, 0, 0),

    # legs 
    "LeftUpLeg":   (+180, 2, 180),
    "LeftLeg":     (0, 0, -4),
    "LeftFoot":    (60, 0, 0),
    "LeftToe":     (15, 0.003, 0),

    "RightUpLeg":  (+180, -2, -180),
    "RightLeg":    (0, 0, 4),
    "RightFoot":   (60, 0, 0),
    "RightToe":    (15, 0.003, 0),
}

def load_bvh_file(bvh_file, format="lafan1", arm_pose_deg=75.0):
    """
    Must return a dictionary with the following structure:
    {
        "Hips": (position, orientation),
        "Spine": (position, orientation),
        ...
    }
    """
    if format == "lafan1":
        name_mapping = LF_NAME_MAPPING
        rotation_matrix = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        unit_scale = 1.0 / 100.0
    elif format == "nokov":
        name_mapping = NK_NAME_MAPPING
        rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        unit_scale = 1.0 / 100.0
    elif format == "motive_bj":
        name_mapping = BJ_NAME_MAPPING
        rotation_matrix = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        unit_scale = 1.0 / 1.0
    elif format == "motive_sz":
        name_mapping = SZ_NAME_MAPPING
        rotation_matrix = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        unit_scale = 1.0 / 100.0
    else:
        raise ValueError(f"Invalid format: {format}")

    rotation_quat = R.from_matrix(rotation_matrix).as_quat(scalar_first=True)
    qR_inv = utils.quat_inv(rotation_quat)

    data, data_fps = read_bvh(bvh_file)
    
    # ----- calibration pose ------------------------------------------------------
    N = len(data.bones)
    name_to_idx = {n: i for i, n in enumerate(data.bones)}

    # Local positions: use rest offsets (root translation = 0)
    lpos = np.asarray(data.offsets, dtype=float)[None, :, :].copy()
    lpos[:, 0, :] = 0.0

    # Local rotations: identity everywhere (wxyz), then apply a neutral "A-pose" for arms.
    lrot = np.zeros((1, N, 4), dtype=float)
    lrot[..., 0] = 1.0

    if format == "lafan1":

        # Apply your ZYX A-pose in LOCAL joint space (this is the key)
        for jname, (z, y, x) in LAFAN_APOSE_ZYX.items():
            if jname not in name_to_idx:
                continue
            lrot[0, name_to_idx[jname], :] = _wxyz_from_euler_deg(z, y, x, "zyx")

    else:

        left_bvh  = _find_bvh_name_for_canonical(name_mapping, "LeftArm")
        right_bvh = _find_bvh_name_for_canonical(name_mapping, "RightArm")

        if left_bvh is not None and left_bvh in name_to_idx:
            lrot[0, name_to_idx[left_bvh], :] = _wxyz_from_euler_deg(0.0, 0.0, -arm_pose_deg, "xyz")

        if right_bvh is not None and right_bvh in name_to_idx:
            lrot[0, name_to_idx[right_bvh], :] = _wxyz_from_euler_deg(0.0, 0.0,  arm_pose_deg, "xyz")

    gquat, gpos = utils.quat_fk(lrot, lpos, data.parents)  # (1,N,4), (1,N,3)
    gquat0 = gquat[0]
    gpos0 = gpos[0]

    # convert coordinates + units (same as motion)
    gpos0  = (gpos0 @ rotation_matrix.T) * unit_scale
    gquat0 = utils.quat_mul(rotation_quat, gquat0)
    print("Calibration pose loaded.", gquat0)

    # Name mapping (BVH names -> GMR names)
    pose = {}
    for bvh_name, mapped_name in name_mapping.items():
        if bvh_name not in name_to_idx:
            continue
        i = name_to_idx[bvh_name]
        pose[mapped_name] = (gpos0[i].copy(), gquat0[i].copy())

    head_z = pose["Head"][0][2]
    pose["LeftFootMod"] = (pose["LeftFoot"][0], pose["LeftToeBase"][1])
    pose["RightFootMod"] = (pose["RightFoot"][0], pose["RightToeBase"][1])
    foot_z = min(pose["LeftFootMod"][0][2], pose["RightFootMod"][0][2])
    human_height = head_z - foot_z
    print(f"Detected human height from CALI pose: {human_height:.3f} m")

    # ----- motion frames ------------------------------------------------------
    global_data = utils.quat_fk(data.quats, data.pos, data.parents)


    frames = []
    for frame in range(data.pos.shape[0]):
        result = {}
        for i, bone in enumerate(data.bones):
            orientation = utils.quat_mul(rotation_quat, global_data[0][frame, i])
            if format == "motive_bj":
                position = global_data[1][frame, i] @ rotation_matrix.T
            else:
                position = global_data[1][frame, i] @ rotation_matrix.T / 100  # cm to m
            result[bone] = [position, orientation]
            
        
        modified_result = {}
        for key in name_mapping.keys():
            modified_result[name_mapping[key]] = result[key]
        modified_result["LeftFootMod"] = (modified_result["LeftFoot"][0], modified_result["LeftToeBase"][1])
        modified_result["RightFootMod"] = (modified_result["RightFoot"][0], modified_result["RightToeBase"][1])
        frames.append(modified_result)

    
    # human_height = result["Head"][0][2] - min(modified_result["LeftFootMod"][0][2], modified_result["RightFootMod"][0][2])
    # print(f"Detected human height from BVH: {human_height:.3f} m")
    # human_height = human_height + 0.2  # cm to m
    # human_height = 1.75  # cm to m

    return frames, human_height, data_fps, pose
