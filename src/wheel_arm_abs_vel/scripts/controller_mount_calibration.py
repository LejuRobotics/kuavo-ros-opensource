"""Validated controller mounting calibration for the full-body SG100 mapper."""
import copy
import json
import numpy as np


def checked_calibration(config, robot_version, model_sha256=None):
    if not isinstance(config, dict):
        raise ValueError("controller_calibration must be a mapping")
    if not config.get("enabled", False):
        return None
    if (config.get("schema_version") != 2 or
            config.get("eef_convention") != "sg100_400062"):
        raise ValueError("Full-body SG100 requires schema 2 / sg100_400062; legacy wheel rotations are incompatible")
    if int(robot_version) != 400062 or config.get("robot_version") != 400062:
        raise ValueError("Controller calibration robot version mismatch")
    digest = config.get("model_sha256", "")
    if (not isinstance(digest, str) or len(digest) != 64 or
            any(c not in "0123456789abcdef" for c in digest)):
        raise ValueError("Controller calibration requires the exact URDF SHA256")
    if model_sha256 is not None and digest != model_sha256:
        raise ValueError("Controller calibration model SHA256 mismatch")
    metadata = config.get("metadata", {})
    if (metadata.get("quality_gate") != "activation_eligible" or
            metadata.get("orientation_calibrated") is not True):
        raise ValueError("Controller calibration requires passed holds and semantic orientation calibration")
    result = copy.deepcopy(config)
    for side in ("left", "right"):
        values = config.get(side, {})
        offset = np.asarray(values.get("palm_to_controller_offset"), dtype=float)
        quat = np.asarray(values.get("eef_post_rotation_xyzw"), dtype=float)
        if offset.shape != (3,) or not np.isfinite(offset).all() or np.linalg.norm(offset) > .25:
            raise ValueError(side + ": invalid mounting offset (finite metres, <= 0.25 m)")
        if metadata.get("translation_calibrated") is False and np.any(offset != 0.):
            raise ValueError(side + ": unknown translation must not apply a nonzero offset")
        if quat.shape != (4,) or not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1.) > .001:
            raise ValueError(side + ": expected a unit xyzw calibration quaternion")
        result[side]["palm_to_controller_offset"] = offset
        result[side]["eef_post_rotation_xyzw"] = quat / np.linalg.norm(quat)
    return result


def controller_rotations(adapters, calibration, quaternion_to_matrix):
    """Compose B*C after the fixed raw-palm adapter F; optical hands are untouched."""
    if calibration is None:
        return
    for side in ("Left", "Right"):
        angle = np.radians(-15.0 if side == "Left" else 15.0)
        c, s = np.cos(angle), np.sin(angle)
        baseline = np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])
        if not np.allclose(adapters["controller"][side][:3, :3], baseline, atol=1e-10, rtol=0):
            raise ValueError(side + ": controller RPY differs from the calibration baseline (+/-15 degrees)")
        correction = quaternion_to_matrix(calibration[side.lower()]["eef_post_rotation_xyzw"])
        adapters["controller"][side][:3, :3] = adapters["controller"][side][:3, :3] @ correction


def status_json(calibration, stage, offsets=None):
    """Record effective input geometry without depending on a running parameter server."""
    result = {"stage": stage, "enabled": calibration is not None,
              "raw_pose_topic": "/leju_quest_bone_poses_raw"}
    if calibration:
        for name in ("schema_version", "eef_convention", "robot_version", "model_sha256"):
            result[name] = calibration[name]
        metadata = calibration.get("metadata", {})
        result["translation_calibrated"] = metadata.get("translation_calibrated")
        result["translation_status"] = metadata.get("translation_status", "unspecified")
        result["distance_source"] = metadata.get("distance_source", "unspecified")
        result["known_distance_m"] = metadata.get("known_distance_m")
        result["eef_post_rotation_xyzw"] = {
            side: calibration[side]["eef_post_rotation_xyzw"].tolist()
            for side in ("left", "right")}
    if offsets is not None:
        result["effective_palm_to_controller_offset"] = {
            side.lower(): np.asarray(offsets.get(side, [0., 0., 0.])).tolist()
            for side in ("Left", "Right")}
    return json.dumps(result, sort_keys=True)
