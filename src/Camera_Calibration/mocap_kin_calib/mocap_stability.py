"""Route captures by observed torso--shoulder stability, without shoulder URDF.

The four channels have already had recorder link offsets removed. For each hand,
T_torso_hand @ inverse(T_shoulder_hand) recovers T_torso_shoulder in those recorded
reference frames. The hand offset cancels. Translation variability depends on the
recorded shoulder origin; rotation variability does not depend on those offsets.
Thresholds are empirical routing choices, not estimates of measurement truth.
"""
from pathlib import Path
import json

import numpy as np
from scipy.spatial.transform import Rotation


def _pose(observation, label):
    if not isinstance(observation, dict):
        raise ValueError(f"{label}: missing or invalid pose")
    try:
        xyz = np.asarray(observation.get("xyz_mm"), dtype=float)
        quat = np.asarray(observation.get("quaternion_xyzw"), dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label}: invalid xyz_mm or quaternion_xyzw") from exc
    if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
        raise ValueError(f"{label}: xyz_mm must contain 3 finite values")
    if quat.shape != (4,) or not np.all(np.isfinite(quat)) or np.linalg.norm(quat) < 1e-12:
        raise ValueError(f"{label}: quaternion_xyzw must contain a finite nonzero quaternion")
    if "valid_frames" in observation:
        frames = observation["valid_frames"]
        if isinstance(frames, bool) or not isinstance(frames, int) or frames <= 0:
            raise ValueError(f"{label}: valid_frames must be a positive integer")
    return xyz, Rotation.from_quat(quat)


def analyze_capture(path: Path, max_translation_mm=8.0, max_rotation_deg=1.0):
    """Return JSON-serializable routing evidence from every recorded sample.

    Either arm exceeding either threshold selects ``qingtong``; otherwise the
    mode is ``shared``. Missing/invalid channels are errors, never a shared-mode
    fallback. At least two samples are needed; the caller checks the required
    capture count for its workflow (currently 30 fitting samples).
    """
    thresholds = {}
    for name, value in (("max_translation_mm", max_translation_mm),
                        ("max_rotation_deg", max_rotation_deg)):
        if isinstance(value, bool):
            raise ValueError(f"{name} must be finite and positive")
        try:
            value = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be finite and positive") from exc
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
        thresholds[name] = value

    path = Path(path)
    capture = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(capture, dict):
        raise ValueError("capture must be a JSON object")
    samples = capture.get("samples")
    if not isinstance(samples, list) or len(samples) < 2:
        raise ValueError("capture.samples must contain at least 2 samples")
    indices = []
    poses = {"left": [], "right": []}
    for sample in samples:
        if not isinstance(sample, dict):
            raise ValueError("capture sample must be an object")
        index = sample.get("index")
        if isinstance(index, bool) or not isinstance(index, int):
            raise ValueError("capture sample index must be an integer")
        if index in indices:
            raise ValueError(f"duplicate capture index {index}")
        indices.append(index)
        bodies = sample.get("bodies")
        if not isinstance(bodies, dict):
            raise ValueError(f"sample {index}: bodies must be an object")
        for arm, hand in (("left", "l_hand"), ("right", "r_hand")):
            torso_key, shoulder_key = f"{hand}_in_torso", f"{hand}_in_l_shoulder"
            p_t, r_t = _pose(bodies.get(torso_key), f"sample {index} {torso_key}")
            p_s, r_s = _pose(bodies.get(shoulder_key), f"sample {index} {shoulder_key}")
            r_ts = r_t * r_s.inv()
            poses[arm].append((p_t - r_ts.apply(p_s), r_ts.as_quat()))

    first, second = np.triu_indices(len(samples), k=1)
    report = {
        "schema_version": 1,
        "capture": str(path.resolve()),
        "sample_count": len(samples),
        "mode": "shared",
        "thresholds": thresholds,
        "measurement_definition": (
            "T_torso_shoulder = T_torso_hand @ inverse(T_shoulder_hand); "
            "recorded offset-corrected reference origins, no shoulder URDF; "
            "mean and maximum over all distinct sample pairs; "
            "maximum pairwise differences drive empirical routing thresholds"
        ),
        "arms": {},
    }
    for arm, values in poses.items():
        positions = np.asarray([p for p, _ in values])
        rotations = Rotation.from_quat([q for _, q in values])
        distances = np.linalg.norm(positions[first] - positions[second], axis=1)
        angles = np.rad2deg((rotations[first].inv() * rotations[second]).magnitude())
        pos_pair, rot_pair = int(np.argmax(distances)), int(np.argmax(angles))
        metrics = {
            "sample_count": len(samples),
            "mean_translation_mm": float(np.mean(distances)),
            "mean_rotation_deg": float(np.mean(angles)),
            "max_translation_mm": float(distances[pos_pair]),
            "max_rotation_deg": float(angles[rot_pair]),
            "translation_pair_indices": [indices[first[pos_pair]], indices[second[pos_pair]]],
            "rotation_pair_indices": [indices[first[rot_pair]], indices[second[rot_pair]]],
        }
        metrics["exceeds_translation"] = metrics["max_translation_mm"] > thresholds["max_translation_mm"]
        metrics["exceeds_rotation"] = metrics["max_rotation_deg"] > thresholds["max_rotation_deg"]
        if metrics["exceeds_translation"] or metrics["exceeds_rotation"]:
            report["mode"] = "qingtong"
        report["arms"][arm] = metrics
    return report
