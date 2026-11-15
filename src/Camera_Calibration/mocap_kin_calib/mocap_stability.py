"""Report observed torso--shoulder stability, without shoulder URDF.

The four channels have already had recorder link offsets removed. For each hand,
T_torso_hand @ inverse(T_shoulder_hand) recovers T_torso_shoulder in those recorded
reference frames. The hand offset cancels. Translation variability depends on the
recorded shoulder origin; rotation variability does not depend on those offsets.
Translation thresholds use distance from each chain's original-sample mean,
computed once before exclusions. Pairwise distances and rotation are diagnostic
only; quality never selects a CTB fit mode.
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


def analyze_capture(path: Path, max_translation_mm=8.0, max_rotation_deg=1.0,
                    exclude_translation_mm=3.0, max_excluded_poses=3):
    """Return JSON-serializable stability diagnostics from every recorded sample.

    Compute each chain's position mean once over all original samples. Reject
    any distance above the hard limit; otherwise remove the union of poses
    above the exclusion limit, or reject if that union exceeds the budget.
    Never recompute the mean after removal. Rotation remains diagnostic only.
    Missing/invalid channels are errors. At least two samples are needed;
    the caller checks the required capture count for its workflow (currently
    30 fitting samples).
    """
    thresholds = {}
    for name, value in (("max_translation_mm", max_translation_mm),
                        ("exclude_translation_mm", exclude_translation_mm),
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
    if thresholds["exclude_translation_mm"] > thresholds["max_translation_mm"]:
        raise ValueError("exclude_translation_mm must not exceed max_translation_mm")
    if (isinstance(max_excluded_poses, bool) or not isinstance(max_excluded_poses, int)
            or not 0 <= max_excluded_poses <= 3):
        raise ValueError("max_excluded_poses must be an integer in [0, 3]")
    thresholds["max_excluded_poses"] = max_excluded_poses

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
        "schema_version": 3,
        "quality_metric": "distance_from_original_mean",
        "capture": str(path.resolve()),
        "sample_count": len(samples),
        "thresholds": thresholds,
        "measurement_definition": (
            "T_torso_shoulder = T_torso_hand @ inverse(T_shoulder_hand); "
            "recorded offset-corrected reference origins, no shoulder URDF; "
            "Euclidean distance from each chain's original-sample position mean; "
            "mean fixed before exclusions, never recomputed; "
            "pairwise distances and rotation are diagnostic only"
        ),
        "arms": {},
        "poses": [{"index": index, "arms": {}} for index in indices],
    }
    deviations = []
    for arm, values in poses.items():
        positions = np.asarray([p for p, _ in values])
        rotations = Rotation.from_quat([q for _, q in values])
        reference_mean = positions.mean(axis=0)
        delta = positions - reference_mean
        radii = np.linalg.norm(delta, axis=1)
        deviations.append(radii)
        distances = np.linalg.norm(positions[first] - positions[second], axis=1)
        angles = np.rad2deg((rotations[first].inv() * rotations[second]).magnitude())
        pos_pair, rot_pair = int(np.argmax(distances)), int(np.argmax(angles))
        metrics = {
            "sample_count": len(samples),
            "reference_mean_xyz_mm": reference_mean.tolist(),
            "mean_translation_mm": float(np.mean(radii)),
            "mean_rotation_deg": float(np.mean(angles)),
            "max_translation_mm": float(radii.max()),
            "max_translation_pose_index": indices[int(np.argmax(radii))],
            "max_rotation_deg": float(angles[rot_pair]),
            "rotation_pair_indices": [indices[first[rot_pair]], indices[second[rot_pair]]],
            "pairwise": {
                "mean_translation_mm": float(distances.mean()),
                "max_translation_mm": float(distances[pos_pair]),
                "translation_pair_indices": [indices[first[pos_pair]], indices[second[pos_pair]]],
            },
        }
        metrics["exceeds_translation"] = _exceeds(metrics["max_translation_mm"], thresholds["max_translation_mm"])
        metrics["exceeds_rotation"] = metrics["max_rotation_deg"] > thresholds["max_rotation_deg"]
        report["arms"][arm] = metrics
        for row, item in enumerate(report["poses"]):
            item["arms"][arm] = {
                "translation_mm": float(radii[row]),
                "delta_xyz_mm": delta[row].tolist(),
                "exceeds_exclusion_limit": bool(_exceeds(radii[row], thresholds["exclude_translation_mm"])),
                "exceeds_hard_limit": bool(_exceeds(radii[row], thresholds["max_translation_mm"])),
            }
    combined = np.max(deviations, axis=0)
    required = sorted(indices[i] for i in np.flatnonzero(_exceeds(combined, thresholds["exclude_translation_mm"])))
    hard = sorted(indices[i] for i in np.flatnonzero(_exceeds(combined, thresholds["max_translation_mm"])))
    quality = {"status": "pass", "reason": "", "excluded_indices": [],
               "retained_indices": sorted(indices), "required_excluded_indices": required,
               "hard_limit_pose_indices": hard, "problem_pose_indices": required}
    report["quality"] = quality
    if hard:
        quality.update(status="error", reason="hard_translation_limit")
    elif len(required) > max_excluded_poses:
        quality.update(status="error", reason="exclusion_limit")
    elif len(indices) - len(required) < 2:
        quality.update(status="error", reason="insufficient_retained_poses")
    elif required:
        quality.update(status="filtered", excluded_indices=required,
                       retained_indices=sorted(set(indices) - set(required)))
    retained = set(quality["retained_indices"])
    keep = np.array([index in retained for index in indices])
    quality["retained_max_translation_mm"] = {
        arm: float(radii[keep].max()) for arm, radii in zip(("left", "right"), deviations)}
    return report


def _exceeds(value, threshold):
    # Ignore sub-nanometre transform roundoff at exactly 3 / 8 mm.
    return value > threshold + 1e-9


def check_capture_quality(path, config, report_path=None):
    """Print pose IDs, persist the report even on rejection, and enforce gates."""
    options = config.get("workflow", {}).get("shoulder_stability", {})
    report = analyze_capture(path, **options)
    if report_path is not None:
        Path(report_path).write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    limits, quality = report["thresholds"], report["quality"]
    print(f"torso—shoulder 质量门限: 左右链各取一次原始 {report['sample_count']} 点位置均值，删点后不重算；"
          f"距原始均值 > {limits['max_translation_mm']:g} mm 退出；"
          f"> {limits['exclude_translation_mm']:g} mm 剔除，最多 {limits['max_excluded_poses']} 个 pose。")
    for arm, metrics in report["arms"].items():
        print(f"  {arm}: 原始位置均值 {metrics['reference_mean_xyz_mm']} mm；"
              f"距均值最大 {metrics['max_translation_mm']:.6f} mm，pose {metrics['max_translation_pose_index']}；"
              f"两两位置最大 {metrics['pairwise']['max_translation_mm']:.6f} mm、"
              f"旋转最大 {metrics['max_rotation_deg']:.6f} deg（两两指标仅诊断）")
    involved = set(quality["problem_pose_indices"])
    if involved:
        print("距原始均值超限的 pose（左右同步剔除，重复编号只计一次）:")
        for item in sorted(report["poses"], key=lambda p: p["index"]):
            if item["index"] in involved:
                details = "; ".join(f"{arm}: {v['translation_mm']:.6f} mm，Δxyz={v['delta_xyz_mm']} mm"
                                    for arm, v in item["arms"].items())
                print(f"  pose {item['index']}: {details}")
    if quality["status"] == "error":
        if quality["reason"] == "hard_translation_limit":
            detail = (f"pose {quality['hard_limit_pose_indices']} 距原始均值超过 "
                      f"{limits['max_translation_mm']:g} mm")
        elif quality["reason"] == "exclusion_limit":
            detail = (f"距原始均值超过 {limits['exclude_translation_mm']:g} mm 的 pose 共 "
                      f"{len(quality['required_excluded_indices'])} 个，超过最多剔除 {limits['max_excluded_poses']} 个")
        else:
            detail = "剔除后不足 2 个 pose"
        raise ValueError(f"数据质量错误：{detail}；超限 pose {quality['problem_pose_indices']}，停止拟合。")
    print(f"质量筛选: 剔除 pose {quality['excluded_indices']}；保留 {len(quality['retained_indices'])} 点；"
          f"剩余距原始均值最大偏差 {quality['retained_max_translation_mm']} mm（均值未重算）。")
    return report
