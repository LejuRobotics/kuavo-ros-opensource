#!/usr/bin/env python3
"""Mocap kinematic fit: 25 FIT -> 5 VALIDATION -> 30 FINAL.

All translations in this module are millimetres, joint angles are radians and
rotation vectors are degrees.  CTB and ETK translations are fixed by YAML;
their rotations and the fourteen local Ei corrections are fitted.  CTB/ETK
are measurement transforms and are therefore never written into the URDF.
The final fit exports nominal origins, additive origin offsets, and Ei to
kinematic_offsets.json, plus frame_transforms.json.  JSON files are replaced
after validation passes.  --output-urdf also builds a mechanical URDF from
these JSON files; replacing an existing URDF requires --force-output.
Final output requires every Ei translation norm <= 15 mm and rotation-vector
norm <= 10 deg; exceeding either limit leaves existing outputs unchanged.
After --output-urdf succeeds, type 'yes' to back up and replace the runtime
models (all S63 URDF variants, or the main URDF for other versions).
Validation and JSON-only fitting never prompt or install.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import write_urdf


ARMS = ("left", "right")
CALIBRATION_COMMENT_PREFIX = "mocap_kin_calib:"
EI_OUTPUT_TRANSLATION_LIMIT_MM = 15.0
EI_OUTPUT_ROTATION_LIMIT_DEG = 10.0


def _finite_vector(value, size, name):
    out = np.asarray(value, dtype=float)
    if out.shape != (size,) or not np.isfinite(out).all():
        raise ValueError(f"{name} must be a finite vector of length {size}")
    return out


def _rigid(translation_mm, rotvec_deg):
    out = np.eye(4)
    out[:3, 3] = _finite_vector(translation_mm, 3, "translation_mm")
    out[:3, :3] = Rotation.from_rotvec(
        np.deg2rad(_finite_vector(rotvec_deg, 3, "rotvec_deg"))
    ).as_matrix()
    return out


def _origin(element):
    out = np.eye(4)
    if element is None:
        return out
    xyz = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
    rpy = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
    if xyz.shape != (3,) or rpy.shape != (3,):
        raise ValueError("URDF origin requires three xyz and three rpy values")
    out[:3, 3] = xyz * 1000.0
    out[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    return out


def _xml_tree(path):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True, insert_pis=True))
    return ET.parse(path, parser=parser)


@dataclass
class ArmChain:
    urdf_path: Path
    root_link: str
    tip_link: str
    joint_names: List[str]
    origins: np.ndarray
    axes: np.ndarray
    tip_transform: np.ndarray

    def rotations(self, q):
        q = np.atleast_2d(np.asarray(q, dtype=float))
        if q.shape != (len(q), 7) or not np.isfinite(q).all():
            raise ValueError("q must have shape (N,7) and finite radians")
        motion = np.broadcast_to(np.eye(4), (len(q), 7, 4, 4)).copy()
        rv = q[:, :, None] * self.axes[None, :, :]
        motion[:, :, :3, :3] = Rotation.from_rotvec(
            rv.reshape(-1, 3)
        ).as_matrix().reshape(len(q), 7, 3, 3)
        return motion

    def fk(self, q, corrections=None):
        motion = self.rotations(q)
        if corrections is None:
            corrections = np.broadcast_to(np.eye(4), (7, 4, 4))
        corrections = np.asarray(corrections, dtype=float)
        if corrections.shape != (7, 4, 4):
            raise ValueError("corrections must have shape (7,4,4)")
        out = np.broadcast_to(np.eye(4), (len(motion), 4, 4)).copy()
        for i in range(7):
            out = out @ (self.origins[i] @ corrections[i]) @ motion[:, i]
        return out @ self.tip_transform


def load_chain(path: Path, arm: str, root_link: str, tip_link: str) -> ArmChain:
    root = _xml_tree(path).getroot()
    parents = {j.find("child").get("link"): j for j in root.findall("joint")}
    path_joints, visited, link = [], set(), tip_link
    while link != root_link:
        if link in visited or link not in parents:
            raise ValueError(f"no URDF path {root_link} -> {tip_link}")
        visited.add(link)
        joint = parents[link]
        path_joints.append(joint)
        link = joint.find("parent").get("link")
    path_joints.reverse()
    fixed = np.eye(4)
    origins, axes, names = [], [], []
    for joint in path_joints:
        fixed = fixed @ _origin(joint.find("origin"))
        if joint.get("type") == "fixed":
            continue
        if joint.get("type") not in ("revolute", "continuous") or joint.find("mimic") is not None:
            raise ValueError(f"unsupported joint: {joint.get('name')}")
        axis_node = joint.find("axis")
        axis = np.fromstring(
            "1 0 0" if axis_node is None else axis_node.get("xyz", "1 0 0"), sep=" "
        )
        if axis.shape != (3,) or np.linalg.norm(axis) == 0:
            raise ValueError(f"invalid axis on {joint.get('name')}")
        origins.append(fixed)
        axes.append(axis / np.linalg.norm(axis))
        names.append(joint.get("name"))
        fixed = np.eye(4)
    prefix = "l" if arm == "left" else "r"
    expected = [f"zarm_{prefix}{i}_joint" for i in range(1, 8)]
    if names != expected:
        raise ValueError(f"expected {expected}, found {names}")
    return ArmChain(path.resolve(), root_link, tip_link, names,
                    np.asarray(origins), np.asarray(axes), fixed)


def _body_map(capture):
    bodies = capture.get("bodies")
    if not isinstance(bodies, list):
        raise ValueError("capture.bodies must be a list")
    result = {}
    for body in bodies:
        name = body.get("name")
        if not name or name in result:
            raise ValueError(f"duplicate or missing body name: {name!r}")
        result[name] = body
    for name in ("l_hand", "r_hand", "torso"):
        if name not in result:
            raise ValueError(f"capture missing body {name}")
    return result


def _capture_pose_to_matrix(obs):
    xyz = _finite_vector(obs.get("xyz_mm"), 3, "xyz_mm")
    quat = _finite_vector(obs.get("quaternion_xyzw"), 4, "quaternion_xyzw")
    if np.linalg.norm(quat) < 1e-12:
        raise ValueError("zero quaternion")
    out = np.eye(4)
    out[:3, 3] = xyz
    out[:3, :3] = Rotation.from_quat(quat).as_matrix()
    return out


def _indices(values, name):
    try:
        result = [int(v) for v in values]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be integer indices") from exc
    if len(result) != len(set(result)):
        raise ValueError(f"{name} contains duplicate indices")
    return result


def load_dataset(capture_path: Path, cfg: Mapping, *, validation_only=False):
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    meta = capture.get("meta", {})
    kin = cfg["kinematics"]
    expected_layout = cfg["robot"].get("robot_layout", "biped45")
    if meta.get("robot_layout") not in (None, expected_layout):
        raise ValueError(f"capture robot_layout={meta.get('robot_layout')!r}, expected {expected_layout!r}")
    if meta.get("fk_root") not in (None, kin["root_link"]):
        raise ValueError(f"capture fk_root={meta.get('fk_root')!r}, expected {kin['root_link']!r}")
    # The recorder's URDF is used for its FK display. Stored joint angles and
    # mocap poses do not depend on it, so an arm-only display URDF is accepted
    # when fitting or validating the full-body model of the same robot.
    samples = capture.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError("capture.samples is empty")
    by_index = {}
    for sample in samples:
        index = int(sample["index"])
        if index in by_index:
            raise ValueError(f"duplicate capture index {index}")
        by_index[index] = sample
    if validation_only:
        # Frozen validation uses every recorded pose, in capture order. The
        # 25/5 split belongs only to calibration and is not applied here.
        fit_indices = []
        validation_indices = list(by_index)
    else:
        fit_indices = _indices(kin["fit_indices"], "fit_indices")
        validation_indices = _indices(kin["validation_indices"], "validation_indices")
        if set(fit_indices) & set(validation_indices) or len(fit_indices) != 25 or len(validation_indices) != 5:
            raise ValueError("fit_indices must contain 25 and validation_indices 5 disjoint entries")
        missing = (set(fit_indices) | set(validation_indices)) - set(by_index)
        if missing:
            raise ValueError(f"capture is missing configured indices: {sorted(missing)}")
    bodies = _body_map(capture)
    torso_offset = _finite_vector(bodies["torso"].get("link_offset_mm"), 3, "torso link_offset_mm")
    hand_offsets = {
        arm: _finite_vector(bodies["l_hand" if arm == "left" else "r_hand"].get("link_offset_mm"), 3,
                             f"{arm} hand link_offset_mm")
        for arm in ARMS
    }
    # Record format stores hand_in_torso after link offsets were removed. Restore
    # raw marker-frame relative poses exactly once for the fitting model.
    data = {"capture_path": str(capture_path)}
    q_indices = meta.get("joint_q_indices", cfg["robot"]["joint_indices"])
    for arm in ARMS:
        lo, hi = [int(v) for v in q_indices[arm + "_arm"]]
        expected_lo, expected_hi = [int(v) for v in cfg["robot"]["joint_indices"][arm + "_arm"]]
        if (lo, hi) != (expected_lo, expected_hi):
            raise ValueError(
                f"capture {arm} joint_q_indices={(lo, hi)} do not match configured "
                f"{arm} indices={(expected_lo, expected_hi)}"
            )
        if hi - lo != 7:
            raise ValueError(f"{arm} joint_q_indices must span 7 entries")
        qs, observations = [], []
        hand_name = "l_hand" if arm == "left" else "r_hand"
        for index in fit_indices + validation_indices:
            sample = by_index[index]
            q_values = sample.get("joint_q_full")
            if not isinstance(q_values, (list, tuple)):
                raise ValueError(f"joint_q_full must be a list at capture index {index}")
            q_full = _finite_vector(q_values, len(q_values), "joint_q_full")
            if hi > len(q_full):
                raise ValueError(f"{arm} q index out of range at capture index {index}")
            obs = sample.get("bodies", {}).get(hand_name + "_in_torso")
            if not isinstance(obs, dict):
                raise ValueError(f"missing {hand_name}_in_torso at capture index {index}")
            saved = _capture_pose_to_matrix(obs)
            marker = saved.copy()
            marker[:3, 3] += saved[:3, :3] @ hand_offsets[arm] - torso_offset
            qs.append(q_full[lo:hi])
            observations.append(marker)
        data[arm + "_q"] = np.asarray(qs)
        data[arm + "_marker_T"] = np.asarray(observations)
    data["fit_count"] = len(fit_indices)
    data["validation_count"] = len(validation_indices)
    data["fit_indices"] = fit_indices
    data["validation_indices"] = validation_indices
    return data


class FixedPositionModel:
    """Fixed CTB/ETK translations, fitted directions and local Ei matrices."""

    def __init__(self, kin: Mapping, chains: Mapping[str, ArmChain]):
        self.kin, self.chains = kin, chains
        ctb = kin["ctb"]
        if ctb.get("fit_translation") or not ctb.get("fit_rotation", True):
            raise ValueError("This workflow requires fixed CTB position and fitted CTB direction")
        self.ctb_t = _finite_vector(ctb["translation_mm"], 3, "CTB translation_mm")
        self.ctb_r0 = _finite_vector(ctb.get("initial_rotvec_deg", [0, 0, 0]), 3, "CTB initial_rotvec_deg")
        self.etk_t, self.etk_r0 = {}, {}
        for arm in ARMS:
            item = kin["etk"][arm]
            if item.get("fit_translation") or not item.get("fit_rotation", True):
                raise ValueError(f"This workflow requires fixed {arm} ETK position and fitted direction")
            self.etk_t[arm] = _finite_vector(item["translation_mm"], 3, f"{arm} ETK translation_mm")
            self.etk_r0[arm] = _finite_vector(item.get("initial_rotvec_deg", [0, 0, 0]), 3,
                                               f"{arm} ETK initial_rotvec_deg")
        # p = CTB rot(3), left ETK rot(3), right ETK rot(3),
        #     left E1..E7 (42), right E1..E7 (42).
        self.ctb_slice = slice(0, 3)
        self.etk_slice = {"left": slice(3, 6), "right": slice(6, 9)}
        self.ei_slice = {"left": slice(9, 51), "right": slice(51, 93)}
        self.parameter_count = 93

    def unpack(self, p, arm):
        p = np.asarray(p, dtype=float)
        ctb = _rigid(self.ctb_t, self.ctb_r0 + p[self.ctb_slice])
        etk = _rigid(self.etk_t[arm], self.etk_r0[arm] + p[self.etk_slice[arm]])
        blocks = p[self.ei_slice[arm]].reshape(7, 6)
        corrections = np.asarray([_rigid(block[:3], block[3:]) for block in blocks])
        return ctb, etk, corrections

    def predict(self, p, arm, q):
        ctb, etk, corrections = self.unpack(p, arm)
        return np.linalg.inv(ctb) @ self.chains[arm].fk(q, corrections) @ etk

    def residual(self, p, data, subset):
        # Data order is exactly FIT followed by VALIDATION.
        if subset == "fit":
            n, offset = data["fit_count"], 0
        elif subset == "validation":
            n, offset = data["validation_count"], data["fit_count"]
        elif subset == "all":
            n, offset = data["fit_count"] + data["validation_count"], 0
        else:
            raise ValueError(f"unknown data subset {subset!r}")
        rows = []
        residual_cfg = self.kin.get("residual", {})
        use_position = bool(residual_cfg.get("position", True))
        use_orientation = bool(residual_cfg.get("orientation", False))
        if not (use_position or use_orientation):
            raise ValueError("at least one residual component must be enabled")
        p_scale = float(residual_cfg.get("position_scale_mm", 1.0))
        r_scale = float(residual_cfg.get("orientation_scale_deg", 1.0))
        if p_scale <= 0 or r_scale <= 0:
            raise ValueError("residual scales must be positive")
        for arm in ARMS:
            q = data[arm + "_q"][offset:offset + n]
            obs = data[arm + "_marker_T"][offset:offset + n]
            pred = self.predict(p, arm, q)
            if use_position:
                rows.append(((pred[:, :3, 3] - obs[:, :3, 3]) / p_scale).ravel())
            if use_orientation:
                rel = pred[:, :3, :3] @ obs[:, :3, :3].transpose(0, 2, 1)
                rot = np.rad2deg(Rotation.from_matrix(rel).as_rotvec())
                rows.append((rot / r_scale).ravel())
        # Ei priors are explicit numerical regularization, not a data filter.
        ei_cfg = self.kin.get("ei", {})
        tp = float(ei_cfg.get("translation_prior_mm", 0.0))
        rp = float(ei_cfg.get("rotation_prior_deg", 0.0))
        if tp < 0 or rp < 0:
            raise ValueError("Ei prior scales must be nonnegative")
        if tp > 0 or rp > 0:
            priors = []
            for arm in ARMS:
                blocks = p[self.ei_slice[arm]].reshape(7, 6)
                if tp > 0:
                    priors.append((blocks[:, :3] / tp).ravel())
                if rp > 0:
                    priors.append((blocks[:, 3:] / rp).ravel())
            rows.extend(priors)
        return np.concatenate(rows)

    def bounds(self):
        ei_cfg = self.kin.get("ei", {})
        tb_value = ei_cfg.get("translation_bound_mm")
        rb_value = ei_cfg.get("rotation_bound_deg")
        tb = np.inf if tb_value is None else float(tb_value)
        rb = np.inf if rb_value is None else float(rb_value)
        if (tb <= 0 or rb <= 0 or
                np.isnan(tb) or np.isnan(rb) or
                (np.isinf(tb) and tb < 0) or (np.isinf(rb) and rb < 0)):
            raise ValueError("Ei bounds must be positive numbers or null")
        upper = np.full(self.parameter_count, np.inf)
        for arm in ARMS:
            for block in range(7):
                start = self.ei_slice[arm].start + block * 6
                upper[start:start + 3] = tb
                upper[start + 3:start + 6] = rb
        return -upper, upper

    def x_scale(self):
        opt = self.kin.get("optimizer", {})
        # Solver scaling is exposed independently from physical bounds and
        # regularization.  The values are parameter units, not data filters.
        translation_scale = float(opt.get("translation_scale_mm", 1.0))
        rotation_scale = float(opt.get("rotation_scale_deg", 1.0))
        if (translation_scale <= 0 or rotation_scale <= 0 or
                not np.isfinite([translation_scale, rotation_scale]).all()):
            raise ValueError("optimizer translation/rotation scales must be finite and positive")
        values = np.ones(self.parameter_count)
        values[self.ctb_slice] = rotation_scale
        for sl in self.etk_slice.values():
            values[sl] = rotation_scale
        for sl in self.ei_slice.values():
            values[sl.start:sl.start + 42] = np.tile([
                translation_scale, translation_scale, translation_scale,
                rotation_scale, rotation_scale, rotation_scale,
            ], 7)
        if np.any(values <= 0) or not np.isfinite(values).all():
            raise ValueError("optimizer x_scale must be finite and positive")
        return values


def _check_ei_output_limits(model: FixedPositionModel, p):
    """Reject final Ei vector norms above 15 mm / 10 deg before any output."""
    parameters = _finite_vector(p, model.parameter_count, "calibration parameters")
    violations = []
    for arm in ARMS:
        blocks = parameters[model.ei_slice[arm]].reshape(7, 6)
        for name, block in zip(model.chains[arm].joint_names, blocks):
            translation_mm = float(np.linalg.norm(block[:3]))
            rotation_deg = float(np.linalg.norm(block[3:]))
            if (translation_mm > EI_OUTPUT_TRANSLATION_LIMIT_MM or
                    rotation_deg > EI_OUTPUT_ROTATION_LIMIT_DEG):
                violations.append(
                    f"{name}: ||translation_mm||={translation_mm:.17g} mm "
                    f"(limit {EI_OUTPUT_TRANSLATION_LIMIT_MM:g} mm), "
                    f"||rotvec_deg||={rotation_deg:.17g} deg "
                    f"(limit {EI_OUTPUT_ROTATION_LIMIT_DEG:g} deg)"
                )
    if violations:
        raise ValueError(
            "Ei output limits exceeded; final JSON/URDF were not generated "
            "or replaced:\n  " + "\n  ".join(violations)
        )


def fit(model: FixedPositionModel, data, subset: str, initial=None):
    opt = model.kin.get("optimizer", {})
    p0 = np.zeros(model.parameter_count) if initial is None else np.asarray(initial, dtype=float).copy()
    if p0.shape != (model.parameter_count,) or not np.isfinite(p0).all():
        raise ValueError("invalid optimizer initial parameters")
    lo, hi = model.bounds()
    if np.any(p0 < lo) or np.any(p0 > hi):
        raise ValueError("optimizer initial parameters exceed Ei bounds")
    result = least_squares(
        lambda p: model.residual(p, data, subset),
        p0,
        bounds=(lo, hi),
        x_scale=model.x_scale(),
        max_nfev=int(opt.get("max_nfev", 250)),
        ftol=float(opt.get("ftol", 1e-9)),
        xtol=float(opt.get("xtol", 1e-9)),
        gtol=float(opt.get("gtol", 1e-8)),
        method="trf",
    )
    return result.x, result


def fit_frame_angles(model: FixedPositionModel, data, subset: str):
    """Fit only the nine CTB/ETK rotation increments; keep candidate Ei fixed."""
    frame_count = 9
    x0 = np.zeros(frame_count)
    opt = model.kin.get("optimizer", {})

    def residual(frame_delta):
        p = np.zeros(model.parameter_count)
        p[:frame_count] = frame_delta
        return model.residual(p, data, subset)

    result = least_squares(
        residual,
        x0,
        x_scale=model.x_scale()[:frame_count],
        max_nfev=int(opt.get("max_nfev", 250)),
        ftol=float(opt.get("ftol", 1e-9)),
        xtol=float(opt.get("xtol", 1e-9)),
        gtol=float(opt.get("gtol", 1e-8)),
        method="trf",
    )
    p = np.zeros(model.parameter_count)
    p[:frame_count] = result.x
    return p, result


def error_rows(model, p, data, subset):
    if subset == "fit":
        n, offset, indices = data["fit_count"], 0, data["fit_indices"]
    elif subset == "validation":
        n, offset, indices = data["validation_count"], data["fit_count"], data["validation_indices"]
    elif subset == "all":
        n, offset, indices = data["fit_count"] + data["validation_count"], 0, data["fit_indices"] + data["validation_indices"]
    else:
        raise ValueError(f"unknown data subset {subset!r}")
    rows = []
    for arm in ARMS:
        q = data[arm + "_q"][offset:offset + n]
        obs = data[arm + "_marker_T"][offset:offset + n]
        pred = model.predict(p, arm, q)
        dp = np.linalg.norm(pred[:, :3, 3] - obs[:, :3, 3], axis=1)
        rel = pred[:, :3, :3] @ obs[:, :3, :3].transpose(0, 2, 1)
        dr = np.rad2deg(Rotation.from_matrix(rel).magnitude())
        rows.extend({"arm": arm, "index": int(i), "position_mm": float(e), "rotation_deg": float(r)}
                    for i, e, r in zip(indices, dp, dr))
    return rows


def print_metrics(label, rows):
    positions = np.asarray([r["position_mm"] for r in rows], dtype=float)
    rotations = np.asarray([r["rotation_deg"] for r in rows], dtype=float)
    # print(f"{label}: position RMS={np.sqrt(np.mean(positions**2)):.6f} mm, "
    print(f"{label}: position mean error={np.mean(positions):.6f} mm, "
          f"max={positions.max():.6f} mm; orientation RMS={np.sqrt(np.mean(rotations**2)):.6f} deg, "
          f"max={rotations.max():.6f} deg")
    for row in rows:
        print(f"  {row['arm']} index={row['index']}: position={row['position_mm']:.6f} mm, "
              f"orientation={row['rotation_deg']:.6f} deg")


def print_frame_parameters(label, model, p):
    ctb = model.unpack(p, "left")[0]
    print(f"{label}: CTB translation_mm={model.ctb_t.tolist()}, "
          f"rotation_deg={np.rad2deg(Rotation.from_matrix(ctb[:3, :3]).as_rotvec()).tolist()}")
    for arm in ARMS:
        etk = model.unpack(p, arm)[1]
        print(f"{label}: {arm} ETK translation_mm={model.etk_t[arm].tolist()}, "
              f"rotation_deg={np.rad2deg(Rotation.from_matrix(etk[:3, :3]).as_rotvec()).tolist()}")


def _frame_metadata(model, p):
    ctb = model.unpack(p, "left")[0]
    result = {
        "schema_version": 1,
        "ctb": {
            "translation_mm": model.ctb_t.tolist(),
            "rotvec_deg": np.rad2deg(Rotation.from_matrix(ctb[:3, :3]).as_rotvec()).tolist(),
            "rotation_matrix": ctb[:3, :3].tolist(),
        },
        "etk": {},
    }
    for arm in ARMS:
        etk = model.unpack(p, arm)[1]
        result["etk"][arm] = {
            "translation_mm": model.etk_t[arm].tolist(),
            "rotvec_deg": np.rad2deg(Rotation.from_matrix(etk[:3, :3]).as_rotvec()).tolist(),
            "rotation_matrix": etk[:3, :3].tolist(),
        }
    return result


def _write_frame_metadata(root, metadata):
    for child in list(root):
        if child.tag is ET.Comment and (child.text or "").strip().startswith(CALIBRATION_COMMENT_PREFIX):
            root.remove(child)
    payload = json.dumps(metadata, sort_keys=True, separators=(",", ":"))
    root.insert(0, ET.Comment(f" {CALIBRATION_COMMENT_PREFIX}{payload} "))


def _normalise_frame_metadata(metadata, path):
    if metadata.get("schema_version") != 1:
        raise ValueError(f"unsupported mocap calibration metadata version in {path}")
    frames = {"ctb": metadata.get("ctb"), "left ETK": metadata.get("etk", {}).get("left"),
              "right ETK": metadata.get("etk", {}).get("right")}
    for name, frame in frames.items():
        if not isinstance(frame, dict):
            raise ValueError(f"{path} is missing {name} frame metadata")
        matrix = frame.get("rotation_matrix")
        if matrix is not None:
            matrix = np.asarray(matrix, dtype=float)
            if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
                raise ValueError(f"{path} has an invalid {name} rotation_matrix")
            frame["rotvec_deg"] = np.rad2deg(Rotation.from_matrix(matrix).as_rotvec()).tolist()
        elif "rotvec_deg" in frame:
            rotvec = _finite_vector(frame["rotvec_deg"], 3, f"{name} rotvec_deg")
            frame["rotation_matrix"] = Rotation.from_rotvec(np.deg2rad(rotvec)).as_matrix().tolist()
        else:
            raise ValueError(f"{path} is missing {name} rotvec_deg/rotation_matrix")
    return metadata


def _read_frame_metadata(path):
    if path.suffix.lower() == ".json":
        metadata = json.loads(path.read_text(encoding="utf-8"))
        return _normalise_frame_metadata(metadata, path)
    root = _xml_tree(path).getroot()
    for child in root:
        text = (child.text or "").strip() if child.tag is ET.Comment else ""
        if text.startswith(CALIBRATION_COMMENT_PREFIX):
            metadata = json.loads(text[len(CALIBRATION_COMMENT_PREFIX):])
            return _normalise_frame_metadata(metadata, path)
    raise ValueError(
        f"{path} does not contain CTB/ETK metadata; regenerate it with this optimizer"
    )


def _write_frame_metadata_file(path, metadata):
    path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _write_joint_offsets_file(path: Path, source: Path, model: FixedPositionModel, p):
    """Save nominal origins, componentwise URDF xyz/rpy offsets, and local Ei.

    nominal_origin + origin_offset recovers the exported xyz_m/rpy_rad values.
    The RPY difference uses the exported Euler branch without angle wrapping;
    it is a component difference, not the rotation vector of the Ei transform.
    The rigid-transform relation remains calibrated origin = nominal origin @ Ei.
    """
    source_bytes = source.read_bytes()
    joints = {j.get("name"): j for j in ET.fromstring(source_bytes).findall("joint")}
    parameters = _finite_vector(p, model.parameter_count, "calibration parameters")
    metadata = {
        "schema_version": 1,
        "source_urdf_sha256": hashlib.sha256(source_bytes).hexdigest(),
        "composition": "nominal_origin_right_multiply_Ei",
        "joints": {},
    }
    for arm in ARMS:
        blocks = parameters[model.ei_slice[arm]].reshape(7, 6)
        for name, block in zip(model.chains[arm].joint_names, blocks):
            joint = joints[name]
            origin = joint.find("origin")
            attrs = {} if origin is None else origin.attrib
            nominal_xyz = np.fromstring(attrs.get("xyz", "0 0 0"), sep=" ")
            nominal_rpy = np.fromstring(attrs.get("rpy", "0 0 0"), sep=" ")
            # Reuse the exact final-URDF xyz/rpy serialization convention.
            fitted_joint = ET.Element("joint")
            _set_origin(fitted_joint, _origin(origin) @ _rigid(block[:3], block[3:]))
            fitted_attrs = fitted_joint.find("origin").attrib
            fitted_xyz = np.fromstring(fitted_attrs["xyz"], sep=" ")
            fitted_rpy = np.fromstring(fitted_attrs["rpy"], sep=" ")
            # Keep the joint's own origin, not the fixed transforms folded into ArmChain.
            metadata["joints"][name] = {
                "parent_link": joint.find("parent").get("link"),
                "child_link": joint.find("child").get("link"),
                "nominal_origin": {
                    "xyz_m": nominal_xyz.tolist(),
                    "rpy_rad": nominal_rpy.tolist(),
                },
                "origin_offset": {
                    "xyz_m": (fitted_xyz - nominal_xyz).tolist(),
                    "rpy_rad": (fitted_rpy - nominal_rpy).tolist(),
                },
                "Ei": {
                    "translation_mm": block[:3].tolist(),
                    "rotvec_deg": block[3:].tolist(),
                },
            }
    path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                    encoding="utf-8")


def _kinematics_with_frozen_frames(kin, metadata):
    frozen = copy.deepcopy(kin)
    # CTB/ETK positions remain authoritative in the current YAML.  The URDF
    # comment or sidecar carries the candidate directions; validation can use
    # them as frozen values or as the starting point for the frame-only fit.
    frozen["ctb"]["initial_rotvec_deg"] = metadata["ctb"]["rotvec_deg"]
    for arm in ARMS:
        frozen["etk"][arm]["initial_rotvec_deg"] = metadata["etk"][arm]["rotvec_deg"]
    return frozen


def _set_origin(element, transform):
    origin = element.find("origin")
    if origin is None:
        origin = ET.Element("origin")
        element.insert(0, origin)
    origin.set("xyz", " ".join(f"{float(v):.17g}" for v in (transform[:3, 3] / 1000.0)))
    origin.set("rpy", " ".join(f"{float(v):.17g}" for v in Rotation.from_matrix(transform[:3, :3]).as_euler("xyz")))


def _export_urdf_from_json(source: Path, destination: Path, offsets_path: Path,
                           frames_path: Path, model: FixedPositionModel, p):
    """Build joint origins from serialized offsets; use p only for FK verification."""
    source, destination = source.resolve(), destination.resolve()
    if source == destination:
        raise ValueError("candidate URDF must differ from source URDF")
    if destination.exists():
        raise FileExistsError(f"refusing to overwrite existing candidate: {destination}")
    offsets = json.loads(offsets_path.read_text(encoding="utf-8"))
    if offsets.get("schema_version") != 1:
        raise ValueError(f"unsupported joint offsets version in {offsets_path}")
    if offsets.get("source_urdf_sha256") != hashlib.sha256(source.read_bytes()).hexdigest():
        raise ValueError("joint offsets source URDF hash does not match source URDF")
    records = offsets.get("joints", {})
    expected_names = {name for arm in ARMS for name in model.chains[arm].joint_names}
    if set(records) != expected_names:
        raise ValueError("joint offsets must contain exactly the fourteen fitted arm joints")
    tree = _xml_tree(source)
    root = tree.getroot()
    _write_frame_metadata(root, _read_frame_metadata(frames_path))
    joints = {j.get("name"): j for j in root.findall("joint")}
    for name, record in records.items():
        joint = joints[name]
        for label in ("parent", "child"):
            if record.get(label + "_link") != joint.find(label).get("link"):
                raise ValueError(f"joint offsets {label} link mismatch for {name}")
        origin = joint.find("origin")
        if origin is None:
            origin = ET.Element("origin")
            joint.insert(0, origin)
        for attribute, key in (("xyz", "xyz_m"), ("rpy", "rpy_rad")):
            nominal = _finite_vector(record["nominal_origin"][key], 3, f"{name} nominal {key}")
            delta = _finite_vector(record["origin_offset"][key], 3, f"{name} offset {key}")
            original = np.fromstring(origin.get(attribute, "0 0 0"), sep=" ")
            if not np.array_equal(nominal, original):
                raise ValueError(f"joint offsets nominal {key} mismatch for {name}")
            # These are component differences in the exported Euler branch, not Ei.
            final = _finite_vector(nominal + delta, 3, f"{name} final {key}")
            origin.set(attribute, " ".join(f"{float(v):.17g}" for v in final))
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".mocap_kin_", dir=str(destination.parent)) as temp:
        candidate = Path(temp) / "candidate.urdf"
        tree.write(candidate, encoding="utf-8", xml_declaration=True)
        # Reparse the candidate and check the exact corrected FK chain.
        for arm in ARMS:
            checked = load_chain(candidate, arm, model.kin["root_link"], model.chains[arm].tip_link)
            q0 = np.zeros((1, 7))
            expected = model.chains[arm].fk(q0, model.unpack(p, arm)[2])
            actual = checked.fk(q0)
            if np.max(np.abs(expected - actual)) > 1e-7:
                raise RuntimeError(f"URDF FK export check failed for {arm}")
        destination.write_bytes(candidate.read_bytes())


def _resolve_config_path(path: Path, value: str) -> Path:
    p = Path(value).expanduser()
    return p.resolve() if p.is_absolute() else (path.parent / p).resolve()


def _confirm_apply_urdf(source: Path, candidate: Path, target: Path):
    print(f"拟合及导出已通过，候选 URDF: {candidate}")
    all_models = target.name == "biped_s63.urdf"
    installer = write_urdf.install_urdf_directory if all_models else write_urdf.install_urdf
    scope = "S63 目录内全部模型" if all_models else "主 URDF"
    try:
        installer(target, candidate, apply=False, nominal_source=source)
    except (ValueError, OSError, RuntimeError) as exc:
        raise SystemExit(f"拟合结果已保留，但替换前检查失败: {exc}") from None
    print(f"输入 yes 将备份并替换{scope}；运行中的节点不会自动重载。")
    while True:
        try:
            answer = input(f"是否替换{scope}？[yes/no，默认 no]: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            answer = "no"
        if answer in ("", "no"):
            print("未替换运行模型，拟合结果保留在输出目录。")
            return
        if answer == "yes":
            try:
                installer(target, candidate, apply=True, nominal_source=source)
            except (ValueError, OSError, RuntimeError) as exc:
                raise SystemExit(f"拟合结果已保留，但运行模型替换失败: {exc}") from None
            return
        print("请输入 yes 或 no。")


def _config_for_capture(cfg: Mapping, meta: Mapping):
    """Route S62/S63 to the S63 main URDF, retaining the actual robot version."""
    effective = copy.deepcopy(cfg)
    robot = effective["robot"]
    kin = effective["kinematics"]
    configured_version = str(robot.get("robot_version", "56"))
    requested_version = os.environ.get("ROBOT_VERSION", "").strip()
    capture_version = str(meta.get("robot_version") or "")
    if requested_version and capture_version and requested_version != capture_version:
        raise ValueError(
            f"ROBOT_VERSION={requested_version} conflicts with capture.meta.robot_version={capture_version}"
        )
    version = requested_version or capture_version or configured_version
    if version not in ("53", "56", "62", "63"):
        raise ValueError(f"unsupported robot_version={version!r}; expected 53/56/62/63")
    wheel_versions = ("62", "63")
    if (version in wheel_versions) != (configured_version in wheel_versions):
        config_name = "calib_s63.yaml" if version in wheel_versions else "calib_s56.yaml"
        raise ValueError(f"robot_version={version} requires its CTB/ETK config; use --config config/{config_name}")
    if meta.get("fk_root") not in (None, "waist_yaw_link"):
        raise ValueError(f"capture fk_root={meta['fk_root']!r}; main URDF fitting requires waist_yaw_link")
    # S62 uses the S63 nominal geometry; robot/capture versions stay unchanged.
    # --urdf / --validate-urdf remain explicit model overrides in main().
    model_version = "63" if version == "62" else version
    assets = Path(__file__).resolve().parents[2] / "kuavo_assets" / "models"
    kin["source_urdf"] = str(assets / f"biped_s{model_version}" / "urdf" / f"biped_s{model_version}.urdf")
    robot["robot_version"] = version
    robot["robot_layout"] = meta.get("robot_layout", robot["robot_layout"])
    robot["joint_indices"] = meta.get("joint_q_indices", robot["joint_indices"])
    kin["root_link"] = "waist_yaw_link"
    return effective


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--config", type=Path, default=Path(__file__).parent / "config/calib_s56.yaml")
    urdf_args = parser.add_mutually_exclusive_group()
    urdf_args.add_argument("--urdf", type=Path, default=None,
                          help="显式指定拟合模型；默认 S53/S56 用同版本主 URDF，S62/S63 均用 S63 主 URDF")
    urdf_args.add_argument("--validate-urdf", type=Path, default=None,
                          help="使用该 URDF 只计算误差；默认冻结保存的 CTB/ETK 角度，位置仍按 YAML 固定")
    parser.add_argument("--validation-max-mm", type=float, default=None,
                        help="显式覆盖 YAML 验收阈值；不提供且 YAML 为 null 时不生成最终 JSON/URDF")
    frame_angle_args = parser.add_mutually_exclusive_group()
    frame_angle_args.add_argument(
        "--freeze-frame-angles", dest="freeze_frame_angles", action="store_true",
        help="验证时冻结候选 URDF/元数据中的 CTB、ETK 角度（默认）",
    )
    frame_angle_args.add_argument(
        "--no-freeze-frame-angles", "--fit-frame-angles",
        dest="freeze_frame_angles", action="store_false",
        help="验证时重新拟合 CTB、ETK 角度，同时固定候选 URDF 的 Ei",
    )
    parser.set_defaults(freeze_frame_angles=True)
    parser.add_argument(
        "--frame-transforms", type=Path, default=None,
        help="验证时读取 CTB/ETK 旋转矩阵 JSON；默认读取 validate-urdf 的 URDF 元数据",
    )
    parser.add_argument("--output-urdf", action="store_true",
                        help="生成 urdf_final，成功后询问 yes/no 是否备份替换运行模型；S63 同步整个 urdf 目录")
    parser.add_argument("--force-output", action="store_true",
                        help="与 --output-urdf 一起允许覆盖已有 urdf_final；不影响 JSON 覆盖或验收")
    args = parser.parse_args()
    if args.validate_urdf is not None and (args.output_urdf or args.force_output):
        parser.error("--validate-urdf 只验证，不能同时使用 --output-urdf/--force-output")
    if args.validate_urdf is None and (args.frame_transforms is not None or not args.freeze_frame_angles):
        parser.error("--frame-transforms/--no-freeze-frame-angles 只能与 --validate-urdf 一起使用")
    config_path = args.config.resolve()
    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    capture_path = args.capture.expanduser().resolve()
    capture_meta = json.loads(capture_path.read_text(encoding="utf-8")).get("meta", {})
    cfg = _config_for_capture(cfg, capture_meta)
    kin = cfg["kinematics"]
    runtime_target = _resolve_config_path(config_path, kin["source_urdf"])
    if args.validate_urdf is not None:
        source = args.validate_urdf.expanduser().resolve()
        if not source.is_file():
            raise SystemExit(f"validation URDF not found: {source}")
        frame_source = (args.frame_transforms.expanduser().resolve()
                        if args.frame_transforms is not None else source)
        if not frame_source.is_file():
            raise SystemExit(f"frame transforms not found: {frame_source}")
        # In both modes the candidate frame angles are the starting point.
        # The switch only controls whether those nine angles remain fixed.
        kin = _kinematics_with_frozen_frames(kin, _read_frame_metadata(frame_source))
        cfg["kinematics"] = kin
    else:
        try:
            source = (write_urdf.ensure_nominal_source(args.urdf.expanduser().resolve())
                      if args.urdf is not None else write_urdf.resolve_fit_source(runtime_target))
        except (ValueError, OSError) as exc:
            raise SystemExit(f"无法读取原始拟合基准: {exc}") from None
    if not source.is_file():
        raise SystemExit(f"source URDF not found: {source}")
    print(f"capture: {capture_path}")
    print(f"robot_version={cfg['robot']['robot_version']}, root={kin['root_link']}, "
          f"joint_q_indices={cfg['robot']['joint_indices']}")
    print(f"capture display URDF: {capture_meta.get('urdf')}")
    print(f"model URDF: {source}")
    chains = {
        "left": load_chain(source, "left", kin["root_link"], kin["left_tip_link"]),
        "right": load_chain(source, "right", kin["root_link"], kin["right_tip_link"]),
    }
    model = FixedPositionModel(kin, chains)
    data = load_dataset(capture_path, cfg, validation_only=args.validate_urdf is not None)
    if args.validate_urdf is not None:
        print(f"FIXED URDF VALIDATION: {source}")
        if args.freeze_frame_angles:
            validation_p = np.zeros(model.parameter_count)
            print_frame_parameters("FROZEN frames", model, validation_p)
        else:
            validation_p, frame_result = fit_frame_angles(model, data, subset="all")
            print(f"FRAME ANGLE FIT: success={frame_result.success}, "
                  f"message={frame_result.message}, nfev={frame_result.nfev}")
            print_frame_parameters("FRAME ANGLE FIT", model, validation_p)
        rows = error_rows(model, validation_p, data, subset="all")
        print_metrics(f"FIXED VALIDATION({data['validation_count']})", rows)
        threshold = args.validation_max_mm
        if threshold is None:
            threshold = kin.get("validation", {}).get("max_position_error_mm")
        if threshold is not None:
            threshold = float(threshold)
            if not math.isfinite(threshold) or threshold <= 0:
                raise ValueError("validation threshold must be a finite positive number")
            observed_max = max(row["position_mm"] for row in rows)
            passed = observed_max < threshold
            print(f"FIXED VALIDATION gate: max_position_error={observed_max:.6f} mm "
                  f"< threshold={threshold:.6f} mm: {'PASS' if passed else 'FAIL'}")
            return 0 if passed else 3
        return 0

    p25, result25 = fit(model, data, subset="fit")
    print(f"25-point fit: success={result25.success}, message={result25.message}, nfev={result25.nfev}")
    if not result25.success:
        raise SystemExit("25-point fit did not converge; final JSON/URDF were not generated or replaced")
    print_frame_parameters("FIT(25) frames", model, p25)
    print_metrics("FIT(25)", error_rows(model, p25, data, subset="fit"))
    validation_rows = error_rows(model, p25, data, subset="validation")
    print_metrics("VALIDATION(5)", validation_rows)
    threshold = args.validation_max_mm
    if threshold is None:
        threshold = kin.get("validation", {}).get("max_position_error_mm")
    if threshold is None:
        print("未提供 validation.max_position_error_mm；已计算 5 点验证误差，未生成最终 JSON/URDF。")
        threshold_pass = False
    else:
        threshold = float(threshold)
        if not math.isfinite(threshold) or threshold <= 0:
            raise ValueError("validation threshold must be a finite positive number")
        observed_max = max(row["position_mm"] for row in validation_rows)
        threshold_pass = observed_max < threshold
        print(f"VALIDATION gate: max_position_error={observed_max:.6f} mm < threshold={threshold:.6f} mm: "
              f"{'PASS' if threshold_pass else 'FAIL'}")
    output_dir = _resolve_config_path(config_path, kin["output_dir"])
    urdf_final = output_dir / kin["urdf_final_name"]
    if args.output_urdf and urdf_final.resolve() in {
            source.resolve(), runtime_target.resolve(), write_urdf.nominal_backup_path(runtime_target)}:
        raise SystemExit("URDF 输出路径不能是主 URDF 或原始拟合基准；请先导出到 output 目录，再输入 yes 替换。")
    if (args.output_urdf and runtime_target.name == "biped_s63.urdf" and
            runtime_target.parent in urdf_final.resolve().parents):
        raise SystemExit("候选 URDF 必须导出到 S63 运行模型目录之外；目录内的模型只在输入 yes 后替换。")
    if args.output_urdf and urdf_final.exists() and not args.force_output:
        raise SystemExit(f"urdf_final 已存在，拒绝覆盖；需要覆盖时显式追加 --force-output: {urdf_final}")

    if not threshold_pass:
        if threshold is not None:
            raise SystemExit("5-point validation did not pass; final JSON/URDF were not generated")
        return 2
    p30, result30 = fit(model, data, subset="all", initial=p25)
    print(f"30-point fit: success={result30.success}, message={result30.message}, nfev={result30.nfev}")
    if not result30.success:
        raise SystemExit("30-point fit did not converge; final JSON/URDF were not generated or replaced")
    try:
        _check_ei_output_limits(model, p30)
    except ValueError as exc:
        raise SystemExit(str(exc)) from None
    print_frame_parameters("ALL(30) frames", model, p30)
    print_metrics("ALL(30)", error_rows(model, p30, data, subset="all"))
    output_dir.mkdir(parents=True, exist_ok=True)
    # Stage all requested outputs before replacing existing files.  The optional
    # URDF reads the serialized JSON files and must pass the FK export check.
    with tempfile.TemporaryDirectory(prefix=".mocap_output_", dir=str(output_dir)) as temp:
        temporary_frames = Path(temp) / "frame_transforms.json"
        _write_frame_metadata_file(temporary_frames, _frame_metadata(model, p30))
        temporary_offsets = Path(temp) / "kinematic_offsets.json"
        _write_joint_offsets_file(temporary_offsets, source, model, p30)
        if args.output_urdf:
            temporary_final = Path(temp) / kin["urdf_final_name"]
            _export_urdf_from_json(source, temporary_final, temporary_offsets,
                                   temporary_frames, model, p30)
            os.replace(temporary_final, urdf_final)
        os.replace(temporary_frames, output_dir / "frame_transforms.json")
        os.replace(temporary_offsets, output_dir / "kinematic_offsets.json")
    if args.output_urdf:
        print(f"urdf_final: {urdf_final}")
    else:
        print("URDF output disabled; add --output-urdf to generate urdf_final.")
    print(f"frame_transforms: {output_dir / 'frame_transforms.json'}")
    print(f"joint_offsets: {output_dir / 'kinematic_offsets.json'}")
    if args.output_urdf:
        _confirm_apply_urdf(source, urdf_final, runtime_target)


if __name__ == "__main__":
    raise SystemExit(main())
