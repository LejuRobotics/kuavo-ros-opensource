#!/usr/bin/env python3
"""Fit and validate per-user, per-hand gesture distance thresholds.

The calibration input contains the original 35-value fingertip frames.  It
never consumes the continuously republished debug-distance topic, so a stale
debug value cannot be mistaken for a live calibration sample.
"""
from __future__ import annotations

import copy
import datetime
import math
import os
from pathlib import Path
import shutil
import tempfile
from typing import Dict, Iterable, Mapping, Sequence

import numpy as np
import yaml


def default_profile_path():
    """Keep data with this package, independent of cwd and the launch user."""
    package = Path(__file__).resolve().parents[1]
    if not (package / "package.xml").is_file():
        # Installed helpers live under lib/<package>, while package data lives
        # under share/<package>. Catkin source/devel uses the source path above.
        import rospkg
        package = Path(rospkg.RosPack().get_path("sg100_glove_bridge")).resolve()
    return str(package / "config/glove_gesture_calibration/active.yaml")


DEFAULT_PROFILE_PATH = default_profile_path()
STAGES = ("open", "relaxed", "flat", "index", "middle", "little", "fist",
          "left_fist", "right_fist", "xb", "xy")
SIDES = ("left", "right")
FEATURES = ("thumb_index", "thumb_middle", "thumb_little", "fist")
SCHEMA_VERSION = 1
MIN_DURATION_SEC = 3.0
MIN_RATE_HZ = 20.0
MAX_GAP_SEC = 0.3
MAX_DURATION_SEC = 30.0
MAX_SAMPLES_PER_SIDE = 6000
MIN_SEPARATION_M = 0.004
POLL_PERIOD_SEC = 0.02


def _module_map(value):
    if value is None:
        value = ["A", "B", "C", "D", "E"]
    if isinstance(value, str):
        value = [item.strip() for item in value.strip("[]").split(",") if item.strip()]
    if not isinstance(value, (list, tuple)) or len(value) != 5:
        raise ValueError("Windows module maps need exactly five entries")
    result = [str(item).strip().upper() for item in value]
    if any(item not in tuple("ABCDEF") for item in result) or len(set(result)) != 5:
        raise ValueError("Windows module maps must contain five distinct A-F modules")
    return result


def input_contract(backend, left_module_map=None, right_module_map=None,
                   left_id=12, right_id=2):
    """Return the exact input identity to which a profile is bound."""
    backend = str(backend).strip().lower()
    if backend == "windows_udp":
        return {
            "backend": backend,
            "left_module_map": _module_map(left_module_map),
            "right_module_map": _module_map(right_module_map),
        }
    if backend == "linux_sdk":
        try:
            left_id, right_id = int(left_id), int(right_id)
        except (TypeError, ValueError) as exc:
            raise ValueError("Linux SDK board IDs must be integers") from exc
        if not (0 <= left_id <= 255 and 0 <= right_id <= 255):
            raise ValueError("Linux SDK board IDs must be in [0, 255]")
        if left_id == right_id:
            raise ValueError("Left and right Linux SDK board IDs must differ")
        return {"backend": backend, "left_id": left_id, "right_id": right_id}
    raise ValueError("Unsupported glove calibration backend: %s" % backend)


def _finite_number(value, name):
    if isinstance(value, bool):
        raise ValueError(name + " must be a finite number")
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(name + " must be a finite number") from exc
    if not math.isfinite(value):
        raise ValueError(name + " must be a finite number")
    return value


def _validate_thresholds(thresholds):
    if not isinstance(thresholds, dict) or set(thresholds) != set(SIDES):
        raise ValueError("thresholds must contain exactly left and right")
    clean = {}
    for side in SIDES:
        if not isinstance(thresholds[side], dict) or set(thresholds[side]) != set(FEATURES):
            raise ValueError(side + " thresholds are incomplete")
        clean[side] = {}
        for feature in FEATURES:
            values = thresholds[side][feature]
            if not isinstance(values, dict) or set(values) != {"press", "release"}:
                raise ValueError("%s/%s needs press and release" % (side, feature))
            press = _finite_number(values["press"], side + "/" + feature + "/press")
            release = _finite_number(values["release"], side + "/" + feature + "/release")
            if not 0 < press < release <= 0.25:
                raise ValueError("%s/%s requires 0 < press < release <= 0.25 m" %
                                 (side, feature))
            clean[side][feature] = {"press": press, "release": release}
    return clean


def validate_profile(document, expected_input=None):
    """Validate a profile deeply enough for safe runtime activation."""
    if not isinstance(document, dict):
        raise ValueError("Gesture profile must be a mapping")
    required = {"schema_version", "profile_name", "created_at", "input",
                "quality", "thresholds"}
    if set(document) != required or document.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("Gesture profile schema mismatch")
    name = document.get("profile_name")
    if not isinstance(name, str) or not name.strip() or len(name) > 128:
        raise ValueError("profile_name must be a non-empty string up to 128 characters")
    created_at = document.get("created_at")
    if not isinstance(created_at, str):
        raise ValueError("created_at must be an ISO timestamp")
    try:
        datetime.datetime.fromisoformat(created_at)
    except ValueError as exc:
        raise ValueError("created_at must be an ISO timestamp") from exc
    source = document.get("input", {})
    if not isinstance(source, dict) or "backend" not in source:
        raise ValueError("Gesture profile input contract is missing")
    if source["backend"] == "windows_udp":
        normalized = input_contract(source["backend"], source.get("left_module_map"),
                                    source.get("right_module_map"))
    elif source["backend"] == "linux_sdk":
        normalized = input_contract(source["backend"], left_id=source.get("left_id"),
                                    right_id=source.get("right_id"))
    else:
        raise ValueError("Gesture profile backend is unsupported")
    if source != normalized:
        raise ValueError("Gesture profile input contract is not canonical")
    if expected_input is not None and normalized != expected_input:
        raise ValueError("Gesture profile was recorded with a different glove input contract")
    thresholds = _validate_thresholds(document.get("thresholds"))
    quality = document.get("quality")
    if not isinstance(quality, dict) or quality.get("status") != "passed":
        raise ValueError("Gesture profile has not passed validation")
    if quality.get("stages") != list(STAGES) or quality.get("rounds_per_stage") != 3:
        raise ValueError("Gesture profile quality record is incomplete")
    if quality.get("release_clears") is not True or quality.get("stale_clears") is not True:
        raise ValueError("Gesture profile release/stale validation did not pass")
    separation = quality.get("separation_m")
    validation = quality.get("heldout_validation")
    summaries = quality.get("feature_summary")
    if (not isinstance(separation, dict) or not isinstance(validation, dict)
            or not isinstance(summaries, dict)):
        raise ValueError("Gesture profile quality metrics are missing")
    for side in SIDES:
        if (set(separation.get(side, {})) != set(FEATURES)
                or set(summaries.get(side, {})) != set(FEATURES)):
            raise ValueError(side + " separation metrics are incomplete")
        for feature in FEATURES:
            gap = _finite_number(separation[side][feature], side + "/" + feature + "/gap")
            if gap < MIN_SEPARATION_M:
                raise ValueError("Gesture profile separation is below the quality gate")
            summary = summaries[side][feature]
            if not isinstance(summary, dict) or set(summary) != {
                    "positive_p99_m", "negative_p01_m", "positive_samples", "negative_samples"}:
                raise ValueError(side + "/" + feature + " feature summary is incomplete")
            positive_high = _finite_number(
                summary["positive_p99_m"], side + "/" + feature + "/positive_p99_m")
            negative_low = _finite_number(
                summary["negative_p01_m"], side + "/" + feature + "/negative_p01_m")
            fitted_gap = negative_low - positive_high
            if fitted_gap < MIN_SEPARATION_M:
                raise ValueError(side + "/" + feature + " summary separation is invalid")
            if not math.isclose(gap, fitted_gap, rel_tol=0.0, abs_tol=1e-9):
                raise ValueError(side + "/" + feature + " separation report is inconsistent")
            expected_press = positive_high + fitted_gap * 0.25
            expected_release = positive_high + fitted_gap * 0.75
            pair = thresholds[side][feature]
            if (not math.isclose(pair["press"], expected_press,
                                 rel_tol=0.0, abs_tol=1e-9)
                    or not math.isclose(pair["release"], expected_release,
                                        rel_tol=0.0, abs_tol=1e-9)):
                raise ValueError(side + "/" + feature + " thresholds do not match quality data")
            for count_name in ("positive_samples", "negative_samples"):
                count = summary[count_name]
                if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
                    raise ValueError(side + "/" + feature + " sample count is invalid")
    expected_masks = {stage: 0 for stage in STAGES}
    expected_masks.update({"index": 5, "middle": 10, "little": 16, "fist": 32,
                           "xb": 9, "xy": 3})
    for stage in STAGES:
        item = validation.get(stage)
        if not isinstance(item, dict) or item.get("passed") is not True:
            raise ValueError("Held-out validation is missing or failed for " + stage)
        if item.get("expected_mask") != expected_masks[stage]:
            raise ValueError("Held-out expected mask is invalid for " + stage)
        if not isinstance(item.get("observed_masks"), list):
            raise ValueError("Held-out mask record is missing for " + stage)
        for value in item["observed_masks"]:
            if not isinstance(value, int) or not 0 <= value <= 63:
                raise ValueError("Held-out mask record is invalid for " + stage)
        allowed = {0, expected_masks[stage]}
        if not set(item["observed_masks"]).issubset(allowed):
            raise ValueError("Held-out mask record contains an unexpected bit for " + stage)
        rate = _finite_number(item.get("detection_rate"), stage + "/detection_rate")
        if not 0 <= rate <= 1:
            raise ValueError("Held-out detection rate is invalid for " + stage)
        if expected_masks[stage] and expected_masks[stage] not in item["observed_masks"]:
            raise ValueError("Held-out expected mask was not observed for " + stage)
        if expected_masks[stage] not in (0, 16, 32) and rate < 0.8:
            raise ValueError("Held-out detection rate is too low for " + stage)
    result = copy.deepcopy(document)
    result["thresholds"] = thresholds
    return result


def load_profile(path=DEFAULT_PROFILE_PATH, expected_input=None, optional=False):
    path = Path(path).expanduser()
    if path.is_symlink():
        raise ValueError("Gesture profile must not be a symbolic link: " + str(path))
    if not path.exists():
        if optional:
            return None
        raise ValueError("Gesture profile does not exist: " + str(path))
    if not path.is_file():
        raise ValueError("Gesture profile must be a regular file: " + str(path))
    try:
        document = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError("Cannot read gesture profile: " + str(path)) from exc
    return validate_profile(document, expected_input)


def _validate_frames(frames, name, minimum_duration=MIN_DURATION_SEC):
    if not isinstance(frames, list) or len(frames) < 2 or len(frames) > MAX_SAMPLES_PER_SIDE:
        raise ValueError(name + " has an invalid sample count")
    result, previous = [], None
    for index, sample in enumerate(frames):
        if not isinstance(sample, dict) or set(sample) != {"t", "data"}:
            raise ValueError(name + " sample schema mismatch")
        stamp = _finite_number(sample["t"], name + "/t")
        data = sample["data"]
        if not isinstance(data, list) or len(data) != 35:
            raise ValueError(name + " needs exactly 35 fingertip values")
        data = [_finite_number(value, name + "/data") for value in data]
        if previous is not None and stamp <= previous:
            raise ValueError(name + " timestamps must be strictly increasing")
        previous = stamp
        result.append({"t": stamp, "data": data})
    times = np.array([sample["t"] for sample in result])
    duration = float(times[-1] - times[0])
    if duration < minimum_duration or duration > MAX_DURATION_SEC:
        raise ValueError("%s duration must be between %.1f and 30 seconds" %
                         (name, minimum_duration))
    if (len(result) - 1) / duration < MIN_RATE_HZ:
        raise ValueError(name + " average rate is below 20 Hz")
    if float(np.max(np.diff(times))) > MAX_GAP_SEC:
        raise ValueError("%s contains an arrival gap over %.1f seconds" %
                         (name, MAX_GAP_SEC))
    return result


def _validate_side_alignment(left, right, name):
    start_gap = abs(left[0]["t"] - right[0]["t"])
    end_gap = abs(left[-1]["t"] - right[-1]["t"])
    if start_gap > MAX_GAP_SEC or end_gap > MAX_GAP_SEC:
        raise ValueError("%s left/right arrival boundary gap exceeds %.1f seconds" %
                         (name, MAX_GAP_SEC))


def _validate_recording_header(recording):
    if not isinstance(recording, dict) or recording.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("Gesture calibration recording schema mismatch")
    if set(recording) != {"schema_version", "profile_name", "input", "segments"}:
        raise ValueError("Gesture calibration recording schema mismatch")
    name = recording.get("profile_name")
    if not isinstance(name, str) or not name.strip() or len(name) > 128:
        raise ValueError("Recording profile_name is invalid")
    source = recording.get("input", {})
    if not isinstance(source, dict):
        raise ValueError("Recording input contract is invalid")
    if source.get("backend") == "windows_udp":
        normalized = input_contract("windows_udp", source.get("left_module_map"),
                                    source.get("right_module_map"))
    elif source.get("backend") == "linux_sdk":
        normalized = input_contract("linux_sdk", left_id=source.get("left_id"),
                                    right_id=source.get("right_id"))
    else:
        raise ValueError("Recording input contract is invalid")
    if source != normalized:
        raise ValueError("Recording input contract is not canonical")
    segments = recording.get("segments")
    if not isinstance(segments, list):
        raise ValueError("Recording segments must be a list")
    return name.strip(), normalized, segments


def _validate_recording_segment(segment):
    if not isinstance(segment, dict) or set(segment) != {
            "stage", "round", "left", "right", "release"}:
        raise ValueError("Recording segment schema mismatch")
    stage, round_number = segment["stage"], segment["round"]
    if stage not in STAGES or isinstance(round_number, bool) or round_number not in (1, 2, 3):
        raise ValueError("Recording stage or round is invalid")
    key = (stage, round_number)
    release = segment["release"]
    if not isinstance(release, dict) or set(release) != set(SIDES):
        raise ValueError("Recording release segment needs left and right")
    validated = {
        "stage": stage,
        "round": round_number,
        "left": _validate_frames(segment["left"], "%s/%s/left" % key),
        "right": _validate_frames(segment["right"], "%s/%s/right" % key),
        "release": {
            side: _validate_frames(release[side], "%s/%s/release/%s" %
                                   (stage, round_number, side), 1.0)
            for side in SIDES
        },
    }
    _validate_side_alignment(validated["left"], validated["right"],
                             "%s/%s/target" % key)
    _validate_side_alignment(validated["release"]["left"],
                             validated["release"]["right"],
                             "%s/%s/release" % key)
    release_end = max(validated["release"][side][-1]["t"] for side in SIDES)
    target_start = min(validated[side][0]["t"] for side in SIDES)
    if not release_end < target_start:
        raise ValueError("Recording release must finish before target samples")
    return key, validated


def validate_partial_recording(recording, expected_input=None, expected_name=None):
    """Validate a resumable CLI-order prefix without accepting holes."""
    name, source, segments = _validate_recording_header(recording)
    cli_order = [(stage, round_number) for round_number in (1, 2, 3)
                 for stage in STAGES]
    if len(segments) > len(cli_order):
        raise ValueError("Partial recording exceeds the three-round calibration sequence")
    if expected_input is not None:
        probe = {"schema_version": SCHEMA_VERSION, "profile_name": name,
                 "input": expected_input, "segments": []}
        _, normalized_expected, _ = _validate_recording_header(probe)
        if source != normalized_expected:
            raise ValueError("Partial recording uses a different glove input contract")
    if expected_name is not None:
        if not isinstance(expected_name, str) or not expected_name.strip():
            raise ValueError("Expected profile_name is invalid")
        if name != expected_name.strip():
            raise ValueError("Partial recording uses a different profile_name")
    validated_segments = []
    for index, segment in enumerate(segments):
        key, validated = _validate_recording_segment(segment)
        if key != cli_order[index]:
            raise ValueError("Partial recording is not a contiguous CLI-order prefix")
        validated_segments.append(validated)
    return {
        "schema_version": SCHEMA_VERSION,
        "profile_name": name,
        "input": source,
        "segments": validated_segments,
    }


def _validate_recording(recording):
    name, normalized, segments = _validate_recording_header(recording)
    if len(segments) != len(STAGES) * 3:
        raise ValueError("Recording needs exactly three rounds for every stage")
    indexed = {}
    for segment in segments:
        key, validated = _validate_recording_segment(segment)
        if key in indexed:
            raise ValueError("Recording contains a duplicate stage round")
        indexed[key] = validated
    expected = {(stage, round_number) for stage in STAGES for round_number in (1, 2, 3)}
    if set(indexed) != expected:
        raise ValueError("Recording is missing a required stage round")
    return name.strip(), normalized, indexed


def _features(frames):
    # Function-local import keeps this pure helper out of the ROS node's import
    # path and guarantees calibration uses the exact production geometry.
    from glove_gesture_node import fist_span, hand_distances, positions
    result = []
    for sample in frames:
        points = positions(sample["data"])
        distances = hand_distances(points)
        result.append({
            "t": sample["t"],
            "thumb_index": distances[0],
            "thumb_middle": distances[1],
            "thumb_little": distances[2],
            "fist": fist_span(points),
        })
    return result


def _positive(stage, side, feature, suppress_fist_pinches=False):
    if feature == "fist":
        return stage == "fist" or stage == ("left_fist" if side == "left" else "right_fist")
    if stage == "fist":
        return None  # Production suppresses pinch bits while both fists are held.
    if (suppress_fist_pinches and stage == side + "_fist"
            and feature in ("thumb_index", "thumb_middle")):
        # The selected runtime policy masks these bits on the clenched side.
        # The open hand and little-finger mode gesture remain negative evidence.
        return None
    if feature == "thumb_index":
        return stage == "index" or (side == "left" and stage in ("xb", "xy"))
    if feature == "thumb_middle":
        return stage == "middle" or (side == "right" and stage == "xb") or (
            side == "left" and stage == "xy")
    if feature == "thumb_little":
        return stage == "little"
    raise ValueError("Unknown feature: " + feature)


def _fit_thresholds(feature_sets, suppress_fist_pinches=False):
    thresholds, separations, summaries = {}, {}, {}
    for side in SIDES:
        thresholds[side], separations[side], summaries[side] = {}, {}, {}
        for feature in FEATURES:
            positive, negative = [], []
            for stage in STAGES:
                for round_number in (1, 2):
                    negative.extend(
                        item[feature]
                        for item in feature_sets[(stage, round_number)]["release"][side])
            for stage in STAGES:
                label = _positive(stage, side, feature, suppress_fist_pinches=suppress_fist_pinches)
                if label is None:
                    continue
                destination = positive if label else negative
                for round_number in (1, 2):
                    destination.extend(item[feature] for item in feature_sets[(stage, round_number)][side])
            if not positive or not negative:
                raise ValueError("Insufficient labelled samples for %s/%s" % (side, feature))
            positive_high = float(np.percentile(positive, 99))
            negative_low = float(np.percentile(negative, 1))
            gap = negative_low - positive_high
            if not math.isfinite(gap) or gap < MIN_SEPARATION_M:
                raise ValueError(
                    "%s/%s positive and negative distances overlap (gap %.6f m)" %
                    (side, feature, gap))
            press = positive_high + gap * 0.25
            release = positive_high + gap * 0.75
            if not 0 < press < release <= 0.25:
                raise ValueError("Fitted threshold is outside the valid range for %s/%s" %
                                 (side, feature))
            thresholds[side][feature] = {"press": press, "release": release}
            separations[side][feature] = gap
            summaries[side][feature] = {
                "positive_p99_m": positive_high,
                "negative_p01_m": negative_low,
                "positive_samples": len(positive),
                "negative_samples": len(negative),
            }
    return thresholds, separations, summaries


def _new_logic(thresholds, suppress_fist_pinches=False):
    from glove_gesture_node import GloveGestureLogic
    generic_press = {name: thresholds["left"][name]["press"]
                     for name in FEATURES[:3]}
    generic_release = {name: thresholds["left"][name]["release"]
                       for name in FEATURES[:3]}
    return GloveGestureLogic(
        generic_press, generic_release, 2.0, 0.5, 0.1,
        thresholds["left"]["fist"]["press"],
        thresholds["left"]["fist"]["release"],
        side_thresholds=thresholds,
        suppress_fist_pinches=suppress_fist_pinches)


def _replay(logic, left, right, duration=None):
    left_features, right_features = _features(left), _features(right)
    origin = min(left_features[0]["t"], right_features[0]["t"])
    end = max(left_features[-1]["t"], right_features[-1]["t"])
    if duration is not None:
        end = min(end, origin + duration)
    left_index = right_index = 0
    masks, stamps = [], []
    steps = int(math.floor((end - origin) / POLL_PERIOD_SEC)) + 1
    for step in range(steps):
        now = origin + step * POLL_PERIOD_SEC
        while left_index < len(left_features) and left_features[left_index]["t"] <= now + 1e-9:
            item = left_features[left_index]
            logic.update_hand("L", [item[name] for name in FEATURES[:3]], item["fist"], now)
            left_index += 1
        while right_index < len(right_features) and right_features[right_index]["t"] <= now + 1e-9:
            item = right_features[right_index]
            logic.update_hand("R", [item[name] for name in FEATURES[:3]], item["fist"], now)
            right_index += 1
        masks.append(int(logic.poll(now)))
        stamps.append(now - origin)
    return stamps, masks, end


def _validate_heldout(indexed, thresholds, suppress_fist_pinches=False):
    expected = {stage: 0 for stage in STAGES}
    expected.update({"index": 5, "middle": 10, "little": 16, "fist": 32,
                     "xb": 9, "xy": 3})
    results = {}
    logic = _new_logic(thresholds, suppress_fist_pinches=suppress_fist_pinches)
    cursor = 0.0
    last_end = 0.0
    for stage in STAGES:
        segment = indexed[(stage, 3)]
        release_origin = min(segment["release"][side][0]["t"] for side in SIDES)
        release_shift = cursor - release_origin
        release_end = max(segment["release"][side][-1]["t"] + release_shift
                          for side in SIDES)
        target_origin = min(segment[side][0]["t"] for side in SIDES)
        # Human preparation time between the release and target recordings is
        # not gesture evidence.  Compress that sample-free interval while
        # preserving both real traces and the same continuously-used logic.
        target_shift = release_end + 2 * POLL_PERIOD_SEC - target_origin
        combined = {}
        for side in SIDES:
            combined[side] = [
                {"t": sample["t"] + release_shift, "data": sample["data"]}
                for sample in segment["release"][side]
            ] + [
                {"t": sample["t"] + target_shift, "data": sample["data"]}
                for sample in segment[side]
            ]
        target_start = min(segment[side][0]["t"] + target_shift for side in SIDES)
        stamps, masks, end = _replay(logic, combined["left"], combined["right"])
        absolute_stamps = [cursor + stamp for stamp in stamps]
        release_masks = [mask for stamp, mask in zip(absolute_stamps, masks)
                         if stamp <= release_end]
        if not release_masks or release_masks[-1] != 0:
            raise ValueError("Held-out %s did not start from a released state" % stage)
        wanted = expected[stage]
        target_masks = [mask for stamp, mask in zip(absolute_stamps, masks)
                        if stamp >= target_start]
        after_warmup = [mask for stamp, mask in zip(absolute_stamps, masks)
                        if stamp >= target_start + 0.2]
        # Debouncing may legitimately leave the expected bit at zero at the
        # beginning of a target segment.  No other bit is acceptable there,
        # including during that warm-up window.
        allowed = {0, wanted}
        unexpected = [mask for mask in target_masks if mask not in allowed]
        if unexpected:
            raise ValueError("Held-out %s emitted unexpected button mask %s" %
                             (stage, sorted(set(unexpected))))
        count = sum(mask == wanted for mask in (
            target_masks if wanted in (16, 32) else after_warmup))
        if wanted == 16:
            passed = 1 <= count <= int(math.ceil(0.6 / POLL_PERIOD_SEC))
        elif wanted == 32:
            passed = count == 1
        elif wanted:
            passed = bool(after_warmup) and count / len(after_warmup) >= 0.8
        else:
            passed = all(mask == 0 for mask in after_warmup)
        if not passed:
            raise ValueError("Held-out %s did not reliably produce mask %d" % (stage, wanted))

        results[stage] = {
            "expected_mask": wanted,
            "observed_masks": sorted(set(target_masks)),
            "detection_rate": count / len(after_warmup) if after_warmup else 0.0,
            "passed": True,
        }
        last_end = end
        cursor = end + 0.04

    # End on a genuine held-out open segment, then stop input long enough to
    # exercise the production stale-data clearing path.
    final = indexed[("open", 3)]
    raw_origin = min(final[side][0]["t"] for side in SIDES)
    shift = cursor - raw_origin
    opened = {
        side: [{"t": sample["t"] + shift, "data": sample["data"]}
               for sample in final[side]]
        for side in SIDES
    }
    _, masks, last_end = _replay(logic, opened["left"], opened["right"], 0.5)
    if not masks or any(mask != 0 for mask in masks[-5:]):
        raise ValueError("Held-out sequence did not clear on the final open segment")
    if logic.poll(last_end + 0.6) != 0:
        raise ValueError("Held-out sequence did not clear after input timeout")
    return results


def fit_profile(recording, suppress_fist_pinches=False):
    """Fit thresholds and admit them only after production-logic replay."""
    profile_name, source, indexed = _validate_recording(recording)
    feature_sets = {
        key: {**{side: _features(segment[side]) for side in SIDES},
              "release": {side: _features(segment["release"][side]) for side in SIDES}}
        for key, segment in indexed.items()
    }
    thresholds, separation, summaries = _fit_thresholds(
        feature_sets, suppress_fist_pinches=suppress_fist_pinches)
    validation = _validate_heldout(
        indexed, thresholds, suppress_fist_pinches=suppress_fist_pinches)
    profile = {
        "schema_version": SCHEMA_VERSION,
        "profile_name": profile_name,
        "created_at": datetime.datetime.now().astimezone().isoformat(),
        "input": source,
        "quality": {
            "status": "passed",
            "stages": list(STAGES),
            "rounds_per_stage": 3,
            "training_rounds": [1, 2],
            "heldout_round": 3,
            "minimum_separation_m": MIN_SEPARATION_M,
            "separation_m": separation,
            "feature_summary": summaries,
            "heldout_validation": validation,
            "release_clears": True,
            "stale_clears": True,
        },
        "thresholds": thresholds,
    }
    return validate_profile(profile, source)


def atomic_save_profile(path, profile):
    """Validate first, then atomically install with a timestamped backup."""
    document = validate_profile(profile)
    path = Path(path).expanduser()
    if path.is_symlink():
        raise ValueError("Refusing to replace a symbolic-link gesture profile")
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and not path.is_file():
        raise ValueError("Gesture profile target is not a regular file")
    backup = None
    if path.exists():
        backup = path.with_name(
            path.name + ".before-" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
        shutil.copy2(path, backup)
    descriptor, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "w") as handle:
            yaml.safe_dump(document, handle, sort_keys=False, allow_unicode=True)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, (path.stat().st_mode & 0o777) if path.exists() else 0o644)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return backup
