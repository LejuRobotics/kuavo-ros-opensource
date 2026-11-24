"""Configuration parsing, input profiles and calibration files."""

import configparser
import importlib.util
import json
from pathlib import Path


def load_config(path):
    path = Path(path).expanduser().resolve()
    parser = configparser.ConfigParser()
    parser.optionxform = str
    with path.open("r", encoding="utf-8-sig") as stream:
        parser.read_file(stream)
    config = {}
    for section in parser.sections():
        target = config
        if section != "root":
            for part in section.split("."):
                target = target.setdefault(part, {})
        for key, value in parser.items(section):
            target[key] = json.loads(value)
    return config


def make_solver_config(cfg_dict):
    from .types import SolverConfig

    return SolverConfig(**(cfg_dict.get("solver", {}) or {}))


def select_input_profile(in_tf):
    out = dict(in_tf)
    profiles = out.pop("input_transform_profiles", None)
    profile_name = out.get("input_transform_profile")
    if profiles is None:
        return out
    if not isinstance(profiles, dict):
        raise ValueError("input_transform_profiles must be a dict")
    if profile_name is None:
        profile_name = next(iter(profiles), None)
    if profile_name not in profiles:
        raise ValueError("unknown input_transform_profile=" + repr(profile_name))
    profile = profiles[profile_name]
    if not isinstance(profile, dict):
        raise ValueError("input_transform profile " + repr(profile_name) + " must be a dict")
    out.update(profile)
    out["input_transform_profile"] = profile_name
    return out


def load_calibration(in_tf, cfg_dir):
    name = str(in_tf.get("calib_file", "")).strip()
    if not name:
        return in_tf
    path = Path(name)
    if path.suffix == ".py":
        names = [path]
    else:
        names = [
            path.with_suffix(".py"),
            Path("calib_" + name + "_v10.py"),
            Path("calib_" + name + ".py"),
        ]
    candidates = [p if p.is_absolute() else Path(cfg_dir) / p for p in names]
    calib_path = next((p for p in candidates if p.is_file()), None)
    if calib_path is None:
        print("[calib] warning: cannot find calib_file=" + repr(name))
        return in_tf
    spec = importlib.util.spec_from_file_location("sg100_calib", calib_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    calib = getattr(module, "CALIB", None)
    if not isinstance(calib, dict):
        print("[calib] warning: " + str(calib_path) + " has no CALIB dict")
        return in_tf
    out = dict(in_tf)
    for key in (
        "scale_before_offset",
        "offset_xyz",
        "scale",
        "per_finger_offset_xyz",
        "per_finger_scale",
    ):
        if key in calib:
            out[key] = calib[key]
    print(
        "[calib] loaded "
        + str(getattr(module, "CALIB_NAME", calib_path))
        + " <- "
        + str(calib_path)
    )
    return out


def load_config_with_path(path):
    config_path = Path(path).expanduser().resolve()
    return config_path, load_config(config_path)
