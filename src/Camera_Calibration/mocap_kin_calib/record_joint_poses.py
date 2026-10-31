#!/usr/bin/env python3
"""Own the capture configuration; reuse mocap_joint_calib's recording flow.

The recorder's config loader and model-default hook are replaced only during
this call. Its motion, sampling, fixture processing and JSON writer are reused.
All original CLI arguments, including explicit --config/--urdf, are forwarded.
"""

import argparse
import copy
import os
from pathlib import Path
import sys

import yaml

from robot_variants import ASSETS, default_config_name, is_wheel_version, main_urdf

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))


def main():
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--robot-version", default="auto")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--kuavo-json", type=Path)
    parser.add_argument("--urdf", type=Path)
    parser.add_argument("--robot-layout")
    parser.add_argument("--fk-root")
    args, _ = parser.parse_known_args()

    from mocap_joint_calib import record_joint_poses as recorder

    if "--help" in sys.argv[1:] or "-h" in sys.argv[1:]:
        return recorder.main()
    version = (os.environ.get("ROBOT_VERSION", "") if args.robot_version == "auto"
               else args.robot_version).strip()
    config_name = default_config_name(version)
    config_path = (args.config or HERE / "config" / config_name).expanduser().resolve()
    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    robot, calibration = cfg["robot"], cfg["calibration"]
    robot["robot_version"] = version
    layout = args.robot_layout or ("wheel62" if is_wheel_version(version) else "biped56")
    root = args.fk_root or "waist_yaw_link"
    urdf = (args.urdf or main_urdf(version)).expanduser().resolve()
    kuavo_json = (args.kuavo_json or ASSETS / "config" / f"kuavo_v{version}" / "kuavo.json").expanduser().resolve()
    for path in (urdf, kuavo_json):
        if not path.is_file():
            raise FileNotFoundError(f"ROBOT_VERSION={version}: 文件不存在: {path}")
    # These existing YAML paths use a calibration-package directory as base.
    # Make them absolute before handing off, independent of recorder location.
    for key in ("teach_left", "teach_right", "teach_head"):
        if calibration.get(key):
            calibration[key] = str((HERE / Path(calibration[key]).expanduser()).resolve())

    def apply_model(robot, calibration, robot_version):
        robot["robot_layout"] = layout
        calibration.update(urdf=str(urdf), fk_root=root)
        return {"robot_layout": layout, "urdf": str(urdf), "fk_root": root}

    apply_model(robot, calibration, version)
    overrides = {
        "_load_config": lambda path: copy.deepcopy(cfg),
        "_apply_robot_calibration_defaults": apply_model,
    }
    originals = {name: getattr(recorder, name) for name in overrides}
    previous_argv = sys.argv
    try:
        for name, value in overrides.items():
            setattr(recorder, name, value)
        sys.argv = [*sys.argv, "--robot-version", version, "--config", str(config_path),
                    "--kuavo-json", str(kuavo_json), "--urdf", str(urdf),
                    "--robot-layout", layout, "--fk-root", root]
        return recorder.main()
    finally:
        for name, value in originals.items():
            setattr(recorder, name, value)
        sys.argv = previous_argv


if __name__ == "__main__":
    raise SystemExit(main())
