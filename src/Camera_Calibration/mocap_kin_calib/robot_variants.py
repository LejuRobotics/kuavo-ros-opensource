"""Shared calibration routing; keep full versions in model/config paths."""

from pathlib import Path
import re


ASSETS = Path(__file__).resolve().parents[2] / "kuavo_assets"


def is_wheel_version(version):
    value = str(version)
    # RobotVersion encodes patch * 100000 + major * 10 + minor.
    # The version registry also maps special IDs 210062/310062 to wheel 62.7/62.8.
    return bool(re.fullmatch(r"[1-9][0-9]*", value)) and (
        int(value) % 100000 in (62, 63) or value in ("210062", "310062")
    )


def main_urdf(version):
    return ASSETS / "models" / f"biped_s{version}" / "urdf" / f"biped_s{version}.urdf"


def is_wheel_urdf(path):
    match = re.fullmatch(r"biped_s([0-9]+)\.urdf", Path(path).name)
    return match is not None and is_wheel_version(match.group(1))


def default_config_name(version):
    if is_wheel_version(version):
        return "calib_s63.yaml"
    if str(version) in ("53", "56"):
        return "calib_s56.yaml"
    raise ValueError(f"unsupported ROBOT_VERSION={version!r}; expected 53/56 or 62/63 variants")


if __name__ == "__main__":
    import sys

    try:
        print(default_config_name(sys.argv[1] if len(sys.argv) > 1 else ""))
    except ValueError as exc:
        raise SystemExit(str(exc))
