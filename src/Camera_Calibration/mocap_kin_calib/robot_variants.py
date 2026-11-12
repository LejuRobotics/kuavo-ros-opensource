"""Shared calibration routing; keep full versions in model/config paths."""

from pathlib import Path
import re
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import robot_versions


ASSETS = Path(__file__).resolve().parents[2] / "kuavo_assets"


def is_wheel_version(version):
    value = str(version)
    # RobotVersion encodes patch * 100000 + major * 10 + minor.
    # The version registry also maps special IDs 210062/310062/410062/410063 to wheel variants.
    return bool(re.fullmatch(r"[1-9][0-9]*", value)) and (
        int(value) % 100000 in (62, 63) or value in ("210062", "310062", "410062", "410063")
    )


def main_urdf(version):
    return ASSETS / "models" / f"biped_s{version}" / "urdf" / f"biped_s{version}.urdf"


def is_wheel_urdf(path):
    match = re.fullmatch(r"biped_s([0-9]+)\.urdf", Path(path).name)
    return match is not None and is_wheel_version(match.group(1))


def default_config_name(version):
    try:
        layout = robot_versions.require(version, "运动学标定").layout
    except RuntimeError as exc:
        raise ValueError(str(exc)) from exc
    if layout == "wheel62":
        return "calib_s63.yaml"
    if layout == "biped56":
        return "calib_s56.yaml"
    raise ValueError(f"运动学标定不支持 ROBOT_VERSION={version!r} 的布局 {layout}")


if __name__ == "__main__":
    import sys

    try:
        print(default_config_name(sys.argv[1] if len(sys.argv) > 1 else ""))
    except ValueError as exc:
        raise SystemExit(str(exc))
