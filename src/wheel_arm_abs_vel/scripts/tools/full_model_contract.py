"""Private 62/63/400062 full-URDF startup contract; no ROS dependency."""
from pathlib import Path


def full_model_relative_path(version):
    version = int(version)
    if version not in (62, 63, 400062):
        raise ValueError("Absolute velocity IK supports only 62, 63 and 400062")
    model = f"biped_s{version}"
    return f"{model}/urdf/{model}.urdf"


def require_full_model(version, assets, config):
    expected = Path(assets) / "models" / full_model_relative_path(version)
    relative = config.get("velocity_ik_urdf")
    if not isinstance(relative, str) or not relative or not assets:
        raise ValueError(f"Absolute velocity IK version {version} requires an explicit full-model contract")
    actual = Path(assets) / "models" / relative
    if actual.resolve(strict=True) != expected.resolve(strict=True):
        raise ValueError(f"Absolute velocity IK version {version} requires its own complete URDF: {expected}")
    return actual
