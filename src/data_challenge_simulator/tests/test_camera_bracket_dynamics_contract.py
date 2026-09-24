"""Keep the S400062 MuJoCo camera brackets consistent with the URDF."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE.parent
ASSET_MODEL = WORKSPACE_SRC / "kuavo_assets/models/biped_s400062"
URDF = ASSET_MODEL / "urdf/biped_s400062.urdf"
MJCF_MODELS = (
    ASSET_MODEL / "xml/biped_s400062.xml",
    ASSET_MODEL / "xml/biped_s400062_icra.xml",
    PACKAGE / "models/biped_s400062/xml/biped_s400062_icra.xml",
)


def _numbers(value):
    return tuple(float(item) for item in value.split())


def test_camera_bracket_inertials_match_urdf():
    urdf_root = ET.parse(URDF).getroot()

    for side in ("l", "r"):
        name = f"camera_{side}_base"
        urdf_inertial = urdf_root.find(f"./link[@name='{name}']/inertial")
        assert urdf_inertial is not None

        expected_mass = float(urdf_inertial.find("mass").attrib["value"])
        expected_pos = _numbers(urdf_inertial.find("origin").attrib["xyz"])
        inertia = urdf_inertial.find("inertia").attrib
        expected_fullinertia = tuple(
            float(inertia[key])
            for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
        )

        for model_path in MJCF_MODELS:
            root = ET.parse(model_path).getroot()
            body = root.find(f".//body[@name='{name}']")
            assert body is not None, f"{name} missing from {model_path}"
            mjcf_inertial = body.find("inertial")
            assert mjcf_inertial is not None, (
                f"{name} is visual-only in {model_path}"
            )
            assert float(mjcf_inertial.attrib["mass"]) == expected_mass
            assert _numbers(mjcf_inertial.attrib["pos"]) == expected_pos
            assert _numbers(mjcf_inertial.attrib["fullinertia"]) == expected_fullinertia


def test_arm_cameras_use_the_bracket_optical_frames():
    expected = {
        "l": {
            "pos": (0.0146, 0.0, 0.021),
            "xyaxes": (0.0, -1.0, 0.0, 0.0, 0.0, 1.0),
        },
        "r": {
            "pos": (0.0146, 0.0, -0.021),
            "xyaxes": (0.0, 1.0, 0.0, 0.0, 0.0, -1.0),
        },
    }

    for model_path in MJCF_MODELS[1:]:
        root = ET.parse(model_path).getroot()
        parent_by_child = {
            child: parent
            for parent in root.iter()
            for child in parent
        }

        for side, mount in expected.items():
            camera = root.find(f".//camera[@name='cam_{side}']")
            assert camera is not None, model_path
            assert parent_by_child[camera].attrib.get("name") == \
                f"camera_{side}_base", model_path
            assert _numbers(camera.attrib["pos"]) == mount["pos"], model_path
            assert _numbers(camera.attrib["xyaxes"]) == \
                mount["xyaxes"], model_path
            assert float(camera.attrib["fovy"]) == 90.0, model_path
