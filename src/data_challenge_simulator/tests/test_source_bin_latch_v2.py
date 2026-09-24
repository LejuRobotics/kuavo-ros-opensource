import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts/source_bin_latch_v2.py"
)
SPEC = importlib.util.spec_from_file_location("source_bin_latch_v2", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_relative_pose_round_trip_preserves_full_object_pose():
    angle = math.radians(30.0)
    parent_pose = (
        (0.62, -0.196, 0.65),
        (math.cos(angle / 2.0), 0.0, math.sin(angle / 2.0), 0.0),
    )
    child_pose = ((0.55, -0.10, 0.68), (1.0, 0.0, 0.0, 0.0))

    relative_pose = MODULE._relative_pose(parent_pose, child_pose)
    reconstructed_pose = MODULE._compose_pose(parent_pose, relative_pose)

    for actual, expected in zip(reconstructed_pose[0], child_pose[0]):
        assert actual == pytest.approx(expected, abs=1e-12)
    for actual, expected in zip(reconstructed_pose[1], child_pose[1]):
        assert actual == pytest.approx(expected, abs=1e-12)


def test_only_centres_inside_source_bin_are_followed():
    assert MODULE._inside_source_bin((0.0, 0.0, 0.035))
    assert not MODULE._inside_source_bin((0.30, 0.0, 0.035))
    assert not MODULE._inside_source_bin((0.0, -0.30, 0.035))
    assert not MODULE._inside_source_bin((0.0, 0.0, 0.30))


def test_captured_source_object_moves_with_bin_but_other_objects_do_not():
    latch = MODULE.SourceBinLatch.__new__(MODULE.SourceBinLatch)
    latch._lock = MODULE.threading.Lock()
    latch._source_bin_pose = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    latch._object_poses = {
        "cylinder_1": ((0.10, 0.10, 0.035), (1.0, 0.0, 0.0, 0.0)),
        "cylinder_2": ((1.00, 0.00, 0.035), (1.0, 0.0, 0.0, 0.0)),
        "cylinder_3": ((0.00, -1.00, 0.035), (1.0, 0.0, 0.0, 0.0)),
    }
    requests = []
    latch._set_object_position = lambda request: (
        requests.append(request) or SimpleNamespace(success=True))

    followers = latch._capture_bin_followers()
    assert sorted(followers) == ["cylinder_1"]

    latch._source_bin_pose = ((0.20, 0.0, -0.10), (1.0, 0.0, 0.0, 0.0))
    latch._sync_bin_followers(followers)

    assert len(requests) == 1
    assert requests[0].object_name == "cylinder_1"
    assert requests[0].position.x == pytest.approx(0.30)
    assert requests[0].position.y == pytest.approx(0.10)
    assert requests[0].position.z == pytest.approx(-0.065)
