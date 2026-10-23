"""The high lift/carry join remains continuous and clears the fixtures."""
import ast
import math
from pathlib import Path

import numpy as np


def test_join_keeps_velocity_and_height_through_both_corner_boundaries():
    source = Path(__file__).resolve().parents[1] / "examples/task1.py"
    node = next(n for n in ast.parse(source.read_text()).body
                if isinstance(n, ast.FunctionDef) and n.name == "_rounded_waypoint_point")
    namespace = {"math": math}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), "exec"), namespace)
    point = namespace["_rounded_waypoint_point"]
    start, via, end = [0., 0., 0.68], [0., 0., 0.81], [0.3, -0.2, 0.81]
    first, second = math.dist(start, via), math.dist(via, end)
    enter, leave = (first - 0.015) / (first + second), (first + 0.015) / (first + second)
    np.testing.assert_allclose(point(start, via, end, 0), start)
    np.testing.assert_allclose(point(start, via, end, 1), end)
    for fraction in np.linspace(0, 1, 501):
        p = point(start, via, end, fraction)
        if fraction < enter:
            np.testing.assert_allclose(p[:2], start[:2], atol=1e-12)
        else:
            assert p[2] >= 0.795 - 1e-12
        if fraction >= leave:
            assert abs(p[2] - 0.81) < 1e-12
    delta = 1e-6
    for boundary in (enter, leave):
        center = np.array(point(start, via, end, boundary))
        before = (center - point(start, via, end, boundary - delta)) / delta
        after = (np.array(point(start, via, end, boundary + delta)) - center) / delta
        np.testing.assert_allclose(before, after, atol=1e-4)
        assert np.linalg.norm(before) > 0.1
