#!/usr/bin/env python3
"""Chest->Head freshness used by the vr_follow timeout, without starting ROS."""
import ast
import unittest
from pathlib import Path

source = Path(__file__).resolve().parents[1] / "scripts/head_control_node.py"
tree = ast.parse(source.read_text())
func = next(
    node for node in tree.body
    if isinstance(node, ast.FunctionDef) and node.name == "chest_head_tf_is_fresh"
)
namespace = {}
exec(compile(ast.Module(body=[func], type_ignores=[]), str(source), "exec"), namespace)


class ChestHeadFreshnessTest(unittest.TestCase):
    def test_recent_stamp_is_fresh(self):
        fresh = namespace["chest_head_tf_is_fresh"]
        self.assertTrue(fresh(10.0, 9.5, 3.0))
        self.assertTrue(fresh(10.0, 7.0, 3.0))
        self.assertTrue(fresh(10.0, 10.0, 3.0))

    def test_stale_or_missing_stamp_is_not_fresh(self):
        fresh = namespace["chest_head_tf_is_fresh"]
        self.assertFalse(fresh(10.0, 6.9, 3.0))
        self.assertFalse(fresh(10.0, None, 3.0))
        self.assertFalse(fresh(10.0, 10.1, 3.0))


if __name__ == "__main__":
    unittest.main()
