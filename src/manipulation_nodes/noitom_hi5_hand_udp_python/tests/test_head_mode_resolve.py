#!/usr/bin/env python3
"""Chest->Head freshness and vr_follow pause/resume, without starting ROS."""
import ast
import unittest
import xml.etree.ElementTree as ET
from types import SimpleNamespace
from unittest.mock import Mock
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


class VrFollowPauseTest(unittest.TestCase):
    def test_missing_stale_and_recovered_tf_preserve_mode(self):
        node_class = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                          and n.name == "HeadTrackHandNode")
        spin = next(n for n in node_class.body if isinstance(n, ast.FunctionDef)
                    and n.name == "spin")
        # Startup without TF, reception, disconnection beyond timeout, recovery.
        stamps = [None, 10.0, 10.0, 20.0]
        times = [0.0, 10.0, 19.0, 20.0]
        index = [0]
        ros = SimpleNamespace(
            Rate=lambda hz: SimpleNamespace(sleep=lambda: index.__setitem__(0, index[0] + 1)),
            is_shutdown=lambda: index[0] >= len(stamps),
            logwarn_throttle=Mock(), ROSInterruptException=RuntimeError,
        )
        scope = {"rospy": ros, "HeadControlMode": SimpleNamespace(FIXED="fixed", VR_FOLLOW="vr_follow")}
        exec(compile(ast.Module(body=[spin], type_ignores=[]), str(source), "exec"), scope)
        published = []
        node = SimpleNamespace(
            rate_hz=50.0, manager=SimpleNamespace(mode="vr_follow"),
            head_control_mode="vr_follow", _release_head_command=False,
            _vr_chest_head_is_fresh=lambda: namespace["chest_head_tf_is_fresh"](
                times[index[0]], stamps[index[0]], 3.0),
            _update_vr_follow=lambda: published.append(index[0]),
            _publish_head_mode=Mock(), _apply_public=Mock(),
        )
        scope["spin"](node)
        self.assertEqual(published, [1, 3])
        self.assertEqual(node.manager.mode, "vr_follow")
        self.assertEqual(node.head_control_mode, "vr_follow")
        node._apply_public.assert_not_called()
        self.assertEqual(node._publish_head_mode.call_count, 4)


class FixedReleaseTest(unittest.TestCase):
    def run_fixed(self, released):
        node_class = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                          and n.name == "HeadTrackHandNode")
        spin = next(n for n in node_class.body if isinstance(n, ast.FunctionDef)
                    and n.name == "spin")
        index = [0]
        ros = SimpleNamespace(
            Rate=lambda hz: SimpleNamespace(sleep=lambda: index.__setitem__(0, index[0] + 1)),
            is_shutdown=lambda: index[0] >= 5, ROSInterruptException=RuntimeError,
        )
        scope = {"rospy": ros, "robotHeadMotionData": SimpleNamespace,
                 "HeadControlMode": SimpleNamespace(FIXED="fixed", VR_FOLLOW="vr_follow")}
        exec(compile(ast.Module(body=[spin], type_ignores=[]), str(source), "exec"), scope)
        node = SimpleNamespace(
            rate_hz=50.0, manager=SimpleNamespace(mode="fixed"),
            _release_head_command=released, head_pub=Mock(),
            _publish_head_mode=Mock(),
        )
        # No TF lookup methods: fixed must release even when TF is absent.
        scope["spin"](node)
        return node

    def test_startup_fixed_does_not_publish(self):
        node = self.run_fixed(True)
        node.head_pub.publish.assert_not_called()
        self.assertEqual(node._publish_head_mode.call_count, 5)

    def test_explicit_fixed_publishes_once_then_releases(self):
        node = self.run_fixed(False)
        node.head_pub.publish.assert_called_once()
        self.assertEqual(node.head_pub.publish.call_args.args[0].joint_data, [0.0, 0.0])
        self.assertTrue(node._release_head_command)
        self.assertEqual(node._publish_head_mode.call_count, 5)


class HeadBaseFrameTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        resolver = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                        and n.name == "resolve_head_base_frame")
        scope = {"ET": ET}
        exec(compile(ast.Module(body=[resolver], type_ignores=[]), str(source), "exec"), scope)
        cls.resolve = staticmethod(scope["resolve_head_base_frame"])

    def test_explicit_frame_does_not_require_urdf(self):
        self.assertEqual(self.resolve(" base_link ", ""), "base_link")

    def test_actual_robot_models(self):
        assets = source.parents[3] / "kuavo_assets/models"
        for version, expected in (("40", "base_link"), ("45", "base_link"),
                                  ("49", "base_link"), ("400049", "base_link"),
                                  ("400053", "waist_yaw_link"),
                                  ("400062", "waist_yaw_link")):
            with self.subTest(version=version):
                urdf = assets / ("biped_s" + version) / "urdf" / ("biped_s" + version + ".urdf")
                self.assertEqual(self.resolve("", urdf.read_text()), expected)

    def test_invalid_model_requires_explicit_override(self):
        for xml in ("", "<robot/>",
                    '<robot><joint name="zhead_1_joint"><parent link="missing"/></joint></robot>'):
            with self.subTest(xml=xml):
                with self.assertRaises((ValueError, ET.ParseError)):
                    self.resolve("", xml)


if __name__ == "__main__":
    unittest.main()
