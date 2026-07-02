import importlib.util
import pathlib
import unittest


MODULE_PATH = pathlib.Path(__file__).with_name("setup_kuavo_ros_control_remote.py")
SPEC = importlib.util.spec_from_file_location("setup_kuavo_ros_control_remote", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)

if SPEC.loader is None:
    raise RuntimeError(f"Unable to load module from {MODULE_PATH}")

SPEC.loader.exec_module(MODULE)


class SetupKuavoRosControlRemoteTest(unittest.TestCase):
    def test_allowed_remote_accepts_factory_url(self):
        self.assertTrue(MODULE.is_allowed_remote(MODULE.FACTORY_URL, MODULE.KUAVO_ROS_ALLOWED_URLS))

    def test_allowed_remote_accepts_gitcode_url(self):
        self.assertTrue(MODULE.is_allowed_remote(MODULE.GITCODE_URL, MODULE.KUAVO_ROS_ALLOWED_URLS))

    def test_allowed_remote_accepts_gitee_url(self):
        self.assertTrue(MODULE.is_allowed_remote(MODULE.GITEE_URL, MODULE.KUAVO_ROS_ALLOWED_URLS))

    def test_clone_sources_default_is_auto(self):
        self.assertEqual(
            MODULE.clone_sources(),
            [MODULE.FACTORY_URL, MODULE.GITCODE_URL, MODULE.GITEE_URL],
        )

    def test_clone_sources_auto_prefers_factory_then_gitcode_then_gitee(self):
        self.assertEqual(
            MODULE.clone_sources(MODULE.SOURCE_MODE_AUTO),
            [MODULE.FACTORY_URL, MODULE.GITCODE_URL, MODULE.GITEE_URL],
        )

    def test_clone_sources_factory_only(self):
        self.assertEqual(
            MODULE.clone_sources(MODULE.SOURCE_MODE_FACTORY),
            [MODULE.FACTORY_URL],
        )

    def test_clone_sources_gitcode_only(self):
        self.assertEqual(
            MODULE.clone_sources(MODULE.SOURCE_MODE_GITCODE),
            [MODULE.GITCODE_URL],
        )

    def test_clone_sources_gitee_only(self):
        self.assertEqual(
            MODULE.clone_sources(MODULE.SOURCE_MODE_GITEE),
            [MODULE.GITEE_URL],
        )

    def test_branch_exists_matches_refs_output(self):
        ls_remote_output = (
            "1111111111111111111111111111111111111111\trefs/heads/master\n"
            "2222222222222222222222222222222222222222\trefs/heads/beta\n"
        )
        self.assertTrue(MODULE.branch_exists(ls_remote_output, "beta"))
        self.assertFalse(MODULE.branch_exists(ls_remote_output, "dev"))

    def test_commit_presence_detects_origin_factory_branch(self):
        contains_output = "  origin/master\n  origin_factory/master\n"
        self.assertTrue(MODULE.commit_exists_in_origin_factory(contains_output))
        self.assertFalse(MODULE.commit_exists_in_origin_factory("  origin/master\n"))


if __name__ == "__main__":
    unittest.main()
