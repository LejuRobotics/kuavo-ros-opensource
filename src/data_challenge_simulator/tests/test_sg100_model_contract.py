"""Static checks for the shared collect/model SG100 hand contract."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
HELPER = PACKAGE / "examples/helperfunc.py"
ENTRY = PACKAGE / "scripts/model_entry.py"
COMPAT = PACKAGE / "scripts/rosbag_compat_publisher.py"
GRIPPER = PACKAGE / "utils/gripper_controller.py"
LAUNCH = PACKAGE / "launch"


def test_collect_records_sg100_action_not_internal_cb_commands():
    source = HELPER.read_text()
    topics = source[source.index("ROS_TOPICS = ["):
                    source.index("]", source.index("ROS_TOPICS = ["))]
    assert "'/sg100_hand_command'" in topics
    assert "'/sg100_hand_state'" in topics
    assert "'/cb_left_hand_control_cmd'" not in topics
    assert "'/cb_right_hand_control_cmd'" not in topics


def test_task_gripper_uses_direct_sg100_interface():
    source = GRIPPER.read_text()
    assert '"/sg100_hand_command", SG100HandCommand' in source
    assert '"/sg100_hand_state", SG100HandState' in source
    assert 'message.control_mode = SG100HandCommand.MODE_JOINT_POSITION' in source
    assert 'message.left_enable_mask = 0x07FF' in source
    assert 'message.right_enable_mask = 0x07FF' in source
    assert '"/cb_' not in source


def test_model_waits_for_sg100_state_not_legacy_gripper_state():
    source = ENTRY.read_text()
    required = source[source.index("required = ("):
                      source.index("for topic in required:")]
    assert '"/sg100_hand_state"' in required
    assert '"/cb_left_hand_state"' not in required
    assert '"/cb_right_hand_state"' not in required
    assert '"/gripper/state"' not in required


def test_model_wrappers_enable_direct_heiman_command_gate():
    for task_id in (1, 2, 3):
        model = (LAUNCH / "load_kuavo_mujoco_model{}.launch".format(
            task_id)).read_text()
        assert '<arg name="require_model_command_gate" value="true"/>' in model
        assert '<arg name="publish_cmd_vel_fallback" value="false"/>' in model
        assert "blackman_gripper_compat" not in model


def test_collect_launches_use_direct_heiman_without_hand_mirroring():
    names = (
        "load_kuavo_mujoco_sim1.launch",
        "load_kuavo_mujoco_sim2.launch",
        "load_kuavo_mujoco_sim3.launch",
    )
    compat = COMPAT.read_text()
    assert 'rospy.get_param("~publish_cmd_vel_fallback", True)' in compat
    assert "SG100HandCommand" not in compat
    assert "SG100HandState" not in compat
    assert '"/cb_' not in compat
    for name in names:
        source = (LAUNCH / name).read_text()
        assert '<arg name="require_model_command_gate" default="false"/>' in source
        assert 'value="$(arg require_model_command_gate)"' in source
        assert '<arg name="publish_cmd_vel_fallback" default="true"/>' in source
        assert 'value="$(arg publish_cmd_vel_fallback)"' in source


def test_blackman_bridge_is_not_installed():
    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    assert "blackman_gripper_compat.py" not in cmake


def test_compat_node_does_not_publish_arm_commands():
    source = COMPAT.read_text()
    assert '"/kuavo_arm_traj", JointState' in source
    assert "self.arm_command_pub" not in source
    assert "_arm_fallback_command" not in source


def test_collect_cmd_vel_fallback_generates_only_stationary_commands():
    source = COMPAT.read_text()
    assert '"/cmd_vel", Twist' in source
    assert 'rospy.get_param("~cmd_vel_publish_rate", 100.0)' in source
    assert 'self.last_external_cmd_vel_time = time.monotonic()' in source
    assert 'self.last_external_cmd_vel_is_zero = self._is_zero_cmd_vel(message)' in source
    fallback = source[source.index("def _cmd_vel_fallback_command"):
                      source.index("def _publish(self, _event)")]
    assert "message = Twist()" in fallback
    assert "last_time is not None and not last_is_zero" in fallback
    assert "message.linear." not in fallback
    assert "message.angular." not in fallback
    assert "self._publish_cmd_vel" in source
    shared_timer = source[source.index("def _publish(self, _event)"):
                          source.index("def _publish_cmd_vel(self, _event)")]
    assert "_cmd_vel_fallback_command" not in shared_timer


def test_chassis_motion_publishes_at_100_hz_by_default():
    source = (PACKAGE / "examples/task2_base_motion.py").read_text()
    assert "yaw_tolerance_deg, publish_rate=100.0" in source
