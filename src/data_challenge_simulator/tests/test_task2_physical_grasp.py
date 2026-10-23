"""Task 2 contacts must be observations, never kinematic attachments."""
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]


def test_task2_has_no_grasp_or_reposition_override():
    source = (ROOT / "src/mujoco/src/mujoco_node.cc").read_text()
    for forbidden in (
        "task2_grasp_enabled", "task2_reposition_enabled",
        "applyBimanualLatchControls", "setBimanualFingertipCollisionEnabled",
        "setBimanualObjectCollisionEnabled", "captureTask2RepositionLatch",
    ):
        assert forbidden not in source
    observer = source.split("void updateBimanualContactFollowers()", 1)[1]
    observer = observer.split("void applyTask2ConveyorForce()", 1)[0]
    assert "mj_contactForce" in observer
    for forbidden in ("d->qpos", "d->qvel", "d->ctrl", "m->geom_contype"):
        assert forbidden not in observer


def test_task_requires_actual_lift_and_opens_hands_for_release():
    source = (PACKAGE / "examples/task2_pick.py").read_text()
    assert "minimum_lift_m" in source and "wait_for_hand_targets" in source
    assert "not grasp_state.held[docking.box_name]" in source
    assert "rise < minimum_rise" in source
    assert "wait_until_released" in source
    assert "task2_grasp_enabled" not in source
    assert "task2_reposition_enabled" not in source


def test_task2_arm_execution_has_no_extra_motor_pd():
    source = (ROOT / "src/mujoco/src/mujoco_node.cc").read_text()
    scene = (PACKAGE / "models/biped_s400062/xml/task2.xml").read_text()
    for removed in ("applyArmMotorPositionControl", "arm_motor_command",
                    "simulate_arm_motor_pd"):
        assert removed not in source
        assert removed not in scene
    assert "tau[i] = msg->tau[i];" in source
    assert "d->ctrl[*iter] = tau_cmd[i++];" in source


def test_joint_command_preserves_input_torque(tmp_path):
    """Exercise the actual callback with nonzero targets, speeds and gains."""
    import subprocess

    source = (ROOT / "src/mujoco/src/mujoco_node.cc").read_text()
    callback = source[source.index("void jointCmdCallback("):
                      source.index("void clawCmdCallback(")]
    harness = r"""
#include <cassert>
#include <memory>
#include <mutex>
#include <vector>
#define ROS_WARN_STREAM_THROTTLE(...) ((void)0)
namespace kuavo_msgs {
struct jointCmd {
  using ConstPtr = std::shared_ptr<const jointCmd>;
  std::vector<double> joint_q, joint_v, tau, tau_ratio, tau_max, joint_kp, joint_kd;
  std::vector<int> control_modes;
};
}
size_t numJoints = 20;
std::mutex queueMutex;
std::vector<double> joint_tau_cmd;
bool cmd_updated = false;
"""
    checks = r"""
int main() {
  auto msg = std::make_shared<kuavo_msgs::jointCmd>();
  msg->joint_q.assign(numJoints, 1.5);
  msg->joint_v.assign(numJoints, -2.0);
  msg->joint_kp.assign(numJoints, 600.0);
  msg->joint_kd.assign(numJoints, 42.0);
  msg->tau_ratio.assign(numJoints, 0.5);
  msg->tau_max.assign(numJoints, 100.0);
  msg->control_modes.assign(numJoints, 2);
  for (size_t i = 0; i < numJoints; ++i)
    msg->tau.push_back((static_cast<int>(i) - 10) * 0.75);
  jointCmdCallback(msg);
  assert(cmd_updated && joint_tau_cmd == msg->tau);

  msg->joint_q.assign(numJoints, -1.0);
  msg->joint_v.assign(numJoints, 3.0);
  msg->joint_kp.assign(numJoints, 900.0);
  msg->joint_kd.assign(numJoints, 80.0);
  cmd_updated = false;
  jointCmdCallback(msg);
  assert(cmd_updated && joint_tau_cmd == msg->tau);

  const auto previous = joint_tau_cmd;
  msg->tau.assign(numJoints, 9.0);
  msg->joint_v.pop_back();
  cmd_updated = false;
  jointCmdCallback(msg);
  assert(!cmd_updated && joint_tau_cmd == previous);
}
"""
    cpp = tmp_path / "joint_cmd.cpp"
    exe = tmp_path / "joint_cmd"
    cpp.write_text(harness + callback + checks)
    subprocess.run(["g++", "-std=c++14", str(cpp), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
