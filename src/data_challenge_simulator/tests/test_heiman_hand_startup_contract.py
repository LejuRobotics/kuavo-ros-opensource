"""Static contract for motionless SG100 simulation startup."""

from pathlib import Path


WORKSPACE_SRC = Path(__file__).resolve().parents[2]
HEIMAN_HAND = WORKSPACE_SRC / "mujoco/src/dexhand/heiman_hand.hpp"
DEXHAND_NODE = WORKSPACE_SRC / "mujoco/src/dexhand_mujoco_node.cpp"


def test_heiman_hand_holds_qpos_until_first_real_command():
    source = HEIMAN_HAND.read_text()

    assert "kLeftRestPose" not in source
    assert "kRightRestPose" not in source
    assert "filtered_positions_[i] = d->qpos[*iter]" in source
    assert "target_positions_[i] = d->qpos[*iter]" in source
    assert "command_arrived_before_first_write" in source


def test_heiman_node_does_not_inject_a_rest_pose():
    source = DEXHAND_NODE.read_text()

    assert "HeimanHand::kLeftRestPose" not in source
    assert "HeimanHand::kRightRestPose" not in source
    assert "std::make_shared<HeimanHand>(model, l_hand_address)" in source
    assert "std::make_shared<HeimanHand>(model, r_hand_address)" in source
