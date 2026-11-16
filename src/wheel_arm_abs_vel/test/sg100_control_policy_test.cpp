// Pure policy regression tests: no ROS master, SDK or physical hand required.
#include "hardware_node/sg100_safety.h"
#include "motion_capture_ik/SG100BridgeGate.h"

#include <cstdio>

namespace {
int failures = 0;
int checks = 0;
void expect(bool condition, const char* name) {
    ++checks;
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", name);
    }
}

struct Command {
    int8_t control_mode = 0;
    uint16_t left_enable_mask = 0, right_enable_mask = 0;
    std::vector<float> left_hand_positions, right_hand_positions;
    std::vector<float> left_hand_velocities, right_hand_velocities;
    std::vector<int8_t> left_hand_control_mode, right_hand_control_mode;
    std::vector<float> left_hand_kp, right_hand_kp, left_hand_kd, right_hand_kd;
    std::vector<float> left_hand_torque_ff, right_hand_torque_ff;
    std::vector<float> left_hand_output_limit, right_hand_output_limit;
};

void testGate() {
    HighlyDynamic::SG100BridgeGate gate;
    expect(!gate.allow(false), "no input: no startup motion");
    expect(gate.allow(true), "first fresh input auto-enables VR");
    expect(!gate.allow(false), "stale input pauses VR");
    expect(gate.allow(true), "fresh input resumes without another enable call");
    expect(gate.enable(false, false), "manual disable works even without input");
    expect(!gate.allow(true), "fresh input cannot override manual disable");
    expect(!gate.enable(true, false), "explicit enable requires fresh input");
    expect(!gate.allow(true), "failed enable leaves manual disable intact");
    expect(gate.enable(true, true) && gate.allow(true), "explicit fresh enable succeeds");
    gate.selectService();
    expect(gate.allow(false), "service authority works without VR input");
    expect(gate.enable(false, false) && !gate.allow(true), "disable stops service output too");
    expect(gate.enable(true, false) && gate.allow(false), "service re-enable needs no VR input");
    gate.selectVr();
    expect(!gate.allow(false), "return to VR restores timeout protection");
    expect(gate.allow(true), "fresh VR works after explicit handover");
}

void testOwnership() {
    using sg100_safety::Owner;
    Owner vr;
    expect(!vr.claim(1, 0), "first owner needs no NULL handover");
    expect(!vr.expire(0.5), "0.5 s warning does not release VR ownership");
    expect(!vr.expire(1.999), "VR ownership lasts until 2 s");
    expect(vr.resumingVr(1, 0.5), "VR pause requires feedback filter reseed");
    expect(!vr.resumingVr(1, 0.499), "continuous VR does not reseed every frame");
    expect(!vr.allows(3), "script cannot steal fresh VR ownership");
    expect(vr.expire(2), "VR ownership expires at 2 s");
    expect(!vr.owned && vr.source == 1 && vr.ever_owned,
           "expiry only releases ownership, retaining previous source");
    expect(vr.allows(3), "script may take over expired VR");
    expect(vr.resumingVr(1, 2), "expired VR can resume with feedback reseed");
    expect(!vr.claim(1, 2), "same VR resumption does not inject NULL");
    vr.expire(4);
    expect(vr.claim(3, 4), "different source keeps protected handover");
    expect(!vr.allows(1), "VR cannot reclaim a hand taken by script");
    expect(!vr.expire(100) && vr.owned, "script one-shot ownership never expires");
    expect(vr.claim(0, 100), "debug can explicitly preempt script");
    expect(!vr.expire(1000) && vr.owned, "debug one-shot ownership never expires");
    expect(!vr.allows(1), "background VR cannot reclaim debug-controlled hand");

    Owner other;
    other.claim(2, 0);
    expect(!other.expire(100), "other source policy remains unchanged");
}

void testIndependentHands() {
    using sg100_safety::Owner;
    using sg100_safety::canApplyActiveCommand;
    Owner left, right;
    expect(!canApplyActiveCommand(left, 1, 1, 0, false, false, true),
           "disconnected/reconnecting left rejects its own command");
    expect(canApplyActiveCommand(right, 1, 1, 0, true, false, false),
           "healthy right accepts the same dual-hand command");
    left.claim(0, 0);
    expect(!canApplyActiveCommand(left, 1, 1, 0, true, false, false),
           "debug-owned left rejects VR");
    expect(canApplyActiveCommand(right, 1, 1, 0, true, false, false),
           "left ownership conflict does not block right");
    expect(!canApplyActiveCommand(right, 1, 1, 1, true, false, false),
           "pre-reconnect command cannot replay after recovery");
    expect(!canApplyActiveCommand(right, 1, 1, 0, true, true, false),
           "feedback timeout still blocks that hand");
    right.claim(1, 0);
    left.claim(0, 0);
    right.expire(2);
    expect(left.owned && !right.owned, "lease expiry is independent per hand");
    expect(sg100_safety::needsVrFilterReseed(1, 1.1, 1, 0),
           "short input pause still reseeds the VR filter");
    expect(!sg100_safety::needsVrFilterReseed(1, 0.9, 1, 0),
           "pre-resume pending command cannot consume reseed request");
    expect(!sg100_safety::needsVrFilterReseed(1, 1.1, 1, 1),
           "same resume request does not reseed every command");
    expect(!sg100_safety::needsVrFilterReseed(0, 1.1, 1, 0) &&
           !sg100_safety::needsVrFilterReseed(3, 1.1, 1, 0),
           "VR resume request never reseeds debug/script filters");
}

void testUnchangedContracts() {
    Command cmd;
    cmd.left_hand_positions.assign(11, 0);
    cmd.right_hand_positions.assign(11, 0);
    expect(sg100_safety::validate(cmd), "default NULL command remains valid");
    cmd.control_mode = 7;
    cmd.left_enable_mask = cmd.right_enable_mask = 0x07ff;
    expect(sg100_safety::validate(cmd), "explicit position mode remains valid");
    cmd.control_mode = 9;
    expect(sg100_safety::validate(cmd), "impedance mode remains valid");
    cmd.control_mode = 8;
    expect(!sg100_safety::validate(cmd), "unsupported modes remain rejected");
    cmd.control_mode = 7;
    cmd.right_enable_mask = 0;
    cmd.right_hand_positions.resize(10);
    expect(!sg100_safety::validate(cmd), "inactive hand still requires 11 positions");
    cmd.right_hand_positions.resize(11);
    cmd.left_hand_output_limit.assign(11, 0.5f);
    expect(sg100_safety::validate(cmd), "normalized output limit remains valid");
    cmd.left_hand_output_limit[0] = 1.1f;
    expect(!sg100_safety::validate(cmd), "normalized output limit above 1 is rejected");
    expect(sg100_safety::activeMask(0, 0x07ff, std::vector<int8_t>(11, 7)) == 0,
           "global NULL overrides per-joint motion modes");

    sg100_safety::StallProtection stall;
    expect(stall.update(0, 0, true, 1.1f, 0, 0, 0.3f) ==
           sg100_safety::ProtectionEvent::NONE, "stall timing begins normally");
    expect(stall.update(0, 1, true, 1.1f, 0, 0, 0.3f) ==
           sg100_safety::ProtectionEvent::STALL_TRIP, "stall still trips at 1 s");
    sg100_safety::Owner vr;
    vr.claim(1, 0);
    vr.expire(2);
    vr.claim(1, 2);
    expect(stall.latched == 1, "VR expiry/resumption never clears stall latch");
    stall.update(0, 3, false, 0, 0, 0, 0);
    expect(stall.latched == 1, "disabled joint retains stall latch");
}
}  // namespace

int main() {
    testGate();
    testOwnership();
    testIndependentHands();
    testUnchangedContracts();
    std::printf("SG100 control policy: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
