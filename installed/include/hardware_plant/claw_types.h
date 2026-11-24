#ifndef CLAW_TYPES_H_
#define CLAW_TYPES_H_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>
#include <functional>

namespace eef_controller {

struct JointState {
    std::vector<std::string> name;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
};

struct ClawState {
    JointState data;
    std::vector<int8_t> state = {0, 0};
    bool is_high_freq = false;
    // DH-only feedback metadata. Cached reads retain identity and query-start time.
    uint32_t feedback_sequence = 0;
    std::chrono::steady_clock::time_point feedback_sample_time{};
};

struct lejuClawCommand {
    std::vector<double> position;
    std::vector<std::string> name;
    std::vector<double> velocity;
    std::vector<double> effort;
};

struct ControlClawRequest {
    lejuClawCommand data;
};

struct ControlClawResponse {
    bool success;
    std::string message;
};

using LejuClawDebugCallback = std::function<void(const std::string&)>;

// Shared gripper names for lejuclaw / dhclaw command & state messages
const std::string kLeftGripperName = "left_claw";
const std::string kRightGripperName = "right_claw";

using LejuClawTargetCallback = std::function<void(const std::vector<std::string>& name,
                                                  const std::vector<double>& position,
                                                  const std::vector<double>& velocity,
                                                  const std::vector<double>& effort,
                                                  const std::vector<int>& kp,
                                                  const std::vector<int>& kd,
                                                  bool zero_kp)>;

} // namespace eef_controller

#endif 
