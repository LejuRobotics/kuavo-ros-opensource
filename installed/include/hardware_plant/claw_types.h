#ifndef CLAW_TYPES_H_
#define CLAW_TYPES_H_

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

} // namespace eef_controller

#endif 
