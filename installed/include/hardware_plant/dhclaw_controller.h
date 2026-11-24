#ifndef __DHCLAW_CONTROLLER_H__
#define __DHCLAW_CONTROLLER_H__

#include <array>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "claw_types.h"

namespace eef_controller {

class DhClawController;
using DhClawControllerPtr = std::unique_ptr<DhClawController>;

/**
 * @brief Get current DH claw state in the same layout as leju claw.
 */
ClawState GetDhClawState(const DhClawControllerPtr &controller_ptr);

/**
 * @brief Adapter controller: DhClaw backend, leju claw command/state semantics.
 *
 * External position/velocity/effort match /leju_claw_command:
 *   position 0~100, velocity 0~100, effort leju-style amps or DH percent force.
 * Left/right map to yaml devices Llejuclaw / Rlejuclaw when present.
 *
 * Note: driver lives in namespace dhclaw; this header intentionally does not
 * include it so EndEffectorType::dhclaw (global unscoped enum) can coexist.
 */
class DhClawController {
public:
    enum class State : int8_t {
        kError = -1,
        kUnknown = 0,
        kMoving = 1,
        kReached = 2,
        kGrabbed = 3,
        kDropped = 4,
    };

    static DhClawControllerPtr Create();

    ~DhClawController();

    bool controlGripper(ControlClawRequest &req, ControlClawResponse &res);
    void command(const lejuClawCommand &msg);

    /// @param init_bmapilib unused (kept for API symmetry with LejuClawController)
    bool initialize(bool init_bmapilib = false);
    void setDebugCallback(LejuClawDebugCallback callback);
    bool recover(const std::string &direction, float current, int duration_ms,
                 int repeat, std::string &err_msg);

    void close();

    // Copy position, status and sample identity together under the cache lock.
    ClawState get_feedback();
    std::vector<double> get_positions();
    std::vector<double> get_torque();
    std::vector<double> get_velocity();
    std::array<State, 2> get_state() const;

    /// @param enable_left / enable_right: false 表示本帧不驱动该侧（保持原位）
    std::array<State, 2> move_paw(const std::vector<double> &positions,
                                  const std::vector<double> &velocity,
                                  const std::vector<double> &torque,
                                  bool enable_left = true,
                                  bool enable_right = true);

private:
    // Opaque wrapper so headers that also see EndEffectorType::dhclaw never
    // open namespace dhclaw in the same translation unit.
    struct ClawDevice;

    // Defined in .cc so unique_ptr<ClawDevice> can use incomplete type in this header.
    DhClawController();

    bool execute(const lejuClawCommand &data, std::string &err_msg);
    void workerThread();
    void updateState(std::array<State, 2> new_state);
    void update_vr_detection();
    /// 限频读 CAN 刷新 position/state 缓存；force=true 跳过间隔（运动等待内用）
    void refreshFeedbackCache(bool force = false);

    static uint8_t mapEffort(double effort);
    static uint8_t mapSpeed(double velocity);
    static State mapGripStatus(int gs);

    bool openSide(const std::string &device_name, std::unique_ptr<ClawDevice> &out);
    bool initOne(std::unique_ptr<ClawDevice> &claw, const char *side_name);
    bool setOne(std::unique_ptr<ClawDevice> &claw, double position, double velocity,
                double effort);

    std::unique_ptr<ClawDevice> left_claw_;
    std::unique_ptr<ClawDevice> right_claw_;

    std::chrono::steady_clock::time_point last_command_time_{};
    bool is_vr_control_mode_ = false;
    static constexpr int VR_CONTROL_DETECTION_INTERVAL_MS = 200;
    /// 与手臂共总线时，同步读寄存器过快会打满 CAN 发送队列（MESSAGE_QUEUE_FULL）
    static constexpr int kFeedbackPollIntervalMs = 20;  // 50 Hz

    using TaskFunc = std::function<void()>;
    std::optional<TaskFunc> current_task_;
    std::mutex task_mutex_;
    std::condition_variable condition_;
    std::thread worker_;
    std::atomic_bool stop_worker_{false};

    std::atomic_bool claw_is_executing_{false};
    std::mutex feedback_poll_mutex_;
    mutable std::mutex feedback_mutex_;
    std::array<State, 2> gripper_state_ = {State::kUnknown, State::kUnknown};
    std::vector<double> last_position_{0.0, 0.0};
    std::chrono::steady_clock::time_point last_feedback_poll_{};
    std::chrono::steady_clock::time_point feedback_sample_time_{};
    uint32_t feedback_sequence_ = 0;
    LejuClawDebugCallback debug_callback_;

    // last commanded effort (no continuous torque feedback from DH)
    std::vector<double> last_effort_{0.0, 0.0};
    std::vector<double> last_velocity_{50.0, 50.0};
};

}  // namespace eef_controller

#endif  // __DHCLAW_CONTROLLER_H__
