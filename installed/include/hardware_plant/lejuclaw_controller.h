#ifndef __LEJU_CLAW_CONTROLLER_H__
#define __LEJU_CLAW_CONTROLLER_H__
#include <memory>
#include <vector>
#include <array>
#include <thread>
#include <mutex>
#include <optional>
#include <functional>
#include <condition_variable>
#include <atomic>
#include "claw_types.h"
#include "lejuclaw.h"
#include "lejuclaw_can_customed.h"

class LeJuClaw;

namespace eef_controller {
class LejuClawController;
using LejuClawControllerPtr = std::unique_ptr<LejuClawController>;

/**
 * @brief Copy the leju claw config file to the robot's config file
 *
 * @param src_file the path of the src file
 * @return true if copy success
 */
bool CopyOrUpdateLejuClawConfigFile(const std::string &src_file);

/**
 * @brief Get current claw state
 * 
 * @param controller_ptr 
 * @return ClawState current state of the claw
 */
ClawState GetClawState(const LejuClawControllerPtr &controller_ptr);

const std::string kLeftGripperName = "left_claw";
const std::string kRightGripperName = "right_claw";

class LejuClawController {
public:
    enum class State : int8_t {
        kError = -1,                // 错误
        kUnknown = 0,               // none
        kMoving = 1,                // 移动中
        kReached = 2,               // 到达位置
        kGrabbed = 3,               // 抓取到物品 
    };   

    static LejuClawControllerPtr Create(bool is_can_protocol) {
        return LejuClawControllerPtr(new LejuClawController(is_can_protocol));
    }
    // 兼容旧测试：无参版本默认走 CAN 协议
    static LejuClawControllerPtr Create() {
        return LejuClawControllerPtr(new LejuClawController(false));
    }
    ~LejuClawController();
    
    /**
     * @brief Control gripper callback
     * 
     * @param req   Control request
     * @param res   Control response
     * @return false if failed. 
     */
    bool controlGripper(ControlClawRequest &req, ControlClawResponse &res, bool is_high_freq);

    /**
     * @brief ROS topic callback
     *
     * @param msg   ROS topic message
     * @param is_high_freq  高频（连续流/允许覆盖）或低频（一次性定位/执行中拒绝）
     */
    void command(const lejuClawCommand &msg, bool is_high_freq);

    /**
     * @brief Initialize the Leju Claw
     * 
     * @param init_bmapilib  whether to initialize the bmapilib, avoid multiple initialization.
     * 
     * @note if other functions were called `BM_Init`, please set `init_bmapilib` to false.
     * 
     * @return false if initialize failed.
     */
    bool initialize(bool init_bmapilib);
    void setDebugCallback(LejuClawDebugCallback callback);

    /**
     * @brief 设置真实下发指令回调（观察口）：驱动层每次实际下发 PTM 前回报最终参数
     *
     * 串口/CAN 回调签名不同（串口一次回两爪数组，CAN 逐电机回），控制器内部
     * 各自适配为统一的 LejuClawTargetCallback（claw_types.h 定义）再上报。
     */
    void setTargetCallback(LejuClawTargetCallback callback);
    bool recover(const std::string& direction, float current, int duration_ms, int repeat, std::string& err_msg);

    /**
     * @brief close the claws and stop control thread.
     *     
     * */
    void close();

    /**
     * @brief Get the current postion of the claws
     * 
     * @return position of the claws {left, right}
     */
    std::vector<double> get_positions();

    /**
     * @brief Get the current torque of the claws
     * 
     * @return torque of the claws {left, right}
     */
    std::vector<double> get_torque();
    
    /*
    /**
     * @brief Get the current torque of the claws
     * 
     * @return torque of the claws {left, right}
     */
    std::vector<double> get_velocity();

    /**
     * @brief Get the current speed of the claws
     * 
     * @return speed of the claws {left, right}
     */
    std::array<State, 2> get_state() const {
        return gripper_state_;
    }

    /**
     * @brief Get the current control mode
     *
     * @return true if the claw is in high-freq (continuous stream) mode, false if low-freq (one-shot)
     */
    bool get_is_high_freq() const {
        return current_is_high_freq_.load();
    }

    /**
     * @brief control the claws to move to the given position
     * 
     * @note  this function is synchronous, it will block until the claws reach the given position
     * 
     * @param positions  5 ~ 95, the percentage of the claw's opening angle
     *                   5: closed limit, 95: open limit
     * @param velocity   0 ~ 100
     * @param torque     torque/current, better 1A ~ 2A
     * @return std::array<State, 2>     the state of the claws
     */
    std::array<State, 2> move_paw(const std::vector<double> &positions, const std::vector<double> &velocity,const std::vector<double> &torque, bool is_high_freq);

private:
    bool execute(const lejuClawCommand &data, bool is_high_freq, std::string &err_msg);

    /**
     * @brief 武装/刷新断流看门狗（高频命令到达时调用）
     */
    void armStreamWatchdog();
    /**
     * @brief 解除断流看门狗武装（低频命令/关闭时调用）
     */
    void disarmStreamWatchdog();
    /**
     * @brief 把 target_callback_ 以对应协议的适配 lambda 装进已存在的驱动
     * @note  setTargetCallback 与 initialize（驱动创建后）都会调用，保证两种时序下都能装上
     */
    void installTargetCallback();

    bool is_can_protocol_;
    explicit LejuClawController(bool is_can_protocol) : is_can_protocol_(is_can_protocol) {}
    void workerThread();
    /**
     * @brief update the state of the claws
     * 
     * @param new_state 
     */
    void updateState(std::array<State, 2> new_state);

    LeJuClaw *claw_ptr_ = nullptr;
    lejuclaw_can::LeJuClawCan *claw_can_ptr_ = nullptr;

    using TaskFunc = std::function<void()>;
    std::optional<TaskFunc> current_task_;
    std::mutex task_mutex_;
    std::condition_variable condition_;
    std::thread worker_;
    bool stop_worker_ = false;

    std::atomic_bool claw_is_executing_{false};
    std::atomic_bool recovery_in_progress_{false};

    std::array<State, 2> gripper_state_ = {State::kUnknown, State::kUnknown};
    std::atomic_bool current_is_high_freq_{false};   // 当前控制模式：true=高频，false=低频
    LejuClawDebugCallback debug_callback_;
    std::mutex target_callback_mutex_;
    LejuClawTargetCallback target_callback_;         // 真实下发指令回调（观察口）
};
} // namespace eef_controller

#endif
