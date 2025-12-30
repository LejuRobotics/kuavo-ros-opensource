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

    static LejuClawControllerPtr Create() {
        return LejuClawControllerPtr(new LejuClawController());
    }
    ~LejuClawController();
    
    /**
     * @brief Control gripper callback
     * 
     * @param req   Control request
     * @param res   Control response
     * @return false if failed. 
     */
    bool controlGripper(ControlClawRequest &req, ControlClawResponse &res);

    /**
     * @brief ROS topic callback
     * 
     * @param msg   ROS topic message
     */
    void command(const lejuClawCommand &msg);

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
     * @brief control the claws to move to the given position
     * 
     * @note  this function is synchronous, it will block until the claws reach the given position
     * 
     * @param positions  0 ~ 100, the percentage of the claw's opening angle
     *                   0: closed, 100: open   
     * @param velocity   0 ~ 100
     * @param torque     torque/current, better 1A ~ 2A
     * @return std::array<State, 2>     the state of the claws
     */
    std::array<State, 2> move_paw(const std::vector<double> &positions, const std::vector<double> &velocity,const std::vector<double> &torque);

private:
    bool execute(const lejuClawCommand &data, std::string &err_msg);

    LejuClawController() = default;
    void workerThread();
    /**
     * @brief update the state of the claws
     * 
     * @param new_state 
     */
    void updateState(std::array<State, 2> new_state);

    LeJuClaw *claw_ptr_ = nullptr;

    using TaskFunc = std::function<void()>;
    std::optional<TaskFunc> current_task_;
    std::mutex task_mutex_;
    std::condition_variable condition_;
    std::thread worker_;
    bool stop_worker_ = false;

    std::atomic_bool claw_is_executing_;

    std::array<State, 2> gripper_state_ = {State::kUnknown, State::kUnknown};
};
} // namespace eef_controller

#endif