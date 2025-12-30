#ifndef _DEXHAND_TOUCH_CONTROLLER_H_
#define _DEXHAND_TOUCH_CONTROLLER_H_
#include "dexhand_base.h"
#include "dexhand_def.h"

#include <memory>
#include <array>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <optional>
#include <functional>

namespace eef_controller {  
using namespace dexhand;
using DualHandsArray = std::array<FingerArray, 2>;
using UnsignedDualHandsArray = std::array<UnsignedFingerArray, 2>;
using FingerStatusPtrArray = std::array<FingerStatusPtr, 2>;
using FingerTouchStatusPtrArray = std::array<FingerTouchStatusPtr, 2>;

// open dual hand positions
const UnsignedDualHandsArray kDualHandOpenPositions = {kOpenFingerPositions, kOpenFingerPositions};
// close dual hand positions
const UnsignedDualHandsArray kDualHandClosePositions = {kCloseFingerPositions, kCloseFingerPositions};     

/**
 * @brief 触觉灵巧手控制器
 * 
 */
class DexhandController;
using TouchDexhandControllerPtr = std::unique_ptr<DexhandController>;
class DexhandController {
public:    
    static TouchDexhandControllerPtr Create(const std::string &action_sequences_path, bool is_touch_dexhand) {
        return std::unique_ptr<DexhandController>(new DexhandController(action_sequences_path, is_touch_dexhand));
    }

    ~DexhandController();

    /**
     * @brief Initialize the controller
     * 
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool init();

    /**
     * @brief Close the controller and release resources
     * 
     * @return true if close successful
     * @return false if close failed
     */
    bool close();

    /**
     * @brief Send position command to left hand
     * 
     * @param position range: 0~100, 0 for open, 100 for close
     */
    void send_left_position(const UnsignedFingerArray & position);

    /**
     * @brief Send position command to right hand
     * 
     * @param position range: 0~100, 0 for open, 100 for close
     */
    void send_right_position(const UnsignedFingerArray & position);

    /**
     * @brief Send position commands to both hands
     * 
     * @param finger_positions array[2] containing positions for left and right hands
     */
    void send_position(const UnsignedDualHandsArray & finger_positions);

    /**
     * @brief Send speed command to left hand
     * 
     * @param speeds range: -100~100, positive for flexion, negative for extension
     */
    void send_left_speed(const FingerArray & speeds);

    /**
     * @brief Send speed command to right hand
     * 
     * @param speeds range: -100~100, positive for flexion, negative for extension
     */
    void send_right_speed(const FingerArray & speeds);

    /**
     * @brief Send speed commands to both hands
     * 
     * @param speeds array[2] containing speeds for left and right hands
     */
    void send_speed(const DualHandsArray & speeds);
    
    /**
     * @brief Set the force level for both hands
     * 
     * @param level Force level to set
     * @return true if successful for at least one hand
     * @return false if failed for both hands
     */
    bool set_hand_force_level(GripForce level);

    /**
     * @brief Set the force level for right hand
     * 
     * @param level Force level to set
     * @return true if successful
     * @return false if failed or right hand not connected
     */
    bool set_right_hand_force_level(GripForce level);

    /**
     * @brief Set the force level for left hand
     * 
     * @param level Force level to set
     * @return true if successful
     * @return false if failed or left hand not connected
     */
    bool set_left_hand_force_level(GripForce level);

    /**
     * @brief Get the finger status of both hands
     * 
     * @return FingerStatusPtrArray array[2] containing status for left and right hands
     */
    FingerStatusPtrArray get_finger_status();

    /**
     * @brief Get the touch sensor status of both hands
     * 
     * @return FingerTouchStatusPtrArray array[2] containing touch status for left and right hands
     */
    FingerTouchStatusPtrArray get_touch_status();

    /**
     * @brief Enable the touch sensor for left hand
     * 
     * @param mask mask to enable touch sensor
     * @return true if successful
     * @return false if failed or left hand not connected
     */
    bool enable_left_hand_touch_sensor(uint8_t mask);
    
    /**
     * @brief Enable the touch sensor for right hand
     * 
     * @param mask mask to enable touch sensor
     * @return true if successful
     * @return false if failed or right hand not connected
     */
    bool enable_right_hand_touch_sensor(uint8_t mask);

    /**
     * @brief List all gestures
     * 
     * @return std::vector<GestureInfoPtr> vector of gesture info pointers
     */
    std::vector<GestureInfoPtr> list_gestures();
    
    /**
     * @brief  execute a list of gestures
     * 
     * @param gesture_tasks  the list of gestures to execute
     * @param err_msg        the error message
     * @return true if success, false otherwise 
     */
    bool execute_gestures(const GestureExecuteInfoVec& gesture_tasks, std::string &err_msg);

    /**
     * @brief check if a gesture is currently executing.
     * 
     * @return true if a gesture is currently executing, false otherwise
     */
    bool is_gesture_executing();

private:
    DexhandController(const std::string &action_sequences_path, bool is_touch_dexhand);
    DexhandController(DexhandController&&) = delete;
    DexhandController(const DexhandController&) = delete;

    bool init_touch_dexhand();
    bool init_normal_dexhand();

    void control_thread_func();
    void gesture_thread_func();
    bool sleep_for_100ms(int ms_count);

    /* data */
    bool is_touch_dexhand_{false};
    std::atomic<bool> l_position_updated_{false};
    std::atomic<bool> r_position_updated_{false};
    UnsignedFingerArray right_position_;
    UnsignedFingerArray left_position_;

    std::atomic<bool> l_speed_updated_{false};
    std::atomic<bool> r_speed_updated_{false};
    FingerArray right_speed_;
    FingerArray left_speed_;

    FingerStatusPtrArray finger_status_;
    FingerTouchStatusPtrArray finger_touch_status_;

    std::unique_ptr<dexhand::DexHandBase> left_dexhand_ = nullptr;
    std::unique_ptr<dexhand::DexHandBase> right_dexhand_ = nullptr;

    std::thread control_thread_;
    std::atomic<bool> running_{false};
    
    // gesture related
    std::string action_sequences_path_;
    std::unordered_map<std::string, GestureInfoPtr> gesture_map_;
    struct GestureExecuteTask {
        HandSide        hand_side;  /* which hand */
        GestureInfoPtr  gesture;    /* gesture */
    };
    using BatchGestureTask = std::vector<GestureExecuteTask>;

    using TaskFunc = std::function<void()>;
    std::optional<TaskFunc> current_task_;
    std::mutex task_mutex_;
    std::condition_variable task_cv_;
    std::atomic<bool> gesture_executing_{false};
    std::atomic_bool abort_running_task_flag_ = false;  /* abort task execute! */
    std::thread gesture_thread_;

    static constexpr int control_frequency_ = 200; // Hz
};
} // namespace eef_controller
#endif