
#ifndef DEXHAND_CONTROLLER_H_
#define DEXHAND_CONTROLLER_H_
#include <stdint.h>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <condition_variable>
#include <array>

#include "dexhand_def.h"
#include "mujoco_hand_base.hpp"
#include "kuavo_msgs/gestureInfo.h"

namespace eef_controller {

using namespace mujoco_node;
class DexhandController;
using DexhandControllerPtr = std::unique_ptr<DexhandController>;

class DexhandController {
public:
    static DexhandControllerPtr Create(const std::string &gesture_filepath) {
        return std::unique_ptr<DexhandController>(new DexhandController(gesture_filepath));
    }

    ~DexhandController();

    bool init(MujocoHandBasePtr l_dexhand, MujocoHandBasePtr r_dexhand);
    bool close();
    
    /**
     * @brief Get the finger status of both hands
     * 
     * @return FingerStatusPtrArray array[2] containing status for left and right hands
     */
    FingerStatusPtrArray get_finger_status();

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
     * @brief  execute a list of gestures
     * 
     * @param gesture_tasks  the list of gestures to execute
     * @param err_msg        the error message
     * @return true if success, false otherwise 
     */
    bool execute_gestures(const GestureExecuteInfoVec& gesture_tasks, std::string &err_msg);

    /**
     * @brief List all gestures
     * 
     * @return std::vector<GestureInfoPtr> vector of gesture info pointers
     */
    std::vector<GestureInfoPtr> list_gestures();

    /**
     * @brief check if a gesture is currently executing.
     * 
     * @return true if a gesture is currently executing, false otherwise
     */
    bool is_gesture_executing();

private:
    DexhandController(const std::string &gesture_filepath);
    void gesture_thread_func();
    bool sleep_for_100ms(int ms_count);

private:
    MujocoHandBasePtr left_dexhand_ = nullptr;
    MujocoHandBasePtr right_dexhand_ = nullptr;
    std::string gesture_file_path_;                                 /* gesture config file */
    std::unordered_map<std::string, GestureInfoPtr> gesture_map_;   /* gesture info map */

    /* Task Queue Define! */
    struct GestureExecuteTask {
        HandSide        hand_side;  /* which hand */
        GestureInfoPtr  gesture;    /* gesture */
    };
    using BatchGestureTask = std::vector<GestureExecuteTask>;
    const int kTaskQueueSize_ = 1;            
    std::queue<BatchGestureTask> task_queue_;
    std::mutex task_queue_mtx_;
    std::condition_variable gesture_execute_cv_;
    std::atomic_bool abort_running_task_flag_ = false;  /* abort task execute! */
    std::atomic_bool gesture_executing_ = false;

    /* thread control */
    std::atomic_bool gesture_thread_exit_ = false;
    std::unique_ptr<std::thread> gesture_thread_ = nullptr;
};


} // namespace eef_controller

#endif