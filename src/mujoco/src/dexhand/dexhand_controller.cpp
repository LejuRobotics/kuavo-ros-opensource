#include "dexhand_controller.h"
#include "dexhand_utils.h"
#include <iostream>
#include <fstream>
#include <chrono>

namespace eef_controller
{
    using namespace dexhand_utils;
    /**************** Class DexhandController  ****************/
    DexhandController::DexhandController(const std::string &gesture_filepath):gesture_file_path_(gesture_filepath)
    {

    }

    DexhandController::~DexhandController() 
    {
        this->close();
    }

    bool DexhandController::init(MujocoHandBasePtr l_dexhand, MujocoHandBasePtr r_dexhand)
    {
        left_dexhand_ = l_dexhand;
        right_dexhand_ = r_dexhand;

        /* read gesture file */
        std::vector<GestureInfoPtr> gesture_infos;
        ParseGestureInfos(gesture_file_path_, gesture_infos);

        /* add gesture to map */
        for(const auto &gesture_info : gesture_infos) {
            gesture_map_[gesture_info->gesture_name] = gesture_info;
            for (const auto &alias_name : gesture_info->alias) {
                gesture_map_[alias_name] = gesture_info;
            }
        }
        
        /* start gesture thread */
        gesture_thread_ = std::unique_ptr<std::thread>(new std::thread(&DexhandController::gesture_thread_func, this));

        return true;
    }

    bool DexhandController::close()
    {           
        /* stop gesture thread */
        if (gesture_thread_ && gesture_thread_->joinable()) {
            gesture_thread_exit_ = true;
            gesture_execute_cv_.notify_all();
            gesture_thread_->join();
            gesture_thread_ = nullptr;
        }

        left_dexhand_ = nullptr;
        right_dexhand_ = nullptr;

        return true;
    }
    void DexhandController::send_left_position(const UnsignedFingerArray & position)
    {
        if (!left_dexhand_) {
            std::cerr << "[DexhandController] Left dexhand is not connected." << std::endl;
        }

        // update left position
        left_dexhand_->setFingerPositions(position);
    }

    void DexhandController::send_right_position(const UnsignedFingerArray & position)
    {
        if (!right_dexhand_) {
            std::cerr << "[DexhandController] Right dexhand is not connected." << std::endl;
        }

        // update right position
        right_dexhand_->setFingerPositions(position);
    }

    void DexhandController::send_position(const UnsignedDualHandsArray & finger_positions)
    {
        if (left_dexhand_) {
            left_dexhand_->setFingerPositions(finger_positions[0]);
        }

        if (right_dexhand_) {
            right_dexhand_->setFingerPositions(finger_positions[1]);
        }
    }

    void DexhandController::send_left_speed(const FingerArray & speeds)
    {
        if (!left_dexhand_) {
            std::cerr << "[DexhandController] Left dexhand is not connected." << std::endl;
        }
        left_dexhand_->setFingerSpeeds(speeds);
    }

    void DexhandController::send_right_speed(const FingerArray & speeds)
    {
        if (!right_dexhand_) {
            // std::cerr << "[DexhandController] Right dexhand is not connected." << std::endl;
        }

        // update right speed
        right_dexhand_->setFingerSpeeds(speeds);
    }

    void DexhandController::send_speed(const DualHandsArray & speeds)
    {
        if (left_dexhand_) {
            left_dexhand_->setFingerSpeeds(speeds[0]);
        }

        if (right_dexhand_) {
            right_dexhand_->setFingerSpeeds(speeds[1]);
        }
    }

    FingerStatusPtrArray DexhandController::get_finger_status()
    {
        FingerStatusPtrArray finger_status;
        finger_status[0] = left_dexhand_->getFingerStatus();
        finger_status[1] = right_dexhand_->getFingerStatus();        
        return finger_status;
    }   

    bool DexhandController::execute_gestures(const GestureExecuteInfoVec& gesture_tasks, std::string &err_msg)
    {
        bool no_gesture = false;
        err_msg = "Not found gesture:";

        BatchGestureTask batch_task;
        for(const auto& task: gesture_tasks) {
            if (gesture_map_.find(task.gesture_name) == gesture_map_.end()) {
                no_gesture = true;
                err_msg += "'" + task.gesture_name + "', ";
                continue;
            }
            /* add to batch! */
            batch_task.emplace_back(GestureExecuteTask{task.hand_side, gesture_map_[task.gesture_name]});        
        }

        if(batch_task.size() == 0) {
            // std::cout << "DexhandController execute_gestures: no gesture to execute.\n";
            err_msg = "no gesture to execute.";
            return false;
        }

        /* execute gesture */
        std::unique_lock<std::mutex> lock(task_queue_mtx_);
        while(task_queue_.size() > kTaskQueueSize_) { /* clear task queue. */
            // std::cout << "task pop:" << task_queue_.front().at(0).gesture->gesture_name << "\n";
            task_queue_.pop();
        }
        task_queue_.emplace(batch_task);
        abort_running_task_flag_ = true;        /* abort previous gesture */
        gesture_execute_cv_.notify_all();       /* wake up gesture thread */

        if(!no_gesture) {
            err_msg = "success!";
        }
        
        return true;
    }

    std::vector<GestureInfoPtr> DexhandController::list_gestures()
    {
        std::vector<GestureInfoPtr> gestures;
        for (const auto &gesture : gesture_map_) {
            if(std::find(gestures.begin(), gestures.end(), gesture.second) == gestures.end()) {
                gestures.push_back(gesture.second);
            }
        }
        return gestures;
    }

    bool DexhandController::is_gesture_executing() {
        return gesture_executing_;
    }

    bool DexhandController::sleep_for_100ms(int ms_count)
    {
        int i = 0;
        while(i < ms_count) {
            if(abort_running_task_flag_) {
                // std::cout << "sleep_for_100ms: abort!\n";
                return false;
            }

            /* thread sleep for 100ms loop. */
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // std::cout << "sleep_for 100ms ...\n";
            i++;
        }

        return true;
    }

    void DexhandController::gesture_thread_func()
    {
        while (!gesture_thread_exit_) {
            BatchGestureTask batch_task;
            {
                /* waitting for gesture execute request. */
                std::unique_lock<std::mutex> lock(task_queue_mtx_);
                gesture_execute_cv_.wait(lock, [this]{ return (gesture_thread_exit_ || !task_queue_.empty()); }); 
                if(gesture_thread_exit_) return;
                batch_task = task_queue_.front(); 
                task_queue_.pop();
            }

            
            gesture_executing_ = true;          /* gesture executing flag. */
            abort_running_task_flag_ = false;   /* reset abort flag. */
            
            bool batch_task_aborted = false;
            for(auto &task : batch_task) {
                if(gesture_thread_exit_) return;    /* thread exit! */
                // std::cout << "gesture_thread_func execute: " << task.gesture->gesture_name << "\n";
                for (auto &as : task.gesture->action_sequences) {
                    if(gesture_thread_exit_) return; /* thread exit! */
                    if(abort_running_task_flag_) {
                        // std::cout << "gesture_thread_func: previous task abort:" << task.gesture->gesture_name << "\n";;
                        batch_task_aborted = true;
                        break;  /* abort! */
                    }
                    
                    // std::cout << "gesture_thread_func: send_position:";
                    // for(int i = 0; i < 6; i++) {
                    //     std::cout << " " << static_cast<int>(as.positions[i]);
                    // }
                    // std::cout << "\n";
                    UnsignedFingerArray pos;
                    for (int i = 0; i < 6; i++) {
                        pos[i] = static_cast<uint16_t>(as.positions[i]);
                    }
                    if(task.hand_side == HandSide::LEFT) {
                        
                        left_dexhand_->setFingerPositions(pos);
                    }
                    else if(task.hand_side == HandSide::RIGHT){
                        right_dexhand_->setFingerPositions(pos);
                    }
                    else {
                        left_dexhand_->setFingerPositions(pos);
                        right_dexhand_->setFingerPositions(pos);
                    }

                    /* sleep duration */
                    if(!this->sleep_for_100ms(std::max(1, as.duration_ms/100))) {
                        batch_task_aborted = true;
                        break;
                    }
                } // range-for action_sequences of task.

                if(batch_task_aborted) {
                    std::cout << "gesture_thread_func: previous task abort:" << task.gesture->gesture_name << "\n";
                    break;
                }
            } // range-for batch task.

            gesture_executing_ = false;          /* gesture executing flag. */
        }
    }
} // eef_controller