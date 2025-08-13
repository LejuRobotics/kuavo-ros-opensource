#ifndef _REVO2_HAND_CONTROLLER_H_
#define _REVO2_HAND_CONTROLLER_H_
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


/**
 * @brief revo2灵巧手控制器
 * 
 */
class Revo2HandController;
using Revo2HandControllerPtr = std::unique_ptr<Revo2HandController>;
class Revo2HandController {
public:    
    static Revo2HandControllerPtr Create(const std::string &action_sequences_path, bool is_touch_dexhand) {
        return std::unique_ptr<Revo2HandController>(new Revo2HandController(action_sequences_path, is_touch_dexhand));
    }

    ~Revo2HandController();

    /**
     * @brief 初始化控制器
     * 
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool init();

    /**
     * @brief 关闭控制器并释放资源
     * 
     * @return true 关闭成功
     * @return false 关闭失败
     */
    bool close();

    /**
     * @brief 发送左手位置指令
     * 
     * @param position 范围: 0~100, 0为张开, 100为闭合
     */
    void send_left_position(const UnsignedFingerArray & position);

    /**
     * @brief 发送右手位置指令
     * 
     * @param position 范围: 0~100, 0为张开, 100为闭合
     */
    void send_right_position(const UnsignedFingerArray & position);

    /**
     * @brief 同时发送双手位置指令
     * 
     * @param finger_positions array[2]，分别为左手和右手的位置
     */
    void send_position(const UnsignedDualHandsArray & finger_positions);

    /**
     * @brief 发送左手速度指令
     * 
     * @param speeds 范围: -100~100, 正为屈曲, 负为伸展
     */
    void send_left_speed(const FingerArray & speeds);

    /**
     * @brief 发送右手速度指令
     * 
     * @param speeds 范围: -100~100, 正为屈曲, 负为伸展
     */
    void send_right_speed(const FingerArray & speeds);

    /**
     * @brief 同时发送双手速度指令
     * 
     * @param speeds array[2]，分别为左手和右手的速度
     */
    void send_speed(const DualHandsArray & speeds);
    
    /**
     * @brief 设置双手抓力等级
     * 
     * @param level 抓力等级
     * @return true 至少一只手设置成功
     * @return false 两只手都失败
     */
    bool set_hand_force_level(GripForce level);

    /**
     * @brief 设置右手抓力等级
     * 
     * @param level 抓力等级
     * @return true 设置成功
     * @return false 设置失败或右手未连接
     */
    bool set_right_hand_force_level(GripForce level);

    /**
     * @brief 设置左手抓力等级
     * 
     * @param level 抓力等级
     * @return true 设置成功
     * @return false 设置失败或左手未连接
     */
    bool set_left_hand_force_level(GripForce level);

    /**
     * @brief 获取双手手指状态
     * 
     * @return FingerStatusPtrArray array[2]，分别为左手和右手的状态
     */
    FingerStatusPtrArray get_finger_status();

    /**
     * @brief 获取双手触摸传感器状态
     * 
     * @return FingerTouchStatusPtrArray array[2]，分别为左手和右手的触摸状态
     */
    FingerTouchStatusPtrArray get_touch_status();

    /**
     * @brief 使能左手触摸传感器
     * 
     * @param mask 使能掩码
     * @return true 成功
     * @return false 失败或左手未连接
     */
    bool enable_left_hand_touch_sensor(uint8_t mask);
    
    /**
     * @brief 使能右手触摸传感器
     * 
     * @param mask 使能掩码
     * @return true 成功
     * @return false 失败或右手未连接
     */
    bool enable_right_hand_touch_sensor(uint8_t mask);

    /**
     * @brief 列出所有手势
     * 
     * @return std::vector<GestureInfoPtr> 手势信息指针列表
     */
    std::vector<GestureInfoPtr> list_gestures();
    
    /**
     * @brief 执行一组手势
     * 
     * @param gesture_tasks 需要执行的手势列表
     * @param err_msg 错误信息
     * @return true 成功
     * @return false 失败
     */
    bool execute_gestures(const GestureExecuteInfoVec& gesture_tasks, std::string &err_msg);

    /**
     * @brief 检查是否有手势正在执行
     * 
     * @return true 有手势正在执行
     * @return false 没有手势在执行
     */
    bool is_gesture_executing();

private:
    Revo2HandController(const std::string &action_sequences_path, bool is_touch_dexhand);
    Revo2HandController(Revo2HandController&&) = delete;
    Revo2HandController(const Revo2HandController&) = delete;

    bool init_revo2hand();

    bool init_revo2_dexhand();
    bool init_normal_dexhand();

    /**
     * @brief 解析手势配置文件
     * 
     * @param gesture_file_path 手势配置文件路径
     * @param gesture_infos 输出的手势信息列表
     * @return true 解析成功
     * @return false 解析失败
     */
    bool ParseActionSequenceFile(const std::string & gesture_file_path, std::vector<GestureInfoPtr> &gesture_infos);

    void control_thread_func();
    void gesture_thread_func();
    bool sleep_for_100ms(int ms_count);

    // 数据成员
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
    
    // 手势相关
    std::string action_sequences_path_;
    std::unordered_map<std::string, GestureInfoPtr> gesture_map_;
    struct GestureExecuteTask {
        HandSide        hand_side;  // 哪只手
        GestureInfoPtr  gesture;    // 手势
    };
    using BatchGestureTask = std::vector<GestureExecuteTask>;

    using TaskFunc = std::function<void()>;
    std::optional<TaskFunc> current_task_;
    std::mutex task_mutex_;
    std::condition_variable task_cv_;
    std::atomic<bool> gesture_executing_{false};
    std::atomic_bool abort_running_task_flag_ = false;  // 终止任务执行
    std::thread gesture_thread_;

    static constexpr int control_frequency_ = 200; // Hz
};
} // namespace eef_controller
#endif
