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
#include <vector>
#include <string>
#include <unordered_map>

namespace dexhand {
class ModbusDexhand;   // 仅用于缓存句柄指针，实现细节在 modbus.h
}

namespace eef_controller {  
using namespace dexhand;
using DualHandsArray = std::array<FingerArray, 2>;
using UnsignedDualHandsArray = std::array<UnsignedFingerArray, 2>;
using FingerStatusArray = std::array<dexhand::FingerStatus, 2>;
using FingerTouchStatusArray = std::array<dexhand::TouchSensorStatusArray, 2>;

/**
 * @brief 触觉灵巧手控制器
 * 
 */
class DexhandController;
using TouchDexhandControllerPtr = std::unique_ptr<DexhandController>;
class DexhandController {
public:    
    static TouchDexhandControllerPtr Create(const std::string &action_sequences_path, bool is_touch_dexhand, bool is_can_protocol) {
        return std::unique_ptr<DexhandController>(new DexhandController(action_sequences_path, is_touch_dexhand, is_can_protocol));
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
     * @return FingerStatusArray array[2] containing status for left and right hands
     */
    FingerStatusArray get_finger_status();

    /**
     * @brief Get the touch sensor status of both hands
     * 
     * @return FingerTouchStatusArray array[2] containing touch status for left and right hands
     */
    FingerTouchStatusArray get_touch_status();

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

    /**
     * @brief Abort the currently executing gesture
     */
    void abort_gesture();

    /**
     * @brief 设置双手的turbo模式开关
     *
     * @param enabled true为开启turbo模式，false为关闭
     * @return true 至少有一只手设置成功
     * @return false 两只手都设置失败
     */
    bool set_turbo_mode_enabled(bool enabled);

    /**
     * @brief 设置右手的turbo模式开关
     *
     * @param enabled true为开启turbo模式，false为关闭
     * @return true 设置成功
     * @return false 设置失败或右手未连接
     */
    bool set_right_hand_turbo_mode_enabled(bool enabled);

    /**
     * @brief 设置左手的turbo模式开关
     *
     * @param enabled true为开启turbo模式，false为关闭
     * @return true 设置成功
     * @return false 设置失败或左手未连接
     */
    bool set_left_hand_turbo_mode_enabled(bool enabled);

    /**
     * @brief 查询左手是否开启了turbo模式
     *
     * @return true 左手已开启turbo模式
     * @return false 左手未开启turbo模式或左手未连接
     */
    bool is_left_hand_turbo_mode_enabled();

    /**
     * @brief 查询右手是否开启了turbo模式
     *
     * @return true 右手已开启turbo模式
     * @return false 右手未开启turbo模式或右手未连接
     */
    bool is_right_hand_turbo_mode_enabled();

    /**
     * @brief 查询双手是否都开启了turbo模式
     *
     * @return true 双手都已开启turbo模式
     * @return false 至少有一只手未开启turbo模式或未连接
     */
    bool is_turbo_mode_enabled();

    void set_linker_hand(bool is_linker);

private:
    DexhandController(const std::string &action_sequences_path,
                      bool is_touch_dexhand,
                      bool is_can_protocol);
    DexhandController(DexhandController&&) = delete;
    DexhandController(const DexhandController&) = delete;

    bool init_touch_dexhand();
    bool init_normal_dexhand();
    bool init_revo1_normal_can_customed();


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
    /// 提交一侧手待发的位置/速度命令。返回 false = 同手锁被占，命令保留到下一周期。
    void flushHandCommands(int side);
    /// CAN 下用 try 提交（回读线程会占锁），抢不到返回 false；485 下直接提交。
    bool submitPositions(dexhand::DexHandBase* hand, const UnsignedFingerArray& positions);
    bool submitSpeeds(dexhand::DexHandBase* hand, const FingerArray& speeds);
    /// 485 专有：状态轮询（CAN 下不查，请求/应答会把控制循环卡住）。
    void pollFingerStatus485();
    /// 触觉手传感器状态（仅 V1_TOUCH 实现该接口）。
    void pollTouchStatus();
    void feedback_thread_func();
    /// 查询并更新单侧快照：分类 Busy/Failed、按样本年龄告警、成功则写快照。
    void pollFeedbackSide(int side);
    /// 睡到本轮周期结束（扣掉本轮耗时），可被 close() 唤醒。
    void waitFeedbackCycle(int64_t cycle_start_ms);
    /// 线程退出时打印两侧累计计数（区分"抢不到锁"与"手不应答"）。
    void logFeedbackSummary();
    /// 解析 KUAVO_HAND_FEEDBACK_MS：0=关闭，kFeedbackPeriodMinMs~MaxMs 生效，
    /// 未设置或非法值返回 fallback_ms（并告警）。
    static int resolveFeedbackPeriodMs(int fallback_ms);
    /// steady_clock 毫秒，用于算样本年龄与周期。
    static int64_t monoNowMs();
    void gesture_thread_func();
    bool sleep_for_100ms(int ms_count);

    /* data */
    bool is_can_protocol_{false};
    bool is_touch_dexhand_{false};
    bool is_linker_hand_{false};
    std::atomic<bool> l_position_updated_{false};
    std::atomic<bool> r_position_updated_{false};
    UnsignedFingerArray right_position_;
    UnsignedFingerArray left_position_;

    std::atomic<bool> l_speed_updated_{false};
    std::atomic<bool> r_speed_updated_{false};
    FingerArray right_speed_;
    FingerArray left_speed_;

    FingerStatusArray finger_status_;
    FingerTouchStatusArray finger_touch_status_;

    // 真实反馈快照（CAN 模式，由 feedback_thread_ 定期刷新）
    // positions 已换算为 0~100，与 /dexhand/command 同口径；速度/电流保留 SDK 原值
    FingerStatusArray feedback_snapshot_;
    bool feedback_valid_[2] = {false, false};
    mutable std::mutex feedback_mutex_;
    std::thread feedback_thread_;

    /// 单侧反馈线程的运行时状态（按侧聚合，避免多个平行数组靠下标对齐）
    struct FeedbackSideState {
        const char* name = "";
        int64_t last_ok_ms = 0;      // 上次成功查询的时刻，用于算样本年龄
        int64_t last_warn_ms = 0;    // 上次陈旧告警的时刻，用于限流
        bool first_logged = false;   // 只打印一次首样本
        bool cast_warned = false;    // 非 ModbusDexhand 只告警一次
        uint64_t ok = 0;
        uint64_t busy = 0;           // 锁被占（正常让位）
        uint64_t failed = 0;         // SDK 异常或无数据
    };
    static constexpr int kHandCount = 2;
    FeedbackSideState feedback_sides_[kHandCount] = {{"left"}, {"right"}};
    // 启动时缓存，避免每轮 dynamic_cast；close() 在 join 之后置空
    dexhand::ModbusDexhand* feedback_hands_[kHandCount] = {nullptr, nullptr};
    // 反馈线程的周期等待：条件变量让运行标志一变就立即退出（对应内核 wait_event_timeout）
    std::mutex feedback_wait_mutex_;
    std::condition_variable feedback_wait_cv_;
    // 反馈线程参数（单位 ms）
    static constexpr int kFeedbackPeriodDefaultMs = 200;   // 5Hz/手，验证过的上限
    static constexpr int kFeedbackPeriodMinMs = 200;       // KUAVO_HAND_FEEDBACK_MS 下限
    static constexpr int kFeedbackPeriodMaxMs = 60000;     // 上限（约 0.017Hz）
    static constexpr int kFeedbackStaleWarnMs = 2000;      // 样本年龄超过此值才告警
    static constexpr int kFeedbackWarnIntervalMs = 10000;  // 同侧告警限流间隔
    int feedback_period_ms_{kFeedbackPeriodDefaultMs};

    // 线程安全保护
    mutable std::mutex finger_status_mutex_;
    mutable std::mutex touch_status_mutex_;

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
