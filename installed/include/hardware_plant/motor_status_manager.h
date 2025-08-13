#ifndef MOTOR_STATUS_MANAGER_H_
#define MOTOR_STATUS_MANAGER_H_

#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>

#include "actuators_interface.h"

namespace HighlyDynamic
{

/**
 * @brief 电机状态枚举
 */
enum class MotorStatus : int
{
    ENABLE = 0,          // 启用状态
    ERROR = 1,           // 错误状态
    DISABLED = 2,        // 禁用状态
    UNKNOWN = 255        // 未知状态
};

/**
 * @brief 单个电机的状态信息
 */
struct MotorStatusInfo
{
    int joint_id;                           // 关节ID
    MotorStatus current_status;             // 当前状态
    MotorStatus previous_status;            // 上一次状态
    int status_code;                        // 原始状态码
    int error_count;                        // 连续错误计数
    std::chrono::steady_clock::time_point last_error_time;  // 最后错误时间
    std::chrono::steady_clock::time_point last_log_time;    // 最后日志输出时间
    int position_error_count;               // 位置错误计数器
    bool status_changed;                    // 状态是否改变
    std::string last_error_msg;             // 最后的错误信息
    
    // 默认构造函数
    MotorStatusInfo() : joint_id(-1), current_status(MotorStatus::UNKNOWN),
                       previous_status(MotorStatus::UNKNOWN), status_code(0),
                       error_count(0), status_changed(false),
                       last_error_time(std::chrono::steady_clock::now()),
                       last_log_time(std::chrono::steady_clock::now()),
                       position_error_count(0) {}
    
    // 参数化构造函数
    explicit MotorStatusInfo(int id) : joint_id(id), current_status(MotorStatus::UNKNOWN),
                             previous_status(MotorStatus::UNKNOWN), status_code(0),
                             error_count(0), status_changed(false),
                             last_error_time(std::chrono::steady_clock::now()),
                             last_log_time(std::chrono::steady_clock::now()),
                             position_error_count(0) {
    }
};

/**
 * @brief 电机状态管理器
 * 负责管理所有电机的状态，避免刷屏输出
 */
class MotorStatusManager
{
public:
    /**
     * @brief 构造函数
     * @param num_joints 关节数量
     * @param error_threshold 错误阈值（当前用于错误计数统计，不触发自动状态改变）
     * @param log_interval_ms 日志输出间隔（毫秒）
     */
    explicit MotorStatusManager(size_t num_joints, int error_threshold = 10, int log_interval_ms = 1000);
    
    /**
     * @brief 析构函数
     */
    ~MotorStatusManager() = default;

    /**
     * @brief 更新关节状态
     * @param joint_id 关节ID
     * @param status_code 状态码
     * @param position 关节位置
     * @param velocity 关节速度
     * @param torque 关节力矩
     * @return 是否存在错误
     */
    bool updateJointStatus(int joint_id, int status_code, double position = 0.0, 
                          double velocity = 0.0, double torque = 0.0);

    /**
     * @brief 检查所有关节状态
     * @param joint_data 关节数据
     * @param joint_ids 关节ID列表
     * @param error_msg 错误信息输出
     * @return 是否所有关节正常
     */
    bool checkAllJointsStatus(const std::vector<JointParam_t> &joint_data,
                             const std::vector<uint8_t> &joint_ids,
                             std::string &error_msg);

    /**
     * @brief 设置关节位置限制
     * @param min_limits 最小位置限制数组
     * @param max_limits 最大位置限制数组
     * @param num_joints 关节数量
     */
    void setJointPositionLimits(const std::vector<double>& min_limits, 
                               const std::vector<double>& max_limits);



    /**
     * @brief 获取所有关节的状态信息
     * @return 关节ID到状态的映射
     */
    std::map<int, MotorStatus> getAllJointsStatus() const;

    /**
     * @brief 获取所有关节的详细状态信息
     * @return 关节ID到详细状态信息的映射
     */
    std::map<int, MotorStatusInfo> getAllJointsDetailedStatus() const;

    /**
     * @brief 打印所有电机的详细状态信息
     * @param reason 打印状态的原因
     */
    void printAllMotorStatus(const std::string& reason = "System shutdown") const;

    /**
     * @brief 初始化电机状态
     * @param joint_data 关节数据
     * @param joint_ids 关节ID列表
     */
    void initMotorStatus(const std::vector<JointParam_t> &joint_data,
                        const std::vector<uint8_t> &joint_ids);

    /**
     * @brief 设置错误阈值
     * @param threshold 错误阈值（用于错误计数统计，不触发自动状态改变）
     */
    void setErrorThreshold(int threshold) { error_threshold_ = threshold; }

    /**
     * @brief 设置日志输出间隔
     * @param interval_ms 间隔时间（毫秒）
     */
    void setLogInterval(int interval_ms) { log_interval_ms_ = interval_ms; }

    /**
     * @brief 获取状态统计信息
     * @param interval_s 输出间隔（秒），默认10秒
     * @return 状态统计字符串
     */
    std::string getStatusSummary(int interval_s = 10) const;

    /**
     * @brief 是否需要触发急停
     * @return 是否需要急停
     */
    bool shouldTriggerEmergencyStop() const;

    /**
     * @brief 设置关节状态
     * @param joint_id 关节ID
     * @param target_status 目标状态
     * @param reason 状态变更原因
     */
    void setJointStatus(int joint_id, MotorStatus target_status, const std::string& reason = "");
    
    /**
     * @brief 手动清除关节错误状态
     * @param joint_id 关节ID
     * @param reason 清除原因
     * @return 是否成功清除
     */
    bool clearJointError(int joint_id, const std::string& reason = "Manual error clear");

private:
    /**
     * @brief 收集关节状态信息
     * @param joint_id 关节ID
     * @param new_errors 新错误列表
     * @param status_changes 状态变化列表
     */
    void collectJointStatusInfo(int joint_id, std::vector<std::string>& new_errors,
                               std::vector<std::string>& status_changes);

    /**
     * @brief 构建错误消息
     * @param new_errors 新错误列表
     * @param status_changes 状态变化列表
     * @param oss 输出流
     */
    void buildErrorMessage(const std::vector<std::string>& new_errors,
                          const std::vector<std::string>& status_changes,
                          std::ostringstream& oss);

    /**
     * @brief 更新状态码和计数器
     * @param info 电机状态信息
     * @param target_status 目标状态
     */
    void updateStatusCodeAndCounters(MotorStatusInfo& info, MotorStatus target_status);

    /**
     * @brief 记录状态变化
     * @param joint_id 关节ID
     * @param info 电机状态信息
     * @param reason 原因
     */
    void logStatusChange(int joint_id, const MotorStatusInfo& info, const std::string& reason);

    /**
     * @brief 打印报告头部
     * @param reason 原因
     */
    void printReportHeader(const std::string& reason) const;

    /**
     * @brief 打印状态摘要
     * @param total_motors 总电机数
     * @param enable_count 启用数量
     * @param error_count 错误数量
     * @param disabled_count 禁用数量
     * @param unknown_count 未知数量
     */
    void printStatusSummary(size_t total_motors, int enable_count, int error_count, 
                           int disabled_count, int unknown_count) const;

    /**
     * @brief 打印详细状态表格
     * @param motor_list 电机状态列表
     */
    void printDetailedStatusTable(const std::vector<MotorStatusInfo>& motor_list) const;

    /**
     * @brief 打印错误详情
     * @param motor_list 电机状态列表
     * @param error_count 错误数量
     * @param disabled_count 禁用数量
     */
    void printErrorDetails(const std::vector<MotorStatusInfo>& motor_list, 
                          int error_count, int disabled_count) const;

    /**
     * @brief 统计电机状态数量（内联函数）
     * @param status 电机状态
     * @param enable_count 启用数量
     * @param error_count 错误数量
     * @param disabled_count 禁用数量
     * @param unknown_count 未知数量
     */
    inline void countMotorStatus(MotorStatus status, int& enable_count, int& error_count, 
                                int& disabled_count, int& unknown_count) const {
        switch (status) {
            case MotorStatus::ENABLE:
                enable_count++;
                break;
            case MotorStatus::ERROR:
                error_count++;
                break;
            case MotorStatus::DISABLED:
                disabled_count++;
                break;
            default:
                unknown_count++;
                break;
        }
    }

    /**
     * @brief 将状态码转换为电机状态（内联函数）
     * @param status_code 原始状态码
     * @return 电机状态
     */
    inline MotorStatus statusCodeToMotorStatus(int status_code) const {
        // 根据EcDemoApp.h中定义的状态码进行映射：
        // MOTOR_STATUS_NO_ERROR = 0, MOTOR_STATUS_ERROR = 1, MOTOR_STATUS_REINIT = 2
        switch (status_code) {
            case 0:  // MOTOR_STATUS_NO_ERROR
                return MotorStatus::ENABLE;
            case 1:  // MOTOR_STATUS_ERROR
                return MotorStatus::ERROR;
            case 2:  // MOTOR_STATUS_REINIT
                return MotorStatus::DISABLED;  // 重新初始化状态视为禁用
            default:
                return MotorStatus::UNKNOWN;
        }
    }

    /**
     * @brief 检查是否需要输出日志（内联函数）
     * @param motor_info 电机状态信息
     * @return 是否需要输出日志
     */
    inline bool shouldLogError(const MotorStatusInfo& motor_info) const {
        if (motor_info.current_status != MotorStatus::ERROR) {
            return false;
        }
        
        auto now = std::chrono::steady_clock::now();
        auto time_since_last_log = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - motor_info.last_log_time).count();
        
        // 如果距离上次日志输出超过指定间隔，允许输出
        return time_since_last_log >= log_interval_ms_;
    }

    /**
     * @brief 获取状态名称（内联函数）
     * @param status 电机状态
     * @return 状态名称字符串
     */
    inline std::string getStatusName(MotorStatus status) const {
        switch (status) {
            case MotorStatus::ENABLE: return "ENABLE";
            case MotorStatus::ERROR: return "ERROR";
            case MotorStatus::DISABLED: return "DISABLED";
            case MotorStatus::UNKNOWN: return "UNKNOWN";
            default: return "UNDEFINED";
        }
    }

    /**
     * @brief 重置所有电机状态（内联函数）
     * @param motor_list 电机状态列表
     * @param total_errors 总错误数
     */
    inline void resetAllMotorStatus(std::vector<MotorStatusInfo>& motor_list, int& total_errors) {
        for (auto& info : motor_list) {
            info.current_status = MotorStatus::UNKNOWN;
            info.previous_status = MotorStatus::UNKNOWN;
            info.status_code = 0;
            info.error_count = 0;
            info.status_changed = false;
            info.last_error_msg.clear();
        }
        total_errors = 0;
    }

    // ===== 线程安全的私有辅助函数 =====
    
    /**
     * @brief 获取电机状态信息指针（线程安全）
     * @param joint_id 关节ID
     * @return 电机状态信息指针，如果不存在返回nullptr
     */
    inline MotorStatusInfo* getMotorStatusInfoPtr(int joint_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = joint_id_to_index_.find(joint_id);
        return (it != joint_id_to_index_.end()) ? &motor_status_list_[it->second] : nullptr;
    }
    
    inline const MotorStatusInfo* getMotorStatusInfoPtr(int joint_id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = joint_id_to_index_.find(joint_id);
        return (it != joint_id_to_index_.end()) ? &motor_status_list_[it->second] : nullptr;
    }
    
    /**
     * @brief 获取所有电机状态的副本（线程安全）
     * @return 电机状态列表的副本
     */
    inline std::vector<MotorStatusInfo> getAllMotorStatusCopy() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return motor_status_list_;  // 返回副本
    }
    
    /**
     * @brief 更新单个电机状态（线程安全）
     * @param joint_id 关节ID
     * @param updater 更新函数，接收MotorStatusInfo&参数
     * @return 是否更新成功
     */
    template<typename UpdateFunc>
    inline bool updateMotorStatusSafe(int joint_id, UpdateFunc updater) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = joint_id_to_index_.find(joint_id);
        if (it != joint_id_to_index_.end()) {
            updater(motor_status_list_[it->second]);
            return true;
        }
        return false;
    }
    
    /**
     * @brief 批量更新电机状态（线程安全）
     * @param updater 更新函数，接收motor_status_list_引用
     */
    template<typename UpdateFunc>
    inline void batchUpdateMotorStatusSafe(UpdateFunc updater) {
        std::lock_guard<std::mutex> lock(mutex_);
        updater(motor_status_list_, joint_id_to_index_, total_errors_);
    }
    
    /**
     * @brief 获取单个电机状态（线程安全）
     * @param joint_id 关节ID
     * @param default_status 如果找不到时的默认状态
     * @return 电机状态
     */
    inline MotorStatus getJointStatusSafe(int joint_id, MotorStatus default_status = MotorStatus::UNKNOWN) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = joint_id_to_index_.find(joint_id);
        return (it != joint_id_to_index_.end()) ? motor_status_list_[it->second].current_status : default_status;
    }
    
    /**
     * @brief 检查电机是否为特定状态（线程安全）
     * @param joint_id 关节ID
     * @param status 要检查的状态
     * @return 是否为指定状态
     */
    inline bool isJointInStatus(int joint_id, MotorStatus status) const {
        return getJointStatusSafe(joint_id) == status;
    }

private:
    std::vector<MotorStatusInfo> motor_status_list_;    // 电机状态列表
    std::unordered_map<int, size_t> joint_id_to_index_; // 关节ID到索引的映射
    
    int error_threshold_;           // 错误阈值
    int log_interval_ms_;          // 日志输出间隔
    size_t num_joints_;            // 关节数量
    
    // 位置限制
    std::vector<double> min_joint_position_limits_;  // 最小位置限制
    std::vector<double> max_joint_position_limits_;  // 最大位置限制
    bool position_limits_set_{false};                       // 是否设置了位置限制
    
    // 统计信息
    mutable std::chrono::steady_clock::time_point last_summary_time_;
    mutable int total_errors_{0};
    mutable std::mutex mutex_; // 互斥锁
};

} // namespace HighlyDynamic 

#endif // MOTOR_STATUS_MANAGER_H_