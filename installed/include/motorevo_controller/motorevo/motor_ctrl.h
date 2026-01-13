#ifndef _REVOMOTOR_CTRL_H_
#define _REVOMOTOR_CTRL_H_
#include <atomic>
#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <map>
#include "motorevo/motor_def.h"
#include "canbus_sdk/canbus_sdk.h"

namespace motorevo {

class RevoMotor {
public:
    RevoMotor(uint8_t canbus_id, MotorId id, MotorMode mode);
    RevoMotor(const RevoMotor& other);
    RevoMotor(RevoMotor&& other) noexcept;
    RevoMotor& operator=(const RevoMotor& other);
    RevoMotor& operator=(RevoMotor&& other) noexcept;
    virtual ~RevoMotor();

    ///////////////////////////////////////////////////////////////
    /*** Motor Info ***/

    /** @brief 获取电机的ID */
    uint32_t getId() const;

    /** @brief  获取电机运行状态
     *  @return 电机的运行状态， Rest State（休眠模式），Motor State（运行模式）
     */
    MotorState getState() const;

    /** @brief  获取电机控制模式
     *  @return 控制模式
     */
    MotorMode getControlMode() const;

    ///////////////////////////////////////////////////////////////
    /*** Motor Feedback ***/

    /** @brief  获取电机位置
     *  @return 位置(rad)，原始数据
     */
    float position() const;

    /** @brief  获取电机当前力矩
     *  @return 扭矩(N·m)，原始数据
     */
    float torque() const;

    /** @brief  获取电机当前速度
     *  @return 速度(rad/s)，原始数据
     */
    float velocity() const;

    /** @brief  获取电机温度
     *  @return 温度(℃)
     */
    uint8_t temperature() const;

    /** @brief  获取电机错误代码
     *  @return 错误代码
     */
    MotorErrCode getErrorCode() const;
    
    ///////////////////////////////////////////////////////////////
    /*** Motor Control ***/
    /* WARN: controlPTM, controlTorque, controlVelocity 等函数调用需要和 MotorMode 匹配，且和实际电机设置的控制模式一致
     *       !!! 否则！会出大问题 !!!
     *       !!! 否则！会出大问题 !!!
     *       !!! 否则！会出大问题 !!!
     */

    /** @brief  发送休眠模式指令
     *  @return 发送状态
     */
    bool enterRestState();

    /** @brief  发送电机运行模式指令
     *  @return 发送状态
     */
    bool enterMotorState();

    /** @brief  发送设置电机零位指令
     *  @return 发送状态
     */ 
    bool setZeroPosition();
    
    /** @brief  发送多圈编码器清零指令
     *  @return 发送状态
     */
    bool multiTurnZero();

    /** @brief 在 P-T-M 力位混合模式下运行电机
     *  @param pos θ_ref 位置
     *  @param vel V_ref 速度
     *  @param torque T_ref 力矩
     *  @param pos_kp 位置比例增益
     *  @param pos_kd 位置微分增益
     *  @return 成功/失败
     */
    bool controlPTM(float pos, float vel, float torque, float pos_kp, float pos_kd);

    /** @brief 在 Velocity 速度模式下运行电机
     *  @param vel V_ref 速度
     *  @param vel_kp 速度比例增益
     *  @param vel_ki 速度积分增益
     *  @param vel_kd 速度微分增益
     *  @return 成功/失败
     */
    bool controlVelocity(float vel, float vel_kp, float vel_ki, float vel_kd);

    /** @brief 在Torque 力矩模式下运行电机
     *  @param torque T_ref 力矩
     *  @return 成功/失败
     */
    bool controlTorque(float torque);

    ///////////////////////////////////////////////////////////////
    /*** Motor Helper ***/

    bool receiveFeedback(const FeedbackFrame& frame);

    ///////////////////////////////////////////////////////////////
private:
    struct data_t;
    MotorState          state_;            // 电机运行状态
    MotorMode           ctrl_mode_;        // 电机控制模式
    data_t*             data_;             // 电机原始数据
    uint8_t             canbus_id_;        // CAN总线ID
    MotorId             id_;               // 电机ID
};


class RevoMotorControl {
public:
    /** @brief 构造RevoMotor控制器
     *  @param canbus_name CAN总线名称
     */
    RevoMotorControl(const std::string& canbus_name);

    /** @brief 初始化电机控制器
     *  @param motor_configs 电机配置列表
     *  @param calibrate 是否进行零点校准
     *  @return 初始化是否成功
     */
    bool init(const std::vector<RevoMotorConfig_t> &motor_configs, bool calibrate);

    /** @brief 使能所有电机 */
    bool enableAll();
    
    /** @brief 失能所有电机 */
    bool disableAll();
    
    /** @brief 清零所有电机的多圈编码器 */
    void multiTurnZeroAll();

    /** @brief 使能指定电机
     *  @param id 电机ID
     *  @param timeout_ms 等待超时时间(毫秒)，默认100ms
     *  @return 返回值含义：
     *          - 0: 使能成功
     *          - 1: 操作超时
     *          - 2: 发送消息失败
     *          - 3: 电机不存在
     *          - 负数: 电机故障码的负数 => -MotorErrCode
     */
    int enableMotor(MotorId id, int timeout_ms = 100);

    /** @brief 失能指定电机
     *  @param id 电机ID
     *  @param timeout_ms 等待超时时间(毫秒)，默认100ms
     *  @return 返回值含义：
     *          - 0: 失能成功
     *          - 1: 操作超时
     *          - 2: 发送消息失败
     *          - 3: 电机不存在
     *          - 负数: 电机故障码的负数 => -MotorErrCode
     */
    int disableMotor(MotorId id, int timeout_ms = 100);

    /** @brief 设置0-torque模式
     *  @param enable 是否启用0-torque模式
     */
    void setZeroTorqueMode(bool enable);

    /** @brief 设置电机回零时的目标初始位置
     *  @param target_positions 电机ID到目标位置的映射表(rad)，如果某个电机ID不在映射表中，则默认回零到0
     *  @note 此函数用于设置moveToZero()函数中每个电机的目标位置，允许外部根据业务需求设置不同的初始位置
     */
    void setInitialTargetPositions(const std::map<MotorId, float>& target_positions);

    /** @brief 多圈编码器清零
     *  @param id 电机ID
     *  @param timeout_ms 等待超时时间(毫秒)，默认100ms
     *  @return 清零是否成功
     */
    bool multiTurnZero(MotorId id, int timeout_ms = 100);

    /** @brief 设置电机配置
     *  @param id 电机ID
     *  @param config 新的电机配置
     *  @return 设置是否成功
     */
    bool setMotorConfig(MotorId id, const RevoMotorConfig_t& config);

    
    /** @brief 获取指定电机的配置
     *  @param id 电机ID
     *  @return 电机配置，如果电机不存在则返回默认配置
     */
    RevoMotorConfig_t getMotorConfig(MotorId id) const;

    /** @brief 获取所有电机的零点偏移值
     *  @return 电机ID到零点偏移值的映射表(rad)
     */
    std::map<MotorId, float> getZeroOffsets();

    /** @brief 获取所有电机原始位置
     *  @return 电机ID到原始位置的映射表(未处理零点和方向)
     */
    std::map<MotorId, float> getRawPositions();

    /** @brief 获取所有电机原始速度
     *  @return 电机ID到原始速度的映射表(未处理方向)
     */
    std::map<MotorId, float> getRawVelocities();

    /** @brief 获取所有电机原始力矩
     *  @return 电机ID到原始力矩的映射表(未处理方向)
     */
    std::map<MotorId, float> getRawTorques();

    /** @brief 获取所有电机经过零点和方向计算后的位置
     *  @return 电机ID到处理后位置的映射表(已应用零点偏移和方向修正)
     */
    std::map<MotorId, float> getPositions();

    /** @brief 获取所有电机当前速度
     *  @return 电机ID到速度的映射表
     */
    std::map<MotorId, float> getVelocities();

    /** @brief 获取所有电机当前力矩
     *  @return 电机ID到力矩的映射表
     */
    std::map<MotorId, float> getTorques();

    /** @brief 获取所有电机的状态信息
     *  @return 电机ID到运行状态的映射表
     */
    std::map<MotorId, MotorState> getMotorStates();

    /** @brief 设置所有电机的目标位置、速度和力矩
     *  @param targets 电机ID到目标控制参数的映射表
     *  @note 此函数只更新目标值，不发送到电机，需要调用write()函数发送
     */
    void setTargetPositions(const std::map<MotorId, RevoMotorCmd_t>& targets);

    /** @brief 发送电机控制指令到所有电机
     *  @note 只有当目标更新标志为true时才会发送，发送后重置标志
     */
    void write();

    /** @brief 析构函数，清理资源 */
    ~RevoMotorControl();

private:
    // 指令操作
    enum class Operation {
        IDLE,           // 空闲
        ENABLE,         // 使能
        DISABLE,        // 失能
        MULTI_TURN_ZERO // 多圈清零
    };

    enum class OperationStatus {
        PENDING,        // 等待确认
        SUCCESS,        // 成功
        TIMEOUT,        // 超时
        FAILED          // 失败
    };
    // 更新电机控制命令
    void updateMotorCmd(MotorId id, double pos, double vel, double torque, double kp, double kd);

    // 从当前位置插值到零点
    void moveToZero(float zero_timeout = 1.0f);
    // 校准零点
    bool calibrateMotors();
    
    // CAN消息回调函数
    static void internalMessageCallback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context);

    // TEF事件回调函数
    static void internalTefEventCallback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context);

    // 等待操作状态完成
    /**
     * @brief 等待电机操作完成并检查状态
     * @param id 电机ID
     * @param expected_op 期望的操作类型
     * @param expected_status 期望的操作状态
     * @param timeout_ms 超时时间(毫秒)
     * @param operation_name 操作名称(用于日志)
     * @return 返回值含义：
     *         - 0: 操作成功完成
     *         - 1: 操作超时
     *         - 负数: 电机故障码的负数
     */
    int waitForOperationStatus(MotorId id, Operation expected_op, OperationStatus expected_status, int timeout_ms, const char* operation_name);

    // 等待所有电机第一个反馈到达
    bool waitForAllMotorsFeedback(int timeout_ms);

    // 初始化CAN总线并注册电机设备
    bool initializeCanBusAndDevices(const std::vector<RevoMotorConfig_t>& motor_configs);
    
private:
    // 回调上下文
    canbus_sdk::CallbackContext msg_callback_context_;
    canbus_sdk::CallbackContext tef_callback_context_;

    struct MotorCtrlData {
        std::atomic<Operation> operation{Operation::IDLE};              // 当前操作
        std::atomic<OperationStatus> operation_status{OperationStatus::SUCCESS}; // 操作状态
        std::atomic<bool> feedback_received{false};          // 是否收到反馈
        ////////////////////////////////////////////////
        RevoMotor motor;                  // 电机实例
        RevoMotorConfig_t config;         // 电机配置
        RevoMotorCmd_t cmd;               // 电机控制命令
        std::atomic<MotorErrCode> fault_code;  // 故障代码
        mutable std::mutex cmd_mutex;     // 保护cmd字段的互斥锁

        MotorCtrlData(bool fr, RevoMotor&& m, const RevoMotorConfig_t& c)
            : operation(Operation::IDLE), operation_status(OperationStatus::SUCCESS),
              feedback_received(fr), motor(std::move(m)), config(c),
              cmd{0.0, 0.0, 0.0, 0.0, 0.0}, fault_code{MotorErrCode::NO_FAULT}, cmd_mutex() {}

        bool isEnabled() const {
            if(fault_code != MotorErrCode::NO_FAULT) return false;  // 有故障码
            return operation.load() == Operation::ENABLE
                && feedback_received.load()
                && (operation_status.load() == OperationStatus::SUCCESS || operation_status.load() == OperationStatus::PENDING) ;
        }

        // 线程安全地获取电机命令
        RevoMotorCmd_t getCmd() const {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            return cmd;
        }

        // 线程安全地设置电机命令
        void setCmd(const RevoMotorCmd_t& new_cmd) {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            cmd = new_cmd;
        }

        // 开始新的操作 (线程安全)
        void startOperation(Operation op) {
            operation.store(op);
            operation_status.store(OperationStatus::PENDING);
            feedback_received.store(false);
        }

        // 更新操作状态 (线程安全)
        void updateOperationStatus(OperationStatus status) {
            operation_status.store(status);
        }

        MotorErrCode getFaultCode() const {
            return fault_code.load();
        }
    };
    std::atomic<bool> target_updated_{false};
    std::atomic<bool> zero_torque_mode_{false};  // 0-torque模式标志

    std::string canbus_name_;  // CAN总线名称
    std::map<MotorId, MotorCtrlData> motor_ctrl_datas_;
    std::map<MotorId, float> initial_target_positions_;  // 电机回零时的目标初始位置(rad)，如果某个电机ID不在map中，则默认回零到0
    mutable std::mutex initial_target_positions_mutex_;  // 保护initial_target_positions_的互斥锁

};

} // namespace motorevo
#endif