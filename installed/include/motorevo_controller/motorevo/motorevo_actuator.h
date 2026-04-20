#ifndef _MOTOREVO_ACTUATOR_H_
#define _MOTOREVO_ACTUATOR_H_

#include <string>
#include <vector>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <pthread.h>
#include <cstring>
#include <iostream>

#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/config_parser.h"
#include "motorevo/motor_def.h"
#include "motorevo/motor_ctrl.h"
#include "ruiwo_actuator_base.h"

namespace motorevo {

class MotorevoActuator : public RuiwoActuatorBase {
public:
    /**
     * @brief 构造函数
     * @param config_file 配置文件路径
     * @param cali 是否需要校准零点
     * @param control_frequency 控制频率(Hz)，默认250
     */
    MotorevoActuator(const std::string& config_file, bool cali, int control_frequency = 250);

    /**
     * @brief 析构函数
     */
    ~MotorevoActuator();

    /******************************************************************************/
    /*                          基类接口兼容 (MotorActuatorBase)                  */
    /******************************************************************************/
    virtual int initialize() override;
    virtual int enable() override;
    virtual int disable() override;
    virtual bool disableMotor(int motorIndex) override;
    virtual void close() override;
    // 把当前位置记作零点
    virtual void saveAsZeroPosition() override;
    // 把当前位置保存到零点文件
    virtual void saveZeroPosition() override;
    virtual void set_teach_pendant_mode(int mode) override;
    virtual void changeEncoderZeroRound(int index, double direction) override;
    virtual void adjustZeroPosition(int index, double offset) override;
    virtual std::vector<double> getMotorZeroPoints() override;
    virtual MotorStateDataVec get_motor_state() override;
    virtual void set_positions(const std::vector<uint8_t> &index,
                             const std::vector<double> &positions,
                             const std::vector<double> &torque,
                             const std::vector<double> &velocity,
                             const std::vector<double> &kp = {},
                             const std::vector<double> &kd = {}) override;
    virtual void set_torque(const std::vector<uint8_t> &index,
                           const std::vector<double> &torque) override;
    virtual void set_velocity(const std::vector<uint8_t> &index,
                             const std::vector<double> &velocity) override;
    virtual std::vector<double> get_positions() override;
    virtual std::vector<double> get_torque() override;
    virtual std::vector<double> get_velocity() override;
    
    /**
     * @brief 设置指定关节的kp_pos和kd_pos参数
     *
     * @param joint_indices 关节索引列表 (0-based)
     * @param kp_pos kp_pos值列表，如果为空则不修改
     * @param kd_pos kd_pos值列表，如果为空则不修改
     */
    virtual void set_joint_gains(const std::vector<int> &joint_indices,
        const std::vector<double> &kp_pos,
        const std::vector<double> &kd_pos) override;

    /**
    * @brief 获取指定关节的kp_pos和kd_pos参数
    *
    * @param joint_indices 关节索引列表，如果为空则返回所有关节
    * @return std::vector<std::vector<double>> 第一个vector是kp_pos，第二个是kd_pos
    */
    virtual std::vector<std::vector<double>> get_joint_gains(const std::vector<int> &joint_indices = {}) override;
    /******************************************************************************/
    /*                          现有接口 (MotorevoActuator特有)                    */
    /******************************************************************************/
    
    
    /**
     * @brief 获取默认零点文件路径
     * @return 默认零点文件路径 $HOME/.config/lejuconfig/arms_zero.yaml
     */
    static std::string getDefaultZeroFilePath();

    /**
     * @brief 初始化执行器
     * @return int 返回0表示成功，否则返回错误码
     *  0: 成功
     *  1: 配置错误，如配置文件不存在、解析错误等
     *  2: CAN总线错误，如CAN总线初始化失败等
     *  3: 电机控制器错误，如电机控制器初始化失败等
     */
    int init();

    /**
     * @brief 重置执行器，停止线程并回收资源
     */
    void reset();

    /**
     * @brief 获取所有电机位置
     * @return 电机位置数据列表
     */
    std::vector<double> getPositions();

    /**
     * @brief 获取所有电机速度
     * @return 电机速度数据列表
     */
    std::vector<double> getVelocities();

    /**
     * @brief 获取所有电机扭矩
     * @return 电机扭矩数据列表
     */
    std::vector<double> getTorques();

    /**
     * @brief 设置目标位置、速度、扭矩和控制参数
     * @param targets 电机控制命令列表（按索引对应电机ID）
     */
    void setTargets(const std::vector<RevoMotorCmd_t>& targets);

    /**
     * @brief 设置控制线程CPU亲和性
     * @param cpu_core CPU核心编号
     * @return 设置是否成功
     */
    bool setControlThreadAffinity(int cpu_core);

    /**
     * @brief 使能所有电机
     * @return 使能是否成功
     */
    bool enableAll();

    /**
     * @brief 失能所有电机
     * @return 失能是否成功
     */
    bool disableAll();

    /**
     * @brief 设置电机回零时的目标初始位置
     * @param target_positions 电机ID到目标位置的映射表(rad)，如果某个电机ID不在映射表中，则默认回零到0
     * @note 此函数用于设置moveToZero()函数中每个电机的目标位置，允许外部根据业务需求设置不同的初始位置
     * @note 可以在init()之前或之后调用。如果在init()之前调用，会在init()内部自动应用；如果在init()之后调用，会立即应用
     */
    void setInitialTargetPositions(const std::map<MotorId, float>& target_positions);

    /**
     * @brief 设置保存零点时的偏移调整参数
     * @param zero_offset_adjustments 电机索引到偏移量的映射（弧度），在保存零点时会应用到对应电机
     */
    virtual void setZeroOffsetAdjustments(const std::map<size_t, double>& zero_offset_adjustments) override;
    
    /**
     * @brief 应用零点偏移调整到运行时电机配置（在按下'o'开始站立时调用）
     * 此方法会将之前设置的调整参数应用到运行时的电机零点值
     */
    void applyZeroOffsetAdjustments();

private:
    struct MotorControlRef;
    /**
     * @brief 保存电机零点位置到文件
     */
    void saveZeroToFile();

    /**
     * @brief 构建索引与电机ID的映射关系
     */
    void buildIndexMappings(const std::vector<MotorControlRef>& motor_refs);

    /**
     * @brief 将目标位置应用到所有已初始化的CAN总线组（私有辅助函数）
     * @param target_positions 目标位置映射表
     */
    void applyInitialTargetPositionsToAllGroups(const std::map<MotorId, float>& target_positions);

    // 控制线程管理
    bool startControlThread();
    // 控制线程函数
    void controlThreadFunc();
    
private:
    // 控制线程相关
    std::thread control_thread_;              // 控制线程
    std::atomic<bool> running_;               // 线程运行标志
    std::atomic<int> control_frequency_;      // 控制频率(Hz)
    
    std::string config_file_;                 // 配置文件路径
    bool cali_;                               // 是否需要校准零点   
    std::map<MotorId, float> pending_initial_target_positions_;  // 待设置的目标初始位置（在init()之前设置）
    mutable std::mutex pending_target_positions_mutex_;  // 保护pending_initial_target_positions_的互斥锁
    std::map<size_t, double> zero_offset_adjustments_;  // 保存零点时的偏移调整参数（电机索引到偏移量的映射，弧度）
    mutable std::mutex zero_offset_adjustments_mutex_;  // 保护zero_offset_adjustments_的互斥锁

    // 多CAN总线管理
    struct CanBusGroup {
        std::string name;
        std::shared_ptr<motorevo::RevoMotorControl> motor_control;
    };

    // 电机控制引用（O(1)访问优化）
    struct MotorControlRef {
        size_t bus_group_index;      // CAN总线组索引
        motorevo::MotorId id;        // 电机ID
        std::string name;            // 电机名称
        bool ignore;                 // 被标记为忽略
        motorevo::MotorParam_t rt_params; // 运行时控制参数
    };

    // 索引与电机ID映射（按电机ID直接索引）
    constexpr static size_t MAX_MOTOR_ID = 0x1F;  // 最大电机ID (1-31, 0x00为非法)
    size_t id2indexs_[MAX_MOTOR_ID + 1];        // 电机ID到索引的映射（原生数组）
    std::vector<MotorControlRef> motor_refs_;   // 电机控制引用缓存（O(1)访问）=> index2ids
    std::vector<CanBusGroup> can_bus_groups_;  // CAN总线组列表
};

} // namespace motorevo

#endif