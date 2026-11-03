#ifndef _TOUCH_DEXHAND_H_
#define _TOUCH_DEXHAND_H_
#include <array>
#include <cstdint>
#include <mutex>
#include "dexhand_base.h"

struct DeviceHandler;
namespace dexhand {
std::ostream& operator<<(std::ostream& os, const TouchSensorStatus_t& status);
std::ostream& operator<<(std::ostream& os, const FingerStatusPtr& status);

/// 手部六指的原始反馈（SDK 口径：位置/速度 0~1000，电流与状态位原值）。
///
/// 需要单独定义这个 POD，是因为 AARCH64 与 x86 的 SDK 反馈类型不同
/// （MotorStatusData / CMotorStatusData），用它做架构无关的搬运中介；
/// 再由 feedbackToFingerStatus() 统一换算成 API 口径，避免各调用点各写一份 ÷10/clamp。
struct FingerFeedback {
    std::array<uint16_t, 6> positions{};
    std::array<int16_t, 6> speeds{}, currents{};
    std::array<uint8_t, 6> states{};
};


class ModbusDexhand: public DexHandBase {
protected:
    ModbusDexhand(const ModbusDexhand&) = delete;
    ModbusDexhand& operator=(const ModbusDexhand&) = delete;
    virtual ~ModbusDexhand();
public:


    // See DexHandBase::getDexHandFwType for details    
    DexHandFwType getDexHandFwType() override;

    // See DexHandBase::getDeviceInfo for details
    bool getDeviceInfo(DeviceInfo_t& info) override;

    // See DexHandBase::setFingerPositions for details
    void setFingerPositions(const UnsignedFingerArray &positions) override;
    
    // See DexHandBase::setFingerSpeeds for details
    void setFingerSpeeds(const FingerArray &speeds) override;

    // See DexHandBase::getFingerStatus for details
    bool getFingerStatus(FingerStatus& status) override;

    // See DexHandBase::setGripForce for details
    void setGripForce(GripForce level) override;

    // See DexHandBase::getGripForce for details
    GripForce  getGripForce() override;

    // See DexHandBase::setTurboModeEnabled for details
    void setTurboModeEnabled(bool enabled) override;

    // See DexHandBase::isTurboModeEnabled for details
    bool isTurboModeEnabled() override;

    // See DexHandBase::getTurboConfig for details
    TurboConfig_t getTurboConfig() override;


    // See DexHandBase::runActionSequence for details
    void runActionSequence(ActionSequenceId_t seq_id) override;

    // See DexHandBase::setActionSequence for details
    bool setActionSequence(ActionSequenceId_t seq_id, const ActionSeqDataTypeVec &sequences) override;

    virtual bool usesRevo2MotorApi() const { return false; }

    /// 单手手指数量（SDK 反馈数组固定 6 项）。
    static constexpr size_t kFingerCount = 6;

    /// 反馈侧等待同手锁的预算（ms）。取一个控制周期：既给回读线程一次公平机会，
    /// 又不让命令侧多等。命令侧仍是零等待 try_lock，不受该值影响。
    static constexpr int kFeedbackLockWaitMs = 5;

    /// readFingerFeedback() 的结果。Ok=拿到数据；Failed=SDK 返回 null 或抛异常；
    /// Busy=同手锁被占，本次未发出查询。Busy 是正常的让位结果，不是错误，
    /// 调用方只应计数、不应逐条告警。
    enum class FingerQueryResult : int {
        Ok = 0,
        Failed = 1,
        Busy = 2,
    };

    // 真实回读：向手查询六指当前反馈（SDK 口径），与 getFingerStatus() 并列：
    //   getFingerStatus()     阻塞取锁，485 路径控制线程在用
    //   readFingerFeedback()  抢不到锁就返回 Busy，给独立回读线程用
    virtual FingerQueryResult readFingerFeedback(FingerFeedback& out);

    // SDK 口径 → API 口径（÷10 并 clamp）。485 的 getFingerStatus 与 CAN 的真实回读
    // 共用这一份实现，保证 /dexhand/state 的数值语义与原来完全一致。
    static void feedbackToFingerStatus(const FingerFeedback& feedback, FingerStatus& out);

    // 非阻塞提交：抢不到同手锁就直接返回 false，由调用方下个周期重试。
    // 控制线程与回读线程共用一把锁时使用，避免查询把命令卡住。
    bool trySubmitPositions(const UnsignedFingerArray& positions);
    bool trySubmitSpeeds(const FingerArray& speeds);

protected:
    explicit ModbusDexhand(DeviceHandler* handle, uint8_t slave_id_);

    /// 真正发一次状态查询并把六指原值搬进 out。封装 AARCH64/x86 的 SDK 类型差异
    /// 与堆对象释放（RAII），485 的 getFingerStatus() 与 CAN 的 readFingerFeedback()
    /// 共用同一段 C 互操作代码。返回 false = SDK 未给数据或抛异常。
    /// 调用方必须已持有 sdk_mutex_。
    bool queryMotorStatus(FingerFeedback& out);

// private:
    // 同一只手的全部 SDK 调用（读写）都走这把递归锁：
    // 485 路径原本只有控制线程串行调用，加锁后行为不变；
    // CAN 路径新增回读线程后，靠它保证不会两条线程同时进闭源 SDK。
    // 用 recursive_timed_mutex 是为了反馈侧能 try_lock_for 有界等待，
    // 递归是因为 trySubmit*() 持锁后还会调用同样加锁的 setFinger*()。
    std::recursive_timed_mutex sdk_mutex_;
    // 查询异常日志限流：485 以 200Hz 轮询，异常时不能每拍打一条
    static constexpr int64_t kQueryErrorLogIntervalMs = 5000;
    int64_t last_query_error_log_ms_ = 0;
    DeviceHandler* mb_handle_ = nullptr;
    uint8_t slave_id_;
};
} // namespace dexhand
#endif