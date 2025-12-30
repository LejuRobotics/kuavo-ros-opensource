#ifndef _TOUCH_DEXHAND_H_
#define _TOUCH_DEXHAND_H_
#include <mutex>
#include "dexhand_base.h"

struct ModbusHandle;
namespace dexhand {
std::ostream& operator<<(std::ostream& os, const TouchSensorStatus_t& status);
std::ostream& operator<<(std::ostream& os, const FingerStatusPtr& status);

class TouchDexhand: public DexHandBase {
public:
    TouchDexhand(const TouchDexhand&) = delete;
    TouchDexhand& operator=(const TouchDexhand&) = delete;
    virtual ~TouchDexhand();

    /**
     * @brief   Connect to the device
     * 
     * @param port 串口名称，例如："/dev/ttyUSB0"
     * @param slave_id 设备ID，默认为1，范围为1~255, 0 为广播地址
     * @param baudrate  波特率，115200, 57600, 19200, 460800
     * @return std::unique_ptr<TouchDexhand> 失败返回 nullptr
     */
    static std::unique_ptr<TouchDexhand> Connect(
        const std::string& port,
        uint8_t slave_id,
        uint32_t baudrate
    );

    // See DexHandBase::getDexHandFwType for details    
    DexHandFwType getDexHandFwType() override;

    // See DexHandBase::getDeviceInfo for details
    DeviceInfo_t getDeviceInfo() override;

    // See DexHandBase::setFingerPositions for details
    void setFingerPositions(const UnsignedFingerArray &positions) override;
    
    // See DexHandBase::setFingerSpeeds for details
    void setFingerSpeeds(const FingerArray &speeds) override;

    // See DexHandBase::getFingerStatus for details
    FingerStatusPtr getFingerStatus() override;

    // See DexHandBase::setGripForce for details
    void setGripForce(GripForce level) override;

    // See DexHandBase::getGripForce for details
    GripForce  getGripForce() override;

    /**
     * @brief Get the Touch Status object
     * 
     * @return FingerTouchStatusPtr 
     */
    FingerTouchStatusPtr getTouchStatus();

    // See DexHandBase::setTurboModeEnabled for details
    void setTurboModeEnabled(bool enabled) override;

    // See DexHandBase::isTurboModeEnabled for details
    bool isTurboModeEnabled() override;

    // See DexHandBase::getTurboConfig for details
    TurboConfig_t getTurboConfig() override;

    /**
     * @brief 重置触觉传感器采集通道
     * @note 在执行该指令时，手指传感器尽量不要受力, 0b00000001 表示重置第一个传感器
     * 
     * @param bits 
     */
    void resetTouchSensor(uint8_t bits = 0xFF);

    /**
     * @brief 启用触觉传感器
     * @note 0b00000001 表示启用第一个传感器
     * 
     * @param bits 
     */
    void enableTouchSensor(uint8_t bits = 0xFF);

    // See DexHandBase::runActionSequence for details
    void runActionSequence(ActionSequenceId_t seq_id) override;

    // See DexHandBase::setActionSequence for details
    bool setActionSequence(ActionSequenceId_t seq_id, const ActionSeqDataTypeVec &sequences) override;
    
private:
    explicit TouchDexhand(ModbusHandle* handle, uint8_t slave_id_);

private:
    ModbusHandle* mb_handle_ = nullptr;
    uint8_t slave_id_;
};
} // namespace dexhand
#endif