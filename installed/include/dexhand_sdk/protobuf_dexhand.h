#ifndef _PROTOBUF_DEXHAND_H_
#define _PROTOBUF_DEXHAND_H_
#include "dexhand_base.h"

class ProtobufDexhandImpl;
namespace dexhand {
class ProtobufDexhand: public DexHandBase {
public:
    ProtobufDexhand(const ProtobufDexhand&) = delete;
    ProtobufDexhand& operator=(const ProtobufDexhand&) = delete;
    virtual ~ProtobufDexhand();

    /**
     * @brief Connect to the device
     * 
     * @param serial_port 串口名称，例如："/dev/ttyUSB0","/dev/stark_serial_R"
     * @param device_id 设备UUID，用于唯一标识设备
     * @param slave_id 设备ID，默认为10，范围为10~253, 254 为广播地址
     * @param baudrate  波特率，115200, 57600, 19200, 460800
     * @return std::unique_ptr<ProtobufDexhand> 失败返回 nullptr
     */
    static std::unique_ptr<ProtobufDexhand> Connect(
        const std::string& serial_port,
        const std::string& device_id,
        uint8_t slave_id = 10,
        uint32_t baudrate = 115200
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

    // /**
    //  * @brief Get the Touch Status object
    //  * 
    //  * @return FingerTouchStatusPtr 
    //  */
    // FingerTouchStatusPtr getTouchStatus();

    // See DexHandBase::setTurboModeEnabled for details
    void setTurboModeEnabled(bool enabled) override;

    // See DexHandBase::isTurboModeEnabled for details
    bool isTurboModeEnabled() override;

    // See DexHandBase::getTurboConfig for details
    TurboConfig_t getTurboConfig() override;

    // See DexHandBase::runActionSequence for details
    // WARN: libstartk.so say: "this operation is not supported for serial device"
    void runActionSequence(ActionSequenceId_t seq_id) override;

    // See DexHandBase::setActionSequence for details
    // WARN: libstartk.so say: "this operation is not supported for serial device"
    bool setActionSequence(ActionSequenceId_t seq_id, const ActionSeqDataTypeVec &sequences) override;
    
private:
    explicit ProtobufDexhand(ProtobufDexhandImpl* impl);

private:
    ProtobufDexhandImpl* impl_;
};
} // namespace dexhand
#endif // _PROTOBUF_DEXHAND_H_