#ifndef _TOUCH_DEXHAND_H_
#define _TOUCH_DEXHAND_H_
#include <mutex>
#include "dexhand_base.h"

struct ModbusHandle;
namespace dexhand {
std::ostream& operator<<(std::ostream& os, const TouchSensorStatus_t& status);
std::ostream& operator<<(std::ostream& os, const FingerStatusPtr& status);



class ModbusDexhand: public DexHandBase {
protected:
    ModbusDexhand(const ModbusDexhand&) = delete;
    ModbusDexhand& operator=(const ModbusDexhand&) = delete;
    virtual ~ModbusDexhand();
public:


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
    
protected:
    explicit ModbusDexhand(ModbusHandle* handle, uint8_t slave_id_);

// private:
    ModbusHandle* mb_handle_ = nullptr;
    uint8_t slave_id_;
};
} // namespace dexhand
#endif